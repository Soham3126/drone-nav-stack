#!/usr/bin/env python3
"""scripted_flight.py - fly a waypoint pattern through ArduPilot SITL in GPS mode (guidebook A0.4).

No bridge, no ROS: talks MAVLink straight to SITL on its spare port (tcp:127.0.0.1:5763 =
SERIAL2), so MAVProxy (5760) and the future bridge (5762) are untouched.

  python3 scripted_flight.py --world w0_box_room                      # default pattern of the world
  python3 scripted_flight.py --pattern drone_sim/patterns/w3_tree_maze/maze_lap.yaml --speed 1.0

Sequence: wait for SITL -> set WP_SPD -> GUIDED -> arm (retries until the pre-arm checks pass,
e.g. GPS lock) -> take off to the pattern height -> visit every waypoint -> LAND -> disarm.

Frames. Waypoints are in the Gazebo WORLD frame (x east, y north: ENU), as written by
make_patterns.py. ArduPilot's local frame is NED with its origin where the EKF started. The
ardupilot_gazebo plugin maps Gazebo ENU to NED (north = world +y, east = world +x), so
    north = n0 + (y - y_start),  east = e0 + (x - x_start),  down = d0 - height
where (n0, e0, d0) is the vehicle's own LOCAL_POSITION_NED on the ground before take-off.
Test A0.3 checks this mapping against Gazebo's ground truth.
Heading is held at the start heading for the whole flight: yaw_ned = pi/2 - yaw_world.
"""
import argparse
import json
import math
import os
import sys
import time

import yaml
from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(HERE)
DEFAULT_PATTERN = {'w0': 'square_3m', 'w1': 'corridor_out_back', 'w2': 'turns_tour', 'w5': 'loop_lap'}

MAV_FRAME_LOCAL_NED = 1
TYPE_MASK_POS_YAW = 2552           # use x, y, z and yaw; ignore velocity, acceleration, yaw rate
MSG_INTERVALS = {32: 20, 30: 10, 193: 2, 24: 1, 1: 1}   # message id -> Hz


def default_pattern_path(world):
    name = DEFAULT_PATTERN.get(world[:2], 'maze_lap')
    return os.path.join(PKG_DIR, 'patterns', world, f'{name}.yaml')


class FlightError(RuntimeError):
    pass


class Flyer:
    """Minimal pymavlink pilot. on_sample(t_wall, n, e, d) is called for every LOCAL_POSITION_NED."""

    def __init__(self, url='tcp:127.0.0.1:5763', log=print, on_sample=None):
        self.log = log
        self.on_sample = on_sample
        self.url = url
        self.m = None
        self.lpos = None          # (t_wall, n, e, d, vn, ve, vd)
        self.armed = False
        self.custom_mode = None
        self.statustext = []
        self.ekf_flags = 0

    # ------------------------------------------------------------------ link
    def connect(self, timeout=60):
        t_end = time.time() + timeout
        last = None
        while time.time() < t_end:
            try:
                self.m = mavutil.mavlink_connection(self.url, source_system=250, source_component=191)
                hb = self._autopilot_heartbeat(timeout=10)
                if hb is not None:
                    break
            except OSError as e:          # SITL not listening yet
                last = e
            time.sleep(2)
        else:
            raise FlightError(f'no heartbeat from {self.url} ({last})')
        self.log(f'connected to system {self.m.target_system} on {self.url}')
        for msg_id, hz in MSG_INTERVALS.items():
            self.command(511, msg_id, 1e6 / hz, wait_ack=False)
        self.pump(1.0)

    def _autopilot_heartbeat(self, timeout):
        """First HEARTBEAT from a real autopilot. Ground stations (MAVProxy, other tools) also send
        heartbeats through SITL's ports; locking onto one of them gives target_system 0/255."""
        t_end = time.time() + timeout
        while time.time() < t_end:
            hb = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if hb is None:
                continue
            if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID and hb.type != mavutil.mavlink.MAV_TYPE_GCS \
                    and hb.get_srcSystem() not in (0, 255):
                self.m.target_system = hb.get_srcSystem()
                self.m.target_component = hb.get_srcComponent()
                return hb
        return None

    def pump(self, duration=0.0):
        """Read everything waiting (and keep reading for `duration` seconds)."""
        t_end = time.time() + duration
        while True:
            msg = self.m.recv_match(blocking=True, timeout=0.02)
            if msg is not None:
                self._handle(msg)
            elif time.time() >= t_end:
                return

    def _handle(self, msg):
        t = msg.get_type()
        if t == 'LOCAL_POSITION_NED':
            self.lpos = (time.time(), msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
            if self.on_sample:
                self.on_sample(self.lpos[0], msg.x, msg.y, msg.z)
        elif t == 'HEARTBEAT' and msg.get_srcSystem() == self.m.target_system and msg.get_srcComponent() == 1:
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.custom_mode = msg.custom_mode
        elif t == 'STATUSTEXT':
            text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors='ignore')
            self.statustext.append((time.time(), text))
            self.log(f'  [ap] {text}')
        elif t == 'EKF_STATUS_REPORT':
            self.ekf_flags = msg.flags
        elif t == 'COMMAND_ACK':
            self._last_ack = (msg.command, msg.result, time.time())

    def command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, wait_ack=True, timeout=3.0):
        self._last_ack = None
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0,
                                     p1, p2, p3, p4, p5, p6, p7)
        if not wait_ack:
            return None
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.pump(0.05)
            if self._last_ack and self._last_ack[0] == cmd:
                return self._last_ack[1]
        return None

    def set_param(self, name, value, timeout=5.0):
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(),
                                  float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t_end = time.time() + timeout
        while time.time() < t_end:
            msg = self.m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.2)
            if msg is None:
                continue
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            if pid.rstrip('\x00') == name:
                if abs(msg.param_value - value) > 1e-4 * max(1, abs(value)):
                    raise FlightError(f'{name} stayed {msg.param_value}, wanted {value}')
                return
        raise FlightError(f'no reply setting {name} (unknown parameter?)')

    # ------------------------------------------------------------------ actions
    def set_mode(self, name, timeout=10.0):
        mode_id = self.m.mode_mapping()[name]
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.m.set_mode(mode_id)
            self.pump(0.5)
            if self.custom_mode == mode_id:
                self.log(f'mode {name}')
                return
        raise FlightError(f'could not enter {name}')

    def wait_position(self, timeout=120):
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.pump(0.2)
            if self.lpos is not None:
                return
        raise FlightError('no LOCAL_POSITION_NED (EKF has no position yet)')

    def arm(self, timeout=180):
        """Retry until the pre-arm checks pass (GPS lock and EKF settle take ~30 s after start)."""
        t_end = time.time() + timeout
        n = 0
        while time.time() < t_end:
            n += 1
            self.command(400, 1)
            self.pump(1.0)
            if self.armed:
                self.log(f'armed (attempt {n})')
                return
            self.pump(2.0)
        recent = [t for _, t in self.statustext[-5:]]
        raise FlightError(f'arming refused for {timeout} s; last messages: {recent}')

    def takeoff(self, height, d0, timeout=40):
        res = self.command(22, p7=height)
        if res not in (0, None):
            raise FlightError(f'takeoff refused (result {res})')
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.pump(0.2)
            if self.lpos and (d0 - self.lpos[3]) >= height - 0.1:
                self.log(f'at {d0 - self.lpos[3]:.2f} m')
                return
            if not self.armed:
                raise FlightError('disarmed during takeoff')
        raise FlightError('takeoff height not reached')

    def goto(self, n, e, d, yaw_ned, accept, timeout):
        t_end = time.time() + timeout
        next_send = 0.0
        while time.time() < t_end:
            now = time.time()
            if now >= next_send:
                self.m.mav.set_position_target_local_ned_send(
                    0, self.m.target_system, self.m.target_component, MAV_FRAME_LOCAL_NED,
                    TYPE_MASK_POS_YAW, n, e, d, 0, 0, 0, 0, 0, 0, yaw_ned, 0)
                next_send = now + 0.2
            self.pump(0.05)
            if not self.armed:
                raise FlightError('vehicle disarmed in flight')
            p = self.lpos
            if p and math.hypot(p[1] - n, p[2] - e) < accept and abs(p[3] - d) < 0.2:
                return now
        raise FlightError(f'waypoint ({n:.2f}, {e:.2f}) not reached in {timeout:.0f} s')

    def land(self, timeout=90):
        self.set_mode('LAND')
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.pump(0.5)
            if not self.armed:
                self.log('landed and disarmed')
                return
        raise FlightError('still armed after LAND timeout')


def fly_pattern(pattern, url='tcp:127.0.0.1:5763', speed=None, log=print, on_sample=None, yaw_mode='fixed',
                extra_params=None):
    """Fly one pattern dict (as loaded from YAML). Returns a result dict.
    yaw_mode 'fixed': hold the start heading; 'travel': point the nose along each leg
    (ArduPilot turns while it starts the leg). extra_params: {NAME: value} set before arming."""
    speed = speed or pattern.get('speed_m_s', 0.5)
    h = pattern.get('height_m', 1.1)
    accept = pattern.get('accept_m', 0.25)
    sx, sy, syaw = pattern['start']
    yaw_ned = math.pi / 2 - syaw
    f = Flyer(url, log=log, on_sample=on_sample)
    result = {'pattern': pattern['pattern'], 'world': pattern['world'], 'speed_m_s': speed, 'yaw_mode': yaw_mode,
              'waypoints_total': len(pattern['waypoints']), 'waypoints_reached': 0, 'ok': False}
    t0 = time.time()
    try:
        f.connect()
        f.set_param('WP_SPD', speed)
        for name, val in (extra_params or {}).items():
            f.set_param(name, val)
        f.wait_position()
        f.pump(1.0)
        _, n0, e0, d0, *_ = f.lpos
        result['origin_ned'] = [n0, e0, d0]
        f.set_mode('GUIDED')
        f.arm()
        f.takeoff(h, d0)
        result['t_takeoff_done'] = time.time()
        prev = (sx, sy)
        reached = []
        for k, (wx, wy) in enumerate(pattern['waypoints']):
            n, e, d = n0 + (wy - sy), e0 + (wx - sx), d0 - h
            leg = math.dist(prev, (wx, wy))
            if yaw_mode == 'travel' and leg > 0.05:
                yaw_ned = math.atan2(wx - prev[0], wy - prev[1])     # NED yaw: from north towards east
            t = f.goto(n, e, d, yaw_ned, accept, timeout=leg / speed * 3 + 20)
            reached.append(t)
            result['waypoints_reached'] = k + 1
            prev = (wx, wy)
            log(f'waypoint {k + 1}/{len(pattern["waypoints"])} ({wx:.2f}, {wy:.2f})')
        result['t_last_waypoint'] = time.time()
        f.land()
        result['ok'] = True
    except FlightError as err:
        result['error'] = str(err)
        log(f'FLIGHT ERROR: {err}')
        try:
            if f.m is not None and f.armed:
                f.land()
        except Exception as e2:  # noqa: BLE001
            log(f'landing after error failed: {e2}')
    result['duration_s'] = round(time.time() - t0, 1)
    result['statustext'] = [t for _, t in f.statustext]
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', help='use this world\'s default pattern')
    ap.add_argument('--pattern', help='path to a pattern YAML')
    ap.add_argument('--speed', type=float, help='override speed (m/s), sets WP_SPD')
    ap.add_argument('--url', default='tcp:127.0.0.1:5763')
    ap.add_argument('--out', help='folder for flight_result.json')
    ap.add_argument('--yaw', choices=['fixed', 'travel'], default='fixed', help='heading during the flight')
    args = ap.parse_args()
    path = args.pattern or (default_pattern_path(args.world) if args.world else None)
    if not path:
        ap.error('give --world or --pattern')
    pattern = yaml.safe_load(open(path))
    print(f"pattern {pattern['pattern']} in {pattern['world']}: {len(pattern['waypoints'])} waypoints, "
          f"{pattern['path_length_m']} m")
    res = fly_pattern(pattern, url=args.url, speed=args.speed, yaw_mode=args.yaw)
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        json.dump(res, open(os.path.join(args.out, 'flight_result.json'), 'w'), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != 'statustext'}, indent=1))
    return 0 if res['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
