#!/usr/bin/env python3
"""A1.11 Sensor interruption, characterisation (guidebook 4.3) - W3, gps, gates on /imu and /scan.

    GATE=1 scripts/run_test.sh A1.11 w3_tree_maze gps 1200

The drone flies on GPS throughout, so pausing Cartographer's inputs cannot affect the flight.
Phase 1 hovering, phase 2 flying along the maze_lap: in each phase pause /imu 2 s, /scan 2 s,
/imu 10 s, /scan 10 s (15 s apart). For every pause it records:
  max_pose_age_s     longest wall-clock gap between map->base_link updates (from the pause start to 10 s after)
  stamp_lag_max_s    largest (wall time - pose timestamp) seen: a pose that keeps coming but is OLD
  recovery_s         time after resume until the pose error is back within 3 cm of its pre-pause value
  lasting_error_cm   pose error 10 s after resume minus pose error just before the pause
No pass/fail: the result is a table for the future "SLAM degraded" detector (book: watch pose age).
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'tools'))
import rclpy  # noqa: E402
import yaml  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

from a1_common import append_meta  # noqa: E402
from scripted_flight import Flyer, FlightError, default_pattern_path  # noqa: E402

PAUSES = [('imu', 2), ('scan', 2), ('imu', 10), ('scan', 10)]
GAP_S = 15


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self):
        super().__init__('a1_11_recorder')
        self.lock = threading.Lock()
        self.tf, self.truth, self.events = [], [], []
        self.create_subscription(TFMessage, '/tf', self.cb_tf, QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb_truth, qos_profile_sensor_data)
        self.cli = {k: self.create_client(SetBool, f'/{k}_gate/pause') for k in ('imu', 'scan')}

    def cb_tf(self, m):
        w = time.time()
        for tr in m.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'base_link':
                t = tr.transform.translation
                s = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                with self.lock:
                    self.tf.append((w, s, t.x, t.y, yaw_q(tr.transform.rotation)))

    def cb_truth(self, m):
        p = m.pose.pose.position
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            self.truth.append((time.time(), s, p.x, p.y, p.z, yaw_q(m.pose.pose.orientation)))

    def gate(self, which, pause):
        c = self.cli[which]
        if not c.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'/{which}_gate/pause not available: run with GATE=1')
        fut = c.call_async(SetBool.Request(data=pause))
        t_end = time.time() + 5
        while not fut.done() and time.time() < t_end:
            time.sleep(0.01)
        return time.time()


def run_pauses(rec, phase, keep_flying):
    """keep_flying(seconds) blocks while the drone hovers or flies; returns False when the path is done."""
    for which, dur in PAUSES:
        keep_flying(GAP_S)
        t0 = rec.gate(which, True)
        print(f'  [{phase}] {which} paused {dur} s')
        keep_flying(dur)
        t1 = rec.gate(which, False)
        rec.events.append({'phase': phase, 'sensor': which, 'pause_s': dur, 'wall_start': t0, 'wall_end': t1})
    keep_flying(GAP_S)


def analyse(rec):
    F = np.array(rec.tf)
    T = np.array(rec.truth)
    T = T[np.argsort(T[:, 1])]
    yT = np.unwrap(T[:, 5])
    a = (np.interp(F[0, 1], T[:, 1], T[:, 2]), np.interp(F[0, 1], T[:, 1], T[:, 3]), np.interp(F[0, 1], T[:, 1], yT))
    c, s = math.cos(a[2]), math.sin(a[2])
    ex, ey = a[0] + c * F[:, 2] - s * F[:, 3], a[1] + s * F[:, 2] + c * F[:, 3]
    err = np.hypot(ex - np.interp(F[:, 1], T[:, 1], T[:, 2]), ey - np.interp(F[:, 1], T[:, 1], T[:, 3]))
    # simulated wall time <-> sim time mapping from the truth stream (arrival wall, stamp sim)
    sim_now = lambda w: np.interp(w, T[:, 0], T[:, 1])  # noqa: E731
    rows = []
    for e in rec.events:
        w0, w1 = e['wall_start'], e['wall_end']
        win = (F[:, 0] >= w0 - 0.5) & (F[:, 0] <= w1 + 10)
        arr = np.r_[w0, F[win & (F[:, 0] >= w0), 0], w1 + 10]
        age = float(np.diff(np.sort(arr)).max())
        lag = sim_now(F[win, 0]) - F[win, 1]
        before = err[(F[:, 0] > w0 - 2) & (F[:, 0] < w0)]
        e_before = float(np.median(before)) if len(before) else float('nan')
        after_idx = np.where(F[:, 0] > w1)[0]
        rec_t = None
        for i in after_idx:
            if F[i, 0] > w1 + 30:
                break
            if abs(err[i] - e_before) < 0.03:
                rec_t = float(F[i, 0] - w1)
                break
        e10 = err[(F[:, 0] > w1 + 9.5) & (F[:, 0] < w1 + 10.5)]
        rows.append({**{k: e[k] for k in ('phase', 'sensor', 'pause_s')},
                     'max_pose_age_s': round(age, 2),
                     'stamp_lag_max_s': round(float(lag.max()), 2) if len(lag) else None,
                     'error_before_cm': round(e_before * 100, 1),
                     'max_error_during_cm': round(float(err[win].max()) * 100, 1) if win.any() else None,
                     'recovery_s': None if rec_t is None else round(rec_t, 2),
                     'lasting_error_cm': round((float(np.median(e10)) - e_before) * 100, 1) if len(e10) else None})
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default='w3_tree_maze')
    args, _ = ap.parse_known_args()
    append_meta(args.out, speed=0.5, laps=0, yaw_mode='fixed')
    pattern = yaml.safe_load(open(default_pattern_path(args.world)))
    rclpy.init()
    rec = Recorder()
    ex = SingleThreadedExecutor()
    ex.add_node(rec)
    threading.Thread(target=ex.spin, daemon=True).start()
    f = Flyer()
    ok = True
    try:
        f.connect()
        f.set_param('WP_SPD', 0.5)
        f.wait_position()
        f.pump(1.0)
        _, n0, e0, d0, *_ = f.lpos
        sx, sy, syaw = pattern['start']
        yaw_ned = math.pi / 2 - syaw
        f.set_mode('GUIDED')
        f.arm()
        f.takeoff(1.1, d0)

        def hover(sec):
            t_end = time.time() + sec
            while time.time() < t_end:
                f.goto(n0, e0, d0 - 1.1, yaw_ned, 10.0, 0.3)
                f.pump(0.2)
        print('phase 1: hovering')
        run_pauses(rec, 'hover', hover)

        wps = iter(pattern['waypoints'])
        state = {'target': None}

        def fly(sec):
            t_end = time.time() + sec
            while time.time() < t_end:
                if state['target'] is None:
                    nxt = next(wps, None)
                    if nxt is None:
                        f.goto(*state['last'], yaw_ned, 10.0, 0.3)
                        continue
                    state['target'] = (n0 + (nxt[1] - sy), e0 + (nxt[0] - sx), d0 - 1.1)
                n, e, d = state['target']
                f.m.mav.set_position_target_local_ned_send(0, f.m.target_system, f.m.target_component, 1, 2552,
                                                           n, e, d, 0, 0, 0, 0, 0, 0, yaw_ned, 0)
                f.pump(0.2)
                p = f.lpos
                if p and math.hypot(p[1] - n, p[2] - e) < 0.25:
                    state['last'] = state['target']
                    state['target'] = None
        print('phase 2: flying the maze')
        run_pauses(rec, 'flying', fly)
        f.land()
    except (FlightError, RuntimeError) as e:
        ok = False
        print('ERROR:', e)
        try:
            for k in ('imu', 'scan'):
                rec.gate(k, False)
            if f.m is not None and f.armed:
                f.land()
        except Exception:  # noqa: BLE001
            pass
    time.sleep(2)
    with rec.lock:
        rows = analyse(rec) if rec.events and len(rec.tf) > 100 else []
    ex.shutdown()
    rec.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    json.dump({'flight_ok': ok, 'events': rows}, open(os.path.join(args.out, 'A1_11_report.json'), 'w'), indent=1)
    print(f"\n{'phase':7s} {'sensor':6s} {'pause':>5s} {'pose age':>9s} {'stamp lag':>9s} {'err before':>10s} "
          f"{'max err':>8s} {'recovery':>9s} {'lasting':>8s}")
    for r in rows:
        print(f"{r['phase']:7s} {r['sensor']:6s} {r['pause_s']:5d} {r['max_pose_age_s']:8.2f}s {str(r['stamp_lag_max_s']):>8s}s "
              f"{r['error_before_cm']:8.1f}cm {str(r['max_error_during_cm']):>6s}cm {str(r['recovery_s']):>8s}s "
              f"{str(r['lasting_error_cm']):>6s}cm")
    print('\nA1.11 is a characterisation: no pass/fail. Report: A1_11_report.json')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
