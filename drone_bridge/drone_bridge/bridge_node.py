#!/usr/bin/env python3
"""bridge_node.py - the messenger between ROS 2 and ArduPilot (guidebook 2.9).

    ros2 run drone_bridge bridge_node --ros-args -p use_sim_time:=true -p pose_source:=topic \
        -p pose_topic:=/truth/pose

Jobs
  pose -> EKF3      VISION_POSITION_ESTIMATE at 30 Hz (pose from TF map->base_link, or a PoseStamped topic)
  origin and home   SET_GPS_GLOBAL_ORIGIN + MAV_CMD_DO_SET_HOME, repeated until GPS_GLOBAL_ORIGIN comes back
  telemetry         MAV_CMD_SET_MESSAGE_INTERVAL for the messages below, after the first HEARTBEAT
  path following    carrot follower on /bridge/follow_path, SET_POSITION_TARGET_LOCAL_NED at 10 Hz
  commands          /bridge/command (guided | arm | takeoff | land | hold), 3 s ACK wait, 2 retries
  status            /bridge/status at 10 Hz; /bridge/link_stats (JSON text) every 5 s: rates in/out, pose age at send, own CPU (B1, B9)
It identifies itself as system 1, component 191 (onboard computer) and sends HEARTBEAT at 1 Hz.
If the link goes quiet for 3 s it closes and reconnects (test B1 step 4).
"""
import json
import math
import os
import threading
import time

os.environ.setdefault('MAVLINK20', '1')       # VISION_POSITION_ESTIMATE needs MAVLink 2 extension fields

import rclpy  # noqa: E402
from drone_interfaces.msg import BridgeStatus  # noqa: E402
from drone_interfaces.srv import FlightCommand  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from pymavlink import mavutil  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String, UInt8  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

from .carrot import CarrotFollower  # noqa: E402
from .commands import COPTER_GUIDED, MAV_MODE_FLAG_SAFETY_ARMED, CommandRunner  # noqa: E402
from .frames import flu_to_ned_rpy, quat_to_rpy  # noqa: E402
from .mavlink_link import MavLink  # noqa: E402
from .offset import OffsetEstimator, PoseBuffer  # noqa: E402

M = mavutil.mavlink
# Telemetry requested (guidebook 2.9): message id -> rate Hz
STREAMS = {M.MAVLINK_MSG_ID_HEARTBEAT: 1, M.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 30, M.MAVLINK_MSG_ID_ATTITUDE: 30,
           M.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 2, M.MAVLINK_MSG_ID_DISTANCE_SENSOR: 10, M.MAVLINK_MSG_ID_SYS_STATUS: 2}
STREAM_NAMES = {M.MAVLINK_MSG_ID_HEARTBEAT: 'HEARTBEAT', M.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 'LOCAL_POSITION_NED',
                M.MAVLINK_MSG_ID_ATTITUDE: 'ATTITUDE', M.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 'EKF_STATUS_REPORT',
                M.MAVLINK_MSG_ID_DISTANCE_SENSOR: 'DISTANCE_SENSOR', M.MAVLINK_MSG_ID_SYS_STATUS: 'SYS_STATUS'}
COPTER_MODES = {0: 'STABILIZE', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 9: 'LAND', 16: 'POSHOLD'}
# EKF_STATUS_REPORT flags needed for "EKF uses our pose" (guidebook 2.10)
EKF_NEEDED = (M.EKF_ATTITUDE | M.EKF_POS_HORIZ_REL | M.EKF_POS_HORIZ_ABS | M.EKF_POS_VERT_ABS)
TYPE_MASK_POS_YAW = 2552           # position + yaw (guidebook 2.9)


class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge')
        P = self.declare_parameter
        self.url = P('fc_url', 'tcp:127.0.0.1:5762').value
        self.pose_source = P('pose_source', 'tf').value          # tf | topic
        self.pose_topic = P('pose_topic', '/truth/pose').value
        self.reset_topic = P('reset_topic', '').value            # e.g. /pose_mangler/reset_counter
        self.map_frame = P('map_frame', 'map').value
        self.base_frame = P('base_frame', 'base_link').value
        self.pose_rate = P('pose_rate_hz', 30.0).value
        self.flight_height = P('flight_height_m', 1.1).value
        self.lookahead = P('lookahead_m', 0.6).value             # B5: 0.6 m with WP_SPD 0.8
        self.origin_lat = P('origin_lat', -35.363262).value       # any fixed point: indoors it only names the origin
        self.origin_lon = P('origin_lon', 149.165237).value
        self.origin_alt = P('origin_alt_m', 584.0).value
        self.takeoff_tol = P('takeoff_tolerance_m', 0.1).value
        self.pose_timeout = P('pose_timeout_s', 0.3).value          # older poses are never sent (B6)
        self.rf_topic = P('rangefinder_topic', '').value          # unused in simulation now (range_relay.py)
        self.ground_clearance = P('ground_clearance_m', 0.0).value  # rangefinder reading when landed

        self.lock = threading.Lock()
        self.st = dict(mode='', custom_mode=-1, armed=False, ekf_flags=0, origin_set=False, height=float('nan'),
                       rf_raw=None,
                       battery=float('nan'), lpos=None, att=None, lpos_t=0.0, status_texts=[])
        self.rx_counts = {}
        self.rx_window_start = time.monotonic()
        # B9: what we send, how old each pose is when sent, our own CPU use and the sim time per window
        self.tx_counts = {}
        self.tx_age_ms = []
        self.win_sim_start = None
        self.win_cpu_start = time.process_time()
        self.streams_requested_for = 0          # connection number the streams were requested on
        self.pose = None                        # (stamp_s, x, y, z, roll, pitch, yaw) in map FLU
        self.reset_counter = 0
        self.pose_buf = PoseBuffer(1.0)
        self.pose_stale = True
        self.height_bias = 0.0                  # added to the height target; measured at take-off (B8)
        self.last_sent_stamp = 0.0
        self.last_boot_ms = None                # autopilot uptime, to tell a reboot from a link drop
        self.offset = OffsetEstimator(0.1)
        # C4: shorten the lookahead on bends so the drone cuts at most this far inside the path (0 = fixed
        # lookahead, as tested in B5)
        self.max_cut = float(P('follow_max_cut_m', 0.05).value)
        self.carrot = CarrotFollower(self.lookahead, max_cut=self.max_cut if self.max_cut > 0 else None)
        self.following = False
        self.takeoff_yaw_ned = None
        self.hold_target = None
        self.cmd_busy = threading.Lock()

        self.link = MavLink(self.url, self.on_mav, lambda s: self.get_logger().info(s), on_connect=self.on_connect)
        self.runner = CommandRunner(self.link)

        self.add_on_set_parameters_callback(self.on_params)
        # Two single-threaded executors instead of one multi-threaded one (B9: rclpy's MultiThreadedExecutor
        # spent ~90% of the bridge's 45% of a core just scheduling). Timers, subscriptions and TF run on this
        # node; the command service, whose calls block for seconds (takeoff, land), runs on its own small node
        # and thread, so it never holds up the pose stream.
        self.cmd_node = rclpy.create_node('bridge_command')
        if self.pose_source == 'tf':
            # map -> base_link straight from /tf (Cartographer: provide_odom_frame = false publishes it directly).
            # A full tf2 listener costs about twice as much CPU at Cartographer's 200 Hz (B9).
            self.tf_latest = None
            self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        else:
            self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, qos_profile_sensor_data)
        if self.reset_topic:
            self.create_subscription(UInt8, self.reset_topic, self.on_reset, 10)
        self.create_subscription(Path, '/bridge/follow_path', self.on_path, 10)
        if self.rf_topic:
            self.create_subscription(LaserScan, self.rf_topic, self.on_range, qos_profile_sensor_data)
        self.pub_status = self.create_publisher(BridgeStatus, '/bridge/status', 10)
        self.pub_stats = self.create_publisher(String, '/bridge/link_stats', 10)
        self.cmd_node.create_service(FlightCommand, '/bridge/command', self.on_command)
        self.create_timer(1.0 / self.pose_rate, self.tick_pose)
        self.create_timer(0.1, self.tick_follow)
        self.create_timer(0.1, self.tick_status)
        self.create_timer(1.0, self.tick_1hz)
        self.create_timer(5.0, self.tick_stats)
        self.get_logger().info(f'bridge: fc {self.url}, pose from {self.pose_source}'
                               f'{" " + self.pose_topic if self.pose_source == "topic" else ""}, L={self.lookahead} m')

    def on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'lookahead_m':
                self.lookahead = float(p.value)
                self.carrot.L = self.lookahead
                self.carrot.window = 2.0 * self.lookahead + 0.5
                self.get_logger().info(f'lookahead now {self.lookahead} m')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ helpers
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ MAVLink input
    def on_connect(self, n):
        """(Re)connected: forget the autopilot's state until it reports it again. Path following and the frame
        offset are kept - only a reboot of the autopilot (see _check_reboot) clears them. A companion freeze also
        ends in a "reconnect" (no heartbeat seen for 5 s), and the path must then simply continue (B6)."""
        with self.lock:
            self.st.update(mode='', custom_mode=-1, armed=False, ekf_flags=0, origin_set=False,
                           height=float('nan'), rf_raw=None, lpos=None, att=None)

    def _check_reboot(self, time_boot_ms):
        """Autopilot uptime went backwards: it rebooted. Clear everything that belonged to the old session."""
        if self.last_boot_ms is not None and time_boot_ms + 2000 < self.last_boot_ms:
            self.get_logger().info('autopilot rebooted: clearing path, hold target and frame offset')
            self.offset = OffsetEstimator(0.1)
            self.following = False
            self.hold_target = None
            self.carrot.clear()
            self.takeoff_yaw_ned = None
        self.last_boot_ms = time_boot_ms

    def on_mav(self, m, t_mono):
        t = m.get_type()
        self.rx_counts[t] = self.rx_counts.get(t, 0) + 1
        with self.lock:
            st = self.st
            if t == 'HEARTBEAT':
                st['custom_mode'] = m.custom_mode
                st['mode'] = COPTER_MODES.get(m.custom_mode, str(m.custom_mode))
                st['armed'] = bool(m.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == 'LOCAL_POSITION_NED':
                st['lpos'] = (m.x, m.y, m.z, m.vx, m.vy, m.vz)
                st['lpos_t'] = self.now_s()
                self._offset_sample()
            elif t == 'ATTITUDE':
                st['att'] = (m.roll, m.pitch, m.yaw)
                self._check_reboot(m.time_boot_ms)
            elif t == 'EKF_STATUS_REPORT':
                st['ekf_flags'] = m.flags
            elif t == 'DISTANCE_SENSOR':
                # a reading at or outside the sensor's limits (e.g. 0 on the ground) is not a height
                st['rf_raw'] = (m.current_distance, m.min_distance, m.max_distance, m.orientation, m.id)
                valid = m.min_distance < m.current_distance < m.max_distance
                st['height'] = m.current_distance / 100.0 - self.ground_clearance if valid else float('nan')
            elif t == 'SYS_STATUS':
                st['battery'] = float(m.battery_remaining)
            elif t == 'GPS_GLOBAL_ORIGIN':
                if not st['origin_set']:
                    self.get_logger().info('EKF origin confirmed (GPS_GLOBAL_ORIGIN)')
                st['origin_set'] = True
            elif t == 'STATUSTEXT':
                txt = str(m.text).strip()
                st['status_texts'] = (st['status_texts'] + [(time.monotonic(), txt)])[-20:]
                self.get_logger().info(f'[ap] {txt}')

    def _offset_sample(self):
        """Frame-offset tracking: FC position + attitude vs the SLAM pose closest in time (lock held)."""
        lp, att = self.st['lpos'], self.st['att']
        if lp is None or att is None:
            return
        hit = self.pose_buf.closest(self.st['lpos_t'], max_gap=0.2)
        if hit is None:
            return
        _, (n, e, yaw) = hit
        self.offset.update(lp[0], lp[1], att[2], n, e, yaw)

    # ------------------------------------------------------------------ pose input
    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        r, pi, y = quat_to_rpy(q.x, q.y, q.z, q.w)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.pose = (stamp, p.x, p.y, p.z, r, pi, y)

    def on_range(self, msg):
        """Simulation stand-in for a rangefinder wired to the flight controller: forward it as DISTANCE_SENSOR."""
        if not msg.ranges or not self.link.connected:
            return
        r = msg.ranges[0]
        lo, hi = int(msg.range_min * 100), int(msg.range_max * 100)
        if math.isnan(r):
            return
        cm = hi + 1 if math.isinf(r) or r > msg.range_max else max(0, int(round(r * 100)))
        self.link.send('distance_sensor_send', int(time.monotonic() * 1000) & 0xFFFFFFFF, lo, hi, cm,
                       M.MAV_DISTANCE_SENSOR_LASER, 0, M.MAV_SENSOR_ROTATION_PITCH_270, 0)

    def on_reset(self, msg):
        self.reset_counter = int(msg.data) % 256

    def on_tf(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id == self.map_frame and tr.child_frame_id == self.base_frame:
                self.tf_latest = tr

    def _pose_from_tf(self):
        tr = self.tf_latest
        if tr is None:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
        stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        return (stamp, t.x, t.y, t.z, r, p, y)

    # ------------------------------------------------------------------ timers
    def tick_pose(self):
        if self.pose_source == 'tf':
            pz = self._pose_from_tf()
            if pz is not None:
                with self.lock:
                    self.pose = pz
        with self.lock:
            pz = self.pose
        if pz is None or not self.link.connected:
            return
        stamp, x, y, z, r, p, yw = pz
        # Never repeat an old pose: if the pose source stops (SLAM stalls, topic paused), ArduPilot must see
        # the ExternalNav stream stop and time out. Repeating the last pose pins the EKF to one place while the
        # drone moves (B6, first run: the drone flew 6.4 m down the corridor during a 10 s pose pause).
        age = self.now_s() - stamp
        if age > self.pose_timeout:
            self.pose_stale = True
            return
        self.pose_stale = False
        if stamp <= self.last_sent_stamp:
            return                               # this pose was already sent
        self.last_sent_stamp = stamp
        with self.lock:
            self.tx_counts['VISION_POSITION_ESTIMATE'] = self.tx_counts.get('VISION_POSITION_ESTIMATE', 0) + 1
            self.tx_age_ms.append(age * 1000.0)
        roll, pitch, yaw = flu_to_ned_rpy(r, p, yw)
        cov = [float('nan')] + [0.0] * 20       # NaN first element: ArduPilot uses VISO_POS_M_NSE / VISO_YAW_M_NSE
        self.link.send('vision_position_estimate_send', int(stamp * 1e6), x, -y, -z, roll, pitch, yaw,
                       cov, self.reset_counter)
        with self.lock:
            self.pose_buf.add(stamp, x, -y, yaw)

    def tick_1hz(self):
        # our own HEARTBEAT (onboard controller)
        self.link.send('heartbeat_send', M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID, 0, 0,
                       M.MAV_STATE_ACTIVE)
        if not self.link.connected:
            return
        # telemetry requests, once per (re)connection, after the first HEARTBEAT
        if self.streams_requested_for != self.link.connect_count:
            for mid, hz in STREAMS.items():
                self.link.send('command_long_send', 1, 1, M.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                               mid, 1e6 / hz, 0, 0, 0, 0, 0)
            self.streams_requested_for = self.link.connect_count
            with self.lock:
                self.st['origin_set'] = False
            self.get_logger().info('telemetry rates requested')
        # origin and home, repeated until GPS_GLOBAL_ORIGIN arrives
        with self.lock:
            need_origin = not self.st['origin_set']
        if need_origin:
            lat, lon, alt = int(self.origin_lat * 1e7), int(self.origin_lon * 1e7), int(self.origin_alt * 1000)
            self.link.send('set_gps_global_origin_send', 1, lat, lon, alt, int(time.time() * 1e6))
            self.link.send('command_long_send', 1, 1, M.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0,
                           self.origin_lat, self.origin_lon, self.origin_alt)
            self.link.send('command_long_send', 1, 1, M.MAV_CMD_REQUEST_MESSAGE, 0,
                           M.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 0, 0, 0, 0, 0, 0)

    def tick_follow(self):
        """Carrot follower at 10 Hz (and 'hold' targets)."""
        with self.lock:
            pz, st = self.pose, dict(self.st)
        if not self.link.connected or not st['armed'] or st['custom_mode'] != COPTER_GUIDED:
            return
        if self.hold_target is not None and not self.following:
            n, e, yaw = self.hold_target
            self._send_target(n, e, yaw)
            return
        if not self.following or pz is None or not self.carrot.active:
            return
        _, x, y, _, _, _, _ = pz
        cx, cy = self.carrot.update(x, y)
        with self.lock:
            n, e = self.offset.slam_to_fc(cx, -cy)
        yaw = self.takeoff_yaw_ned if self.takeoff_yaw_ned is not None else (st['att'][2] if st['att'] else 0.0)
        self._send_target(n, e, yaw)
        lp = st['lpos']
        speed = math.hypot(lp[3], lp[4]) if lp else 0.0
        self.carrot.check_arrived(x, y, speed, self.now_s())

    def _send_target(self, n, e, yaw):
        with self.lock:
            self.tx_counts['SET_POSITION_TARGET_LOCAL_NED'] = self.tx_counts.get('SET_POSITION_TARGET_LOCAL_NED', 0) + 1
        self.link.send('set_position_target_local_ned_send', 0, 1, 1, M.MAV_FRAME_LOCAL_NED, TYPE_MASK_POS_YAW,
                       n, e, -(self.flight_height + self.height_bias), 0, 0, 0, 0, 0, 0, yaw, 0)

    def tick_status(self):
        with self.lock:
            st = dict(self.st)
            pz = self.pose
            off_m, off_deg = self.offset.magnitude()
        s = BridgeStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = self.map_frame
        s.fc_connected = self.link.connected
        if not s.fc_connected:          # nothing below is current without a link
            st.update(armed=False, ekf_flags=0, mode='', height=float('nan'), lpos=None)
        s.mode = st['mode']
        s.armed = st['armed']
        s.ekf_position_ok = (st['ekf_flags'] & EKF_NEEDED) == EKF_NEEDED and not (st['ekf_flags'] & M.EKF_CONST_POS_MODE)
        s.origin_set = st['origin_set']
        s.height_m = float(self._height(st))
        s.battery_percent = float(st['battery'])
        s.frame_offset_m = float(off_m)
        s.frame_offset_deg = float(off_deg)
        s.following_path = self.following and not self.carrot.arrived
        s.path_arrived = self.carrot.arrived
        s.distance_to_path_end_m = float(self.carrot.distance_to_end(pz[1], pz[2])) if (pz and self.carrot.active) else 0.0
        self.pub_status.publish(s)

    def tick_stats(self):
        now = time.monotonic()
        dt = now - self.rx_window_start
        counts, self.rx_counts, self.rx_window_start = self.rx_counts, {}, now
        rates = {k: round(v / dt, 2) for k, v in sorted(counts.items())}
        sim_now, cpu_now = self.now_s(), time.process_time()
        sim_dt = (sim_now - self.win_sim_start) if self.win_sim_start is not None else None
        cpu_pct = 100.0 * (cpu_now - self.win_cpu_start) / dt if dt > 0 else None
        self.win_sim_start, self.win_cpu_start = sim_now, cpu_now
        with self.lock:
            rf, lp = self.st['rf_raw'], self.st['lpos']
            tx, self.tx_counts = self.tx_counts, {}
            ages, self.tx_age_ms = sorted(self.tx_age_ms), []
        age = ({'n': len(ages), 'median': round(ages[len(ages) // 2], 1),
                'p95': round(ages[min(len(ages) - 1, int(0.95 * len(ages)))], 1), 'max': round(ages[-1], 1)}
               if ages else {'n': 0})
        self.pub_stats.publish(String(data=json.dumps({'window_s': round(dt, 2), 'rx_hz': rates,
                                                       'rangefinder_cm_min_max_orient_id': rf,
                                                       'ekf_height_m': round(-lp[2], 3) if lp else None,
                                                       'tx_hz': {k: round(v / dt, 2) for k, v in sorted(tx.items())},
                                                       'window_sim_s': round(sim_dt, 3) if sim_dt is not None else None,
                                                       'pose_age_at_send_ms': age,
                                                       'cpu_pct_one_core': round(cpu_pct, 2) if cpu_pct is not None else None,
                                                       'connected': self.link.connected,
                                                       'connections': self.link.connect_count})))

    # ------------------------------------------------------------------ ROS inputs
    def on_path(self, msg):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if not pts:
            self.following = False
            self.carrot.clear()
            self.get_logger().info('empty path: following stopped')
            return
        self.carrot.set_path(pts)
        self.following = True
        self.hold_target = None
        self.get_logger().info(f'following a path of {len(self.carrot.pts)} points, {self.carrot.length:.2f} m')

    def on_command(self, req, res):
        name, value = req.command.strip().lower(), float(req.value)
        if not self.cmd_busy.acquire(blocking=False):
            res.success, res.message = False, 'another command is running'
            return res
        try:
            if not self.link.connected:
                res.success, res.message = False, 'flight controller not connected'
                return res
            if name == 'hold':
                res.success, res.message = self._hold()
                return res
            self.link.capture = True
            while not self.link.cmd_queue.empty():
                self.link.cmd_queue.get_nowait()
            try:
                ok, msg = self.runner.run(name, value)
                if ok and name == 'takeoff':
                    with self.lock:
                        att, lp = self.st['att'], self.st['lpos']
                    self.takeoff_yaw_ned = att[2] if att else 0.0
                    # ArduPilot's own take-off ends when its *raw* rangefinder reads the height: 14.5 cm low with
                    # our sensor (B8). Taking over while still on the ground cancels the take-off (B8, 4th run), so:
                    # 1. let ArduPilot's take-off finish (airborne and no longer climbing),
                    # 2. hold there with a position target at the requested height,
                    # 3. measure what is left and add it to the target once or twice (height_bias).
                    self.flight_height = float(value)
                    self.height_bias = 0.0
                    reached = self._settled(min_height=0.15, timeout=15.0)
                    if lp is not None and reached is not None:
                        self.following = False
                        self.hold_target = (lp[0], lp[1], self.takeoff_yaw_ned)
                        for _ in range(3):
                            time.sleep(2.0)             # let the drone respond to the new target first
                            cur = self._settled(min_height=0.15, timeout=8.0)
                            if cur is None or abs(value - cur) <= 0.02:
                                break
                            self.height_bias = max(-0.3, min(0.3, self.height_bias + (value - cur)))
                        msg += f' | bridge height trim {self.height_bias * 100:+.1f} cm'
                    ok, extra = self._wait_height(value, timeout=20.0)
                    msg += ' | ' + extra
                if ok and name == 'land':
                    ok2, extra = self._wait_disarm(timeout=60.0)
                    msg += ' | ' + extra
                    ok = ok and ok2
                    self.following = False
                    self.hold_target = None
            except ValueError as e:
                ok, msg = False, str(e)
            finally:
                self.link.capture = False
            res.success, res.message = ok, msg
            self.get_logger().info(f'command {name} {value:g}: {"OK" if ok else "FAILED"} - {msg}')
            return res
        finally:
            self.cmd_busy.release()

    def _hold(self):
        with self.lock:
            lp, att = self.st['lpos'], self.st['att']
        if lp is None:
            return False, 'no LOCAL_POSITION_NED yet'
        self.following = False
        yaw = self.takeoff_yaw_ned if self.takeoff_yaw_ned is not None else (att[2] if att else 0.0)
        self.hold_target = (lp[0], lp[1], yaw)
        self._send_target(*self.hold_target)
        return True, f'holding at n={lp[0]:.2f} e={lp[1]:.2f}'

    @staticmethod
    def _height(st):
        """Rangefinder height when it is in range, otherwise the EKF's height (-z of LOCAL_POSITION_NED)."""
        if not math.isnan(st['height']):
            return st['height']
        return -st['lpos'][2] if st['lpos'] else float('nan')

    def _settled(self, min_height, timeout):
        """Wait until the drone is above min_height and its vertical speed has stayed below 5 cm/s for 1 s.
        Returns the mean height over that second, or None if that never happened."""
        t0 = time.monotonic()
        calm, hs = None, []
        while time.monotonic() - t0 < timeout:
            with self.lock:
                cur = self._height(self.st)
                lp = self.st['lpos']
            vz = abs(lp[5]) if lp else 1.0
            if not math.isnan(cur) and cur > min_height and vz < 0.05:
                calm = calm or time.monotonic()
                hs.append(cur)
                if time.monotonic() - calm >= 1.0:
                    return sum(hs) / len(hs)
            else:
                calm, hs = None, []
            time.sleep(0.05)
        return None

    def _wait_height(self, h, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self.lock:
                cur = self._height(self.st)
            if abs(cur - h) <= self.takeoff_tol:
                return True, f'height {cur:.2f} m reached in {time.monotonic() - t0:.1f} s'
            time.sleep(0.1)
        return False, f'height {h} m not reached within {timeout:.0f} s'

    def _wait_disarm(self, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self.lock:
                armed = self.st['armed']
            if not armed:
                return True, f'disarmed after {time.monotonic() - t0:.1f} s'
            time.sleep(0.2)
        return False, f'still armed after {timeout:.0f} s'


def main():
    rclpy.init()
    node = BridgeNode()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex_cmd = SingleThreadedExecutor()
    ex_cmd.add_node(node.cmd_node)
    threading.Thread(target=ex_cmd.spin, daemon=True, name='commands').start()
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.link.stop = True
        ex_cmd.shutdown(timeout_sec=1.0)
        node.cmd_node.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
