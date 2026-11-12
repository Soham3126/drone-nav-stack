#!/usr/bin/env python3
"""A1.10 CPU and pose latency (guidebook 4.3) - W4, gps, maze_lap at 0.5 m/s, with carto-v1.

    scripts/run_test.sh A1.10 w4_maze_rooms gps 1800

Measures, while the lap is flown:
  scan_delivery_ms   wall-clock arrival of /scan minus its timestamp (sensor + Gazebo bridge)
  match_latency_ms   wall-clock arrival of /scan_matched_points2 minus the timestamp of the scan it
                     was matched from = Cartographer's processing delay per matched pose (book step 2)
  effective_delay_ms the time shift between the pose stream AS RECEIVED (map->base_link, by wall-clock
                     arrival) and ground truth that minimises the position error (book step 3).
                     This is what ArduPilot would experience -> VISO_DELAY_MS (before the bridge's own delay).
  pose_jumps         sudden pose corrections (loop closure): size and count. The bridge must absorb them.
CPU per process comes from cpu.csv (run_test.sh) via eval_run.py.
Sim time and wall time are related through /clock (sim time -> wall time when it arrived).
Writes A1_10_report.json and prints the VISO_DELAY_MS line to commit.
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
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import LaserScan, PointCloud2  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

from a1_common import append_meta  # noqa: E402
from scripted_flight import default_pattern_path, fly_pattern  # noqa: E402


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Latency(Node):
    def __init__(self):
        super().__init__('a1_10_latency')
        self.lock = threading.Lock()
        self.clock, self.scan, self.match, self.tf, self.truth = [], [], [], [], []
        self.create_subscription(Clock, '/clock', self.cb_clock, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/scan_matched_points2', self.cb_match,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(TFMessage, '/tf', self.cb_tf, QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb_truth, qos_profile_sensor_data)

    def cb_clock(self, m):
        with self.lock:
            self.clock.append((time.time(), m.clock.sec + m.clock.nanosec * 1e-9))

    def cb_scan(self, m):
        with self.lock:
            self.scan.append((time.time(), st(m.header)))

    def cb_match(self, m):
        with self.lock:
            self.match.append((time.time(), st(m.header)))

    def cb_tf(self, m):
        w = time.time()
        for tr in m.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'base_link':
                t = tr.transform.translation
                with self.lock:
                    self.tf.append((w, st(tr.header), t.x, t.y, yaw_q(tr.transform.rotation)))

    def cb_truth(self, m):
        p = m.pose.pose.position
        with self.lock:
            self.truth.append((st(m.header), p.x, p.y, p.z, yaw_q(m.pose.pose.orientation)))


def stats(ms):
    a = np.asarray(ms)
    return {'n': int(len(a)), 'median': round(float(np.median(a)), 1), 'p95': round(float(np.percentile(a, 95)), 1),
            'max': round(float(a.max()), 1)} if len(a) else {'n': 0}


def analyse(c):
    C = np.array(c.clock)
    C = C[np.argsort(C[:, 1])]
    _, u = np.unique(C[:, 1], return_index=True)
    C = C[u]

    def wall_of(ts):                       # sim time -> wall time it was reached
        return np.interp(ts, C[:, 1], C[:, 0])
    rep = {}
    S, M = np.array(c.scan), np.array(c.match)
    rep['scan_delivery_ms'] = stats((S[:, 0] - wall_of(S[:, 1])) * 1000)
    rep['match_latency_ms'] = stats((M[:, 0] - wall_of(M[:, 1])) * 1000)
    rep['rtf'] = round(float((C[-1, 1] - C[0, 1]) / (C[-1, 0] - C[0, 0])), 3)
    # effective delay: pose received at wall w vs truth at wall w - d
    F = np.array(c.tf)
    T = np.array(c.truth)
    T = T[np.argsort(T[:, 0])]
    z0 = np.median(T[:50, 3])
    yT = np.unwrap(T[:, 4])
    t_air = T[T[:, 3] > z0 + 0.3, 0]
    F = F[np.argsort(F[:, 0])]
    a = (np.interp(F[0, 1], T[:, 0], T[:, 1]), np.interp(F[0, 1], T[:, 0], T[:, 2]), np.interp(F[0, 1], T[:, 0], yT))
    cc, ss = math.cos(a[2]), math.sin(a[2])
    ex, ey = a[0] + cc * F[:, 2] - ss * F[:, 3], a[1] + ss * F[:, 2] + cc * F[:, 3]
    Tw = wall_of(T[:, 0])
    # compare VELOCITIES (differences over 0.2 s), so constant offsets such as drift or a loop-closure
    # correction do not bias the delay; positions are compared too, for reference
    g = np.arange(max(F[0, 0], wall_of(t_air[0]) + 5), min(F[-1, 0], wall_of(t_air[-1]) - 5), 0.02)
    gx, gy = np.interp(g, F[:, 0], ex), np.interp(g, F[:, 0], ey)
    h = 10                                                   # 10 x 0.02 s = 0.2 s
    vex, vey = (gx[h:] - gx[:-h]) / 0.2, (gy[h:] - gy[:-h]) / 0.2
    shifts = np.arange(-0.1, 0.4, 0.005)
    errs, perr = [], []
    for d in shifts:
        tx, ty = np.interp(g - d, Tw, T[:, 1]), np.interp(g - d, Tw, T[:, 2])
        vtx, vty = (tx[h:] - tx[:-h]) / 0.2, (ty[h:] - ty[:-h]) / 0.2
        errs.append(float(np.sqrt(np.mean((vex - vtx) ** 2 + (vey - vty) ** 2))))
        perr.append(float(np.sqrt(np.mean((gx - tx) ** 2 + (gy - ty) ** 2))))
    k = int(np.argmin(errs))
    rep['effective_delay_ms'] = round(float(shifts[k] * 1000), 0)
    rep['velocity_rms_at_best_shift_m_s'] = round(errs[k], 4)
    rep['position_rms_at_best_shift_m'] = round(perr[k], 4)
    rep['position_rms_at_zero_shift_m'] = round(perr[int(np.argmin(np.abs(shifts)))], 4)
    # pose jumps: estimate step minus true step between consecutive poses (< 50 ms apart)
    tx, ty = np.interp(F[:, 1], T[:, 0], T[:, 1]), np.interp(F[:, 1], T[:, 0], T[:, 2])
    de = np.hypot(np.diff(ex), np.diff(ey))
    dt_ = np.hypot(np.diff(tx), np.diff(ty))
    gap = np.diff(F[:, 1])
    j = (de - dt_ > 0.02) & (gap < 0.05)
    rep['pose_jumps'] = {'count_over_2cm': int(j.sum()), 'max_cm': round(float((de - dt_)[j].max() * 100), 1) if j.any() else 0.0,
                         'times_s': [round(float(F[i, 1] - F[0, 1]), 1) for i in np.where(j)[0][:20]]}
    rep['pose_rate_hz'] = round(float(len(F) / (F[-1, 0] - F[0, 0])), 1)
    return rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default='w4_maze_rooms')
    args, _ = ap.parse_known_args()
    append_meta(args.out, speed=0.5, laps=1, yaw_mode='fixed')
    pattern = yaml.safe_load(open(default_pattern_path(args.world)))
    rclpy.init()
    node = Latency()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    print(f'A1.10: {pattern["pattern"]} in {args.world}, measuring delays ...')
    res = fly_pattern(pattern, speed=0.5)
    time.sleep(3)
    with node.lock:
        rep = analyse(node)
    ex.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    rep['flight_ok'] = res['ok']
    viso = int(round(max(0.0, rep['effective_delay_ms']) / 10.0) * 10)
    rep['suggested_VISO_DELAY_MS'] = viso
    json.dump(rep, open(os.path.join(args.out, 'A1_10_report.json'), 'w'), indent=1)
    print(json.dumps(rep, indent=1))
    print(f'\nSuggested: VISO_DELAY_MS {viso}   (pose delay only; the bridge adds its own, measured in B9)')
    print('Commit with: sed -i "s/^VISO_DELAY_MS .*/VISO_DELAY_MS   %d        # measured in A1.10/" '
          'drone_bringup/params/ardupilot/sitl_extnav.parm' % viso)
    return 0 if res['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
