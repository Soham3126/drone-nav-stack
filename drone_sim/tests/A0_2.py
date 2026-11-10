#!/usr/bin/env python3
"""Test A0.2 - sensor streams (guidebook 4.3).

Run with the simulation up and the drone on the ground:
    scripts/sim_up.sh w0_box_room gps          (or HEADLESS=0 to watch)
    python3 drone_sim/tests/A0_2.py --world w0_box_room --out ~/drone_results/A0_2

Listens for --duration seconds (default 30) and checks:
  /scan  rate 9-11 Hz, frame 'laser', 500 ranges, range 0.05-12.0 m, increment ~0.72 deg
  /imu   rate 180-220 Hz, frame 'imu_link', |a| = 9.81 +- 0.3 m/s^2 pointing up (+z)
  TF     base_link -> laser and base_link -> imu_link at z = 0.12 m
  sync   at each scan's arrival, newest /imu stamp - scan stamp: median < 50 ms; stamps increasing
  room   the scan matches the ground-truth world (median range error < 5 cm),
         computed by ray-casting meta.json's walls and boxes from the start pose
Rates are measured from message timestamps (sensor rate) and also reported per wall-clock second.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def ray_rect(px, py, dx, dy, r):
    """Distance along ray (px,py)+t(dx,dy) to axis-aligned rectangle r, or inf."""
    tmin, tmax = -math.inf, math.inf
    for p, d, lo, hi in ((px, dx, r[0], r[2]), (py, dy, r[1], r[3])):
        if abs(d) < 1e-12:
            if p < lo or p > hi:
                return math.inf
        else:
            t1, t2 = (lo - p) / d, (hi - p) / d
            tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    if tmax < max(tmin, 0.0):
        return math.inf
    return tmin if tmin > 0 else math.inf


class Collector(Node):
    def __init__(self):
        super().__init__('a0_2_sensor_check')
        self.scans, self.imus = [], []
        self.first_scan = None
        self.newest_imu = None
        self.scan_imu_gaps = []   # at each scan arrival: newest IMU stamp - scan stamp
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)

    def on_scan(self, m):
        self.scans.append((time.monotonic(), stamp(m), m.header.frame_id))
        if self.newest_imu is not None:
            self.scan_imu_gaps.append(self.newest_imu - stamp(m))
        if self.first_scan is None:
            self.first_scan = m
        self.last_scan = m

    def on_imu(self, m):
        a = m.linear_acceleration
        self.newest_imu = stamp(m)
        self.imus.append((time.monotonic(), stamp(m), m.header.frame_id, a.x, a.y, a.z))


def rate(samples, idx):
    if len(samples) < 3:
        return 0.0
    t = [s[idx] for s in samples]
    return (len(t) - 1) / (t[-1] - t[0]) if t[-1] > t[0] else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='w0_box_room')
    ap.add_argument('--out', default=None)
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--laser-z', type=float, default=0.315, help='LiDAR height above the floor on the ground')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Collector()
    print(f'Listening for {args.duration:.0f} s ...')
    t_end = time.monotonic() + args.duration
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    res = {}

    # ---- /scan
    sc = node.scans
    scan_rate = rate(sc, 1)
    m = node.first_scan
    ok = m is not None
    scan = {'messages': len(sc), 'rate_hz_stamp': round(scan_rate, 2), 'rate_hz_wall': round(rate(sc, 0), 2)}
    if ok:
        scan.update(frame_id=m.header.frame_id, ranges=len(m.ranges), range_min=m.range_min,
                    range_max=m.range_max, angle_increment=m.angle_increment)
        ok = (9 <= scan_rate <= 11 and m.header.frame_id == 'laser' and len(m.ranges) == 500
              and abs(m.range_min - 0.05) < 1e-3 and abs(m.range_max - 12.0) < 1e-3
              and abs(m.angle_increment - 2 * math.pi / 500) < 2e-4)
    scan['pass'] = bool(ok)
    res['scan'] = scan

    # ---- /imu
    im = node.imus
    imu = {'messages': len(im), 'rate_hz_stamp': round(rate(im, 1), 2), 'rate_hz_wall': round(rate(im, 0), 2)}
    ok = len(im) > 10
    if ok:
        a = np.array([s[3:6] for s in im[-200:]])
        mean = a.mean(axis=0)
        mag = float(np.linalg.norm(mean))
        imu.update(frame_id=im[-1][2], accel_mean=[round(float(v), 3) for v in mean], accel_norm=round(mag, 3))
        ok = (180 <= imu['rate_hz_stamp'] <= 220 and im[-1][2] == 'imu_link'
              and abs(mag - 9.81) < 0.3 and mean[2] > 9.5)
    imu['pass'] = bool(ok)
    res['imu'] = imu

    # ---- TF
    tfr = {}
    for child in ('laser', 'imu_link'):
        try:
            t = node.tf.lookup_transform('base_link', child, rclpy.time.Time(),
                                         timeout=rclpy.duration.Duration(seconds=2.0))
            tr = t.transform.translation
            tfr[child] = {'xyz': [round(tr.x, 3), round(tr.y, 3), round(tr.z, 3)],
                          'pass': abs(tr.x) < 1e-3 and abs(tr.y) < 1e-3 and abs(tr.z - 0.12) < 1e-3}
        except Exception as e:  # noqa: BLE001
            tfr[child] = {'error': str(e), 'pass': False}
    tfr['pass'] = all(v['pass'] for v in tfr.values())
    res['tf'] = tfr

    # ---- timestamp sync
    # Measured at the moment each scan ARRIVES: how far the newest IMU stamp is ahead of
    # that scan's stamp. Comparing the two newest messages at the end of the run (first
    # version of this test) is wrong: with 10 Hz scans the last scan can be up to 100 ms
    # old purely by chance. A median gap near 0 = same clock, no lag; a steady gap near
    # 100 ms = the scan is delivered one LiDAR period after its timestamp.
    sync = {'pass': False}
    g = np.array(node.scan_imu_gaps)
    if len(g) > 5:
        inc = all(b[1] > a[1] for a, b in zip(sc, sc[1:])) and all(b[1] >= a[1] for a, b in zip(im, im[1:]))
        sync = {'scans_measured': int(len(g)),
                'gap_ms_median': round(float(np.median(g)) * 1000, 1),
                'gap_ms_min': round(float(g.min()) * 1000, 1),
                'gap_ms_max': round(float(g.max()) * 1000, 1),
                'increasing': inc,
                'pass': bool(abs(np.median(g)) < 0.05 and inc)}
    res['sync'] = sync

    # ---- scan vs. ground truth room
    room = {'pass': False}
    meta_p = os.path.join(PKG, 'maps', args.world, 'meta.json')
    if m is not None and os.path.isfile(meta_p):
        meta = json.load(open(meta_p))
        st = meta['start']
        rects = [w for w in meta['walls']]
        rects += [[b['x'] - b['sx'] / 2, b['y'] - b['sy'] / 2, b['x'] + b['sx'] / 2, b['y'] + b['sy'] / 2]
                  for b in meta['boxes'] + meta['low_boxes'] if b['h'] >= args.laser_z]
        s = node.last_scan
        errs = []
        for i, r in enumerate(s.ranges):
            if not (s.range_min < r < s.range_max):
                continue
            ang = s.angle_min + i * s.angle_increment + st['yaw']
            exp = min(ray_rect(st['x'], st['y'], math.cos(ang), math.sin(ang), rc) for rc in rects)
            if math.isfinite(exp):
                errs.append(abs(r - exp))
        if errs:
            e = np.array(errs)
            room = {'beams_compared': len(errs), 'median_error_m': round(float(np.median(e)), 4),
                    'p95_error_m': round(float(np.percentile(e, 95)), 4),
                    'pass': float(np.median(e)) < 0.05}
    res['room_geometry'] = room

    res['pass'] = all(v.get('pass') for v in res.values())
    print(json.dumps(res, indent=1))
    print(f"\nA0.2: {'PASS' if res['pass'] else 'FAIL'}")
    print('Manual step 6: look at /scan in RViz (Fixed Frame: laser) and compare with the room.')
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        json.dump(res, open(os.path.join(args.out, 'A0_2_report.json'), 'w'), indent=1)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if res['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
