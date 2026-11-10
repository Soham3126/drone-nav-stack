#!/usr/bin/env python3
"""replay_collect.py - record what eval_run needs during a Cartographer replay, straight into
RUN/eval_data.npz (no rosbag: stopping `ros2 bag record` cleanly from a script proved unreliable).

Listens to /ground_truth/odom, /tf (map -> base_link) and /map. Exits on its own once data has
arrived and nothing new has come for --idle seconds (the bag has finished playing).
    python3 replay_collect.py --out RUN_DIR [--idle 6] [--max 3600]"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tilt_q(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return math.degrees(math.hypot(roll, pitch))


class Collect(Node):
    def __init__(self):
        super().__init__('replay_collect')
        self.truth, self.tilt, self.est = [], [], []
        self.map, self.maps_seen = None, 0
        self.t_last = None
        rel = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb_odom, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.cb_tf, rel)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map,
                                 QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))

    def cb_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.truth.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, yaw_q(q)))
        self.tilt.append(tilt_q(q))
        self.t_last = time.time()

    def cb_tf(self, m):
        for tr in m.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'base_link':
                t = tr.transform.translation
                self.est.append((tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9, t.x, t.y,
                                 yaw_q(tr.transform.rotation)))

    def cb_map(self, m):
        self.map, self.maps_seen = m, self.maps_seen + 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--idle', type=float, default=6.0)
    ap.add_argument('--max', type=float, default=3600.0)
    args = ap.parse_args()
    rclpy.init()
    n = Collect()
    t0 = time.time()
    try:
        while time.time() - t0 < args.max:
            rclpy.spin_once(n, timeout_sec=0.2)
            if n.t_last and n.est and time.time() - n.t_last > args.idle:
                break
    except KeyboardInterrupt:
        pass
    out = {'truth': np.array(n.truth), 'est': np.array(n.est), 'clock': np.zeros((0, 2)),
           'maps_seen': n.maps_seen, 'tilt_deg': np.array(n.tilt)}
    if n.map is not None:
        info = n.map.info
        out['map_data'] = np.array(n.map.data, dtype=np.int8).reshape(info.height, info.width)
        out['map_info'] = np.array([info.resolution, info.origin.position.x, info.origin.position.y,
                                    yaw_q(info.origin.orientation)])
    np.savez_compressed(os.path.join(args.out, 'eval_data.npz'), **out)
    print(f'collected: truth {len(n.truth)}, estimate {len(n.est)}, maps {n.maps_seen}')
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if n.est and n.truth else 1


if __name__ == '__main__':
    sys.exit(main())
