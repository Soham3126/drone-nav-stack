#!/usr/bin/env python3
"""imu_bias_node.py - remove the gyro bias before Cartographer sees the IMU.

A MEMS gyro (the BOM's MPU6500) reads a small non-zero rate when still. Cartographer integrates it,
so its heading creeps (first noisy-IMU flights: +0.01 deg/s, up to 3 deg, 39 cm position error in W4).

/imu_raw -> /imu with angular_velocity minus the estimated bias.
Bias = mean gyro over the most recent `memory_s` seconds of STILL data. "Still" = a window of
`window_s` in which the gyro hardly varies and the accelerometer reads ~1 g; on a hovering drone
the vibration fails that test, so the estimate is taken on the ground (before take-off, after landing)
and held in flight. Until `min_still_s` of still data exists, messages pass through unchanged.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class BiasEstimator:
    """Pure logic (no ROS), so it can be tested on its own."""

    def __init__(self, window_s=1.0, min_still_s=5.0, memory_s=60.0, still_gyro_std=0.004,
                 still_gyro_abs=0.03, still_acc_tol=0.4):
        self.window_s, self.min_still_s, self.memory_s = window_s, min_still_s, memory_s
        self.g_std, self.g_abs, self.a_tol = still_gyro_std, still_gyro_abs, still_acc_tol
        self.win = []                       # (t, gx, gy, gz, |a|)
        self.accepted = deque()             # (t_end, n, sum_gx, sum_gy, sum_gz)
        self.bias = (0.0, 0.0, 0.0)
        self.ready = False

    def add(self, t, g, a_norm):
        self.win.append((t, g[0], g[1], g[2], a_norm))
        if t - self.win[0][0] < self.window_s:
            return
        n = len(self.win)
        sums = [sum(s[k] for s in self.win) for k in (1, 2, 3)]
        means = [v / n for v in sums]
        std = max(math.sqrt(max(0.0, sum((s[k] - means[k - 1]) ** 2 for s in self.win) / n)) for k in (1, 2, 3))
        a_mean = sum(s[4] for s in self.win) / n
        still = std < self.g_std and max(abs(m) for m in means) < self.g_abs and abs(a_mean - 9.81) < self.a_tol
        if still:
            self.accepted.append((t, n, *sums))
            while self.accepted and t - self.accepted[0][0] > self.memory_s:
                self.accepted.popleft()
            total = sum(w[1] for w in self.accepted)
            dur = len(self.accepted) * self.window_s
            if dur >= self.min_still_s:
                self.bias = tuple(sum(w[2 + k] for w in self.accepted) / total for k in range(3))
                self.ready = True
        self.win = []


class ImuBiasNode(Node):
    def __init__(self):
        super().__init__('imu_bias_node')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.est = BiasEstimator(p('window_s', 1.0), p('min_still_s', 5.0), p('memory_s', 60.0))
        self.pub = self.create_publisher(Imu, '/imu', qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu_raw', self.cb, qos_profile_sensor_data)
        self.was_ready = False

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        w, a = m.angular_velocity, m.linear_acceleration
        self.est.add(t, (w.x, w.y, w.z), math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z))
        if self.est.ready and not self.was_ready:
            self.was_ready = True
            self.get_logger().info('gyro bias (deg/s): ' + ', '.join(f'{math.degrees(b):+.4f}' for b in self.est.bias))
        bx, by, bz = self.est.bias
        w.x, w.y, w.z = w.x - bx, w.y - by, w.z - bz
        self.pub.publish(m)


def main():
    rclpy.init()
    n = ImuBiasNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
