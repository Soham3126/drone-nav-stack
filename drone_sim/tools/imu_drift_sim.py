#!/usr/bin/env python3
"""imu_drift_sim.py - SIMULATION ONLY: add a slowly wandering gyro bias to the Gazebo IMU.

Gazebo's IMU noise gives white noise and a bias that is constant for the whole run. A constant
bias is removed completely by drone_slam's ground calibration, which would make the simulation
too kind. A real MEMS gyro's bias also wanders after calibration (bias instability). This node
adds that part: a first-order Gauss-Markov process per axis.
  /imu_sim (Gazebo) -> /imu_raw (as the real IMU driver would publish it)
Defaults: stddev 1e-4 rad/s (0.0057 deg/s, typical consumer MEMS), correlation time 300 s.
Replace with values measured on the real MPU6500 (5+ minutes at rest) when available.
"""
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class DriftSim(Node):
    def __init__(self):
        super().__init__('imu_drift_sim')
        self.sigma = self.declare_parameter('bias_stddev', 1e-4).value
        self.tau = self.declare_parameter('correlation_time_s', 300.0).value
        seed = self.declare_parameter('seed', -1).value
        self.rng = random.Random(None if seed < 0 else seed)
        self.b = [self.rng.gauss(0, self.sigma) for _ in range(3)]
        self.t_last = None
        self.pub = self.create_publisher(Imu, '/imu_raw', qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu_sim', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t_last is not None and t > self.t_last:
            a = math.exp(-(t - self.t_last) / self.tau)
            s = self.sigma * math.sqrt(1 - a * a)
            self.b = [b * a + self.rng.gauss(0, s) for b in self.b]
        self.t_last = t
        w = m.angular_velocity
        w.x, w.y, w.z = w.x + self.b[0], w.y + self.b[1], w.z + self.b[2]
        self.pub.publish(m)


def main():
    rclpy.init()
    n = DriftSim()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
