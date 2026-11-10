#!/usr/bin/env python3
"""truth_relay.py - /ground_truth/odom -> /truth/pose (guidebook 4.2, test A0.3).

Re-expresses Gazebo's ground-truth pose in the stack's "map" convention: zeroed at the
drone's pose once it has been still for `settle_s` seconds after start-up, so (0, 0, yaw 0)
is where the drone stood, facing the way it faced (principle 7).

Ground truth is for test tools only. It must never reach Cartographer, the planner, the
frontier server or the mission brain (the bridge reads it only in profile 'truth').
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_rotate(q, v):
    x, y, z, _ = q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), q_conj(q))
    return (x, y, z)


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class TruthRelay(Node):
    def __init__(self):
        super().__init__('truth_relay')
        self.settle_s = self.declare_parameter('settle_s', 2.0).value
        self.still_m = self.declare_parameter('still_tolerance_m', 0.005).value
        self.still_deg = self.declare_parameter('still_tolerance_deg', 0.2).value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PoseStamped, '/truth/pose', qos)
        self.sub = self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, qos)
        self.ref = None          # (t, p, q) start of the current still window
        self.zero = None         # (p0, q_yaw0)
        self.get_logger().info('waiting for the drone to be still to set the truth zero')

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        p = (p.x, p.y, p.z)
        q = (o.x, o.y, o.z, o.w)

        if self.zero is None:
            if self.ref is None:
                self.ref = (t, p, q)
                return
            t0, p0, q0 = self.ref
            moved = math.dist(p, p0) > self.still_m or \
                abs(math.degrees(math.remainder(yaw_of(q) - yaw_of(q0), 2 * math.pi))) > self.still_deg
            if moved:
                self.ref = (t, p, q)
                return
            if t - t0 < self.settle_s:
                return
            # zero = position and HEADING only; roll/pitch stay physical
            self.zero = (p, yaw_quat(yaw_of(q)))
            self.get_logger().info(
                f'truth zero set at world x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} '
                f'yaw={math.degrees(yaw_of(q)):.2f} deg')

        p0, qy0 = self.zero
        inv = q_conj(qy0)
        rel_p = q_rotate(inv, (p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]))
        rel_q = q_mul(inv, q)
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.pose.position.x, out.pose.position.y, out.pose.position.z = rel_p
        (out.pose.orientation.x, out.pose.orientation.y,
         out.pose.orientation.z, out.pose.orientation.w) = rel_q
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TruthRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
