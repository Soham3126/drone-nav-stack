"""sim_common.py - shared pieces for flight tests: a background ROS collector for ground truth,
the world-frame check, and helpers to load tools from drone_sim/tools."""
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'tools'))

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TruthCollector(Node):
    """Records, with the wall-clock time of arrival.
    (Callback names are deliberately unusual: rclpy's Node already owns attributes such as
    _clock, and a method with that name gets overwritten -> 'ROSClock object is not callable'.)
       odom  : /ground_truth/odom  (t_wall, t_sim, x, y, z, yaw)
       truth : /truth/pose         (t_wall, x, y, z, yaw)
       clock : /clock              (t_wall, t_sim), at most 10 per second"""

    def __init__(self):
        super().__init__('test_truth_collector')
        self.lock = threading.Lock()
        self.odom, self.truth, self.clock = [], [], []
        self.t_last_clock_rx = 0.0
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb_ground_truth, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/truth/pose', self.cb_truth_pose, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.cb_sim_clock, qos_profile_sensor_data)

    def cb_ground_truth(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            self.odom.append((time.time(), ts, p.x, p.y, p.z, yaw_of(q)))

    def cb_truth_pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        with self.lock:
            self.truth.append((time.time(), p.x, p.y, p.z, yaw_of(q)))

    def cb_sim_clock(self, m):
        now = time.time()
        if now - self.t_last_clock_rx >= 0.1:
            self.t_last_clock_rx = now
            with self.lock:
                self.clock.append((now, m.clock.sec + m.clock.nanosec * 1e-9))


class Collector:
    def __enter__(self):
        rclpy.init()
        self.node = TruthCollector()
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.th = threading.Thread(target=self.ex.spin, daemon=True)
        self.th.start()
        return self.node

    def __exit__(self, *exc):
        # Ctrl+C makes rclpy shut its context down by itself; only shut down what is still up
        try:
            self.ex.shutdown(timeout_sec=2.0)
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
        return False


def world_offset(odom, start):
    """Gazebo's odometry publisher may report the pose in the world frame or relative to the
    spawn pose. Decide from the first sample (drone on the ground at the start)."""
    x, y = odom[0][2], odom[0][3]
    if math.hypot(x - start['x'], y - start['y']) < 0.3:
        return (0.0, 0.0), 'world'
    if math.hypot(x, y) < 0.3:
        return (start['x'], start['y']), 'relative to spawn'
    return (0.0, 0.0), f'UNKNOWN (first sample {x:.2f}, {y:.2f}; start {start["x"]}, {start["y"]})'


def rtf(clock):
    if len(clock) < 2 or clock[-1][0] <= clock[0][0]:
        return None
    return (clock[-1][1] - clock[0][1]) / (clock[-1][0] - clock[0][0])


def wait_for(cond, timeout, period=0.2):
    t_end = time.time() + timeout
    while time.time() < t_end:
        if cond():
            return True
        time.sleep(period)
    return False
