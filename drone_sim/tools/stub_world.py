#!/usr/bin/env python3
"""stub_world.py - the stub world as a ROS node (guidebook 4.5, C4-C7). Replaces Gazebo + ArduPilot + Cartographer
+ the bridge with a fast stand-in (stub_core.py), and runs faster than real time.

It provides exactly what the decision layer uses from those parts:
  /clock                       simulation time (everything else runs with use_sim_time:=true)
  TF map -> base_link          the drone (map frame = its start, as Cartographer's)
  /map                         what the simulated LiDAR has seen, Cartographer-like values, 1 Hz
  /bridge/status               10 Hz, as the bridge's
  /bridge/command  (service)   guided / arm / takeoff / land / hold, with the bridge's blocking behaviour
  /bridge/follow_path          paths to follow (the bridge's carrot follower, lookahead 0.6 m)
  /stub/metrics                JSON once a second: coverage, contacts, distance flown, position
Parameters: world (e.g. w3_tree_maze), start_index (0 = the world's own start; others: seeded lattice cells),
speed_factor (simulated seconds per real second), seed; C6 faults: fault_arm_refuse (bool), fault_status_stop_s
(simulated time from which /bridge/status is no longer published; -1 = never).
"""
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from drone_interfaces.msg import BridgeStatus  # noqa: E402
from drone_interfaces.srv import FlightCommand  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

from stub_core import RES, StubCore, start_poses  # noqa: E402


def stamp(t):
    s = int(t)
    return Time(sec=s, nanosec=int((t - s) * 1e9))


class StubWorld(Node):
    def __init__(self):
        super().__init__('stub_world', parameter_overrides=[])
        P = self.declare_parameter
        world = P('world', 'w3_tree_maze').value
        idx = int(P('start_index', 0).value)
        self.speed = float(P('speed_factor', 5.0).value)
        seed = int(P('seed', 0).value)
        self.loop_hz = float(P('loop_hz', 100.0).value)
        start = start_poses(world, idx + 1, seed=seed)[idx]
        self.core = StubCore(world, start_xy=start, seed=seed)
        # C6 fault injection: refuse the arm command; stop publishing /bridge/status from this simulated time
        self.core.fault_arm_refuse = bool(P('fault_arm_refuse', False).value)
        self.status_stop_s = float(P('fault_status_stop_s', -1.0).value)
        self.lock = threading.Lock()
        tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', tl)
        self.pub_status = self.create_publisher(BridgeStatus, '/bridge/status', 10)
        self.pub_metrics = self.create_publisher(String, '/stub/metrics', tl)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Path, '/bridge/follow_path', self.on_path, 10)
        self.create_service(FlightCommand, '/bridge/command', self.on_command)
        self.next_status = self.next_map = self.next_metrics = 0.0
        self.get_logger().info(f'stub world {world}, start {start} (index {idx}), speed x{self.speed}')
        self.running = True
        self.th = threading.Thread(target=self.loop, daemon=True)
        self.th.start()

    # ------------------------------------------------------------------ inputs
    def on_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self.lock:
            self.core.follow(pts)

    def on_command(self, req, res):
        name, value = req.command.strip().lower(), float(req.value)
        with self.lock:
            ok, msg = self.core.command(name, value)
        # the bridge's commands return when they are done: take-off at height, land when disarmed
        if ok and name in ('takeoff', 'land'):
            t0 = time.time()
            while time.time() - t0 < 60.0:
                with self.lock:
                    done = self.core.phase == 'air' if name == 'takeoff' else not self.core.armed
                    z = self.core.z
                if done:
                    break
                time.sleep(0.005)
            msg += f' | height {z:.2f} m reached' if name == 'takeoff' else ' | disarmed'
            ok = ok and done
        res.success, res.message = ok, msg
        return res

    # ------------------------------------------------------------------ the simulation loop (real time x speed)
    def loop(self):
        period = 1.0 / self.loop_hz
        nxt = time.monotonic()
        while self.running and rclpy.ok():
            with self.lock:
                dt = self.speed * period
                while dt > 1e-9:
                    h = min(0.05, dt)
                    self.core.step(h)
                    dt -= h
                t = self.core.t
                self.publish(t)
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))

    def publish(self, t):
        st = stamp(t)
        self.pub_clock.publish(Clock(clock=st))
        x, y = self.core.pose_map()
        tr = TransformStamped()
        tr.header.stamp, tr.header.frame_id, tr.child_frame_id = st, 'map', 'base_link'
        tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = x, y, self.core.z
        tr.transform.rotation.w = 1.0
        self.tf.sendTransform(tr)
        if t >= self.next_status and not (0.0 <= self.status_stop_s <= t):
            self.next_status = t + 0.1
            m = BridgeStatus()
            m.header.stamp = st
            for k, v in self.core.status().items():
                setattr(m, k, v)
            self.pub_status.publish(m)
        if t >= self.next_map:
            self.next_map = t + 1.0
            g = OccupancyGrid()
            g.header.stamp, g.header.frame_id = st, 'map'
            g.info.resolution = RES
            g.info.height, g.info.width = self.core.known.shape
            g.info.origin.position.x, g.info.origin.position.y = self.core.map_origin()
            g.info.origin.orientation.w = 1.0
            g.data = np.clip(self.core.known, -1, 100).astype(np.int8).ravel().tolist()
            self.pub_map.publish(g)
        if t >= self.next_metrics:
            self.next_metrics = t + 1.0
            self.pub_metrics.publish(String(data=json.dumps(self.core.metrics())))


def main():
    rclpy.init()
    n = StubWorld()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.running = False
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
