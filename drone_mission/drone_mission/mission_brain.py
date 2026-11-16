#!/usr/bin/env python3
"""mission_brain - the only commander (guidebook 2.8): take off, explore until no frontiers remain, return, land.

The decisions are in mission_core.py (no ROS); this node connects them to the system:
  in:   /bridge/status, /frontiers, /map (for the "map present" check), TF map -> base_link, planner action
        /compute_path_to_pose, optional /scan and /imu rates (check_sensors:=true on the real drone)
  out:  /bridge/follow_path, /bridge/command calls, /mission/state (latched text), /mission/events (JSON lines)
  services: /mission/start, /mission/abort (std_srvs/Trigger)
The decision loop runs in its own thread on simulation time (use_sim_time:=true in simulation and the stub).
"""
import json
import threading
import time

import numpy as np
import rclpy
from drone_interfaces.msg import BridgeStatus, FrontierArray
from drone_interfaces.srv import FlightCommand
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .mission_core import MissionCore, Params

STATUS_FIELDS = ('fc_connected', 'mode', 'armed', 'ekf_position_ok', 'origin_set', 'height_m', 'battery_percent',
                 'frame_offset_m', 'frame_offset_deg', 'following_path', 'path_arrived', 'distance_to_path_end_m')


class Iface:
    """mission_core's view of the world, backed by ROS."""

    def __init__(self, node):
        self.n = node

    def now(self):
        return self.n.get_clock().now().nanoseconds * 1e-9

    def sleep(self, dt):
        end = self.now() + dt
        while self.now() < end and rclpy.ok():
            time.sleep(0.002)

    def pose(self):
        try:
            t = self.n.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:  # noqa: BLE001
            return None

    def status(self):
        return self.n.status

    def status_age(self):
        return self.now() - self.n.status_t if self.n.status is not None else 1e9

    def frontiers(self):
        return self.n.fseq, self.n.fr

    def frontiers_age(self):
        return self.now() - self.n.fr_t if self.n.fseq else 1e9

    def plan(self, goal):
        g = ComputePathToPose.Goal()
        g.goal = PoseStamped()
        g.goal.header.frame_id = 'map'
        g.goal.header.stamp = self.n.get_clock().now().to_msg()
        g.goal.pose.position.x, g.goal.pose.position.y = float(goal[0]), float(goal[1])
        g.goal.pose.orientation.w = 1.0
        g.planner_id = self.n.planner_id
        # plan from where the drone is NOW (TF), not from the cost map's own idea of the robot pose: that one
        # lags, and at 5x simulation speed it was up to 0.5 m behind, so paths began on the far side of a wall end
        pose = self.pose()
        g.use_start = pose is not None
        if pose is not None:
            g.start.header = g.goal.header
            g.start.pose.position.x, g.start.pose.position.y = float(pose[0]), float(pose[1])
            g.start.pose.orientation.w = 1.0
        if not self.n.planner.wait_for_server(timeout_sec=5.0):
            self.log('warning', what='planner action server not available')
            return None
        h = self._wait(self.n.planner.send_goal_async(g), 10.0)
        if h is None or not h.accepted:
            return None
        r = self._wait(h.get_result_async(), 15.0)
        if r is None or r.status != 4:
            return None
        pts = [(p.pose.position.x, p.pose.position.y) for p in r.result.path.poses]
        self.n.plan_ms.append((r.result.planning_time.sec + r.result.planning_time.nanosec * 1e-9) * 1000)
        return pts or None

    def _wait(self, fut, timeout):
        # the planner and command clients live on a private node that only this (decision) thread spins:
        # calling an action client from one thread while another executor handles its replies loses replies
        # in Humble's rclpy (the brain waited out every timeout and stalled)
        self.n.io_exec.spin_until_future_complete(fut, timeout_sec=timeout)
        if not fut.done():
            self.log('warning', what='no reply in time', timeout_s=timeout)
            return None
        return fut.result()

    def follow(self, path):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.n.get_clock().now().to_msg()
        for x, y in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.n.pub_path.publish(msg)

    def command(self, name, value):
        if not self.n.cmd.wait_for_service(timeout_sec=5.0):
            return False, 'service /bridge/command not available'
        r = self._wait(self.n.cmd.call_async(FlightCommand.Request(command=name, value=float(value))), 120.0)
        return (False, 'no answer from /bridge/command') if r is None else (r.success, r.message)

    def map_grid(self):
        m = self.n.map
        if m is None:
            return None
        if getattr(self, '_grid_msg', None) is not m:          # one array per /map message (wall field cache)
            self._grid_msg = m
            self._grid = (np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width),
                          m.info.resolution, (m.info.origin.position.x, m.info.origin.position.y))
        return self._grid

    def checks(self):
        n = self.n
        out = {}
        m, pose = n.map, self.pose()
        near = False
        if m is not None and pose is not None:
            g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
            r = int((pose[1] - m.info.origin.position.y) / m.info.resolution)
            c = int((pose[0] - m.info.origin.position.x) / m.info.resolution)
            k = int(1.0 / m.info.resolution)
            win = g[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1]
            near = win.size > 0 and float((win >= 0).mean()) > 0.2   # walls hide what is behind them: ~35 % known at the start
        out['map_present'] = (near, '')
        out['planner'] = (n.planner.server_is_ready(), '')
        out['frontiers'] = (self.frontiers_age() < 3.0, '')
        if n.check_sensors:
            now = time.time()
            scan = sum(1 for t in n.scan_t if now - t <= 5.0) / 5.0
            imu = sum(1 for t in n.imu_t if now - t <= 5.0) / 5.0
            out['lidar_rate'] = (9.0 <= scan <= 11.0, f'{scan:.1f} Hz')
            out['imu_rate'] = (imu >= 100.0, f'{imu:.0f} Hz')
        return out

    def log(self, event, **data):
        rec = dict(t=round(self.now(), 2), event=event, **data)
        self.n.pub_events.publish(String(data=json.dumps(rec, default=str)))
        if event == 'state':
            self.n.pub_state.publish(String(data=data['state']))
        # rclpy refuses one call site logging at two severities: keep the two calls on separate lines
        text = f'{event} {json.dumps(data, default=str)}'
        if event in ('warning', 'blacklist', 'escape', 'stop_land'):
            self.n.get_logger().warning(text)
        else:
            self.n.get_logger().info(text)


class MissionBrain(Node):
    def __init__(self):
        super().__init__('mission_brain')
        P = self.declare_parameter
        p = Params()
        for k, v in vars(p).items():
            setattr(p, k, type(v)(P(k, v).value))
        self.planner_id = P('planner_id', 'GridBased').value
        self.check_sensors = P('check_sensors', False).value
        self.status, self.status_t = None, 0.0
        self.fseq, self.fr, self.fr_t = 0, [], 0.0
        self.map = None
        self.scan_t, self.imu_t = [], []
        self.plan_ms = []
        cb = ReentrantCallbackGroup()
        tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(BridgeStatus, '/bridge/status', self.on_status, 10, callback_group=cb)
        self.create_subscription(FrontierArray, '/frontiers', self.on_frontiers, 10, callback_group=cb)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, tl, callback_group=cb)
        if self.check_sensors:
            self.create_subscription(LaserScan, '/scan', lambda m: self._rate(self.scan_t), qos_profile_sensor_data)
            self.create_subscription(Imu, '/imu', lambda m: self._rate(self.imu_t), qos_profile_sensor_data)
        self.pub_path = self.create_publisher(Path, '/bridge/follow_path', 10)
        self.pub_state = self.create_publisher(String, '/mission/state', tl)
        self.pub_events = self.create_publisher(String, '/mission/events', 100)
        self.io_node = rclpy.create_node('mission_brain_io', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=self.get_parameter('use_sim_time').value)])
        self.io_exec = SingleThreadedExecutor()
        self.io_exec.add_node(self.io_node)
        self.planner = ActionClient(self.io_node, ComputePathToPose, '/compute_path_to_pose')
        self.cmd = self.io_node.create_client(FlightCommand, '/bridge/command')
        self.create_service(Trigger, '/mission/start', self.on_start, callback_group=cb)
        self.create_service(Trigger, '/mission/abort', self.on_abort, callback_group=cb)
        self.core = MissionCore(Iface(self), p)
        self.th = threading.Thread(target=self.run, daemon=True)
        self.th.start()

    def _rate(self, lst):
        now = time.time()
        lst.append(now)
        while lst and now - lst[0] > 6.0:
            lst.pop(0)

    def on_status(self, m):
        self.status = {k: getattr(m, k) for k in STATUS_FIELDS}
        self.status_t = self.get_clock().now().nanoseconds * 1e-9

    def on_frontiers(self, m):
        self.fr = [dict(middle=(f.middle.x, f.middle.y), centroid=(f.centroid.x, f.centroid.y), cost=f.cost,
                        size_m=f.size_m) for f in m.frontiers]
        self.fseq += 1
        self.fr_t = self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, m):
        self.map = m

    def on_start(self, req, res):
        ok = self.core.state == 'WAIT_READY'
        self.core.start_requested = ok
        res.success, res.message = ok, ('mission starts when the checks pass' if ok else f'state {self.core.state}')
        return res

    def on_abort(self, req, res):
        self.core.abort_requested = True
        res.success, res.message = True, 'abort requested: STOP + LAND'
        return res

    def run(self):
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:        # wait for /clock in simulation
            time.sleep(0.05)
        final = self.core.run()
        st = self.core.stats
        Iface(self).log('summary', final=final, reason=self.core.reason, goal_switches=st.goal_switches,
                        blacklisted=st.blacklisted, plans=st.plans, plan_failures=st.plan_failures,
                        escapes=self.core.escapes, path_fixes=self.core.path_fixes,
                        planning_ms=dict(n=len(self.plan_ms), p95=round(float(np.percentile(self.plan_ms, 95)), 1)
                                         if self.plan_ms else None,
                                         max=round(max(self.plan_ms), 1) if self.plan_ms else None))


def main():
    rclpy.init()
    n = MissionBrain()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
