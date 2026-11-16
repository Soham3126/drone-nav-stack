#!/usr/bin/env python3
"""fault_planner.py - C6 fault injection: a relay between the mission brain and the planner server.

The brain is started with planner_action:=/c6/compute_path_to_pose; this node serves that action and passes each
request on to the real /compute_path_to_pose, unchanged. From simulated time fail_after_s on, the FIRST goal
requested becomes the target: every request for a goal within target_radius_m of it is aborted (as the planner
does when it finds no path) for hold_s seconds; all other requests still go through.
The target and the refusals are written to <record> (JSON) for C6 to check against the brain's events.
"""
import json
import math
import threading
import time

import rclpy
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FaultPlanner(Node):
    def __init__(self):
        super().__init__('fault_planner')
        P = self.declare_parameter
        self.after = float(P('fail_after_s', 60.0).value)
        self.radius = float(P('target_radius_m', 0.6).value)
        self.hold = float(P('hold_s', 60.0).value)
        self.record = P('record', '').value
        self.target, self.t_target, self.refused, self.passed = None, None, [], 0
        self.lock = threading.Lock()
        cb = ReentrantCallbackGroup()
        self.real = ActionClient(self, ComputePathToPose, '/compute_path_to_pose', callback_group=cb)
        ActionServer(self, ComputePathToPose, '/c6/compute_path_to_pose', self.execute, callback_group=cb,
                     goal_callback=lambda g: GoalResponse.ACCEPT, cancel_callback=lambda g: CancelResponse.ACCEPT)
        self.get_logger().info(f'fault planner: refuse the first goal after {self.after} s (and goals within '
                               f'{self.radius} m of it) for {self.hold} s')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def save(self):
        if self.record:
            json.dump(dict(target=self.target, t_target=self.t_target, refused=self.refused, passed=self.passed),
                      open(self.record, 'w'), indent=1)

    def execute(self, handle):
        g = handle.request.goal.pose.position
        xy, t = (round(g.x, 3), round(g.y, 3)), self.now()
        with self.lock:
            if self.target is None and t >= self.after:
                self.target, self.t_target = xy, round(t, 2)
                self.get_logger().warn(f'target {xy} at t {t:.1f}: refusing it for {self.hold} s')
            refuse = (self.target is not None and t - self.t_target <= self.hold
                      and math.hypot(xy[0] - self.target[0], xy[1] - self.target[1]) <= self.radius)
            if refuse:
                self.refused.append([round(t, 2), list(xy)])
                self.save()
        if refuse:
            handle.abort()
            return ComputePathToPose.Result()
        # pass it on to the real planner
        if not self.real.wait_for_server(timeout_sec=5.0):
            handle.abort()
            return ComputePathToPose.Result()
        fut = self.real.send_goal_async(handle.request)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 10.0:
            time.sleep(0.002)
        h = fut.result() if fut.done() else None
        if h is None or not h.accepted:
            handle.abort()
            return ComputePathToPose.Result()
        rf = h.get_result_async()
        t0 = time.time()
        while not rf.done() and time.time() - t0 < 15.0:
            time.sleep(0.002)
        r = rf.result() if rf.done() else None
        with self.lock:
            self.passed += 1
        if r is None or r.status != 4:
            handle.abort()
            return r.result if r is not None else ComputePathToPose.Result()
        handle.succeed()
        return r.result


def main():
    rclpy.init()
    n = FaultPlanner()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.save()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
