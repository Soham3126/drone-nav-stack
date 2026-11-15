#!/usr/bin/env python3
"""cpu_probe.py - how much CPU does an otherwise idle rclpy node spend just receiving a topic? (B9)

  python3 cpu_probe.py --mode clock|tf|both|none [--seconds 15]
    clock  use_sim_time=True: rclpy receives every /clock message (Gazebo: one per 1 ms physics step)
    tf     a tf2 TransformListener (Cartographer publishes map->base_link at 200 Hz)
    both   the two together (what the bridge does in the slam profile)
    none   nothing (the baseline cost of spinning)
Prints one line: mode, messages per second received, CPU % of one core.
"""
import argparse
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='both')
    ap.add_argument('--seconds', type=float, default=15.0)
    a = ap.parse_args()
    rclpy.init()
    sim = a.mode in ('clock', 'both')
    node = Node('cpu_probe_' + a.mode, parameter_overrides=[Parameter('use_sim_time', value=sim)])
    counts = {'clock': 0, 'tf': 0}
    if sim:   # count the /clock messages rclpy's time source is already receiving
        node.create_subscription(Clock, '/clock', lambda m: counts.__setitem__('clock', counts['clock'] + 1),
                                 qos_profile_sensor_data)
    if a.mode in ('tf', 'both'):
        from tf2_ros import Buffer, TransformListener
        node.buf = Buffer()
        node.tl = TransformListener(node.buf, node)
        node.create_subscription(TFMessage, '/tf', lambda m: counts.__setitem__('tf', counts['tf'] + 1), 100)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    t_end = time.monotonic() + 3.0
    while time.monotonic() < t_end:          # settle (discovery)
        ex.spin_once(timeout_sec=0.1)
    counts.update(clock=0, tf=0)
    c0, w0 = time.process_time(), time.monotonic()
    while time.monotonic() - w0 < a.seconds:
        ex.spin_once(timeout_sec=0.1)
    dw = time.monotonic() - w0
    cpu = 100.0 * (time.process_time() - c0) / dw
    print(f"probe {a.mode:5s}: /clock {counts['clock'] / dw:7.1f}/s  /tf {counts['tf'] / dw:6.1f}/s  "
          f"CPU {cpu:5.1f}% of one core", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
