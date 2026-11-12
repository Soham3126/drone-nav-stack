#!/usr/bin/env python3
"""stream_gate.py - relay one topic and pause it on request (guidebook tool list; A1.11, B6).

    ros2 run drone_sim stream_gate.py --ros-args -r __node:=scan_gate -p type:=sensor_msgs/msg/LaserScan \
        -p input:=/scan -p output:=/scan_gated
Pause / resume:  ros2 service call /scan_gate/pause std_srvs/srv/SetBool "{data: true}"
While paused, messages are dropped (as when a sensor or its driver stops), not queued.
"""
import importlib

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool


def msg_class(name):
    pkg, _, cls = name.split('/')
    return getattr(importlib.import_module(f'{pkg}.msg'), cls)


class StreamGate(Node):
    def __init__(self):
        super().__init__('stream_gate')
        cls = msg_class(self.declare_parameter('type', 'sensor_msgs/msg/LaserScan').value)
        src = self.declare_parameter('input', '/scan').value
        dst = self.declare_parameter('output', '/scan_gated').value
        self.paused, self.dropped = False, 0
        self.pub = self.create_publisher(cls, dst, qos_profile_sensor_data)
        self.create_subscription(cls, src, self.cb, qos_profile_sensor_data)
        self.create_service(SetBool, '~/pause', self.on_pause)
        self.get_logger().info(f'{src} -> {dst} ({cls.__name__})')

    def cb(self, m):
        if self.paused:
            self.dropped += 1
        else:
            self.pub.publish(m)

    def on_pause(self, req, res):
        self.paused = bool(req.data)
        self.get_logger().info(('PAUSED' if self.paused else f'RESUMED (dropped {self.dropped})'))
        if not self.paused:
            self.dropped = 0
        res.success, res.message = True, 'paused' if self.paused else 'running'
        return res


def main():
    rclpy.init()
    n = StreamGate()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
