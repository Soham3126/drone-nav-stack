#!/usr/bin/env python3
"""range_relay.py - simulation stand-in for a rangefinder wired to the flight controller.

The Gazebo downward beam (/range_down, LaserScan with one ray) is sent to ArduPilot as DISTANCE_SENSOR
(RNGFND1_TYPE 10, MAVLink) through its own MAVLink link: MAVProxy's extra output udp 127.0.0.1:14570
(added by sim_up.sh). It is deliberately NOT inside the bridge, so freezing or killing the bridge (test B6)
leaves the flight controller's height sensor running, as on the real drone where the MTF-01P is wired to
the flight controller.

    ros2 run drone_sim range_relay.py --ros-args -p use_sim_time:=true
"""
import math
import os
import threading
import time

os.environ.setdefault('MAVLINK20', '1')
import rclpy  # noqa: E402
from pymavlink import mavutil  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

M = mavutil.mavlink


class RangeRelay(Node):
    def __init__(self):
        super().__init__('range_relay')
        self.url = self.declare_parameter('url', 'udpin:127.0.0.1:14570').value
        topic = self.declare_parameter('topic', '/range_down').value
        self.conn = None
        self.peer_seen = False
        self.sent = 0
        self.lock = threading.Lock()
        threading.Thread(target=self.reader, daemon=True).start()
        self.create_subscription(LaserScan, topic, self.cb, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'range_relay: {topic} -> DISTANCE_SENSOR on {self.url}')

    def reader(self):
        """Opens the UDP port and keeps reading, so MAVLink knows where MAVProxy is (replies go there)."""
        while rclpy.ok():
            if self.conn is None:
                try:
                    self.conn = mavutil.mavlink_connection(self.url, source_system=1,
                                                           source_component=M.MAV_COMP_ID_PERIPHERAL)
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warn(f'open {self.url}: {e}')
                    time.sleep(1.0)
                    continue
            try:
                m = self.conn.recv_match(blocking=True, timeout=1.0)
            except Exception:  # noqa: BLE001
                m = None
            if m is not None and not self.peer_seen:
                self.peer_seen = True
                self.get_logger().info('MAVProxy output reached: sending DISTANCE_SENSOR')

    def cb(self, msg):
        if not self.peer_seen or not msg.ranges:
            return
        r = msg.ranges[0]
        if math.isnan(r):
            return
        lo, hi = int(msg.range_min * 100), int(msg.range_max * 100)
        cm = hi + 1 if (math.isinf(r) or r > msg.range_max) else max(0, int(round(r * 100)))
        with self.lock:
            try:
                self.conn.mav.distance_sensor_send(int(time.monotonic() * 1000) & 0xFFFFFFFF, lo, hi, cm,
                                                   M.MAV_DISTANCE_SENSOR_LASER, 0,
                                                   M.MAV_SENSOR_ROTATION_PITCH_270, 0)
                self.sent += 1
            except Exception:  # noqa: BLE001
                pass

    def report(self):
        self.get_logger().info(f'range_relay: {self.sent} DISTANCE_SENSOR sent in 10 s')
        self.sent = 0


def main():
    rclpy.init()
    n = RangeRelay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
