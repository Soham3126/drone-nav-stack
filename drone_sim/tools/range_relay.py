#!/usr/bin/env python3
"""range_relay.py - simulated MTF-01P (BOM: optical flow + ToF rangefinder), talking MAVLink to ArduPilot
the way the real module does: DISTANCE_SENSOR (RNGFND1_TYPE 10) and OPTICAL_FLOW (FLOW_TYPE 5).

Distance: the Gazebo downward beam (/range_down, LaserScan with one ray).
Flow: computed from Gazebo's true motion, like a real flow sensor sees it, in rad/s:
    flow_rate_x = -v_right / range + roll_rate        flow_rate_y = v_forward / range + pitch_rate
(body front-right-down; the same equations as ArduPilot's own SITL flow sensor), plus noise. ArduPilot's
built-in simulated flow uses ArduPilot's own height/speed from the Gazebo link, which proved unusable here
(B6 with SIM_FLOW: estimate 2.5-3.4 m off in 5 s; B7 no benefit).
It sends through its own MAVLink link (MAVProxy extra output udp 127.0.0.1:14570, sim_up.sh), not through the
bridge, so freezing or killing the bridge (B6) leaves the sensor running - as on the real drone.

    ros2 run drone_sim range_relay.py --ros-args -p use_sim_time:=true [-p flow:=false] [-p flow_noise:=0.02]
"""
import math
import os
import random
import threading
import time

os.environ.setdefault('MAVLINK20', '1')
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from pymavlink import mavutil  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Imu, LaserScan  # noqa: E402

M = mavutil.mavlink


def quat_rotate_inv(q, v):
    """Rotate world vector v into the body frame of unit quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    # conjugate rotation: v_body = q* v q
    x, y, z = -x, -y, -z
    tx = 2 * (y * v[2] - z * v[1])
    ty = 2 * (z * v[0] - x * v[2])
    tz = 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx))


class Mtf01Sim(Node):
    def __init__(self):
        super().__init__('range_relay')
        self.url = self.declare_parameter('url', 'udpin:127.0.0.1:14570').value
        topic = self.declare_parameter('topic', '/range_down').value
        self.flow_on = self.declare_parameter('flow', True).value
        self.flow_noise = self.declare_parameter('flow_noise', 0.02).value      # rad/s, 1 sigma
        self.conn = None
        self.peer_seen = False
        self.sent = 0
        self.lock = threading.Lock()
        self.prev = None                 # (t, x, y, z) world position
        self.vel_w = (0.0, 0.0, 0.0)     # world velocity, low-passed
        self.q = (0.0, 0.0, 0.0, 1.0)
        self.gyro = (0.0, 0.0, 0.0)      # body FLU rad/s
        threading.Thread(target=self.reader, daemon=True).start()
        self.create_subscription(LaserScan, topic, self.cb_range, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu_sim', self.cb_imu, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)
        self.add_on_set_parameters_callback(self.on_params)     # B7 (d): `ros2 param set /range_relay flow false`
        self.get_logger().info(f'range_relay (MTF-01P stand-in): {topic} -> DISTANCE_SENSOR'
                               f'{" + OPTICAL_FLOW" if self.flow_on else ""} on {self.url}')

    def on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'flow':
                self.flow_on = bool(p.value)
                self.get_logger().info(f'flow output {"on" if self.flow_on else "OFF"}')
        return SetParametersResult(successful=True)

    def reader(self):
        """Keeps the UDP port open and read, so replies go back to MAVProxy."""
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
                self.get_logger().info('MAVProxy output reached: sending sensor data')

    def cb_odom(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.prev is not None and t > self.prev[0]:
            dt = t - self.prev[0]
            v = ((p.x - self.prev[1]) / dt, (p.y - self.prev[2]) / dt, (p.z - self.prev[3]) / dt)
            a = min(1.0, dt / 0.05)                       # ~50 ms smoothing of the differenced velocity
            self.vel_w = tuple(self.vel_w[i] + a * (v[i] - self.vel_w[i]) for i in range(3))
        self.prev = (t, p.x, p.y, p.z)
        self.q = (o.x, o.y, o.z, o.w)

    def cb_imu(self, m):
        w = m.angular_velocity
        self.gyro = (w.x, w.y, w.z)

    def cb_range(self, msg):
        if not self.peer_seen or not msg.ranges:
            return
        r = msg.ranges[0]
        if math.isnan(r):
            return
        lo, hi = int(msg.range_min * 100), int(msg.range_max * 100)
        valid = not math.isinf(r) and msg.range_min < r <= msg.range_max
        cm = max(0, int(round(r * 100))) if valid else hi + 1
        now_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        with self.lock:
            try:
                self.conn.mav.distance_sensor_send(now_ms, lo, hi, cm, M.MAV_DISTANCE_SENSOR_LASER, 0,
                                                   M.MAV_SENSOR_ROTATION_PITCH_270, 0)
                if self.flow_on:
                    vf, vl, _ = quat_rotate_inv(self.q, self.vel_w)       # body FLU velocity
                    v_fwd, v_right = vf, -vl
                    p_rate, q_rate = self.gyro[0], -self.gyro[1]         # FLU -> FRD roll / pitch rate
                    if valid and r > 0.05:
                        fx = -v_right / r + p_rate + random.gauss(0, self.flow_noise)
                        fy = v_fwd / r + q_rate + random.gauss(0, self.flow_noise)
                        quality = 200
                    else:
                        fx = fy = 0.0
                        quality = 0
                    self.conn.mav.optical_flow_send(int(time.monotonic() * 1e6), 0, 0, 0, 0.0, 0.0, quality,
                                                    r if valid else -1.0, fx, fy)
                self.sent += 1
            except Exception:  # noqa: BLE001
                pass

    def report(self):
        self.get_logger().info(f'range_relay: {self.sent} distance{"/flow" if self.flow_on else ""} messages in 10 s')
        self.sent = 0


def main():
    rclpy.init()
    n = Mtf01Sim()
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
