#!/usr/bin/env python3
"""pose_mangler.py - pose in -> pose out with configurable delay, noise, drift, steps and dropouts
(guidebook tools table; used by B3, B4, B7). The maths is in class Mangler (no ROS) and unit-tested in B0.

ROS node:
  ros2 run ... / python3 pose_mangler.py --ros-args -p input:=/truth/pose -p output:=/pose_in \
      -p delay:=0.05 -p noise:=0.02 -p seed:=1 \
      -p drift_start:=30.0 -p drift_duration:=3.0 -p drift_vx:=0.1 -p drift_vy:=0.0 \
      -p step_time:=30.0 -p step_x:=0.3 -p step_y:=0.0 -p step_yaw_deg:=0.0 -p step_reset:=false \
      -p dropout_start:=-1.0 -p dropout_duration:=0.0
Times (drift_start, step_time, dropout_start) are seconds since the first input message.
The node also publishes /pose_mangler/reset_counter (std_msgs/UInt8): incremented by a step with
step_reset:=true, so the bridge can tell ArduPilot that the pose jumped on purpose.
"""
import math
import random


class Mangler:
    def __init__(self, delay=0.0, noise=0.0, seed=0,
                 drift_start=-1.0, drift_duration=0.0, drift_vx=0.0, drift_vy=0.0,
                 step_time=-1.0, step_x=0.0, step_y=0.0, step_yaw=0.0, step_reset=False,
                 dropout_start=-1.0, dropout_duration=0.0):
        self.delay = delay
        self.noise = noise
        self.rng = random.Random(seed)
        self.drift = (drift_start, drift_duration, drift_vx, drift_vy)
        self.step = (step_time, step_x, step_y, step_yaw, step_reset)
        self.dropout = (dropout_start, dropout_duration)
        self.queue = []          # (release_time, t_stamp, pose)
        self.reset_counter = 0
        self._stepped = False

    def offset(self, t):
        """(dx, dy, dyaw) added at time t (seconds since start), excluding noise."""
        dx = dy = dyaw = 0.0
        s, dur, vx, vy = self.drift
        if s >= 0 and t > s:
            k = min(t, s + dur) - s
            dx += vx * k
            dy += vy * k
        st, sx, sy, syaw, _ = self.step
        if st >= 0 and t >= st:
            dx += sx
            dy += sy
            dyaw += syaw
        return dx, dy, dyaw

    def dropped(self, t):
        s, dur = self.dropout
        return s >= 0 and s <= t < s + dur

    def apply(self, t, x, y, z, yaw):
        """Modified pose for an input at time t, or None if dropped. Stamp is unchanged
        (a delayed pose still carries its original time, as a slow SLAM would)."""
        if self.dropped(t):
            return None
        st, _, _, _, reset = self.step
        if reset and st >= 0 and t >= st and not self._stepped:
            self._stepped = True
            self.reset_counter = (self.reset_counter + 1) % 256
        dx, dy, dyaw = self.offset(t)
        if self.noise > 0:
            dx += self.rng.gauss(0.0, self.noise)
            dy += self.rng.gauss(0.0, self.noise)
        yaw = math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw))
        return x + dx, y + dy, z, yaw

    def push(self, t, x, y, z, yaw, extra=None):
        """Queue an input; it becomes available `delay` seconds later. `extra` is passed through."""
        out = self.apply(t, x, y, z, yaw)
        if out is not None:
            self.queue.append((t + self.delay, t, out, extra))

    def pop_ready(self, now):
        """All queued (stamp, pose, extra) whose release time has come, in order."""
        ready = [(ts, p, ex) for (rel, ts, p, ex) in self.queue if rel <= now + 1e-9]
        self.queue = [q for q in self.queue if q[0] > now + 1e-9]
        return ready


def main():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from std_msgs.msg import UInt8

    class ManglerNode(Node):
        def __init__(self):
            super().__init__('pose_mangler')
            p = {k: self.declare_parameter(k, v).value for k, v in dict(
                input='/truth/pose', output='/pose_in', delay=0.0, noise=0.0, seed=0,
                drift_start=-1.0, drift_duration=0.0, drift_vx=0.0, drift_vy=0.0,
                step_time=-1.0, step_x=0.0, step_y=0.0, step_yaw_deg=0.0, step_reset=False,
                dropout_start=-1.0, dropout_duration=0.0).items()}
            self.m = Mangler(p['delay'], p['noise'], p['seed'], p['drift_start'], p['drift_duration'],
                             p['drift_vx'], p['drift_vy'], p['step_time'], p['step_x'], p['step_y'],
                             math.radians(p['step_yaw_deg']), p['step_reset'],
                             p['dropout_start'], p['dropout_duration'])
            self.t0 = None
            self.last_msg = None
            self.pub = self.create_publisher(PoseStamped, p['output'], 10)
            self.pub_reset = self.create_publisher(UInt8, '/pose_mangler/reset_counter', 10)
            self.create_subscription(PoseStamped, p['input'], self.cb, 10)
            self.create_timer(0.005, self.flush)
            self.get_logger().info(f'pose_mangler {p}')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def cb(self, msg):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.t0 is None:
                self.t0 = t
            q = msg.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.last_msg = msg
            p = msg.pose.position
            self.m.push(t - self.t0, p.x, p.y, p.z, yaw, (yaw, (q.x, q.y, q.z, q.w)))

        def flush(self):
            if self.t0 is None:
                return
            for ts, (x, y, z, yaw), (yaw_in, q) in self.m.pop_ready(self.now() - self.t0):
                out = PoseStamped()
                out.header.frame_id = self.last_msg.header.frame_id
                sec = ts + self.t0
                out.header.stamp.sec = int(sec)
                out.header.stamp.nanosec = int((sec - int(sec)) * 1e9)
                out.pose.position.x, out.pose.position.y, out.pose.position.z = x, y, z
                # keep roll/pitch: rotate the input attitude about the vertical by the yaw change
                h = (yaw - yaw_in) / 2
                cz, sz = math.cos(h), math.sin(h)
                qx, qy, qz, qw = q
                o = out.pose.orientation
                o.x, o.y = cz * qx - sz * qy, cz * qy + sz * qx
                o.z, o.w = cz * qz + sz * qw, cz * qw - sz * qz
                self.pub.publish(out)
            self.pub_reset.publish(UInt8(data=self.m.reset_counter))

    rclpy.init()
    node = ManglerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
