"""i_common.py - shared pieces for the integration tests I1-I7 (guidebook 4.6): watch a full mission in Gazebo
and measure it against ground truth.

Frames: Cartographer's map frame is the drone's pose at the start (it locks on the ground before take-off, start
yaw 0 in every world), so a map point + the world start = the Gazebo world point.
Measured:
  coverage     share of the drone-reachable floor (maps/<world>/reachable_drone.pgm, eroded 0.1 m as in the book)
               that is known free in the final /map (0 <= value < 65, the planner's rule)
  contacts     the drone's 0.27 m circle (ground truth) against walls and tall boxes, while airborne; a contact
               is counted when it goes more than 2 cm in (map cells are 5 cm)
  landing      horizontal distance between the ground-truth start and the final ground-truth position
  ATE          RMS of |Cartographer's pose (moved into the world) - ground truth| while airborne
  frame offset the bridge's own estimate of FC frame vs pose frame (BridgeStatus.frame_offset_m), largest seen
"""
import json
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'tools'))
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from drone_interfaces.msg import BridgeStatus  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

from geom import ROBOT_R, clearance_at, load_meta, obstacle_rects  # noqa: E402
from make_fixtures import load_drone_reach  # noqa: E402

CONTACT_TOL = 0.02
AIR_DZ = 0.3


class MissionWatch(Node):
    def __init__(self, world):
        super().__init__('i_test_watch')
        self.meta = load_meta(world)
        self.world = world
        self.start = (self.meta['start']['x'], self.meta['start']['y'])
        self.rects = obstacle_rects(self.meta, flight=True)
        self.lock = threading.Lock()
        self.events, self.state, self.status = [], '', None
        self.truth, self.est = [], []                # (t_wall, x, y[, z]) world frame / map frame
        self.z0 = None
        self.map = None
        self.offset_max = 0.0
        self.contacts, self.in_contact, self.min_clear = 0, False, 9.0
        self.contact_log = []
        tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/mission/events', self.cb_event, 200)
        self.create_subscription(String, '/mission/state', lambda m: setattr(self, 'state', m.data), tl)
        self.create_subscription(BridgeStatus, '/bridge/status', self.cb_status, 20)
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb_truth, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), tl)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.start_cli = self.create_client(Trigger, '/mission/start')
        self.create_timer(0.1, self.sample_est)

    # ------------------------------------------------------------------ inputs
    def cb_event(self, m):
        with self.lock:
            self.events.append(json.loads(m.data))

    def cb_status(self, m):
        self.status = m
        if self.airborne():
            self.offset_max = max(self.offset_max, float(m.frame_offset_m))

    def cb_truth(self, m):
        p = m.pose.pose.position
        if self.z0 is None:
            self.z0 = p.z
        with self.lock:
            self.truth.append((time.time(), p.x, p.y, p.z))
            if len(self.truth) > 400000:
                del self.truth[:200000]
        if p.z - self.z0 > AIR_DZ:
            c = clearance_at(p.x, p.y, self.rects) - ROBOT_R
            self.min_clear = min(self.min_clear, c)
            touching = c < -CONTACT_TOL
            if touching and not self.in_contact:
                self.contacts += 1
                self.contact_log.append(dict(t=round(time.time(), 1), at=[round(p.x, 2), round(p.y, 2)],
                                             depth_m=round(c, 3)))
            self.in_contact = touching

    def sample_est(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return
        with self.lock:
            self.est.append((time.time(), t.transform.translation.x, t.transform.translation.y))

    def airborne(self):
        with self.lock:
            if not self.truth or self.z0 is None:
                return False
            return self.truth[-1][3] - self.z0 > AIR_DZ

    # ------------------------------------------------------------------ actions
    def wait_ready(self, timeout=240.0, settle=20.0):
        """Until the bridge reports the FC linked, EKF using our pose and origin set, then settle seconds more
        (ArduPilot's own pre-arm checks take a while after that in SITL)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = self.status
            if s is not None and s.fc_connected and s.ekf_position_ok and s.origin_set:
                time.sleep(settle)
                return True
            time.sleep(0.5)
        return False

    def start_mission(self):
        if not self.start_cli.wait_for_service(timeout_sec=60.0):
            return False, 'no /mission/start'
        fut = self.start_cli.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 10.0:
            time.sleep(0.05)
        r = fut.result() if fut.done() else None
        return (bool(r and r.success), r.message if r else 'no answer')

    def wait_end(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.state in ('DONE', 'STOP') and any(e.get('event') == 'summary' for e in self.events):
                time.sleep(3.0)
                return True
            time.sleep(0.5)
        return False

    # ------------------------------------------------------------------ results
    def coverage(self):
        m = self.map
        if m is None:
            return None
        mres = self.meta['map']['resolution']
        # guidebook 4.6 (coverage): the reachable free space eroded 0.1 m - a 5 cm map cannot put a wall face
        # on the exact cell, and SLAM moves the whole map a few cm (I2 run 3: the unshrunk floor gave 93.9 %
        # with not one unknown cell left; every missed cell was a 1-2 cell strip along a wall face)
        from scipy import ndimage
        reach = ndimage.binary_erosion(load_drone_reach(self.world), iterations=int(round(0.1 / mres)))
        ox, oy = self.meta['map']['origin']
        rr, cc = np.nonzero(reach)
        wx, wy = ox + (cc + 0.5) * mres, oy + (rr + 0.5) * mres
        g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        mx, my = wx - self.start[0], wy - self.start[1]                           # world -> map frame
        c = np.floor((mx - m.info.origin.position.x) / m.info.resolution).astype(int)
        r = np.floor((my - m.info.origin.position.y) / m.info.resolution).astype(int)
        inside = (r >= 0) & (r < g.shape[0]) & (c >= 0) & (c < g.shape[1])
        v = np.full(rr.shape, -1)
        v[inside] = g[r[inside], c[inside]]
        return round(float(((v >= 0) & (v < 65)).mean()), 4)

    def ate(self):
        with self.lock:
            T = np.array([p for p in self.truth if self.z0 is not None and p[3] - self.z0 > AIR_DZ])
            E = np.array(self.est)
        if len(T) < 10 or len(E) < 10:
            return None
        ex = np.interp(T[:, 0], E[:, 0], E[:, 1]) + self.start[0]
        ey = np.interp(T[:, 0], E[:, 0], E[:, 2]) + self.start[1]
        ok = (T[:, 0] >= E[0, 0]) & (T[:, 0] <= E[-1, 0])
        if ok.sum() < 10:
            return None
        return round(float(np.sqrt(np.mean((ex[ok] - T[ok, 1]) ** 2 + (ey[ok] - T[ok, 2]) ** 2))), 4)

    def landing(self):
        with self.lock:
            if not self.truth:
                return None
            x, y = self.truth[-1][1], self.truth[-1][2]
        return round(math.hypot(x - self.start[0], y - self.start[1]), 3)


def spin_in_thread(node):
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    return ex
