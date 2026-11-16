"""stub_core.py - the "stub world" for workstream C (guidebook 4.5, C4-C7), without ROS.

A fast stand-in for Gazebo + ArduPilot + Cartographer + the bridge, good enough to test the decision layer:
  * the drone: a point that follows a path with the bridge's own carrot follower (lookahead 0.6 m) at 0.5 m/s,
    with a 0.3 s response lag; take-off / land / hold like the bridge's commands
  * the LiDAR: 360 deg, 10 m, raycast in the TRUE world several times a second
  * the map: what the LiDAR has seen, in Cartographer-like values (free 0-30, walls 60-95, unknown -1),
    in a map frame whose origin is the drone's start (as Cartographer's)
  * referee: contacts (the drone's 0.27 m circle against the true walls), coverage of the reachable free space,
    distance flown
Everything runs on its own simulated clock (step(dt)); the ROS node (stub_world.py) drives it faster than real time.
"""
import math
import os
import sys

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_fixtures import RES, load_drone_reach, load_world  # noqa: E402

for _p in (os.path.join(os.path.dirname(os.path.dirname(HERE)), 'drone_bridge'),):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.append(_p)
from drone_bridge.carrot import CarrotFollower  # noqa: E402

ROBOT_R = 0.27


class StubCore:
    def __init__(self, world, start_xy=None, speed=0.5, lag=0.3, lidar_range=10.0, rays=1080, scan_hz=5.0,
                 lookahead=0.6, flight_height=1.1, contact_tol=0.02, seed=0, max_cut=0.05):
        self.meta, self.occ, self.reach = load_world(world)
        self.dreach = load_drone_reach(world) & ~self.occ
        self.world = world
        self.ox, self.oy = self.meta['map']['origin']
        if start_xy is None:
            start_xy = (self.meta['start']['x'], self.meta['start']['y'])
        self.start_w = tuple(start_xy)                     # start in WORLD coordinates
        self.rng = np.random.default_rng(seed)
        self.speed, self.lag = speed, lag
        self.range, self.rays, self.scan_period = lidar_range, rays, 1.0 / scan_hz
        self.flight_height, self.contact_tol = flight_height, contact_tol
        self.known = np.full(self.occ.shape, -1, np.int16)
        self.t = 0.0
        self.p = np.array(self.start_w, float)            # world frame
        self.v = np.zeros(2)
        self.z = 0.0
        self.armed, self.mode = False, 'STABILIZE'
        self.phase = 'ground'                              # ground | takeoff | air | landing
        # the stand-in drone keeps the bend-aware follower it passed C4-C7 with (its crude speed model needs it);
        # the real bridge uses the fixed lookahead (carrot.py)
        self.carrot = CarrotFollower(lookahead, max_cut=max_cut, keep_speed=True)
        self.following = False
        self.hold = None                                   # world (x, y) to hold
        self.next_scan = 0.0
        self.contacts, self.in_contact, self.min_clear = 0, False, 9.0
        self.contact_log = []
        self.fault_arm_refuse = False  # C6: the arm command is refused          # one entry per contact: where, how deep, was the wall in the map yet
        self.flown = 0.0
        self.angles = np.linspace(0, 2 * math.pi, rays, endpoint=False)
        self.steps = np.arange(0.0, lidar_range, RES * 0.5)
        self.scan()

    # ------------------------------------------------------------------ frames
    def to_map(self, xy):
        return (xy[0] - self.start_w[0], xy[1] - self.start_w[1])

    def to_world(self, xy):
        return (xy[0] + self.start_w[0], xy[1] + self.start_w[1])

    def map_origin(self):
        """Origin of the published map in the MAP frame."""
        return (self.ox - self.start_w[0], self.oy - self.start_w[1])

    def pose_map(self):
        return self.to_map(self.p)

    # ------------------------------------------------------------------ commands (bridge semantics)
    def command(self, name, value=0.0):
        """Starts a command; returns (accepted, message). Completion is seen in the state (z, armed, phase)."""
        if name == 'guided':
            self.mode = 'GUIDED'
            return True, 'mode GUIDED'
        if name == 'arm':
            if self.fault_arm_refuse:                    # C6 fault injection
                return False, 'PreArm: injected fault (C6)'
            if self.mode != 'GUIDED':
                return False, 'arm refused: not in GUIDED'
            self.armed = True
            return True, 'armed'
        if name == 'takeoff':
            if not self.armed:
                return False, 'takeoff refused: not armed'
            self.flight_height = float(value) if value else self.flight_height
            self.phase = 'takeoff'
            self.hold = tuple(self.p)
            return True, 'takeoff accepted'
        if name == 'land':
            self.following = False
            self.mode = 'LAND'
            self.phase = 'landing'
            self.hold = tuple(self.p)
            return True, 'land accepted'
        if name == 'hold':
            self.following = False
            self.hold = tuple(self.p)
            return True, 'holding'
        return False, f'unknown command {name}'

    def follow(self, pts_map):
        if not pts_map:
            self.following = False
            self.hold = tuple(self.p)
            return
        self.carrot.set_path([self.to_world(q) for q in pts_map])
        self.following = True

    # ------------------------------------------------------------------ simulation
    def step(self, dt):
        self.t += dt
        if self.phase == 'takeoff':
            self.z = min(self.flight_height, self.z + 0.5 * dt)
            if self.z >= self.flight_height - 1e-6:
                self.phase = 'air'
        elif self.phase == 'landing':
            self.z = max(0.0, self.z - 0.4 * dt)
            if self.z <= 0.0:
                self.phase, self.armed, self.mode = 'ground', False, 'LAND'
                self.v[:] = 0
        target = None
        if self.phase == 'air' and self.following and self.carrot.active:
            target = np.array(self.carrot.update(*self.p))
            self.carrot.check_arrived(self.p[0], self.p[1], float(np.hypot(*self.v)), self.t)
        elif self.phase in ('air', 'takeoff', 'landing') and self.hold is not None:
            target = np.array(self.hold)
        if target is not None:
            d = target - self.p
            n = float(np.hypot(*d))
            # speed command toward the target, slowing in the last 0.5 m (a position controller)
            vc = d / n * min(self.speed, n / 0.5 * self.speed) if n > 1e-6 else np.zeros(2)
        else:
            vc = np.zeros(2)
        self.v += (vc - self.v) * min(1.0, dt / self.lag)
        step = self.v * dt
        self.p += step
        self.flown += float(np.hypot(*step))
        if self.z > 0.05:
            c = self.clearance(*self.p)
            self.min_clear = min(self.min_clear, c)
            touching = c < -self.contact_tol
            if touching and not self.in_contact:
                self.contacts += 1
                mx, my = self.pose_map()
                self.contact_log.append(dict(t=round(self.t, 1), at=[round(mx, 2), round(my, 2)], depth_m=round(c, 3),
                                             wall_in_map=self.wall_known_near(0.30), following=bool(self.following)))
                self.contact_log = self.contact_log[-50:]
            elif touching and self.contact_log:
                self.contact_log[-1]['depth_m'] = min(self.contact_log[-1]['depth_m'], round(c, 3))
            self.in_contact = touching
        if self.t >= self.next_scan:
            self.scan()
            self.next_scan = self.t + self.scan_period

    def wall_known_near(self, r):
        """Is a wall cell already in the published map within r of the drone (centre)?"""
        x, y = self.p
        k = int(r / RES) + 1
        r0, c0 = int((y - self.oy) / RES), int((x - self.ox) / RES)
        win = self.known[max(0, r0 - k):r0 + k + 1, max(0, c0 - k):c0 + k + 1]
        return bool((win >= 65).any())

    def clearance(self, x, y, window=0.6):
        """Distance from the drone's edge to the nearest true wall square (m)."""
        r0 = int(math.floor((y - self.oy) / RES))
        c0 = int(math.floor((x - self.ox) / RES))
        k = int(window / RES) + 1
        rs = slice(max(0, r0 - k), r0 + k + 1)
        cs = slice(max(0, c0 - k), c0 + k + 1)
        rr, cc = np.nonzero(self.occ[rs, cs])
        if not len(rr):
            return window - ROBOT_R
        x0 = self.ox + (cc + cs.start) * RES
        y0 = self.oy + (rr + rs.start) * RES
        dx = np.maximum(np.maximum(x0 - x, 0), x - (x0 + RES))
        dy = np.maximum(np.maximum(y0 - y, 0), y - (y0 + RES))
        return float(np.min(np.hypot(dx, dy))) - ROBOT_R

    def scan(self):
        x, y = self.p
        H, W = self.occ.shape
        cols = np.floor((x + self.steps[None, :] * np.cos(self.angles)[:, None] - self.ox) / RES).astype(int)
        rows = np.floor((y + self.steps[None, :] * np.sin(self.angles)[:, None] - self.oy) / RES).astype(int)
        inside = (rows >= 0) & (rows < H) & (cols >= 0) & (cols < W)
        hit = np.zeros_like(inside)
        hit[inside] = self.occ[rows[inside], cols[inside]]
        stop = ~inside | hit
        first = np.where(stop.any(axis=1), stop.argmax(axis=1), self.steps.size)
        before = (np.arange(self.steps.size)[None, :] < first[:, None]) & inside
        fr, fc = rows[before], cols[before]
        new_free = self.known[fr, fc] < 0
        self.known[fr[new_free], fc[new_free]] = self.rng.integers(0, 31, int(new_free.sum()))
        k = np.nonzero(first < self.steps.size)[0]
        hk = k[hit[k, first[k]]]
        wr, wc = rows[hk, first[hk]], cols[hk, first[hk]]
        # a wall cell next to seen free space is seen (grazing rays), as in the C0 fixtures
        seen = np.zeros(self.occ.shape, bool)
        seen[fr, fc] = True
        r0, r1 = max(0, int((y - self.range - self.oy) / RES) - 2), min(H, int((y + self.range - self.oy) / RES) + 3)
        c0, c1 = max(0, int((x - self.range - self.ox) / RES) - 2), min(W, int((x + self.range - self.ox) / RES) + 3)
        win = ndimage.binary_dilation(seen[r0:r1, c0:c1], structure=np.ones((3, 3))) & self.occ[r0:r1, c0:c1]
        wrr, wcc = np.nonzero(win)
        wr = np.concatenate([wr, wrr + r0])
        wc = np.concatenate([wc, wcc + c0])
        newwall = self.known[wr, wc] < 60
        self.known[wr[newwall], wc[newwall]] = self.rng.integers(60, 96, int(newwall.sum()))

    # ------------------------------------------------------------------ status, referee
    def status(self):
        arrived = bool(self.following and self.carrot.arrived)
        dist_end = self.carrot.distance_to_end(*self.p) if (self.following and self.carrot.active) else 0.0
        return dict(fc_connected=True, mode=self.mode, armed=self.armed, ekf_position_ok=True, origin_set=True,
                    height_m=float(self.z), battery_percent=100.0, frame_offset_m=0.0, frame_offset_deg=0.0,
                    following_path=bool(self.following and not arrived), path_arrived=arrived,
                    distance_to_path_end_m=float(dist_end))

    def coverage(self):
        """Share of the free space the drone can reach that the map shows as known free."""
        tgt = self.dreach
        return float(((self.known >= 0) & (self.known < 65) & tgt).sum() / max(1, tgt.sum()))

    def metrics(self):
        mx, my = self.pose_map()
        return dict(t=round(self.t, 2), coverage=round(self.coverage(), 4), contacts=self.contacts,
                    min_clearance_m=round(self.min_clear, 3), flown_m=round(self.flown, 2),
                    pos_map=[round(mx, 3), round(my, 3)], dist_from_start_m=round(math.hypot(mx, my), 3),
                    z=round(self.z, 2), armed=self.armed, phase=self.phase, contact_log=self.contact_log)


def start_poses(world, n, seed):
    """n different start poses (world frame): lattice cell centres the drone fits in, seeded."""
    import random
    meta, occ, _ = load_world(world)
    dre = load_drone_reach(world) & ~occ
    face = ndimage.distance_transform_edt(~occ) * RES
    ox, oy = meta['map']['origin']
    cell = meta['cell_size_m']
    nx, ny = meta['lattice_cells']
    ok = []
    for iy in range(ny):
        for ix in range(nx):
            x, y = (ix + 0.5) * cell, (iy + 0.5) * cell
            r, c = int((y - oy) / RES), int((x - ox) / RES)
            if dre[r, c] and face[r, c] >= ROBOT_R + 0.1:
                ok.append((x, y))
    s0 = (meta['start']['x'], meta['start']['y'])
    rnd = random.Random(seed)
    rest = [p for p in ok if math.hypot(p[0] - s0[0], p[1] - s0[1]) > 0.1]
    rnd.shuffle(rest)
    return ([s0] + rest)[:n]
