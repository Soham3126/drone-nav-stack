#!/usr/bin/env python3
"""c4_offline.py - the whole decision loop WITHOUT ROS, for quick checks of the mission brain's logic:
stub world (stub_core) + frontier search (drone_frontier.frontier_core) + a grid planner that imitates Nav2's
Smac 2D on the same cost map (lethal >= 65, 0.27 m inscribed radius, inflation 0.5 m, unknown not allowed,
0.5 m goal tolerance) + the mission brain (drone_mission.mission_core).

    python3 drone_sim/tests/c4_offline.py w3_tree_maze [--start-index 0] [--limit 1200]

It is NOT the C4 test (that one runs the real planner server, frontier server and mission brain nodes in ROS);
it is how the brain's logic was developed and checked before going to ROS.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(os.path.dirname(HERE))
for p in (os.path.join(os.path.dirname(HERE), 'tools'), os.path.join(WS, 'drone_frontier'),
          os.path.join(WS, 'drone_mission'), os.path.join(WS, 'drone_bridge')):
    sys.path.insert(0, p)
from drone_frontier.frontier_core import Params as FParams, search  # noqa: E402
from drone_mission.mission_core import MissionCore, Params  # noqa: E402
from stub_core import RES, StubCore, start_poses  # noqa: E402


class GridPlanner:
    """Dijkstra over the cost map, rebuilt when the map snapshot changes."""

    def __init__(self, inscribed=0.27, inflation=0.5, k=5.0, travel=5.0, tol=0.5, inflate_unknown=False):
        self.ins, self.inf, self.k, self.travel, self.tol = inscribed, inflation, k, travel, tol
        self.inflate_unknown = inflate_unknown
        self.smoothing = True
        self.key = None

    def build(self, known, key):
        if key == self.key:
            return
        self.key = key
        lethal = known >= 65
        if self.inflate_unknown:
            lethal = lethal | (known < 0)
        d = ndimage.distance_transform_edt(~lethal) * RES if lethal.any() else np.full(known.shape, 9.0)
        self.valid = (known >= 0) & ~lethal & (d > self.ins)
        cost = np.where(d < self.inf, 252 * np.exp(-self.k * (d - self.ins)), 0.0)
        H, W = known.shape
        idx = np.arange(H * W).reshape(H, W)
        rows, cols, w = [], [], []

        def pair(a, dr, dc):
            """a[r, c] and a[r + dr, c + dc] for every (r, c) where both exist."""
            r0, r1 = max(0, -dr), H - max(0, dr)
            c0, c1 = max(0, -dc), W - max(0, dc)
            return a[r0:r1, c0:c1], a[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
        for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
            va, vb = pair(self.valid, dr, dc)
            ia, ib = pair(idx, dr, dc)
            ca, cb = pair(cost, dr, dc)
            m = va & vb
            ww = math.hypot(dr, dc) * RES * (1 + self.travel * (ca[m] + cb[m]) / 2 / 252)
            rows += [ia[m], ib[m]]
            cols += [ib[m], ia[m]]
            w += [ww, ww]
        r, c, ww = np.concatenate(rows), np.concatenate(cols), np.concatenate(w)
        self.g = csr_matrix((ww, (r, c)), shape=(H * W, H * W))
        self.shape = (H, W)

    def plan(self, origin, start, goal):
        H, W = self.shape
        ox, oy = origin
        cell = lambda p: (int((p[1] - oy) / RES), int((p[0] - ox) / RES))  # noqa: E731
        sr, sc = cell(start)
        if not (0 <= sr < H and 0 <= sc < W) or not self.valid[sr, sc]:
            return None                                          # start in collision: Smac refuses too
        gr, gc = cell(goal)
        k = int(self.tol / RES) + 1
        rr, cc = np.mgrid[max(0, gr - k):min(H, gr + k + 1), max(0, gc - k):min(W, gc + k + 1)]
        near = (np.hypot(rr - gr, cc - gc) * RES <= self.tol) & self.valid[rr, cc]
        if 0 <= gr < H and 0 <= gc < W and self.valid[gr, gc]:
            targets = [(gr, gc)]
        else:
            targets = list(zip(rr[near], cc[near]))
        if not targets:
            return None
        dist_, pred = dijkstra(self.g, indices=sr * W + sc, return_predecessors=True)
        tid = [r * W + c for r, c in targets]
        best = min(tid, key=lambda i: dist_[i] + 1e-3 * math.hypot(i // W - gr, i % W - gc))
        if not np.isfinite(dist_[best]):
            return None
        out, i = [], best
        while i != -9999 and i != sr * W + sc:
            out.append(i)
            i = pred[i]
        out.append(sr * W + sc)
        out.reverse()
        pts = [(ox + (i % W + 0.5) * RES, oy + (i // W + 0.5) * RES) for i in out]
        return self.smooth(pts, origin) if self.smoothing else pts

    def smooth(self, pts, origin, w_smooth=0.3, w_data=0.2, max_iter=1000, tol=1e-10):
        """Smac's smoother (Humble, nav2_smac_planner Smoother::smoothImpl): pull every inner point toward its
        neighbours' mean and toward its original place; if an iteration puts a point in collision (within the
        drone's radius of a wall), return the path from before that iteration."""
        if len(pts) < 3:
            return pts
        ox, oy = origin
        H, W = self.shape
        orig = np.array(pts, float)
        cur = orig.copy()
        for _ in range(max_iter):
            last = cur.copy()
            change = 0.0
            for i in range(1, len(cur) - 1):
                old = cur[i].copy()
                cur[i] += w_data * (orig[i] - cur[i]) + w_smooth * (cur[i - 1] + cur[i + 1] - 2 * cur[i])
                change += float(np.abs(cur[i] - old).sum())
                r, c = int((cur[i][1] - oy) / RES), int((cur[i][0] - ox) / RES)
                if not (0 <= r < H and 0 <= c < W) or not self.valid[r, c]:
                    return [tuple(p) for p in last]
            if change < tol:
                break
        return [tuple(p) for p in cur]


class OfflineIface:
    LAG = False
    def __init__(self, stub, log_events=True):
        self.s = stub
        self.planner = GridPlanner()
        self.events = []
        self.fseq, self.fr, self.ft = 0, [], -1.0
        self.next_map, self.snap, self.snap_t = 0.0, None, -1
        self.prev_snap, self.prev_t = None, -1
        self.verbose = log_events
        self.update()

    def update(self):
        if self.s.t >= self.next_map:                      # /map at 1 Hz, frontiers right after
            self.prev_snap, self.prev_t = self.snap, self.snap_t
            self.snap = self.s.known.copy()
            self.snap_t = self.s.t
            self.next_map = self.s.t + 1.0
            fr, _ = search(self.snap, RES, self.s.map_origin(), self.s.pose_map(), FParams())
            self.fr = [dict(middle=f.middle, centroid=f.centroid, cost=f.cost, size_m=f.size_m) for f in fr]
            self.fseq += 1
            self.ft = self.s.t

    def now(self):
        return self.s.t

    def sleep(self, dt):
        end = self.s.t + dt
        while self.s.t < end - 1e-9:
            self.s.step(min(0.05, end - self.s.t))
            self.update()

    def pose(self):
        return self.s.pose_map()

    def status(self):
        return self.s.status()

    def status_age(self):
        return 0.0

    def frontiers(self):
        return self.fseq, self.fr

    def frontiers_age(self):
        return self.s.t - self.ft

    def plan(self, goal):
        # ROS: the cost map is refreshed at 1 Hz from a /map that is itself up to 1 s old -> use the previous
        # snapshot (LAG=True) to imitate that; the start is where the drone was a moment ago
        snap, key = (self.prev_snap, self.prev_t) if (self.LAG and self.prev_snap is not None) else (self.snap,
                                                                                                    self.snap_t)
        self.planner.build(snap, key)
        return self.planner.plan(self.s.map_origin(), self.s.pose_map(), goal)

    def follow(self, path):
        self.s.follow(path)

    def map_grid(self):
        return (self.snap, RES, self.s.map_origin()) if self.snap is not None else None

    def command(self, name, value):
        ok, msg = self.s.command(name, value)
        if ok and name == 'takeoff':
            self.sleep(0.1)
            while self.s.phase == 'takeoff':
                self.sleep(0.1)
            msg += f' | height {self.s.z:.2f} m reached'
        if ok and name == 'land':
            while self.s.armed:
                self.sleep(0.1)
            msg += ' | disarmed'
        return ok, msg

    def checks(self):
        return {'map': (self.snap is not None and (self.snap >= 0).any(), ''), 'planner': (True, ''),
                'frontiers': (self.fseq > 0, '')}

    def log(self, event, **data):
        self.events.append(dict(t=round(self.s.t, 2), event=event, **data))
        if self.verbose and event in ('state', 'blacklist', 'warning', 'explore_done', 'home', 'stop_land'):
            print(f'  [{self.s.t:7.1f}] {event} {data}', flush=True)


def run(world, start_index=0, limit=1200.0, verbose=True, seed=0):
    sp = start_poses(world, start_index + 1, seed=seed)[start_index]
    stub = StubCore(world, start_xy=sp, seed=seed)
    io = OfflineIface(stub, verbose)
    m = MissionCore(io, Params())
    m.start_requested = True
    # cap the simulated time: abort when over
    orig = io.sleep

    def capped(dt):
        orig(dt)
        if stub.t > limit:
            m.abort_requested = True
    io.sleep = capped
    t0 = time.time()
    final = m.run()
    met = stub.metrics()
    sw = [x[0] for x in m.stats.goal_switches]
    explore_t = next((e['t'] for e in io.events if e['event'] == 'state' and e['state'] == 'RETURN'), stub.t)
    start_t = next((e['t'] for e in io.events if e['event'] == 'state' and e['state'] == 'EXPLORE'), 0.0)
    return dict(world=world, start=sp, final=final, reason=m.reason, wall_s=round(time.time() - t0, 1),
                goal_switches=len(sw), explore_s=round(explore_t - start_t, 1),
                switches_per_4s=round(len(sw) / max(1.0, (explore_t - start_t) / 4.0), 3),
                aba=aba_switches(m.stats.goal_switches), blacklisted=len(m.stats.blacklisted),
                plans=m.stats.plans, plan_failures=m.stats.plan_failures, **met)


def aba_switches(sw, window=10.0, r=0.5):
    """A -> B -> A within `window` s, where going back to A ABANDONED B (B was still an open frontier).
    Coming back to A because B was reached / explored / unreachable is not a change of mind. sw: (t, xy[, abandoned])."""
    n = 0
    for i in range(2, len(sw)):
        t, a = sw[i][0], sw[i][1]
        if len(sw[i]) > 2 and not sw[i][2]:
            continue
        for j in range(i - 1):
            b = sw[j][1]
            left = sw[j + 1][0]
            if t - left <= window and math.hypot(a[0] - b[0], a[1] - b[1]) <= r:
                n += 1
                break
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world')
    ap.add_argument('--start-index', type=int, default=0)
    ap.add_argument('--limit', type=float, default=1200.0)
    a = ap.parse_args()
    print(json.dumps(run(a.world, a.start_index, a.limit), indent=1))


if __name__ == '__main__':
    main()
