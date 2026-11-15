#!/usr/bin/env python3
"""C3 Frontier detection (guidebook 4.5) - no simulator needed.

    scripts/run_c.sh C3                 (about 3 minutes)
    python3 drone_sim/tests/C3.py --offline --out /tmp/c3     (same judging on the search code alone, no ROS)

Nav2's map_server serves each fixture, the test broadcasts the drone at the world's start pose (TF map ->
base_link), and drone_frontier's frontier_server publishes /frontiers. For every world (W2-W8, wr_s1-20):
  partial            must report every passable opening listed in openings.json (recall 100 %)
  partial + speckle  the same with 1 % of known free space replaced by 1-3-cell unknown holes
  carto              fully explored, so it must report nothing
  carto + speckle    the same with holes
A reported frontier "matches" a listed frontier if its centroid is within 0.5 m (or half the frontier's length) of
it. A frontier that matches nothing listed is FALSE. Frontiers the drone cannot reach (seen through the 0.45 m gap
of W7) are listed too: they are real, and the mission brain drops them when the planner finds no path.
Also: search time on every map (book: < 50 ms on 300 x 300; W8 is 320 x 320), and an evidence pass on the carto
fixtures with the book's free_threshold 25 (not judged): what the "integration trap" looks like.
Pass (book): recall 100 % of true openings; <= 1 false frontier per fixture on average; timing met.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c_common import FIXTURES, RES, Procs, Truth, fixture_values, fixture_yaml, save, start_lifecycle, start_map_server, stats  # noqa: E402,E501

WS = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(WS, 'drone_frontier'))


def speckle(values, seed):
    """1 % of known free cells turned into unknown holes of 1-3 cells (never touching the drone's start area)."""
    rng = np.random.default_rng(seed)
    v = values.copy()
    fr, fc = np.nonzero((v >= 0) & (v < 65))
    target = int(0.01 * len(fr))
    done = 0
    while done < target:
        i = rng.integers(len(fr))
        r, c = fr[i], fc[i]
        cells = [(r, c)] + [(r + dr, c + dc) for dr, dc in ((0, 1), (1, 0))][:rng.integers(0, 3)]
        for rr, cc in cells:
            if 0 <= rr < v.shape[0] and 0 <= cc < v.shape[1] and 0 <= v[rr, cc] < 65:
                v[rr, cc] = -1
                done += 1
    return v


def listed(world, kind):
    if kind.startswith('partial'):
        return json.load(open(os.path.join(FIXTURES, world, 'openings.json')))['openings']
    return []


def judge(reported, groups):
    """reported: list of dicts (centroid, size_m). Returns dict with recall, false frontiers, unreachable ones."""
    def match(f, g):
        d = math.hypot(f['centroid'][0] - g['centroid'][0], f['centroid'][1] - g['centroid'][1])
        return d <= max(0.5, g['extent_m'] / 2)
    must = [g for g in groups if g['passable']]
    found = [g for g in must if any(match(f, g) for f in reported)]
    false = [f for f in reported if not any(match(f, g) for g in groups if g['real_opening'])]
    unreach = [f for f in reported if any(match(f, g) for g in groups if g['real_opening'] and not g['drone_reachable'])]
    return dict(must=len(must), found=len(found), missed=[g['centroid'] for g in must if g not in found],
                reported=len(reported), false=len(false), false_at=[[round(v, 2) for v in f['centroid']] for f in false],
                unreachable_reported=len(unreach))


# ----------------------------------------------------------------------------- the two ways of running
def run_offline(jobs, free_threshold=65):
    from drone_frontier.frontier_core import Params, search
    out = []
    for world, kind, values, start, origin in jobs:
        t0 = time.perf_counter()
        fr, _ = search(values, RES, origin, start, Params(free_threshold=free_threshold))
        ms = (time.perf_counter() - t0) * 1000
        out.append(([dict(centroid=f.centroid, size_m=f.size_m) for f in fr], ms))
    return out


class RosRunner:
    """map_server + frontier_server as child processes; this node broadcasts the drone pose and listens."""

    def __init__(self, out, procs, free_threshold):
        import rclpy
        from drone_interfaces.msg import FrontierArray
        from geometry_msgs.msg import TransformStamped
        from nav2_msgs.srv import LoadMap
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Float32
        from tf2_ros import TransformBroadcaster
        self.rclpy, self.LoadMap, self.TS = rclpy, LoadMap, TransformStamped
        self.out, self.procs = out, procs
        self.node = rclpy.create_node(f'c3_client_{free_threshold}')
        self.br = TransformBroadcaster(self.node)
        self.pose = (0.0, 0.0)
        self.lock = threading.Lock()
        self.msgs = []           # (t, FrontierArray)
        self.ms = []             # (t, search ms)
        self.node.create_subscription(FrontierArray, '/frontiers', lambda m: self._add(self.msgs, m), 10)
        self.node.create_subscription(Float32, '/frontiers/search_ms', lambda m: self._add(self.ms, m.data), 10)
        self.node.create_timer(0.05, self.tick)
        self.load = self.node.create_client(LoadMap, '/map_server/load_map')
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.th = threading.Thread(target=self.ex.spin, daemon=True)
        self.th.start()

    def _add(self, lst, m):
        with self.lock:
            lst.append((time.time(), m))

    def tick(self):
        t = self.TS()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.pose
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

    def wait_future(self, fut, timeout):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def run(self, jobs, tmp):
        from make_fixtures import write_raw
        res = []
        for world, kind, values, start, origin in jobs:
            meta = {'map': {'origin': list(origin)}}
            base = os.path.join(tmp, f'{world}_{kind}')
            write_raw(base, values, meta, f'C3 {world} {kind}')
            self.pose = start
            time.sleep(0.3)
            r = self.wait_future(self.load.call_async(self.LoadMap.Request(map_url=base + '.yaml')), 20)
            if r is None or r.result != 0:
                res.append((None, None))
                continue
            t_load = time.time()
            # the server searches at 4 Hz: take the third message after the new map (it has surely seen it by then)
            msg, ms, t0 = None, None, time.time()
            while time.time() - t0 < 10:
                with self.lock:
                    after = [m for t, m in self.msgs if t > t_load]
                    times = [v for t, v in self.ms if t > t_load]
                if len(after) >= 3:
                    msg, ms = after[2], (times[2] if len(times) >= 3 else None)
                    break
                time.sleep(0.05)
            if msg is None:
                res.append((None, None))
                continue
            res.append(([dict(centroid=(f.centroid.x, f.centroid.y), size_m=f.size_m) for f in msg.frontiers], ms))
        return res

    def close(self):
        self.ex.shutdown(timeout_sec=1.0)
        self.node.destroy_node()


def build_jobs(worlds):
    jobs = []
    for i, w in enumerate(worlds):
        t = Truth(w)
        start = (t.meta['start']['x'], t.meta['start']['y'])
        origin = tuple(t.meta['map']['origin'])
        part, carto = fixture_values(w, 'partial'), fixture_values(w, 'carto')
        jobs += [(w, 'partial', part, start, origin), (w, 'partial_speckle', speckle(part, 100 + i), start, origin),
                 (w, 'carto', carto, start, origin), (w, 'carto_speckle', speckle(carto, 200 + i), start, origin)]
    return jobs


def main():
    from make_fixtures import DEFAULT
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--offline', action='store_true', help='judge the search code directly, without ROS')
    ap.add_argument('--worlds', nargs='*', default=DEFAULT)
    a, _ = ap.parse_known_args()
    os.makedirs(a.out, exist_ok=True)
    jobs = build_jobs(a.worlds)
    evidence_jobs = [j for j in jobs if j[1] == 'carto']
    lines, res = [], {'mode': 'offline' if a.offline else 'ros', 'fixtures': []}

    if a.offline:
        got = run_offline(jobs)
        ev = run_offline(evidence_jobs, free_threshold=25)
    else:
        import rclpy
        rclpy.init()
        tmp = os.path.join(a.out, 'maps')
        os.makedirs(tmp, exist_ok=True)
        got, ev = [], []
        for thr, js, sink in ((65, jobs, got), (25, evidence_jobs, ev)):
            with Procs(a.out) as procs:
                start_map_server(procs, fixture_yaml(a.worlds[0], 'partial'))
                start_lifecycle(procs, ['map_server'])
                procs.start(f'frontier_server_{thr}', f'ros2 run drone_frontier frontier_server --ros-args '
                            f'-p rate_hz:=4.0 -p free_threshold:={thr} -p use_sim_time:=false')
                rr = RosRunner(a.out, procs, thr)
                if not rr.load.wait_for_service(timeout_sec=30):
                    lines.append('FAIL map_server did not come up')
                    sink += [(None, None)] * len(js)
                else:
                    time.sleep(1.0)
                    sink += rr.run(js, tmp)
                rr.close()
        rclpy.shutdown()

    n_fix, n_false, recall_ok, times, broken = 0, 0, True, [], 0
    for (w, kind, values, _, _), (rep, ms) in zip(jobs, got):
        if rep is None:
            broken += 1
            lines.append(f'FAIL {w}/{kind}: no /frontiers message after loading the map')
            continue
        j = judge(rep, listed(w, kind))
        n_fix += 1
        n_false += j['false']
        if ms is not None:
            times.append((w, values.shape, ms))
        ok = j['found'] == j['must']
        recall_ok &= ok
        rec = dict(world=w, fixture=kind, search_ms=None if ms is None else round(ms, 2), **j)
        res['fixtures'].append(rec)
        if not ok or j['false'] or kind == 'partial':
            lines.append(f"{'ok  ' if ok else 'FAIL'} {w:20s} {kind:16s} found {j['found']}/{j['must']} openings, "
                         f"{j['reported']} reported, false {j['false']}"
                         + (f" at {j['false_at']}" if j['false'] else '')
                         + (f", {j['unreachable_reported']} unreachable (behind a gap)" if j['unreachable_reported'] else '')
                         + (f"; missed {j['missed']}" if j['missed'] else ''))
    false_avg = n_false / max(1, n_fix)
    big = [ms for w, shape, ms in times if shape[0] * shape[1] >= 300 * 300]
    st_all, st_big = stats([ms for _, _, ms in times]), stats(big)
    time_ok = bool(big) and max(big) < 50
    ev_counts = [len(rep) if rep is not None else None for rep, _ in ev]
    res.update(fixtures_judged=n_fix, broken=broken, false_total=n_false, false_per_fixture=round(false_avg, 3),
               search_ms_all=st_all, search_ms_300x300=st_big,
               evidence_free_threshold_25={w: c for (w, *_), c in zip(evidence_jobs, ev_counts)})
    lines.append(f"{'ok  ' if recall_ok and not broken else 'FAIL'} recall: every passable opening found in "
                 f"{n_fix} fixtures" if recall_ok else 'FAIL recall: openings missed (above)')
    lines.append(f"{'ok  ' if false_avg <= 1 else 'FAIL'} false frontiers: {n_false} in {n_fix} fixtures "
                 f"({false_avg:.2f} per fixture, limit 1)")
    lines.append(f"{'ok  ' if time_ok else 'FAIL'} search time on maps >= 300 x 300: max "
                 f"{st_big.get('max')} ms over {st_big['n']} searches (limit 50 ms); all maps median "
                 f"{st_all.get('median')} ms")
    nz = [c for c in ev_counts if c]
    lines.append(f"     evidence, book's free_threshold 25 on the carto fixtures: frontiers reported on {len(nz)} of "
                 f"{len(ev_counts)} fully explored maps (should be 0), up to {max(nz) if nz else 0} per map")
    ok = recall_ok and not broken and false_avg <= 1 and time_ok
    save(a.out, 'C3', res, ok, lines)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
