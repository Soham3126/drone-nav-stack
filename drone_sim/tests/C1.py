#!/usr/bin/env python3
"""C1 Planner validity (guidebook 4.5) - no simulator needed.

    scripts/run_c.sh C1                (about 2-3 minutes)

Nav2's map_server serves the C0 fixtures, a static transform map -> base_link stands in for the drone, and the
planner server runs drone_nav/config/planner.yaml (Smac 2D, 0.27 m radius, lethal threshold 65, tolerance 0.5 m).
For each case in c1_cases.py the test asks /compute_path_to_pose for a path (with an explicit start pose) and judges
the result on the TRUE map of the world:

  case                         expected (book)                                       judged by
  corridor 1.0/1.3/1.5 m (W6)  success, path centred                                 mean offset from the centre line
                                                                                     < 0.05 m on the straight part
  0.45 m gap (W7)              no valid path                                         planner fails, full and carto-like
  0.60 m gap (W7)              success on full; carto-like: recorded                 reaches the room
  0.90 m gap (W7)              success                                               (extra case)
  goal in unknown, 0.3 m from  success; path ends within 0.5 m of the goal, in      partial fixtures W2, W3, W4, W8
  known free                   known free space
  goal inside a wall (W3)      fails, or a path to free space within 0.5 m
  start 0.2 m from a wall (W3) fails or finds a way out: recorded
  longest path (W8)            success; planning time recorded
  20 more W8 goals             for the timing figure
Every path: clearance >= 0 (the drone's 0.27 m circle never touches a wall), judged with a tolerance of one map
cell (0.05 m) because the map itself is drawn in 5 cm cells; the start-in-inflation path is judged only after it
has left the wall's inflation. Paths on partial maps are judged against the walls the partial map shows (a planner
cannot avoid a wall nobody has seen); their clearance to the true walls is reported next to it. Planning time p95 < 200 ms over the W8 cases.

Humble note: Nav2 Humble's ComputePathToPose returns no error code (205-208 came in a later Nav2); a failure is
an aborted goal, and the planner's reason is in planner_server.log.
"""
import argparse
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c_common import (Procs, Truth, fixture_values, fixture_yaml, planner_params, save, start_lifecycle,  # noqa: E402
                      start_map_server, start_planner, start_static_tf, stats)

CELL_TOL = 0.05


class Client:
    def __init__(self):
        import rclpy
        from nav2_msgs.action import ComputePathToPose
        from nav2_msgs.srv import LoadMap
        from nav_msgs.msg import OccupancyGrid
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self.rclpy, self.Goal = rclpy, ComputePathToPose.Goal
        self.node = rclpy.create_node('c1_client')
        self.act = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        self.load = self.node.create_client(LoadMap, '/map_server/load_map')
        self.LoadReq = LoadMap.Request
        self.cost = None
        self.cost_n = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_cost, qos)

    def on_cost(self, m):
        self.cost = m
        self.cost_n += 1

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def load_map(self, yaml_path, values, timeout=20.0):
        """Load a fixture and wait until the planner's cost map shows it: same size, and its lethal cells (100)
        exactly the fixture's cells >= 65."""
        fut = self.load.call_async(self.LoadReq(map_url=yaml_path))
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        if fut.result() is None or fut.result().result != 0:
            return False, 'map_server could not load it'
        want = int((values >= 65).sum())
        n0, t0 = self.cost_n, time.time()
        while time.time() - t0 < timeout:
            self.spin(0.2)
            m = self.cost
            if m is not None and self.cost_n > n0 and (m.info.height, m.info.width) == values.shape:
                got = int((np.asarray(m.data) == 100).sum())
                if got == want:
                    return True, f'cost map ready ({got} lethal cells)'
        last = None if self.cost is None else int((np.asarray(self.cost.data) == 100).sum())
        return False, f'cost map never matched the fixture (lethal cells {last}, expected {want})'

    def plan(self, start, goal, timeout=10.0):
        from geometry_msgs.msg import PoseStamped
        g = self.Goal()
        for ps, (x, y) in ((g.start, start), (g.goal, goal)):
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
        g.use_start, g.planner_id = True, 'GridBased'
        _ = PoseStamped
        t0 = time.time()
        fut = self.act.send_goal_async(g)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        h = fut.result()
        if h is None or not h.accepted:
            return dict(ok=False, why='goal rejected', wall_s=time.time() - t0)
        rf = h.get_result_async()
        self.rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result()
        wall = time.time() - t0
        if r is None:
            return dict(ok=False, why='no result in time', wall_s=wall)
        if r.status != 4:                                        # 4 = SUCCEEDED
            return dict(ok=False, why=f'aborted (status {r.status})', wall_s=wall)
        pts = [(p.pose.position.x, p.pose.position.y) for p in r.result.path.poses]
        pt = r.result.planning_time.sec + r.result.planning_time.nanosec * 1e-9
        return dict(ok=bool(pts), why='' if pts else 'empty path', path=pts, planning_s=pt, wall_s=wall)


class KnownWalls:
    """Clearance against a given wall mask (e.g. the walls a partial map knows) on the world's grid."""

    def __init__(self, t, occ):
        self.t, self.occ = t, occ

    def clearance(self, x, y):
        saved = self.t.occ
        self.t.occ = self.occ
        try:
            return self.t.clearance(x, y)
        finally:
            self.t.occ = saved


def path_len(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def centring(t, case, pts):
    """Mean |offset| from the corridor centre line, on path points more than 0.5 m from both run ends and with
    a wall on both sides within one corridor width (side openings and junctions are skipped)."""
    (sx, sy), (gx, gy) = case['start'], case['goal']
    reach = t.meta['cell_size_m']
    steps = np.arange(0, reach, 0.005)

    def wall_at(x, y, dx, dy):
        return next((d for d in steps if t.occ[t.cell(x + dx * d, y + dy * d)]), None)
    offs = []
    for x, y in pts:
        along, lo_hi = (x, (min(sx, gx), max(sx, gx))) if case['run_axis'] == 'x' else (y, (min(sy, gy), max(sy, gy)))
        if not (lo_hi[0] + 0.5 <= along <= lo_hi[1] - 0.5):
            continue
        if case['run_axis'] == 'x':
            lo, hi = wall_at(x, y, 0, -1), wall_at(x, y, 0, 1)
        else:
            lo, hi = wall_at(x, y, -1, 0), wall_at(x, y, 1, 0)
        if lo is None or hi is None:
            continue
        offs.append(abs((hi - lo) / 2))
    return (float(np.mean(offs)), float(np.max(offs)), len(offs)) if offs else (float('nan'), float('nan'), 0)


def judge(case, r, t, part):
    """Returns (ok, line, record)."""
    rec = {k: v for k, v in case.items() if k not in ('start', 'goal')}
    rec.update(start=case['start'], goal=case['goal'], planned=r['ok'], why=r.get('why', ''),
               planning_ms=round(r['planning_s'] * 1000, 1) if r.get('planning_s') is not None else None)
    exp = case['expect']
    txt = f"{case['id']:22s} {case['world']}/{case['fixture']:7s}"
    if not r['ok']:
        ok = exp in ('fail', 'fail_or_tolerance', 'record') or (exp == 'record')
        return ok, f"{'ok  ' if ok else 'FAIL'} {txt} no path ({r['why']}); expected {exp}", rec
    pts = r['path']
    truth_clear = [t.clearance(x, y) for x, y in pts]
    if case['fixture'] == 'partial':
        # judged against the walls the planner could know (the partial map); walls it never saw are reported only
        known = KnownWalls(t, part == 100)
        clear = [known.clearance(x, y) for x, y in pts]
    else:
        clear = truth_clear
    if case['judge'] == 'record':                              # start inside the inflation: judge after leaving it
        k = next((i for i, c in enumerate(clear) if c >= 0), len(clear))
        clear = clear[k:]
    cmin = min(clear) if clear else float('nan')
    end = pts[-1]
    d_goal = math.hypot(end[0] - case['goal'][0], end[1] - case['goal'][1])
    rec.update(points=len(pts), length_m=round(path_len(pts), 2), min_clearance_m=round(cmin, 3),
               end=[round(end[0], 3), round(end[1], 3)], end_to_goal_m=round(d_goal, 3))
    clear_ok = not clear or cmin >= -CELL_TOL
    extra = ''
    if exp == 'fail':
        ok = False
        extra = 'found a path where none should exist'
    elif case['judge'] == 'centred':
        mean_off, max_off, n = centring(t, case, pts)
        rec.update(centre_offset_mean_m=round(mean_off, 3), centre_offset_max_m=round(max_off, 3), centre_points=n)
        ok = n > 0 and mean_off < 0.05 and d_goal <= 0.1
        extra = f'centre offset mean {mean_off * 100:.1f} cm (max {max_off * 100:.1f}) over {n} points'
    elif case['judge'] == 'near_goal_known_free':
        r_, c_ = t.cell(*end)
        known = bool(part[r_, c_] == 0)
        ok = d_goal <= 0.5 and known and t.is_free(*end)
        rec.update(end_known_free=known)
        extra = f'ends {d_goal:.2f} m from the goal, in known free space: {known}'
    elif case['judge'] == 'near_goal_free':
        ok = d_goal <= 0.5 and t.is_free(*end)
        extra = f'ends {d_goal:.2f} m from the goal (inside the wall), in free space: {t.is_free(*end)}'
    elif case['judge'] == 'record':
        ok = True
        extra = f'found a way out ({len(pts)} points)'
    else:                                                       # reach_goal
        ok = d_goal <= 0.1 or exp == 'record'
        extra = f'ends {d_goal:.2f} m from the goal'
    ok = ok and clear_ok
    if case['fixture'] == 'partial':
        tmin = min(truth_clear)
        rec['min_clearance_truth_m'] = round(tmin, 3)
        if tmin < -CELL_TOL:
            extra += f' [passes {-tmin * 100:.0f} cm too close to a wall the partial map does not show yet]'
    pm = rec['planning_ms']
    line = (f"{'ok  ' if ok else 'FAIL'} {txt} path {rec['length_m']} m, {extra}; clearance min "
            f"{cmin * 100:+.1f} cm{'' if clear_ok else ' (TOUCHES A WALL)'}; planning {pm} ms; expected {exp}")
    return ok, line, rec


def main():
    import rclpy
    from c1_cases import build_cases

    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a, _ = ap.parse_known_args()
    cases = build_cases()
    lines, res = [], {'cases': []}
    params, src = planner_params(a.out)
    res['planner_yaml'] = src
    rclpy.init()
    cl = Client()
    with Procs(a.out) as procs:
        first = cases[0]
        start_map_server(procs, fixture_yaml(first['world'], first['fixture']))
        start_static_tf(procs)
        start_planner(procs, params)
        start_lifecycle(procs, ['map_server', 'planner_server'])
        up = cl.act.wait_for_server(timeout_sec=60.0) and cl.load.wait_for_service(timeout_sec=10.0)
        if not up:
            lines.append('FAIL planner server or map server did not come up (see planner_server.log, lifecycle.log)')
            save(a.out, 'C1', res, False, lines)
            return 1
        cl.spin(2.0)
        current, part_cache, truth_cache, n_ok = None, {}, {}, 0
        for case in cases:
            key = (case['world'], case['fixture'])
            if key != current:
                vals = fixture_values(*key)
                ready, why = cl.load_map(fixture_yaml(*key), vals)
                if not ready:
                    lines.append(f"FAIL {case['world']}/{case['fixture']}: {why}")
                    res['cases'].append(dict(id=case['id'], error=why))
                    current = None
                    continue
                current = key
            t = truth_cache.setdefault(case['world'], Truth(case['world']))
            part = part_cache.setdefault(case['world'], fixture_values(case['world'], 'partial'))
            r = cl.plan(case['start'], case['goal'])
            ok, line, rec = judge(case, r, t, part)
            rec['pass'] = ok
            res['cases'].append(rec)
            lines.append(line)
            n_ok += ok
        alive = procs.alive()
    cl.node.destroy_node()
    rclpy.shutdown()

    w8 = [c['planning_ms'] for c in res['cases'] if c.get('world') == 'w8_arena_full' and c.get('fixture') == 'full'
          and c.get('planning_ms') is not None]
    st = stats(w8)
    time_ok = st['n'] > 0 and st['p95'] < 200
    res.update(w8_planning_ms=st, cases_ok=n_ok, cases_total=len(cases), nodes_alive_at_end=alive)
    lines.append(f"{'ok  ' if n_ok == len(cases) else 'FAIL'} {n_ok} of {len(cases)} cases as expected")
    lines.append(f"{'ok  ' if time_ok else 'FAIL'} W8 planning time: median {st.get('median')} ms, p95 "
                 f"{st.get('p95')} ms, max {st.get('max')} ms over {st['n']} paths (limit p95 < 200 ms)")
    ok = n_ok == len(cases) and time_ok
    save(a.out, 'C1', res, ok, lines)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
