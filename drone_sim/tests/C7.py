#!/usr/bin/env python3
"""C7 Timing (guidebook 4.5): what the decision layer costs, measured on W8 at real-time speed.

    scripts/run_c.sh C7 w8 -- [--runs 1]                        (one W8 mission at 1x: about 15 minutes)

One mission (c4_one.py --timing) on the stub at speed 1.0, so every time below is real time:
  planner time per request    the planner's own planning_time (simulated clock = real at 1x) and, as a cross-check,
                              the brain's request -> answer time on the wall clock (includes the ROS round trip)
  frontier search time        /frontiers/search_ms, every search
  mission-loop period         time between the decision loop's 0.5 s sleeps (sleep + the work of one loop)
  CPU per process             % of one core for each node (with its children), sampled once a second
Pass (book): planner p95 < 200 ms; frontier search < 50 ms; mission loop period 0.5 s +- 0.05 s; values recorded
for the Pi 5 comparison. Here "frontier search < 50 ms" is judged on the 95th percentile and the loop period on
its 5th-95th percentile range; the maxima are reported too.
The mission itself must also complete (DONE), as in C4.
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c_common import save  # noqa: E402


def pct(v):
    a = np.asarray(v, float)
    return dict(n=int(a.size), median=round(float(np.median(a)), 1), p95=round(float(np.percentile(a, 95)), 1),
                max=round(float(a.max()), 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default='w8_arena_full')
    ap.add_argument('--runs', type=int, default=1)
    a, _ = ap.parse_known_args()
    t0 = time.time()
    lines, ok, data = [], True, []
    for i in range(a.runs):
        d = os.path.join(a.out, f'{a.world}_{i:02d}')
        env = dict(os.environ, ROS_DOMAIN_ID=str(80 + i))
        with open(d + '.log', 'w') as log:
            subprocess.run([sys.executable, os.path.join(HERE, 'c4_one.py'), '--world', a.world, '--index', str(i),
                            '--out', d, '--speed', '1.0', '--timing'], env=env, stdout=log, stderr=subprocess.STDOUT)
        r = json.load(open(os.path.join(d, 'result.json')))
        ev = [json.loads(x) for x in open(os.path.join(d, 'events.jsonl'))]
        summ = next((e for e in ev if e.get('event') == 'summary'), {})
        data.append(r)
        if r.get('final') != 'DONE':
            ok = False
            lines.append(f"FAIL run {i}: mission ended {r.get('final')} {r.get('reason', '')}")
        # planner: planning_time reported by the planner (sim clock = real time at 1x) and wall-clock round trip
        pm = summ.get('planning_ms') or {}
        pw = r.get('plan_wall_ms') or {}
        p95 = pm.get('p95')
        good = p95 is not None and p95 < 200.0
        ok &= good
        lines.append(f"{'ok  ' if good else 'FAIL'} planner time: p95 {p95} ms, max {pm.get('max')} ms over "
                     f"{pm.get('n')} paths (limit p95 < 200 ms); request->answer incl. ROS: median "
                     f"{pw.get('median')} ms, p95 {pw.get('p95')} ms, max {pw.get('max')} ms")
        fs = r.get('frontier_search_ms') or []
        if fs:
            f = pct(fs)
            good = f['p95'] < 50.0
            ok &= good
            lines.append(f"{'ok  ' if good else 'FAIL'} frontier search: median {f['median']} ms, p95 {f['p95']} ms, "
                         f"max {f['max']} ms over {f['n']} searches (limit 50 ms)")
        else:
            ok = False
            lines.append('FAIL frontier search: no /frontiers/search_ms received')
        lp = r.get('loop_period_s') or {}
        good = bool(lp) and 0.45 <= lp['p5'] and lp['p95'] <= 0.55
        ok &= good
        lines.append(f"{'ok  ' if good else 'FAIL'} mission loop period: p5 {lp.get('p5')} s, median {lp.get('median')} s, "
                     f"p95 {lp.get('p95')} s, max {lp.get('max')} s over {lp.get('n')} loops (limit 0.5 +- 0.05 s)")
        for name, c in sorted((r.get('cpu_pct_one_core') or {}).items()):
            lines.append(f"     CPU {name:16s} mean {c['mean']:5.1f} %  p95 {c['p95']:5.1f} %  max {c['max']:5.1f} % "
                         f"of one core")
        lines.append(f"     mission: {r.get('final')} in {r.get('mission_s')} s, coverage {r.get('coverage')}, "
                     f"contacts {r.get('contacts')}, home {r.get('home_m')} m")
    lines.append(f'     {a.runs} run(s) in {(time.time() - t0) / 60:.1f} min (stub at 1x: real time); '
                 'these values are the reference for the Pi 5')
    save(a.out, 'C7', {'runs': data}, ok, lines)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
