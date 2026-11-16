#!/usr/bin/env python3
"""C5 Monte Carlo campaign (guidebook 4.5).

    scripts/run_c.sh C5 random -- --jobs 3                    (100 mazes; about 30-40 minutes)
    scripts/run_c.sh C5 random -- --seeds 1-10 --jobs 3       (a quick look)

Random mazes wr_s1 ... wr_s100 (drone_sim/tools/make_maze.py --random: 8-15 m, corridors 1.0-1.5 m, rooms 0-6,
loops 0-2), each flown once from its own start by c4_one.py (stub world x5 + planner + frontier server + mission
brain, one ROS domain per parallel job). Mazes that do not exist yet are generated first (same seed, same maze).
Each mission is judged as in C4 (DONE, coverage >= 98 %, home within 0.3 m, no contacts, calm goal switching), and
every failure is put in a class by what went wrong:
  stopped     the brain ended in STOP (its reason says why)
  timeout     no DONE within the time limit
  coverage    DONE with less than 98 % explored
  home        DONE more than 0.3 m from the start
  contact     the drone touched a wall
  switching   more than 1 goal change per 4 s, or an A -> B -> A switch
  crashed     the mission produced no result (a process failed)
Pass (book): at least 97 of 100 complete; every failure classified (here) and then explained or fixed (by hand,
in the C guide). Also reported: exploration time per square metre of reachable floor, the baseline for later
improvements.
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from C4 import one  # noqa: E402
from concurrent.futures import ThreadPoolExecutor  # noqa: E402
from c_common import save  # noqa: E402

PKG = os.path.dirname(HERE)
RES = 0.05


def seeds(text):
    out = []
    for part in text.split(','):
        a, _, b = part.partition('-')
        out += list(range(int(a), int(b or a) + 1))
    return out


def ensure_worlds(ss):
    missing = [s for s in ss if not os.path.exists(os.path.join(PKG, 'maps', f'wr_s{s}', 'meta.json'))]
    if missing:
        spec = ','.join(str(s) for s in missing)
        print(f'generating {len(missing)} random mazes ...', flush=True)
        subprocess.run([sys.executable, os.path.join(PKG, 'tools', 'make_maze.py'), '--random', '--seeds', spec],
                       check=True, stdout=subprocess.DEVNULL)


def classify(r):
    why = []
    if r.get('final') in (None, 'crashed', 'none') or 'error' in r:
        return ['crashed']
    if r['final'] == 'STOP':
        why.append('stopped')
    elif r['final'] != 'DONE':
        why.append('timeout')
    if (r.get('coverage') or 0) < 0.98 and r['final'] == 'DONE':
        why.append('coverage')
    if r['final'] == 'DONE' and (r.get('home_m') is None or r['home_m'] > 0.3):
        why.append('home')
    if r.get('contacts'):
        why.append('contact')
    if (r.get('switches_per_4s') or 0) > 1.0 or r.get('aba'):
        why.append('switching')
    return why


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seeds', default='1-100')
    ap.add_argument('--jobs', type=int, default=3)
    ap.add_argument('--speed', type=float, default=5.0)
    ap.add_argument('--limit', type=float, default=1200.0)
    a, _ = ap.parse_known_args()
    ss = seeds(a.seeds)
    ensure_worlds(ss)
    jobs = [(k % a.jobs, f'wr_s{s}', 0, a.out, a.speed, a.limit) for k, s in enumerate(ss)]
    t0 = time.time()
    results = []
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        slots = [[j for j in jobs if j[0] == s] for s in range(a.jobs)]
        for res in ex.map(lambda js: [one(j) for j in js], slots):
            results += res
    results.sort(key=lambda r: int(r['world'][4:]))
    lines, classes, per_m2 = [], {}, []
    for r in results:
        meta = json.load(open(os.path.join(PKG, 'maps', r['world'], 'meta.json')))
        area = meta['cells']['reachable_drone'] * RES * RES
        r['area_m2'] = round(area, 1)
        r['classes'] = classify(r) if not r.get('pass') else []
        if r.get('pass') and r.get('explore_s'):
            per_m2.append(r['explore_s'] / area)
        for c in r['classes']:
            classes.setdefault(c, []).append(r['world'])
        if r['classes']:
            lines.append(f"FAIL {r['world']:8s} [{', '.join(r['classes'])}] {r.get('final')} {r.get('reason', '')} "
                         f"coverage {r.get('coverage')} contacts {r.get('contacts')} home {r.get('home_m')} m "
                         f"switches/4s {r.get('switches_per_4s')} ABA {r.get('aba')} mission {r.get('mission_s')} s "
                         f"{r.get('error', '')}")
    n_ok = sum(1 for r in results if r.get('pass'))
    need = len(results) - len(results) // 33          # 97 of 100
    ok = n_ok >= need
    ms = [r['mission_s'] for r in results if r.get('pass') and r.get('mission_s')]
    lines.append(f"{'ok  ' if ok else 'FAIL'} {n_ok}/{len(results)} mazes completed (need {need})")
    for c, ws in sorted(classes.items()):
        lines.append(f"     {c:10s} {len(ws)}: {' '.join(ws)}")
    if per_m2:
        lines.append(f"     exploration time per m2 of reachable floor: median {statistics.median(per_m2):.2f} s/m2, "
                     f"p90 {sorted(per_m2)[int(0.9 * (len(per_m2) - 1))]:.2f} s/m2 (baseline for frontier scoring)")
    if ms:
        lines.append(f"     mission time: median {statistics.median(ms):.0f} s, longest {max(ms):.0f} s "
                     f"(limit {a.limit:.0f})")
    lines.append(f'     {len(results)} missions in {(time.time() - t0) / 60:.1f} min ({a.jobs} at a time, '
                 f'stub speed x{a.speed})')
    save(a.out, 'C5', {'runs': results}, ok, lines)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
