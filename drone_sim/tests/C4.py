#!/usr/bin/env python3
"""C4 Mission brain on the stub (guidebook 4.5).

    scripts/run_c.sh C4 stub -- --runs 10 --jobs 3          (70 missions; about 30-45 minutes)
    scripts/run_c.sh C4 stub -- --runs 1                    (one mission per world, about 5 minutes)

Worlds W3, W4, W5, W6 (1.0 / 1.3 / 1.5 m) and W8; --runs missions each from different start poses (index 0 = the
world's own start, then seeded lattice cells). Each mission = c4_one.py: stub world (speed x5) + planner server +
frontier server + mission brain, in its own ROS domain so --jobs missions can run side by side.
Every mission must (book):
  reach DONE                            within 20 min of simulated time
  coverage >= 98 %                      of the free space the drone can reach, known in the map at the end
  end within 0.3 m of its start         (the RETURN state)
  no contacts                           drone's 0.27 m circle against the true walls (2 cm tolerance: map cells)
  goal changes <= 1 per 4 s on average, and no A -> B -> A switch within 10 s
Pass: 10 of 10 missions per world.
"""
import argparse
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c_common import save, stats  # noqa: E402

WORLDS = ['w3_tree_maze', 'w4_maze_rooms', 'w5_maze_loop', 'w6_width_100', 'w6_width_130', 'w6_width_150',
          'w8_arena_full']


def one(job):
    slot, world, idx, out, speed, limit = job
    d = os.path.join(out, f'{world}_{idx:02d}')
    env = dict(os.environ, ROS_DOMAIN_ID=str(60 + slot))
    with open(os.path.join(out, f'{world}_{idx:02d}.log'), 'w') as log:
        subprocess.run([sys.executable, os.path.join(HERE, 'c4_one.py'), '--world', world, '--index', str(idx),
                        '--out', d, '--speed', str(speed), '--limit', str(limit)], env=env, stdout=log,
                       stderr=subprocess.STDOUT)
    try:
        return json.load(open(os.path.join(d, 'result.json')))
    except Exception as e:  # noqa: BLE001
        return dict(world=world, index=idx, final='crashed', error=repr(e), **{'pass': False})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--runs', type=int, default=10)
    ap.add_argument('--jobs', type=int, default=3)
    ap.add_argument('--speed', type=float, default=5.0)
    ap.add_argument('--limit', type=float, default=1200.0)
    ap.add_argument('--worlds', nargs='*', default=WORLDS)
    a, _ = ap.parse_known_args()
    jobs = [(0, w, i, a.out, a.speed, a.limit) for i in range(a.runs) for w in a.worlds]
    jobs = [(k % a.jobs,) + j[1:] for k, j in enumerate(jobs)]
    t0 = time.time()
    results = []
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        # each slot keeps its own ROS domain: run the jobs of one slot one after another
        slots = [[j for j in jobs if j[0] == s] for s in range(a.jobs)]
        for res in ex.map(lambda js: [one(j) for j in js], slots):
            results += res
    lines, by = [], {}
    for r in sorted(results, key=lambda r: (r['world'], r['index'])):
        by.setdefault(r['world'], []).append(r)
        if not r.get('pass'):
            lines.append(f"FAIL {r['world']} start {r['index']}: {r.get('final')} {r.get('reason', '')} "
                         f"coverage {r.get('coverage')} contacts {r.get('contacts')} home {r.get('home_m')} m "
                         f"switches/4s {r.get('switches_per_4s')} ABA {r.get('aba')} {r.get('error', '')}")
            for c in (r.get('contact_log') or [])[:3]:
                lines.append(f"       contact t {c['t']} s at {c['at']} depth {c['depth_m']} m, wall in map: "
                             f"{c['wall_in_map']}, following a path: {c['following']}")
    ok = True
    for w in a.worlds:
        rs = by.get(w, [])
        n = sum(r.get('pass', False) for r in rs)
        good = n == a.runs
        ok &= good
        ms = [r['mission_s'] for r in rs if r.get('mission_s')]
        lines.append(f"{'ok  ' if good else 'FAIL'} {w:16s} {n}/{a.runs} missions ok; mission time median "
                     f"{stats(ms).get('median')} s, longest {max(ms) if ms else None} s (limit {a.limit:.0f}); coverage min {min((r.get('coverage') or 0) for r in rs) if rs else '-'}; "
                     f"contacts {sum((r.get('contacts') or 0) for r in rs)}; home max "
                     f"{max((r.get('home_m') or 9) for r in rs) if rs else '-'} m")
    lines.append(f'     {len(results)} missions in {(time.time() - t0) / 60:.1f} min ({a.jobs} at a time, '
                 f'stub speed x{a.speed})')
    save(a.out, 'C4', {'runs': results}, ok, lines)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
