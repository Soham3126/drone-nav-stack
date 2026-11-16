#!/usr/bin/env python3
"""C6 Failure handling (guidebook 4.5): the mission brain stays sane when its helpers fail.

    scripts/run_c.sh C6 faults -- --jobs 3                   (15 missions; about 10-15 minutes)

Each fault 3 times (start poses 0, 1, 2; the gap fault: W7's own start with 3 noise seeds), each mission = c4_one.py on the stub with the fault injected:
  plan_fail          from t = 60 s a relay between the brain and the planner refuses the first goal asked for
                     (and goals within 0.6 m of it) for 60 s. Expected: that frontier is blacklisted ("no path")
                     or got round with approach points, another goal flown within 20 s, the mission completes.
  gap                world W7 (side rooms behind 0.45 / 0.60 / 0.90 m gaps): the room behind the 0.45 m gap can
                     be planned to only up to the planner's tolerance and never reached. Expected: blacklisted by
                     the progress timeout ("no progress"), the mission completes (DONE, coverage >= 98 % of the
                     space the drone can reach).
  arm_refuse         the stub refuses the arm command. Expected: STOP, reason "arm refused", land requested.
  frontier_silence   the frontier server is frozen from t = 60 s for 10 s. Expected: a "no /frontiers" warning,
                     the current goal kept, no RETURN during the silence or because of it, goals chosen again
                     after it, the mission completes (DONE).
  status_stop        the stub stops publishing /bridge/status at t = 60 s. Expected: STOP after 3 s (between
                     63 and 64.5 s), reason "no /bridge/status", land requested.
Every mission also: the mission brain process still running at the end (no crash).
Pass: each fault behaves as expected 3 of 3 times.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c_common import save  # noqa: E402

FAULT_T = 60.0
FAULTS = [  # name, world, c4_one fault
    ('plan_fail', 'w5_maze_loop', 'plan_fail'),
    ('gap', 'w7_gaps', 'none'),
    ('arm_refuse', 'w3_tree_maze', 'arm_refuse'),
    ('frontier_silence', 'w4_maze_rooms', 'frontier_silence'),
    ('status_stop', 'w3_tree_maze', 'status_stop'),
]
GAP_045_MAP = (1.5, 0.5)       # W7: the 0.45 m gap at world (2.0, 1.0), start at (0.5, 0.5) -> map frame


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def run_one(job):
    slot, name, world, fault, idx, out, speed = job
    # gap: always from W7's own start (in front of the 0.45 m gap), with 3 different sensor-noise seeds;
    # the others: 3 different start poses
    start, seed = (0, idx) if name == 'gap' else (idx, 0)
    d = os.path.join(out, f'{name}_{idx}')
    env = dict(os.environ, ROS_DOMAIN_ID=str(70 + slot))
    with open(d + '.log', 'w') as log:
        subprocess.run([sys.executable, os.path.join(HERE, 'c4_one.py'), '--world', world, '--index', str(start),
                        '--seed', str(seed),
                        '--out', d, '--speed', str(speed), '--fault', fault, '--fault_t', str(FAULT_T)],
                       env=env, stdout=log, stderr=subprocess.STDOUT)
    try:
        r = json.load(open(os.path.join(d, 'result.json')))
        ev = [json.loads(line) for line in open(os.path.join(d, 'events.jsonl'))]
    except Exception as e:  # noqa: BLE001
        return dict(name=name, index=idx, ok=False, why=[f'no result: {e!r}'])
    ok, why = judge(name, r, ev)
    if 'mission_brain' not in (r.get('alive_at_end') or []):
        ok, why = False, why + ['mission brain process died']
    return dict(name=name, index=idx, ok=ok, why=why, final=r.get('final'), reason=r.get('reason'),
                coverage=r.get('coverage'), home_m=r.get('home_m'), contacts=r.get('contacts'),
                handled_by=r.get('handled_by'))


def judge(name, r, ev):
    why = []
    states = [(e['t'], e['state']) for e in ev if e['event'] == 'state']
    final, reason = r.get('final'), r.get('reason') or ''
    landed = any(e['event'] in ('stop_land', 'command') and (e['event'] == 'stop_land' or e.get('name') == 'land')
                 for e in ev)
    if name in ('plan_fail', 'gap', 'frontier_silence'):
        if final != 'DONE':
            why.append(f'final {final} ({reason}), expected DONE')
        if (r.get('coverage') or 0) < 0.98:
            why.append(f"coverage {r.get('coverage')}")
        # (wall contacts are reported, not judged: the book's C6 does not ask for them, and the 0.60 m gap in W7
        # leaves the 0.54 m drone 3 cm a side - a documented limitation, decided 2026-11-16)
    if name == 'plan_fail':
        fp = r.get('fault_planner') or {}
        tgt = fp.get('target')
        if not tgt:
            why.append('the relay never chose a target (no goal asked for after 60 s?)')
        else:
            # book: the frontier is blacklisted and the next candidate chosen. The brain can also get round the
            # refusal with its "approach" points (flying part of the way shows the frontier, which then goes);
            # both count, as long as another goal is being flown within 20 s of the first refusal.
            t_ref = fp.get('t_target')
            bl = [e for e in ev if e['event'] == 'blacklist' and e['why'] == 'no path' and dist(e['xy'], tgt) <= 1.0]
            after = [e for e in ev if e['event'] == 'goal' and e['t'] >= t_ref and dist(e['xy'], tgt) > 1.0]
            if not after:
                why.append('no other goal chosen after the refusals')
            elif after[0]['t'] - t_ref > 20.0:
                why.append(f"next goal only {after[0]['t'] - t_ref:.0f} s after the first refusal")
            r['handled_by'] = 'blacklist' if bl else 'approach'
    if name == 'gap':
        bl = [e for e in ev if e['event'] == 'blacklist' and 'no progress' in e['why']
              and dist(e['xy'], GAP_045_MAP) <= 1.2]
        if not bl:
            why.append('the room behind the 0.45 m gap was not blacklisted by the progress timeout')
    if name == 'arm_refuse':
        if final != 'STOP' or 'arm refused' not in reason:
            why.append(f'final {final} ({reason}), expected STOP "arm refused"')
        if not landed:
            why.append('no land requested')
    if name == 'frontier_silence':
        sil = r.get('silence') or {}
        t0, t1 = sil.get('paused'), sil.get('resumed')
        if t0 is None or t1 is None:
            why.append(f'the silence was not injected ({sil})')
        else:
            warn = [e for e in ev if e['event'] == 'warning' and e.get('what') == 'no /frontiers' and t0 <= e['t'] <= t1 + 1]
            if not warn:
                why.append('no "no /frontiers" warning during the silence')
            ret = [t for t, s in states if s == 'RETURN' and t0 <= t <= t1 + 3.0]
            if ret:
                why.append(f'went to RETURN at {ret[0]} s, during/just after the silence')
            if not [e for e in ev if e['event'] == 'goal' and e['t'] > t1] and final == 'DONE' and \
                    not [t for t, s in states if s == 'RETURN' and t > t1 + 3.0]:
                why.append('no goals after the silence')
    if name == 'status_stop':
        stop = [t for t, s in states if s == 'STOP']
        if final != 'STOP' or 'no /bridge/status' not in reason:
            why.append(f'final {final} ({reason}), expected STOP "no /bridge/status"')
        elif not (FAULT_T + 3.0 <= stop[0] <= FAULT_T + 4.5):
            why.append(f'STOP at {stop[0]} s, expected 63-64.5 s')
        if not landed:
            why.append('no land requested')
    return not why, why


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--jobs', type=int, default=3)
    ap.add_argument('--speed', type=float, default=5.0)
    ap.add_argument('--runs', type=int, default=3)
    ap.add_argument('--only', nargs='*', default=None, help='fault names to run (default: all)')
    a, _ = ap.parse_known_args()
    faults = [f for f in FAULTS if not a.only or f[0] in a.only]
    jobs = [(name, world, fault, i) for i in range(a.runs) for name, world, fault in faults]
    jobs = [(k % a.jobs,) + j + (a.out, a.speed) for k, j in enumerate(jobs)]
    t0 = time.time()
    results = []
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        slots = [[j for j in jobs if j[0] == s] for s in range(a.jobs)]
        for res in ex.map(lambda js: [run_one(j) for j in js], slots):
            results += res
    lines, ok = [], True
    for name, world, _ in faults:
        rs = sorted((r for r in results if r['name'] == name), key=lambda r: r['index'])
        n = sum(r['ok'] for r in rs)
        good = n == len(rs) == a.runs
        ok &= good
        lines.append(f"{'ok  ' if good else 'FAIL'} {name:17s} {n}/{a.runs} as expected ({world})")
        for r in rs:
            if not r['ok']:
                lines.append(f"       start {r['index']}: {'; '.join(r['why'])}")
        notes = [f"run {r['index']}: refusal handled by {r['handled_by']}" for r in rs if r.get('handled_by')]
        notes += [f"run {r['index']}: {r['contacts']} wall contact(s) (not judged)" for r in rs if r.get('contacts')]
        if notes:
            lines.append('       ' + '; '.join(notes))
    lines.append(f'     {len(results)} missions in {(time.time() - t0) / 60:.1f} min')
    save(a.out, 'C6', {'runs': results}, ok, lines)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
