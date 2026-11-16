#!/usr/bin/env python3
"""I2-I6 Full missions in Gazebo (guidebook 4.6): the whole stack - Gazebo, ArduPilot SITL, Cartographer, bridge,
planner server, frontier server, mission brain (profile "full") - explores, returns and lands on its own.

One mission per run:   scripts/run_test.sh I2 w3_tree_maze full 1800
Several runs + table:  scripts/run_i.sh I2 w3_tree_maze 10
    I2 W3 x10 | I3 W4 x10 | I4 W6 (3 widths), W7, W5 x5 each | I5 W8 x10 (flow as decided in B7) + 5 the other way
    I6 random mazes wr_s101-130 x1
Procedure: wait until the bridge reports the EKF on our pose (+20 s for ArduPilot's own checks), call
/mission/start, wait for DONE or STOP (or the time limit), evaluate against ground truth (i_common.py).
Pass per run (book): DONE; coverage >= 95 %; landing within 0.3 m of the start; no contact; ATE < 0.15 m;
frame offset < 0.10 m. W7 (I4): the room behind the 0.45 m gap never entered.
The same script serves I3-I6 (they differ only in worlds and counts); I3.py ... I6.py import it.
"""
import argparse
import glob
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

LIMIT_S = {'w8_arena_full': 1500.0}          # mission time limit (wall s; Gazebo runs about real time)


def gap045_entered(truth, meta):
    """W7: the side room behind the 0.45 m door (truth path, world frame)."""
    doors = [d for d in meta.get('doors', []) if abs(d['width'] - 0.45) < 0.01]
    if not doors:
        return None
    d = doors[0]
    return any(abs(p[1] - d['x']) <= 0.5 and p[2] > d['y'] + 0.10 for p in truth)


def run(test_id, out, world):
    import rclpy
    from b_common import finish, save
    from i_common import MissionWatch, spin_in_thread
    rclpy.init()
    w = MissionWatch(world)
    spin_in_thread(w)
    res = dict(world=world)
    lines = [f'{test_id} full mission on {world}']
    if abs(w.meta['start'].get('yaw', 0.0)) > 1e-6:
        lines.append('WARNING: start yaw is not 0: the map-to-world shift used here assumes it is')
    if not w.wait_ready():
        lines.append('FAIL the bridge never reported the EKF on our pose')
        save(out, test_id, res, False, lines)
        finish(1)
    ok, msg = w.start_mission()
    lines.append(f'/mission/start: {msg}')
    t0 = time.time()
    ended = ok and w.wait_end(LIMIT_S.get(world, 1200.0))
    mission_s = round(time.time() - t0, 1)
    summ = next((e for e in w.events if e.get('event') == 'summary'), {})
    with w.lock:
        truth = list(w.truth)
    res.update(final=w.state or 'none', reason=summ.get('reason', ''), mission_wall_s=mission_s,
               coverage=w.coverage(), contacts=w.contacts, min_clearance_m=round(w.min_clear, 3),
               contact_log=w.contact_log, landing_m=w.landing(), ate_m=w.ate(),
               frame_offset_max_m=round(w.offset_max, 3),
               goal_switches=len(summ.get('goal_switches', [])), blacklisted=len(summ.get('blacklisted', [])),
               planning_ms=summ.get('planning_ms'))
    if world == 'w7_gaps':
        res['gap045_entered'] = gap045_entered(truth, w.meta)
    checks = [
        ('mission ended DONE', res['final'] == 'DONE' and ended, f"{res['final']} {res['reason']}"),
        ('coverage >= 95 %', (res['coverage'] or 0) >= 0.95, res['coverage']),
        ('landing within 0.3 m of the start', res['landing_m'] is not None and res['landing_m'] <= 0.3,
         f"{res['landing_m']} m"),
        ('no contact', res['contacts'] == 0,
         f"{res['contacts']} (closest {res['min_clearance_m'] * 100:.1f} cm)"),
        ('ATE < 0.15 m', res['ate_m'] is not None and res['ate_m'] < 0.15, f"{res['ate_m']} m"),
        ('frame offset < 0.10 m', res['frame_offset_max_m'] < 0.10, f"{res['frame_offset_max_m']} m"),
    ]
    if 'gap045_entered' in res:
        checks.append(('0.45 m gap never entered', res['gap045_entered'] is False, res['gap045_entered']))
    good = True
    for name, ok_, val in checks:
        good &= bool(ok_)
        lines.append(f"{'ok  ' if ok_ else 'FAIL'} {name:34s} {val}")
    lines.append(f"     mission {mission_s} s wall, {res['goal_switches']} goals, {res['blacklisted']} blacklisted")
    for c in w.contact_log[:3]:
        lines.append(f"     contact at world {c['at']} depth {c['depth_m']} m")
    save(out, test_id, res, good, lines)
    finish(0 if good else 1)


def summary(test_id, world, n):
    rows = []
    for d in sorted(glob.glob(os.path.expanduser(f'~/drone_results/*_{test_id}_{world}_*')), reverse=True)[:n]:
        f = os.path.join(d, 'result.json')
        if os.path.exists(f):
            rows.append(json.load(open(f)))
    if not rows:
        print(f'no results for {test_id} {world}')
        return 1
    k = sum(1 for r in rows if r.get('pass'))
    print(f"{test_id} {world}: {k}/{len(rows)} runs pass (newest {n})")
    for r in rows:
        print(f"  {'ok  ' if r.get('pass') else 'FAIL'} {r.get('final')} coverage {r.get('coverage')} "
              f"landing {r.get('landing_m')} m contacts {r.get('contacts')} ATE {r.get('ate_m')} m offset "
              f"{r.get('frame_offset_max_m')} m time {r.get('mission_wall_s')} s {r.get('reason', '')}")
    return 0 if k == len(rows) else 1


def main(test_id='I2'):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out')
    ap.add_argument('--world', default='w3_tree_maze')
    ap.add_argument('--summary', type=int, default=0, help='table of the newest N runs of this test and world')
    a = ap.parse_args()
    if a.summary:
        sys.exit(summary(test_id, a.world, a.summary))
    run(test_id, os.path.expanduser(a.out), a.world)


if __name__ == '__main__':
    main('I2')
