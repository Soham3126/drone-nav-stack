#!/usr/bin/env python3
"""B5 Path following with the carrot (guidebook 4.4) - W2, truth profile.

One (speed, lookahead) combination per run, from the environment:
    SPEED=0.8 LOOK=0.6 scripts/run_test.sh B5 w2_turns_junctions truth 900
All 9 combinations x 3 repeats:  scripts/batch.sh b5 "scripts/run_b5.sh"

Paths (map frame = drone start at world (0.5, 3.5), heading +x), flown one after another, each ending
with a hold until the bridge reports "arrived":
  straight    8 m straight along the start corridor
  l_turn      on to the east L-turn and up its arm
  t_junction  back, left at the crossroads, up to the T-junction and right through it
  u_turn      back to the crossroads, into the south arm, 180 deg turn, back through the crossroads home
Measured from ground truth on each path:
  lateral_straight_max  largest distance from the path away from corners (more than L + 0.3 m from one)
  lateral_corner_max    the same near corners (the carrot cuts corners by up to 0.354 L by design)
  clearance_min         smallest (distance to the nearest wall face - drone radius 0.27 m)
  arrival_error         distance from the path's end when the bridge reports arrival
  time_s                path time
Pass (book): lateral error <= 0.12 m (taken on straights; corners are judged by clearance),
clearance margin >= 0.08 m, arrival within 0.15 m - on every path.
Summary by combination:  python3 drone_sim/tests/B5.py --summary
"""
import argparse
import glob
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

START_W = (0.5, 3.5)
PATHS = [
    ('straight', [(0, 0), (8, 0)]),
    ('l_turn', [(8, 0), (10, 0), (10, 3)]),
    ('t_junction', [(10, 3), (10, 0), (5, 0), (5, 5), (8, 5)]),
    ('u_turn', [(8, 5), (5, 5), (5, 0), (5, -2.5), (5, 0), (0, 0)]),
]
ROBOT_R = 0.27


def to_world(pts):
    return [(x + START_W[0], y + START_W[1]) for x, y in pts]


def evaluate(tr, path_map, look, rects, end_xy):
    from b_common import dist_to_polyline, dist_to_rects, near_vertex
    pw = to_world(path_map)
    lat_s, lat_c, clr = [0.0], [0.0], []
    for _, x, y, _ in tr:
        d, _ = dist_to_polyline(x, y, pw)
        (lat_c if near_vertex(x, y, pw, look + 0.3) else lat_s).append(d)
        clr.append(dist_to_rects(x, y, rects) - ROBOT_R)
    ex, ey = pw[-1]
    return dict(lateral_straight_max=round(max(lat_s), 3), lateral_corner_max=round(max(lat_c), 3),
                clearance_min=round(min(clr), 3) if clr else None,
                arrival_error=round(((end_xy[0] - ex) ** 2 + (end_xy[1] - ey) ** 2) ** 0.5, 3) if end_xy else None)


def run(out, world):
    from b_common import Client, load_meta, save, set_param
    speed = float(os.environ.get('SPEED', '0.5'))
    look = float(os.environ.get('LOOK', '0.5'))
    maxcut = float(os.environ.get('MAXCUT', '0'))        # I2: the bend limit the full mission profile uses (0.05)
    rects = load_meta(world)['walls']
    res = dict(speed=speed, lookahead=look, max_cut=maxcut, paths={})
    lines = [f'B5 speed {speed} m/s, lookahead {look} m, bend limit {maxcut} m']
    import subprocess
    with Client() as cl:
        from b_common import wait_for
        wait_for(lambda: cl.last() is not None, 120)          # the bridge is up
        for name, val in (('lookahead_m', look), ('follow_max_cut_m', maxcut)):
            # retried: right after start-up the bridge can answer "undeclared parameter"
            for _ in range(10):
                r = subprocess.run(['bash', '-c', 'source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/'
                                    f'setup.bash; ros2 param set /bridge {name} {val}'],
                                   capture_output=True, text=True, timeout=60)
                if 'successful' in r.stdout.lower():
                    break
                time.sleep(3)
            lines.append(f'{name} set: {r.stdout.strip() or r.stderr.strip()}')
        wsp = set_param('WP_SPD', speed)
        lines.append(f'WP_SPD read back: {wsp}')
        ok, msg = cl.take_off(1.1)
        lines.append(msg)
        if not ok:
            cl.command('land')
            save(out, 'B5', res, False, lines)
            return 1
        time.sleep(3)
        all_ok = True
        for name, pts in PATHS:
            length = sum(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5 for a, b in zip(pts, pts[1:]))
            arrived, secs, t0, t1 = cl.follow(pts, timeout=length / speed * 3 + 30)
            tr = cl.truth_between(t0, t1)
            end_xy = (tr[-1][1], tr[-1][2]) if tr else None
            m = evaluate(tr, pts, look, rects, end_xy)
            m.update(arrived=arrived, time_s=round(secs, 1), length_m=round(length, 1))
            good = (arrived and m['lateral_straight_max'] <= 0.12 and m['clearance_min'] is not None
                    and m['clearance_min'] >= 0.08 and m['arrival_error'] is not None and m['arrival_error'] <= 0.15)
            m['pass'] = good
            res['paths'][name] = m
            all_ok &= good
            lines.append(f"{'ok  ' if good else 'FAIL'} {name:11s} {length:4.1f} m in {secs:5.1f} s: lateral "
                         f"{m['lateral_straight_max']*100:4.1f} cm straight / {m['lateral_corner_max']*100:4.1f} cm corners, "
                         f"clearance {m['clearance_min']*100 if m['clearance_min'] is not None else float('nan'):4.1f} cm, "
                         f"arrival {m['arrival_error']*100 if m['arrival_error'] is not None else float('nan'):4.1f} cm"
                         f"{'' if arrived else ' (NOT ARRIVED)'}")
            if not arrived:
                break
            time.sleep(2)                             # hold at the end
        okl, ml, _ = cl.command('land')
        lines.append(f'land: {ml}')
    save(out, 'B5', res, all_ok and okl, lines)
    return 0 if all_ok and okl else 1


def summary(root):
    by = {}
    for f in sorted(glob.glob(os.path.join(os.path.expanduser(root), '*_B5_*', 'result.json'))):
        r = json.load(open(f))
        if 'speed' in r and r.get('paths'):
            by.setdefault((r['speed'], r['lookahead'], r.get('max_cut', 0.0)), []).append(r)
    print(f"{'speed':>5s} {'L':>4s} {'cut':>4s} {'runs':>4s} {'pass':>4s} {'lat straight':>12s} {'lat corner':>10s} "
          f"{'clearance':>9s} {'arrival':>7s} {'time s':>6s}   (worst over runs and paths; time = mean total)")
    for (sp, L, mc), rs in sorted(by.items()):
        ps = [p for r in rs for p in r['paths'].values()]
        npass = sum(r['pass'] for r in rs)
        print(f"{sp:5.1f} {L:4.1f} {mc:4.2f} {len(rs):4d} {npass:4d} {max(p['lateral_straight_max'] for p in ps)*100:10.1f}cm "
              f"{max(p['lateral_corner_max'] for p in ps)*100:8.1f}cm "
              f"{min(p['clearance_min'] for p in ps if p['clearance_min'] is not None)*100:7.1f}cm "
              f"{max(p['arrival_error'] for p in ps if p['arrival_error'] is not None)*100:5.1f}cm "
              f"{sum(sum(p['time_s'] for p in r['paths'].values()) for r in rs) / len(rs):6.0f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out')
    ap.add_argument('--world', default='w2_turns_junctions')
    ap.add_argument('--summary', action='store_true')
    ap.add_argument('--root', default='~/drone_results')
    a = ap.parse_args()
    if a.summary:
        summary(a.root)
        return 0
    return run(a.out, a.world)


if __name__ == '__main__':
    code = main()
    from b_common import finish
    finish(code)
