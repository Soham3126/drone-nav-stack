#!/usr/bin/env python3
"""I1 SLAM pose in the loop (guidebook 4.6) - W2, slam profile, carto-v1, B5's chosen speed and lookahead.

    scripts/run_test.sh I1 w2_turns_junctions slam 900          (about 6 minutes)

The bridge flies on Cartographer's pose instead of ground truth. Procedure: take off to 1.1 m, hover 60 s, then
fly B5's four paths (straight, L-turn, T-junction, U-turn), land. Measured from Gazebo's ground truth:
  hover drift            largest horizontal distance from the position at the start of the hover
  lateral error          largest distance from the path away from corners (B5's definition)
  clearance margin       smallest (distance to the nearest wall face - 0.27 m drone radius)
Pass (book): hover drift < 0.08 m; max lateral error <= 0.15 m; clearance margin >= 0.05 m.
Expected: slightly worse than with ground truth (B3/B5).
"""
import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

SPEED, LOOK, HOVER_S = 0.8, 0.6, 60.0


def run(out, world):
    from B5 import PATHS, evaluate
    from b_common import Client, finish, load_meta, save, set_param, wait_for
    import subprocess
    rects = load_meta(world)['walls']
    res = dict(speed=SPEED, lookahead=LOOK, paths={})
    lines = [f'I1 on Cartographer pose: speed {SPEED} m/s, lookahead {LOOK} m']
    ok_all = True
    with Client() as cl:
        wait_for(lambda: cl.last() is not None, 120)
        r = subprocess.run(['bash', '-c', 'source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; '
                            f'ros2 param set /bridge lookahead_m {LOOK}'], capture_output=True, text=True, timeout=60)
        lines.append(f'lookahead set: {r.stdout.strip() or r.stderr.strip()}')
        lines.append(f"WP_SPD read back: {set_param('WP_SPD', SPEED)}")
        ok, msg = cl.take_off(1.1)
        lines.append(msg)
        if not ok:
            cl.command('land')
            save(out, 'I1', res, False, lines)
            finish(1)
        time.sleep(5)                                   # settle after the climb
        t0 = time.time()
        time.sleep(HOVER_S)
        tr = cl.truth_between(t0, time.time())
        x0, y0 = tr[0][1], tr[0][2]
        drift = max(math.hypot(p[1] - x0, p[2] - y0) for p in tr)
        offs = [s.frame_offset_m for t, s in list(cl.status) if t >= t0]
        res.update(hover_drift_m=round(drift, 4), hover_offset_max_m=round(max(offs), 4) if offs else None)
        good = drift < 0.08
        ok_all &= good
        lines.append(f"{'ok  ' if good else 'FAIL'} hover {HOVER_S:.0f} s: drift {drift*100:.1f} cm (limit 8); "
                     f"bridge frame offset up to {res['hover_offset_max_m']} m")
        for name, pts in PATHS:
            length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
            arrived, secs, a0, a1 = cl.follow(pts, timeout=length / SPEED * 3 + 30)
            tr = cl.truth_between(a0, a1)
            end_xy = (tr[-1][1], tr[-1][2]) if tr else None
            m = evaluate(tr, pts, LOOK, rects, end_xy)
            m.update(arrived=arrived, time_s=round(secs, 1), length_m=round(length, 1))
            good = (arrived and m['lateral_straight_max'] <= 0.15 and m['clearance_min'] is not None
                    and m['clearance_min'] >= 0.05)
            m['pass'] = good
            res['paths'][name] = m
            ok_all &= good
            lines.append(f"{'ok  ' if good else 'FAIL'} {name:11s} {length:4.1f} m in {secs:5.1f} s: lateral "
                         f"{m['lateral_straight_max']*100:4.1f} cm straight / {m['lateral_corner_max']*100:4.1f} cm "
                         f"corners (limit 15 cm), clearance "
                         f"{m['clearance_min']*100 if m['clearance_min'] is not None else float('nan'):4.1f} cm "
                         f"(limit 5), arrival "
                         f"{m['arrival_error']*100 if m['arrival_error'] is not None else float('nan'):4.1f} cm"
                         f"{'' if arrived else ' (NOT ARRIVED)'}")
            if not arrived:
                break
            time.sleep(2)
        okl, ml, _ = cl.command('land')
        lines.append(f'land: {ml}')
    save(out, 'I1', res, ok_all and okl, lines)
    finish(0 if ok_all and okl else 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default='w2_turns_junctions')
    a = ap.parse_args()
    run(os.path.expanduser(a.out), a.world)


if __name__ == '__main__':
    main()
