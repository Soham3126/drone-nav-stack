#!/usr/bin/env python3
"""A1.7 Near-ground phantom walls: W1 or W2, gps, one take-off style per run (env STYLE):
  a  brisk vertical: default climb rate to 1.1 m, hover 8 s, land            (pass: no phantom cluster > 0.2 m)
  b  slow vertical : WP_SPD_UP 0.1 m/s: ~6 s below 0.6 m, so the 5 s timer inserts scans low  (recorded)
  c  disturbed     : take off to 0.5 m, move 0.15 m sideways and back, then climb to 1.1 m (recorded)
               (a 0.3 m step hit the wall: a 1 m corridor leaves the 0.27 m-radius drone only ~0.2 m each side)
eval_run compares maps before lift-off, after passing 0.6 m, before descending through 0.6 m and after
landing, and counts phantom cells added below 0.6 m.
    for st in a b c; do for r in 1 2 3; do STYLE=$st scripts/run_test.sh A1.7 w1_corridor_plain gps; done; done"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'tools'))
from geom import load_meta  # noqa: E402
from scripted_flight import Flyer, FlightError  # noqa: E402
from a1_common import append_meta  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--world', default='w1_corridor_plain')
args, _ = ap.parse_known_args()
style = os.environ.get('STYLE', 'a').lower()
if style not in 'abc' or len(style) != 1:
    sys.exit('STYLE must be a, b or c')
if args.world not in ('w1_corridor_plain', 'w2_turns_junctions'):
    sys.exit('A1.7 is defined for W1 and W2')
append_meta(args.out, style=style, speed=0.5, yaw_mode='fixed', laps=0)
start = load_meta(args.world)['start']
yaw_ned = math.pi / 2 - start['yaw']
res = {'style': style, 'ok': False}
f = Flyer()
print(f'A1.7 style {style} in {args.world}')
try:
    f.connect()
    f.set_param('WP_SPD', 0.5)
    if style == 'b':
        f.set_param('WP_SPD_UP', 0.1)
    f.wait_position()
    f.pump(1.0)
    _, n0, e0, d0, *_ = f.lpos
    f.set_mode('GUIDED')
    f.arm()
    if style == 'c':
        f.takeoff(0.5, d0)
        # "sideways" = 90 deg left of the heading; heading east (yaw 0) -> north
        dn, de = math.cos(start['yaw']) * 0.15, -math.sin(start['yaw']) * 0.15
        f.goto(n0 + dn, e0 + de, d0 - 0.5, yaw_ned, 0.08, 20)
        print(f'  sidestep 0.15 m at {d0 - f.lpos[3]:.2f} m height')
        f.goto(n0, e0, d0 - 0.5, yaw_ned, 0.08, 20)
        print(f'  back over the start at {d0 - f.lpos[3]:.2f} m height')
        f.goto(n0, e0, d0 - 1.1, yaw_ned, 0.15, 30)
        print(f'  climbed to {d0 - f.lpos[3]:.2f} m')
    else:
        t0 = time.time()
        f.takeoff(1.1, d0, timeout=90)
        print(f'  climbed to 1.1 m in {time.time() - t0:.1f} s')
    t_end = time.time() + 8
    while time.time() < t_end:
        f.goto(n0, e0, d0 - 1.1, yaw_ned, 10.0, 1)     # hold position
        f.pump(0.2)
    print('  hovered 8 s; landing')
    f.land()
    res['ok'] = True
except FlightError as e:
    res['error'] = str(e)
    print('FLIGHT ERROR:', e)
    try:
        if f.armed:
            f.land()
    except Exception:  # noqa: BLE001
        pass
time.sleep(3)
json.dump(res, open(os.path.join(args.out, 'flight_result.json'), 'w'), indent=1)
print(f"flight {'completed' if res['ok'] else 'FAILED'}; evaluation follows after the simulation stops")
sys.exit(0 if res['ok'] else 1)
