#!/usr/bin/env python3
"""Test A0.4 - scripted flight tool (guidebook 4.3).

One run = one world's pattern flown once, with ground truth recorded:
  scripts/run_test.sh A0.4 w0_box_room gps
The book asks for W0, W1, W2, W3, each twice. All eight runs in one go (about 45 min):
  for w in w0_box_room w1_corridor_plain w2_turns_junctions w3_tree_maze; do
    for r in 1 2; do scripts/run_test.sh A0.4 $w gps 1200; done; done
  python3 drone_sim/tests/A0_4.py --summary ~/drone_results

Checks per run: every waypoint reached, then landed; ground-truth clearance margin
(distance from the drone centre to the nearest wall/box face minus the 0.27 m radius)
stays > 0.05 m while airborne. Pass for A0.4 = 2 passing runs in each of the four worlds.
"""
import argparse
import glob
import json
import os
import sys
import time

import numpy as np
import yaml

from sim_common import Collector, wait_for, world_offset  # noqa: F401
from geom import ROBOT_R, clearance_at, load_meta, obstacle_rects  # noqa: E402
from scripted_flight import default_pattern_path, fly_pattern  # noqa: E402

WORLDS = ['w0_box_room', 'w1_corridor_plain', 'w2_turns_junctions', 'w3_tree_maze']


def one_run(args):
    os.makedirs(args.out, exist_ok=True)
    meta = load_meta(args.world)
    rects = obstacle_rects(meta, flight=True)
    path = args.pattern or default_pattern_path(args.world)
    pattern = yaml.safe_load(open(path))
    with Collector() as node:
        if not wait_for(lambda: len(node.odom) > 10, 30):
            print('FAIL: no /ground_truth/odom')
            return 1
        res = fly_pattern(pattern, url=args.url, speed=args.speed)
        time.sleep(1)
        with node.lock:
            odom = list(node.odom)
    off, frame_kind = world_offset(odom, meta['start'])
    O = np.array(odom)
    x, y, z = O[:, 2] + off[0], O[:, 3] + off[1], O[:, 4]
    air = z > z[:20].mean() + 0.3
    margins = np.array([clearance_at(px, py, rects) - ROBOT_R for px, py in zip(x[air], y[air])])
    worst = int(np.argmin(margins)) if len(margins) else None
    report = {
        'world': args.world, 'pattern': pattern['pattern'], 'odom_frame': frame_kind,
        'flight': {k: v for k, v in res.items() if k != 'statustext'},
        'airborne_samples': int(air.sum()),
        'clearance_margin_min_m': round(float(margins.min()), 3) if len(margins) else None,
        'clearance_margin_where': [round(float(x[air][worst]), 2), round(float(y[air][worst]), 2)] if worst is not None else None,
        'clearance_margin_p5_m': round(float(np.percentile(margins, 5)), 3) if len(margins) else None,
    }
    report['pass'] = bool(res['ok'] and len(margins) and margins.min() > 0.05)
    np.savez_compressed(os.path.join(args.out, 'A0_4_data.npz'), odom=O)
    json.dump(report, open(os.path.join(args.out, 'A0_4_report.json'), 'w'), indent=1)
    print(json.dumps(report, indent=1))
    print(f"\nA0.4 run ({args.world}): {'PASS' if report['pass'] else 'FAIL'}")
    return 0 if report['pass'] else 1


def summary(root):
    rows = {}
    for p in sorted(glob.glob(os.path.join(os.path.expanduser(root), '*A0.4*', 'A0_4_report.json'))):
        r = json.load(open(p))
        rows.setdefault(r['world'], []).append((os.path.basename(os.path.dirname(p)), r))
    ok_all = True
    print(f"{'world':20s} {'runs':>4s} {'pass':>4s}  min clearance margins (m)")
    for w in WORLDS:
        runs = rows.get(w, [])
        n_pass = sum(1 for _, r in runs if r['pass'])
        margins = ', '.join(f"{r['clearance_margin_min_m']}" for _, r in runs)
        print(f"{w:20s} {len(runs):4d} {n_pass:4d}  {margins}")
        ok_all &= n_pass >= 2
    print(f"\nA0.4: {'PASS' if ok_all else 'NOT YET (need 2 passing runs per world)'}")
    return 0 if ok_all else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='w0_box_room')
    ap.add_argument('--pattern')
    ap.add_argument('--speed', type=float, default=0.5)
    ap.add_argument('--out', default=os.path.expanduser('~/drone_results/A0_4_manual'))
    ap.add_argument('--url', default='tcp:127.0.0.1:5763')
    ap.add_argument('--summary', metavar='RESULTS_DIR')
    args, _ = ap.parse_known_args()
    if args.summary:
        return summary(args.summary)
    return one_run(args)


if __name__ == '__main__':
    sys.exit(main())
