#!/usr/bin/env python3
"""nav_check.py - is the map right AROUND THE DRONE? (addition to guidebook A1.5/A1.8)

The drone plans and steers with its own pose estimate inside its own map. If map and pose drift
together, it is still safe; what matters is LOCAL consistency. Every 0.5 s of the flight:

  wall_missing   true walls within 0.8 m of the true drone, moved into the map with the drone's
                 ESTIMATED pose: how far is each from the nearest mapped wall? A large value = a real
                 wall the drone's map does not show where the drone would expect it -> collision risk.
  wall_misplaced mapped walls within 1.5 m of the estimated drone, moved into the world with the TRUE
                 pose: how far is each from the nearest true wall? Large = doubled / ghost walls nearby
                 (corridors look narrower; false blockages).

Reported over the airborne flight: 99th percentile and maximum of each, in metres.
Suggested limits (1 m corridors leave ~0.2 m each side of the 0.54 m drone):
  wall_missing p99 <= 0.10 m   and   wall_misplaced p99 <= 0.15 m

Usage: python3 nav_check.py RUN_DIR [...]   or   --glob '*_A1.5_w4_maze_rooms_*__replay_v2_cur'
Uses RUN_DIR/eval_data.npz (written by eval_run.py).
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_run as ev  # noqa: E402

LIM_MISSING, LIM_MISPLACED = 0.10, 0.15
NEAR_TRUE, NEAR_EST, DT, AIR_DZ = 0.8, 1.5, 0.5, 0.3


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def check(data, world):
    gt = ev.GroundTruth(world)
    T = data['truth'][np.argsort(data['truth'][:, 0])]
    E = data['est'][np.argsort(data['est'][:, 0])]
    _, ui = np.unique(E[:, 0], return_index=True)
    E = E[ui]
    md, mi = data['map_data'], data['map_info']
    mx, my = ev.map_cells_world(md, mi, (0.0, 0.0, 0.0))        # map frame
    occ = md >= ev.OCC_T
    known = md >= 0
    P = np.stack([mx[occ], my[occ]], 1)
    K = np.stack([mx[known], my[known]], 1)
    tree_m, tree_k = cKDTree(P), cKDTree(K)
    jt, it = np.where(gt.occ)
    G = np.stack([gt.ox + (it + .5) * gt.res, gt.oy + (jt + .5) * gt.res], 1)
    tree_g = cKDTree(G)
    z0 = float(np.median(T[:50, 3]))
    yt, ye = np.unwrap(T[:, 4]), np.unwrap(E[:, 3])
    t0, t1 = max(T[0, 0], E[0, 0]), min(T[-1, 0], E[-1, 0])
    miss_all, mis_all, worst = [], [], (0.0, None)
    for t in np.arange(t0, t1, DT):
        if np.interp(t, T[:, 0], T[:, 3]) < z0 + AIR_DZ:
            continue
        tp = np.array([np.interp(t, T[:, 0], T[:, 1]), np.interp(t, T[:, 0], T[:, 2])])
        ep = np.array([np.interp(t, E[:, 0], E[:, 1]), np.interp(t, E[:, 0], E[:, 2])])
        Rt, Re = rot(np.interp(t, T[:, 0], yt)), rot(np.interp(t, E[:, 0], ye))
        # true walls near the drone -> map frame via the estimated pose
        g = G[tree_g.query_ball_point(tp, NEAR_TRUE)]
        if len(g):
            gm = ep + ((g - tp) @ Rt) @ Re.T
            in_known = tree_k.query(gm)[0] <= gt.res        # only where the map has an opinion
            if in_known.any():
                d = tree_m.query(gm[in_known])[0]
                miss_all.append(d)
                if d.max() > worst[0]:
                    worst = (float(d.max()), [round(float(v), 2) for v in tp])
        # mapped walls near the estimate -> world via the true pose
        p = P[tree_m.query_ball_point(ep, NEAR_EST)]
        if len(p):
            pw = tp + ((p - ep) @ Re) @ Rt.T
            mis_all.append(tree_g.query(pw)[0])
    out = {'samples': len(miss_all)}
    if not miss_all:
        out['error'] = 'no airborne samples'
        return out
    a, b = np.concatenate(miss_all), np.concatenate(mis_all) if mis_all else np.zeros(1)
    out.update(wall_missing_p99=round(float(np.percentile(a, 99)), 3), wall_missing_max=round(float(a.max()), 3),
               wall_misplaced_p99=round(float(np.percentile(b, 99)), 3), wall_misplaced_max=round(float(b.max()), 3),
               worst_missing_at=worst[1])
    out['pass'] = out['wall_missing_p99'] <= LIM_MISSING and out['wall_misplaced_p99'] <= LIM_MISPLACED
    return out


def check_run(run):
    meta = ev.read_meta_txt(run)
    d = dict(np.load(os.path.join(run, 'eval_data.npz'), allow_pickle=True))
    if 'map_data' not in d or len(d['est']) < 10:
        return {'error': 'no map or estimate in eval_data.npz'}
    r = check(d, meta.get('world'))
    json.dump(r, open(os.path.join(run, 'nav_check.json'), 'w'), indent=1)
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='*')
    ap.add_argument('--glob', action='append', default=[], help="pattern in ~/drone_results (repeatable)")
    a = ap.parse_args()
    runs = list(a.runs)
    for g in a.glob:
        runs += sorted(glob.glob(os.path.join(os.path.expanduser('~/drone_results'), g)))
    runs = [r for r in runs if os.path.isfile(os.path.join(r, 'eval_data.npz'))]
    if not runs:
        ap.error('no evaluated run folders given')
    print(f"{'run':62s} {'missing p99/max':>16s} {'misplaced p99/max':>18s}  result")
    n = 0
    for r in runs:
        x = check_run(r)
        if 'error' in x:
            print(f"{os.path.basename(r)[:62]:62s} {x['error']}")
            continue
        n += x['pass']
        print(f"{os.path.basename(r)[:62]:62s} {x['wall_missing_p99']:7.3f}/{x['wall_missing_max']:<7.3f}  "
              f"{x['wall_misplaced_p99']:8.3f}/{x['wall_misplaced_max']:<8.3f}  {'PASS' if x['pass'] else 'FAIL'}")
    print(f"\n{n}/{len(runs)} pass (limits: wall_missing p99 <= {LIM_MISSING} m, wall_misplaced p99 <= {LIM_MISPLACED} m)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
