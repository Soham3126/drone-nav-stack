#!/usr/bin/env python3
"""nav_check.py - could the drone actually fly the map it built? (addition to guidebook A1.5/A1.8)

Compares the final /map of a run with the true world the way the route planner sees them:
every occupied cell is grown by the drone radius (0.27 m), so each cell says "the drone's
centre may be here" or not. Then, inside the area the map has explored:

  max_intrusion_m   worst case where the MAP allows the drone's centre but the REAL drone would
                    overlap a wall, by this much. 0 = the map never leads into a wall.
                    (crash risk; the planner also keeps an inflation margin, so a few cm is tolerable)
  cells_cut_off     1 m lattice cells the drone can really reach but cannot reach in the map
                    (false blockages: parts of the maze exploration would miss)
  missed_fraction   share of truly flyable centre space (explored area) the map forbids
  dangerous_cells   centre cells the map allows that are truly too close to a wall

Usage: python3 nav_check.py RUN_DIR [RUN_DIR ...]      (folders with eval_data.npz, i.e. evaluated runs)
       python3 nav_check.py --glob '*_A1.5_w4_maze_rooms_*__replay_v2_cur'
Suggested limits: max_intrusion_m <= 0.05 and cells_cut_off == 0.
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_run as ev  # noqa: E402
from geom import ROBOT_R  # noqa: E402

LIM_INTRUSION, LIM_CUT = 0.05, 0


def alignment(T, E):
    T = T[np.argsort(T[:, 0])]
    E = E[np.argsort(E[:, 0])]
    t_a = max(E[0, 0], T[0, 0])
    yaw = np.unwrap(T[:, 4])
    return (float(np.interp(t_a, T[:, 0], T[:, 1])), float(np.interp(t_a, T[:, 0], T[:, 2])),
            float(np.interp(t_a, T[:, 0], yaw)))


def check(md, mi, align, world):
    gt = ev.GroundTruth(world)
    res = gt.res
    wx, wy = ev.map_cells_world(md, mi, align)
    occ_m = np.zeros_like(gt.occ)
    known_m = np.zeros_like(gt.occ)
    for sel, grid in ((md >= ev.OCC_T, occ_m), (md >= 0, known_m)):
        i, j, ins = gt.idx(wx[sel], wy[sel])
        grid[j[ins], i[ins]] = True
    free_m = known_m & ~occ_m
    # centre space
    true_clear = ndimage.distance_transform_edt(~gt.occ) * res
    map_clear = ndimage.distance_transform_edt(~occ_m) * res
    c_true = true_clear >= ROBOT_R
    c_map = (map_clear >= ROBOT_R) & free_m
    s = gt.meta['start']
    si, sj, _ = gt.idx(np.array([s['x']]), np.array([s['y']]))
    si, sj = int(si[0]), int(sj[0])
    lt, _ = ndimage.label(c_true)
    lm, _ = ndimage.label(c_map)
    r_true = lt == lt[sj, si]
    if not c_map[sj, si]:
        # start pose blocked in the map: take the map component closest to the start
        d = ndimage.distance_transform_edt(~c_map, return_indices=True)[1]
        sj2, si2 = d[0][sj, si], d[1][sj, si]
        r_map = (lm == lm[sj2, si2]) & c_map
    else:
        r_map = lm == lm[sj, si]
    explored = ndimage.binary_dilation(known_m, iterations=int(round(ROBOT_R / res)))
    out = {}
    danger = r_map & ~c_true
    intr = np.clip(ROBOT_R - true_clear[r_map], 0, None)
    out['max_intrusion_m'] = round(float(intr.max()) if intr.size else 0.0, 3)
    out['dangerous_cells'] = int(danger.sum())
    if danger.any():
        jj, ii = np.unravel_index(np.argmax(np.where(danger, ROBOT_R - true_clear, -1)), danger.shape)
        out['worst_at'] = [round(float(gt.ox + (ii + .5) * res), 2), round(float(gt.oy + (jj + .5) * res), 2)]
    need = r_true & explored
    out['missed_fraction'] = round(float((need & ~r_map).sum() / max(1, need.sum())), 4)
    # lattice cells cut off
    c = gt.meta['cell_size_m']
    nx, ny = gt.meta['lattice_cells']
    cut = []
    for ix in range(nx):
        for iy in range(ny):
            x, y = (ix + .5) * c, (iy + .5) * c
            i, j = int((x - gt.ox) / res), int((y - gt.oy) / res)
            if not (r_true[j, i] and explored[j, i]):
                continue
            # cell counts as reachable in the map if any centre-space cell within 0.25 m is reachable
            k = int(0.25 / res)
            if not r_map[max(0, j - k):j + k + 1, max(0, i - k):i + k + 1].any():
                cut.append([round(x, 2), round(y, 2)])
    out['cells_cut_off'] = len(cut)
    out['cut_off_cells'] = cut[:8]
    out['pass'] = out['max_intrusion_m'] <= LIM_INTRUSION and out['cells_cut_off'] <= LIM_CUT
    return out


def check_run(run):
    meta = ev.read_meta_txt(run)
    world = meta.get('world')
    d = np.load(os.path.join(run, 'eval_data.npz'), allow_pickle=True)
    if 'map_data' not in d or len(d['est']) < 10:
        return {'error': 'no map or estimate in eval_data.npz'}
    r = check(d['map_data'], d['map_info'], alignment(d['truth'], d['est']), world)
    r['world'] = world
    json.dump(r, open(os.path.join(run, 'nav_check.json'), 'w'), indent=1)
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='*')
    ap.add_argument('--glob', help="pattern inside ~/drone_results, e.g. '*_A1.5_w4_*__replay_v2_cur'")
    a = ap.parse_args()
    runs = list(a.runs)
    if a.glob:
        runs += sorted(glob.glob(os.path.join(os.path.expanduser('~/drone_results'), a.glob)))
    runs = [r for r in runs if os.path.isfile(os.path.join(r, 'eval_data.npz'))]
    if not runs:
        ap.error('no evaluated run folders given')
    print(f"{'run':62s} {'intrusion':>9s} {'cut off':>7s} {'missed':>7s} {'danger':>6s}  result")
    n_ok = 0
    for r in runs:
        x = check_run(r)
        if 'error' in x:
            print(f"{os.path.basename(r)[:62]:62s} {x['error']}")
            continue
        n_ok += x['pass']
        print(f"{os.path.basename(r)[:62]:62s} {x['max_intrusion_m']:9.3f} {x['cells_cut_off']:7d} "
              f"{x['missed_fraction']:7.3f} {x['dangerous_cells']:6d}  {'PASS' if x['pass'] else 'FAIL'}"
              + (f"  worst at {x['worst_at']}" if x.get('worst_at') and x['max_intrusion_m'] > LIM_INTRUSION else '')
              + (f"  cut off e.g. {x['cut_off_cells'][:3]}" if x['cells_cut_off'] else ''))
    print(f"\n{n_ok}/{len(runs)} pass (limits: intrusion <= {LIM_INTRUSION} m, no cells cut off)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
