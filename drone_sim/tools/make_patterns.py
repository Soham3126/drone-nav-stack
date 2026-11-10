#!/usr/bin/env python3
"""make_patterns.py - waypoint patterns for scripted flights, generated from meta.json (guidebook A0.4).

  w0_*  square_3m          start -> 3 m square placed for the most clearance from walls and pillars -> start
  w1_*  corridor_out_back  10 m along the corridor and back
  w2_*  turns_tour         visits every 1 m cell (depth-first), so every turn, junction and crossroads
  w5_*  loop_lap           every cell; the depth-first tour also runs round the loop
  other maze_lap           every cell of the maze, rooms included, back to the start

Output: drone_sim/patterns/<world>/<pattern>.yaml with waypoints in the Gazebo WORLD frame
(x, y metres), flight height, speed and the path's smallest clearance from any wall face.
Every segment must keep at least --min-clear (default 0.35 m = 0.27 m drone radius + margin
for GPS noise) from every wall and tall box; cell links that cannot are left out, so narrow
gaps (W7's 0.45 / 0.60 m) are never flown through.

Usage: python3 make_patterns.py --all          (every world that has a meta.json)
       python3 make_patterns.py --world w2_turns_junctions
"""
import argparse
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geom import PKG_DIR, load_meta, obstacle_rects, path_clearance, seg_rect_dist, clearance_at  # noqa: E402

HEIGHT = 1.1
SPEED = 0.5


def square_pattern(meta, rects, min_clear, side=3.0):
    sx, sy = meta['start']['x'], meta['start']['y']
    nx, ny = meta['lattice_cells']
    c = meta['cell_size_m']
    best = None
    steps = int(round((nx * c) / 0.05))
    for i in range(steps):
        for j in range(int(round((ny * c) / 0.05))):
            x0, y0 = i * 0.05, j * 0.05
            sq = [(x0, y0), (x0 + side, y0), (x0 + side, y0 + side), (x0, y0 + side)]
            # enter at the corner nearest the start, go round once, return
            k = min(range(4), key=lambda q: math.dist(sq[q], (sx, sy)))
            loop = sq[k:] + sq[:k] + [sq[k]]
            pts = [(sx, sy)] + loop + [(sx, sy)]
            cl = path_clearance(pts, rects)
            if best is None or cl > best[0]:
                best = (cl, pts)
    if best[0] < min_clear:
        raise RuntimeError(f"no 3 m square fits with {min_clear} m clearance (best {best[0]:.2f} m)")
    return best[1]


def out_back_pattern(meta, rects, min_clear, length=10.0):
    sx, sy, yaw = meta['start']['x'], meta['start']['y'], meta['start']['yaw']
    far = (sx + length * math.cos(yaw), sy + length * math.sin(yaw))
    pts = [(sx, sy), far, (sx, sy)]
    if path_clearance(pts, rects) < min_clear:
        raise RuntimeError('corridor_out_back does not fit')
    return pts


def tour_pattern(meta, rects, min_clear):
    """Depth-first tour of the cell lattice, returning to the start cell."""
    nx, ny = meta['lattice_cells']
    c = meta['cell_size_m']
    sx, sy = meta['start']['x'], meta['start']['y']

    def centre(i, j):
        return ((i + 0.5) * c, (j + 0.5) * c)

    ok = {(i, j) for i in range(nx) for j in range(ny) if clearance_at(*centre(i, j), rects) >= min_clear}
    adj = {n: [] for n in ok}
    for (i, j) in ok:
        for di, dj in ((1, 0), (0, 1), (-1, 0), (0, -1)):   # E, N, W, S: deterministic order
            m = (i + di, j + dj)
            if m in ok:
                a, b = centre(i, j), centre(*m)
                if min(seg_rect_dist(a[0], a[1], b[0], b[1], r) for r in rects) >= min_clear:
                    adj[(i, j)].append(m)
    start = (int(sx // c), int(sy // c))
    if start not in ok:
        raise RuntimeError('start cell has too little clearance')
    seq, seen = [start], {start}

    def dfs(n):
        for m in adj[n]:
            if m not in seen:
                seen.add(m)
                seq.append(m)
                dfs(m)
                seq.append(n)
    sys.setrecursionlimit(10000)
    dfs(start)
    pts = [(sx, sy)] + [centre(*n) for n in seq[1:]]
    if math.dist(pts[-1], (sx, sy)) > 1e-6:
        pts.append((sx, sy))
    # drop points in the middle of straight runs
    out = [pts[0]]
    for p, q in zip(pts[1:], pts[2:]):
        a = out[-1]
        cross = (p[0] - a[0]) * (q[1] - p[1]) - (p[1] - a[1]) * (q[0] - p[0])
        dot = (p[0] - a[0]) * (q[0] - p[0]) + (p[1] - a[1]) * (q[1] - p[1])
        if abs(cross) > 1e-9 or dot < 0:
            out.append(p)
    out.append(pts[-1])
    unreached = len(ok) - len(seen)
    return out, len(seen), unreached


def build(world, min_clear):
    meta = load_meta(world)
    rects = obstacle_rects(meta, flight=True)
    info = {}
    if world.startswith('w0'):
        name, pts = 'square_3m', square_pattern(meta, rects, min_clear)
    elif world.startswith('w1'):
        name, pts = 'corridor_out_back', out_back_pattern(meta, rects, min_clear)
    else:
        name = {'w2': 'turns_tour', 'w5': 'loop_lap'}.get(world[:2], 'maze_lap')
        pts, visited, unreached = tour_pattern(meta, rects, min_clear)
        info = {'cells_visited': visited, 'cells_not_reachable_with_clearance': unreached}
    length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    doc = {
        'world': world, 'pattern': name, 'frame': 'world',
        'height_m': HEIGHT, 'speed_m_s': SPEED, 'accept_m': 0.25,
        'start': [meta['start']['x'], meta['start']['y'], meta['start']['yaw']],
        'path_length_m': round(length, 2),
        'min_clearance_to_face_m': round(path_clearance(pts, rects), 3),
        **info,
        'waypoints': [[round(x, 3), round(y, 3)] for x, y in pts],
    }
    d = os.path.join(PKG_DIR, 'patterns', world)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, f'{name}.yaml'), 'w') as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)
    return doc


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--world', action='append', default=[])
    ap.add_argument('--min-clear', type=float, default=0.35)
    args = ap.parse_args()
    worlds = args.world
    if args.all:
        worlds = sorted(d for d in os.listdir(os.path.join(PKG_DIR, 'maps'))
                        if os.path.isfile(os.path.join(PKG_DIR, 'maps', d, 'meta.json')))
    if not worlds:
        ap.print_help()
        return 1
    for w in worlds:
        doc = build(w, args.min_clear)
        extra = f" cells={doc['cells_visited']}" if 'cells_visited' in doc else ''
        print(f"{w:20s} {doc['pattern']:18s} waypoints={len(doc['waypoints']):3d} "
              f"length={doc['path_length_m']:6.1f} m  clearance={doc['min_clearance_to_face_m']:.3f} m{extra}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
