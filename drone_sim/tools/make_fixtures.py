#!/usr/bin/env python3
"""make_fixtures.py - repeatable test maps for workstream C (guidebook 4.5, test C0).

For every world it writes drone_sim/fixtures/<world>/:
  full.pgm/.yaml      the true map: walls 100, free space the drone can see 0, everything else unknown (-1)
  partial.pgm/.yaml   what the LiDAR (360 deg, 10 m) sees from a few viewpoints near the start: walls 100,
                      seen free 0, the rest unknown -> it has real frontiers
  carto.pgm/.yaml     "Cartographer-like": free 0-30, walls 60-95, a slight halo of 31-59 next to walls,
                      1% random speckle (0-64, below the wall threshold) in free space; unknown stays unknown
  openings.json       the sidecar for partial: every frontier (unknown cells next to seen free cells, grouped),
                      with its size, centre, and whether it is a real opening into free space
  info.json           viewpoints, value counts, the seed

Maps are written in map_server's "raw" mode: pixel value = map value 0..100, 255 = unknown. The map frame is
the world frame (same origin as maps/<world>/<world>.yaml).

Usage
  python3 make_fixtures.py                     # W2-W8 and wr_s1 ... wr_s20 (worlds must exist: make_maze.py)
  python3 make_fixtures.py --worlds w7_gaps    # one world
  python3 make_fixtures.py --check             # re-check the value rules on existing fixtures
Deterministic: the same world always gives the same fixtures.
"""
import argparse
import json
import math
import os
import sys
import zlib

import numpy as np
from scipy import ndimage

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = 0.05
ROBOT_R = 0.27
LIDAR_RANGE = 10.0
N_RAYS = 1440
EXPLORE_R = 2.5            # partial: viewpoints = lattice cell centres within this path distance of the start
MIN_OPENING = 0.55         # guidebook 2.7 min_frontier_size
UNKNOWN = -1

CATALOGUE = ['w2_turns_junctions', 'w3_tree_maze', 'w4_maze_rooms', 'w5_maze_loop', 'w6_width_100',
             'w6_width_130', 'w6_width_150', 'w7_gaps', 'w8_arena_full']
DEFAULT = CATALOGUE + [f'wr_s{i}' for i in range(1, 21)]


# ----------------------------------------------------------------------------- reading the true world
def read_pgm(path):
    """PGM -> array with row 0 = BOTTOM of the map (map coordinates), as make_maze.py rasterised it."""
    with open(path, 'rb') as f:
        data = f.read()
    parts, pos = [], 0
    while len(parts) < 4:                         # P5, width, height, maxval (comments not used by make_maze)
        while data[pos:pos + 1].isspace():
            pos += 1
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        parts.append(data[start:pos])
    pos += 1
    w, h = int(parts[1]), int(parts[2])
    img = np.frombuffer(data[pos:pos + w * h], dtype=np.uint8).reshape(h, w)
    return img[::-1, :].copy()


def load_world(name):
    mdir = os.path.join(PKG, 'maps', name)
    meta = json.load(open(os.path.join(mdir, 'meta.json')))
    occ = read_pgm(os.path.join(mdir, f'{name}.pgm')) == 0
    reach = read_pgm(os.path.join(mdir, 'reachable_free.pgm')) > 128
    return meta, occ, reach


def load_drone_reach(name):
    return read_pgm(os.path.join(PKG, 'maps', name, 'reachable_drone.pgm')) > 128


def to_cell(meta, x, y):
    ox, oy = meta['map']['origin']
    return int(math.floor((y - oy) / RES)), int(math.floor((x - ox) / RES))      # (row, col)


def to_xy(meta, r, c):
    ox, oy = meta['map']['origin']
    return ox + (c + 0.5) * RES, oy + (r + 0.5) * RES


# ----------------------------------------------------------------------------- fixtures
def full_map(occ, reach):
    m = np.full(occ.shape, UNKNOWN, dtype=np.int16)
    m[reach] = 0
    m[occ] = 100
    return m


def viewpoints(meta, occ):
    """Lattice cell centres the drone can reach within EXPLORE_R of the start (path distance), start first."""
    dist = ndimage.distance_transform_edt(~occ) * RES
    fits = dist >= ROBOT_R
    s = meta['start']
    sr, sc = to_cell(meta, s['x'], s['y'])
    # path distance over cells where the drone fits (4-connected BFS)
    geo = np.full(occ.shape, np.inf)
    geo[sr, sc] = 0.0
    frontier = [(sr, sc)]
    while frontier:
        nxt = []
        for r, c in frontier:
            d = geo[r, c] + RES
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < occ.shape[0] and 0 <= cc < occ.shape[1] and fits[rr, cc] and geo[rr, cc] > d:
                    geo[rr, cc] = d
                    nxt.append((rr, cc))
        frontier = nxt
    cell = meta['cell_size_m']
    nx, ny = meta['lattice_cells']
    pts = [(s['x'], s['y'])]
    for iy in range(ny):
        for ix in range(nx):
            x, y = (ix + 0.5) * cell, (iy + 0.5) * cell
            r, c = to_cell(meta, x, y)
            if 0 <= r < occ.shape[0] and 0 <= c < occ.shape[1] and 0 < geo[r, c] <= EXPLORE_R:
                pts.append((x, y))
    return pts, geo


def raycast(meta, occ, pts):
    """Cells seen free and walls hit from each viewpoint (360 deg, LIDAR_RANGE), vectorised over rays."""
    seen_free = np.zeros(occ.shape, bool)
    seen_wall = np.zeros(occ.shape, bool)
    ox, oy = meta['map']['origin']
    H, W = occ.shape
    steps = np.arange(0.0, LIDAR_RANGE, RES * 0.5)
    angles = np.linspace(0, 2 * math.pi, N_RAYS, endpoint=False)
    ca, sa = np.cos(angles)[:, None], np.sin(angles)[:, None]
    for x, y in pts:
        cols = np.floor((x + steps * ca - ox) / RES).astype(int)          # rays x steps
        rows = np.floor((y + steps * sa - oy) / RES).astype(int)
        inside = (rows >= 0) & (rows < H) & (cols >= 0) & (cols < W)
        hit = np.zeros_like(inside)
        hit[inside] = occ[rows[inside], cols[inside]]
        stop = ~inside | hit
        first = np.where(stop.any(axis=1), stop.argmax(axis=1), steps.size)   # first blocked step per ray
        before = np.arange(steps.size)[None, :] < first[:, None]
        seen_free[rows[before & inside], cols[before & inside]] = True
        k = np.nonzero(first < steps.size)[0]
        hk = k[hit[k, first[k]]]
        seen_wall[rows[hk, first[hk]], cols[hk, first[hk]]] = True
    seen_free &= ~seen_wall
    # a wall face next to seen free space is seen too (rays hit it at a grazing angle or just miss its cell)
    seen_wall |= occ & ndimage.binary_dilation(seen_free, structure=np.ones((3, 3)))
    return seen_free, seen_wall


def partial_map(seen_free, seen_wall):
    m = np.full(seen_free.shape, UNKNOWN, dtype=np.int16)
    m[seen_free] = 0
    m[seen_wall] = 100
    return m


def frontier_groups(m, meta, occ, reach, dreach):
    """Frontier cells = unknown cells with a free 4-neighbour; 8-connected groups = frontiers."""
    free = m == 0
    unk = m == UNKNOWN
    nb = np.zeros_like(free)
    nb[1:, :] |= free[:-1, :]
    nb[:-1, :] |= free[1:, :]
    nb[:, 1:] |= free[:, :-1]
    nb[:, :-1] |= free[:, 1:]
    fcells = unk & nb
    lab, n = ndimage.label(fcells, structure=np.ones((3, 3)))
    out = []
    for k in range(1, n + 1):
        rr, cc = np.nonzero(lab == k)
        truth_free = float(np.mean(reach[rr, cc] & ~occ[rr, cc]))
        xs, ys = zip(*(to_xy(meta, r, c) for r, c in zip(rr, cc)))
        ext = math.hypot(max(xs) - min(xs), max(ys) - min(ys)) + RES
        size = len(rr) * RES
        real = truth_free >= 0.5
        # can the drone (0.27 m circle) actually get to it? some of its cells inside the space it can sweep
        reachable = bool(dreach[rr, cc].any())
        out.append(dict(id=k, cells=int(len(rr)), size_m=round(size, 3), extent_m=round(ext, 3),
                        centroid=[round(float(np.mean(xs)), 3), round(float(np.mean(ys)), 3)],
                        truth_free_fraction=round(truth_free, 3), real_opening=bool(real),
                        drone_reachable=reachable,
                        passable=bool(real and reachable and size >= MIN_OPENING)))
    out.sort(key=lambda g: -g['cells'])
    return out


def carto_map(full, occ, seed):
    rng = np.random.default_rng(seed)
    m = full.copy()
    free = full == 0
    wall = full == 100
    m[free] = rng.integers(0, 31, int(free.sum()))
    m[wall] = rng.integers(60, 96, int(wall.sum()))
    # slight blur: free cells touching a wall (8-neighbourhood) get an in-between value
    halo = ndimage.binary_dilation(wall, structure=np.ones((3, 3))) & free
    m[halo] = rng.integers(31, 60, int(halo.sum()))
    # 1% speckle in free space: any value below the wall threshold (0-64). A speckle cell >= 65 in free space is a
    # false wall, and one false wall cell blocks a 1 m corridor for the 0.54 m drone (first C1 run: with speckle up
    # to 100, 0.43% of free cells became walls and no path existed on any carto-like map). False walls are a SLAM
    # fault (workstream A's phantom cells), not something the planner fixture should contain.
    fr, fc = np.nonzero(free)
    k = int(round(0.01 * len(fr)))
    pick = rng.choice(len(fr), k, replace=False)
    m[fr[pick], fc[pick]] = rng.integers(0, 65, k)
    speck = np.zeros_like(free)
    speck[fr[pick], fc[pick]] = True
    return m, halo, speck


def write_raw(path_base, m, meta, note):
    """map_server raw mode: pixel = value (0..100), 255 = unknown. PGM row 0 = top."""
    img = np.where(m < 0, 255, m).astype(np.uint8)[::-1, :]
    with open(path_base + '.pgm', 'wb') as f:
        f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
        f.write(img.tobytes())
    ox, oy = meta['map']['origin']
    name = os.path.basename(path_base)
    with open(path_base + '.yaml', 'w') as f:
        f.write(f"# {note}\nimage: {name}.pgm\nmode: raw\nresolution: {RES}\n"
                f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def counts(m):
    return {'unknown': int((m < 0).sum()), 'v0_30': int(((m >= 0) & (m <= 30)).sum()),
            'v31_59': int(((m >= 31) & (m <= 59)).sum()), 'v60_95': int(((m >= 60) & (m <= 95)).sum()),
            'v96_100': int((m >= 96).sum())}


def check(name, out_dir):
    """The C0 value rules, on the files as written (read back from disk)."""
    meta, occ, reach = load_world(name)
    problems = []
    def rd(kind):
        img = read_pgm(os.path.join(out_dir, kind + '.pgm')).astype(np.int16)
        return np.where(img == 255, UNKNOWN, img)
    full, part, carto = rd('full'), rd('partial'), rd('carto')
    info = json.load(open(os.path.join(out_dir, 'info.json')))
    if not np.array_equal(full == 100, occ) or not np.array_equal(full == 0, reach & ~occ):
        problems.append('full: walls/free differ from the true map')
    if set(np.unique(full)) - {UNKNOWN, 0, 100} or set(np.unique(part)) - {UNKNOWN, 0, 100}:
        problems.append('full/partial: values other than -1/0/100')
    if ((part == 0) & occ).any() or ((part == 100) & ~occ).any():
        problems.append('partial: a seen cell disagrees with the true map')
    free, wall = full == 0, full == 100
    halo = ndimage.binary_dilation(wall, structure=np.ones((3, 3))) & free
    sp = np.zeros_like(free)
    for r, c in info['speckle_cells']:
        sp[r, c] = True
    plain = free & ~halo & ~sp
    if ((carto[plain] < 0) | (carto[plain] > 30)).any():
        problems.append('carto: free cell outside 0-30')
    if ((carto[wall] < 60) | (carto[wall] > 95)).any():
        problems.append('carto: wall cell outside 60-95')
    if ((carto[halo & ~sp] < 31) | (carto[halo & ~sp] > 59)).any():
        problems.append('carto: halo cell outside 31-59')
    if not np.array_equal(carto < 0, full < 0):
        problems.append('carto: unknown cells differ from full')
    frac = sp.sum() / max(1, free.sum())
    if not 0.009 <= frac <= 0.011:
        problems.append(f'carto: speckle {frac*100:.2f}% (should be 1%)')
    if not os.path.exists(os.path.join(out_dir, 'openings.json')):
        problems.append('partial: openings.json missing')
    return problems


def make(name, root):
    meta, occ, reach = load_world(name)
    out = os.path.join(root, name)
    os.makedirs(out, exist_ok=True)
    seed = zlib.crc32(name.encode())
    full = full_map(occ, reach)
    pts, _ = viewpoints(meta, occ)
    sf, sw = raycast(meta, occ, pts)
    part = partial_map(sf, sw)
    carto, halo, speck = carto_map(full, occ, seed)
    write_raw(os.path.join(out, 'full'), full, meta, f'{name}: true map (C0 fixture "full")')
    write_raw(os.path.join(out, 'partial'), part, meta, f'{name}: seen from {len(pts)} viewpoints (C0 "partial")')
    write_raw(os.path.join(out, 'carto'), carto, meta, f'{name}: Cartographer-like values (C0 "carto-like")')
    groups = frontier_groups(part, meta, occ, reach, load_drone_reach(name))
    json.dump({'world': name, 'resolution': RES, 'min_opening_m': MIN_OPENING,
               'rule': 'frontier = unknown cell with a free 4-neighbour, 8-connected groups; real_opening = most of '
                       'its cells are truly free; drone_reachable = some of its cells lie in the space the 0.27 m '
                       'drone can sweep from the start; passable = real and drone_reachable and size_m >= '
                       'min_opening_m (the frontiers the explorer must find)',
               'openings': groups}, open(os.path.join(out, 'openings.json'), 'w'), indent=1)
    sr, sc = np.nonzero(speck)
    json.dump({'world': name, 'seed': seed, 'viewpoints': [[round(x, 3), round(y, 3)] for x, y in pts],
               'lidar_range_m': LIDAR_RANGE, 'rays': N_RAYS, 'explore_radius_m': EXPLORE_R,
               'counts': {'full': counts(full), 'partial': counts(part), 'carto': counts(carto)},
               'speckle_cells': [[int(r), int(c)] for r, c in zip(sr, sc)]},
              open(os.path.join(out, 'info.json'), 'w'))
    real = [g for g in groups if g['real_opening']]
    return dict(world=name, viewpoints=len(pts), frontiers=len(groups), real=len(real),
                passable=sum(g['passable'] for g in real), problems=check(name, out))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--worlds', nargs='*', default=DEFAULT)
    ap.add_argument('--root', default=os.path.join(PKG, 'fixtures'))
    ap.add_argument('--check', action='store_true', help='only re-check existing fixtures')
    a = ap.parse_args()
    bad = 0
    for w in a.worlds:
        if not os.path.exists(os.path.join(PKG, 'maps', w, 'meta.json')):
            print(f'{w:22s} MISSING world (generate it: make_maze.py --random --seeds ...)')
            bad += 1
            continue
        if a.check:
            p = check(w, os.path.join(a.root, w))
            print(f"{w:22s} {'ok' if not p else 'FAIL ' + '; '.join(p)}")
        else:
            r = make(w, a.root)
            print(f"{w:22s} viewpoints {r['viewpoints']:3d}  frontiers {r['frontiers']:3d} "
                  f"(real openings {r['real']}, passable {r['passable']})  "
                  f"{'ok' if not r['problems'] else 'FAIL ' + '; '.join(r['problems'])}")
            p = r['problems']
        bad += bool(p)
    print('all fixtures ok' if not bad else f'{bad} world(s) with problems')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
