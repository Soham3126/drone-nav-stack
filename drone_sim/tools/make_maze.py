#!/usr/bin/env python3
"""make_maze.py - world generator (guidebook 4.2, test A0.1).

Turns a world description into
  worlds/<name>.sdf                  Gazebo world with walls, boxes and iris_with_lidar
  maps/<name>/<name>.pgm + .yaml     exact ground-truth occupancy map, 5 cm
  maps/<name>/reachable_free.pgm     free cells connected to the start (geometric)
  maps/<name>/reachable_drone.pgm    free space the 0.27 m drone circle can sweep from the start
  maps/<name>/meta.json              walls, boxes, rooms, doors, start pose, cell counts

Usage
  python3 make_maze.py --all                 # W0-W8
  python3 make_maze.py --random --seeds 1-5  # wr_s1 ... wr_s5
  python3 make_maze.py --world w3_tree_maze  # one catalogue world

How a world is described
  The floor is a lattice of square cells (cell size = corridor width). Walls are
  0.05 m thick and lie on cell edges. A wall is placed on every edge of a free cell
  that is not "open". Two ways to say which cells are free and which edges are open:
    blocks : a text picture, one character per cell. '#' or ' ' = solid,
             any other character = free. Neighbouring free cells with the SAME
             character are joined (edge open); different characters are separated
             by a wall. 'S' counts as '.' and marks nothing special.
    maze   : a random spanning-tree maze (seeded, so repeatable), with optional
             2x2-cell rooms and extra openings that create loops.
  Extra items: boxes (pillars, obstacles) and doors (gaps cut into a wall).

Ground truth = everything that crosses the LiDAR plane in flight (1.1 m + 0.12 m).
Boxes lower than that are written to the world but kept out of the map
(listed as low_boxes in meta.json).
"""
import argparse
import json
import math
import os
import random
import sys
from collections import deque

import numpy as np
from scipy import ndimage

RES = 0.05            # map resolution (m)
WALL_T = 0.05         # wall thickness (m)
WALL_H = 2.44         # wall height (m)
MARGIN = 0.5          # map border around the lattice (m)
ROBOT_R = 0.27        # drone radius (guidebook 1.8)
FLIGHT_Z = 1.1
LIDAR_DZ = 0.12
LIDAR_PLANE_Z = FLIGHT_Z + LIDAR_DZ
SPAWN_Z = 0.195       # iris standing on the ground
EPS = 1e-6

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ----------------------------------------------------------------------------- lattice
class Lattice:
    """nx x ny cells of size `cell`; cell (ix, iy) spans [ix*c,(ix+1)*c] x [iy*c,(iy+1)*c]."""

    def __init__(self, nx, ny, cell):
        self.nx, self.ny, self.cell = nx, ny, cell
        self.free = np.zeros((ny, nx), dtype=bool)
        self.open = set()   # frozenset({(ix,iy),(jx,jy)}) of adjacent cells joined

    def inside(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def is_free(self, ix, iy):
        return self.inside(ix, iy) and bool(self.free[iy, ix])

    def set_open(self, a, b):
        self.open.add(frozenset((a, b)))

    def is_open(self, a, b):
        return frozenset((a, b)) in self.open

    def wall_edges(self):
        """Unit edges carrying a wall: ('h', ix, iy) runs along x at y=iy*c,
        ('v', ix, iy) runs along y at x=ix*c."""
        edges = set()
        for iy in range(self.ny):
            for ix in range(self.nx):
                if not self.free[iy, ix]:
                    continue
                for (nx_, ny_), key in (((ix, iy - 1), ('h', ix, iy)),
                                        ((ix, iy + 1), ('h', ix, iy + 1)),
                                        ((ix - 1, iy), ('v', ix, iy)),
                                        ((ix + 1, iy), ('v', ix + 1, iy))):
                    if not (self.is_free(nx_, ny_) and self.is_open((ix, iy), (nx_, ny_))):
                        edges.add(key)
        return edges

    def wall_segments(self):
        """Merge collinear unit edges into long walls.
        Returns dicts {o: 'h'|'v', c: fixed coord, a: start, b: end} in metres."""
        c = self.cell
        edges = self.wall_edges()
        segs = []
        for orient in ('h', 'v'):
            lines = {}
            for o, ix, iy in edges:
                if o != orient:
                    continue
                if orient == 'h':
                    lines.setdefault(iy, []).append(ix)
                else:
                    lines.setdefault(ix, []).append(iy)
            for fixed, idx in sorted(lines.items()):
                idx.sort()
                start = prev = idx[0]
                for k in idx[1:] + [None]:
                    if k is not None and k == prev + 1:
                        prev = k
                        continue
                    segs.append({'o': orient, 'c': round(fixed * c, 6),
                                 'a': round(start * c, 6), 'b': round((prev + 1) * c, 6),
                                 'ext_a': True, 'ext_b': True})
                    if k is not None:
                        start = prev = k
        return segs


def cut_doors(segs, doors):
    """Cut a gap of doors[i]['width'] centred on (x, y) into the wall that contains it.
    Door edges are not extended, so the opening is exactly the requested width."""
    for d in doors:
        x, y, w = d['x'], d['y'], d['width']
        hit = None
        for s in segs:
            if s['o'] == 'h' and abs(s['c'] - y) < EPS and s['a'] + EPS < x < s['b'] - EPS:
                hit, pos = s, x
                break
            if s['o'] == 'v' and abs(s['c'] - x) < EPS and s['a'] + EPS < y < s['b'] - EPS:
                hit, pos = s, y
                break
        if hit is None:
            raise ValueError(f"door at ({x}, {y}) is not on any wall")
        segs.remove(hit)
        lo, hi = round(pos - w / 2, 6), round(pos + w / 2, 6)
        if lo - hit['a'] > EPS:
            segs.append(dict(hit, b=lo, ext_b=False))
        if hit['b'] - hi > EPS:
            segs.append(dict(hit, a=hi, ext_a=False))
    return segs


def seg_to_rect(s):
    """Wall segment -> axis-aligned rectangle [x0, y0, x1, y1] (ends extended by T/2 at corners)."""
    h = WALL_T / 2
    a = s['a'] - (h if s['ext_a'] else 0.0)
    b = s['b'] + (h if s['ext_b'] else 0.0)
    if s['o'] == 'h':
        return [a, s['c'] - h, b, s['c'] + h]
    return [s['c'] - h, a, s['c'] + h, b]


# ----------------------------------------------------------------------------- generators
def from_blocks(rows, cell):
    """rows[0] is the TOP row of the picture."""
    rows = [r.rstrip('\n') for r in rows]
    ny, nx = len(rows), max(len(r) for r in rows)
    lat = Lattice(nx, ny, cell)
    label = {}
    for r, line in enumerate(rows):
        iy = ny - 1 - r
        for ix, ch in enumerate(line.ljust(nx)):
            if ch not in '# ':
                lat.free[iy, ix] = True
                label[(ix, iy)] = '.' if ch == 'S' else ch
    for (ix, iy), ch in label.items():
        for n in ((ix + 1, iy), (ix, iy + 1)):
            if label.get(n) == ch:
                lat.set_open((ix, iy), n)
    return lat, []


def from_maze(nx, ny, cell, seed, n_rooms=0, n_loops=0, start=(0, 0)):
    """Random spanning-tree maze. Each 2x2 room is one node of the tree, so it is
    entered through one or more 1-cell doors. Loops: extra openings between cells far
    apart in the tree."""
    rng = random.Random(seed)
    lat = Lattice(nx, ny, cell)
    lat.free[:, :] = True

    # rooms: 2x2 blocks, not touching each other, not on the start cell or next to it
    rooms, node = [], {}
    tries = 0
    while len(rooms) < n_rooms and tries < 5000:
        tries += 1
        rx, ry = rng.randrange(0, nx - 1), rng.randrange(0, ny - 1)
        cells = [(rx, ry), (rx + 1, ry), (rx, ry + 1), (rx + 1, ry + 1)]
        if any(abs(cx - start[0]) + abs(cy - start[1]) <= 1 for cx, cy in cells):
            continue
        bad = False
        for (ox, oy) in rooms:
            if abs(rx - ox) < 3 and abs(ry - oy) < 3:   # keep a 1-cell gap between rooms
                bad = True
                break
        if bad:
            continue
        rooms.append((rx, ry))
        for cc in cells:
            node[cc] = ('room', len(rooms) - 1)
        for a in cells:
            for b in cells:
                if a < b and abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1:
                    lat.set_open(a, b)
    if len(rooms) < n_rooms:
        raise RuntimeError(f"could only place {len(rooms)} of {n_rooms} rooms")

    def node_of(c):
        return node.get(c, c)

    members = {}
    for iy in range(ny):
        for ix in range(nx):
            members.setdefault(node_of((ix, iy)), []).append((ix, iy))

    # depth-first spanning tree over nodes
    tree = {}  # cell -> neighbouring cells joined through the tree (for loop distance)
    visited = {node_of(start)}
    stack = [node_of(start)]
    while stack:
        cur = stack[-1]
        cand = []
        for (ix, iy) in members[cur]:
            for n in ((ix + 1, iy), (ix - 1, iy), (ix, iy + 1), (ix, iy - 1)):
                if lat.inside(*n) and node_of(n) not in visited:
                    cand.append(((ix, iy), n))
        if not cand:
            stack.pop()
            continue
        a, b = rng.choice(cand)
        lat.set_open(a, b)
        tree.setdefault(a, []).append(b)
        tree.setdefault(b, []).append(a)
        visited.add(node_of(b))
        stack.append(node_of(b))

    # loops
    if n_loops:
        def tree_dist(a, b):
            # BFS over open edges (rooms included)
            q, seen = deque([(a, 0)]), {a}
            while q:
                c, d = q.popleft()
                if c == b:
                    return d
                for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
                    if lat.inside(*n) and n not in seen and lat.is_open(c, n):
                        seen.add(n)
                        q.append((n, d + 1))
            return 10 ** 9
        closed = []
        for iy in range(ny):
            for ix in range(nx):
                for n in ((ix + 1, iy), (ix, iy + 1)):
                    if lat.inside(*n) and not lat.is_open((ix, iy), n) \
                            and (ix, iy) not in node and n not in node:
                        closed.append(((ix, iy), n))
        rng.shuffle(closed)
        made = 0
        for a, b in closed:
            if made >= n_loops:
                break
            if tree_dist(a, b) >= 8:
                lat.set_open(a, b)
                made += 1
        if made < n_loops:
            raise RuntimeError(f"could only make {made} of {n_loops} loops")

    room_rects = [{'x0': rx * cell, 'y0': ry * cell, 'x1': (rx + 2) * cell, 'y1': (ry + 2) * cell}
                  for rx, ry in rooms]
    return lat, room_rects


# ----------------------------------------------------------------------------- catalogue
def cell_center(ix, iy, c):
    return (ix + 0.5) * c, (iy + 0.5) * c


def catalogue():
    W = {}

    def w0():
        lat, rooms = from_blocks(['......'] * 6, 1.0)
        boxes = [dict(x=x, y=y, sx=0.3, sy=0.3, h=WALL_H)            # four asymmetric pillars
                 for x, y in ((1.5, 1.5), (4.3, 1.7), (1.9, 4.4), (4.5, 4.2))]
        return dict(lat=lat, rooms=rooms, boxes=boxes, doors=[], start=(3.0, 3.0, 0.0),
                    layout='6 x 6 m closed room with four 0.3 m pillars')
    W['w0_box_room'] = w0

    def w1():
        lat, rooms = from_blocks(['.' * 12], 1.0)
        return dict(lat=lat, rooms=rooms, boxes=[], doors=[], start=(0.5, 0.5, 0.0),
                    layout='12 m straight corridor, 1.0 m wide, closed ends')
    W['w1_corridor_plain'] = w1

    def w2():
        rows = ['##.......##',
                '##.##.#####',
                '##.##.####.',
                '#####.####.',
                '#####.####.',
                '...........',
                '#####.#####',
                '#####.#####',
                '#####.#####']
        lat, rooms = from_blocks(rows, 1.0)
        return dict(lat=lat, rooms=rooms, boxes=[], doors=[], start=(0.5, 3.5, 0.0),
                    layout='1.0 m corridors: crossroads at (5.5,3.5), L-turns at (10.5,3.5) and '
                           '(2.5,8.5), T-junction at (5.5,8.5)')
    W['w2_turns_junctions'] = w2

    def w3(cell=1.0):
        lat, rooms = from_maze(8, 8, cell, seed=3)
        s = cell_center(0, 0, cell)
        return dict(lat=lat, rooms=rooms, boxes=[], doors=[], start=(s[0], s[1], 0.0),
                    layout=f'8 x 8 cell tree maze (no loops), {cell} m corridors')
    W['w3_tree_maze'] = w3

    def w4():
        lat, rooms = from_maze(10, 10, 1.0, seed=4, n_rooms=4)
        return dict(lat=lat, rooms=rooms, boxes=[], doors=[], start=(0.5, 0.5, 0.0),
                    layout='10 x 10 m maze with four 2 x 2 m rooms (1 m doors)')
    W['w4_maze_rooms'] = w4

    def w5():
        lat, rooms = from_maze(10, 10, 1.0, seed=5, n_loops=1)
        return dict(lat=lat, rooms=rooms, boxes=[], doors=[], start=(0.5, 0.5, 0.0),
                    layout='10 x 10 m maze with one loop around an island')
    W['w5_maze_loop'] = w5

    for width, tag in ((1.0, '100'), (1.3, '130'), (1.5, '150')):
        W[f'w6_width_{tag}'] = (lambda c=width: w3(c))

    def w7():
        # 12 m corridor along the bottom; three 2 x 2 m side rooms reached through
        # gaps of 0.45 m (impassable), 0.60 m (just passable) and 0.90 m.
        rows = ['#aa##bb##cc#',
                '#aa##bb##cc#',
                '............']
        lat, rooms = from_blocks(rows, 1.0)
        rooms = [{'x0': x0, 'y0': 1.0, 'x1': x0 + 2.0, 'y1': 3.0} for x0 in (1.0, 5.0, 9.0)]
        doors = [dict(x=2.0, y=1.0, width=0.45), dict(x=6.0, y=1.0, width=0.60),
                 dict(x=10.0, y=1.0, width=0.90)]
        return dict(lat=lat, rooms=rooms, boxes=[], doors=doors, start=(0.5, 0.5, 0.0),
                    layout='corridor with side-room gaps of 0.45, 0.60 and 0.90 m')
    W['w7_gaps'] = w7

    def w8():
        lat, rooms = from_maze(15, 15, 1.0, seed=8, n_rooms=6, n_loops=1)
        r0, r1 = rooms[0], rooms[1]
        boxes = [dict(x=(r0['x0'] + r0['x1']) / 2, y=(r0['y0'] + r0['y1']) / 2,
                      sx=0.4, sy=0.4, h=2.0, note='tall box'),
                 dict(x=(r1['x0'] + r1['x1']) / 2, y=(r1['y0'] + r1['y1']) / 2,
                      sx=0.4, sy=0.4, h=0.3, note='low box, below the LiDAR plane')]
        return dict(lat=lat, rooms=rooms, boxes=boxes, doors=[], start=(0.5, 0.5, 0.0),
                    layout='15 x 15 m: corridors, junctions, six rooms, one tall and one low box')
    W['w8_arena_full'] = w8
    return W


def random_world(seed):
    rng = random.Random(1000 + seed)
    cell = round(rng.uniform(1.0, 1.5), 1)
    size_m = rng.uniform(8.0, 15.0)
    n = max(6, int(round(size_m / cell)))
    n_rooms = min(rng.randint(0, 6), max(0, (n * n) // 20))
    n_loops = rng.randint(0, 2)
    lat, rooms = from_maze(n, n, cell, seed=seed, n_rooms=n_rooms, n_loops=n_loops)
    s = cell_center(0, 0, cell)
    return dict(lat=lat, rooms=rooms, boxes=[], doors=[], start=(s[0], s[1], 0.0),
                layout=f'random maze seed {seed}: {n} x {n} cells of {cell} m '
                       f'({n * cell:.1f} m), {n_rooms} rooms, {n_loops} loops')


# ----------------------------------------------------------------------------- outputs
def box_rect(b):
    return [b['x'] - b['sx'] / 2, b['y'] - b['sy'] / 2, b['x'] + b['sx'] / 2, b['y'] + b['sy'] / 2]


def rasterize(rects, origin, shape):
    """Cell (i, j) is occupied if its square overlaps the rectangle by more than EPS."""
    grid = np.zeros(shape, dtype=bool)          # [row=y index from bottom, col=x index]
    ox, oy = origin
    for x0, y0, x1, y1 in rects:
        i0 = max(0, math.floor((x0 - ox) / RES + EPS))
        i1 = min(shape[1] - 1, math.ceil((x1 - ox) / RES - EPS) - 1)
        j0 = max(0, math.floor((y0 - oy) / RES + EPS))
        j1 = min(shape[0] - 1, math.ceil((y1 - oy) / RES - EPS) - 1)
        if i1 >= i0 and j1 >= j0:
            grid[j0:j1 + 1, i0:i1 + 1] = True
    return grid


def write_pgm(path, mask_occ=None, mask_white=None):
    """Write a map-server PGM (row 0 = top). mask_occ True -> black (0), else white (254)."""
    if mask_occ is not None:
        img = np.where(mask_occ, 0, 254).astype(np.uint8)
    else:
        img = np.where(mask_white, 254, 0).astype(np.uint8)
    img = img[::-1, :]
    with open(path, 'wb') as f:
        f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
        f.write(img.tobytes())


def sdf_world(name, walls, boxes, start):
    out = [f'''<?xml version="1.0"?>
<!-- Generated by drone_sim/tools/make_maze.py. Do not edit by hand. -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <sky></sky>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      </link>
    </model>''']

    def box_model(mname, cx, cy, sx, sy, h, rgb):
        return f'''
    <model name="{mname}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} {h / 2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx:.4f} {sy:.4f} {h:.4f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.4f} {sy:.4f} {h:.4f}</size></box></geometry>
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>
      </link>
    </model>'''

    for k, (x0, y0, x1, y1) in enumerate(walls, 1):
        out.append(box_model(f'wall_{k:03d}', (x0 + x1) / 2, (y0 + y1) / 2,
                             x1 - x0, y1 - y0, WALL_H, '0.85 0.85 0.8'))
    for k, b in enumerate(boxes, 1):
        out.append(box_model(f'box_{k:03d}', b['x'], b['y'], b['sx'], b['sy'], b['h'], '0.7 0.4 0.2'))
    sx, sy, syaw = start
    out.append(f'''
    <include>
      <uri>model://iris_with_lidar</uri>
      <name>iris_with_lidar</name>
      <pose>{sx:.4f} {sy:.4f} {SPAWN_Z} 0 0 {syaw:.6f}</pose>
    </include>
  </world>
</sdf>
''')
    return ''.join(out)


def build(name, spec, root):
    lat = spec['lat']
    segs = cut_doors(lat.wall_segments(), spec['doors'])
    walls = [seg_to_rect(s) for s in segs]
    boxes = spec['boxes']
    tall = [b for b in boxes if b['h'] >= LIDAR_PLANE_Z]
    low = [b for b in boxes if b['h'] < LIDAR_PLANE_Z]

    origin = (-MARGIN, -MARGIN)
    shape = (int(round((lat.ny * lat.cell + 2 * MARGIN) / RES)),
             int(round((lat.nx * lat.cell + 2 * MARGIN) / RES)))
    occ = rasterize(walls + [box_rect(b) for b in tall], origin, shape)

    sx, sy, syaw = spec['start']
    si, sj = int((sx - origin[0]) / RES), int((sy - origin[1]) / RES)
    if occ[sj, si]:
        raise RuntimeError(f"{name}: start cell is occupied")

    # geometric reachability (4-connected free cells containing the start)
    lab, _ = ndimage.label(~occ)
    reach_free = lab == lab[sj, si]
    # drone reachability: centres >= ROBOT_R from any occupied cell, connected to start,
    # then everything the drone circle sweeps from those centres
    dist = ndimage.distance_transform_edt(~occ) * RES
    centres_ok = dist >= ROBOT_R
    lab2, _ = ndimage.label(centres_ok)
    if not centres_ok[sj, si]:
        raise RuntimeError(f"{name}: drone does not fit at the start pose")
    centres = lab2 == lab2[sj, si]
    sweep = ndimage.distance_transform_edt(~centres) * RES <= ROBOT_R
    reach_drone = sweep & ~occ
    touches_border = reach_free[0, :].any() or reach_free[-1, :].any() \
        or reach_free[:, 0].any() or reach_free[:, -1].any()
    if touches_border:
        raise RuntimeError(f"{name}: free space leaks to the map border (world not closed)")

    wdir = os.path.join(root, 'worlds')
    mdir = os.path.join(root, 'maps', name)
    os.makedirs(wdir, exist_ok=True)
    os.makedirs(mdir, exist_ok=True)
    with open(os.path.join(wdir, f'{name}.sdf'), 'w') as f:
        f.write(sdf_world(name, walls, boxes, spec['start']))
    write_pgm(os.path.join(mdir, f'{name}.pgm'), mask_occ=occ)
    write_pgm(os.path.join(mdir, 'reachable_free.pgm'), mask_white=reach_free)
    write_pgm(os.path.join(mdir, 'reachable_drone.pgm'), mask_white=reach_drone)
    with open(os.path.join(mdir, f'{name}.yaml'), 'w') as f:
        f.write(f"image: {name}.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    # nearest wall face from the start
    def rect_dist(px, py, r):
        dx = max(r[0] - px, 0, px - r[2])
        dy = max(r[1] - py, 0, py - r[3])
        return math.hypot(dx, dy)
    start_clear = min(rect_dist(sx, sy, r) for r in walls + [box_rect(b) for b in boxes])
    meta = {
        'name': name,
        'layout': spec['layout'],
        'generator': 'drone_sim/tools/make_maze.py',
        'cell_size_m': lat.cell,
        'lattice_cells': [lat.nx, lat.ny],
        'wall_thickness_m': WALL_T, 'wall_height_m': WALL_H,
        'lidar_plane_z_m': LIDAR_PLANE_Z, 'robot_radius_m': ROBOT_R,
        'map': {'resolution': RES, 'origin': list(origin), 'width': shape[1], 'height': shape[0],
                'pgm': f'{name}.pgm', 'yaml': f'{name}.yaml'},
        'start': {'x': sx, 'y': sy, 'z': SPAWN_Z, 'yaw': syaw,
                  'clearance_to_nearest_face_m': round(start_clear, 4)},
        'walls': [[round(v, 4) for v in r] for r in walls],
        'boxes': [dict(b) for b in tall],
        'low_boxes': [dict(b) for b in low],
        'rooms': spec['rooms'],
        'doors': spec['doors'],
        'cells': {'occupied': int(occ.sum()),
                  'reachable_free': int(reach_free.sum()),
                  'reachable_drone': int(reach_drone.sum())},
    }
    with open(os.path.join(mdir, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    return meta


def parse_seeds(text):
    out = []
    for part in text.split(','):
        if '-' in part:
            a, b = part.split('-')
            out += list(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--all', action='store_true', help='generate W0-W8')
    ap.add_argument('--world', action='append', default=[], help='generate one catalogue world by name')
    ap.add_argument('--random', action='store_true', help='generate random worlds wr_s<seed>')
    ap.add_argument('--seeds', default='1-5', help='seeds for --random, e.g. 1-5 or 1,4,9')
    ap.add_argument('--root', default=PKG_DIR, help='drone_sim folder (default: this package)')
    ap.add_argument('--list', action='store_true', help='list catalogue names')
    args = ap.parse_args()

    cat = catalogue()
    if args.list:
        print('\n'.join(cat))
        return 0
    jobs = []
    if args.all:
        jobs += [(n, f) for n, f in cat.items()]
    for n in args.world:
        if n not in cat:
            sys.exit(f"unknown world {n}; use --list")
        jobs.append((n, cat[n]))
    if args.random:
        jobs += [(f'wr_s{s}', (lambda s=s: random_world(s))) for s in parse_seeds(args.seeds)]
    if not jobs:
        ap.print_help()
        return 1
    for name, fn in jobs:
        meta = build(name, fn(), args.root)
        c = meta['cells']
        print(f"{name:20s} walls={len(meta['walls']):3d} boxes={len(meta['boxes'])}+{len(meta['low_boxes'])}low "
              f"rooms={len(meta['rooms'])} map={meta['map']['width']}x{meta['map']['height']} "
              f"reach_free={c['reachable_free']} reach_drone={c['reachable_drone']} "
              f"start_clear={meta['start']['clearance_to_nearest_face_m']:.3f} m")
    return 0


if __name__ == '__main__':
    sys.exit(main())
