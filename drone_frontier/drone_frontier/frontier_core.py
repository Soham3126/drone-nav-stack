"""frontier_core.py - frontier search on an occupancy grid (guidebook 2.7). No ROS: unit-tested on its own.

Same algorithm as explore_lite (m-explore-ros2), written with numpy/scipy so a 300 x 300 map takes milliseconds
in Python:
  1. convert the map to exact free / wall / unknown (the integration trap: Cartographer's free cells are not 0)
  2. start from the free cell nearest the drone; keep only free cells 4-connected to it (reachable space)
  3. frontier cell = unknown cell with at least one reachable free 4-neighbour
  4. 8-connected frontier cells form one frontier: size, centroid, middle cell, nearest cell to the drone
  5. drop frontiers shorter than min_frontier_size
  6. cost = potential_scale x distance - gain_scale x size; cheapest first
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy import ndimage

FREE, WALL, UNKNOWN = 0, 100, -1
_N4 = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], bool)
_N8 = np.ones((3, 3), bool)


@dataclass
class Params:
    free_threshold: int = 65          # map value below which an OBSERVED cell is free (see convert())
    occupied_threshold: int = 65      # map value from which a cell is a wall
    min_frontier_size: float = 0.55   # m: ignore openings the drone cannot pass
    potential_scale: float = 3.0      # weight of distance
    gain_scale: float = 1.0           # weight of size


@dataclass
class Frontier:
    centroid: tuple
    middle: tuple
    nearest: tuple
    size_m: float
    distance_m: float
    cost: float
    cells: int


def convert(data, p):
    """Map values -> exact FREE / WALL / UNKNOWN. -1 stays unknown; >= occupied_threshold -> wall;
    < free_threshold -> free; anything in between -> unknown."""
    d = np.asarray(data)
    out = np.full(d.shape, UNKNOWN, np.int8)
    known = d >= 0
    out[known & (d < p.free_threshold)] = FREE
    out[known & (d >= p.occupied_threshold)] = WALL
    return out


def search(data, res, origin, robot_xy, p=Params()):
    """data: 2-D array (rows = y from the map origin up), values -1..100. Returns (frontiers sorted, info)."""
    grid = convert(data, p)
    free = grid == FREE
    h, w = grid.shape
    ox, oy = origin
    rx, ry = robot_xy
    rc = int(math.floor((rx - ox) / res)), int(math.floor((ry - oy) / res))     # (col, row)
    info = {'start_cell': None, 'reachable_free': 0}
    if not free.any():
        return [], info
    # 1-2. the free cell nearest the drone, then everything 4-connected to it
    if 0 <= rc[1] < h and 0 <= rc[0] < w and free[rc[1], rc[0]]:
        sr, sc = rc[1], rc[0]
    else:
        _, (ir, ic) = ndimage.distance_transform_edt(~free, return_indices=True)
        r0, c0 = min(max(rc[1], 0), h - 1), min(max(rc[0], 0), w - 1)
        sr, sc = int(ir[r0, c0]), int(ic[r0, c0])
    lab, _ = ndimage.label(free, structure=_N4)
    reach = lab == lab[sr, sc]
    info.update(start_cell=(sr, sc), reachable_free=int(reach.sum()))
    # 3. unknown cells with a reachable free 4-neighbour
    fcell = (grid == UNKNOWN) & ndimage.binary_dilation(reach, structure=_N4)
    # 4. 8-connected groups
    flab, n = ndimage.label(fcell, structure=_N8)
    if n == 0:
        return [], info
    rr, cc = np.nonzero(flab)
    ids = flab[rr, cc]
    xs, ys = ox + (cc + 0.5) * res, oy + (rr + 0.5) * res
    dist = np.hypot(xs - rx, ys - ry)
    order = np.argsort(ids, kind='stable')
    ids, xs, ys, dist = ids[order], xs[order], ys[order], dist[order]
    bounds = np.flatnonzero(np.diff(ids)) + 1
    out = []
    for gx, gy, gd in zip(np.split(xs, bounds), np.split(ys, bounds), np.split(dist, bounds)):
        size = len(gx) * res
        if size < p.min_frontier_size:                          # 5.
            continue
        cx, cy = float(gx.mean()), float(gy.mean())
        k = int(np.argmin(np.hypot(gx - cx, gy - cy)))         # middle: the frontier cell closest to the centroid
        j = int(np.argmin(gd))
        d = float(gd[j])
        out.append(Frontier(centroid=(cx, cy), middle=(float(gx[k]), float(gy[k])),
                            nearest=(float(gx[j]), float(gy[j])), size_m=size, distance_m=d,
                            cost=p.potential_scale * d - p.gain_scale * size, cells=len(gx)))
    out.sort(key=lambda f: f.cost)                              # 6.
    return out, info
