"""geom.py - small 2D geometry helpers shared by make_patterns.py and the tests.

Rectangles are axis-aligned [x0, y0, x1, y1] in the Gazebo world frame (metres).
"""
import json
import math
import os

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOT_R = 0.27


def load_meta(world):
    with open(os.path.join(PKG_DIR, 'maps', world, 'meta.json')) as f:
        return json.load(f)


def box_rect(b):
    return [b['x'] - b['sx'] / 2, b['y'] - b['sy'] / 2, b['x'] + b['sx'] / 2, b['y'] + b['sy'] / 2]


def obstacle_rects(meta, flight=True):
    """Walls + boxes. flight=True: boxes lower than the flight height are left out
    (the drone flies over them); flight=False: every box (on the ground)."""
    rects = [list(w) for w in meta['walls']]
    rects += [box_rect(b) for b in meta['boxes']]
    if not flight:
        rects += [box_rect(b) for b in meta.get('low_boxes', [])]
    return rects


def point_rect_dist(px, py, r):
    dx = max(r[0] - px, 0.0, px - r[2])
    dy = max(r[1] - py, 0.0, py - r[3])
    return math.hypot(dx, dy)


def point_seg_dist(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def seg_hits_rect(ax, ay, bx, by, r):
    """Liang-Barsky: does segment A-B touch rectangle r?"""
    t0, t1 = 0.0, 1.0
    dx, dy = bx - ax, by - ay
    for p, q in ((-dx, ax - r[0]), (dx, r[2] - ax), (-dy, ay - r[1]), (dy, r[3] - ay)):
        if p == 0:
            if q < 0:
                return False
        else:
            t = q / p
            if p < 0:
                t0 = max(t0, t)
            else:
                t1 = min(t1, t)
            if t0 > t1:
                return False
    return True


def seg_rect_dist(ax, ay, bx, by, r):
    if seg_hits_rect(ax, ay, bx, by, r):
        return 0.0
    corners = ((r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3]))
    d = min(point_rect_dist(ax, ay, r), point_rect_dist(bx, by, r))
    for cx, cy in corners:
        d = min(d, point_seg_dist(cx, cy, ax, ay, bx, by))
    return d


def path_clearance(points, rects):
    """Smallest distance from any segment of the polyline to any rectangle face."""
    best = math.inf
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        for r in rects:
            best = min(best, seg_rect_dist(ax, ay, bx, by, r))
    if len(points) == 1:
        best = min(point_rect_dist(points[0][0], points[0][1], r) for r in rects)
    return best


def clearance_at(px, py, rects):
    return min(point_rect_dist(px, py, r) for r in rects)
