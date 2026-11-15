"""c1_cases.py - the C1 goal list, computed from the worlds' true maps (no ROS needed; testable offline).

Each case: world, fixture, start (x, y), goal (x, y), what is expected, and extra data for judging.
All coordinates are in the map frame = world frame."""
import math
import random
from collections import deque

import numpy as np
from scipy import ndimage

from c_common import RES, ROBOT_R, Truth, fixture_values


def lattice_open(t, a, b):
    """Are neighbouring lattice cells a=(ix,iy) and b open to each other (no wall between)? Checks the edge
    midpoint and the points 0.15 m either side of it along the edge: all must be free."""
    cell = t.meta['cell_size_m']
    (ax, ay), (bx, by) = a, b
    mx, my = (ax + bx + 1) * cell / 2, (ay + by + 1) * cell / 2
    pts = [(mx, my)]
    if ax != bx:
        pts += [(mx, my - 0.15), (mx, my + 0.15)]
    else:
        pts += [(mx - 0.15, my), (mx + 0.15, my)]
    return all(not t.occ[t.cell(x, y)] for x, y in pts)


def straight_runs(t):
    """Longest straight corridor run of open lattice cells, horizontal or vertical."""
    nx, ny = t.meta['lattice_cells']
    best = []
    for horiz in (True, False):
        outer, inner = (ny, nx) if horiz else (nx, ny)
        for o in range(outer):
            run = []
            for i in range(inner):
                c = (i, o) if horiz else (o, i)
                if run and lattice_open(t, run[-1], c):
                    run.append(c)
                else:
                    if len(run) > len(best):
                        best = run
                    run = [c]
            if len(run) > len(best):
                best = run
    return best


def centre(t, c):
    cell = t.meta['cell_size_m']
    return ((c[0] + 0.5) * cell, (c[1] + 0.5) * cell)


def geodesic(t, start_xy):
    """Path distance (m) over cells where the drone fits, from start."""
    fits = t.face >= ROBOT_R
    g = np.full(t.occ.shape, np.inf)
    s = t.cell(*start_xy)
    g[s] = 0.0
    q = deque([s])
    while q:
        r, c = q.popleft()
        d = g[r, c] + RES
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < g.shape[0] and 0 <= cc < g.shape[1] and fits[rr, cc] and g[rr, cc] > d:
                g[rr, cc] = d
                q.append((rr, cc))
    return g


def lattice_centres(t):
    cell = t.meta['cell_size_m']
    nx, ny = t.meta['lattice_cells']
    return [((ix + 0.5) * cell, (iy + 0.5) * cell) for iy in range(ny) for ix in range(nx)]


def build_cases():
    cases = []
    # 1. corridors 1.0 / 1.3 / 1.5 m (W6): along the longest straight run; centring judged on its straight part
    for w in ('w6_width_100', 'w6_width_130', 'w6_width_150'):
        t = Truth(w)
        run = straight_runs(t)
        a, b = centre(t, run[0]), centre(t, run[-1])
        cases.append(dict(id=f'corridor_{w[-3:]}', world=w, fixture='full', start=a, goal=b, expect='success',
                          judge='centred', run_axis='x' if a[1] == b[1] else 'y', run_len_m=round(len(run) *
                          t.meta['cell_size_m'], 2), free_width_m=None))
    # 2. W7 gaps: rooms behind 0.45 / 0.60 / 0.90 m gaps, on the full and the carto-like fixture
    t = Truth('w7_gaps')
    s = (t.meta['start']['x'], t.meta['start']['y'])
    for x, gap in ((2.0, '045'), (6.0, '060'), (10.0, '090')):
        for fx in ('full', 'carto'):
            exp = 'fail' if gap == '045' else ('success' if fx == 'full' or gap == '090' else 'record')
            cases.append(dict(id=f'gap_{gap}_{fx}', world='w7_gaps', fixture=fx, start=s, goal=(x, 2.0),
                              expect=exp, judge='reach_goal'))
    # 3. goal in unknown space 0.3 m from known free (partial fixtures). The known free space next to the goal must
    # be space the drone can reach from the start in the partial map (not free space only seen through a gap).
    for w in ('w2_turns_junctions', 'w3_tree_maze', 'w4_maze_rooms', 'w8_arena_full'):
        t = Truth(w)
        part = fixture_values(w, 'partial')
        known_free = part == 0
        d_known = ndimage.distance_transform_edt(~known_free) * RES
        fits_known = known_free & (ndimage.distance_transform_edt(part != 100) * RES >= ROBOT_R + 0.05)
        s = (t.meta['start']['x'], t.meta['start']['y'])
        lab, _ = ndimage.label(fits_known)
        home = lab == lab[t.cell(*s)]
        near_home = ndimage.distance_transform_edt(~home) * RES
        cand = (part < 0) & t.reach & ~t.occ & (np.abs(d_known - 0.30) <= 0.026) & (t.face >= ROBOT_R) \
            & (near_home <= 0.40)
        rr, cc = np.nonzero(cand)
        if len(rr):
            # deterministic: the candidate deepest into an opening = farthest from the start's known space centre
            k = int(np.argmin([near_home[r, c] - 0.001 * math.hypot(*np.subtract(t.xy(r, c), s))
                               for r, c in zip(rr, cc)]))
            g = t.xy(rr[k], cc[k])
            cases.append(dict(id=f'unknown_goal_{w.split("_")[0]}', world=w, fixture='partial', start=s,
                              goal=(round(g[0], 3), round(g[1], 3)), expect='success', judge='near_goal_known_free'))
    # 4. goal inside a wall (W3): middle of the longest interior wall
    t = Truth('w3_tree_maze')
    nx, ny = t.meta['lattice_cells']
    cell = t.meta['cell_size_m']
    inner = [r for r in t.meta['walls'] if r[0] > 0.1 and r[1] > 0.1 and r[2] < nx * cell - 0.1 and
             r[3] < ny * cell - 0.1]
    wr = max(inner, key=lambda r: max(r[2] - r[0], r[3] - r[1]))
    s = (t.meta['start']['x'], t.meta['start']['y'])
    cases.append(dict(id='goal_in_wall', world='w3_tree_maze', fixture='full', start=s,
                      goal=((wr[0] + wr[2]) / 2, (wr[1] + wr[3]) / 2), expect='fail_or_tolerance',
                      judge='near_goal_free'))
    # 5. start inside the inflation (W3): 0.2 m from a wall, goal 3 m away along the maze
    g = geodesic(t, s)
    far = [p for p in lattice_centres(t) if 2.5 <= g[t.cell(*p)] <= 4.0]
    goal = far[0] if far else lattice_centres(t)[-1]
    best, err = None, 1.0
    for dx, dy in ((0, -1), (-1, 0), (0, 1), (1, 0)):
        for k in range(1, 41):                                  # up to 0.4 m from the cell centre
            p = (s[0] + dx * 0.01 * k, s[1] + dy * 0.01 * k)
            e = abs(t.face_dist(*p) - 0.20)
            if e < err:
                best, err = (round(p[0], 3), round(p[1], 3)), e
    cases.append(dict(id='start_in_inflation', world='w3_tree_maze', fixture='full', start=best, goal=goal,
                      expect='record', judge='record'))
    # 6. longest path (W8) + 20 more goals for the timing figure
    t = Truth('w8_arena_full')
    s = (t.meta['start']['x'], t.meta['start']['y'])
    g = geodesic(t, s)
    reach = [p for p in lattice_centres(t) if np.isfinite(g[t.cell(*p)])]
    far = max(reach, key=lambda p: g[t.cell(*p)])
    cases.append(dict(id='longest_w8', world='w8_arena_full', fixture='full', start=s, goal=far, expect='success',
                      judge='reach_goal', path_len_geodesic_m=round(float(g[t.cell(*far)]), 2)))
    rnd = random.Random(8)
    for i, p in enumerate(rnd.sample([q for q in reach if q != far], 20)):
        cases.append(dict(id=f'timing_w8_{i:02d}', world='w8_arena_full', fixture='full', start=s, goal=p,
                          expect='success', judge='reach_goal'))
    return cases


if __name__ == '__main__':
    import json
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    for c in build_cases():
        print(json.dumps({k: (list(v) if isinstance(v, tuple) else v) for k, v in c.items()}))
