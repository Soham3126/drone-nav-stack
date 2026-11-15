"""c_common.py - shared pieces for workstream C tests: starting ROS helper nodes (map_server, planner server,
lifecycle manager, static TF) as child processes, loading fixtures, ground-truth geometry, saving results.

The C tests do not need Gazebo or ArduPilot: they run on map files, so each takes seconds to minutes."""
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'tools'))
from make_fixtures import RES, UNKNOWN, load_drone_reach, load_world, read_pgm  # noqa: E402,F401

FIXTURES = os.path.join(PKG, 'fixtures')
ROBOT_R = 0.27


# ----------------------------------------------------------------------------- fixtures and truth
def fixture_yaml(world, kind):
    return os.path.join(FIXTURES, world, f'{kind}.yaml')


def fixture_values(world, kind):
    """The fixture as map values (-1..100), row 0 = bottom (map coordinates)."""
    img = read_pgm(os.path.join(FIXTURES, world, f'{kind}.pgm')).astype(np.int16)
    return np.where(img == 255, UNKNOWN, img)


class Truth:
    """Ground truth of a world: walls, distance from each cell to the nearest wall face (m)."""

    def __init__(self, world):
        self.meta, self.occ, self.reach = load_world(world)
        self.dreach = load_drone_reach(world)
        self.ox, self.oy = self.meta['map']['origin']
        # distance to the nearest occupied cell centre, minus half a cell = distance to its face
        self.face = ndimage.distance_transform_edt(~self.occ) * RES - RES / 2

    def cell(self, x, y):
        r = int(math.floor((y - self.oy) / RES))
        c = int(math.floor((x - self.ox) / RES))
        return min(max(r, 0), self.occ.shape[0] - 1), min(max(c, 0), self.occ.shape[1] - 1)

    def xy(self, r, c):
        return self.ox + (c + 0.5) * RES, self.oy + (r + 0.5) * RES

    def face_dist(self, x, y, window=1.0):
        """Exact distance (m) from point (x, y) to the nearest occupied map square (its edge, not its centre)."""
        r0, c0 = self.cell(x, y)
        k = int(window / RES) + 1
        rs, cs = slice(max(0, r0 - k), r0 + k + 1), slice(max(0, c0 - k), c0 + k + 1)
        rr, cc = np.nonzero(self.occ[rs, cs])
        if not len(rr):
            return window
        rr, cc = rr + rs.start, cc + cs.start
        x0, y0 = self.ox + cc * RES, self.oy + rr * RES
        dx = np.maximum(np.maximum(x0 - x, 0), x - (x0 + RES))
        dy = np.maximum(np.maximum(y0 - y, 0), y - (y0 + RES))
        return float(np.min(np.hypot(dx, dy)))

    def clearance(self, x, y):
        """Distance from the drone's edge to the nearest wall face (m): >= 0 means no contact."""
        return self.face_dist(x, y) - ROBOT_R

    def is_free(self, x, y):
        r, c = self.cell(x, y)
        return bool(self.reach[r, c] and not self.occ[r, c])


# ----------------------------------------------------------------------------- child processes
class Procs:
    """ROS nodes as child processes; all stopped on exit. Logs go to <out>/<name>.log."""

    def __init__(self, out):
        self.out, self.ps = out, []

    def start(self, name, cmd):
        log = open(os.path.join(self.out, f'{name}.log'), 'w')
        full = 'source /opt/ros/humble/setup.bash; ' \
               '[ -f ~/drone_ws/install/setup.bash ] && source ~/drone_ws/install/setup.bash; exec ' + cmd
        p = subprocess.Popen(['bash', '-c', full], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.ps.append((name, p))
        return p

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        for _, p in self.ps:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(1.5)
        for _, p in self.ps:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def alive(self):
        return [n for n, p in self.ps if p.poll() is None]


def start_map_server(procs, first_yaml):
    procs.start('map_server', f'ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={first_yaml} '
                              f'-p use_sim_time:=false')


def start_lifecycle(procs, nodes):
    names = '[' + ','.join(f"'{n}'" for n in nodes) + ']'
    procs.start('lifecycle', f'ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args '
                             f'-r __node:=lifecycle_manager_c -p autostart:=true -p use_sim_time:=false '
                             f'-p "node_names:={names}"')


def start_static_tf(procs, x=0.0, y=0.0):
    procs.start('static_tf', f'ros2 run tf2_ros static_transform_publisher --x {x} --y {y} --z 0 '
                             f'--yaw 0 --pitch 0 --roll 0 --frame-id map --child-frame-id base_link')


def planner_params(out, overrides=None):
    """drone_nav/config/planner.yaml with use_sim_time off (no simulator clock here) and optional overrides
    {'section.key': value} for the global cost map, e.g. {'lethal_cost_threshold': 100}."""
    import yaml
    src = os.path.join(os.path.dirname(PKG), 'drone_nav', 'config', 'planner.yaml')
    p = yaml.safe_load(open(src))
    p['planner_server']['ros__parameters']['use_sim_time'] = False
    gc = p['global_costmap']['global_costmap']['ros__parameters']
    gc['use_sim_time'] = False
    for k, v in (overrides or {}).items():
        gc[k] = v
    path = os.path.join(out, 'planner_params.yaml')
    yaml.safe_dump(p, open(path, 'w'))
    return path, src


def start_planner(procs, params_yaml):
    procs.start('planner_server', f'ros2 run nav2_planner planner_server --ros-args --params-file {params_yaml}')


# ----------------------------------------------------------------------------- results
def stats(v):
    a = np.asarray(v, float)
    if not len(a):
        return {'n': 0}
    return {'n': int(len(a)), 'median': round(float(np.median(a)), 4), 'p95': round(float(np.percentile(a, 95)), 4),
            'max': round(float(a.max()), 4)}


def save(out_dir, test_id, result, ok, lines):
    result['test'], result['pass'] = test_id, ok
    with open(os.path.join(out_dir, 'result.json'), 'w') as f:
        json.dump(result, f, indent=1, default=str)
    with open(os.path.join(out_dir, 'report.txt'), 'w') as f:
        f.write('\n'.join(lines) + f'\n{test_id}: {"PASS" if ok else "FAIL"}\n')
    for ln in lines:
        print(ln)
    print(f'{test_id}: {"PASS" if ok else "FAIL"}', flush=True)


def tmpdir():
    return tempfile.mkdtemp(prefix='ctest_')
