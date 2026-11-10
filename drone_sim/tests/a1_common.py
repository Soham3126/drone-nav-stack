"""a1_common.py - shared flight runner for the A1 tests. The flight only produces data; the
verdict comes from eval_run.py, which run_test.sh calls after the bag is closed.

Environment overrides (run_test.sh passes the environment through):
  SPEED=1.0   cruise speed m/s (sets WP_SPD)        LAPS=3   repeat the pattern's loop
  YAW=fixed|travel                                  WP_ACC=… extra ArduPilot acceleration (A1.6)
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'tools'))
import yaml  # noqa: E402
from scripted_flight import default_pattern_path, fly_pattern  # noqa: E402


def repeat_laps(p, laps):
    """square_3m waypoints are [start, c1, c2, c3, c4, c1, start]: fly c1..c4 `laps` times,
    then close the square at c1 and return to the start."""
    if laps <= 1:
        return p
    wps = p['waypoints']
    loop = wps[1:-2]           # c1 .. c4
    p = dict(p)
    p['waypoints'] = [wps[0]] + loop * laps + [wps[-2], wps[-1]]
    pts = [p['start'][:2]] + p['waypoints']
    p['path_length_m'] = round(sum(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5 for a, b in zip(pts, pts[1:])), 1)
    return p


def append_meta(out, **kv):
    with open(os.path.join(out, 'meta.txt'), 'a') as f:
        for k, v in kv.items():
            f.write(f'{k}={v}\n')


def run(test_id, default_world, laps=1, yaw='fixed', speed=0.5, extra_params=None, worlds=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default=default_world)
    args, _ = ap.parse_known_args()
    if worlds and args.world not in worlds:
        print(f'{test_id} is defined for {worlds}, not {args.world}')
        return 2
    speed = float(os.environ.get('SPEED', speed))
    laps = int(os.environ.get('LAPS', laps))
    yaw = os.environ.get('YAW', yaw)
    params = dict(extra_params or {})
    if os.environ.get('WP_ACC'):
        params['WP_ACC'] = float(os.environ['WP_ACC'])
    p = repeat_laps(yaml.safe_load(open(default_pattern_path(args.world))), laps)
    append_meta(args.out, speed=speed, laps=laps, yaw_mode=yaw, pattern=p['pattern'],
                extra_params=json.dumps(params))
    print(f"{test_id}: {p['pattern']} in {args.world}, {len(p['waypoints'])} waypoints, "
          f"{p['path_length_m']} m, speed {speed} m/s, heading {yaw}")
    res = fly_pattern(p, speed=speed, yaw_mode=yaw, extra_params=params)
    time.sleep(3)    # let Cartographer publish the final map after landing
    json.dump(res, open(os.path.join(args.out, 'flight_result.json'), 'w'), indent=1)
    print(f"flight {'completed' if res['ok'] else 'FAILED: ' + res.get('error', '')}; "
          f"evaluation follows after the simulation stops")
    return 0 if res['ok'] else 1
