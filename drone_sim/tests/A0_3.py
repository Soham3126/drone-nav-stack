#!/usr/bin/env python3
"""Test A0.3 - ground truth and real-time factor (guidebook 4.3).

  scripts/run_test.sh A0.3 w0_box_room gps        (recommended: headless, recorded)
  or, with the simulation already up:  python3 drone_sim/tests/A0_3.py --out DIR

Flies square_3m at 1.1 m and 0.5 m/s with scripted_flight (SITL in GPS mode) while recording
/ground_truth/odom, /truth/pose, /clock and SITL's own LOCAL_POSITION_NED. Checks:
  agreement  ground truth vs SITL position within 0.25 m throughout the flight. Both are
             zeroed on the ground before take-off; ground truth (ENU) is mapped to NED with
             north = +y, east = +x. The three other axis mappings are also scored, so a
             frame mix-up shows up by name.
  zero       /truth/pose starts at (0, 0, yaw 0) within 2 cm / 0.5 deg
  rtf        Gazebo real-time factor (sim seconds per wall second, from /clock) >= 0.95
             (book: headless; with the Gazebo window open expect less - recorded, not failed,
             when the run is not headless)
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import yaml

from sim_common import Collector, rtf, wait_for, world_offset  # noqa: F401 (sets sys.path)
from geom import load_meta  # noqa: E402
from scripted_flight import default_pattern_path, fly_pattern  # noqa: E402

MAPPINGS = {   # name -> (north, east) from world (dx, dy)
    'N=+y, E=+x (expected)': lambda dx, dy: (dy, dx),
    'N=+x, E=-y': lambda dx, dy: (dx, -dy),
    'N=-y, E=-x': lambda dx, dy: (-dy, -dx),
    'N=-x, E=+y': lambda dx, dy: (-dx, dy),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='w0_box_room')
    ap.add_argument('--out', default=os.path.expanduser('~/drone_results/A0_3_manual'))
    ap.add_argument('--speed', type=float, default=0.5)
    ap.add_argument('--url', default='tcp:127.0.0.1:5763')
    args, _ = ap.parse_known_args()
    os.makedirs(args.out, exist_ok=True)
    headless = os.environ.get('HEADLESS', '1') == '1'
    meta = load_meta(args.world)
    pattern = yaml.safe_load(open(default_pattern_path(args.world)))
    sitl = []

    with Collector() as node:
        if not wait_for(lambda: len(node.odom) > 10, 30):
            print('FAIL: no /ground_truth/odom (is sim_up running with profile gps?)')
            return 1
        res = fly_pattern(pattern, url=args.url, speed=args.speed,
                          on_sample=lambda t, n, e, d: sitl.append((t, n, e, d)))
        time.sleep(1)
        with node.lock:
            odom, truth, clock = list(node.odom), list(node.truth), list(node.clock)

    report = {'flight': {k: v for k, v in res.items() if k != 'statustext'}}
    np.savez_compressed(os.path.join(args.out, 'A0_3_data.npz'), odom=np.array(odom),
                        truth=np.array(truth) if truth else np.zeros((0, 5)), sitl=np.array(sitl))

    # --- agreement
    off, frame_kind = world_offset(odom, meta['start'])
    O = np.array(odom)
    S = np.array(sitl)
    d0 = S[0, 3]
    alt = d0 - S[:, 3]
    ground = alt < 0.1
    first_air = np.argmax(alt > 0.2) if (alt > 0.2).any() else len(S)
    g_idx = np.where(ground[:first_air])[0]
    agree = {'odom_frame': frame_kind, 'pass': False}
    if res['ok'] and len(g_idx) > 5:
        t_g = S[g_idx, 0]
        tx = np.interp(t_g, O[:, 0], O[:, 2]).mean()
        ty = np.interp(t_g, O[:, 0], O[:, 3]).mean()
        sn, se = S[g_idx, 1].mean(), S[g_idx, 2].mean()
        air = alt > 0.5
        t_a = S[air, 0]
        dx = np.interp(t_a, O[:, 0], O[:, 2]) - tx
        dy = np.interp(t_a, O[:, 0], O[:, 3]) - ty
        scores = {}
        for name, f in MAPPINGS.items():
            tn, te = f(dx, dy)
            err = np.hypot(S[air, 1] - sn - tn, S[air, 2] - se - te)
            scores[name] = {'max_m': round(float(err.max()), 3), 'rms_m': round(float(np.sqrt((err ** 2).mean())), 3)}
        exp = scores['N=+y, E=+x (expected)']
        best = min(scores, key=lambda k: scores[k]['rms_m'])
        agree.update(samples_airborne=int(air.sum()), max_error_m=exp['max_m'], rms_error_m=exp['rms_m'],
                     best_mapping=best, all_mappings=scores, pass_=exp['max_m'] <= 0.25)
        agree['pass'] = agree.pop('pass_')
    report['agreement'] = agree

    # --- /truth/pose zero
    zero = {'pass': False}
    if truth:
        x, y, z, yaw = truth[0][1:]
        zero = {'first_truth_pose': [round(x, 4), round(y, 4), round(z, 4), round(math.degrees(yaw), 3)],
                'pass': math.hypot(x, y) < 0.02 and abs(math.degrees(yaw)) < 0.5}
    report['truth_zero'] = zero

    # --- real-time factor
    r = rtf(clock)
    report['rtf'] = {'mean': round(r, 3) if r else None, 'headless': headless,
                     'pass': bool(r and r >= 0.95) or (not headless)}
    if not headless:
        report['rtf']['note'] = 'Gazebo window was open: RTF recorded, not judged (book: judge headless)'

    report['pass'] = bool(res['ok'] and agree['pass'] and zero['pass'] and report['rtf']['pass'])
    json.dump(report, open(os.path.join(args.out, 'A0_3_report.json'), 'w'), indent=1)
    print(json.dumps(report, indent=1))
    print(f"\nA0.3: {'PASS' if report['pass'] else 'FAIL'}")
    return 0 if report['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
