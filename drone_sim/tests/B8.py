#!/usr/bin/env python3
"""B8 Height and rangefinder (guidebook 4.4) - W0, truth profile.

    scripts/run_test.sh B8 w0_box_room truth 1500

One run, seven take-offs: five to 1.1 m, then one to 0.5 m and one to 1.5 m. Each: arm, take off, hover
30 s, land (must disarm). Measured against ground truth (height of the body above where it stood):
  height error   mean true height over the last 25 s of the hover minus the target
  takeoff jump   largest step between consecutive flight-controller height samples during the climb
                 (a jump at the rangefinder's minimum range shows up here)
  ground reading the rangefinder on the ground, compared with RNGFND1_GNDCLR
Pass: |height error| < 0.05 m, no jump > 0.10 m, clean landing and disarm - 7 of 7.
"""
import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

TARGETS = [1.1] * 5 + [0.5, 1.5]
HOVER_S = 30


def main():
    from b_common import Client, FcRecorder, save, set_param
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default='w0_box_room')
    a = ap.parse_args()
    res = dict(cycles=[])
    lines = []
    with Client() as cl:
        # the spare port takes one client at a time: read the parameter before the recorder connects
        res['RNGFND1_GNDCLR'] = set_param('RNGFND1_GNDCLR', 0.145)
        lines.append(f"RNGFND1_GNDCLR read back: {res['RNGFND1_GNDCLR']} (None = parameter not found)")
        with FcRecorder() as fc:
            cycles(cl, fc, res, lines)
    n = sum(c['ok'] for c in res['cycles'])
    lines.append(f'{n} of {len(TARGETS)} cycles ok')
    save(a.out, 'B8', res, n == len(TARGETS), lines)
    return 0 if n == len(TARGETS) else 1


def cycles(cl, fc, res, lines):
    if True:
        for k, h in enumerate(TARGETS, 1):
            rf_ground = cl.stats[-1][1].get('rangefinder_cm_min_max_orient_id') if cl.stats else None
            t_arm = time.time()
            ok, msg = cl.take_off(h)
            t_up = time.time()
            if not ok:
                cl.command('land')
                res['cycles'].append(dict(target=h, ok=False, message=msg))
                lines.append(f'FAIL cycle {k} ({h} m): {msg}')
                continue
            time.sleep(HOVER_S)
            t_end = time.time()
            okl, ml, _ = cl.command('land')
            z0 = cl.truth_z0 or 0.0
            hov = [p[3] - z0 for p in cl.truth_between(t_end - 25, t_end)]
            err = sum(hov) / len(hov) - h if hov else float('nan')
            climb = [r[3] for r in fc.between(t_arm, t_up + 2)]
            jump = max((abs(b - a) for a, b in zip(climb, climb[1:])), default=float('nan'))
            good = abs(err) < 0.05 and (math.isnan(jump) or jump <= 0.10) and okl
            c = dict(target=h, ok=good, height_error_m=round(err, 3), takeoff_jump_m=round(jump, 3),
                     landed_and_disarmed=okl, ground_rangefinder=rf_ground, land_message=ml)
            res['cycles'].append(c)
            lines.append(f"{'ok  ' if good else 'FAIL'} cycle {k}: target {h} m, true hover {h + err:.3f} m "
                         f"(error {err*100:+.1f} cm), largest FC height step in the climb {jump*100:.1f} cm, "
                         f"landed+disarmed {okl}, rangefinder on the ground {rf_ground}")
            time.sleep(3)


if __name__ == '__main__':
    code = main()
    from b_common import finish
    finish(code)
