#!/usr/bin/env python3
"""B7 Optical-flow velocity fusion (guidebook 4.4) - W2, truth profile, pose through pose_mangler.py.

Decides whether EK3_SRC1_VELXY = 5 (flow velocity together with our position) improves the system.
Runs with and without FLOW=1 (sitl_flow.parm: simulated flow sensor, flow velocity into EKF3):
    MANGLE=drone_sim/tests/b7_slip.yaml FLOW=1 scripts/run_test.sh B7 w2_turns_junctions truth 900
All 6 runs (3 with, 3 without) and the comparison:  scripts/batch.sh b7 "scripts/run_b7.sh"

One flight per run covers the book's four scenarios:
  (a) hover 60 s at the start                    -> hover drift (largest distance from the hover start)
  (b) the B5 straight path, 8 m                  -> largest lateral error
  (c) hold at the end, then the pose slips 0.1 m/s along the corridor for 3 s (as if SLAM slipped)
                                                 -> how far the drone physically follows the false slip (10 s)
  (d) the flow sensor stops mid-hover (range_relay flow:=false) -> largest movement in the next 20 s
Pass (decided in the summary): slip-following reduced >= 30 % with flow; hover drift and lateral error not
worse than +10 %; no instability in (d). Decision recorded either way.
"""
import argparse
import glob
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

START_W = (0.5, 3.5)


def run(out):
    from b_common import Client, call_trigger, dist_to_polyline, near_vertex, save, set_param
    flow = os.environ.get('FLOW', '0') == '1'
    res = dict(flow=flow)
    lines = [f"B7 with flow fusion: {flow}"]
    with Client() as cl:
        if flow:
            res['EK3_SRC1_VELXY'] = set_param('EK3_SRC1_VELXY', 5)
            lines.append(f"EK3_SRC1_VELXY read back: {res['EK3_SRC1_VELXY']}")
        ok, msg = cl.take_off(1.1)
        lines.append(msg)
        if not ok:
            cl.command('land')
            save(out, 'B7', res, False, lines)
            return 1
        time.sleep(3)
        # (a) hover
        t0 = time.time()
        time.sleep(60)
        tr = cl.truth_between(t0, time.time())
        res['a_hover_drift_m'] = round(max(math.hypot(p[1] - tr[0][1], p[2] - tr[0][2]) for p in tr), 4)
        # (b) straight path
        arrived, secs, t1, t2 = cl.follow([(0.0, 0.0), (8.0, 0.0)], 60)
        pw = [(START_W[0], START_W[1]), (START_W[0] + 8.0, START_W[1])]
        lat = [dist_to_polyline(p[1], p[2], pw)[0] for p in cl.truth_between(t1, t2)
               if not near_vertex(p[1], p[2], pw, 0.0)]
        res['b_lateral_max_m'] = round(max(lat), 4) if lat else None
        res['b_time_s'] = round(secs, 1)
        # (c) synthetic slip while holding at the end
        time.sleep(10)
        base = cl.truth_between(time.time() - 3, time.time())
        bx = sum(p[1] for p in base) / len(base)
        okd, md = call_trigger(cl, '/pose_mangler/drift_now')
        lines.append(f'slip started: {okd} {md}')
        t3 = time.time()
        time.sleep(10)
        slip = cl.truth_between(t3, time.time())
        res['c_slip_follow_m'] = round(max(abs(p[1] - bx) for p in slip), 4) if slip else None
        # (d) flow loss mid-hover
        time.sleep(5)
        t4 = time.time()
        import subprocess
        r = subprocess.run(['bash', '-c', 'source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; '
                            'ros2 param set /range_relay flow false'], capture_output=True, text=True, timeout=60)
        res['d_flow_switched_off'] = r.stdout.strip() or r.stderr.strip()
        time.sleep(20)
        dl = cl.truth_between(t4, time.time())
        res['d_move_after_flow_loss_m'] = round(max(math.hypot(p[1] - dl[0][1], p[2] - dl[0][2]) for p in dl), 4)
        modes = {m.mode for t, m in cl.status if t >= t4}
        res['d_modes'] = sorted(modes)
        okl, ml, _ = cl.command('land')
        lines.append(f'land: {ml}')
    lines.append(f"(a) hover drift {res['a_hover_drift_m']*100:.1f} cm; (b) lateral {res['b_lateral_max_m']*100:.1f} cm "
                 f"in {res['b_time_s']} s; (c) followed the false 0.3 m slip by {res['c_slip_follow_m']*100:.1f} cm; "
                 f"(d) moved {res['d_move_after_flow_loss_m']*100:.1f} cm after flow loss, modes {res['d_modes']}")
    save(out, 'B7', res, bool(okl and arrived), lines)
    return 0 if okl and arrived else 1


def summary(root):
    files = sorted(glob.glob(os.path.join(os.path.expanduser(root), '*_B7_*', 'result.json')))
    if not files:
        print('no B7 runs')
        return
    commit = os.path.basename(os.path.dirname(files[-1])).rsplit('_', 1)[-1]
    files = [f for f in files if os.path.basename(os.path.dirname(f)).endswith('_' + commit)]
    print(f'B7 runs of commit {commit} only ({len(files)} runs)')
    rows = [json.load(open(f)) for f in files]
    rows = [r for r in rows if 'c_slip_follow_m' in r]

    def mean(rs, k):
        v = [r[k] for r in rs if r.get(k) is not None]
        return sum(v) / len(v) if v else float('nan')
    w, wo = [r for r in rows if r['flow']], [r for r in rows if not r['flow']]
    print(f"{'':14s} {'runs':>4s} {'hover drift':>11s} {'lateral':>8s} {'slip follow':>11s} {'after flow loss':>15s}")
    for name, rs in (('without flow', wo), ('with flow', w)):
        print(f"{name:14s} {len(rs):4d} {mean(rs, 'a_hover_drift_m')*100:9.1f}cm {mean(rs, 'b_lateral_max_m')*100:6.1f}cm "
              f"{mean(rs, 'c_slip_follow_m')*100:9.1f}cm {mean(rs, 'd_move_after_flow_loss_m')*100:13.1f}cm")
    if w and wo:
        red = 1 - mean(w, 'c_slip_follow_m') / mean(wo, 'c_slip_follow_m')
        dh = mean(w, 'a_hover_drift_m') / mean(wo, 'a_hover_drift_m') - 1
        dl = mean(w, 'b_lateral_max_m') / mean(wo, 'b_lateral_max_m') - 1
        stable = max(r['d_move_after_flow_loss_m'] for r in w) < 0.3
        ok = red >= 0.30 and dh <= 0.10 and dl <= 0.10 and stable
        print(f"slip-following reduced by {red*100:.0f} % (needs >= 30); hover drift {dh*100:+.0f} %, lateral "
              f"{dl*100:+.0f} % (needs <= +10); flow loss stable: {stable}")
        print(f"B7 decision: {'ADOPT flow fusion (sitl_flow.parm into the default configuration)' if ok else 'do NOT adopt flow fusion'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out')
    ap.add_argument('--world', default='w2_turns_junctions')
    ap.add_argument('--summary', action='store_true')
    ap.add_argument('--root', default='~/drone_results')
    a = ap.parse_args()
    if a.summary:
        summary(a.root)
        return 0
    return run(a.out)


if __name__ == '__main__':
    code = main()
    from b_common import finish
    finish(code)
