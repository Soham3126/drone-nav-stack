#!/usr/bin/env python3
"""B4 Frame-offset tracking (guidebook 4.4) - W2, truth profile, pose through pose_mangler.py.

One scenario per run, chosen by the MANGLE yaml (drone_sim/tests/b4_cases): none; 0.3 m step in x with
reset_counter unchanged; the same with reset_counter incremented; 5 deg yaw step. Run all with
scripts/run_b4.sh.

Procedure: take off at the start, fly to the crossroads centre (world 5.5, 3.5) and hover 20 s
(baseline), apply the scenario (service /pose_mangler/step_now), record 30 s, land.
Recorded: ground truth, the flight controller's position and heading (spare port), the bridge's frame
offset. Plot b4_step.png in the run folder.
Pass: undisturbed offset < 0.02 m (baseline of every run); behaviour under steps documented (no pass/fail).
Summary: python3 drone_sim/tests/B4.py --summary
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

BASE_S, AFTER_S = 20, 30
JUNCTION_MAP = (5.0, 0.0)          # crossroads (world 5.5, 3.5) in the map frame (start at world 0.5, 3.5)


def scenario():
    path = os.environ.get('MANGLE', 'none')
    if path in ('', 'none'):
        return 'none', {}
    import yaml
    with open(os.path.expanduser(path)) as f:
        return os.path.splitext(os.path.basename(path))[0], (yaml.safe_load(f) or {})


def mean_xy(rows, i=1, j=2):
    return (sum(r[i] for r in rows) / len(rows), sum(r[j] for r in rows) / len(rows)) if rows else (0.0, 0.0)


def run(out):
    from b_common import Client, FcRecorder, call_trigger, save
    name, cfg = scenario()
    stepping = any(float(cfg.get(k, 0) or 0) != 0 for k in ('step_x', 'step_y', 'step_yaw_deg'))
    res = dict(scenario=name, config=cfg)
    lines = [f'scenario {name}: {cfg}']
    with Client() as cl, FcRecorder() as fc:
        ok, msg = cl.take_off(1.1)
        lines.append(msg)
        if not ok:
            cl.command('land')
            save(out, 'B4', res, False, lines)
            return 1
        arrived, secs, _, _ = cl.follow([(0.0, 0.0), JUNCTION_MAP], 60)
        lines.append(f'flew to the crossroads: arrived {arrived} in {secs:.1f} s')
        time.sleep(3)
        t_b0 = time.time()
        time.sleep(BASE_S)
        t_step = time.time()
        if stepping:
            okt, mt = call_trigger(cl, '/pose_mangler/step_now')
            lines.append(f'step applied: {okt} {mt}')
        time.sleep(AFTER_S)
        t_end = time.time()
        okl, ml, _ = cl.command('land')
        lines.append(f'land: {ml}')
        tr = cl.truth_between(t_b0, t_end)
        st = [(t, m) for t, m in cl.status if t_b0 <= t <= t_end]
        fcr = fc.between(t_b0, t_end)

    base_tr = [r for r in tr if r[0] < t_step]
    after_tr = [r for r in tr if r[0] >= t_step]
    bx, by = mean_xy(base_tr)
    base_off = [m.frame_offset_m for t, m in st if t < t_step and not math.isnan(m.frame_offset_m)]
    after_off = [(t - t_step, m.frame_offset_m, m.frame_offset_deg) for t, m in st if t >= t_step]
    res['baseline_offset_max_m'] = round(max(base_off), 4) if base_off else None
    disp = [(r[0] - t_step, math.hypot(r[1] - bx, r[2] - by), r[1] - bx, r[2] - by) for r in after_tr]
    if disp:
        res['moved_max_m'] = round(max(d[1] for d in disp), 3)
        last = [d for d in disp if d[0] > AFTER_S - 5]
        res['moved_final_xy_m'] = [round(sum(d[2] for d in last) / len(last), 3),
                                   round(sum(d[3] for d in last) / len(last), 3)] if last else None
    if fcr:
        fb = [r for r in fcr if t_step - 1.0 <= r[0] < t_step]
        fa = [r for r in fcr if t_step + 0.5 <= r[0] < t_step + 1.5]
        if fb and fa:
            (n0, e0), (n1, e1) = mean_xy(fb), mean_xy(fa)
            res['fc_jump_1s_m'] = round(math.hypot(n1 - n0, e1 - e0), 3)
    if after_off:
        res['offset_peak_m'] = round(max(o[1] for o in after_off), 3)
        res['offset_peak_deg'] = round(max(o[2] for o in after_off), 2)
        settle = None
        for i, (t, m, d) in enumerate(after_off):
            if all(x[1] < 0.02 for x in after_off[i:]):
                settle = t
                break
        res['offset_back_below_2cm_s'] = round(settle, 1) if settle is not None else None
    ok = res['baseline_offset_max_m'] is not None and res['baseline_offset_max_m'] < 0.02
    lines.append(f"baseline (20 s hover) frame offset max {res['baseline_offset_max_m']*100:.1f} cm (needs < 2)"
                 if res['baseline_offset_max_m'] is not None else 'no baseline offset')
    if stepping:
        lines.append(f"after the step: drone moved up to {res.get('moved_max_m', float('nan'))*100:.0f} cm, ends at "
                     f"{res.get('moved_final_xy_m')} m from where it hovered; flight controller's position jumped "
                     f"{res.get('fc_jump_1s_m', float('nan'))*100:.0f} cm in 1 s; frame offset peak "
                     f"{res.get('offset_peak_m', float('nan'))*100:.0f} cm / {res.get('offset_peak_deg', float('nan')):.1f} deg, "
                     f"back below 2 cm after {res.get('offset_back_below_2cm_s')} s")
    json.dump({'t_step': t_step, 'truth': tr, 'fc': fcr, 'offset': [(t, m.frame_offset_m, m.frame_offset_deg) for t, m in st]},
              open(os.path.join(out, 'b4_series.json'), 'w'))
    plot(out, name, t_step, tr, fcr, st, bx, by)
    save(out, 'B4', res, ok and okl, lines)
    return 0 if ok and okl else 1


def plot(out, name, t_step, tr, fcr, st, bx, by):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:  # noqa: BLE001
        return
    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax[0].plot([r[0] - t_step for r in tr], [r[1] - bx for r in tr], label='true x (from hover mean)')
    ax[0].plot([r[0] - t_step for r in tr], [r[2] - by for r in tr], label='true y')
    if fcr:
        fb = [r for r in fcr if r[0] < t_step] or fcr
        n0, e0 = mean_xy(fb)
        ax[0].plot([r[0] - t_step for r in fcr], [r[1] - n0 for r in fcr], '--', label="FC belief x (north)")
        ax[0].plot([r[0] - t_step for r in fcr], [-(r[2] - e0) for r in fcr], '--', label="FC belief y (-east)")
        ax[2].plot([r[0] - t_step for r in fcr], [math.degrees(r[4]) for r in fcr], label='FC heading (deg)')
    ax[0].set_ylabel('m')
    ax[0].legend(fontsize=8)
    ax[1].plot([t - t_step for t, _ in st], [m.frame_offset_m * 100 for _, m in st], label='frame offset (cm)')
    ax[1].legend(fontsize=8)
    ax[2].plot([t - t_step for t, _ in st], [m.frame_offset_deg for _, m in st], label='frame offset (deg)')
    ax[2].legend(fontsize=8)
    ax[2].set_xlabel('time from the step (s)')
    for a in ax:
        a.axvline(0, color='k', lw=0.8)
        a.grid(alpha=0.3)
    fig.suptitle(f'B4 {name}')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'b4_step.png'), dpi=110)
    plt.close(fig)


def summary(root):
    for f in sorted(glob.glob(os.path.join(os.path.expanduser(root), '*_B4_*', 'result.json'))):
        r = json.load(open(f))
        print(f"{r.get('scenario', '?'):16s} baseline offset {100 * (r.get('baseline_offset_max_m') or float('nan')):5.1f} cm  "
              f"moved max {100 * r.get('moved_max_m', float('nan')):5.0f} cm  final {r.get('moved_final_xy_m')}  "
              f"FC jump {100 * r.get('fc_jump_1s_m', float('nan')):4.0f} cm  offset peak "
              f"{100 * r.get('offset_peak_m', float('nan')):4.0f} cm/{r.get('offset_peak_deg', float('nan')):4.1f} deg  "
              f"back<2cm {r.get('offset_back_below_2cm_s')} s  {'PASS' if r['pass'] else 'FAIL'}")


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
