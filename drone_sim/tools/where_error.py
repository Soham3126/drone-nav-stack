#!/usr/bin/env python3
"""Show WHERE in the maze the SLAM position error happened during one flight.

  python3 where_error.py RUN_DIR [RUN_DIR ...]

For each run folder (needs RUN_DIR/eval_data.npz, written by eval_run.py) this saves
RUN_DIR/where_error.png:
  left  - maze walls with the true flight path coloured by position error (cm)
  right - position error and heading error over time
and prints the worst moments: time, true position, error, heading error, and how long
the drone had been flying straight (and in which corridor) when it happened.
"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_run  # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def world_of(run_dir):
    name = os.path.basename(os.path.normpath(run_dir))
    for w in sorted(os.listdir(os.path.join(PKG, 'maps')), key=len, reverse=True):
        if w in name:
            return w
    raise SystemExit(f'cannot tell the world from folder name {name}')


def straight_run(tx, ty, k):
    """metres flown in a straight line (same axis) up to sample k"""
    dist, axis = 0.0, None
    for i in range(k, 0, -1):
        dx, dy = tx[i] - tx[i - 1], ty[i] - ty[i - 1]
        step = np.hypot(dx, dy)
        if step < 1e-4:
            continue
        a = 'E-W' if abs(dx) > abs(dy) else 'N-S'
        if axis is None:
            axis = a
        if a != axis:
            break
        dist += step
    return dist, axis


def one(run_dir):
    world = world_of(run_dir)
    data = eval_run.extract(run_dir)
    m, s = eval_run.compute(data, world, run_dir)
    if 'err' not in s:
        print(run_dir, ': no airborne data'); return
    t, err, hdg, tx, ty = s['t10'] - s['t10'][0], s['err'] * 100, s['hdg'], s['tx'], s['ty']
    meta = json.load(open(os.path.join(PKG, 'maps', world, 'meta.json')))

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(15, 6.5), gridspec_kw={'width_ratios': [1, 1.2]})
    for r in meta['rooms']:
        a1.add_patch(Rectangle((r['x0'], r['y0']), r['x1'] - r['x0'], r['y1'] - r['y0'], color='#fde7c8', lw=0))
    for w in meta['walls']:
        a1.add_patch(Rectangle((w[0], w[1]), w[2] - w[0], w[3] - w[1], color='#1f2937', lw=0))
    sc = a1.scatter(tx, ty, c=err, s=6, cmap='turbo', vmin=0, vmax=max(15, err.max()))
    k = int(np.argmax(err))
    a1.plot(tx[k], ty[k], 'o', ms=14, mfc='none', mec='red', mew=2)
    fig.colorbar(sc, ax=a1, label='position error (cm)')
    a1.set_aspect('equal'); a1.autoscale_view(); a1.set_xlabel('east (m)'); a1.set_ylabel('north (m)')
    a1.set_title(f'{world}: error along the true path (red ring = worst)')

    a2.plot(t, err, color='#2563eb', label='position error (cm)')
    a2.axhline(10, color='#dc2626', ls='--', lw=1, label='10 cm')
    a2.set_xlabel('flight time (s)'); a2.set_ylabel('cm'); a2.grid(alpha=0.3)
    b = a2.twinx(); b.plot(t, hdg, color='#f59e0b', lw=0.8, label='heading error (deg)'); b.set_ylabel('deg')
    h1, l1 = a2.get_legend_handles_labels(); h2, l2 = b.get_legend_handles_labels()
    a2.legend(h1 + h2, l1 + l2, loc='upper left', fontsize=8)
    a2.set_title(f"ATE {m.get('ate_rmse_m', 0)*100:.1f} cm rms, max {m.get('ate_max_m', 0)*100:.1f} cm")
    fig.tight_layout()
    out = os.path.join(run_dir, 'where_error.png'); fig.savefig(out, dpi=120); plt.close(fig)

    print(f'=== {os.path.basename(os.path.normpath(run_dir))}   -> {out}')
    print('   time_s   x_m    y_m   err_cm  heading_deg  straight_before_m  corridor')
    shown = []
    for i in np.argsort(-err):
        if any(abs(t[i] - t[j]) < 10 for j in shown):
            continue
        shown.append(i)
        d, ax = straight_run(tx, ty, i)
        print(f'  {t[i]:7.1f} {tx[i]:6.2f} {ty[i]:6.2f} {err[i]:7.1f} {hdg[i]:11.2f} {d:17.1f}  {ax}')
        if len(shown) == 5:
            break
    above = err > 10
    if above.any():
        print(f'  error above 10 cm for {above.mean() * 100:.0f}% of the flight, '
              f'first at {t[np.argmax(above)]:.0f} s')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    for d in sys.argv[1:]:
        one(d)
