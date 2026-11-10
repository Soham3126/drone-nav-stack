#!/usr/bin/env python3
"""sweep_table.py - table of replay results per setting (guidebook A1.3 step 3, A1.9).
  python3 sweep_table.py RUN_DIR [RUN_DIR ...]     (the original runs; finds their __replay_* folders)
Per setting: mean and worst over the runs of ATE, max lengthwise error, end error, phantom cells,
and Cartographer CPU (mean % of one core)."""
import glob
import json
import os
import sys

import numpy as np

KEYS = [('ate_rmse_m', 'ATE'), ('lengthwise_max_m', 'lengthwise max'), ('end_error_m', 'end err'),
        ('phantom_cells', 'phantom'), ('precision', 'precision')]


def main():
    runs = sys.argv[1:]
    if not runs:
        print(__doc__)
        return 1
    by = {}
    for r in runs:
        for d in glob.glob(os.path.abspath(r).rstrip('/') + '__replay_*'):
            p = os.path.join(d, 'metrics.json')
            if os.path.isfile(p):
                by.setdefault(d.rsplit('__replay_', 1)[1], []).append(json.load(open(p)))
    if not by:
        print('no replay results found')
        return 1
    hdr = f"{'setting':10s} {'n':>2s} " + ' '.join(f'{lab + " mean/worst":>24s}' for _, lab in KEYS) + f" {'carto CPU %':>11s}"
    print(hdr)
    for name in sorted(by, key=lambda n: (n != 'base', n)):
        ms = by[name]
        cells = []
        for k, _ in KEYS:
            v = np.array([m[k] for m in ms if m.get(k) is not None], float)
            if not len(v):
                cells.append(f"{'-':>24s}")
                continue
            worst = v.min() if k == 'precision' else v.max()
            cells.append(f"{v.mean():>11.4f} / {worst:<10.4f}")
        cpu = [m['cpu']['cartographer']['cpu_mean'] for m in ms if 'cartographer' in m.get('cpu', {})]
        print(f"{name:10s} {len(ms):2d} " + ' '.join(cells) + f" {np.mean(cpu) if cpu else float('nan'):>11.1f}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
