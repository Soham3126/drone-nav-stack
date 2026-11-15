#!/usr/bin/env python3
"""C0 Map fixtures (guidebook 4.5) - no simulator needed.

    scripts/run_c.sh C0                (about 1 minute)

Fixtures are made beforehand by drone_sim/tools/make_fixtures.py (full, partial, carto-like + openings.json for
W2-W8 and wr_s1-20). This test:
  1. re-checks the value rules on the files (make_fixtures.py --check): full/partial only -1/0/100 and agreeing
     with the true map; carto-like free 0-30, walls 60-95, wall halo 31-59, 1% speckle; openings.json present
  2. loads every fixture into Nav2's map_server (its /map_server/load_map service) and compares the map it
     serves cell by cell with the file - this proves map_server reads our "raw" files as intended
Pass (book): all fixtures load; value distributions as specified; each partial fixture lists its openings.
"recorded" fixtures (final maps from workstream A's recordings) are not part of this run (book: "once available").
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c_common import (FIXTURES, Procs, fixture_values, fixture_yaml, save, start_lifecycle,  # noqa: E402
                      start_map_server)

KINDS = ('full', 'partial', 'carto')


def main():
    import rclpy
    from nav2_msgs.srv import LoadMap
    from make_fixtures import DEFAULT, check

    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--worlds', nargs='*', default=DEFAULT)
    a, _ = ap.parse_known_args()
    lines, res = [], {'worlds': {}}
    worlds = [w for w in a.worlds if os.path.isdir(os.path.join(FIXTURES, w))]
    missing = sorted(set(a.worlds) - set(worlds))
    if missing:
        lines.append(f'FAIL fixtures missing for: {", ".join(missing)} (run make_fixtures.py)')

    # 1. value rules on the files
    rule_bad = 0
    for w in worlds:
        p = check(w, os.path.join(FIXTURES, w))
        res['worlds'][w] = {'rules': p or 'ok'}
        if p:
            rule_bad += 1
            lines.append(f'FAIL {w}: ' + '; '.join(p))
    lines.append(f"{'ok  ' if not rule_bad else 'FAIL'} value rules: {len(worlds) - rule_bad} of {len(worlds)} worlds")

    # 2. load each fixture through map_server and compare
    rclpy.init()
    node = rclpy.create_node('c0_loader')
    load_bad, n_loaded, times = 0, 0, []
    with Procs(a.out) as procs:
        start_map_server(procs, fixture_yaml(worlds[0], 'full'))
        start_lifecycle(procs, ['map_server'])
        cli = node.create_client(LoadMap, '/map_server/load_map')
        if not cli.wait_for_service(timeout_sec=30.0):
            lines.append('FAIL map_server did not come up (see map_server.log, lifecycle.log)')
        else:
            time.sleep(1.0)
            for w in worlds:
                for k in KINDS:
                    req = LoadMap.Request()
                    req.map_url = fixture_yaml(w, k)
                    t0 = time.time()
                    fut = cli.call_async(req)
                    rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
                    times.append(time.time() - t0)
                    r = fut.result()
                    key = f'{w}/{k}'
                    if r is None or r.result != 0:
                        load_bad += 1
                        lines.append(f'FAIL {key}: map_server could not load it '
                                     f'(result {None if r is None else r.result})')
                        continue
                    m = r.map
                    got = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
                    want = fixture_values(w, k)
                    same = got.shape == want.shape and np.array_equal(got, want)
                    n_loaded += 1
                    res['worlds'][w][k] = {'loaded': True, 'identical': bool(same),
                                           'size': [int(m.info.width), int(m.info.height)],
                                           'resolution': round(m.info.resolution, 4)}
                    if not same:
                        load_bad += 1
                        diff = int((got != want).sum()) if got.shape == want.shape else 'shape'
                        lines.append(f'FAIL {key}: served map differs from the file ({diff} cells)')
    node.destroy_node()
    rclpy.shutdown()
    total = len(worlds) * len(KINDS)
    lines.append(f"{'ok  ' if load_bad == 0 and n_loaded == total else 'FAIL'} map_server: {n_loaded} of {total} "
                 f"fixtures loaded and identical cell by cell (load time median {np.median(times):.2f} s)"
                 if times else 'FAIL map_server: nothing loaded')
    res.update(worlds_n=len(worlds), loaded=n_loaded, total=total, rule_failures=rule_bad, load_failures=load_bad)
    ok = not missing and rule_bad == 0 and load_bad == 0 and n_loaded == total
    save(a.out, 'C0', res, ok, lines)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
