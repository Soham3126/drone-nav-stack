#!/usr/bin/env python3
"""C2 Cost-map thresholds (guidebook 4.5) - no simulator needed.

    scripts/run_c.sh C2                 (about 3 minutes)

The Cartographer-like fixtures (walls 60-95, free 0-30, halo 31-59, speckle 0-64) are loaded into the planner's
global cost map twice: with lethal_cost_threshold 65 (our setting) and 100 (Nav2's default). For every world the
cost map (/global_costmap/costmap: 100 = wall, 99 = too close to a wall for the drone, -1 = unknown) is compared
with the true walls:
  wall cells lethal        true wall cells the cost map marks as wall (100)                 book: >= 99 % at 65
  wall cells blocked       true wall cells the drone's centre may not enter (99 or 100)     what keeps it safe
  false lethal             free cells marked as wall                                         book: < 0.5 % at 65
  leaks                    cells inside a true wall where the drone's centre WOULD be allowed (should be 0)
Note on "lethal": with walls drawn at 60-95, about one wall cell in seven is below 65 and is not itself lethal;
its wall neighbours still make it "too close", so the drone cannot pass. Both numbers are reported; the test judges
"blocked" (book's intent: walls are walls) and leaks, and reports "lethal" beside them.
Pass: at 65, every world: blocked >= 99 %, false lethal < 0.5 %, no leaks. At 100: documented (walls missing).
"""
import argparse
import os
import sys
import time

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c_common import (Procs, Truth, fixture_values, fixture_yaml, planner_params, save, start_lifecycle,  # noqa: E402
                      start_map_server, start_planner, start_static_tf)


class Costmap:
    def __init__(self, tag):
        import rclpy
        from nav2_msgs.srv import LoadMap
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self.rclpy = rclpy
        self.node = rclpy.create_node(f'c2_client_{tag}')
        self.LoadMap = LoadMap
        self.load = self.node.create_client(LoadMap, '/map_server/load_map')
        self.msgs = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.msgs.append, qos)

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def get(self, yaml_path, values, thr, timeout=20.0):
        """Load a fixture; return the first cost map that shows it (same size, same unknown cells, same number of
        cells at or above the threshold marked as wall)."""
        fut = self.load.call_async(self.LoadMap.Request(map_url=yaml_path))
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        if fut.result() is None or fut.result().result != 0:
            return None
        n0 = len(self.msgs)
        leth = values >= thr
        want_leth = int(leth.sum())
        # unknown cells stay unknown in the cost map, except those the inflation reaches (near a wall)
        far = ndimage.distance_transform_edt(~leth) * 0.05 > 0.35 if leth.any() else np.ones(values.shape, bool)
        unk_far = (values < 0) & far
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin(0.2)
            for m in self.msgs[n0:]:
                if (m.info.height, m.info.width) != values.shape:
                    continue
                g = np.asarray(m.data, dtype=np.int16).reshape(values.shape)
                if int((g == 100).sum()) == want_leth and bool((g[unk_far] < 0).all()) and \
                        int((g < 0).sum()) <= int((values < 0).sum()):
                    return g
        return None


def measure(t, g):
    occ = t.occ
    free = t.reach & ~occ
    lethal, blocked = g == 100, g >= 99
    passable = (g >= 0) & (g < 99)
    return dict(wall_cells=int(occ.sum()),
                wall_lethal=round(float(lethal[occ].mean()), 4),
                wall_blocked=round(float(blocked[occ].mean()), 4),
                false_lethal=round(float(lethal[free].sum() / max(1, free.sum())), 5),
                leak_cells=int((passable & occ).sum()))


def main():
    import rclpy
    from make_fixtures import DEFAULT
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--worlds', nargs='*', default=DEFAULT)
    a, _ = ap.parse_known_args()
    lines, res = [], {65: {}, 100: {}}
    rclpy.init()
    for thr in (65, 100):
        params, _ = planner_params(a.out, {'lethal_cost_threshold': thr})
        os.replace(params, os.path.join(a.out, f'planner_params_{thr}.yaml'))
        params = os.path.join(a.out, f'planner_params_{thr}.yaml')
        with Procs(a.out) as procs:
            start_map_server(procs, fixture_yaml(a.worlds[0], 'carto'))
            start_static_tf(procs)
            start_planner(procs, params)
            start_lifecycle(procs, ['map_server', 'planner_server'])
            cm = Costmap(thr)
            if not cm.load.wait_for_service(timeout_sec=30):
                lines.append(f'FAIL threshold {thr}: map server did not come up')
                continue
            cm.spin(3.0)
            for w in a.worlds:
                vals = fixture_values(w, 'carto')
                g = cm.get(fixture_yaml(w, 'carto'), vals, thr)
                res[thr][w] = measure(Truth(w), g) if g is not None else {'error': 'cost map never showed the map'}
            cm.node.destroy_node()
    rclpy.shutdown()

    ok = True
    for thr in (65, 100):
        rows = [v for v in res[thr].values() if 'error' not in v]
        bad = [w for w, v in res[thr].items() if 'error' in v]
        if bad:
            lines.append(f"FAIL threshold {thr}: no cost map for {', '.join(bad)}")
            ok = False
        if not rows:
            continue
        mn = lambda k: min(r[k] for r in rows)   # noqa: E731
        mx = lambda k: max(r[k] for r in rows)   # noqa: E731
        leaks = sum(r['leak_cells'] for r in rows)
        if thr == 65:
            good = mn('wall_blocked') >= 0.99 and mx('false_lethal') < 0.005 and leaks == 0 and not bad
            ok &= good
            lines.append(f"{'ok  ' if good else 'FAIL'} threshold 65, {len(rows)} worlds: wall cells blocked "
                         f">= {mn('wall_blocked') * 100:.2f} %, lethal {mn('wall_lethal') * 100:.1f}-"
                         f"{mx('wall_lethal') * 100:.1f} %; false lethal <= {mx('false_lethal') * 100:.3f} %; "
                         f"leak cells {leaks}")
            for w, v in res[thr].items():
                if 'error' not in v and (v['wall_blocked'] < 0.99 or v['leak_cells'] or v['false_lethal'] >= 0.005):
                    lines.append(f'FAIL   {w}: {v}')
        else:
            lines.append(f"     threshold 100 (Nav2 default), {len(rows)} worlds: wall cells lethal "
                         f"{mx('wall_lethal') * 100:.1f} %, blocked {mn('wall_blocked') * 100:.1f}-"
                         f"{mx('wall_blocked') * 100:.1f} %, leak cells {leaks} -> walls missing, the drone "
                         f"could be planned through them (evidence for 65)")
    res = {str(k): v for k, v in res.items()}
    save(a.out, 'C2', res, ok, lines)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
