#!/usr/bin/env python3
"""eval_run.py - run folder (bag + meta) -> metrics.json, report.txt, plots (guidebook 4.2, "Metrics").

  python3 eval_run.py RUN_DIR [--world W]          evaluate one run (run_test.sh calls this)
  python3 eval_run.py --summary A1.2               table of every run of a test, newest first

Pipeline
  1. extract()  read bag/ (mcap) once -> RUN_DIR/eval_data.npz      (needs ROS; cached)
       truth  : /ground_truth/odom   t_sim, x, y, z, yaw   (Gazebo world)
       est    : /tf map->base_link   t_sim, x, y, yaw      (Cartographer, map frame)
       map    : last /map            + its origin/resolution
       clock  : /clock with the bag's receive time (for the real-time factor)
  2. compute()  pure numpy/scipy: every metric of the table in section 4.2
  3. judge()    the pass criteria of the test named in meta.txt

Frame alignment (book): the map frame is the drone's ground-truth pose at the first estimate
sample (on the ground). Estimates are moved into the Gazebo world with that pose.
Interpretation used for "phantom cluster": 8-connected group of phantom cells whose bounding-box
diagonal is longer than 0.20 m (A1.5's wording); single stray cells are counted, not clustered.
"""
import argparse
import glob
import json
import math
import os
import subprocess
import sys

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from geom import ROBOT_R, load_meta  # noqa: E402

OCC_T = 65                      # occupied: value >= 65 (book). Known-free: 0 <= value < 65, the same rule the
                                # Nav2 static layer uses with trinary_costmap (Cartographer's free cells sit near 40-49)
AIR_DZ = 0.3                    # airborne = truth height > ground + 0.3 m
NEAR_WALL, PHANTOM_D, CLUSTER_LEN = 0.10, 0.20, 0.20
RAY_RANGE, N_RAYS = 8.0, 720

CRITERIA = {
    'A1.1': [('static_drift_m', '<', 0.01), ('static_drift_deg', '<', 0.2), ('phantom_cells', '==', 0)],
    'A1.2': [('ate_rmse_m', '<', 0.05), ('end_error_m', '<', 0.08), ('precision', '>=', 0.95),
             ('phantom_clusters', '==', 0)],
    'A1.3': [('lengthwise_max_m', '<', 0.30), ('end_error_m', '<', 0.15), ('sideways_max_m', '<', 0.05)],
    'A1.4': [('heading_after_turn_max_deg', '<', 1.0), ('ate_rmse_m', '<', 0.08)],
    'A1.5': [('ate_rmse_m', '<', 0.10), ('end_error_m', '<', 0.15), ('precision', '>=', 0.95),
             ('recall', '>=', 0.90), ('phantom_fraction', '<', 0.005), ('phantom_clusters', '==', 0)],
    'A1.6': [('phantom_clusters', '==', 0), ('ate_rmse_m', '<', 0.15)],
}


# ============================================================================ extraction (ROS)
def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def extract(run_dir):
    cache = os.path.join(run_dir, 'eval_data.npz')
    if os.path.isfile(cache):
        return dict(np.load(cache, allow_pickle=True))
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = os.path.join(run_dir, 'bag')
    if not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
        print('bag has no metadata.yaml (recorder stopped hard): reindexing ...')
        subprocess.run(['ros2', 'bag', 'reindex', bag, 'mcap'], check=False)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = [t for t in ('/ground_truth/odom', '/tf', '/map', '/clock') if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    truth, est, clock, last_map, maps_seen = [], [], [], None, 0
    while reader.has_next():
        topic, data, t_rx = reader.read_next()
        m = deserialize_message(data, cls[topic])
        if topic == '/ground_truth/odom':
            p, q = m.pose.pose.position, m.pose.pose.orientation
            truth.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, yaw_q(q)))
        elif topic == '/tf':
            for tr in m.transforms:
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'base_link':
                    t = tr.transform.translation
                    est.append((tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9, t.x, t.y,
                                yaw_q(tr.transform.rotation)))
        elif topic == '/map':
            last_map, maps_seen = m, maps_seen + 1
        elif topic == '/clock':
            clock.append((t_rx * 1e-9, m.clock.sec + m.clock.nanosec * 1e-9))
    out = {'truth': np.array(truth), 'est': np.array(est), 'clock': np.array(clock), 'maps_seen': maps_seen}
    if last_map is not None:
        info = last_map.info
        o = info.origin
        out['map_data'] = np.array(last_map.data, dtype=np.int8).reshape(info.height, info.width)
        out['map_info'] = np.array([info.resolution, o.position.x, o.position.y, yaw_q(o.orientation)])
    np.savez_compressed(cache, **out)
    return out


# ============================================================================ helpers
def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    parts = data.split(maxsplit=4)
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(parts[4][:w * h], dtype=np.uint8).reshape(h, w)[::-1, :]


class GroundTruth:
    """Ground-truth raster of the world (row 0 = bottom), with distance-to-wall lookup."""

    def __init__(self, world):
        self.meta = load_meta(world)
        mdir = os.path.join(PKG, 'maps', world)
        mp = self.meta['map']
        self.res, (self.ox, self.oy) = mp['resolution'], mp['origin']
        self.occ = read_pgm(os.path.join(mdir, mp['pgm'])) < 128
        self.reach = read_pgm(os.path.join(mdir, 'reachable_free.pgm')) > 128
        self.h, self.w = self.occ.shape
        self.dist = ndimage.distance_transform_edt(~self.occ) * self.res

    def idx(self, x, y):
        i = np.floor((np.asarray(x) - self.ox) / self.res).astype(int)
        j = np.floor((np.asarray(y) - self.oy) / self.res).astype(int)
        inside = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        return i, j, inside

    def wall_dist(self, x, y):
        i, j, inside = self.idx(x, y)
        d = np.full(np.shape(x), np.inf)
        d[inside] = self.dist[j[inside], i[inside]]
        return d

    def visible_walls(self, xs, ys):
        """Ground-truth wall cells hit first by 2D rays (<= 8 m) from the given positions."""
        vis = np.zeros_like(self.occ)
        ang = np.linspace(-math.pi, math.pi, N_RAYS, endpoint=False)
        steps = np.arange(1, int(RAY_RANGE / (self.res / 2)) + 1) * (self.res / 2)
        cx, cy = np.cos(ang)[:, None] * steps, np.sin(ang)[:, None] * steps
        for x, y in zip(xs, ys):
            i, j, inside = self.idx(x + cx, y + cy)
            ic, jc = np.clip(i, 0, self.w - 1), np.clip(j, 0, self.h - 1)
            hit = self.occ[jc, ic] & inside
            any_hit = hit.any(axis=1)
            first = hit.argmax(axis=1)
            r = np.where(any_hit)[0]
            vis[jc[r, first[r]], ic[r, first[r]]] = True
        return vis


def map_cells_world(map_data, map_info, align):
    """Centres of map cells in the world frame, with their values."""
    res, mx, my, myaw = map_info
    h, w = map_data.shape
    jj, ii = np.mgrid[0:h, 0:w]
    lx, ly = (ii + 0.5) * res, (jj + 0.5) * res
    px = mx + math.cos(myaw) * lx - math.sin(myaw) * ly          # map frame
    py = my + math.sin(myaw) * lx + math.cos(myaw) * ly
    x0, y0, yaw0 = align
    wx = x0 + math.cos(yaw0) * px - math.sin(yaw0) * py          # world frame
    wy = y0 + math.sin(yaw0) * px + math.cos(yaw0) * py
    return wx, wy


def to_world(est, align):
    x0, y0, yaw0 = align
    c, s = math.cos(yaw0), math.sin(yaw0)
    return (x0 + c * est[:, 1] - s * est[:, 2], y0 + s * est[:, 1] + c * est[:, 2],
            np.unwrap(est[:, 3] + yaw0))


def wrap_deg(a):
    return (np.degrees(a) + 180.0) % 360.0 - 180.0


def corridor_axes(world):
    """Direction (unit vector) of the commanded path segment nearest each position, from the pattern."""
    import yaml
    from scripted_flight import default_pattern_path
    try:
        p = yaml.safe_load(open(default_pattern_path(world)))
    except OSError:
        return None
    pts = np.array([p['start'][:2]] + p['waypoints'], dtype=float)
    segs = [(a, b) for a, b in zip(pts, pts[1:]) if np.linalg.norm(b - a) > 1e-6]
    return segs


# ============================================================================ metrics
def compute(data, world, run_dir=None):
    gt = GroundTruth(world)
    T, E = data['truth'], data['est']
    m = {'world': world}
    if len(T) < 10 or len(E) < 10:
        m['error'] = f'not enough data: truth={len(T)} estimate={len(E)} (Cartographer running? bag recorded?)'
        return m, {}
    T = T[np.argsort(T[:, 0])]
    E = E[np.argsort(E[:, 0])]
    _, ui = np.unique(E[:, 0], return_index=True)
    E = E[ui]
    z_ground = float(np.median(T[:50, 3]))
    # --- alignment: ground-truth pose at the first estimate sample
    t_a = max(E[0, 0], T[0, 0])
    yaw_t = np.unwrap(T[:, 4])
    align = (float(np.interp(t_a, T[:, 0], T[:, 1])), float(np.interp(t_a, T[:, 0], T[:, 2])),
             float(np.interp(t_a, T[:, 0], yaw_t)))
    ex, ey, eyaw = to_world(E, align)
    m['alignment'] = {'x': round(align[0], 4), 'y': round(align[1], 4), 'yaw_deg': round(math.degrees(align[2]), 3)}

    def est_at(t):
        return np.interp(t, E[:, 0], ex), np.interp(t, E[:, 0], ey), np.interp(t, E[:, 0], eyaw)

    valid_t = (T[:, 0] >= E[0, 0]) & (T[:, 0] <= E[-1, 0])
    air = (T[:, 3] > z_ground + AIR_DZ) & valid_t
    series = {}
    if air.any():
        # 10 Hz samples while airborne
        t10 = np.arange(T[air, 0][0], T[air, 0][-1], 0.1)
        t10 = t10[np.interp(t10, T[:, 0], T[:, 3]) > z_ground + AIR_DZ]
        tx, ty = np.interp(t10, T[:, 0], T[:, 1]), np.interp(t10, T[:, 0], T[:, 2])
        tyaw = np.interp(t10, T[:, 0], yaw_t)
        qx, qy, qyaw = est_at(t10)
        err = np.hypot(qx - tx, qy - ty)
        m['airborne_s'] = round(float(t10[-1] - t10[0]), 1)
        m['ate_rmse_m'] = round(float(np.sqrt(np.mean(err ** 2))), 4)
        m['ate_max_m'] = round(float(err.max()), 4)
        hdg = wrap_deg(qyaw - tyaw)
        m['heading_err_max_deg'] = round(float(np.abs(hdg).max()), 3)
        series.update(t10=t10, err=err, hdg=hdg, tx=tx, ty=ty, qx=qx, qy=qy)
        # touchdown: first ground sample after the last airborne one
        last_air = np.where(air)[0][-1]
        after = np.where((T[:, 3] < z_ground + 0.05) & (np.arange(len(T)) > last_air) & valid_t)[0]
        if len(after):
            k = after[0]
            px, py, _ = est_at(T[k, 0])
            m['end_error_m'] = round(float(math.hypot(px - T[k, 1], py - T[k, 2])), 4)
        # lengthwise / sideways relative to the nearest commanded segment
        segs = corridor_axes(world)
        if segs:
            P = np.stack([tx, ty], 1)
            best_d = np.full(len(P), np.inf)
            axis = np.zeros((len(P), 2))
            for a, b in segs:
                v = b - a
                L = np.linalg.norm(v)
                u = v / L
                s = np.clip(((P - a) @ u), 0, L)
                d = np.linalg.norm(P - (a + s[:, None] * u), axis=1)
                better = d < best_d
                best_d[better], axis[better] = d[better], u
            e = np.stack([qx - tx, qy - ty], 1)
            lw = np.abs((e * axis).sum(1))
            sw = np.abs(e[:, 0] * axis[:, 1] - e[:, 1] * axis[:, 0])
            m['lengthwise_max_m'] = round(float(lw.max()), 4)
            m['lengthwise_rms_m'] = round(float(np.sqrt(np.mean(lw ** 2))), 4)
            m['sideways_max_m'] = round(float(sw.max()), 4)
            m['sideways_p95_m'] = round(float(np.percentile(sw, 95)), 4)
            series.update(lengthwise=lw, sideways=sw)
        # heading error 1 s after each turn (truth yaw rate > 20 deg/s, then settles)
        rate = np.degrees(np.gradient(yaw_t, T[:, 0]))
        turning = (np.abs(rate) > 20) & air
        turns, k = [], 0
        idx = np.where(turning)[0]
        while k < len(idx):
            start = k
            while k + 1 < len(idx) and T[idx[k + 1], 0] - T[idx[k], 0] < 0.5:
                k += 1
            t_end = T[idx[k], 0]
            t_chk = t_end + 1.0
            if t_chk < T[air, 0][-1]:
                ty1 = np.interp(t_chk, T[:, 0], yaw_t)
                turns.append(round(abs(float(wrap_deg(est_at(t_chk)[2] - ty1))), 3))
            k += 1
            _ = start
        m['turns_detected'] = len(turns)
        if turns:
            m['heading_after_turn_max_deg'] = max(turns)
            m['heading_after_turn_deg'] = turns
    else:
        # static run (A1.1). Cartographer publishes ~200 poses/s: IMU-extrapolated between 10 Hz scan
        # matches, so single samples shake by a few mm. DRIFT = how far the 1 s average moves over
        # seconds 15-60 (what the book's "pose drift" means); JITTER = raw samples around their own
        # 1 s average (reported, not judged).
        w = (E[:, 0] >= E[0, 0] + 15) & (E[:, 0] <= E[0, 0] + 60)
        if w.sum() > 5:
            t, x, y, yaw = E[w, 0], ex[w], ey[w], eyaw[w]
            sec = np.floor(t - t[0]).astype(int)
            n = np.bincount(sec)
            ok = n >= 0.5 * np.median(n[n > 0])     # skip a part-filled last second
            mx = np.bincount(sec, x)[ok] / n[ok]
            my = np.bincount(sec, y)[ok] / n[ok]
            myaw = np.bincount(sec, yaw)[ok] / n[ok]
            m['static_window_s'] = [15, 60]
            m['static_drift_m'] = round(float(np.hypot(mx - mx[0], my - my[0]).max()), 5)
            m['static_drift_deg'] = round(float(np.abs(np.degrees(myaw - myaw[0])).max()), 4)
            full = np.zeros(len(n))
            full[ok] = 1
            jx = x - (np.bincount(sec, x) / np.maximum(n, 1))[sec]
            jy = y - (np.bincount(sec, y) / np.maximum(n, 1))[sec]
            jit = np.hypot(jx, jy)
            m['static_jitter_std_m'] = round(float(np.sqrt((jit ** 2).mean())), 5)
            m['static_jitter_max_m'] = round(float(jit.max()), 5)
            m['static_raw_max_from_start_m'] = round(float(np.hypot(x - x[0], y - y[0]).max()), 5)
            m['pose_rate_hz'] = round(float(w.sum() / (t[-1] - t[0])), 1)

    # --- map metrics
    if 'map_data' in data:
        md, mi = data['map_data'], data['map_info']
        wx, wy = map_cells_world(md, mi, align)
        occ = md >= OCC_T
        free = (md >= 0) & (md < OCC_T)
        d_occ = gt.wall_dist(wx[occ], wy[occ])
        n_occ = int(occ.sum())
        m['map_occupied_cells'] = n_occ
        m['maps_seen'] = int(data.get('maps_seen', 0))
        if n_occ:
            m['precision'] = round(float((d_occ <= NEAR_WALL).mean()), 4)
            ph = np.zeros_like(occ)
            ph[occ] = d_occ > PHANTOM_D
            m['phantom_cells'] = int(ph.sum())
            m['phantom_fraction'] = round(m['phantom_cells'] / n_occ, 5)
            lab, n = ndimage.label(ph, structure=np.ones((3, 3)))
            clusters = []
            for sl in ndimage.find_objects(lab):
                ext = math.hypot((sl[0].stop - sl[0].start), (sl[1].stop - sl[1].start)) * mi[0]
                if ext > CLUSTER_LEN:
                    cy_, cx_ = (sl[0].start + sl[0].stop) // 2, (sl[1].start + sl[1].stop) // 2
                    clusters.append({'length_m': round(ext, 3),
                                     'at': [round(float(wx[cy_, cx_]), 2), round(float(wy[cy_, cx_]), 2)]})
            m['phantom_clusters'] = len(clusters)
            m['phantom_cluster_list'] = sorted(clusters, key=lambda c: -c['length_m'])[:10]
        # recall: visible ground-truth wall cells near an occupied map cell
        pos = np.stack([T[valid_t, 1], T[valid_t, 2]], 1)
        if len(pos):
            keep = [0]
            for k in range(1, len(pos)):
                if np.linalg.norm(pos[k] - pos[keep[-1]]) >= 0.25:
                    keep.append(k)
            vis = gt.visible_walls(pos[keep, 0], pos[keep, 1])
            mapped = np.zeros_like(gt.occ)
            gi, gj, inside = gt.idx(wx[occ], wy[occ])
            mapped[gj[inside], gi[inside]] = True
            d_map = ndimage.distance_transform_edt(~mapped) * gt.res if mapped.any() else np.full(gt.occ.shape, np.inf)
            n_vis = int(vis.sum())
            m['visible_wall_cells'] = n_vis
            if n_vis:
                m['recall'] = round(float((d_map[vis] <= NEAR_WALL).mean()), 4)
        # coverage: known-free cells inside the reachable free space eroded 0.1 m
        target = ndimage.binary_erosion(gt.reach, iterations=int(round(0.1 / gt.res)))
        fmask = np.zeros_like(gt.occ)
        gi, gj, inside = gt.idx(wx[free], wy[free])
        fmask[gj[inside], gi[inside]] = True
        if target.any():
            m['coverage'] = round(float((fmask & target).sum() / target.sum()), 4)
        series.update(map_occ_world=(wx[occ], wy[occ]))

    # --- clearance margin (ground truth) and RTF
    if air.any():
        cm = gt.wall_dist(T[air, 1], T[air, 2]) - ROBOT_R
        m['clearance_margin_min_m'] = round(float(cm.min()), 3)
    C = data.get('clock')
    if C is not None and len(C) > 2 and C[-1, 0] > C[0, 0]:
        m['rtf_mean'] = round(float((C[-1, 1] - C[0, 1]) / (C[-1, 0] - C[0, 0])), 3)
    if run_dir and os.path.isfile(os.path.join(run_dir, 'cpu.csv')):
        m['cpu'] = cpu_summary(os.path.join(run_dir, 'cpu.csv'))
    series.update(T=T, ex=ex, ey=ey, gt=gt)
    return m, series


def cpu_summary(path):
    import csv
    rows = {}
    for r in csv.DictReader(open(path)):
        rows.setdefault(r['label'], []).append((float(r['cpu_percent']), float(r['rss_mb'])))
    out = {}
    for lab, v in rows.items():
        a = np.array(v)
        out[lab] = {'cpu_mean': round(float(a[:, 0].mean()), 1), 'cpu_max': round(float(a[:, 0].max()), 1),
                    'mem_growth_mb': round(float(a[-1, 1] - a[0, 1]), 1)}
    return out


def judge(test_id, m):
    crit = CRITERIA.get(test_id)
    if not crit:
        return None, []
    lines, ok = [], True
    for key, op, lim in crit:
        v = m.get(key)
        if v is None:
            good = False
        elif op == '<':
            good = v < lim
        elif op == '>=':
            good = v >= lim
        else:
            good = v == lim
        ok &= good
        lines.append(f"  {'ok  ' if good else 'FAIL'} {key:28s} {v!s:>10s}  (needs {op} {lim})")
    return ok, lines


def plots(series, m, run_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    gt = series.get('gt')
    fig, ax = plt.subplots(1, 2, figsize=(15, 7))
    if gt is not None:
        ext = [gt.ox, gt.ox + gt.w * gt.res, gt.oy, gt.oy + gt.h * gt.res]
        ax[0].imshow(np.where(gt.occ, 0.0, 1.0), cmap='gray', origin='lower', extent=ext, vmin=0, vmax=1)
    if 'map_occ_world' in series:
        ax[0].plot(*series['map_occ_world'], 'r.', ms=1.5, label='map occupied')
    T = series['T']
    ax[0].plot(T[:, 1], T[:, 2], 'g-', lw=1, label='ground truth')
    ax[0].plot(series['ex'], series['ey'], 'b-', lw=0.8, label='Cartographer')
    ax[0].set_aspect('equal')
    ax[0].legend(loc='upper right', fontsize=8)
    ax[0].set_title(f"{m['world']}  ATE {m.get('ate_rmse_m', '-')} m  precision {m.get('precision', '-')}")
    if 't10' in series:
        t = series['t10'] - series['t10'][0]
        ax[1].plot(t, series['err'], label='position error (m)')
        if 'lengthwise' in series:
            ax[1].plot(t, series['lengthwise'], label='lengthwise (m)', alpha=0.7)
            ax[1].plot(t, series['sideways'], label='sideways (m)', alpha=0.7)
        ax2 = ax[1].twinx()
        ax2.plot(t, series['hdg'], 'k-', lw=0.6, alpha=0.5, label='heading error (deg)')
        ax2.set_ylabel('heading error (deg)')
        ax[1].set_xlabel('airborne time (s)')
        ax[1].legend(loc='upper left', fontsize=8)
    plt.tight_layout()
    plt.savefig(os.path.join(run_dir, 'eval.png'), dpi=110)
    plt.close(fig)


def read_meta_txt(run_dir):
    out = {}
    p = os.path.join(run_dir, 'meta.txt')
    if os.path.isfile(p):
        for line in open(p):
            if '=' in line:
                k, v = line.strip().split('=', 1)
                out[k] = v
    return out


def evaluate(run_dir, world=None):
    meta = read_meta_txt(run_dir)
    world = world or meta.get('world')
    test_id = meta.get('test', '')
    data = extract(run_dir)
    m, series = compute(data, world, run_dir)
    m['test'] = test_id
    m['run'] = os.path.basename(os.path.abspath(run_dir))
    m['params'] = {k: meta[k] for k in ('speed', 'yaw_mode', 'laps', 'git', 'headless') if k in meta}
    ok, lines = judge(test_id, m)
    m['pass'] = ok
    json.dump(m, open(os.path.join(run_dir, 'metrics.json'), 'w'), indent=1)
    if series:
        plots(series, m, run_dir)
    keys = ['ate_rmse_m', 'ate_max_m', 'end_error_m', 'lengthwise_max_m', 'sideways_max_m', 'heading_err_max_deg',
            'heading_after_turn_max_deg', 'turns_detected', 'static_drift_m', 'static_drift_deg', 'static_jitter_std_m',
            'static_jitter_max_m', 'static_raw_max_from_start_m', 'pose_rate_hz', 'precision',
            'recall', 'phantom_cells', 'phantom_fraction', 'phantom_clusters', 'coverage',
            'clearance_margin_min_m', 'rtf_mean']
    print(f"=== {test_id} {world}  {m['run']}")
    if 'error' in m:
        print('ERROR:', m['error'])
    for k in keys:
        if k in m:
            print(f"  {k:28s} {m[k]}")
    if m.get('phantom_cluster_list'):
        print('  phantom clusters:', m['phantom_cluster_list'][:3])
    if 'cpu' in m:
        busy = sorted(m['cpu'].items(), key=lambda kv: -kv[1]['cpu_mean'])[:4]
        print('  cpu % of one core (mean/max):', ', '.join(f"{k} {v['cpu_mean']}/{v['cpu_max']}" for k, v in busy))
    if ok is not None:
        print(f"criteria for {test_id}:")
        print('\n'.join(lines))
        print(f"{test_id}: {'PASS' if ok else 'FAIL'}")
    return 0 if ok in (True, None) else 1


def summary(test_id, root):
    rows = []
    for p in glob.glob(os.path.join(os.path.expanduser(root), f'*_{test_id}_*', 'metrics.json')):
        rows.append(json.load(open(p)))
    rows.sort(key=lambda r: r['run'], reverse=True)
    keys = [k for k, *_ in CRITERIA.get(test_id, [])]
    print(f"{'run':52s} {'speed':>5s} " + ' '.join(f'{k[:14]:>14s}' for k in keys) + '  result')
    for r in rows:
        sp = r.get('params', {}).get('speed', '')
        vals = ' '.join(f"{str(r.get(k, '-'))[:14]:>14s}" for k in keys)
        print(f"{r['run'][:52]:52s} {sp:>5s} {vals}  {'PASS' if r.get('pass') else 'FAIL'}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir', nargs='?')
    ap.add_argument('--world')
    ap.add_argument('--summary', metavar='TEST_ID')
    ap.add_argument('--root', default='~/drone_results')
    args = ap.parse_args()
    if args.summary:
        return summary(args.summary, args.root)
    if not args.run_dir:
        ap.error('give a run folder or --summary TEST_ID')
    return evaluate(args.run_dir, args.world)


if __name__ == '__main__':
    sys.exit(main())
