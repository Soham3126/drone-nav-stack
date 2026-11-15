#!/usr/bin/env python3
"""B9 Latency and load (guidebook 4.4) - W2, slam profile (Cartographer carto-v1 flies the drone for the first
time in workstream B), then the same flight again under artificial CPU load (stress-ng --cpu 4).

    scripts/run_test.sh B9 w2_turns_junctions slam 1500        (needs: sudo apt install stress-ng)

Two phases in one run, identical flights: take off to 1.1 m, hover 20 s, fly out along the start corridor and
up the L-turn arm, fly back, land.  Phase 1 without load, phase 2 with stress-ng running.

Measured in each phase, while airborne:
  delay components (ms)
    scan_delivery   /scan arrival minus its timestamp (lidar + Gazebo bridge)
    cartographer    /scan_matched_points2 arrival minus the scan's timestamp (scan matching)
    tf_age          map->base_link arrival minus its timestamp (Cartographer publishes extrapolated poses)
    bridge_age      pose age when the bridge sends VISION_POSITION_ESTIMATE (bridge /bridge/link_stats)
    effective       the time shift between the pose stream as received and ground truth (velocity match,
                    as A1.10) - what ArduPilot effectively sees, before the bridge
    total           effective + (bridge_age - tf_age): the delay to put in VISO_DELAY_MS
  message rates     in simulation time (Hz): /scan, map->base_link TF, everything the bridge receives from
                    and sends to ArduPilot. Simulation time, because under load Gazebo may run slower than
                    real time (lockstep): wall-clock rates would then drop although nothing is starved.
  bridge CPU        % of one core (the bridge measures its own process time)
  real-time factor  timing results are valid only with RTF >= 0.9 (book 4.2, note 4)
  flight controller its LOCAL_POSITION_NED against ground truth (error, and its own delay)
Pass (book): every rate under load within 10% of the unloaded rate; bridge below 10% of one core; both flights
completed; RTF >= 0.9.  The suggested VISO_DELAY_MS (phase 1 total, rounded to 10 ms) is then committed.
"""
import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

START_W = (0.5, 3.5)                      # W2 start (map frame = drone start, heading +x), as B5
OUT_PATH = [(0.0, 0.0), (8.0, 0.0), (10.0, 0.0), (10.0, 3.0)]
BACK_PATH = [(10.0, 3.0), (10.0, 0.0), (0.0, 0.0)]
HOVER_S = 20
RATE_TOL = 0.10
CPU_LIMIT = 10.0
STREAMED = {'HEARTBEAT', 'LOCAL_POSITION_NED', 'ATTITUDE', 'EKF_STATUS_REPORT', 'DISTANCE_SENSOR', 'SYS_STATUS',
            'VISION_POSITION_ESTIMATE', 'SET_POSITION_TARGET_LOCAL_NED'}


def med(v):
    v = [x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return float(np.median(v)) if v else float('nan')


def vel_delay(tw, xs, ys, T, lo, hi):
    """Shift (s) of a position stream (wall times tw) against ground truth T rows (t_wall, x, y, z) that
    best matches velocities over 0.2 s; returns (shift_ms, velocity_rms, position_rms_at_zero_shift)."""
    T = np.asarray(T)
    g = np.arange(lo, hi, 0.02)
    if len(g) < 50 or len(tw) < 20:
        return float('nan'), float('nan'), float('nan')
    gx, gy = np.interp(g, tw, xs), np.interp(g, tw, ys)
    h = 10
    vx, vy = (gx[h:] - gx[:-h]) / 0.2, (gy[h:] - gy[:-h]) / 0.2
    best = (float('inf'), 0.0)
    for d in np.arange(-0.1, 0.4, 0.005):
        tx, ty = np.interp(g - d, T[:, 0], T[:, 1]), np.interp(g - d, T[:, 0], T[:, 2])
        e = float(np.sqrt(np.mean((vx - (tx[h:] - tx[:-h]) / 0.2) ** 2 + (vy - (ty[h:] - ty[:-h]) / 0.2) ** 2)))
        best = min(best, (e, float(d)))
    tx, ty = np.interp(g, T[:, 0], T[:, 1]), np.interp(g, T[:, 0], T[:, 2])
    prms = float(np.sqrt(np.mean((gx - tx) ** 2 + (gy - ty) ** 2)))
    return round(best[1] * 1000, 0), round(best[0], 4), round(prms, 4)


def phase(name, cl, fc, lat, lines):
    from A1_10 import analyse
    from b_common import dist_to_polyline
    r = dict(name=name)
    ok, msg = cl.take_off(1.1)
    r['takeoff'] = msg
    if not ok:
        cl.command('land')
        r['ok'] = False
        lines.append(f'FAIL {name}: {msg}')
        return r
    time.sleep(2)
    t_air = time.time()
    time.sleep(HOVER_S)
    a1, s1, _, _ = cl.follow(OUT_PATH, 120)
    a2, s2, _, _ = cl.follow(BACK_PATH, 120) if a1 else (False, 0.0, 0, 0)
    t_end = time.time()
    okl, ml, _ = cl.command('land')
    r.update(path_out_arrived=a1, path_back_arrived=a2, path_s=round(s1 + s2, 1), landed=okl, land_message=ml)
    r['ok'] = bool(a1 and a2 and okl)

    # ---- the SLAM pose stream (A1.10's analysis on this phase's airborne window)
    with lat.lock:
        c = types.SimpleNamespace(clock=list(lat.clock), truth=list(lat.truth),
                                  scan=[s for s in lat.scan if t_air <= s[0] <= t_end],
                                  match=[m for m in lat.match if t_air <= m[0] <= t_end],
                                  tf=[f for f in lat.tf if t_air <= f[0] <= t_end])
    try:
        a = analyse(c)
    except Exception as e:  # noqa: BLE001
        a = {'error': repr(e)}
    C = np.array([k for k in c.clock if t_air <= k[0] <= t_end])
    r['rtf'] = round(float((C[-1, 1] - C[0, 1]) / (C[-1, 0] - C[0, 0])), 3) if len(C) > 10 else float('nan')
    Ca = np.array(c.clock)
    Ca = Ca[np.argsort(Ca[:, 1])]
    F = np.array(c.tf)
    tf_age = (F[:, 0] - np.interp(F[:, 1], Ca[:, 1], Ca[:, 0])) * 1000 if len(F) else np.array([])

    def sim_rate(stamps):
        return round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 2) if len(stamps) > 10 and stamps[-1] > stamps[0] else None
    rates = {'/scan': sim_rate(sorted(s[1] for s in c.scan)), 'tf map->base_link': sim_rate(sorted(f[1] for f in c.tf))}

    # ---- the bridge's own numbers (5 s windows fully inside the phase)
    with cl.lock:
        st = [d for t, d in cl.stats if t_air + 5 <= t <= t_end]
    keys = {}
    for d in st:
        k = (d.get('window_s') or 0) / d['window_sim_s'] if d.get('window_sim_s') else None
        if not k:
            continue
        for kind in ('rx_hz', 'tx_hz'):
            for m, v in (d.get(kind) or {}).items():
                if m not in STREAMED:
                    continue           # occasional messages (STATUSTEXT, COMMAND_ACK) have no rate to hold
                keys.setdefault(f"{'from FC' if kind == 'rx_hz' else 'to FC'} {m}", []).append(v * k)
    for m, v in keys.items():
        rates[m] = round(float(np.mean(v)), 2)
    bridge_age = med([d.get('pose_age_at_send_ms', {}).get('median') for d in st])
    bridge_cpu = [d.get('cpu_pct_one_core') for d in st if d.get('cpu_pct_one_core') is not None]
    r['rates_sim_hz'] = rates
    r['bridge_cpu_pct'] = {'median': round(med(bridge_cpu), 2), 'max': round(max(bridge_cpu), 2) if bridge_cpu else None}

    eff = a.get('effective_delay_ms', float('nan'))
    tfa = float(np.median(tf_age)) if len(tf_age) else float('nan')
    total = eff + max(0.0, bridge_age - tfa) if not math.isnan(bridge_age) and not math.isnan(tfa) else eff
    r['delay_ms'] = {'scan_delivery': a.get('scan_delivery_ms'), 'cartographer': a.get('match_latency_ms'),
                     'tf_age_median': round(tfa, 1), 'bridge_age_median': round(bridge_age, 1),
                     'effective': eff, 'total': round(total, 1)}
    r['slam'] = {k: a.get(k) for k in ('velocity_rms_at_best_shift_m_s', 'position_rms_at_best_shift_m',
                                       'position_rms_at_zero_shift_m', 'pose_jumps', 'error') if k in a}

    # ---- the flight controller's estimate against ground truth
    rows = np.array(fc.between(t_air, t_end))
    with cl.lock:
        T = [p for p in cl.truth if t_air - 1 <= p[0] <= t_end + 1]
    if len(rows) > 20 and len(T) > 20:
        fx, fy = rows[:, 1] + START_W[0], -rows[:, 2] + START_W[1]
        d, vr, prms = vel_delay(rows[:, 0], fx, fy, T, t_air + 1, t_end - 1)
        r['fc_estimate'] = {'delay_vs_truth_ms': d, 'position_rms_m': prms}
    lat_err = [dist_to_polyline(x - START_W[0], y - START_W[1], OUT_PATH)[0] for t, x, y, z in T]
    r['max_distance_from_path_m'] = round(max(lat_err), 3) if lat_err else None

    r['extnav_lost_count'] = sum('ExtNav lost' in x for t, x in fc.texts if t_air <= t <= t_end)
    dl = r['delay_ms']
    lines.append(f"{'ok  ' if r['ok'] else 'FAIL'} {name}: path {'completed' if r['ok'] else 'NOT completed'} "
                 f"({r['path_s']} s), real-time factor {r['rtf']}, bridge CPU {r['bridge_cpu_pct']['median']}% of one core "
                 f"(max {r['bridge_cpu_pct']['max']}); pose stream lost (flow fallback) {r['extnav_lost_count']} times")
    lines.append(f"       delays (median ms): scan delivery {(dl['scan_delivery'] or {}).get('median')}, "
                 f"Cartographer {(dl['cartographer'] or {}).get('median')}, TF age {dl['tf_age_median']}, "
                 f"bridge send age {dl['bridge_age_median']}; effective {dl['effective']} -> total {dl['total']} ms")
    fe = r.get('fc_estimate', {})
    lines.append(f"       flight controller vs truth: {fe.get('position_rms_m')} m rms, delay {fe.get('delay_vs_truth_ms')} ms; "
                 f"drone's largest distance from the path {r['max_distance_from_path_m']} m")
    return r


def main():
    from A1_10 import Latency
    from b_common import Client, FcRecorder, save
    from rclpy.executors import SingleThreadedExecutor
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default='w2_turns_junctions')
    a = ap.parse_args()
    res, lines = {}, []
    if not shutil.which('stress-ng'):
        lines.append('stress-ng is not installed: sudo apt install stress-ng')
        save(a.out, 'B9', res, False, lines)
        return 1
    stress = None
    with Client() as cl, FcRecorder() as fc:
        lat = Latency()
        ex = SingleThreadedExecutor()
        ex.add_node(lat)
        threading.Thread(target=ex.spin, daemon=True).start()
        try:
            print('B9 phase 1: no load', flush=True)
            res['unloaded'] = phase('unloaded', cl, fc, lat, lines)
            time.sleep(5)
            print('B9 phase 2: stress-ng --cpu 4', flush=True)
            stress = subprocess.Popen(['stress-ng', '--cpu', '4', '--timeout', '900s', '--quiet'],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
            time.sleep(10)
            res['loaded'] = phase('loaded (stress-ng --cpu 4)', cl, fc, lat, lines)
        finally:
            if stress is not None:
                os.killpg(stress.pid, signal.SIGTERM)
            ex.shutdown(timeout_sec=2.0)

    u, l = res['unloaded'], res.get('loaded', {})
    ok = u.get('ok', False) and l.get('ok', False)
    worst = 0.0
    lines.append('rates in simulation time (Hz): unloaded -> loaded')
    for k, v in sorted(u.get('rates_sim_hz', {}).items()):
        w = l.get('rates_sim_hz', {}).get(k)
        if not v or w is None:
            lines.append(f'       {k}: {v} -> {w} (not compared)')
            continue
        ch = (w - v) / v
        worst = max(worst, abs(ch))
        lines.append(f"  {'ok  ' if abs(ch) <= RATE_TOL else 'FAIL'} {k}: {v} -> {w} ({ch*100:+.1f}%)")
    rates_ok = worst <= RATE_TOL and bool(l.get('rates_sim_hz'))
    cpu_ok = all((p.get('bridge_cpu_pct', {}).get('max') or 99) < CPU_LIMIT for p in (u, l))
    rtf_ok = all((p.get('rtf') or 0) >= 0.9 for p in (u, l))
    viso = int(round(max(0.0, u.get('delay_ms', {}).get('total', float('nan'))) / 10.0) * 10) \
        if not math.isnan(u.get('delay_ms', {}).get('total', float('nan'))) else None
    res.update(largest_rate_change=round(worst, 3), rates_ok=rates_ok, bridge_cpu_ok=cpu_ok, rtf_ok=rtf_ok,
               suggested_VISO_DELAY_MS=viso)
    lines.append(f"{'ok  ' if rates_ok else 'FAIL'} largest rate change under load {worst*100:.1f}% (limit 10%)")
    lines.append(f"{'ok  ' if cpu_ok else 'FAIL'} bridge CPU below {CPU_LIMIT:.0f}% of one core in both phases")
    lines.append(f"{'ok  ' if rtf_ok else 'FAIL'} real-time factor >= 0.9 in both phases "
                 f"({u.get('rtf')} / {l.get('rtf')}; below it timing results are not valid)")
    lines.append(f'suggested VISO_DELAY_MS {viso}  (unloaded total pose delay {u.get("delay_ms", {}).get("total")} ms; '
                 f'currently 10 from A1.10)')
    passed = ok and rates_ok and cpu_ok and rtf_ok
    save(a.out, 'B9', res, passed, lines)
    return 0 if passed else 1


if __name__ == '__main__':
    from b_common import finish
    finish(main())
