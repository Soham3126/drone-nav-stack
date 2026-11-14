#!/usr/bin/env python3
"""B3 Pose into EKF3: hover (guidebook 4.4) - W0, truth profile, pose through pose_mangler.py.

One case per run; the case comes from the MANGLE yaml (delay s, noise m) that sim_up.sh hands to the
mangler. VISO_DELAY_MS is set to 50 for B3, as the book says. Run all cases with scripts/run_b3.sh.

    MANGLE=drone_sim/tests/b3_cases/d050_n00.yaml scripts/run_test.sh B3 w0_box_room truth

Procedure: wait until the EKF uses our pose, guided, arm, take off to 1.1 m, hover 60 s, land.
Measured over the 60 s hover (ground truth from Gazebo):
  drift_m      largest horizontal distance from the position at the start of the hover
  osc_m        largest horizontal distance from the hover's mean position (oscillation / jitter size)
  offset_m/deg largest bridge frame offset (FC frame vs pose frame) during the hover
Pass rules:
  matched case (delay 50 ms, noise 0): drift < 0.05 m and offset < 0.02 m
  delay mismatch <= 50 ms and noise <= 2 cm: osc <= 0.10 m
  other cases: recorded only (the book expects oscillation / jitter there)
Summary of all cases:  python3 drone_sim/tests/B3.py --summary
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

VISO_DELAY_S = 0.050
HOVER_S = 60


def case_from_env():
    path = os.environ.get('MANGLE', 'none')
    if path in ('', 'none'):
        return 'none', 0.0, 0.0
    import yaml
    with open(os.path.expanduser(path)) as f:
        d = yaml.safe_load(f) or {}
    return os.path.basename(path), float(d.get('delay', 0.0)), float(d.get('noise', 0.0))


def record_fc(seconds):
    """LOCAL_POSITION_NED from SITL's spare port for `seconds` (wall time, n, e, d). Falls back to sleeping."""
    from b_common import gcs_heartbeat, spare_heartbeat
    from pymavlink import mavutil
    c = spare_heartbeat(10)
    out, t0 = [], time.time()
    if c is None:
        time.sleep(seconds)
        return out
    c.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 1e5, 0, 0, 0, 0, 0)
    t_hb = 0.0
    while time.time() - t0 < seconds:
        if time.time() - t_hb > 1.0:
            gcs_heartbeat(c)
            t_hb = time.time()
        m = c.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.5)
        if m is not None and m.get_srcComponent() == 1:
            out.append((time.time(), m.x, m.y, m.z))
    c.close()
    return out


def plot(out, tr, fc, st, z0, title):
    """Hover picture: true path vs the flight controller's belief, and the frame offset over time."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:  # noqa: BLE001
        return
    t0 = tr[0][0]
    tx, ty = tr[0][1], tr[0][2]
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
    ax[0].plot([p[1] - tx for p in tr], [p[2] - ty for p in tr], lw=0.8, label='true (Gazebo)')
    if fc:
        n0, e0 = fc[0][1], fc[0][2]
        ax[0].plot([f[1] - n0 for f in fc], [-(f[2] - e0) for f in fc], lw=0.8, label="flight controller's belief")
    ax[0].plot(0, 0, 'k+', ms=12)
    ax[0].set_aspect('equal')
    ax[0].set_xlabel('x (m)')
    ax[0].set_ylabel('y (m)')
    ax[0].legend(fontsize=8)
    ax[0].set_title('hover path (from the hover start)')
    ax[1].plot([p[0] - t0 for p in tr], [math.hypot(p[1] - tx, p[2] - ty) * 100 for p in tr], lw=0.8,
               label='true distance from start (cm)')
    ts = [t for t, _ in st]
    ax[1].plot([t - t0 for t in ts], [m.frame_offset_m * 100 for _, m in st], lw=0.8, label='bridge frame offset (cm)')
    ax[1].set_xlabel('hover time (s)')
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'b3_hover.png'), dpi=110)
    plt.close(fig)


def judge(delay, noise, r):
    """(rule text, ok or None when only recorded)."""
    if abs(delay - VISO_DELAY_S) < 1e-6 and noise == 0:
        ok = r['drift_m'] < 0.05 and r['offset_max_m'] < 0.02
        return f"matched: drift {r['drift_m']*100:.1f} cm < 5, offset {r['offset_max_m']*100:.1f} cm < 2", ok
    if abs(delay - VISO_DELAY_S) <= 0.050 + 1e-6 and noise <= 0.02 + 1e-9:
        return f"mismatch <= 50 ms, noise <= 2 cm: oscillation {r['osc_m']*100:.1f} cm <= 10", r['osc_m'] <= 0.10
    return f"recorded only: oscillation {r['osc_m']*100:.1f} cm, drift {r['drift_m']*100:.1f} cm", None


def run(out):
    from b_common import Client, save, set_param, wait_for
    name, delay, noise = case_from_env()
    res = dict(case=name, mangle_delay_s=delay, mangle_noise_m=noise, viso_delay_ms=None)
    lines = [f'case {name}: pose delay {delay*1000:.0f} ms (VISO_DELAY_MS 50), noise {noise*100:.0f} cm']
    with Client() as cl:
        if not wait_for(lambda: cl.last() is not None and cl.last().fc_connected, 120):
            save(out, 'B3', res, False, lines + ['bridge never connected'])
            return 1
        res['viso_delay_ms'] = set_param('VISO_DELAY_MS', VISO_DELAY_S * 1000)
        lines.append(f"VISO_DELAY_MS read back: {res['viso_delay_ms']}")
        if not wait_for(lambda: cl.last().ekf_position_ok and cl.last().origin_set, 180):
            save(out, 'B3', res, False, lines + ['EKF never used our pose'])
            return 1
        ok, msg, _ = cl.command('guided')
        t0 = time.time()
        while True:                                   # ArduPilot's own checks may need a moment
            cl.command('guided')
            ok, msg, _ = cl.command('arm')
            if ok or time.time() - t0 > 60:
                break
            time.sleep(5)
        if not ok:
            save(out, 'B3', res, False, lines + [f'arm failed: {msg}'])
            return 1
        ok, msg, _ = cl.command('takeoff', 1.1)
        lines.append(f'takeoff: {msg}')
        if not ok:
            cl.command('land')
            save(out, 'B3', res, False, lines)
            return 1
        time.sleep(3)                                 # settle after the climb
        n_tr, n_st = len(cl.truth), len(cl.status)
        fc = record_fc(HOVER_S)                       # the flight controller's own position, same 60 s
        tr = cl.truth[n_tr:]
        st_t = list(cl.status[n_st:])
        st = [m for _, m in st_t]
        ok_land, msg_land, _ = cl.command('land')
        lines.append(f'land: {msg_land}')

    xs, ys = [p[1] for p in tr], [p[2] for p in tr]
    x0, y0 = xs[0], ys[0]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    res.update(
        hover_samples=len(tr),
        drift_m=round(max(math.hypot(x - x0, y - y0) for x, y in zip(xs, ys)), 4),
        osc_m=round(max(math.hypot(x - mx, y - my) for x, y in zip(xs, ys)), 4),
        height_mean_m=round(sum(p[3] for p in tr) / len(tr) - (cl.truth_z0 or 0.0), 3),
        offset_max_m=round(max((m.frame_offset_m for m in st if not math.isnan(m.frame_offset_m)), default=float('nan')), 4),
        offset_max_deg=round(max((m.frame_offset_deg for m in st if not math.isnan(m.frame_offset_deg)), default=float('nan')), 3),
        landed=ok_land)
    res['fc_samples'] = len(fc)
    if fc:
        n0, e0 = fc[0][1], fc[0][2]
        res['fc_belief_drift_m'] = round(max(math.hypot(f[1] - n0, f[2] - e0) for f in fc), 4)
    json.dump({'truth': tr, 'fc': fc, 'offset': [(t, m.frame_offset_m) for t, m in st_t]},
              open(os.path.join(out, 'b3_series.json'), 'w'))
    plot(out, tr, fc, st_t, cl.truth_z0, f'B3 {name}: delay {delay*1000:.0f} ms, noise {noise*100:.0f} cm')
    rule, ok = judge(delay, noise, res)
    res['rule'] = rule
    lines.append(f"hover {HOVER_S} s: drift {res['drift_m']*100:.1f} cm, oscillation {res['osc_m']*100:.1f} cm, "
                 f"frame offset max {res['offset_max_m']*100:.1f} cm / {res['offset_max_deg']:.2f} deg, "
                 f"true height {res['height_mean_m']:.2f} m")
    if 'fc_belief_drift_m' in res:
        lines.append(f"flight controller's own position moved {res['fc_belief_drift_m']*100:.1f} cm during the hover")
    lines.append(rule)
    passed = (ok is not False) and ok_land
    save(out, 'B3', res, passed, lines)
    return 0 if passed else 1


def summary(root):
    rows = []
    for f in sorted(glob.glob(os.path.join(os.path.expanduser(root), '*_B3_*', 'result.json'))):
        r = json.load(open(f))
        if 'drift_m' in r:
            rows.append(r)
    print(f"{'case':16s} {'delay ms':>8s} {'noise cm':>8s} {'drift cm':>8s} {'osc cm':>7s} {'offset cm':>9s}  verdict")
    for r in rows:
        print(f"{r['case']:16s} {r['mangle_delay_s']*1000:8.0f} {r['mangle_noise_m']*100:8.0f} {r['drift_m']*100:8.1f} "
              f"{r['osc_m']*100:7.1f} {r['offset_max_m']*100:9.1f}  {'PASS' if r['pass'] else 'FAIL'} - {r['rule']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out')
    ap.add_argument('--world', default='w0_box_room')
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
