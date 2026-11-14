#!/usr/bin/env python3
"""B1 Link and telemetry (guidebook 4.4) - W0, profile truth.

    scripts/run_test.sh B1 w0_box_room truth

1. The bridge sees the autopilot's HEARTBEAT and sends its own at 1 Hz (counted on SITL's spare port 5763,
   where ArduPilot forwards the bridge's messages).
2. Received rate of each requested message over 60 s (the bridge's /bridge/link_stats).
3. /bridge/status at 10 Hz with the right mode, armed state and height.
4. ArduPilot alone is restarted; time from ArduPilot being back (heartbeat on the spare port) until the bridge
   reports fc_connected again.
Pass: all rates within +-10 % of requested; status correct; reconnect within 5 s.
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from b_common import Client, finish, gcs_heartbeat, save, sitl_start, sitl_stop, spare_heartbeat, wait_for  # noqa: E402

REQUESTED = {'HEARTBEAT': 1, 'LOCAL_POSITION_NED': 30, 'ATTITUDE': 30, 'EKF_STATUS_REPORT': 2,
             'DISTANCE_SENSOR': 10, 'SYS_STATUS': 2}
WINDOW_S = 60


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default='w0_box_room')
    a = ap.parse_args()
    res, lines, ok = {}, [], True

    with Client() as cl:
        print('waiting for the bridge to connect to ArduPilot ...', flush=True)
        if not wait_for(lambda: cl.last() is not None and cl.last().fc_connected, 120):
            save(a.out, 'B1', {'error': 'bridge never reported fc_connected'}, False, ['bridge not connected'])
            return 1
        time.sleep(10)                      # streams requested; let rates settle

        # 1 + 3: our heartbeat and the autopilot's mode, from the spare port; status rate
        spare = spare_heartbeat(30)
        print(f'measuring for {WINDOW_S} s ...', flush=True)
        n_status0 = len(cl.status)
        n_stats0 = len(cl.stats)
        t0 = time.time()
        hb_bridge, ap_modes = 0, set()
        t_hb = 0.0
        while time.time() - t0 < WINDOW_S:
            if spare and time.time() - t_hb >= 1.0:
                gcs_heartbeat(spare)
                t_hb = time.time()
            m = spare.recv_match(type='HEARTBEAT', blocking=True, timeout=1) if spare else None
            if m is None:
                continue
            if m.get_srcComponent() == 191:
                hb_bridge += 1
            elif m.get_srcComponent() == 1:
                ap_modes.add(m.custom_mode)
        dur = time.time() - t0
        status_hz = (len(cl.status) - n_status0) / dur
        res['bridge_heartbeat_hz'] = round(hb_bridge / dur, 3)
        res['status_hz'] = round(status_hz, 2)
        okhb = abs(res['bridge_heartbeat_hz'] - 1.0) <= 0.1
        okst = abs(status_hz - 10.0) <= 1.0
        lines.append(f"{'ok  ' if okhb else 'FAIL'} bridge HEARTBEAT {res['bridge_heartbeat_hz']:.2f} Hz (needs 1 +-10 %)")
        lines.append(f"{'ok  ' if okst else 'FAIL'} /bridge/status {status_hz:.2f} Hz (needs 10 +-10 %)")
        ok &= okhb and okst

        # 2: received rates (average of the link_stats windows inside the measurement)
        wins = [s for _, s in cl.stats[n_stats0:]]
        res['link_stats_windows'] = len(wins)
        for name, want in REQUESTED.items():
            vals = [w['rx_hz'].get(name, 0.0) for w in wins]
            got = sum(vals) / len(vals) if vals else 0.0
            good = abs(got - want) <= 0.1 * want
            res[f'rx_{name}_hz'] = round(got, 2)
            lines.append(f"{'ok  ' if good else 'FAIL'} {name:20s} {got:6.2f} Hz (requested {want})")
            ok &= good

        # 3: status content
        st = cl.last()
        if cl.stats:
            lines.append(f"     rangefinder/EKF height detail: {cl.stats[-1][1].get('rangefinder_cm_min_max_orient_id')}"
                         f" / {cl.stats[-1][1].get('ekf_height_m')} m")
        mode_ok = st.mode != '' and (not ap_modes or any(st.mode == {0: 'STABILIZE', 4: 'GUIDED', 5: 'LOITER', 9: 'LAND'}
                                                           .get(cm, str(cm)) for cm in ap_modes))
        res.update(status_mode=st.mode, status_armed=st.armed, status_height_m=round(st.height_m, 3),
                   status_origin_set=st.origin_set, status_ekf_position_ok=st.ekf_position_ok,
                   frame_offset_m=round(st.frame_offset_m, 4), frame_offset_deg=round(st.frame_offset_deg, 3))
        good = mode_ok and not st.armed and abs(st.height_m) < 0.2
        lines.append(f"{'ok  ' if good else 'FAIL'} status: mode {st.mode}, armed {st.armed}, height {st.height_m:.3f} m "
                     f"(on the ground), origin {st.origin_set}, EKF position {st.ekf_position_ok}, "
                     f"frame offset {st.frame_offset_m * 100:.1f} cm / {st.frame_offset_deg:.2f} deg")
        ok &= good
        if spare:
            spare.close()

        # 4: restart ArduPilot alone
        print('restarting ArduPilot (SITL) alone ...', flush=True)
        conns_before = cl.stats[-1][1].get('connections', 0) if cl.stats else 0
        n_before = len(cl.status)
        stopped = sitl_stop()
        lost = wait_for(lambda: not cl.last().fc_connected, 15)
        trace = [(round(t - time.time(), 2), m.fc_connected) for t, m in cl.status[n_before:]]
        res['connected_trace_after_stop'] = [x for i, x in enumerate(trace) if i == 0 or x[1] != trace[i - 1][1]]
        sitl_start(a.out)
        spare = spare_heartbeat(120)
        t_back = time.time()
        rec = wait_for(lambda: cl.last().fc_connected, 30, period=0.05)
        reconnect_s = time.time() - t_back if rec else float('inf')
        time.sleep(6)                       # one more link_stats window
        conns_after = cl.stats[-1][1].get('connections', 0) if cl.stats else 0
        lost = lost or conns_after > conns_before     # a new connection number also proves the loss
        res.update(sitl_stopped=stopped, link_loss_seen=lost, reconnect_s=round(reconnect_s, 2),
                   connections_before=conns_before, connections_after=conns_after)
        res['link_stats_last'] = cl.stats[-1][1] if cl.stats else None
        good = lost and reconnect_s <= 5.0
        lines.append(f"{'ok  ' if good else 'FAIL'} reconnect {reconnect_s:.2f} s after ArduPilot was back "
                     f"(needs <= 5 s; ArduPilot stopped: {stopped}, loss seen: {lost})")
        ok &= good
        if spare:
            spare.close()

    save(a.out, 'B1', res, ok, lines)
    return 0 if ok else 1


if __name__ == '__main__':
    finish(main())
