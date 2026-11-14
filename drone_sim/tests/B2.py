#!/usr/bin/env python3
"""B2 Flight commands (guidebook 4.4) - W0, profile truth, ground-truth pose streaming.

    scripts/run_test.sh B2 w0_box_room truth 1800

Three times:
 1. happy path: guided, arm, takeoff 1.1, hover 5 s, land - every response checked, plus the
    resulting state in /bridge/status (mode, armed, height);
 2. stop ArduPilot (Ctrl+C in its window) and call arm: expect failure within 9 s (3 tries x 3 s);
 3. restart ArduPilot and call arm at once, before the EKF has its origin/position: expect a refusal
    whose message carries ArduPilot's own text (PreArm: ...).
Pass: all three cases behave as expected, 3 of 3 times.
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from b_common import Client, finish, save, sitl_start, sitl_stop, wait_for  # noqa: E402

REPEATS = 3


def happy_path(cl, log):
    steps = []

    def step(name, value=0.0, expect=None, timeout=90):
        ok, msg, dt = cl.command(name, value, timeout)
        time.sleep(0.5)
        st = cl.last()
        good = ok and (expect is None or expect(st))
        th = cl.truth_height()
        steps.append(dict(cmd=name, ok=ok, state_ok=good, seconds=round(dt, 2), message=msg,
                          mode=st.mode, armed=st.armed, height=round(st.height_m, 3), truth_height=round(th, 3)))
        log(f"   {name:8s} {'ok  ' if good else 'FAIL'} {dt:5.1f} s  mode {st.mode:8s} armed {st.armed!s:5s} "
            f"height {st.height_m:.2f} (true {th:.2f})  | {msg}")
        if cl.stats:
            log(f"            rangefinder {cl.stats[-1][1].get('rangefinder_cm_min_max_orient_id')}, "
                f"EKF height {cl.stats[-1][1].get('ekf_height_m')} m")
        return good

    def arm_when_ready(limit=60):
        """ArduPilot refuses to arm while its own pre-flight checks settle after a (re)start
        ('Accels inconsistent', 'Battery unhealthy' ...). The book's 'FC ready' check waits for that;
        here: retry every 5 s for up to `limit` s and record how many tries it took."""
        t0 = time.time()
        while True:
            if step('arm', expect=lambda s: s.armed):
                return True
            if time.time() - t0 > limit:
                return False
            time.sleep(5)
            step('guided', expect=lambda s: s.mode == 'GUIDED')

    good = (step('guided', expect=lambda s: s.mode == 'GUIDED')
            and arm_when_ready()
            and step('takeoff', 1.1, expect=lambda s: abs(s.height_m - 1.1) <= 0.1))
    if good:
        time.sleep(5)
    good = step('land', expect=lambda s: not s.armed) and good
    return good, steps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default='w0_box_room')
    a = ap.parse_args()
    res = {'rounds': []}
    lines = []

    def log(s):
        lines.append(s)
        print(s, flush=True)

    with Client() as cl:
        for k in range(1, REPEATS + 1):
            rnd = {}
            log(f'round {k}')
            # wait until the EKF uses our pose (it needs origin + position after every restart)
            ready = wait_for(lambda: cl.last() is not None and cl.last().fc_connected
                             and cl.last().ekf_position_ok and cl.last().origin_set, 180)
            if not ready:
                log('   EKF never reported a position from our pose: cannot fly')
            ok1, steps = happy_path(cl, log) if ready else (False, [])
            rnd['happy'] = dict(ok=ok1, steps=steps)
            log(f" case 1 happy path: {'ok' if ok1 else 'FAIL'}")

            # case 2 of the book's list order is the refusal; we need ArduPilot down first, so: timeout case
            if not sitl_stop():
                log('   could not stop ArduPilot')
            time.sleep(0.5)
            okc, msg, dt = cl.command('arm', timeout=30)
            ok3 = (not okc) and dt <= 9.5
            rnd['timeout'] = dict(ok=ok3, success=okc, seconds=round(dt, 2), message=msg)
            log(f" case 3 ArduPilot stopped, arm: {'ok' if ok3 else 'FAIL'} - failed after {dt:.1f} s | {msg}")

            # refusal: restart and call arm as soon as the bridge is connected, before the EKF is ready
            wait_for(lambda: not cl.last().fc_connected, 15)
            sitl_start(a.out)
            wait_for(lambda: cl.last().fc_connected, 120, period=0.05)
            time.sleep(3)                     # let ArduPilot finish booting so it can answer at all
            st = cl.last()
            okc, msg, dt = cl.command('arm', timeout=30)
            has_text = any(w in msg for w in ('PreArm', 'Arm:', 'prearm', 'Need'))
            ok2 = (not okc) and has_text
            rnd['refusal'] = dict(ok=ok2, success=okc, seconds=round(dt, 2), message=msg,
                                  ekf_ok_at_call=st.ekf_position_ok, origin_at_call=st.origin_set)
            log(f" case 2 arm before origin/position: {'ok' if ok2 else 'FAIL'} "
                f"(EKF ready at call: {st.ekf_position_ok}) | {msg}")
            if okc:                           # it armed after all: disarm safely
                cl.command('land')
            res['rounds'].append(rnd)

    counts = {c: sum(r[c]['ok'] for r in res['rounds']) for c in ('happy', 'refusal', 'timeout')}
    res['counts'] = counts
    lines.append(f"happy path {counts['happy']}/{REPEATS}, refusal {counts['refusal']}/{REPEATS}, "
                 f"timeout {counts['timeout']}/{REPEATS}")
    ok = all(v == REPEATS for v in counts.values())
    save(a.out, 'B2', res, ok, lines[-1:])
    return 0 if ok else 1


if __name__ == '__main__':
    finish(main())
