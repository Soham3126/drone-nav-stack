#!/usr/bin/env python3
"""B6 Companion freeze (guidebook 4.4) - W2, truth profile, straight path at the chosen speed.

One case per run (env B6_CASE):
  freeze  midway, the bridge is frozen (kill -STOP) for 5 s, then resumed (kill -CONT)
  kill    midway, the bridge is killed (kill -9); after 5 s a new bridge is started and the rest of
          the path is sent again
  pose    midway, only the pose stream is paused for 10 s (stream_gate.py; needs GATE=1)
    B6_CASE=freeze scripts/run_test.sh B6 w2_turns_junctions truth 600
    B6_CASE=pose GATE=1 scripts/run_test.sh B6 w2_turns_junctions truth 600
All cases x 2:  scripts/batch.sh b6 "scripts/run_b6.sh"
Measured: the last carrot (drone progress + L when the bridge stopped), how far the drone ends from it after
5 s (hold error, mean of the last second) and its largest overshoot past it; whether the path is then completed.
Pose case: when the bridge reports the EKF losing position, flight mode changes, and physical drift.
Pass: hold within 0.1 m of the last carrot in freeze and kill cases; pose case documented.
"""
import math
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

START_W = (0.5, 3.5)
PATH = [(0.0, 0.0), (8.0, 0.0)]
MIDWAY_X = 4.0          # map x where the disturbance starts
L = 0.6                 # bridge lookahead (default since B5)


def bridge_pid():
    r = subprocess.run(['pgrep', '-f', 'lib/drone_bridge/bridge_node'], capture_output=True, text=True)
    pids = [int(p) for p in r.stdout.split()]
    return pids[0] if pids else None


def start_bridge(out):
    cmd = ('source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; '
           'exec ros2 run drone_bridge bridge_node --ros-args -r __node:=bridge -p use_sim_time:=true '
           '-p pose_source:=topic -p pose_topic:=/truth/pose -p ground_clearance_m:=0.145')
    log = open(os.path.join(out, 'bridge_restart.log'), 'w')
    return subprocess.Popen(['bash', '-c', cmd], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def map_x(p):
    return p[1] - START_W[0], p[2] - START_W[1]


def main():
    import argparse
    from b_common import Client, call_setbool, save, wait_for
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default='w2_turns_junctions')
    a = ap.parse_args()
    case = os.environ.get('B6_CASE', 'freeze')
    res = dict(case=case)
    lines = [f'B6 case {case}']
    new_bridge = None
    with Client() as cl:
        ok, msg = cl.take_off(1.1)
        lines.append(msg)
        if not ok:
            cl.command('land')
            save(a.out, 'B6', res, False, lines)
            return 1
        time.sleep(3)
        cl.follow_async(PATH)
        if not wait_for(lambda: cl.truth and map_x(cl.truth[-1])[0] >= MIDWAY_X, 60, period=0.02):
            save(a.out, 'B6', res, False, lines + ['never reached midway'])
            return 1
        x_f = map_x(cl.truth[-1])[0]
        carrot = (min(x_f + L, PATH[-1][0]), 0.0)
        t0 = time.time()
        pid = bridge_pid()
        res.update(x_at_disturbance=round(x_f, 3), last_carrot_map=carrot, bridge_pid=pid)
        st_before = cl.last()
        if case == 'freeze':
            os.kill(pid, signal.SIGSTOP)
            time.sleep(5)
            os.kill(pid, signal.SIGCONT)
        elif case == 'kill':
            os.kill(pid, signal.SIGKILL)
            time.sleep(5)
        elif case == 'pose':
            okp, mp = call_setbool(cl, '/pose_gate/pause', True)
            lines.append(f'pose paused: {okp} {mp}')
            time.sleep(10)
            call_setbool(cl, '/pose_gate/pause', False)
        t1 = time.time()
        during = cl.truth_between(t0, t1)
        if case == 'kill':
            new_bridge = start_bridge(a.out)
            wait_for(lambda: cl.last().fc_connected and (time.time() - cl.status[-1][0]) < 0.5, 60)
            time.sleep(2)
            x_now = map_x(cl.truth[-1])[0]
            arrived, secs, _, _ = cl.follow([(x_now, 0.0), PATH[-1]], 60)
        else:
            arrived = wait_for(lambda: cl.last().path_arrived, 60, period=0.05)
        sts = [(t, m) for t, m in cl.status if t0 - 2 <= t <= t1 + 5]
        okl, ml, _ = cl.command('land')
        lines.append(f'land: {ml}')

    cw = (carrot[0] + START_W[0], carrot[1] + START_W[1])
    last1 = [p for p in during if p[0] >= t1 - 1.0]
    if last1:
        mx = sum(p[1] for p in last1) / len(last1)
        my = sum(p[2] for p in last1) / len(last1)
        res['hold_error_m'] = round(math.hypot(mx - cw[0], my - cw[1]), 3)
    res['overshoot_past_carrot_m'] = round(max([p[1] - cw[0] for p in during] + [0.0]), 3)
    res['max_sideways_m'] = round(max([abs(p[2] - cw[1]) for p in during] + [0.0]), 3)
    res['completed_path_after'] = bool(arrived)
    if case == 'pose':
        lost = [t - t0 for t, m in sts if t >= t0 and not m.ekf_position_ok]
        res['ekf_position_lost_after_s'] = round(lost[0], 1) if lost else None
        res['modes_seen'] = sorted({m.mode for _, m in sts})
        xs = [map_x(p)[0] for p in during]
        res['drift_during_pause_m'] = round(max(xs) - min(xs), 3) if xs else None
    res['status_before'] = dict(mode=st_before.mode, ekf_ok=st_before.ekf_position_ok)
    if new_bridge is not None:
        try:
            os.killpg(new_bridge.pid, signal.SIGINT)
            new_bridge.wait(10)
        except Exception:  # noqa: BLE001
            pass

    if case in ('freeze', 'kill'):
        ok = res.get('hold_error_m') is not None and res['hold_error_m'] <= 0.10 and res['completed_path_after']
        lines.append(f"{'ok  ' if ok else 'FAIL'} bridge {case} at x={x_f:.2f} m: last carrot x={carrot[0]:.2f}; "
                     f"after 5 s the drone was {res.get('hold_error_m', float('nan'))*100:.1f} cm from it "
                     f"(needs <= 10), overshoot {res['overshoot_past_carrot_m']*100:.1f} cm, sideways "
                     f"{res['max_sideways_m']*100:.1f} cm; path completed afterwards: {res['completed_path_after']}")
    else:
        ok = True
        lines.append(f"pose paused 10 s at x={x_f:.2f} m: EKF reported position lost after "
                     f"{res['ekf_position_lost_after_s']} s; modes {res['modes_seen']}; drone moved "
                     f"{(res['drift_during_pause_m'] or 0)*100:.1f} cm along the corridor during the pause "
                     f"(hold error {res.get('hold_error_m', float('nan'))*100:.1f} cm from the last carrot); "
                     f"path completed afterwards: {res['completed_path_after']}")
    save(a.out, 'B6', res, ok and okl, lines)
    return 0 if ok and okl else 1


if __name__ == '__main__':
    code = main()
    from b_common import finish
    finish(code)
