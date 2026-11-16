#!/usr/bin/env python3
"""c4_one.py - ONE mission on the stub world (used by C4, C5, C6): starts the stub world, the planner server,
the frontier server and the mission brain as separate ROS processes, calls /mission/start, waits for DONE / STOP
or the time limit, and writes <out>/result.json + events.jsonl + one log per node.

    python3 c4_one.py --world w3_tree_maze --index 0 --out /tmp/run [--speed 5] [--limit 1200] [--fault ...]
The ROS domain comes from ROS_DOMAIN_ID (C4 gives each parallel run its own).
"""
import argparse
import json
import math
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c_common import Procs, planner_params  # noqa: E402


class CpuMeter:
    """CPU use of each started process INCLUDING its children ('ros2 run' starts the node as a child), sampled
    once a second from /proc: % of one core."""

    def __init__(self, ps):
        self.ps = ps
        self.hz = os.sysconf('SC_CLK_TCK')
        self.last, self.series = {}, {}

    @staticmethod
    def _tree_ticks(root):
        kids, ticks = {}, {}
        for d in os.listdir('/proc'):
            if not d.isdigit():
                continue
            try:
                f = open(f'/proc/{d}/stat').read()
            except OSError:
                continue
            rest = f[f.rindex(')') + 2:].split()
            kids.setdefault(int(rest[1]), []).append(int(d))
            ticks[int(d)] = int(rest[11]) + int(rest[12])            # utime + stime
        total, todo = 0, [root]
        while todo:
            q = todo.pop()
            total += ticks.get(q, 0)
            todo += kids.get(q, [])
        return total

    def sample(self):
        now = time.time()
        for name, p in self.ps:
            t = self._tree_ticks(p.pid)
            if name in self.last:
                t0, w0 = self.last[name]
                if now > w0:
                    self.series.setdefault(name, []).append(100.0 * (t - t0) / self.hz / (now - w0))
            self.last[name] = (t, now)

    def summary(self):
        out = {}
        for name, v in self.series.items():
            v = sorted(v)
            out[name] = dict(mean=round(sum(v) / len(v), 1), p95=round(v[int(0.95 * (len(v) - 1))], 1),
                             max=round(v[-1], 1), samples=len(v))
        return out


def aba_switches(sw, window=10.0, r=0.5):
    """A -> B -> A within `window` s, where going back to A ABANDONED B (B was still an open frontier).
    Coming back to A because B was reached / explored / unreachable is not a change of mind. sw: (t, xy[, abandoned])."""
    n = 0
    for i in range(2, len(sw)):
        t, a = sw[i][0], sw[i][1]
        if len(sw[i]) > 2 and not sw[i][2]:
            continue
        for j in range(i - 1):
            b = sw[j][1]
            left = sw[j + 1][0]
            if t - left <= window and math.hypot(a[0] - b[0], a[1] - b[1]) <= r:
                n += 1
                break
    return n


def main():
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--index', type=int, default=0)
    ap.add_argument('--out', required=True)
    ap.add_argument('--speed', type=float, default=5.0)
    ap.add_argument('--limit', type=float, default=1200.0, help='simulated seconds (book: 20 min)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--fault', default='none',
                    choices=['none', 'plan_fail', 'arm_refuse', 'frontier_silence', 'status_stop'],
                    help='C6 fault injection (the 0.45 m gap fault is a world: w7_gaps)')
    ap.add_argument('--fault_t', type=float, default=60.0, help='simulated time the fault starts')
    ap.add_argument('--fault_len', type=float, default=10.0, help='frontier_silence: how long (simulated s)')
    ap.add_argument('--timing', action='store_true', help='C7: record frontier search times and CPU per process')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    params, _ = planner_params(a.out)
    import yaml
    p = yaml.safe_load(open(params))                 # the stub runs on simulated time
    p['planner_server']['ros__parameters']['use_sim_time'] = True
    p['global_costmap']['global_costmap']['ros__parameters']['use_sim_time'] = True
    yaml.safe_dump(p, open(params, 'w'))
    res = dict(world=a.world, index=a.index, speed=a.speed, limit_s=a.limit, domain=os.environ.get('ROS_DOMAIN_ID'))
    events, metrics, state = [], {}, {'s': ''}
    t_wall0 = time.time()
    with Procs(a.out) as procs:
        stub_faults = ''
        if a.fault == 'arm_refuse':
            stub_faults = ' -p fault_arm_refuse:=true'
        elif a.fault == 'status_stop':
            stub_faults = f' -p fault_status_stop_s:={a.fault_t}'
        procs.start('stub_world', f'ros2 run drone_sim stub_world.py --ros-args -p world:={a.world} '
                                  f'-p start_index:={a.index} -p speed_factor:={a.speed} -p seed:={a.seed}'
                                  + stub_faults)
        brain_args = ''
        if a.fault == 'plan_fail':
            procs.start('fault_planner', f'ros2 run drone_sim fault_planner.py --ros-args -p use_sim_time:=true '
                                         f'-p fail_after_s:={a.fault_t} '
                                         f'-p record:={os.path.join(a.out, "fault_planner.json")}')
            brain_args = ' -p planner_action:=/c6/compute_path_to_pose'
        procs.start('planner_server', f'ros2 run nav2_planner planner_server --ros-args --params-file {params}')
        procs.start('lifecycle', "ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args "
                                 "-r __node:=lifecycle_manager_c4 -p autostart:=true -p use_sim_time:=true "
                                 "-p \"node_names:=['planner_server']\"")
        fpr = procs.start('frontier_server', 'ros2 run drone_frontier frontier_server --ros-args '
                                             '-p use_sim_time:=true')
        procs.start('mission_brain', 'ros2 run drone_mission mission_brain --ros-args -p use_sim_time:=true'
                    + brain_args)
        rclpy.init()
        node = rclpy.create_node('c4_watch')
        tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/mission/events', lambda m: events.append(json.loads(m.data)), 200)
        node.create_subscription(String, '/mission/state', lambda m: state.update(s=m.data), tl)
        node.create_subscription(String, '/stub/metrics', lambda m: metrics.update(json.loads(m.data)), tl)
        start = node.create_client(Trigger, '/mission/start')
        ok_start = start.wait_for_service(timeout_sec=60.0)
        res['started'] = False
        if ok_start:
            # the brain accepts the start in WAIT_READY and takes off once its checks pass
            fut = start.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            res['started'] = bool(fut.result() and fut.result().success)
        wall_limit = a.limit / max(a.speed, 0.1) * 3 + 120
        silence = dict(paused=None, resumed=None)
        search_ms, cpu = [], CpuMeter(procs.ps)
        if a.timing:
            from std_msgs.msg import Float32
            node.create_subscription(Float32, '/frontiers/search_ms', lambda m: search_ms.append(m.data), 50)
        next_cpu = time.time() + 1.0
        while res['started'] and time.time() - t_wall0 < wall_limit:
            rclpy.spin_once(node, timeout_sec=0.1)
            if a.timing and time.time() >= next_cpu:
                cpu.sample()
                next_cpu += 1.0
            if a.fault == 'frontier_silence':        # freeze the frontier server for fault_len simulated seconds
                st = metrics.get('t', 0.0)
                if silence['paused'] is None and st >= a.fault_t:
                    os.killpg(fpr.pid, signal.SIGSTOP)
                    silence['paused'] = st
                elif silence['paused'] is not None and silence['resumed'] is None and st >= a.fault_t + a.fault_len:
                    os.killpg(fpr.pid, signal.SIGCONT)
                    silence['resumed'] = st
            if state['s'] in ('DONE', 'STOP') and any(e.get('event') == 'summary' for e in events):
                break
            if metrics.get('t', 0) > a.limit + 60:          # the brain's own state did not end in time
                break
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
        res['alive_at_end'] = procs.alive()
        node.destroy_node()
        rclpy.shutdown()
    with open(os.path.join(a.out, 'events.jsonl'), 'w') as f:
        for e in events:
            f.write(json.dumps(e) + '\n')
    summ = next((e for e in events if e.get('event') == 'summary'), {})
    st = [e for e in events if e.get('event') == 'state']
    t_exp = next((e['t'] for e in st if e['state'] == 'EXPLORE'), None)
    t_ret = next((e['t'] for e in st if e['state'] in ('RETURN', 'LAND', 'STOP')), None)
    t_done = next((e['t'] for e in st if e['state'] == 'DONE'), None)
    sw = [tuple(x) for x in summ.get('goal_switches', [])]
    exp_s = (t_ret - t_exp) if (t_exp is not None and t_ret is not None) else None
    res.update(final=state['s'] or 'none', reason=summ.get('reason', ''),
               mission_s=t_done, explore_s=exp_s, wall_s=round(time.time() - t_wall0, 1),
               coverage=metrics.get('coverage'), contacts=metrics.get('contacts'),
               min_clearance_m=metrics.get('min_clearance_m'), home_m=metrics.get('dist_from_start_m'),
               flown_m=metrics.get('flown_m'), goal_switches=len(sw),
               switches_per_4s=round(len(sw) / max(1.0, exp_s / 4.0), 3) if exp_s else None,
               aba=aba_switches(sw), blacklisted=len(summ.get('blacklisted', [])),
               escapes=summ.get('escapes'), planning_ms=summ.get('planning_ms'),
               blacklist_detail=summ.get('blacklisted', []), path_fixes=summ.get('path_fixes'),
               contact_log=metrics.get('contact_log', []))
    ok = (res['final'] == 'DONE' and (res['coverage'] or 0) >= 0.98 and res['home_m'] is not None
          and res['home_m'] <= 0.3 and res['contacts'] == 0 and res['switches_per_4s'] is not None
          and res['switches_per_4s'] <= 1.0 and res['aba'] == 0)
    res['pass'] = bool(ok)
    res['fault'] = a.fault
    if a.timing:
        res['frontier_search_ms'] = search_ms
        res['cpu_pct_one_core'] = cpu.summary()
        res['plan_wall_ms'] = summ.get('plan_wall_ms')
        res['loop_period_s'] = summ.get('loop_period_s')
    if a.fault == 'frontier_silence':
        res['silence'] = silence
    fp = os.path.join(a.out, 'fault_planner.json')
    if os.path.exists(fp):
        res['fault_planner'] = json.load(open(fp))
    json.dump(res, open(os.path.join(a.out, 'result.json'), 'w'), indent=1)
    print(json.dumps({k: res[k] for k in ('world', 'index', 'final', 'coverage', 'contacts', 'home_m',
                                          'switches_per_4s', 'aba', 'mission_s', 'wall_s', 'pass')}))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
