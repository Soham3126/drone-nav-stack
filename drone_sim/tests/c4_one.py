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
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from c_common import Procs, planner_params  # noqa: E402


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
        procs.start('stub_world', f'ros2 run drone_sim stub_world.py --ros-args -p world:={a.world} '
                                  f'-p start_index:={a.index} -p speed_factor:={a.speed} -p seed:={a.seed}')
        procs.start('planner_server', f'ros2 run nav2_planner planner_server --ros-args --params-file {params}')
        procs.start('lifecycle', "ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args "
                                 "-r __node:=lifecycle_manager_c4 -p autostart:=true -p use_sim_time:=true "
                                 "-p \"node_names:=['planner_server']\"")
        procs.start('frontier_server', 'ros2 run drone_frontier frontier_server --ros-args -p use_sim_time:=true')
        procs.start('mission_brain', 'ros2 run drone_mission mission_brain --ros-args -p use_sim_time:=true')
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
        while res['started'] and time.time() - t_wall0 < wall_limit:
            rclpy.spin_once(node, timeout_sec=0.1)
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
    json.dump(res, open(os.path.join(a.out, 'result.json'), 'w'), indent=1)
    print(json.dumps({k: res[k] for k in ('world', 'index', 'final', 'coverage', 'contacts', 'home_m',
                                          'switches_per_4s', 'aba', 'mission_s', 'wall_s', 'pass')}))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
