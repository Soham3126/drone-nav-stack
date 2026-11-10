#!/usr/bin/env python3
"""cpu_logger.py - CPU % (of one core) and memory per stack process, once a second -> cpu.csv (guidebook 4.2).

  python3 cpu_logger.py --out RUN_DIR/cpu.csv      (stops on Ctrl+C or when the tmux window closes)
"""
import argparse
import csv
import signal
import sys
import time

import psutil

WATCH = {  # label -> substrings looked for in the command line
    'gz_sim': ['gz sim', 'gz-sim'],
    'arducopter': ['arducopter'],
    'mavproxy': ['mavproxy'],
    'cartographer': ['cartographer_node'],
    'carto_grid': ['cartographer_occupancy_grid_node'],
    'gz_bridge': ['parameter_bridge'],
    'robot_state_pub': ['robot_state_publisher'],
    'truth_relay': ['truth_relay'],
    'planner_server': ['planner_server'],
    'frontier_server': ['frontier_server'],
    'mission_brain': ['mission_brain'],
    'bridge': ['bridge_node'],
    'rosbag': ['ros2 bag record', 'rosbag2'],
}


def label_of(p):
    try:
        cmd = ' '.join(p.cmdline())
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return None
    for label, keys in WATCH.items():
        if label == 'cartographer' and 'occupancy_grid' in cmd:
            continue
        if any(k in cmd for k in keys):
            return label
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--period', type=float, default=1.0)
    args = ap.parse_args()
    stop = [False]
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, lambda *_: stop.__setitem__(0, True))
    tracked = {}
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_wall', 'label', 'pid', 'cpu_percent', 'rss_mb'])
        while not stop[0]:
            for p in psutil.process_iter():
                if p.pid not in tracked:
                    lab = label_of(p)
                    if lab:
                        tracked[p.pid] = (p, lab)
                        p.cpu_percent(None)   # prime the counter
            time.sleep(args.period)
            now = time.time()
            for pid, (p, lab) in list(tracked.items()):
                try:
                    w.writerow([f'{now:.2f}', lab, pid, f'{p.cpu_percent(None):.1f}',
                                f'{p.memory_info().rss / 1e6:.1f}'])
                except psutil.NoSuchProcess:
                    del tracked[pid]
            f.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
