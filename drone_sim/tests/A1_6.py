#!/usr/bin/env python3
"""A1.6 Tilt stress: W4, gps, maze_lap fast (WP_SPD 1.5, WP_ACC raised) heading held (YAW=travel to turn the nose),
so the drone tilts 10-15 deg while braking at every corner. Pass: no phantom cluster > 0.2 m, ATE < 0.15 m.
eval_run reports tilt_max_deg from ground truth (expected 10-15 deg).
    scripts/run_test.sh A1.6 w4_maze_rooms gps 1800
Env: SPEED (default 1.5), WP_ACC (default 5.0 m/s^2)."""
import os
import sys
from a1_common import run
acc = float(os.environ.get('WP_ACC', 5.0))
os.environ['WP_ACC'] = str(acc)
sys.exit(run('A1.6', 'w4_maze_rooms', yaw='fixed', speed=1.5, worlds=['w4_maze_rooms']))
