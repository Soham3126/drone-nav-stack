#!/usr/bin/env python3
"""A1.2 Box-room laps: W0, gps, square_3m x 3 laps at 0.5 m/s. Pass: ATE < 0.05 m, end error < 0.08 m,
precision >= 0.95, no phantom clusters; 3 of 3 repeats.
    for r in 1 2 3; do scripts/run_test.sh A1.2 w0_box_room gps; done
    python3 drone_sim/tools/eval_run.py --summary A1.2"""
import sys
from a1_common import run
sys.exit(run('A1.2', 'w0_box_room', laps=3, yaw='fixed', worlds=['w0_box_room']))
