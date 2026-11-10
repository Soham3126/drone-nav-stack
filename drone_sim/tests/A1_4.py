#!/usr/bin/env python3
"""A1.4 Turns and junctions: W2, gps, turns_tour at 0.5 m/s with the nose along each leg, so the drone
really turns. Pass: heading error < 1 deg after every turn, ATE < 0.08 m; 3 of 3 repeats.
    for r in 1 2 3; do scripts/run_test.sh A1.4 w2_turns_junctions gps; done"""
import sys
from a1_common import run
sys.exit(run('A1.4', 'w2_turns_junctions', yaw='travel', worlds=['w2_turns_junctions']))
