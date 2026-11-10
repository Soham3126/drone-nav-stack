#!/usr/bin/env python3
"""A1.5 Maze laps: W3 and W4, gps, maze_lap at 0.5 m/s (sign-off) and 1.0 m/s (recorded), 3 repeats.
Pass at 0.5 m/s: ATE < 0.10 m, end < 0.15 m, precision >= 0.95, recall >= 0.90,
phantom < 0.5 % of occupied cells, no phantom cluster > 0.2 m.
    for w in w3_tree_maze w4_maze_rooms; do for r in 1 2 3; do scripts/run_test.sh A1.5 $w gps 1800; done; done"""
import sys
from a1_common import run
sys.exit(run('A1.5', 'w3_tree_maze', yaw='fixed', worlds=['w3_tree_maze', 'w4_maze_rooms']))
