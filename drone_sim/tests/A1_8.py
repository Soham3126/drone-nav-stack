#!/usr/bin/env python3
"""A1.8 Loop consistency without loop closure: W5, gps, loop_lap (every cell, so the drone comes back
round the island and sees the loop's walls a second time), heading held, 3 repeats.
Pass: wall doubling < 0.10 m (99th percentile distance of mapped walls from the true walls, whole map)
and the narrowest seen corridor in /map >= 0.80 m wide; 3 of 3 runs.
    for r in 1 2 3; do scripts/run_test.sh A1.8 w5_maze_loop gps 1800; done"""
import sys
from a1_common import run
sys.exit(run('A1.8', 'w5_maze_loop', yaw='fixed', worlds=['w5_maze_loop']))
