#!/usr/bin/env python3
"""A1.3 Plain-corridor slip: W1, gps, corridor_out_back at 0.5 and 1.0 m/s, 3 repeats each.
Pass (initial): max lengthwise error < 0.30 m, return error < 0.15 m, sideways error < 0.05 m.
    for s in 0.5 1.0; do for r in 1 2 3; do SPEED=$s scripts/run_test.sh A1.3 w1_corridor_plain gps; done; done
The translation_weight sweep replays these bags (stage 4: carto_replay)."""
import sys
from a1_common import run
sys.exit(run('A1.3', 'w1_corridor_plain', yaw='fixed', worlds=['w1_corridor_plain']))
