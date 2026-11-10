#!/usr/bin/env python3
"""A1.1 Static settle: W0, gps, drone on the ground, no arming. Records ~75 s so eval_run can
judge seconds 15-60 of Cartographer's output: drift < 1 cm and < 0.2 deg; zero phantom cells.
    scripts/run_test.sh A1.1 w0_box_room gps"""
import argparse
import os
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--world', default='w0_box_room')
args, _ = ap.parse_known_args()
with open(os.path.join(args.out, 'meta.txt'), 'a') as f:
    f.write('speed=0\nlaps=0\nyaw_mode=none\n')
print('A1.1: recording 75 s with the drone on the ground ...')
for k in range(15):
    time.sleep(5)
    print(f'  {5 * (k + 1)} s')
sys.exit(0)
