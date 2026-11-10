#!/usr/bin/env python3
"""param_check.py - ask the running ArduPilot SITL for parameters and compare with our .parm files.

Reads every NAME VALUE line from the given .parm files, requests each parameter over MAVLink
and prints OK / DIFFERENT / NOT FOUND. NOT FOUND means the name does not exist in this
ArduPilot version (for example a Copter 4.7 rename the guidebook got wrong).

Usage (simulation running, profile gps loads base + gps):
  python3 param_check.py drone_bringup/params/ardupilot/sitl_base.parm
  python3 param_check.py --names WPNAV_SPEED WP_SPD      # just look names up
Connects to SITL's spare port tcp:127.0.0.1:5763 (SERIAL2) so MAVProxy and the bridge are untouched.
Exit code 0 when everything is OK.
"""
import argparse
import sys
import time

from pymavlink import mavutil


def read_parm(path):
    out = []
    for line in open(path):
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.replace(',', ' ').split()
        if len(parts) >= 2:
            out.append((parts[0], float(parts[1])))
    return out


def get_param(m, name, timeout=3.0):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg is None:
            continue
        pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
        if pid.rstrip('\x00') == name:
            return msg.param_value
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs='*', help='.parm files to verify')
    ap.add_argument('--names', nargs='*', default=[], help='extra names to look up (no expected value)')
    ap.add_argument('--url', default='tcp:127.0.0.1:5763')
    args = ap.parse_args()

    wanted = []
    for f in args.files:
        wanted += [(n, v, f.split('/')[-1]) for n, v in read_parm(f)]
    wanted += [(n, None, 'lookup') for n in args.names]
    if not wanted:
        ap.print_help()
        return 1

    print(f'connecting to {args.url} ...')
    m = mavutil.mavlink_connection(args.url, source_system=250, source_component=191)
    if m.wait_heartbeat(timeout=15) is None:
        print('no heartbeat: is the simulation running (scripts/sim_up.sh)?')
        return 1
    ok = True
    for name, expected, src in wanted:
        val = get_param(m, name)
        if val is None:
            status, ok = 'NOT FOUND', False
            shown = '-'
        else:
            shown = f'{val:g}'
            if expected is None:
                status = 'exists'
            elif abs(val - expected) <= 1e-4 * max(1.0, abs(expected)):
                status = 'OK'
            else:
                status, ok = f'DIFFERENT (file says {expected:g})', False
        print(f'{status:32s} {name:18s} = {shown:10s} [{src}]')
    print('\nALL OK' if ok else '\nPROBLEMS FOUND (see above)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
