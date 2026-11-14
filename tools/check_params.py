#!/usr/bin/env python3
"""Check parameter files against the running SITL: every name must exist and hold its value.

SITL ignores unknown names in --add-param-file files without a word, and ArduPilot renames
parameters between versions (4.7: SRn_* -> MAV<n+1>_*, ARMING_CHECK -> ARMING_SKIPCHK,
WPNAV_SPEED -> WP_SPD in m/s, ...), so a profile can silently do less than it says.

    ./run_sitl.sh --gazebo --nav extnav --fc truf      # first
    python3 tools/check_params.py params/*.parm         # MAVLink on TCP 5766 (Mission Planner port)
    python3 tools/check_params.py --url tcp:127.0.0.1:5762 params/fc_truf.parm

Needs pymavlink (~/venv-ardupilot). Exit status 1 if anything is missing or different.
"""
import argparse
import sys
import time

from pymavlink import mavutil


def read_file(path):
    """{name: value} of an ArduPilot parameter file (NAME VALUE or NAME,VALUE; # comments)."""
    out = {}
    for line in open(path):
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.replace(',', ' ').split()
        if len(parts) >= 2:
            out[parts[0]] = float(parts[1])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+')
    ap.add_argument('--url', default='tcp:127.0.0.1:5766')
    a = ap.parse_args()

    m = mavutil.mavlink_connection(a.url)
    if m.wait_heartbeat(timeout=20) is None:
        sys.exit(f'No heartbeat on {a.url}: is SITL running?')
    m.mav.param_request_list_send(m.target_system, m.target_component)
    params, total, t0 = {}, None, time.time()
    while time.time() - t0 < 90:
        r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if r is None:
            break
        params[r.param_id] = r.param_value
        total = r.param_count
        if total and len(params) >= total:
            break
    print(f'{len(params)} of {total} parameters read from {a.url}')

    bad = 0
    for path in a.files:
        for name, want in read_file(path).items():
            if name not in params:
                print(f'MISSING   {path}: {name} (unknown to this ArduPilot version: renamed?)')
                bad += 1
            elif abs(params[name] - want) > 1e-3 * max(1.0, abs(want)):
                # A later file may set it differently on purpose (e.g. fc_truf.parm after sitl_ports.parm).
                print(f'DIFFERENT {path}: {name} = {params[name]:g}, file says {want:g}')
                bad += 1
    print('all parameters set' if not bad else f'{bad} problem(s)')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
