"""Set the EKF origin of an ArduPilot vehicle that flies without GPS.

Without GPS the EKF has no global origin, so it reports no position (no arming in
GUIDED/LOITER, no map position in QGroundControl). This sends SET_GPS_GLOBAL_ORIGIN until the
autopilot confirms it with GPS_GLOBAL_ORIGIN. Started by run_sitl.sh --gps-denied.

  python3 set_ekf_origin.py [--url tcp:127.0.0.1:5766] [--lat LAT --lon LON] [--alt ALT]

Without --lat/--lon the simulator's own position (SIMSTATE) is used.
"""
import argparse
import sys
import time

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='tcp:127.0.0.1:5766')
    ap.add_argument('--lat', type=float)
    ap.add_argument('--lon', type=float)
    ap.add_argument('--alt', type=float, default=584.0, help='origin altitude AMSL (m)')
    ap.add_argument('--timeout', type=float, default=180)
    args = ap.parse_args()

    deadline = time.monotonic() + args.timeout
    conn = None
    while conn is None:
        try:
            conn = mavutil.mavlink_connection(args.url, source_system=251)
        except OSError:
            if time.monotonic() > deadline:
                sys.exit(f'set_ekf_origin: cannot connect to {args.url}')
            time.sleep(1)
    if conn.wait_heartbeat(timeout=args.timeout) is None:
        sys.exit('set_ekf_origin: no heartbeat')

    lat, lon = args.lat, args.lon
    if lat is None or lon is None:
        conn.mav.request_data_stream_send(conn.target_system, conn.target_component,
                                          mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)
        sim = conn.recv_match(type='SIMSTATE', blocking=True, timeout=30)
        if sim is None:
            sys.exit('set_ekf_origin: no SIMSTATE, pass --lat/--lon')
        lat, lon = sim.lat / 1e7, sim.lng / 1e7

    # The EKF accepts the origin once it is running on ExtNav, which takes a few seconds.
    while time.monotonic() < deadline:
        conn.mav.set_gps_global_origin_send(conn.target_system, int(lat * 1e7), int(lon * 1e7),
                                            int(args.alt * 1000))
        if conn.recv_match(type='GPS_GLOBAL_ORIGIN', blocking=True, timeout=2) is not None:
            print(f'set_ekf_origin: EKF origin set to {lat:.7f}, {lon:.7f}, {args.alt:.1f} m')
            return
    sys.exit('set_ekf_origin: origin not confirmed')


if __name__ == '__main__':
    main()
