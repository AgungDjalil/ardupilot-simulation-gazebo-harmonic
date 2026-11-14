#!/usr/bin/env python3
"""H1's LiDAR forwarder: the simulated Livox Avia, served the way the LiDAR Pi serves its sensor to the station.

    livox_forwarder.py [--ws-port 8766] [--mavlink tcp:127.0.0.1:5843] [--topic /H1/lidar/points]
                       [--tilt-deg 90] [--live-max-pts 10000]

On the aircraft, Symphony's gcs/velodyne_forwarder.py reads the LiDAR, reads the flight controller's pose over
MAVLink, places every point in local ENU (metres east/north/up from the EKF origin) and sends it over a WebSocket.
The station's lidar_map.py (LIDAR_WS_URL) and the LiDAR tab in the UI both read that socket. This does the same
for the simulation, with the same message, so the station sees no difference:

    {"type": "points", "frame_id": "enu" | "body", "timestamp": s, "points": [[x, y, z, intensity], ...],
     "vehicle_enu": [e, n, u] | null, "vehicle_yaw_deg": deg | null, "origin_gps": {...} | null}

  - points: Gazebo's gpu_lidar cloud on --topic (sensor frame: x along the window's axis, y left, z up), through
    the mount (livox_link in make_world.py: 0.05 m ahead, 0.12 m below the body centre, pitched --tilt-deg down),
    then the pose. "body" (x right, y front, z up) until the first pose arrives, as on the Pi.
  - pose: ATTITUDE and LOCAL_POSITION_NED at 50 Hz from the autopilot, asked for with SET_MESSAGE_INTERVAL as the
    Pi does, on SERIAL2 (the station holds SERIAL1); origin_gps from the first GLOBAL_POSITION_INT.
  - rate: one message per scan (10 Hz), thinned to --live-max-pts points/s (the Pi's default 10 000: its WiFi
    budget); 0 sends all 240 000. Returns under 1 m (the Avia's blind zone) and NaN rays are dropped.

Runs on the gz Python bindings (ROS 2 Jazzy's vendor packages, on PYTHONPATH) and this folder's .venv
(websockets, pymavlink; numpy from the system), which run.sh starts it with.
"""
import argparse
import asyncio
import json
import math
import threading
import time

import numpy as np
import websockets
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
from gz.transport13 import Node
from pymavlink import mavutil

POSE_SYSID = 192          # as velodyne_forwarder.py: not 255, which is what a GCS heartbeats as
MOUNT_T = np.array([0.05, 0.0, -0.12])   # livox_link in the body (x front, y left, z up), make_world.livox_avia
FLU_FRD = np.diag([1.0, -1.0, -1.0])     # body front-left-up <-> front-right-down
MIN_RANGE_M = 1.0


class Pose:
    def __init__(self):
        self.lock = threading.Lock()
        self.att = None        # (roll, pitch, yaw) rad, NED
        self.ned = None        # (north, east, down) m from the EKF origin
        self.origin = None


def pose_reader(url: str, pose: Pose):
    while True:
        try:
            conn = mavutil.mavlink_connection(url, source_system=POSE_SYSID)
            if conn.wait_heartbeat(timeout=10) is None:
                print(f"[pose] no heartbeat on {url}, retrying", flush=True)
                time.sleep(3)
                continue
            print(f"[pose] heartbeat from system {conn.target_system} on {url}", flush=True)
            for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                           mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
                conn.mav.command_long_send(conn.target_system, conn.target_component,
                                           mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, 20_000,
                                           0, 0, 0, 0, 0)
            last = time.time()
            while time.time() - last < 10:
                m = conn.recv_match(type=["ATTITUDE", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"],
                                    blocking=True, timeout=1)
                if m is None:
                    continue
                last = time.time()
                with pose.lock:
                    if m.get_type() == "ATTITUDE":
                        pose.att = (m.roll, m.pitch, m.yaw)
                    elif m.get_type() == "LOCAL_POSITION_NED":
                        pose.ned = (m.x, m.y, m.z)
                    elif pose.origin is None:
                        pose.origin = {"lat": m.lat / 1e7, "lon": m.lon / 1e7, "alt_amsl_m": m.alt / 1000.0,
                                       "relative_alt_m": m.relative_alt / 1000.0}
                        print(f"[pose] origin fix {pose.origin['lat']:.7f}, {pose.origin['lon']:.7f}", flush=True)
            print("[pose] no pose for 10 s, reconnecting", flush=True)
        except Exception as exc:              # the link drops when SITL restarts: keep trying
            print(f"[pose] {exc.__class__.__name__}: {exc}", flush=True)
            time.sleep(3)


def rot_body_to_ned(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                     [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                     [-sp, sr * cp, cr * cp]])


def cloud_xyzi(m: PointCloudPacked) -> np.ndarray:
    off = {f.name: f.offset for f in m.field}
    raw = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
    cols = [raw[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in ("x", "y", "z")]
    cols.append(raw[:, off["intensity"]:off["intensity"] + 4].copy().view(np.float32)[:, 0]
                if "intensity" in off else np.zeros(len(raw), np.float32))
    p = np.stack(cols, 1).astype(np.float64)
    p = p[np.isfinite(p[:, :3]).all(1)]
    return p[np.linalg.norm(p[:, :3], axis=1) >= MIN_RANGE_M]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--ws-port", type=int, default=8766)
    ap.add_argument("--mavlink", default="tcp:127.0.0.1:5843")
    ap.add_argument("--topic", default="/H1/lidar/points")
    ap.add_argument("--tilt-deg", type=float, default=90.0)
    ap.add_argument("--live-max-pts", type=int, default=10_000)
    a = ap.parse_args()

    t = math.radians(a.tilt_deg)     # pitch about the body's y (left): + turns the window down
    mount = np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])
    pose = Pose()
    threading.Thread(target=pose_reader, args=(a.mavlink, pose), daemon=True).start()

    clients: set = set()
    loop = asyncio.new_event_loop()
    stats = {"scans": 0, "sent": 0, "t0": time.time()}

    def build(m: PointCloudPacked):
        p = cloud_xyzi(m)
        stats["scans"] += 1
        if not len(p):
            return None
        if a.live_max_pts:
            limit = max(1, int(a.live_max_pts / 10))          # per scan, at 10 scans/s
            if len(p) > limit:
                p = p[::-(-len(p) // limit)]
        body = p[:, :3] @ mount.T + MOUNT_T                     # front-left-up, body centre
        with pose.lock:
            att, ned, origin = pose.att, pose.ned, pose.origin
        if att is None:
            frame, out = "body", np.column_stack((-body[:, 1], body[:, 0], body[:, 2]))   # right, front, up
            venu, yaw_deg = None, None
        else:
            n = (body @ FLU_FRD.T) @ rot_body_to_ned(*att).T + np.asarray(ned or (0.0, 0.0, 0.0))
            frame, out = "enu", np.column_stack((n[:, 1], n[:, 0], -n[:, 2]))
            pos = ned or (0.0, 0.0, 0.0)
            venu, yaw_deg = [round(pos[1], 3), round(pos[0], 3), round(-pos[2], 3)], round(math.degrees(att[2]), 1)
        pts = np.column_stack((out.round(3), p[:, 3].round(1))).tolist()
        stats["sent"] += len(pts)
        return json.dumps({"type": "points", "frame_id": frame, "timestamp": time.time(), "points": pts,
                           "vehicle_enu": venu, "vehicle_yaw_deg": yaw_deg, "origin_gps": origin})

    def on_cloud(m: PointCloudPacked):
        if not clients:
            stats["scans"] += 1
            return
        msg = build(m)
        if msg is not None:
            loop.call_soon_threadsafe(lambda: [asyncio.ensure_future(c.send(msg)) for c in list(clients)])

    async def serve(ws):
        clients.add(ws)
        print(f"[ws] client {ws.remote_address} ({len(clients)})", flush=True)
        try:
            await ws.wait_closed()
        finally:
            clients.discard(ws)
            print(f"[ws] client left ({len(clients)})", flush=True)

    async def report():
        while True:
            await asyncio.sleep(30)
            dt = time.time() - stats["t0"]
            with pose.lock:
                have = "pose" if pose.att is not None else "no pose yet"
            print(f"[stat] {stats['scans'] / dt:.1f} scans/s, {stats['sent'] / dt:.0f} points/s sent, "
                  f"{len(clients)} clients, {have}", flush=True)
            stats.update(scans=0, sent=0, t0=time.time())

    async def run():
        async with websockets.serve(serve, "0.0.0.0", a.ws_port, max_size=None):
            print(f"[ws] serving ws://0.0.0.0:{a.ws_port}, LiDAR from {a.topic}", flush=True)
            await report()

    node = Node()
    if not node.subscribe(PointCloudPacked, a.topic, on_cloud):
        raise SystemExit(f"cannot subscribe to {a.topic}")
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run())


if __name__ == "__main__":
    main()
