#!/usr/bin/env python3
"""The drone's Intel RealSense D455, simulated (started by run_ros_bridge.sh): Gazebo's camera and IMU
published exactly as realsense2_camera publishes them on the real drone, so an algorithm runs on the
simulation and on the drone without a change.

Topics (as the driver with align_depth, unite_imu_method 2):
- /camera/camera/color/image_raw + camera_info: rgb8 640x480 @15 Hz, camera_color_optical_frame;
- /camera/camera/aligned_depth_to_color/image_raw + camera_info: 16UC1 mm (0 = no depth), same
  frame and stamp as the colour image;
- /camera/camera/imu: gyro + accel (no orientation) at 200 Hz, camera_imu_optical_frame;
- with run_gazebo.sh --infra: /camera/camera/infra1|infra2/image_rect_raw + camera_info (mono8
  848x480, camera_infra1|2_optical_frame, infra2's P[3] = -fx * baseline as rectified stereo);
- static TF camera_link -> the optical frames. camera_link itself is placed by the drone's own
  stack (on the real drone: drone_onboard.launch.py's camera_mount TF, same numbers as the model).
Simulation only (never on the real drone, don't use them in an algorithm):
- /sim/truth (+ TF sim_origin -> sim_truth): Gazebo's true body pose relative to the spawn point
  (x forward at spawn), for measuring drift; /sim/world_cloud comes from world_map.py.

As the real D455 (unless realistic:=false):
- depth noise sigma = depth_noise_k * z^2 (measured on the drone's D455 at 848x480: 7.6 cm at 5-6 m),
  no depth below min_depth / beyond max_depth, random holes (dropout), and flying pixels removed at
  depth edges (edge_ratio) as the D455's stereo matching does;
- IMU: gyro at 200 Hz, accel sampled at 63 Hz and held (the Pi's gyro_fps / accel_fps), white
  noise and a constant bias per run (BMI055 data sheet values), and the accel scale measured on
  the drone's uncalibrated D455 (|g| = 8.82, accel_scale 0.899; 1.0 after rs-imu-calibration).

Stamps: the drone's nodes run on the wall clock and compare stamps with it (latency, TF), while
Gazebo stamps with sim time from 0. Every stamp becomes sim + offset, offset = wall arrival - sim
time of the IMU (~1 ms latency), following real-time-factor changes. The offset is recorded per
sim time, and an image gets the offset of its own sim time (not of its arrival, up to ~1 s later):
an image and the IMU sample of the same instant always get the same stamp, also while Gazebo
stalls and catches up. Images keep their capture time, so the render + network delay shows up as
camera latency, as on the real drone. Gazebo must run at real time (RTF ~1) for the drone's
timing to be realistic: the FC (SITL) runs on sim time, the companion on the wall clock.

    python3 ros/sim_camera.py [--ros-args -p realistic:=false -p accel_scale:=1.0 ...]
"""
import bisect
import collections
import json
import math
import re

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

CAM = '/camera/camera'
COLOR_FRAME = 'camera_color_optical_frame'
IMU_FRAME = 'camera_imu_optical_frame'
INFRA_FRAMES = ('camera_infra1_optical_frame', 'camera_infra2_optical_frame')
INFRA_BASELINE = 0.095  # m, the D455's stereo baseline (models/truf_d455: infra1/2 at y = +-0.0475)
# rpy (-90, 0, -90) deg: camera_link (x forward, y left, z up) -> optical (x right, y down, z forward)
OPTICAL_Q = (-0.5, 0.5, -0.5, 0.5)
CAM_TO_OPTICAL = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)


def rpy_matrix(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll), the SDF / tf2 convention."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def camera_mount(model_sdf):
    """roll, pitch, yaw (rad) of the d455_link in the drone model: the single place it is set."""
    s = open(model_sdf).read()
    link = re.search(r'<link name="d455_link">\s*<pose>([^<]+)</pose>', s)
    v = [float(t) for t in link.group(1).split()]
    return v[3], v[4], v[5]


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SimCamera(Node):

    def __init__(self):
        super().__init__('sim_camera')
        p = self.declare_parameter
        p('current', '/tmp/ardupilot_sim/current.json')
        p('realistic', True)
        p('depth_noise_k', 0.0025)   # sigma = k z^2 (m)
        p('min_depth', 0.45)         # m; the D455's minimum depth at 848x480 is ~0.5 m
        p('max_depth', 10.0)         # m
        p('dropout', 0.02)           # fraction of random holes
        p('edge_ratio', 0.04)        # depth jump to a neighbour (fraction of z) that drops both pixels
        p('imu_rate', 200.0)         # gyro_fps
        p('accel_rate', 63.0)        # accel_fps
        p('gyro_noise', 0.0035)      # rad/s per sample at 200 Hz (BMI055: 0.014 deg/s/sqrt(Hz))
        p('accel_noise', 0.021)      # m/s^2 per sample (BMI055: 150 ug/sqrt(Hz))
        p('gyro_bias', 0.0035)       # rad/s, sd of the constant per-run bias (~0.2 deg/s)
        p('accel_bias', 0.05)        # m/s^2, sd of the constant per-run bias
        p('accel_scale', 0.899)      # the drone's D455 reads |g| = 8.82 (uncalibrated)
        p('truth_rate', 15.0)
        p('seed', 0)                 # 0: different noise every run
        g = lambda name: self.get_parameter(name).value
        self.realistic = g('realistic')
        cur = json.load(open(g('current')))
        roll, pitch, yaw = camera_mount(cur['model_sdf'])
        self.get_logger().info(
            f"world {cur['world_name']}, D455 mount roll {math.degrees(roll):.1f} pitch "
            f"{math.degrees(pitch):.1f} deg, infrared {'on' if cur['infra'] else 'off'}, "
            f"realistic {'on' if self.realistic else 'off'}")
        # Gazebo IMU sensor (FRD: iris_with_standoffs rolls it 180 deg) -> body FLU -> camera_link
        # (the mount) -> optical frame.
        frd_to_flu = np.diag([1.0, -1.0, -1.0])
        self.imu_rot = CAM_TO_OPTICAL @ rpy_matrix(roll, pitch, yaw).T @ frd_to_flu
        self.rng = np.random.default_rng(g('seed') or None)
        r = self.realistic
        self.gyro_bias = self.rng.normal(0, g('gyro_bias'), 3) if r else np.zeros(3)
        self.accel_bias = self.rng.normal(0, g('accel_bias'), 3) if r else np.zeros(3)
        self.accel_scale = g('accel_scale') if r else 1.0
        self.imu_period = 1.0 / g('imu_rate')
        self.accel_period = 1.0 / g('accel_rate')
        self.truth_period = 1.0 / g('truth_rate')

        self.offset = None        # wall - sim (s)
        self.offsets = collections.deque(maxlen=5000)  # (sim, offset) per Gazebo IMU message (1 kHz): 5 s
        self.last_imu_sim = self.last_accel_sim = self.last_truth_sim = -math.inf
        self.accel_held = None
        self.truth_origin = None  # (x, y, z, yaw) of the first ground-truth pose

        # Reliable, as realsense2_camera's default: matches reliable and best-effort readers.
        self.color_pub = self.create_publisher(Image, f'{CAM}/color/image_raw', 5)
        self.color_info_pub = self.create_publisher(CameraInfo, f'{CAM}/color/camera_info', 5)
        self.depth_pub = self.create_publisher(Image, f'{CAM}/aligned_depth_to_color/image_raw', 5)
        self.depth_info_pub = self.create_publisher(CameraInfo, f'{CAM}/aligned_depth_to_color/camera_info', 5)
        self.imu_pub = self.create_publisher(Imu, f'{CAM}/imu', 50)
        self.truth_pub = self.create_publisher(Odometry, '/sim/truth', 10)

        # From ros_gz_bridge (run_ros_bridge.sh), under /sim/gz.
        self.create_subscription(Imu, '/sim/gz/imu', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/sim/gz/image', self.color_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/sim/gz/depth', self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/sim/gz/camera_info', self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/gz/odometry', self.truth_cb, qos_profile_sensor_data)
        static = [self.tf('camera_link', COLOR_FRAME, q=OPTICAL_Q), self.tf('camera_link', IMU_FRAME, q=OPTICAL_Q)]
        self.infra_pubs = []
        if cur['infra']:
            for i, frame in enumerate(INFRA_FRAMES):
                n = i + 1
                pubs = (self.create_publisher(Image, f'{CAM}/infra{n}/image_rect_raw', 5),
                        self.create_publisher(CameraInfo, f'{CAM}/infra{n}/camera_info', 5))
                self.infra_pubs.append(pubs)
                self.create_subscription(Image, f'/sim/gz/infra{n}', lambda m, i=i: self.infra_cb(i, m),
                                         qos_profile_sensor_data)
                self.create_subscription(CameraInfo, f'/sim/gz/infra{n}/camera_info',
                                         lambda m, i=i: self.infra_info_cb(i, m), qos_profile_sensor_data)
                static.append(self.tf('camera_link', frame, y=INFRA_BASELINE / 2 * (1 if i == 0 else -1),
                                      q=OPTICAL_Q))
        StaticTransformBroadcaster(self).sendTransform(static)
        self.tf_pub = TransformBroadcaster(self)
        self.counts = {'color': 0, 'depth': 0, 'imu': 0, 'infra': 0}
        self.create_timer(10.0, self.report)

    @staticmethod
    def tf(parent, child, y=0.0, q=(0.0, 0.0, 0.0, 1.0)):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.y = y
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = q
        return t

    @staticmethod
    def seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def to_wall(self, stamp):
        """Sim stamp -> wall stamp with the offset recorded at that sim time (None before the IMU)."""
        if not self.offsets:
            return None
        sim = self.seconds(stamp)
        i = bisect.bisect_left(self.offsets, (sim, -math.inf))
        offset = self.offsets[min(i, len(self.offsets) - 1)][1]
        return Time(nanoseconds=int((sim + offset) * 1e9)).to_msg()

    # -------------------------------------------------------------------- IMU
    def imu_cb(self, msg):
        sim = self.seconds(msg.header.stamp)
        candidate = self.get_clock().now().nanoseconds * 1e-9 - sim
        # Lowest latency wins at once; a growing offset (sim slower than real time, or a pause)
        # is followed within ~0.25 s at 1 kHz.
        if self.offset is None or candidate < self.offset:
            self.offset = candidate
        else:
            self.offset += 0.004 * (candidate - self.offset)
        if self.offsets and sim < self.offsets[-1][0]:
            self.offsets.clear()  # Gazebo restarted
        self.offsets.append((sim, self.offset))
        a_v = msg.linear_acceleration
        if self.accel_held is None or sim - self.last_accel_sim >= self.accel_period - 1e-4:
            self.last_accel_sim = sim
            accel = self.imu_rot @ [a_v.x, a_v.y, a_v.z]
            if self.realistic:
                accel = self.accel_scale * accel + self.accel_bias + self.rng.normal(
                    0, self.get_parameter('accel_noise').value, 3)
            self.accel_held = accel
        if sim - self.last_imu_sim < self.imu_period - 1e-4:
            return
        self.last_imu_sim = sim
        w = msg.angular_velocity
        gyro = self.imu_rot @ [w.x, w.y, w.z]
        if self.realistic:
            gyro = gyro + self.gyro_bias + self.rng.normal(0, self.get_parameter('gyro_noise').value, 3)
        out = Imu()
        out.header.stamp = self.to_wall(msg.header.stamp)
        out.header.frame_id = IMU_FRAME
        out.orientation_covariance[0] = -1.0  # no orientation, as the D455
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = map(float, gyro)
        a = out.linear_acceleration
        a.x, a.y, a.z = map(float, self.accel_held)
        self.imu_pub.publish(out)
        self.counts['imu'] += 1

    # ------------------------------------------------------------- colour/depth
    def color_cb(self, msg):
        stamp = self.to_wall(msg.header.stamp)
        if stamp is None:
            return
        msg.header.stamp, msg.header.frame_id = stamp, COLOR_FRAME
        self.color_pub.publish(msg)
        self.counts['color'] += 1

    def d455_depth(self, z):
        """Exact depth (m, float32) -> what the D455 would measure (m, 0 = no depth)."""
        g = lambda name: self.get_parameter(name).value
        z = np.nan_to_num(z, nan=0.0, posinf=0.0, neginf=0.0)
        valid = (z >= g('min_depth')) & (z <= g('max_depth'))
        # Flying pixels / stereo holes at depth edges: drop both sides of a large jump.
        edge = np.zeros(z.shape, bool)
        jump_x = np.abs(np.diff(z, axis=1)) > g('edge_ratio') * np.minimum(z[:, 1:], z[:, :-1])
        jump_y = np.abs(np.diff(z, axis=0)) > g('edge_ratio') * np.minimum(z[1:, :], z[:-1, :])
        edge[:, 1:] |= jump_x
        edge[:, :-1] |= jump_x
        edge[1:, :] |= jump_y
        edge[:-1, :] |= jump_y
        valid &= ~edge
        valid &= self.rng.random(z.shape, dtype=np.float32) >= g('dropout')
        noisy = z + self.rng.standard_normal(z.shape, dtype=np.float32) * (g('depth_noise_k') * z * z)
        return np.where(valid, noisy, 0.0)

    def depth_cb(self, msg):
        stamp = self.to_wall(msg.header.stamp)
        if stamp is None:
            return
        if msg.encoding != '32FC1':
            self.get_logger().error(f'Unexpected depth encoding {msg.encoding}', throttle_duration_sec=10.0)
            return
        z = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        if self.realistic:
            z = self.d455_depth(z)
        mm = np.clip(np.nan_to_num(z * 1000.0, nan=0.0, posinf=0.0, neginf=0.0), 0, 65535).astype(np.uint16)
        out = Image()
        out.header.stamp, out.header.frame_id = stamp, COLOR_FRAME
        out.height, out.width, out.encoding = msg.height, msg.width, '16UC1'
        out.is_bigendian, out.step = 0, msg.width * 2
        out.data = mm.tobytes()
        self.depth_pub.publish(out)
        self.counts['depth'] += 1

    def info_cb(self, msg):
        stamp = self.to_wall(msg.header.stamp)
        if stamp is None:
            return
        msg.header.stamp, msg.header.frame_id = stamp, COLOR_FRAME
        self.color_info_pub.publish(msg)
        self.depth_info_pub.publish(msg)

    # ----------------------------------------------------------------- infrared
    def infra_cb(self, i, msg):
        stamp = self.to_wall(msg.header.stamp)
        if stamp is None:
            return
        msg.header.stamp, msg.header.frame_id = stamp, INFRA_FRAMES[i]
        self.infra_pubs[i][0].publish(msg)
        self.counts['infra'] += 1

    def infra_info_cb(self, i, msg):
        stamp = self.to_wall(msg.header.stamp)
        if stamp is None:
            return
        msg.header.stamp, msg.header.frame_id = stamp, INFRA_FRAMES[i]
        p = list(msg.p)
        p[3] = -p[0] * INFRA_BASELINE if i == 1 else 0.0  # rectified stereo: Tx of the right camera
        msg.p = p
        self.infra_pubs[i][1].publish(msg)

    # ------------------------------------------------------------- ground truth
    def truth_cb(self, msg):
        sim = self.seconds(msg.header.stamp)
        stamp = self.to_wall(msg.header.stamp)
        if stamp is None or sim - self.last_truth_sim < self.truth_period - 1e-4:
            return
        self.last_truth_sim = sim
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if self.truth_origin is None:
            self.truth_origin = (p.x, p.y, p.z, quat_to_yaw(q))
        x0, y0, z0, yaw0 = self.truth_origin
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        dx, dy = p.x - x0, p.y - y0
        # Rotate the orientation by -yaw0 about z: q' = qz(-yaw0) * q
        hz, hw = math.sin(-yaw0 / 2), math.cos(-yaw0 / 2)
        out = Odometry()
        out.header.stamp, out.header.frame_id = stamp, 'sim_origin'
        out.child_frame_id = 'sim_truth'
        pos = out.pose.pose.position
        pos.x, pos.y, pos.z = c * dx - s * dy, s * dx + c * dy, p.z - z0
        o = out.pose.pose.orientation
        o.w = hw * q.w - hz * q.z
        o.x = hw * q.x - hz * q.y
        o.y = hw * q.y + hz * q.x
        o.z = hw * q.z + hz * q.w
        self.truth_pub.publish(out)
        t = self.tf('sim_origin', 'sim_truth', q=(o.x, o.y, o.z, o.w))
        t.header.stamp = stamp
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pos.x, pos.y, pos.z
        self.tf_pub.sendTransform(t)

    def report(self):
        if not self.offsets:
            self.get_logger().warn('No IMU from Gazebo yet (is the simulation running / SITL started?)')
            return
        c = self.counts
        self.get_logger().info(f"last 10 s: color {c['color']}, depth {c['depth']}, imu {c['imu']}"
                               + (f", infrared {c['infra']}" if self.infra_pubs else ''))
        self.counts = dict.fromkeys(self.counts, 0)


def main():
    rclpy.init()
    node = SimCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
