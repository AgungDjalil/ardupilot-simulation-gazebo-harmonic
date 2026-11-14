"""Ground truth for RViz (started by run_ros_bridge.sh): the obstacles of the running world as a point
cloud, /sim/world_cloud, in frame sim_origin = the drone's spawn point (x forward at spawn), the frame
of /sim/truth (sim_camera.py).

The collisions (<cylinder>, <box>, <sphere>, <ellipsoid>) of static models are sampled at
`resolution`, with full model/link/collision poses (tilted trunks, fallen logs); meshes (cave,
park) are not. Algorithms must NOT use this: on the real drone there is none.

    python3 ros/world_map.py      # world and spawn point from /tmp/ardupilot_sim/current.json
"""
import json
import math
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def sample_cylinder(r, length, res):
    """Surface points of a cylinder along z, centred at the origin."""
    n_ang = max(8, int(2 * math.pi * r / res))
    angles = np.linspace(0, 2 * math.pi, n_ang, endpoint=False)
    zs = np.arange(-length / 2, length / 2 + 1e-6, res)
    a, z = np.meshgrid(angles, zs)
    return np.stack([r * np.cos(a).ravel(), r * np.sin(a).ravel(), z.ravel()], axis=1)


def sample_box(sx, sy, sz, res):
    xs = np.arange(-sx / 2, sx / 2 + 1e-6, res)
    ys = np.arange(-sy / 2, sy / 2 + 1e-6, res)
    zs = np.arange(-sz / 2, sz / 2 + 1e-6, res)
    x, y, z = np.meshgrid(xs, ys, zs, indexing='ij')
    return np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1)


def sample_ellipsoid(a, b, c, res):
    n_lat = max(4, int(math.pi * max(a, b, c) / res))
    pts = []
    for t in np.linspace(0, math.pi, n_lat + 1):
        ring = max(a, b) * math.sin(t)
        n_lon = max(1, int(2 * math.pi * ring / res))
        p = np.linspace(0, 2 * math.pi, n_lon, endpoint=False)
        pts.append(np.stack([a * math.sin(t) * np.cos(p), b * math.sin(t) * np.sin(p),
                             np.full(n_lon, c * math.cos(t))], axis=1))
    return np.concatenate(pts)


def pose_matrix(elem):
    """4x4 transform of an SDF <pose> element (x y z roll pitch yaw; degrees="true" honoured)."""
    T = np.eye(4)
    if elem is None or not (elem.text or '').strip():
        return T
    v = [float(x) for x in elem.text.split()] + [0.0] * 6
    if elem.get('degrees', 'false').lower() in ('true', '1'):
        v[3:6] = [math.radians(a) for a in v[3:6]]
    r, p, y = v[3:6]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = v[:3]
    return T


def load_obstacles(path, res):
    clouds = []
    for model in ET.parse(path).getroot().iter('model'):
        if (model.findtext('static') or '').strip().lower() not in ('true', '1'):
            continue
        Tm = pose_matrix(model.find('pose'))
        for link in model.findall('link'):
            Tl = Tm @ pose_matrix(link.find('pose'))
            for col in link.findall('collision'):
                geom = col.find('geometry')
                if geom is None:
                    continue
                cyl, box, sph, ell = (geom.find(t) for t in ('cylinder', 'box', 'sphere', 'ellipsoid'))
                if cyl is not None:
                    pts = sample_cylinder(float(cyl.findtext('radius')), float(cyl.findtext('length')), res)
                elif box is not None:
                    pts = sample_box(*(float(v) for v in box.findtext('size').split()), res)
                elif sph is not None:
                    r = float(sph.findtext('radius'))
                    pts = sample_ellipsoid(r, r, r, res)
                elif ell is not None:
                    pts = sample_ellipsoid(*(float(v) for v in ell.findtext('radii').split()), res)
                else:
                    continue
                T = Tl @ pose_matrix(col.find('pose'))
                clouds.append(pts @ T[:3, :3].T + T[:3, 3])
    return np.concatenate(clouds).astype(np.float32) if clouds else np.zeros((0, 3), np.float32)


class WorldMap(Node):

    def __init__(self):
        super().__init__('world_map')
        self.declare_parameter('current', '/tmp/ardupilot_sim/current.json')
        self.declare_parameter('resolution', 0.1)
        # Only what matters for flight (tall trees would make a several-MB cloud every second).
        self.declare_parameter('max_height', 4.0)
        cur = json.load(open(self.get_parameter('current').value))
        path, spawn = cur['world_source'], cur['spawn']
        try:
            points = load_obstacles(path, self.get_parameter('resolution').value)
        except ET.ParseError as e:
            self.get_logger().warn(f'{path}: {e}; no obstacle cloud')
            points = np.zeros((0, 3), np.float32)
        points = points[points[:, 2] <= self.get_parameter('max_height').value]
        # Gazebo world -> sim_origin (the spawn point, yaw removed)
        yaw = math.radians(spawn['yaw_deg'])
        c, s = math.cos(-yaw), math.sin(-yaw)
        d = points - np.array([spawn['x'], spawn['y'], spawn['z']], np.float32)
        points = np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1], d[:, 2]], axis=1).astype(np.float32)
        self.get_logger().info(f'{len(points)} obstacle points from {path}')

        self.msg = PointCloud2(
            header=Header(frame_id='sim_origin'),
            height=1, width=len(points), is_bigendian=False, is_dense=True,
            point_step=12, row_step=12 * len(points), data=points.tobytes(),
            fields=[PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate('xyz')])
        self.pub = self.create_publisher(PointCloud2, '/sim/world_cloud', 1)
        self.create_timer(1.0, self.publish)

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = WorldMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
