"""Fly a lawnmower survey through EGO-Planner so RTAB-Map maps the whole simulated world.

Goals go to /goal_pose like RViz's "2D Goal Pose" and EGO-Planner flies around the obstacles. Each
next goal is sent once the drone has stopped at the current one and (spin:=true) turned once in
place: on a lawnmower the drone sees each place from one direction per pass, and from the opposite
side things look different, so RTAB-Map rarely recognises them. Looking around at each waypoint
shows it places again from the directions seen before (loop closures that correct the odometry
drift).

Runs against the drone's onboard stack (the Pi's ~/drone/run_onboard.sh --sim, or
ego_ws/run_onboard_local.sh), in its onboard ROS domain 0 (same DDS settings as
ardupilot_sim/run_ros_bridge.sh). Start it after ./drone_cmd.sh takeoff: the onboard bridge
refuses goals on the ground. Waypoints are in the planner's frame (where the visual odometry
started). Simulation tool only.

    ros2 run ego_ardupilot_sim survey
    ros2 run ego_ardupilot_sim survey --ros-args -p spacing:=3.0 -p y_min:=-6.0 -p y_max:=6.0
    ros2 run ego_ardupilot_sim survey --ros-args -p spin:=false
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty, Float64


def lawnmower(x_min, x_max, y_min, y_max, spacing):
    """Passes along x, alternating direction, stepping `spacing` in y."""
    n = max(1, int(math.floor((y_max - y_min) / spacing + 1e-6)) + 1)
    waypoints = []
    for i in range(n):
        y = min(y_min + i * spacing, y_max)
        xs = (x_min, x_max) if i % 2 == 0 else (x_max, x_min)
        waypoints += [(xs[0], y), (xs[1], y)]
    return waypoints


class Survey(Node):

    def __init__(self):
        super().__init__('survey')
        # Default area: the pillar forest of ego_forest.sdf (x 4..22, y -7..7) plus a margin.
        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 26.0)
        self.declare_parameter('y_min', -8.0)
        self.declare_parameter('y_max', 8.0)
        self.declare_parameter('spacing', 4.0)  # camera: 90 deg HFOV, mapped out to ~5 m
        self.declare_parameter('return_home', True)
        self.declare_parameter('reach_tolerance', 0.5)
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('spin', True)
        self.declare_parameter('spin_deg', 360.0)
        self.declare_parameter('spin_timeout', 60.0)
        g = lambda name: self.get_parameter(name).value

        self.waypoints = lawnmower(g('x_min'), g('x_max'), g('y_min'), g('y_max'), g('spacing'))
        if g('return_home'):
            self.waypoints.append((0.0, 0.0))
        self.tol = g('reach_tolerance')
        self.timeout = g('goal_timeout')
        self.spin_angle = math.radians(g('spin_deg')) if g('spin') else 0.0
        self.spin_timeout = g('spin_timeout')

        self.odom = None
        self.index = -1
        self.sent_at = None
        self.spin_sent_at = None  # set while waiting for the bridge to finish a spin
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.spin_pub = self.create_publisher(Float64, '/ego_ardupilot/spin', 1)
        self.create_subscription(Empty, '/ego_ardupilot/spin_done', self.spin_done_cb, 1)
        # The onboard bridge's planning odometry (the FC's EKF, ego_onboard.launch.py)
        self.declare_parameter('odom_topic', '/drone_0_odom')
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
        self.create_timer(0.5, self.step)
        self.get_logger().info(
            f'Survey with {len(self.waypoints)} waypoints: '
            + ' '.join(f'({x:.0f},{y:.0f})' for x, y in self.waypoints))

    def odom_cb(self, msg):
        self.odom = msg

    def spin_done_cb(self, _):
        if self.spin_sent_at is not None:
            self.spin_sent_at = None
            self.next()

    def next(self):
        if self.index + 1 < len(self.waypoints):
            self.send(self.index + 1)
        else:
            self.get_logger().info('Survey finished. With save:=true on the onboard stack, export the '
                                   'map with ./export_rtabmap_map.sh after stopping it')
            raise SystemExit

    def send(self, index):
        self.index = index
        x, y = self.waypoints[index]
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, 1.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.sent_at = self.get_clock().now()
        self.get_logger().info(f'Waypoint {index + 1}/{len(self.waypoints)}: ({x:.1f}, {y:.1f})')

    def step(self):
        if self.index < 0:
            if self.goal_pub.get_subscription_count() > 0:  # wait for the bridge
                self.send(0)
            return
        if self.spin_sent_at is not None:
            if (self.get_clock().now() - self.spin_sent_at).nanoseconds * 1e-9 > self.spin_timeout:
                self.get_logger().warn(f'Spin at waypoint {self.index + 1} not finished in '
                                       f'{self.spin_timeout:.0f} s, going on')
                self.spin_sent_at = None
                self.next()
            return
        if self.odom is None:
            return
        x, y = self.waypoints[self.index]
        p, v = self.odom.pose.pose.position, self.odom.twist.twist.linear
        reached = math.hypot(p.x - x, p.y - y) < self.tol and math.hypot(v.x, v.y) < 0.2
        waited = (self.get_clock().now() - self.sent_at).nanoseconds * 1e-9
        if not reached and waited < self.timeout:
            return
        if not reached:
            self.get_logger().warn(f'Waypoint {self.index + 1} not reached in {self.timeout:.0f} s, '
                                   'skipping it')
        if reached and self.spin_angle:
            self.get_logger().info(f'Looking around at waypoint {self.index + 1}')
            self.spin_sent_at = self.get_clock().now()
            self.spin_pub.publish(Float64(data=self.spin_angle))
            return
        self.next()


def main():
    rclpy.init()
    node = Survey()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
