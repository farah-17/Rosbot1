"""ROS 2 node: a pretend Raspbot, for testing navigation without the real robot.

    listens to  /cmd_vel       moves a virtual robot (no wheels, no lidar needed)
                /initialpose   "put the robot here" (GUI start point or RViz)
    publishes   TF map -> odom -> base_link   where the virtual robot is
                /scan          a fake lidar scan, worked out from the map
                /sim_pose      the true position (to compare with Cartographer)

With publish_tf:=false it only publishes /scan, so Cartographer can work out
the position itself, like on the real robot.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from raspbot_navigation.grid_map import GridMap
from raspbot_navigation.navigator import yaw_from_quaternion


class SimRobot(Node):

    def __init__(self):
        super().__init__('sim_robot')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('scan_frame', 'base_link')
        self.declare_parameter('scan_range', 8.0)
        self.declare_parameter('speed_scale', 1.0)   # <1: robot slower than commanded

        self.world = GridMap(self.get_parameter('map_yaml').value)
        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = self.get_parameter('start_yaw').value
        self.v = self.w = 0.0

        self.tf = TransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, 'sim_pose', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_initial_pose, 10)
        self.dt = 0.05
        self.create_timer(self.dt, self.step)        # move 20 times a second
        self.create_timer(0.1, self.publish_scan)     # scan 10 times a second
        self.get_logger().info(f'sim_robot at ({self.x:.2f}, {self.y:.2f})')

    def on_cmd_vel(self, msg):
        self.v, self.w = msg.linear.x, msg.angular.z

    def on_initial_pose(self, msg):
        p = msg.pose.pose
        self.x, self.y = p.position.x, p.position.y
        self.yaw = yaw_from_quaternion(p.orientation)
        self.v = self.w = 0.0
        self.get_logger().info(f'moved to start ({self.x:.2f}, {self.y:.2f})')

    def step(self):
        scale = self.get_parameter('speed_scale').value
        self.yaw += self.w * scale * self.dt
        nx = self.x + self.v * scale * math.cos(self.yaw) * self.dt
        ny = self.y + self.v * scale * math.sin(self.yaw) * self.dt
        if not self.world.is_wall(*self.world.world_to_cell(nx, ny)):
            self.x, self.y = nx, ny                  # walls stop the robot
        now = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = now, 'map'
        pose.pose.position.x, pose.pose.position.y = self.x, self.y
        pose.pose.orientation.z = math.sin(self.yaw / 2)
        pose.pose.orientation.w = math.cos(self.yaw / 2)
        self.pose_pub.publish(pose)
        if self.get_parameter('publish_tf').value:
            self.tf.sendTransform([
                self.transform(now, 'map', 'odom', 0.0, 0.0, 0.0),
                self.transform(now, 'odom', 'base_link', self.x, self.y, self.yaw)])

    @staticmethod
    def transform(stamp, parent, child, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return t

    def publish_scan(self):
        """Fake lidar: walk along 360 rays until each one hits a wall."""
        max_range = self.get_parameter('scan_range').value
        step = self.world.resolution
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.get_parameter('scan_frame').value
        scan.angle_min, scan.angle_max = -math.pi, math.pi - math.radians(1)
        scan.angle_increment = math.radians(1)
        scan.scan_time, scan.time_increment = 0.1, 0.1 / 360
        scan.range_min, scan.range_max = 0.05, max_range
        for i in range(360):
            angle = self.yaw + scan.angle_min + i * scan.angle_increment
            cos_a, sin_a = math.cos(angle), math.sin(angle)
            r, hit = step, math.inf
            while r < max_range:
                cell = self.world.world_to_cell(self.x + r * cos_a, self.y + r * sin_a)
                if self.world.is_wall(*cell):
                    hit = r
                    break
                r += step
            scan.ranges.append(hit)
        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = SimRobot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
