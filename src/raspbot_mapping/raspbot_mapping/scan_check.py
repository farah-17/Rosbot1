"""Print the lidar distance straight ahead, left, behind and right, once a second.

Use it to check the lidar works and which way its 0 degree direction points:
put a box 30 cm in front of the robot and the FRONT value should drop.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DIRECTIONS = {'FRONT': 0.0, 'LEFT': 90.0, 'BACK': 180.0, 'RIGHT': -90.0}


class ScanCheck(Node):

    def __init__(self):
        super().__init__('scan_check')
        self.scan = None
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def on_scan(self, msg):
        self.scan = msg

    def distance_at(self, degrees):
        """Smallest valid range within +/-3 degrees of the given direction."""
        s = self.scan
        best = math.inf
        for i, r in enumerate(s.ranges):
            angle = math.degrees(s.angle_min + i * s.angle_increment)
            diff = (angle - degrees + 180.0) % 360.0 - 180.0
            if abs(diff) <= 3.0 and s.range_min < r < s.range_max:
                best = min(best, r)
        return best

    def report(self):
        if self.scan is None:
            self.get_logger().warn('No /scan received yet. Is the lidar driver running?')
            return
        s = self.scan
        valid = sum(1 for r in s.ranges if s.range_min < r < s.range_max)
        parts = [f'{name} {self.distance_at(deg):5.2f} m' for name, deg in DIRECTIONS.items()]
        print(f'{len(s.ranges)} points ({valid} valid) | ' + ' | '.join(parts), flush=True)


def main():
    rclpy.init()
    node = ScanCheck()
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
