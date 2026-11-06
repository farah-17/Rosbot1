#!/usr/bin/env python3
"""Check that the heading Cartographer reports is the way the robot REALLY moves.

If the robot "goes the wrong way", one cause is a lidar mounting angle (laser_yaw)
that does not match the real robot: the estimated heading is then turned by a
fixed angle compared with the robot's true heading. This script finds out.

    Run it with navigate.launch.py running and the start position already set.
    Make sure there is at least 0.5 m of free floor IN FRONT of the robot.

    python3 check_heading.py

It reads the pose, drives straight forward about 0.2 m at 0.08 m/s, reads the
pose again, and compares the direction it moved with the reported heading.
Ctrl+C stops the robot at any time.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

SPEED = 0.08      # m/s
DRIVE_TIME = 2.5  # s  -> about 0.2 m


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Check(Node):

    def __init__(self):
        super().__init__('check_heading')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def wait_pose(self, seconds=6.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            p = self.pose()
            if p:
                return p
        return None

    def settle(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = Check()
    try:
        node.settle(1.0)
        before = node.wait_pose()
        if before is None:
            print('No transform map -> base_link. Set the start position first.')
            return
        print(f'Before: x={before[0]:.2f} y={before[1]:.2f} heading={math.degrees(before[2]):.0f} deg')
        input('The robot will drive FORWARD about 20 cm. Is the floor in front clear? '
              'Press Enter to go (Ctrl+C to cancel) ... ')
        cmd = Twist()
        cmd.linear.x = SPEED
        end = time.monotonic() + DRIVE_TIME
        while time.monotonic() < end:
            node.cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.stop()
        node.settle(2.0)             # let Cartographer catch up
        after = node.wait_pose()
        if after is None:
            print('Lost the transform after driving.')
            return
        dx, dy = after[0] - before[0], after[1] - before[1]
        moved = math.hypot(dx, dy)
        print(f'After:  x={after[0]:.2f} y={after[1]:.2f} heading={math.degrees(after[2]):.0f} deg')
        print(f'The estimated position moved {moved * 100:.0f} cm.')
        if moved < 0.05:
            print('RESULT: the estimate hardly moved. Either the wheels did not turn (check the '
                  'motors / min_pwm) or localisation is not following the robot.')
            return
        direction = math.atan2(dy, dx)
        diff = math.degrees(wrap(direction - before[2]))
        print(f'Direction of motion {math.degrees(direction):.0f} deg, reported heading '
              f'{math.degrees(before[2]):.0f} deg: difference {diff:+.0f} deg.')
        if abs(diff) < 20:
            print('RESULT: OK. The reported heading matches the way the robot moves. If it still '
                  'goes the wrong way, the cause is lag (CPU load) or a wrong start pose.')
        else:
            new_yaw = wrap(math.pi - math.radians(diff))
            print('RESULT: MISMATCH. The reported heading is turned by about '
                  f'{diff:+.0f} deg compared with how the robot really moves.')
            print(f'The lidar angle in the launch (laser_yaw, now 3.14159) is probably wrong. '
                  f'Try laser_yaw:={new_yaw:.2f}  (send me this result before changing it:\n'
                  f'the map was made with the old value, so the start heading must change too).')
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
