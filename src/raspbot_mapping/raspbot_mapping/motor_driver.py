"""ROS 2 node: turns /cmd_vel (geometry_msgs/Twist) into Raspbot motor commands.

    linear.x  (m/s, forward +)       -> both wheels the same way
    angular.z (rad/s, anticlockwise +) -> wheels opposite ways

The Raspbot V1 has no wheel encoders, so speeds are open loop: max_linear
and max_angular are the commands that give full power (max_pwm).
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from raspbot_mapping.raspbot_motors import RaspbotMotors


class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x16)
        self.declare_parameter('left_reverse', False)
        self.declare_parameter('right_reverse', False)
        self.declare_parameter('swap_sides', False)
        self.declare_parameter('max_pwm', 100)       # 0..255, top speed we allow
        self.declare_parameter('min_pwm', 35)        # below this the wheels do not turn
        self.declare_parameter('max_linear', 0.5)    # m/s that means "full forward"
        self.declare_parameter('max_angular', 1.0)   # rad/s that means "full spin"
        self.declare_parameter('cmd_timeout', 0.0)   # s, 0 = keep last command

        self.motors = RaspbotMotors(self.get_parameter('i2c_bus').value,
                                    self.get_parameter('i2c_address').value)
        self.last_cmd = self.get_clock().now()
        self.moving = False
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(0.1, self.check_timeout)
        self.get_logger().info('motor_driver ready, listening on /cmd_vel')

    def p(self, name):
        return self.get_parameter(name).value

    def to_pwm(self, value):
        """-1..1 -> -max_pwm..max_pwm, lifting small values up to min_pwm."""
        value = max(-1.0, min(1.0, value))
        if abs(value) < 0.01:
            return 0
        pwm = abs(value) * self.p('max_pwm')
        pwm = max(pwm, self.p('min_pwm'))
        return int(pwm if value > 0 else -pwm)

    def on_cmd_vel(self, msg):
        # Read the direction settings every time, so "ros2 param set" works live.
        self.motors.left_reverse = self.p('left_reverse')
        self.motors.right_reverse = self.p('right_reverse')
        self.motors.swap_sides = self.p('swap_sides')

        v = msg.linear.x / self.p('max_linear')
        w = msg.angular.z / self.p('max_angular')
        left = self.to_pwm(v - w)
        right = self.to_pwm(v + w)
        if left == 0 and right == 0:
            self.motors.stop()
            self.moving = False
        else:
            self.motors.drive(left, right)
            self.moving = True
        self.last_cmd = self.get_clock().now()

    def check_timeout(self):
        timeout = self.p('cmd_timeout')
        if timeout <= 0.0 or not self.moving:
            return
        age = (self.get_clock().now() - self.last_cmd).nanoseconds / 1e9
        if age > timeout:
            self.motors.stop()
            self.moving = False
            self.get_logger().warn('No /cmd_vel for %.1f s, motors stopped' % age)

    def destroy_node(self):
        self.motors.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorDriver()
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
