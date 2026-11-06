"""Talks to the Raspbot V1 motor board over I2C. No ROS in this file.

The motor board sits on I2C bus 1 at address 0x16 (decimal 22).
  register 0x01 <- [left_dir, left_speed, right_dir, right_speed]
                   dir 1 = forward, 0 = backward, speed 0..255
  register 0x02 <- 0   stops both motors
These are the same bytes Yahboom's YB_Pcb_Car.py library sends.
"""

import smbus

REG_MOTORS = 0x01
REG_STOP = 0x02


class RaspbotMotors:

    def __init__(self, bus=1, address=0x16,
                 left_reverse=False, right_reverse=False, swap_sides=False):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.left_reverse = left_reverse
        self.right_reverse = right_reverse
        self.swap_sides = swap_sides

    def drive(self, left, right):
        """Set wheel speeds from -255 to 255. Negative means backwards."""
        if self.swap_sides:
            left, right = right, left
        if self.left_reverse:
            left = -left
        if self.right_reverse:
            right = -right
        left = max(-255, min(255, int(left)))
        right = max(-255, min(255, int(right)))
        data = [1 if left >= 0 else 0, abs(left),
                1 if right >= 0 else 0, abs(right)]
        self.bus.write_i2c_block_data(self.address, REG_MOTORS, data)

    def stop(self):
        self.bus.write_byte_data(self.address, REG_STOP, 0x00)

    def close(self):
        self.stop()
        self.bus.close()
