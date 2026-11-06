"""Spin each motor on its own so you can check wiring and direction.

    ros2 run raspbot_mapping motor_test              # full test
    ros2 run raspbot_mapping motor_test left         # left side only
    ros2 run raspbot_mapping motor_test right --speed 80 --time 1.5

The direction settings are read from config/motors.yaml, the same file
the motor_driver node uses, so a fix made there applies to both.
"""

import argparse
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory

from raspbot_mapping.raspbot_motors import RaspbotMotors


def load_settings():
    path = os.path.join(get_package_share_directory('raspbot_mapping'),
                        'config', 'motors.yaml')
    with open(path) as f:
        params = yaml.safe_load(f)['motor_driver']['ros__parameters']
    print(f'Settings from {os.path.realpath(path)}')
    for key in ('left_reverse', 'right_reverse', 'swap_sides'):
        print(f'  {key}: {params[key]}')
    return params


def run(motors, label, left, right, seconds):
    print(f'{label:<34} left={left:>4}  right={right:>4}')
    motors.drive(left, right)
    time.sleep(seconds)
    motors.stop()
    time.sleep(0.8)


def main():
    parser = argparse.ArgumentParser(description='Raspbot motor test')
    parser.add_argument('which', nargs='?', default='all',
                        choices=['all', 'left', 'right', 'both'])
    parser.add_argument('--speed', type=int, default=70, help='0..255')
    parser.add_argument('--time', type=float, default=1.0, help='seconds')
    args = parser.parse_args()

    p = load_settings()
    motors = RaspbotMotors(p['i2c_bus'], p['i2c_address'],
                           p['left_reverse'], p['right_reverse'], p['swap_sides'])
    s, t = args.speed, args.time
    try:
        if args.which in ('all', 'left'):
            run(motors, 'LEFT wheels forward', s, 0, t)
            run(motors, 'LEFT wheels backward', -s, 0, t)
        if args.which in ('all', 'right'):
            run(motors, 'RIGHT wheels forward', 0, s, t)
            run(motors, 'RIGHT wheels backward', 0, -s, t)
        if args.which in ('all', 'both'):
            run(motors, 'BOTH forward (robot drives ahead)', s, s, t)
            run(motors, 'BOTH backward', -s, -s, t)
            run(motors, 'Spin LEFT (anticlockwise)', -s, s, t)
            run(motors, 'Spin RIGHT (clockwise)', s, -s, t)
    finally:
        motors.close()
        print('Motors stopped.')


if __name__ == '__main__':
    main()
