
"""Start the YDLIDAR 4ROS driver and say where the lidar sits on the robot.

    ros2 launch raspbot_mapping lidar.launch.py
    ros2 launch raspbot_mapping lidar.launch.py rviz:=true
    ros2 launch raspbot_mapping lidar.launch.py laser_yaw:=3.1416
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('raspbot_mapping')

    params_file = LaunchConfiguration('params_file')
    laser_x = LaunchConfiguration('laser_x')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'ydlidar_4ros.yaml'),
                              description='Lidar parameter file'),
        DeclareLaunchArgument('laser_x', default_value='-0.13',
                              description='Lidar distance in front of the robot centre (m)'),
        DeclareLaunchArgument('laser_z', default_value='0.12',
                              description='Lidar height above the floor (m)'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0',
                              description='Lidar rotation on the robot (rad)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Also open RViz'),

        # The lidar driver: reads the serial port, publishes /scan
        Node(package='ydlidar_ros2_driver',
             executable='ydlidar_ros2_driver_node',
             name='ydlidar_ros2_driver_node',
             output='screen',
             emulate_tty=True,
             parameters=[params_file]),

        # Fixed transform base_link -> laser_frame (where the lidar is on the robot)
        Node(package='tf2_ros',
             executable='static_transform_publisher',
             name='base_to_laser',
             arguments=['--x', laser_x, '--y', '0', '--z', laser_z,
                        '--roll', '0', '--pitch', '0', '--yaw', laser_yaw,
                        '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'lidar.rviz')],
             condition=IfCondition(rviz)),
    ])
