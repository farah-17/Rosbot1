"""Re-use a saved Cartographer map: the robot finds itself on it while you drive.

    ros2 launch raspbot_mapping localization.launch.py \
        map:=/root/raspbot_lidar_ws/maps/my_room.pbstream
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('raspbot_mapping')
    config_dir = os.path.join(pkg, 'config')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/root/raspbot_lidar_ws/maps/my_room.pbstream',
                              description='Full path to the saved .pbstream file'),
        DeclareLaunchArgument('rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'lidar.launch.py'))),

        Node(package='raspbot_mapping', executable='motor_driver', name='motor_driver',
             output='screen',
             parameters=[os.path.join(config_dir, 'motors.yaml')]),

        Node(package='cartographer_ros', executable='cartographer_node',
             name='cartographer_node', output='screen',
             arguments=['-configuration_directory', config_dir,
                        '-configuration_basename', 'cartographer_localization.lua',
                        '-load_state_filename', LaunchConfiguration('map')]),

        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='occupancy_grid_node', output='screen',
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'mapping.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
