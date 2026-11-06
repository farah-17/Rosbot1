"""Navigation on the REAL robot, on a map you saved with raspbot_mapping.

    ros2 launch raspbot_navigation navigate.launch.py map:=my_room

Needs both files of the map in /root/raspbot_lidar_ws/maps:
    my_room.pbstream  (Cartographer: finds where the robot is)
    my_room.yaml/.pgm (the picture: A* plans on it)

Starts: lidar + motor_driver + Cartographer localisation (from raspbot_mapping),
set_start, navigator and nav_gui.
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
    mapping = get_package_share_directory('raspbot_mapping')
    pkg = get_package_share_directory('raspbot_navigation')
    params = os.path.join(pkg, 'config', 'navigation.yaml')

    maps_dir = LaunchConfiguration('maps_dir')
    name = LaunchConfiguration('map_name')
    map_yaml = [maps_dir, '/', name, '.yaml']
    map_pbstream = [maps_dir, '/', name, '.pbstream']

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='my_room',
                              description='Map name, without .yaml or .pbstream'),
        DeclareLaunchArgument('maps_dir', default_value='/root/raspbot_lidar_ws/maps'),
        DeclareLaunchArgument('rviz', default_value='false'),

        # Lidar, motors and Cartographer in localisation mode (previous workbook)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mapping, 'launch',
                                                       'localization.launch.py')),
            launch_arguments={'map': map_pbstream,
                              'rviz': LaunchConfiguration('rviz')}.items()),

        # Turns /initialpose (GUI "Start") into a Cartographer start position
        Node(package='raspbot_navigation', executable='set_start', name='set_start',
             output='screen'),

        Node(package='raspbot_navigation', executable='navigator', name='navigator',
             output='screen', emulate_tty=True,
             parameters=[params, {'map_yaml': map_yaml}]),

        Node(package='raspbot_navigation', executable='nav_gui', name='nav_gui',
             output='screen',
             parameters=[params, {'map_yaml': map_yaml}]),
    ])
