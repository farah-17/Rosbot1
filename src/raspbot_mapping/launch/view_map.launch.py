"""Open a saved map (.yaml + .pgm) and show it in RViz. No robot needed.

    ros2 launch raspbot_mapping view_map.launch.py map:=/root/raspbot_lidar_ws/maps/my_room.yaml
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
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/root/raspbot_lidar_ws/maps/my_room.yaml',
                              description='Full path to the map .yaml file'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # Reads the .yaml/.pgm pair and publishes it on /map
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': map_yaml}]),

        # map_server is a lifecycle node: this switches it on
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server']}]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'map_view.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
