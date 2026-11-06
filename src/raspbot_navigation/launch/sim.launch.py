"""Navigation in the SIMULATOR: no robot, no lidar, no motors needed.

    ros2 launch raspbot_navigation sim.launch.py
    ros2 launch raspbot_navigation sim.launch.py map:=/root/raspbot_lidar_ws/maps/my_room.yaml
    ros2 launch raspbot_navigation sim.launch.py rviz:=true

Starts: sim_robot (pretend robot + fake lidar), navigator (A* + driving), nav_gui.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('raspbot_navigation')
    params = os.path.join(pkg, 'config', 'navigation.yaml')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'sample_room.yaml'),
                              description='Map .yaml file to plan on (and to simulate)'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='false', description='Also open RViz'),

        # The pretend robot: moves on /cmd_vel, publishes TF and a fake /scan
        Node(package='raspbot_navigation', executable='sim_robot', name='sim_robot',
             output='screen',
             parameters=[{'map_yaml': map_yaml,
                          'start_x': LaunchConfiguration('start_x'),
                          'start_y': LaunchConfiguration('start_y')}]),

        Node(package='raspbot_navigation', executable='navigator', name='navigator',
             output='screen', emulate_tty=True,
             parameters=[params, {'map_yaml': map_yaml}]),

        Node(package='raspbot_navigation', executable='nav_gui', name='nav_gui',
             output='screen',
             parameters=[params, {'map_yaml': map_yaml}]),

        # Only for RViz: publish the map on /map
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': map_yaml}], condition=IfCondition(rviz)),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map',
             parameters=[{'autostart': True, 'node_names': ['map_server']}],
             condition=IfCondition(rviz)),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'navigation.rviz')],
             condition=IfCondition(rviz)),
    ])
