"""Build a map of the room: lidar + motors + Cartographer (+ RViz).

    ros2 launch raspbot_mapping mapping.launch.py
    ros2 launch raspbot_mapping mapping.launch.py rviz:=false

Drive the robot from a second terminal with:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
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

    rviz = LaunchConfiguration('rviz')
    resolution = LaunchConfiguration('resolution')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('resolution', default_value='0.05',
                              description='Map cell size in metres'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.12'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'lidar.launch.py')),
            launch_arguments={'laser_yaw': LaunchConfiguration('laser_yaw'),
                              'laser_z': LaunchConfiguration('laser_z')}.items()),

        Node(package='raspbot_mapping', executable='motor_driver', name='motor_driver',
             output='screen',
             parameters=[os.path.join(config_dir, 'motors.yaml')]),

        # SLAM: works out the robot position and builds submaps from /scan
        Node(package='cartographer_ros', executable='cartographer_node',
             name='cartographer_node', output='screen',
             arguments=['-configuration_directory', config_dir,
                        '-configuration_basename', 'cartographer_2d.lua']),

        # Joins the submaps into one occupancy grid on /map
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='occupancy_grid_node', output='screen',
             arguments=['-resolution', resolution, '-publish_period_sec', '1.0']),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'mapping.rviz')],
             condition=IfCondition(rviz)),
    ])
