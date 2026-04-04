#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sjtu_drone_bringup_path, 'launch', 'sjtu_drone_gazebo.launch.py')
        )
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
    )

    joy_teleop_node = Node(
        package='joy_teleop_cpp',
        executable='joy_teleop_node',
        name='joy_teleop_node',
        output='screen',
    )

    return LaunchDescription([
        gazebo_launch,
        joy_node,
        joy_teleop_node,
    ])
