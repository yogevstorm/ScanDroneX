import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_odom_switch = LaunchConfiguration('use_odom_switch')

    declare_use_odom_switch = DeclareLaunchArgument(
        'use_odom_switch',
        default_value='true',
        description='true: odom_switch owns TF (publish_tf=false); false: RF2O owns TF (publish_tf=true)')

    common_params = {
        'laser_scan_topic': 'simple_drone/scan',
        'odom_topic': '/odom_rf2o',
        'base_frame_id': 'simple_drone/base_footprint',
        'odom_frame_id': 'odom_lidar',
        'init_pose_from_topic': '',
        'freq': 20.0,
        'use_sim_time': True,
    }

    rf2o_with_odom_switch = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{**common_params, 'publish_tf': False}],
        condition=IfCondition(use_odom_switch))

    rf2o_direct = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{**common_params, 'publish_tf': True}],
        condition=UnlessCondition(use_odom_switch))

    ld = LaunchDescription()
    ld.add_action(declare_use_odom_switch)
    ld.add_action(rf2o_with_odom_switch)
    ld.add_action(rf2o_direct)
    return ld
