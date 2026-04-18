from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='cmd_vel_ekf',
            executable='cmd_vel_ekf_node',
            name='cmd_vel_ekf_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frequency': 30.0,
                'q_x':   0.05,
                'q_y':   0.05,
                'q_yaw': 0.02,
                'r_x':   0.05,
                'r_y':   0.05,
                'r_yaw': 0.02,
                'mahal_threshold': 5.0,
            }]
        ),
    ])
