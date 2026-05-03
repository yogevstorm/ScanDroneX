import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_odom_switch = LaunchConfiguration('use_odom_switch')
    default_params_file = os.path.join(get_package_share_directory("slam"),
                                       'config', 'mapper_params_online_async.yaml')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    declare_use_odom_switch = DeclareLaunchArgument(
        'use_odom_switch',
        default_value='true',
        description='true: SLAM uses odom_switch; false: SLAM uses odom_rf2o directly')

    has_node_params = HasNodeParams(source_file=params_file,
                                    node_name='slam_toolbox')

    actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                           ' else "', default_params_file, '"'])

    log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                    ' does not contain slam_toolbox parameters. Using default: ',
                                    default_params_file],
                               condition=UnlessCondition(has_node_params))

    odom_frame = PythonExpression(
        ['"odom_switch" if "', use_odom_switch, '" == "true" else "odom_lidar"'])

    start_async_slam_toolbox_node = Node(
        parameters=[
          actual_params_file,
          {'use_sim_time': use_sim_time,
           'odom_frame': odom_frame}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }],
    )

    slam_scan_toggle = Node(
        package='cmd_vel_odom',
        executable='slam_scan_toggle_node',
        name='slam_scan_toggle',
        output='screen',
        condition=IfCondition(use_odom_switch))

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_odom_switch)
    ld.add_action(log_param_change)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(lifecycle_manager)
    ld.add_action(slam_scan_toggle)

    return ld
