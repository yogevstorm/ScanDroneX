import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('scandronex_sim')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # ── Arguments ──────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.5')

    # ── URDF / robot description ────────────────────────────────────────────
    urdf_file = os.path.join(pkg, 'urdf', 'sjtu_drone_with_lidar.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # ── Gazebo (world1) ─────────────────────────────────────────────────────
    world_file = os.path.join(pkg, 'worlds', 'world1.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items(),
    )

    # ── Robot State Publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # ── Spawn drone in Gazebo ───────────────────────────────────────────────
    # Delayed slightly to give Gazebo time to start
    spawn_drone = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_drone',
                output='screen',
                arguments=[
                    '-entity', 'sjtu_drone',
                    '-topic', 'robot_description',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose,
                ],
            )
        ],
    )

    # ── Auto-takeoff ────────────────────────────────────────────────────────
    # Publish one Empty msg to /simple_drone/takeoff after spawn settles.
    # Without this the drone stays in LANDED state and ignores cmd_vel.
    auto_takeoff = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/simple_drone/takeoff',
                    'std_msgs/msg/Empty', '{}',
                ],
                output='screen',
            )
        ],
    )

    # ── Joystick node ───────────────────────────────────────────────────────
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )

    # ── Teleop twist joy ────────────────────────────────────────────────────
    # Remaps /cmd_vel → /simple_drone/cmd_vel (sjtu_drone topic)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            os.path.join(pkg, 'config', 'joy_teleop.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/cmd_vel', '/simple_drone/cmd_vel'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.5'),

        gazebo,
        robot_state_publisher,
        spawn_drone,
        auto_takeoff,
        joy_node,
        teleop_node,
    ])
