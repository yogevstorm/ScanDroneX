#!/usr/bin/env python3

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# Read the maze definition from generate_maze.py so the drone spawn position
# always stays in sync with the 'S' marker in the ASCII map.
# ---------------------------------------------------------------------------
_WORLDS_DIR = '/home/yogev/Desktop/ScanDroneX/src/sjtu_drone/sjtu_drone_description/worlds'
_MAZE_WORLD = os.path.join(_WORLDS_DIR, 'maze.world')

_spec = importlib.util.spec_from_file_location(
    'generate_maze', os.path.join(_WORLDS_DIR, 'generate_maze.py')
)
_gm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_gm)


def _spawn_from_maze(maze, cell):
    """Return (x, y) world coordinates of the 'S' cell in the maze."""
    rows = len(maze)
    cols = max(len(row) for row in maze)
    offset_x = -(cols * cell) / 2.0 + cell / 2.0
    offset_y =  (rows * cell) / 2.0 - cell / 2.0
    for r, row in enumerate(maze):
        for c, ch in enumerate(row):
            if ch == 'S':
                return offset_x + c * cell, offset_y - r * cell
    return 0.0, 0.0  # fallback: world origin


_spawn_x, _spawn_y = _spawn_from_maze(_gm.MAZE, _gm.CELL)


def generate_launch_description():
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')

    print(f'[sjtu_drone_joy_teleop] world    : {_MAZE_WORLD}')
    print(f'[sjtu_drone_joy_teleop] drone spawn: x={_spawn_x:.3f}  y={_spawn_y:.3f}  z=0.300')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sjtu_drone_bringup_path, 'launch', 'sjtu_drone_gazebo.launch.py')
        ),
        launch_arguments={
            'world':   _MAZE_WORLD,
            'spawn_x': str(_spawn_x),
            'spawn_y': str(_spawn_y),
            'spawn_z': '0.3',
        }.items(),
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
