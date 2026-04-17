from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='Localization',
            name='Localization',
            output="screen"
        ),

        Node(
            package='navigation',
            executable='ObstacleMapNode',
            name='ObstacleMapNode',
            parameters=['/home/yogev/Desktop/ScanDroneX/src/navigation/config/obstacle_map_params.yaml'],
            output="screen"
        ),

        Node(
            package='navigation',
            executable='MissionPathNode',
            name='MissionPathNode',
            parameters=['/home/yogev/Desktop/ScanDroneX/src/navigation/config/mission_planner_params.yaml'],
            output="screen"
        ),

        Node(
            package='navigation',
            executable='BehaviorNode',
            name='BehaviorNode',
            parameters=['/home/yogev/Desktop/ScanDroneX/src/navigation/config/behavior_params.yaml'],
            output="screen"
        ),

        Node(
            package='navigation',
            executable='LocalPlannerNode',
            name='LocalPlannerNode',
            parameters=['/home/yogev/Desktop/ScanDroneX/src/navigation/config/local_planner_params.yaml'],
            output="screen"
        ),

        Node(
            package='navigation',
            executable='MotionPlannerNode',
            name='MotionPlannerNode',
            parameters=['/home/yogev/Desktop/ScanDroneX/src/navigation/config/motion_planner_params.yaml'],
            output="screen"
        ),

        Node(
            package='navigation',
            executable='ControlNode',
            name='ControlNode',
            parameters=['/home/yogev/Desktop/ScanDroneX/src/navigation/config/control_params.yaml'],
            output="screen"
        ),

        Node(
            package='navigation',
            executable='ScanNode',
            name='ScanNode',
            output="screen"
        ),
    ])
