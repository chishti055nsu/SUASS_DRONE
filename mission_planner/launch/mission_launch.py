import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mission_planner')

    env_arg = DeclareLaunchArgument(
        'env',
        default_value='sim',
        description='Environment profile: sim, hardware, or stub'
    )

    env_config = PathJoinSubstitution([
        pkg_share,
        'config',
        [LaunchConfiguration('env'), '.yaml']
    ])

    base_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'base.yaml'
    ])

    mission_node = Node(
        package='mission_planner',
        executable='mission_node',
        name='mission_planner_node',
        output='screen',
        parameters=[base_config, env_config]
    )

    return LaunchDescription([
        env_arg,
        mission_node
    ])
