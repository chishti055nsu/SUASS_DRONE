"""
precision_landing.launch.py
===========================
Launches the precision landing ArUco detector node.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="precision_landing",
            executable="precision_node",
            name="precision_landing_node",
            output="screen",
            parameters=[{
                "camera_topic": "/camera/image_raw",
                "marker_size_m": 0.30,
            }],
        )
    ])
