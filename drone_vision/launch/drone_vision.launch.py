"""
drone_vision.launch.py
Vision-only launch: camera + YOLO + Gemma → ROS2 topics
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("drone_vision")
    params_file = os.path.join(pkg, "config", "params.yaml")

    source_arg = DeclareLaunchArgument(
        "source_type", default_value="usb_cam",
        description="Video source: ros_topic | usb_cam | video_file"
    )
    model_arg = DeclareLaunchArgument(
        "yolo_model", default_value="yolov8n.pt",
        description="YOLO model weights path"
    )

    vision_node = Node(
        package="drone_vision",
        executable="vision_node",
        name="drone_vision_node",
        output="screen",
        parameters=[
            params_file,
            {
                "source_type": LaunchConfiguration("source_type"),
                "yolo_model":  LaunchConfiguration("yolo_model"),
            }
        ],
        remappings=[],
    )

    return LaunchDescription([source_arg, model_arg, vision_node])
