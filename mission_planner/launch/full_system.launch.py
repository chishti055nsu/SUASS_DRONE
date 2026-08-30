"""
full_system.launch.py
=====================
Launches the complete production IUB Drone system for physical deployment on Skydio drone:
  1. precision_landing_node (ArUco detector)
  2. drone_vision_node     (YOLO + Gemma)
  3. mission_planner_node  (State Machine + MAVROS Hardware Interface)
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vision_pkg  = get_package_share_directory("drone_vision")
    mission_pkg = get_package_share_directory("mission_planner")

    vision_params  = os.path.join(vision_pkg,  "config", "params.yaml")
    mission_params = os.path.join(mission_pkg, "config", "mission_params.yaml")

    # ── Launch Arguments ───────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("source_type",   default_value="usb_cam"),
        DeclareLaunchArgument("yolo_model",    default_value="yolov8n.pt"),
        DeclareLaunchArgument("mission_type",  default_value="search_and_drop"),
        DeclareLaunchArgument("use_mavros",    default_value="true"),
    ]

    # ── Precision Landing Node ───────────────────────────────────────────────
    precision_node = Node(
        package="precision_landing",
        executable="precision_node",
        name="precision_landing_node",
        output="screen",
        parameters=[{
            "camera_topic": "/camera/image_raw",
            "marker_size_m": 0.30,
        }],
    )

    # ── Vision Node ────────────────────────────────────────────────────────
    vision_node = Node(
        package="drone_vision",
        executable="vision_node",
        name="drone_vision_node",
        output="screen",
        parameters=[
            vision_params,
            {
                "source_type": LaunchConfiguration("source_type"),
                "yolo_model":  LaunchConfiguration("yolo_model"),
            }
        ],
    )

    # ── Mission Planner (delayed 3s to let vision warm up) ─────────────────
    mission_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="mission_planner",
                executable="mission_node",
                name="mission_planner_node",
                output="screen",
                parameters=[
                    mission_params,
                    {
                        "mission_type": LaunchConfiguration("mission_type"),
                        "use_mavros":   LaunchConfiguration("use_mavros"),
                    }
                ],
            )
        ]
    )

    return LaunchDescription(args + [precision_node, vision_node, mission_node])
