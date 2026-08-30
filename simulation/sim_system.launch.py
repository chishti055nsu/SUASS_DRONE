"""
sim_system.launch.py
====================
Unified simulation launch file for IUB Drone SUAS system.

Launches:
  1. Skydio X2 MuJoCo Physics Simulation
  2. Precision Landing Node (ArUco target detector)
  3. Drone Vision Node (YOLOv8 + Gemma)
  4. Mission Planner Node (State Machine + Flight Controller HAL)
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vision_pkg = get_package_share_directory("drone_vision")
    mission_pkg = get_package_share_directory("mission_planner")

    vision_params  = os.path.join(vision_pkg,  "config", "params.yaml")
    mission_params = os.path.join(mission_pkg, "config", "mission_params.yaml")

    # ── Launch Arguments ───────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("source_type",            default_value="ros_topic"),
        DeclareLaunchArgument("camera_topic",           default_value="/camera/image_raw"),
        DeclareLaunchArgument("yolo_model",             default_value="yolov8n.pt"),
        DeclareLaunchArgument("mission_type",           default_value="search_and_drop"),
        DeclareLaunchArgument("flight_controller_type", default_value="mujoco"),
        DeclareLaunchArgument("use_mavros",             default_value="false"),
    ]

    # 1. MuJoCo Physics Simulator Process
    sim_script = os.path.join(os.path.dirname(os.path.dirname(mission_pkg)), "simulation", "skydio_x2_sim.py")
    if not os.path.exists(sim_script):
        sim_script = os.path.abspath(os.path.join("simulation", "skydio_x2_sim.py"))

    mujoco_sim_process = ExecuteProcess(
        cmd=["python3", sim_script],
        output="screen",
    )

    # 2. Precision Landing Node (ArUco Detector)
    precision_node = Node(
        package="precision_landing",
        executable="precision_node",
        name="precision_landing_node",
        output="screen",
        parameters=[{
            "camera_topic": LaunchConfiguration("camera_topic"),
            "marker_size_m": 0.30,
        }],
    )

    # 3. Vision Node (YOLOv8 + Gemma)
    vision_node = Node(
        package="drone_vision",
        executable="vision_node",
        name="drone_vision_node",
        output="screen",
        parameters=[
            vision_params,
            {
                "source_type":  LaunchConfiguration("source_type"),
                "camera_topic": LaunchConfiguration("camera_topic"),
                "yolo_model":   LaunchConfiguration("yolo_model"),
            }
        ],
    )

    # 4. Mission Planner Node (Delayed 2s to allow vision & sim warmup)
    mission_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="mission_planner",
                executable="mission_node",
                name="mission_planner_node",
                output="screen",
                parameters=[
                    mission_params,
                    {
                        "mission_type":           LaunchConfiguration("mission_type"),
                        "flight_controller_type": LaunchConfiguration("flight_controller_type"),
                        "use_mavros":             LaunchConfiguration("use_mavros"),
                    }
                ],
            )
        ]
    )

    return LaunchDescription(args + [
        mujoco_sim_process,
        precision_node,
        vision_node,
        mission_node,
    ])
