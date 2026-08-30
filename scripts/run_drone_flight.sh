#!/bin/bash
# ============================================================
# run_drone_flight.sh
# Run on Jetson Nano WHEN CONNECTED to Matek H743 Drone
# ============================================================
set -e

echo "=================================================="
echo "  IUB DRONE — Jetson Nano Physical Drone Flight Mode"
echo "  (Connected to Matek H743 / Pixhawk via MAVROS)"
echo "=================================================="

# Source ROS 2 base
source /opt/ros/humble/setup.bash

# Navigate to workspace, build all packages, and source
cd ~/ros2_ws
echo "[INFO] Building ROS 2 packages..."
colcon build --packages-select drone_vision_msgs precision_landing drone_vision mission_planner --symlink-install

# Source newly built workspace
source install/setup.bash

echo "[INFO] Launching production software stack with MAVROS hardware interface..."
ros2 launch mission_planner full_system.launch.py use_mavros:=true
