#!/bin/bash
# ============================================================
# run_offline_test.sh
# Run on Jetson Nano WITHOUT any drone connected (Test Mode)
# ============================================================
set -e

echo "=================================================="
echo "  IUB DRONE — Jetson Nano Standalone Test Mode"
echo "  (No Drone / Flight Controller Required)"
echo "=================================================="

# Source ROS 2 base
source /opt/ros/humble/setup.bash

# Navigate to workspace, build all packages, and source
cd ~/ros2_ws
echo "[INFO] Building ROS 2 packages..."
colcon build --packages-select drone_vision_msgs precision_landing drone_vision mission_planner --symlink-install

# Source newly built workspace
source install/setup.bash

echo "[INFO] Launching autonomous software stack in SIM/STUB mode..."
ros2 launch mission_planner full_system.launch.py use_mavros:=false
