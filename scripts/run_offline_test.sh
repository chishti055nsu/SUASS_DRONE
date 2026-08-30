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

# Clean up duplicate package folders if any exist at root src
rm -rf ~/ros2_ws/src/drone_vision ~/ros2_ws/src/drone_vision_msgs ~/ros2_ws/src/mission_planner ~/ros2_ws/src/precision_landing 2>/dev/null || true

# Reset environment & source ROS 2
unset AMENT_PREFIX_PATH
source /opt/ros/humble/setup.bash

# Build workspace cleanly
cd ~/ros2_ws
echo "[INFO] Building ROS 2 packages..."
colcon build --symlink-install
source install/setup.bash

echo "[INFO] Launching autonomous software stack in SIM/STUB mode..."
ros2 launch mission_planner full_system.launch.py use_mavros:=false
