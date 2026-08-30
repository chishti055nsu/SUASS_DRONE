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

# Reset AMENT_PREFIX_PATH to ensure ROS 2 discovers all built packages freshly
unset AMENT_PREFIX_PATH
source /opt/ros/humble/setup.bash

# Navigate to workspace, build, and source
cd ~/ros2_ws
echo "[INFO] Building ROS 2 packages..."
colcon build --symlink-install

# Source newly built install directory
source install/setup.bash

echo "[INFO] AMENT_PREFIX_PATH is set to: $AMENT_PREFIX_PATH"
echo "[INFO] Launching autonomous software stack in SIM/STUB mode..."
ros2 launch mission_planner full_system.launch.py use_mavros:=false
