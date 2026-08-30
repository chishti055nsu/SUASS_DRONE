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
echo "[INFO] Launching production software stack with MAVROS hardware interface..."
ros2 launch mission_planner full_system.launch.py use_mavros:=true
