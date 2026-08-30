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

echo "[INFO] Launching production software stack with MAVROS hardware interface..."
ros2 launch mission_planner full_system.launch.py use_mavros:=true
