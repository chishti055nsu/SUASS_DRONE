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

# Ensure ROS 2 packages are placed directly under ~/ros2_ws/src/
mkdir -p ~/ros2_ws/src
if [ -d ~/ros2_ws/src/SUASS_DRONE/precision_landing ]; then
    cp -r ~/ros2_ws/src/SUASS_DRONE/precision_landing ~/ros2_ws/src/
    cp -r ~/ros2_ws/src/SUASS_DRONE/drone_vision ~/ros2_ws/src/
    cp -r ~/ros2_ws/src/SUASS_DRONE/drone_vision_msgs ~/ros2_ws/src/
    cp -r ~/ros2_ws/src/SUASS_DRONE/mission_planner ~/ros2_ws/src/
fi

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
