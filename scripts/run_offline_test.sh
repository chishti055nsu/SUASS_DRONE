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

echo "[INFO] Launching autonomous software stack in SIM/STUB mode..."
ros2 launch mission_planner full_system.launch.py use_mavros:=false
