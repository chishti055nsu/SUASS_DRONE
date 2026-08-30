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

# Source ROS 2 base and workspace
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
else
    echo "[ERROR] ~/ros2_ws/install/setup.bash not found! Please run 'colcon build' first."
    exit 1
fi

echo "[INFO] Launching production software stack with MAVROS hardware interface..."
ros2 launch mission_planner full_system.launch.py use_mavros:=true
