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

# Source ROS 2 base and workspace
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
else
    echo "[ERROR] ~/ros2_ws/install/setup.bash not found! Please run 'colcon build' first."
    exit 1
fi

echo "[INFO] Launching autonomous software stack in SIM/STUB mode..."
ros2 launch mission_planner full_system.launch.py use_mavros:=false
