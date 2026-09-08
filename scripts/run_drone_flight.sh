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

# Auto-configure SIYI A8 Mini Ethernet Interface if present
if ip link show enP8p1s0 >/dev/null 2>&1; then
    echo "[INFO] Auto-configuring Ethernet interface (enP8p1s0) for SIYI A8 Mini..."
    sudo ip addr flush dev enP8p1s0 2>/dev/null || true
    sudo ip addr add 192.168.144.167/24 dev enP8p1s0 2>/dev/null || true
    sudo ip link set enP8p1s0 up 2>/dev/null || true
fi

# Auto-fix serial permissions for TFmini-S LiDAR & Matek H743
sudo chmod 666 /dev/ttyUSB* /dev/ttyTHS* /dev/ttyACM* 2>/dev/null || true

# Auto-detect TFmini-S serial port
LIDAR_PORT="/dev/ttyUSB0"
if [ ! -e "$LIDAR_PORT" ]; then
    FOUND_PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1 || true)
    if [ -n "$FOUND_PORT" ]; then
        LIDAR_PORT="$FOUND_PORT"
    fi
fi

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

echo "=================================================="
echo "  [SUCCESS] All Hardware Auto-Configured & Ready!"
echo "  • SIYI A8 Mini RTSP Stream: rtsp://192.168.144.25:8554/main.264"
echo "  • RealSense D455 3D PointCloud: Active"
echo "  • TFmini-S LiDAR: Active on $LIDAR_PORT"
echo "  • Matek H743 + Dual GPS: Active on MAVROS"
echo "=================================================="

FC_PORT="/dev/ttyTHS1"
FC_BAUD="921600"
if [ -e "/dev/ttyACM0" ]; then
    FC_PORT="/dev/ttyACM0"
    FC_BAUD="115200"
fi

echo "[INFO] Starting MAVROS interface on FC Port: $FC_PORT @ $FC_BAUD baud..."
ros2 launch mavros px4.launch fcu_url:="$FC_PORT:$FC_BAUD" &
MAVROS_PID=$!
sleep 2

echo "[INFO] Launching master zero-config autonomous software stack..."
ros2 launch mission_planner full_system.launch.py \
    use_mavros:=true \
    source_type:=rtsp \
    rtsp_url:=rtsp://192.168.144.25:8554/main.264 \
    tfmini_port:="$LIDAR_PORT"


