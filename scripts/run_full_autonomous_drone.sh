#!/bin/bash
# ============================================================
# run_full_autonomous_drone.sh
# 🛸 Master Hardware & Autonomous Flight Suite Launcher
# ============================================================
# Forcefully initializes all physical hardware sensors & starts 
# the complete ROS 2 autonomy stack + Web GCS Dashboard.
# ============================================================

set -e

# Capture repository root directory BEFORE changing directories
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=========================================================================="
echo "       🛸 IUB DRONE SUAS 2026 — MASTER AUTONOMOUS FLIGHT SUITE 🛸        "
echo "=========================================================================="
echo "  [1/5] Clearing Serial Locks & Setting Port Permissions..."

# 1. Stop nvgetty service (Jetson serial console conflict lock on /dev/ttyTHS1)
sudo systemctl stop nvgetty.service 2>/dev/null || true
sudo systemctl disable nvgetty.service 2>/dev/null || true

# 2. Apply permissions to all serial devices
sudo chmod 666 /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

# 3. Configure Ethernet interface for SIYI A8 Mini Gimbal Camera
echo "  [2/5] Auto-Configuring Ethernet for SIYI A8 Mini 4K Camera..."
for eth in enP8p1s0 eth0 eth1; do
    if ip link show "$eth" >/dev/null 2>&1; then
        echo "  ✅ Setting IP 192.168.144.167 on interface $eth..."
        sudo ip addr flush dev "$eth" 2>/dev/null || true
        sudo ip addr add 192.168.144.167/24 dev "$eth" 2>/dev/null || true
        sudo ip link set "$eth" up 2>/dev/null || true
        break
    fi
done

# 4. Auto-detect TFmini-S LiDAR port
LIDAR_PORT="/dev/ttyUSB0"
if [ ! -e "$LIDAR_PORT" ]; then
    FOUND_PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1 || true)
    if [ -n "$FOUND_PORT" ]; then
        LIDAR_PORT="$FOUND_PORT"
    fi
fi
echo "  ✅ TFmini-S LiDAR target port: $LIDAR_PORT @ 115200 baud"

# 5. Matek H743 FC Port Configuration
FC_PORT="/dev/ttyTHS1"
FC_BAUD="921600"
if [ -e "/dev/ttyACM0" ]; then
    FC_PORT="/dev/ttyACM0"
    FC_BAUD="115200"
fi
echo "  ✅ Matek H743 Flight Controller target port: $FC_PORT @ $FC_BAUD baud"

# 6. Source ROS 2 Environment
echo "  [3/5] Sourcing ROS 2 Humble Environment & Building Workspace..."
unset AMENT_PREFIX_PATH
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null || true

# Clean duplicate package folders if any exist at root src
rm -rf ~/ros2_ws/src/drone_vision ~/ros2_ws/src/drone_vision_msgs ~/ros2_ws/src/mission_planner ~/ros2_ws/src/precision_landing 2>/dev/null || true

if [ -d ~/ros2_ws ]; then
    cd ~/ros2_ws
    colcon build --symlink-install --packages-select drone_vision_msgs drone_vision mission_planner precision_landing 2>/dev/null || true
    source install/setup.bash 2>/dev/null || true
fi

# Return to repository root directory
cd "$ROOT_DIR"

# 7. Start Web GCS Server on Port 8080 in background
echo "  [4/5] Launching Web GCS Dashboard UI Server (Port 8080)..."
pkill -f drone_web_gui.py 2>/dev/null || true
python3 web_gui/drone_web_gui.py --port 8080 >/dev/null 2>&1 &
WEB_GCS_PID=$!
sleep 1

echo "=========================================================================="
echo "  🎉 Web Dashboard UI Server running at: http://localhost:8080"
echo "  📱 Access from any Phone/Tablet/Laptop: http://<JETSON_IP>:8080"
echo "=========================================================================="
echo "  [5/5] Launching Hardware Drivers & Master ROS 2 Autonomy Stack..."
echo "  • Matek H743 FC Bridge : $FC_PORT @ $FC_BAUD baud"
echo "  • SIYI A8 Mini Camera   : rtsp://192.168.144.25:8554/main.264"
echo "  • RealSense D455 VIO    : 3D PointCloud Obstacle Avoidance Active"
echo "  • TFmini-S LiDAR        : Active on $LIDAR_PORT"
echo "=========================================================================="

# Launch MAVROS FC Connection
ros2 launch mavros px4.launch fcu_url:="$FC_PORT:$FC_BAUD" &
MAVROS_PID=$!
sleep 2

# Launch Full System ROS 2 Node (Perception, Vision, Obstacle Avoidance, State Machine)
ros2 launch mission_planner full_system.launch.py \
    use_mavros:=true \
    source_type:=rtsp \
    rtsp_url:=rtsp://192.168.144.25:8554/main.264 \
    tfmini_port:="$LIDAR_PORT"
