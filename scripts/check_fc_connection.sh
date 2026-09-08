#!/bin/bash
# ============================================================
# check_fc_connection.sh
# Automated Flight Controller (FC) Serial & MAVROS Connectivity Tester
# ============================================================

echo "============================================================"
echo "  🛸 IUB DRONE SUAS 2026 — FLIGHT CONTROLLER HARDWARE TEST  "
echo "============================================================"

FOUND_PORT=""

echo -e "\n[1/3] Checking Physical Serial Hardware Devices..."
for port in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyTHS1 /dev/ttyTHS0; do
    if [ -e "$port" ]; then
        echo -e "  ✅ Detected serial device: \033[1;32m$port\033[0m"
        FOUND_PORT="$port"
    fi
done

if [ -z "$FOUND_PORT" ]; then
    echo -e "  ❌ \033[1;31mNO SERIAL DEVICE DETECTED!\033[0m"
    echo "  👉 Ensure USB / UART cable is plugged into Flight Controller (Matek H743 / Pixhawk)."
    echo "  👉 Ensure Flight Controller has LiPo battery or USB power."
else
    echo -e "  ℹ️ Active Flight Controller Port Candidate: \033[1;33m$FOUND_PORT\033[0m"
fi

echo -e "\n[2/3] Checking Serial Port User Permissions..."
if groups $USER | grep &>/dev/null 'dialout'; then
    echo "  ✅ User '$USER' is in 'dialout' group (Serial access granted)."
else
    echo "  ⚠️ User '$USER' is NOT in 'dialout' group!"
    echo "  👉 Fix by running: sudo usermod -a -G dialout $USER && newgrp dialout"
fi

echo -e "\n[3/3] Checking ROS 2 / MAVROS Heartbeat Stream (/mavros/state)..."
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null || true
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

STATE_OUTPUT=$(timeout 3 ros2 topic echo /mavros/state --once 2>/dev/null)

if echo "$STATE_OUTPUT" | grep -q "connected: true"; then
    echo -e "  🎉 \033[1;32mMAVROS CONNECTED TO FLIGHT CONTROLLER SUCCESSFULLY!\033[0m"
    echo "$STATE_OUTPUT" | grep -E "connected|armed|guided|mode"
elif echo "$STATE_OUTPUT" | grep -q "connected: false"; then
    echo -e "  ⚠️ \033[1;33mMAVROS IS RUNNING BUT NOT CONNECTED TO FC (connected: false).\033[0m"
    echo "  👉 Check baud rate (57600 for Telemetry UART, 115200 for USB)."
    echo "  👉 Ensure MAVLink telemetry protocol (MAVLink 2) is enabled on FC."
else
    echo -e "  ℹ️ MAVROS ROS 2 node is not currently running."
    echo "  👉 Start MAVROS using: ros2 launch mavros px4.launch fcu_url:=$FOUND_PORT:57600"
fi

echo "============================================================"
