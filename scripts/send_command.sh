#!/bin/bash
# ============================================================
# send_command.sh
# User-friendly helper to send flight commands to the drone
# Usage: ./send_command.sh [start | abort | rtl | land]
# ============================================================

CMD=${1:-start}

echo "=================================================="
echo "  Sending Mission Command: '${CMD^^}'"
echo "=================================================="

source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

ros2 topic pub --once /mission_planner/command drone_vision_msgs/msg/MissionCommand "{command: '$CMD'}"
