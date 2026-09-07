#!/bin/bash
# ============================================================
# send_command.sh
# User-friendly helper to send flight commands to the drone
# Usage: 
#   ./send_command.sh [start | abort | rtl | rth | land]
#   ./send_command.sh goto <north_m> <east_m> <down_m>
# ============================================================

CMD=${1:-start}
N=${2:-0.0}
E=${3:-0.0}
D=${4:--15.0}

echo "=================================================="
echo "  Sending Mission Command: '${CMD^^}'"
if [ "$CMD" == "goto" ]; then
    echo "  Target Coordinate: [North=$N m, East=$E m, Down=$D m]"
fi
echo "=================================================="

source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

if [ "$CMD" == "goto" ]; then
    ros2 topic pub --once /mission_planner/command drone_vision_msgs/msg/MissionCommand "{command: '$CMD', goto_ned: [$N, $E, $D]}"
else
    ros2 topic pub --once /mission_planner/command drone_vision_msgs/msg/MissionCommand "{command: '$CMD'}"
fi
