#!/bin/bash
# ============================================================
# drone_control_center.sh
# Interactive Master Control Center for IUB Drone Autonomy System
# Designed for novice students & competition ground operators.
# ============================================================

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

show_banner() {
    clear
    echo -e "${CYAN}${BOLD}"
    echo "=========================================================================="
    echo "       🛸 IUB DRONE SUAS 2026 — INTERACTIVE CONTROL CENTER 🛸            "
    echo "               User-Friendly Ground Control Terminal                      "
    echo "=========================================================================="
    echo -e "${NC}"
}

show_menu() {
    show_banner
    echo -e "${BOLD}Select an action by entering the number:${NC}\n"
    echo -e "  ${GREEN}[1] 🚀 START FULL AUTONOMOUS MISSION${NC}       (Takeoff -> Search -> Vision -> Drop -> RTL -> Land)"
    echo -e "  ${CYAN}[2] 📍 ENTER RAW GPS COORDINATES${NC}           (Input Lat/Lon points from Judges)"
    echo -e "  ${CYAN}[3] 🎯 FLY TO SPECIFIC METER COORDINATE (GOTO)${NC} (Enter North, East, Altitude)"
    echo -e "  ${YELLOW}[4] 🏠 RETURN TO BASE (RTL / RTH)${NC}           (Abort mission & fly back home)"
    echo -e "  ${YELLOW}[5] ⏸️  HOVER / HOLD POSITION${NC}              (Pause flight and hold current spot)"
    echo -e "  ${YELLOW}[6] 🛬 LAND IMMEDIATELY${NC}                   (Descend and land at current spot)"
    echo -e "  ${RED}[7] 🔴 EMERGENCY MOTOR KILL / DISARM${NC}       (SUAS Rule 5.3.8 Motor Cut-off)"
    echo -e "  ${BLUE}[8] 📊 VIEW LIVE TELEMETRY & STATUS${NC}        (Monitor altitude, battery, speed)"
    echo -e "  ${BLUE}[9] 🎮 LAUNCH 3D MUJOCO SIMULATOR${NC}           (Test flight on Mac/Laptop)"
    echo -e "  ${BLUE}[10] 🧪 RUN AUTOMATED DIAGNOSTIC TESTS${NC}    (Verify 34 unit & safety tests)"
    echo -e "  ${RED}[0] 🚪 EXIT CONTROL CENTER${NC}\n"
}

input_raw_gps() {
    show_banner
    echo -e "${CYAN}${BOLD}=== [2] Enter Raw Competition GPS Coordinates ===${NC}\n"
    read -p "Enter Home Latitude (e.g. 38.145000): " home_lat
    read -p "Enter Home Longitude (e.g. -76.427000): " home_lon
    read -p "Enter Target GPS Points (e.g. lat1,lon1;lat2,lon2): " points_str

    if [ -z "$home_lat" ] || [ -z "$home_lon" ]; then
        echo -e "${RED}Error: Home Latitude and Longitude are required.${NC}"
        read -p "Press Enter to return to menu..."
        return
    fi

    echo -e "\n${GREEN}Processing GPS WGS-84 coordinate conversion...${NC}"
    python3 -c "
from mission_planner.waypoint_manager import WaypointManager
wm = WaypointManager()
pts = []
if '$points_str':
    for item in '$points_str'.split(';'):
        if ',' in item:
            lat, lon = item.split(',')
            pts.append({'latitude': float(lat), 'longitude': float(lon), 'altitude': 15.0})
plan = wm.load_raw_gps_coordinates(float('$home_lat'), float('$home_lon'), pts)
print(f'✅ Successfully converted {plan.total()} waypoints relative to Home!')
"
    read -p "Press Enter to return to menu..."
}

fly_to_coordinate() {
    show_banner
    echo -e "${CYAN}${BOLD}=== [3] Fly To Specific Coordinate (GOTO) ===${NC}\n"
    read -p "Enter Target North meters (e.g. 50.0): " north_m
    read -p "Enter Target East meters (e.g. 20.0): " east_m
    read -p "Enter Target Altitude meters (e.g. 15.0): " alt_m

    north_m=${north_m:-0.0}
    east_m=${east_m:-0.0}
    alt_m=${alt_m:-15.0}

    # Convert altitude positive up to NED down negative
    down_m=$(python3 -c "print(-abs(float('$alt_m')))")

    echo -e "\n${GREEN}Dispatching GOTO command: North=${north_m}m, East=${east_m}m, Alt=${alt_m}m...${NC}"
    bash scripts/send_command.sh goto "$north_m" "$east_m" "$down_m"
    read -p "Press Enter to return to menu..."
}

view_telemetry() {
    show_banner
    echo -e "${BLUE}${BOLD}=== [8] Live Telemetry & Mission Status ===${NC}"
    echo -e "Press Ctrl+C to stop monitoring telemetry and return to menu.\n"
    if command -v ros2 &> /dev/null; then
        source /opt/ros/humble/setup.bash 2>/dev/null || true
        ros2 topic echo /mission_planner/status
    else
        python3 -c "
import time
from mission_planner.flight_controller import SimStubFlightController
fc = SimStubFlightController()
for i in range(10):
    t = fc.get_telemetry()
    print(f'\r[Telemetry] Mode={t[\"mode\"]} | Pos={t[\"pos_enu\"]} | Speed={t[\"speed_ms\"]:.1f}m/s | Batt={t[\"battery_pct\"]:.0f}%', end='')
    time.sleep(0.5)
print('')
"
    fi
    read -p "Press Enter to return to menu..."
}

# Main Loop
while true; do
    show_menu
    read -p "Enter choice [0-10]: " choice
    case $choice in
        1)
            show_banner
            echo -e "${GREEN}${BOLD}Initiating Autonomous Mission...${NC}\n"
            bash scripts/send_command.sh start
            read -p "Press Enter to return to menu..."
            ;;
        2)
            input_raw_gps
            ;;
        3)
            fly_to_coordinate
            ;;
        4)
            show_banner
            echo -e "${YELLOW}${BOLD}Sending Return to Home (RTL)...${NC}\n"
            bash scripts/send_command.sh rtl
            read -p "Press Enter to return to menu..."
            ;;
        5)
            show_banner
            echo -e "${YELLOW}${BOLD}Sending Hold / Hover command...${NC}\n"
            bash scripts/send_command.sh hold
            read -p "Press Enter to return to menu..."
            ;;
        6)
            show_banner
            echo -e "${YELLOW}${BOLD}Sending Land command...${NC}\n"
            bash scripts/send_command.sh land
            read -p "Press Enter to return to menu..."
            ;;
        7)
            show_banner
            echo -e "${RED}${BOLD}EMERGENCY DISARM / MOTOR KILL...${NC}\n"
            bash scripts/send_command.sh terminate
            read -p "Press Enter to return to menu..."
            ;;
        8)
            view_telemetry
            ;;
        9)
            show_banner
            echo -e "${BLUE}${BOLD}Launching 3D MuJoCo Physics Simulator...${NC}\n"
            python3 simulation/skydio_x2_sim.py
            read -p "Press Enter to return to menu..."
            ;;
        10)
            show_banner
            echo -e "${BLUE}${BOLD}Running 34 System Safety & Diagnostic Tests...${NC}\n"
            python3 -m unittest discover -s tests -p "test_*.py"
            read -p "Press Enter to return to menu..."
            ;;
        0)
            show_banner
            echo -e "${GREEN}Exiting Control Center. Blue skies! ✈️${NC}\n"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid selection. Please enter a number between 0 and 10.${NC}"
            sleep 1
            ;;
    esac
done
