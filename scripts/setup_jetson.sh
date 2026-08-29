#!/bin/bash
# ============================================================
# setup_jetson.sh
# IUB Drone — Jetson Setup Script
# Run once on a fresh Jetson to install all dependencies
# ============================================================
set -e

echo "========================================"
echo "  IUB Drone Jetson Setup"
echo "========================================"

# ── System dependencies ───────────────────────────────────────────────────
echo "[1/6] Installing system packages..."
sudo apt-get update -qq
sudo apt-get install -y \
    python3-pip python3-dev \
    python3-opencv \
    libopencv-dev \
    curl wget git

# ── Python dependencies ───────────────────────────────────────────────────
echo "[2/6] Installing Python packages..."
pip3 install --upgrade pip
pip3 install \
    ultralytics \
    requests \
    numpy \
    opencv-python-headless

# ── Ollama ────────────────────────────────────────────────────────────────
echo "[3/6] Installing Ollama..."
if ! command -v ollama &> /dev/null; then
    curl -fsSL https://ollama.com/install.sh | sh
    echo "Ollama installed."
else
    echo "Ollama already installed: $(ollama --version)"
fi

# ── Pull Gemma model ──────────────────────────────────────────────────────
echo "[4/6] Pulling Gemma 4 e4b model (this may take a while)..."
ollama pull gemma4:e4b
echo "Gemma model ready."

# ── MAVROS (if not already installed) ────────────────────────────────────
echo "[5/6] Checking MAVROS..."
if ! ros2 pkg list 2>/dev/null | grep -q mavros; then
    echo "Installing MAVROS for ROS2 Humble..."
    sudo apt-get install -y \
        ros-humble-mavros \
        ros-humble-mavros-extras
    # Install GeographicLib datasets
    sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
    echo "MAVROS installed."
else
    echo "MAVROS already installed."
fi

# ── Build ROS2 workspace ──────────────────────────────────────────────────
echo "[6/6] Building ROS2 workspace..."
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "Workspace: $WORKSPACE_DIR"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
colcon build \
    --packages-select drone_vision_msgs drone_vision mission_planner \
    --symlink-install

echo ""
echo "========================================"
echo "  Setup Complete!"
echo "========================================"
echo ""
echo "To run the full system:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $WORKSPACE_DIR/install/setup.bash"
echo "  ros2 launch drone_vision full_system.launch.py"
echo ""
echo "To run vision only (testing):"
echo "  ros2 launch drone_vision drone_vision.launch.py source_type:=usb_cam"
echo ""
echo "To send a mission start command:"
echo "  ros2 topic pub /mission_planner/command drone_vision_msgs/msg/MissionCommand '{command: start}'"
