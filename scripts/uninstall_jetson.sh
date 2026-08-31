#!/bin/bash
# ============================================================
# uninstall_jetson.sh
# IUB Drone — Jetson Cleanup Script
# Removes all software, dependencies, Ollama models, and ROS 2 build artifacts.
# ============================================================
set -e

echo "========================================"
echo "  IUB Drone Jetson Complete Cleanup"
echo "========================================"
echo ""

# ── 1. Stop & Remove Ollama ───────────────────────────────────────────────
echo "[1/5] Removing Ollama service & models..."
if command -v systemctl &> /dev/null && systemctl is-active --quiet ollama 2>/dev/null; then
    echo "Stopping Ollama systemd service..."
    sudo systemctl stop ollama || true
    sudo systemctl disable ollama || true
fi

pkill -f ollama 2>/dev/null || true

if [ -f /etc/systemd/system/ollama.service ]; then
    sudo rm -f /etc/systemd/system/ollama.service
    sudo systemctl daemon-reload || true
fi

sudo rm -f /usr/local/bin/ollama /usr/bin/ollama
rm -rf ~/.ollama
echo "Ollama and models removed."

# ── 2. Uninstall Python Packages ──────────────────────────────────────────
echo "[2/5] Uninstalling Python packages..."
pip3 uninstall -y \
    ultralytics \
    requests \
    opencv-python-headless \
    scipy \
    matplotlib \
    pyyaml 2>/dev/null || true
echo "Python packages uninstalled."

# ── 3. Remove MAVROS & GeographicLib ─────────────────────────────────────
echo "[3/5] Uninstalling MAVROS & GeographicLib datasets..."
sudo apt-get remove --purge -y \
    ros-humble-mavros \
    ros-humble-mavros-extras 2>/dev/null || true
sudo apt-get autoremove -y
sudo rm -rf /usr/share/GeographicLib
echo "MAVROS packages removed."

# ── 4. Remove ROS 2 Build Artifacts ───────────────────────────────────────
echo "[4/5] Cleaning ROS 2 workspace build artifacts..."
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rm -rf "$WORKSPACE_DIR/build" "$WORKSPACE_DIR/install" "$WORKSPACE_DIR/log"
rm -rf ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log 2>/dev/null || true
echo "Workspace build directories removed."

# ── 5. Complete ───────────────────────────────────────────────────────────
echo ""
echo "========================================"
echo "  Cleanup Complete!"
echo "  All software & dependencies removed."
echo "========================================"
