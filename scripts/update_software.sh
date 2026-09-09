#!/bin/bash
# ============================================================
# update_software.sh
# 🔄 One-Click Software Updater for IUB Drone Stack on Jetson Nano
# ============================================================
set -e

echo "=========================================================================="
echo "  🔄 UPDATING IUB DRONE SOFTWARE STACK & SYSTEM SERVICES"
echo "=========================================================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$WORKSPACE_DIR"

# 1. Pull latest code from GitHub main branch
echo "[1/5] Pulling latest updates from git repository..."
git pull origin main

# 2. Source ROS 2 Humble if available
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "[2/5] Sourcing ROS 2 Humble environment..."
    source /opt/ros/humble/setup.bash
fi

# 3. Maximize Jetson CPU & GPU Hardware Performance
echo "[3/5] Maximizing Jetson CPU & GPU hardware performance..."
if [ -f "$SCRIPT_DIR/maximize_jetson_performance.sh" ]; then
    bash "$SCRIPT_DIR/maximize_jetson_performance.sh" || true
fi

# 4. Restart permanent systemd service if installed
echo "[4/5] Reloading and restarting auto-start background service..."
if systemctl is-active --quiet iub_drone.service 2>/dev/null || systemctl is-enabled --quiet iub_drone.service 2>/dev/null; then
    sudo systemctl daemon-reload
    sudo systemctl restart iub_drone.service
    echo "  ✅ iub_drone.service restarted successfully."
else
    echo "  ℹ️  iub_drone.service is not installed. (Run 'sudo bash scripts/install_auto_start.sh' to install auto-boot)."
fi

# 5. Run full test suite verification
echo "[5/5] Running automated unit test suite verification..."
PYTHONPATH=.:drone_vision:mission_planner:precision_landing python3 -m unittest discover -s tests -p "test_*.py"

echo ""
echo "=========================================================================="
echo "  ✅ SOFTWARE UPDATE COMPLETE & VERIFIED SUCCESSFULLY!"
echo "=========================================================================="
