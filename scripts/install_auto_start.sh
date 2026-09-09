#!/bin/bash
# ============================================================
# install_auto_start.sh
# 🛸 Master Headless Auto-Start & Permanent Connection Installer
# Sets up systemd auto-start service, udev serial permissions,
# and static network rules for NVIDIA Jetson companion computer.
# ============================================================

set -e

echo "=========================================================================="
echo "  🛸 INSTALLING PERMANENT HEADLESS AUTO-START SERVICE & HARDWARE RULES"
echo "=========================================================================="

# Check root permissions
if [ "$EUID" -ne 0 ]; then
  echo "[ERROR] Please run with sudo: sudo bash scripts/install_auto_start.sh"
  exit 1
fi

REAL_USER="${SUDO_USER:-$USER}"
if [ "$REAL_USER" == "root" ]; then
    REAL_USER="nvidia"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "  ✅ Target User      : $REAL_USER"
echo "  ✅ Workspace Path   : $WORKSPACE_DIR"

# 1. Stop and disable nvgetty (prevents serial port lock on /dev/ttyTHS1)
echo "[1/4] Disabling nvgetty serial console lock on /dev/ttyTHS1..."
systemctl stop nvgetty.service 2>/dev/null || true
systemctl disable nvgetty.service 2>/dev/null || true
systemctl mask nvgetty.service 2>/dev/null || true

# 2. Write permanent udev permissions rule for Matek FC, LiDAR, and Cameras
echo "[2/4] Writing permanent udev rules for serial ports and video devices..."
UDEV_RULE="/etc/udev/rules.d/99-iub-drone.rules"
cat <<EOF > "$UDEV_RULE"
# IUB Drone Permanent Hardware Permissions
KERNEL=="ttyTHS*", MODE="0666"
KERNEL=="ttyUSB*", MODE="0666"
KERNEL=="ttyACM*", MODE="0666"
KERNEL=="video*", MODE="0666"
EOF
chmod 644 "$UDEV_RULE"
udevadm control --reload-rules 2>/dev/null || true
udevadm trigger 2>/dev/null || true

# 3. Dynamically generate and install systemd auto-start service
echo "[3/4] Creating systemd service (/etc/systemd/system/iub_drone.service)..."
SERVICE_DEST="/etc/systemd/system/iub_drone.service"
cat <<EOF > "$SERVICE_DEST"
[Unit]
Description=IUB Drone Autonomous Flight Stack & Web GCS Permanent Service
After=network.target network-online.target nv-power.service
Wants=network-online.target

[Service]
Type=simple
User=$REAL_USER
WorkingDirectory=$WORKSPACE_DIR
ExecStartPre=/bin/bash $WORKSPACE_DIR/scripts/maximize_jetson_performance.sh
ExecStart=/bin/bash $WORKSPACE_DIR/scripts/run_full_autonomous_drone.sh
Restart=always
RestartSec=3s
Environment=PYTHONUNBUFFERED=1
Environment=DISPLAY=:0

[Install]
WantedBy=multi-user.target
EOF

chmod 644 "$SERVICE_DEST"

# 4. Enable systemd auto-start service
echo "[4/4] Enabling systemd auto-start service on boot..."
systemctl daemon-reload
systemctl enable iub_drone.service

echo ""
echo "=========================================================================="
echo "  🎉 PERMANENT HARDWARE SETUP & AUTO-START SERVICE INSTALLED SUCCESS!"
echo "=========================================================================="
echo "  • On Drone Power-On: Software auto-starts in 5 seconds."
echo "  • Web GCS Server   : Always active at http://192.168.144.100:8080"
echo "  • Control Service  : sudo systemctl start iub_drone"
echo "  • View Live Logs   : journalctl -u iub_drone.service -f"
echo "=========================================================================="
