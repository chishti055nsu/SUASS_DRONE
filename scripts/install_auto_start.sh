#!/bin/bash
# ============================================================
# install_auto_start.sh
# Installs systemd service for headless automatic startup on Jetson Orin Nano
# Runs run_drone_flight.sh automatically when flight battery is plugged in.
# ============================================================

set -e

echo "=================================================="
echo "  Installing IUB Drone Auto-Start Systemd Service "
echo "=================================================="

# Check root permissions
if [ "$EUID" -ne 0 ]; then
  echo "Error: Please run as root (e.g. sudo bash scripts/install_auto_start.sh)"
  exit 1
fi

SERVICE_SRC="$(dirname "$0")/iub_drone.service"
SERVICE_DEST="/etc/systemd/system/iub_drone.service"

cp "$SERVICE_SRC" "$SERVICE_DEST"
chmod 644 "$SERVICE_DEST"

systemctl daemon-reload
systemctl enable iub_drone.service

echo ""
echo "✅ Auto-start service installed successfully!"
echo "Status: System will automatically launch run_drone_flight.sh on boot when battery is connected."
echo "To test manually: sudo systemctl start iub_drone.service"
echo "To view live logs: journalctl -u iub_drone.service -f"
echo "=================================================="
