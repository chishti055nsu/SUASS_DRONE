#!/bin/bash
# ============================================================
# run_master_hardware_suite.sh
# 🛸 Master Drone Hardware Diagnostics & Active Motor Warmup Launcher
# ============================================================

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=========================================================================="
echo "   🛸 IUB DRONE SUAS 2026 — MASTER HARDWARE DIAGNOSTICS & WARMUP 🛸     "
echo "=========================================================================="

# 1. Stop nvgetty service (Jetson serial console conflict lock on /dev/ttyTHS1)
sudo systemctl stop nvgetty.service 2>/dev/null || true
sudo systemctl disable nvgetty.service 2>/dev/null || true

# 2. Set serial & USB video permissions
sudo chmod 666 /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM* /dev/video* 2>/dev/null || true

# 3. Configure Ethernet static IP for SIYI Subnet (192.168.144.167/24)
echo "  [1/3] Auto-Configuring Ethernet for SIYI A8 Mini 4K Camera..."
for eth in enP8p1s0 eth0 eth1; do
    if [ -d "/sys/class/net/$eth" ]; then
        echo "  ✅ Setting IP 192.168.144.167 on interface $eth..."
        sudo ip addr flush dev "$eth" 2>/dev/null || true
        sudo ip addr add 192.168.144.167/24 dev "$eth" 2>/dev/null || true
        sudo ip link set "$eth" up 2>/dev/null || true
        break
    fi
done

# 4. Launch Web GCS Dashboard UI Server in Background
echo "  [2/3] Launching Web GCS Dashboard UI Server (Port 8080)..."
pkill -f drone_web_gui.py 2>/dev/null || true
python3 "$ROOT_DIR/web_gui/drone_web_gui.py" --port 8080 >/dev/null 2>&1 &
sleep 1

echo "=========================================================================="
echo "  🎉 Web Dashboard Server active at: http://localhost:8080"
echo "=========================================================================="
echo "  [3/3] Starting Master Hardware Diagnostics & Motor Warmup Engine..."
echo "=========================================================================="

sudo python3 "$ROOT_DIR/scripts/run_master_hardware_suite.py" "$@"
