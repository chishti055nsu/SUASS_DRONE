#!/bin/bash
# ============================================================
# start_web_gcs.sh
# Starts the Web Ground Control Station Dashboard Server on http://localhost:8080
# Allows novice students to control the drone from any web browser!
# ============================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

# Enable NVIDIA Jetson CUDA & NVDEC Hardware Acceleration
export CUDA_VISIBLE_DEVICES=0
export CUDA_DEVICE_ORDER=PCI_BUS_ID
export OPENCV_VIDEOIO_PRIORITY_GSTREAMER=100

# Force kill any process holding port 8080 or previous instances of drone_web_gui.py
fuser -k 8080/tcp 2>/dev/null || true
pkill -9 -f drone_web_gui.py 2>/dev/null || true
sleep 0.5

python3 web_gui/drone_web_gui.py --port 8080
