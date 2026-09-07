#!/bin/bash
# ============================================================
# start_web_gcs.sh
# Starts the Web Ground Control Station Dashboard Server on http://localhost:8080
# Allows novice students to control the drone from any web browser!
# ============================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

python3 web_gui/drone_web_gui.py
