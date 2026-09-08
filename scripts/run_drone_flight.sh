#!/bin/bash
# ============================================================
# run_drone_flight.sh
# Master Hardware & Autonomous Flight Suite Launcher
# ============================================================
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ROOT_DIR/scripts/run_full_autonomous_drone.sh" "$@"
