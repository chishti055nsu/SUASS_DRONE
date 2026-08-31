#!/bin/bash
# Shortcut to run the zero-trouble master installer
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$SCRIPT_DIR/scripts/setup_jetson.sh"
