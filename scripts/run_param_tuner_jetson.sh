#!/usr/bin/env bash
# Compatibility wrapper. New name: scripts/run_control_panel_jetson.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/run_control_panel_jetson.sh" "$@"
