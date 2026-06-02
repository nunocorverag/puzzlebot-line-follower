#!/usr/bin/env bash
# Live focus assistant on the Jetson, previewed over H264. Point at a textured
# target at the working distance and TURN THE LENS to maximize the score.
# Do this BEFORE camera calibration; don't touch focus afterwards.
#
#   scripts/run_focus_assist_jetson.sh
#   ROI=0.4 scripts/run_focus_assist_jetson.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

ROI="${ROI:-0.5}"

sync_repo
free_camera
start_stream
run_remote_tool "python3 tools/focus_assist.py --roi ${ROI}"
