#!/usr/bin/env bash
# Run the YOLO traffic-sign + traffic-light detector on the Jetson.
# Opens the CSI camera itself — NO separate camera step needed.
#
#   scripts/run_sign_detector_jetson.sh                 # H264 to laptop (default)
#   STREAM=local scripts/run_sign_detector_jetson.sh    # window via ssh -X
#   STREAM=none  scripts/run_sign_detector_jetson.sh    # headless
#   CONFIDENCE=0.4 scripts/run_sign_detector_jetson.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

CONFIDENCE="${CONFIDENCE:-0.45}"

sync_repo
# Make sure ultralytics is available on the Jetson. This script runs the
# detector with the user site enabled because ultralytics is normally installed
# with --user on the Jetson; common.sh keeps it disabled for the calibrator and
# ROS runtime tools to avoid user-site package conflicts.
ssh "${JETSON_USER}@${JETSON_HOST}" \
  "python3 -c 'import ultralytics' 2>/dev/null || python3 -m pip install --user ultralytics --quiet" || true

free_camera
start_stream
run_remote_tool "env -u PYTHONNOUSERSITE python3 tools/sign_detector.py --confidence ${CONFIDENCE}"
