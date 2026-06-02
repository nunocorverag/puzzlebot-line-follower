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
# Make sure ultralytics is available on the Jetson.
ssh "${JETSON_USER}@${JETSON_HOST}" \
  "python3 -c 'import ultralytics' 2>/dev/null || pip3 install ultralytics --quiet" || true

free_camera
start_stream
run_remote_tool "python3 tools/sign_detector.py --confidence ${CONFIDENCE}"
