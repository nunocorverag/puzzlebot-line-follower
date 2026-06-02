#!/usr/bin/env bash
# Live line/intersection calibrator on the Jetson. Default is H264 dashboard
# (overlay + Otsu mask + state panel). Use STREAM=local for OpenCV trackbars.
#
#   scripts/run_line_calibrator_jetson.sh
#   STREAM=local scripts/run_line_calibrator_jetson.sh   # old trackbar UI
#   LABEL=curve_left scripts/run_line_calibrator_jetson.sh
set -euo pipefail

STREAM="${STREAM:-h264}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

LABEL="${LABEL:-sample}"

sync_repo
free_camera
start_stream
run_remote_tool "python3 tools/line_vision_calibrator.py --gstreamer --preview-mode ${STREAM} --camera-params config/camera_params.npz --output-dir debug_dataset --label ${LABEL}"
