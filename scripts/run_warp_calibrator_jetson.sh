#!/usr/bin/env bash
# Live bird's-eye warp calibrator on the Jetson. Default is an H264 dashboard
# (original + trapezoid + bird's-eye view with sliding windows). Tune the four
# warp points live with scripts/set_warp_param.sh until a straight line looks
# vertical, then `scripts/set_warp_param.sh save_lane 1` to persist.
#
#   scripts/run_warp_calibrator_jetson.sh
#   STREAM=local scripts/run_warp_calibrator_jetson.sh   # ssh -X window
set -euo pipefail

STREAM="${STREAM:-h264}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

sync_repo
if [ "${HOLD_DRIVE_OFF:-1}" = "1" ]; then
  "${SCRIPT_DIR}/set_drive_jetson.sh" off >/dev/null 2>&1 || true
fi
free_camera
start_stream
run_remote_tool "python3 tools/warp_calibrator.py --gstreamer --camera-params config/camera_params.npz --illumination-params config/illumination_flatfield.npz --output config/lane_params.json"
