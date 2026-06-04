#!/usr/bin/env bash
# Compute the camera intrinsics ON the Jetson (OpenCV lives there, not on the
# laptop) from the captured datasets/checkerboard/, then pull the resulting
# config/camera_params.npz + undistorted preview back to the laptop.
#
#   scripts/run_calibrate_camera_jetson.sh
#   PATTERN=5x7 SQUARE_MM=25 scripts/run_calibrate_camera_jetson.sh
set -euo pipefail

STREAM=none   # pure compute, no camera/preview

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

PATTERN="${PATTERN:-5x7}"
SQUARE_MM="${SQUARE_MM:-1.0}"

sync_repo
run_remote_tool "python3 tools/calibrate_camera.py --pattern ${PATTERN} --square-size-mm ${SQUARE_MM}"

echo "Fetching results to the laptop..."
fetch_from_jetson "config/camera_params.npz"        "${REPO_DIR}/config/camera_params.npz"
fetch_from_jetson "config/undistorted_preview.jpg"  "${REPO_DIR}/config/undistorted_preview.jpg" || true
echo "Check config/undistorted_preview.jpg and commit config/camera_params.npz."
