#!/usr/bin/env bash
# Auto-guided checkerboard capture on the Jetson, previewed over H264. Move the
# board following the on-screen hints; it captures diverse poses by itself.
# Captured images are pulled back to the laptop's calibration_images/.
#
#   scripts/run_checkerboard_capture_jetson.sh
#   TARGET=40 PATTERN=5x7 scripts/run_checkerboard_capture_jetson.sh   (default 30)
#   RESET=0 scripts/run_checkerboard_capture_jetson.sh                 # append instead of fresh run
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

TARGET="${TARGET:-30}"
PATTERN="${PATTERN:-5x7}"
RESET="${RESET:-1}"

sync_repo
free_camera
if [ "${RESET}" = "1" ]; then
  echo "Limpiando capturas anteriores (RESET=0 para conservarlas)..."
  rm -f "${REPO_DIR}"/calibration_images/calib_*.jpg 2>/dev/null || true
  ssh "${JETSON_USER}@${JETSON_HOST}" "rm -f ${REMOTE_PKG}/calibration_images/calib_*.jpg" 2>/dev/null || true
fi
start_stream
run_remote_tool "python3 tools/calib_capture_checkerboard.py --pattern ${PATTERN} --target ${TARGET}" || true

echo "Fetching captured images to the laptop..."
fetch_from_jetson "calibration_images/" "${REPO_DIR}/calibration_images/"
echo "Done. Now run: scripts/run_calibrate_camera_jetson.sh"
