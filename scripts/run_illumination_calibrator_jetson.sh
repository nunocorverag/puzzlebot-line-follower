#!/usr/bin/env bash
# Auto-guided flat-field (illumination) calibration on the Jetson, previewed over
# H264. Fill the frame with the white banner, press Enter when ready, then it
# averages good frames by itself,
# saves config/illumination_flatfield.npz and pulls it back to the laptop.
# Run this AFTER the checkerboard calibration.
#
#   scripts/run_illumination_calibrator_jetson.sh
#   FRAMES=30 scripts/run_illumination_calibrator_jetson.sh
#   AUTO_START=1 scripts/run_illumination_calibrator_jetson.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

FRAMES="${FRAMES:-25}"
AUTO_START="${AUTO_START:-0}"

sync_repo
free_camera
start_stream
EXTRA_ARGS=""
[ "${AUTO_START}" = "1" ] && EXTRA_ARGS="--auto-start"
run_remote_tool "python3 tools/illumination_calibrator.py --frames ${FRAMES} ${EXTRA_ARGS} --camera-params config/camera_params.npz --output config/illumination_flatfield.npz" || true

echo "Trayendo resultado a la laptop..."
fetch_from_jetson "config/illumination_flatfield.npz" "${REPO_DIR}/config/illumination_flatfield.npz"
fetch_from_jetson "config/illumination_preview.jpg"   "${REPO_DIR}/config/illumination_preview.jpg" || true
echo "Revisa config/illumination_preview.jpg y commitea el .npz."
