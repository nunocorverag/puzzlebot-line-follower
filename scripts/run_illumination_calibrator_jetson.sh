#!/usr/bin/env bash
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
REMOTE_PACKAGE="${REMOTE_WS}/src/puzzlebot_ros"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"${SCRIPT_DIR}/sync_to_jetson.sh"

xhost +local: >/dev/null 2>&1 || true

ssh -X "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  cd "${REMOTE_PACKAGE}"
  python3 tools/illumination_calibrator.py \
    --gstreamer \
    --camera-params config/camera_params.npz \
    --output config/illumination_flatfield.npz
'"
