#!/usr/bin/env bash
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
REMOTE_DATASET="${REMOTE_WS}/src/puzzlebot_ros/debug_dataset/"
LOCAL_DATASET="${LOCAL_DATASET:-debug_dataset/jetson/}"

mkdir -p "${LOCAL_DATASET}"
rsync -az "${JETSON_USER}@${JETSON_HOST}:${REMOTE_DATASET}" "${LOCAL_DATASET}"
echo "Pulled calibration dataset -> ${LOCAL_DATASET}"
