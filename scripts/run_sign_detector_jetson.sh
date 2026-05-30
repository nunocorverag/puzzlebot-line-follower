#!/usr/bin/env bash
# Run the YOLO traffic sign detector on the Jetson with live preview.
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
REMOTE_PKG="${REMOTE_WS}/src/puzzlebot_ros"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${SCRIPT_DIR}/sync_to_jetson.sh"

# Check ultralytics on Jetson
ssh "${JETSON_USER}@${JETSON_HOST}" "python3 -c 'import ultralytics' 2>/dev/null || pip3 install ultralytics --quiet" || true

xhost +local: >/dev/null 2>&1 || true
ssh -X "${JETSON_USER}@${JETSON_HOST}" \
  "source /opt/ros/humble/setup.bash && \
   source /home/puzzlebot/ros2_packages_ws/install/local_setup.bash && \
   source ${REMOTE_WS}/install/local_setup.bash && \
   cd ${REMOTE_PKG} && \
   python3 tools/sign_detector.py --confidence 0.45"
