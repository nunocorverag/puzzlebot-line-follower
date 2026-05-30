#!/usr/bin/env bash
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
REMOTE_PACKAGE="${REMOTE_WS}/src/puzzlebot_ros"

xhost +local: >/dev/null 2>&1 || true
ssh -X "${JETSON_USER}@${JETSON_HOST}" "source /opt/ros/humble/setup.bash && source /home/puzzlebot/ros2_packages_ws/install/local_setup.bash && source ${REMOTE_WS}/install/local_setup.bash && cd ${REMOTE_PACKAGE} && python3 tools/recorder.py --interval 0.5 --output-dir dataset"
