#!/usr/bin/env bash
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"

ssh "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  cd "${REMOTE_WS}"
  source /opt/ros/humble/setup.bash
  source src/puzzlebot_ros/env_jetson.sh
  colcon build --packages-select puzzlebot_ros
'"
