#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 PARAM VALUE" >&2
  echo "Example: $0 min_dash_count 6" >&2
  exit 2
fi

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
COMMAND_FILE="${COMMAND_FILE:-${REMOTE_WS}/src/puzzlebot_ros/debug_dataset/calibrator_commands.txt}"
PARAM="$1"
VALUE="$2"

ssh "${JETSON_USER}@${JETSON_HOST}" "mkdir -p '$(dirname "${COMMAND_FILE}")' && printf '%s=%s\n' '${PARAM}' '${VALUE}' > '${COMMAND_FILE}'"
echo "Set ${PARAM}=${VALUE} via ${JETSON_USER}@${JETSON_HOST}:${COMMAND_FILE}"
