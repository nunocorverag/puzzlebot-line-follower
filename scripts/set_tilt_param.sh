#!/usr/bin/env bash
# Push a command to a running tilt_assistant.py over SSH.
#
#   scripts/set_tilt_param.sh start 1   # ARM capture (begin auto-snapshots)
#   scripts/set_tilt_param.sh stop 1    # stop capture
#   scripts/set_tilt_param.sh mark 1    # force one snapshot now
#   scripts/set_tilt_param.sh save 1    # write current pitch to config/camera_pose.json
#   scripts/set_tilt_param.sh u 1       # toggle undistort
#   scripts/set_tilt_param.sh q 1       # quit
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 CMD VALUE  (start|stop|mark|save|u|q)" >&2
  exit 2
fi

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
COMMAND_FILE="${COMMAND_FILE:-${REMOTE_WS}/src/puzzlebot_ros/debug_dataset/tilt_commands.txt}"

ssh "${JETSON_USER}@${JETSON_HOST}" "mkdir -p '$(dirname "${COMMAND_FILE}")' && printf '%s=%s\n' '$1' '$2' > '${COMMAND_FILE}'"
echo "Set $1=$2 via ${JETSON_USER}@${JETSON_HOST}:${COMMAND_FILE}"
