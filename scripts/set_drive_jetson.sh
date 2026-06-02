#!/usr/bin/env bash
# Toggle the line follower's motion master switch from the terminal.
#
#   scripts/set_drive_jetson.sh on     # allow the robot to move
#   scripts/set_drive_jetson.sh off    # hold still (perception keeps running)
#
# The follower starts with driving DISABLED, so enable it only once the wheels
# are clear / you are ready to test. Publishes std_msgs/Bool to /drive_enable.
set -euo pipefail

case "${1:-}" in
  on|ON|1|true|enable)   VAL=true ;;
  off|OFF|0|false|stop|disable) VAL=false ;;
  *) echo "Usage: $0 on|off" >&2; exit 2 ;;
esac

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"

ssh "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  source /opt/ros/humble/setup.bash
  source \"${REMOTE_WS}/install/setup.bash\" 2>/dev/null || true
  ros2 topic pub --once /drive_enable std_msgs/msg/Bool \"{data: ${VAL}}\"
'"
echo "drive_enable=${VAL} sent to ${JETSON_USER}@${JETSON_HOST}"
