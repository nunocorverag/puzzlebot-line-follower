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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

ssh "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  source /opt/ros/humble/setup.bash
  source "${PACKAGES_WS}/install/local_setup.bash" 2>/dev/null || true
  source "${REMOTE_WS}/install/local_setup.bash" 2>/dev/null || true
  [ -f "${REMOTE_PKG}/env_jetson.sh" ] && source "${REMOTE_PKG}/env_jetson.sh"
  ros2 topic pub --once --wait-matching-subscriptions 0 /drive_enable std_msgs/msg/Bool \"{data: ${VAL}}\" || true
'"
echo "drive_enable=${VAL} sent to ${JETSON_USER}@${JETSON_HOST}"
