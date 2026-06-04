#!/usr/bin/env bash
# Answer the intersection prompt (or reset the state machine) on the running line
# follower, so you don't type raw `ros2 topic pub` at each junction.
#
#   scripts/set_intersection_jetson.sh left       # turn left at the junction
#   scripts/set_intersection_jetson.sh right
#   scripts/set_intersection_jetson.sh straight
#   scripts/set_intersection_jetson.sh reset      # bail out -> back to FOLLOW
#
# left/right/straight publish /intersection_decision; reset publishes
# /intersection_reset (clears phase/decision/commit if it gets stuck in WAIT).
set -euo pipefail

case "${1:-}" in
  left|l|izquierda)          ROS_TOPIC="/intersection_decision"; MSG_TYPE="std_msgs/msg/String"; VAL="{data: left}" ;;
  right|r|derecha)           ROS_TOPIC="/intersection_decision"; MSG_TYPE="std_msgs/msg/String"; VAL="{data: right}" ;;
  straight|s|recto|forward)  ROS_TOPIC="/intersection_decision"; MSG_TYPE="std_msgs/msg/String"; VAL="{data: straight}" ;;
  reset|clear)               ROS_TOPIC="/intersection_reset";    MSG_TYPE="std_msgs/msg/Bool";   VAL="{data: true}" ;;
  *) echo "Usage: $0 left|right|straight|reset" >&2; exit 2 ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

ssh "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  source /opt/ros/humble/setup.bash
  source \"${PACKAGES_WS}/install/local_setup.bash\" 2>/dev/null || true
  source \"${REMOTE_WS}/install/local_setup.bash\" 2>/dev/null || true
  [ -f \"${REMOTE_PKG}/env_jetson.sh\" ] && source \"${REMOTE_PKG}/env_jetson.sh\"
  ros2 topic pub --once --wait-matching-subscriptions 0 ${ROS_TOPIC} ${MSG_TYPE} \"${VAL}\" || true
'"
echo "sent ${ROS_TOPIC} ${VAL}"
