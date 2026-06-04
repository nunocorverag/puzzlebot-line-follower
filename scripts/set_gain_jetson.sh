#!/usr/bin/env bash
# Live-tune the line follower's PD / curve gains WITHOUT recompiling. Targets the
# running node (name: autonomous_racer) via `ros2 param set`. The follower applies
# the new value on the next control tick, so you can kill oscillation iteratively.
#
#   scripts/set_gain_jetson.sh kp 0.0025            # lower P -> less oscillation
#   scripts/set_gain_jetson.sh kd 0.012             # more damping -> less overshoot
#   scripts/set_gain_jetson.sh max_v 0.06           # cap linear speed
#   scripts/set_gain_jetson.sh max_w 0.5            # cap angular speed
#   scripts/set_gain_jetson.sh curve_slow_gain 0.7  # brake harder in curves
#
# Params: kp | kd | max_v | max_w | curve_slow_gain | curve_min_scale
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 PARAM VALUE  (kp|kd|max_v|max_w|curve_slow_gain|curve_min_scale)" >&2
  exit 2
fi

NODE_NAME="${NODE_NAME:-autonomous_racer}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

ssh "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  source /opt/ros/humble/setup.bash
  source \"${PACKAGES_WS}/install/local_setup.bash\" 2>/dev/null || true
  source \"${REMOTE_WS}/install/local_setup.bash\" 2>/dev/null || true
  [ -f \"${REMOTE_PKG}/env_jetson.sh\" ] && source \"${REMOTE_PKG}/env_jetson.sh\"
  ros2 param set /${NODE_NAME} $1 $2
'"
echo "set $1=$2 on /${NODE_NAME}"
