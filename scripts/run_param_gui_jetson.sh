#!/usr/bin/env bash
# Official ROS parameter GUI (rqt_reconfigure): sliders/fields for EVERY live
# parameter of the running follower (kp/kd/max_v/max_w + lane.* warp params).
# Runs on the Jetson, displayed on the laptop via X forwarding.
#
# Needs: the follower running, ros-humble-rqt-reconfigure on the Jetson, and
# working X (these scripts already use `ssh -X`). If X over WiFi is laggy, use the
# terminal tuner instead: scripts/run_param_tuner_jetson.sh
#
#   scripts/run_param_gui_jetson.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

xhost +local: >/dev/null 2>&1 || true
ssh -X "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  source /opt/ros/humble/setup.bash
  source \"${PACKAGES_WS}/install/local_setup.bash\" 2>/dev/null || true
  source \"${REMOTE_WS}/install/local_setup.bash\" 2>/dev/null || true
  [ -f \"${REMOTE_PKG}/env_jetson.sh\" ] && source \"${REMOTE_PKG}/env_jetson.sh\"
  ros2 run rqt_reconfigure rqt_reconfigure
'"
