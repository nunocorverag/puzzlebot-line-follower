#!/usr/bin/env bash
# Interactive PD + warp tuner (curses TUI) for the RUNNING line follower. Adjust
# kp/kd/max_v/max_w and the warp params LIVE, watch /lane_status metrics, and 's'
# saves them to config/lane_params.json + config/control_params.json. Runs on the
# Jetson next to the follower (parameter service is local = robust over WiFi),
# shown in your SSH terminal -- no ROS / X needed on the laptop.
#
# Start the follower first, then in another terminal:
#   scripts/run_param_tuner_jetson.sh
#
# Keys: j/k select   -/= (or left/right) nudge   s save   q quit
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

sync_repo
ssh -t "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  cd \"${REMOTE_PKG}\"
  source /opt/ros/humble/setup.bash
  source \"${PACKAGES_WS}/install/local_setup.bash\" 2>/dev/null || true
  source \"${REMOTE_WS}/install/local_setup.bash\" 2>/dev/null || true
  [ -f env_jetson.sh ] && source env_jetson.sh
  python3 tools/param_tuner.py
'"
