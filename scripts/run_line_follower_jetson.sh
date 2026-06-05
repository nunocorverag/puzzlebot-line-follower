#!/usr/bin/env bash
# Run the autonomous line follower (ros2 run, uses the BUILT package — run
# build_on_jetson.sh after changing node code). Wheels-up first.
#
#   scripts/run_line_follower_jetson.sh                 # H264 to laptop (default)
#   STREAM=local scripts/run_line_follower_jetson.sh    # MJPEG at http://10.10.0.100:8080
#   STREAM=none  scripts/run_line_follower_jetson.sh    # MJPEG server only, no receiver
#   NODE=autonomous_racer scripts/run_line_follower_jetson.sh
#   IGNORE_TRAFFIC_LIGHT=1 scripts/run_line_follower_jetson.sh   # drive w/o needing a GREEN light (testing)
#   CONTROLLER_LOG=1 scripts/run_line_follower_jetson.sh         # log control CSV (puzzlebot_ros/controller_data.csv)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

NODE="${NODE:-line_follower}"

# Map the unified STREAM knob onto the node's ROS params. h264 streams to the
# laptop; anything else uses the built-in MJPEG server (browser at :8080).
start_stream            # launches the H264 receiver locally when STREAM=h264
PARAMS=""
if [ "${STREAM}" = "h264" ]; then
  PARAMS="${PARAMS} -p stream_mode:=h264 -p h264_host:=${H264_HOST} -p h264_port:=${H264_PORT} -p h264_bitrate:=${H264_BITRATE}"
fi
if [ "${IGNORE_TRAFFIC_LIGHT:-0}" = "1" ]; then
  PARAMS="${PARAMS} -p ignore_traffic_light:=true"
fi
if [ "${CONTROLLER_LOG:-0}" = "1" ]; then
  PARAMS="${PARAMS} -p controller_log:=true"   # CSV -> puzzlebot_ros/controller_data.csv
fi
ROS_ARGS=""
if [ -n "${PARAMS}" ]; then
  ROS_ARGS="--ros-args${PARAMS}"
fi

xhost +local: >/dev/null 2>&1 || true

# Kill any follower still running on the Jetson (plain ssh doesn't forward Ctrl-C,
# so a previous run can be orphaned). Used both before launch (no pile-up, no
# camera/cmd_vel fights) and on exit (so Ctrl-C here cleans up the remote node).
remote_kill_follower() {
  ssh "${JETSON_USER}@${JETSON_HOST}" \
    "pkill -f 'puzzlebot_ros (line_follower|autonomous_racer)|lib/puzzlebot_ros/line_follower' 2>/dev/null; true" \
    >/dev/null 2>&1 || true
}

# On exit: kill the node AND pull this run's recorded snapshots (tuner 'r' ->
# debug_dataset/follower_session) to the laptop per session, wiping the Jetson.
FOLLOWER_SESSION="$(date +%Y%m%d_%H%M%S)"
cleanup_follower() {
  remote_kill_follower
  pull_and_clean_session "${REMOTE_PKG}/debug_dataset/follower_session" \
    "${REPO_DIR}/datasets/follower_session/${FOLLOWER_SESSION}"
}

echo "Cleaning up any running follower on the Jetson..."
remote_kill_follower
sleep 1
trap cleanup_follower INT TERM EXIT

ssh -X "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  cd \"${REMOTE_WS}\"
  source /opt/ros/humble/setup.bash
  source src/puzzlebot_ros/env_jetson.sh
  source install/setup.bash
  export PYTHONNOUSERSITE=1
  ros2 run puzzlebot_ros \"${NODE}\" ${ROS_ARGS}
'"
