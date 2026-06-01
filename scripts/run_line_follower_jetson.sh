#!/usr/bin/env bash
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
NODE="${NODE:-line_follower}"

# Optional H264 stream: STREAM_MODE=h264 [H264_HOST=auto] [H264_PORT=5000]
# H264_HOST defaults to this laptop's IP on the robot network (auto-detected),
# so on RoboNet you normally never set it.
STREAM_MODE="${STREAM_MODE:-}"
H264_HOST="${H264_HOST:-}"
H264_PORT="${H264_PORT:-5000}"
ROS_ARGS=""
if [ "${STREAM_MODE}" = "h264" ]; then
  if [ -z "${H264_HOST}" ]; then
    H264_HOST="$(ip route get "${JETSON_HOST}" 2>/dev/null | grep -oP 'src \K[0-9.]+' | head -1)"
  fi
  if [ -z "${H264_HOST}" ]; then
    echo "Could not auto-detect laptop IP; set H264_HOST=<ip>." >&2; exit 1
  fi
  echo "H264 stream -> ${H264_HOST}:${H264_PORT}"
  ROS_ARGS="--ros-args -p stream_mode:=h264 -p h264_host:=${H264_HOST} -p h264_port:=${H264_PORT}"
elif [ -n "${STREAM_MODE}" ]; then
  ROS_ARGS="--ros-args -p stream_mode:=${STREAM_MODE}"
fi

xhost +local: >/dev/null 2>&1 || true

ssh -X "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  cd "${REMOTE_WS}"
  source /opt/ros/humble/setup.bash
  source src/puzzlebot_ros/env_jetson.sh
  source install/setup.bash
  export PYTHONNOUSERSITE=1
  ros2 run puzzlebot_ros "${NODE}" ${ROS_ARGS}
'"
