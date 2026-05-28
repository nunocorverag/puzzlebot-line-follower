#!/usr/bin/env bash
set -euo pipefail

SPEED="${1:-0.04}"
DURATION="${2:-1.5}"
JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"

if ! [[ "$SPEED" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
  echo "Invalid SPEED: $SPEED" >&2
  exit 2
fi
if ! [[ "$DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Invalid DURATION: $DURATION" >&2
  exit 2
fi

echo "Jogging /cmd_vel straight: speed=${SPEED} m/s duration=${DURATION}s"
ssh "${JETSON_USER}@${JETSON_HOST}" SPEED="${SPEED}" DURATION="${DURATION}" REMOTE_WS="${REMOTE_WS}" 'bash -s' <<'REMOTE'
set -e
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "${REMOTE_WS}/install/setup.bash" 2>/dev/null || true
CMD="{linear: {x: ${SPEED}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ZERO="{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
timeout "${DURATION}s" ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist "${CMD}" >/dev/null || true
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "${ZERO}" >/dev/null || true
REMOTE
echo "Stopped /cmd_vel"
