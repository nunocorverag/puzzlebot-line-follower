#!/usr/bin/env bash
set -euo pipefail

SPEED="${1:-0.08}"
DURATION="${2:-2.0}"
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

# ros2 topic pub always waits ~2s for DDS discovery before sending any message.
# Using rclpy directly avoids this: it publishes immediately and the serial node
# receives messages as soon as DDS discovery completes (~1s).
ssh "${JETSON_USER}@${JETSON_HOST}" SPEED="${SPEED}" DURATION="${DURATION}" REMOTE_WS="${REMOTE_WS}" 'bash -s' <<'REMOTE'
set -e
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "${REMOTE_WS}/install/setup.bash" 2>/dev/null || true
python3 - "${SPEED}" "${DURATION}" <<'PY'
import sys, time
import rclpy
from geometry_msgs.msg import Twist

speed = float(sys.argv[1])
duration = float(sys.argv[2])

rclpy.init()
node = rclpy.create_node('jog_node')
pub = node.create_publisher(Twist, '/cmd_vel', 10)

msg = Twist()
msg.linear.x = speed

end_time = time.time() + duration
while time.time() < end_time:
    pub.publish(msg)
    time.sleep(0.05)

msg.linear.x = 0.0
for _ in range(10):
    pub.publish(msg)
    time.sleep(0.05)

node.destroy_node()
rclpy.shutdown()
PY
REMOTE

echo "Stopped /cmd_vel"
