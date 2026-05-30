#!/usr/bin/env bash
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"

# Kill any existing camera processes to avoid competing for the CSI device.
# If this still fails with "Failed to create CaptureSession", nvargus-daemon
# needs a restart: run `sudo systemctl restart nvargus-daemon` on the Jetson.
echo "Cleaning up old camera processes..."
ssh "${JETSON_USER}@${JETSON_HOST}" "pkill -f 'video_source' 2>/dev/null; sleep 2; echo ok" || true

ssh "${JETSON_USER}@${JETSON_HOST}" "source /opt/ros/humble/setup.bash && source /home/puzzlebot/ros2_packages_ws/install/local_setup.bash && source ${REMOTE_WS}/install/local_setup.bash && ros2 launch puzzlebot_ros camera_jetson.launch.py"
