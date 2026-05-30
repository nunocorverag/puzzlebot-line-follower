#!/usr/bin/env bash
# Runs teleop_recorder directly on the Jetson via interactive SSH.
# Controls: W=forward S=back A=left D=right Q=quit
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
REMOTE_PKG="${REMOTE_WS}/src/puzzlebot_ros"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${SCRIPT_DIR}/sync_to_jetson.sh"

echo "Controls: W=forward  S=back  A=left  D=right  Q=quit"
echo ""

xhost +local: >/dev/null 2>&1 || true
ssh -X -t "${JETSON_USER}@${JETSON_HOST}" "source /opt/ros/humble/setup.bash && source /home/puzzlebot/ros2_packages_ws/install/local_setup.bash && source ${REMOTE_WS}/install/local_setup.bash && cd ${REMOTE_PKG} && python3 tools/teleop_recorder.py"
