#!/usr/bin/env bash
# Run teleop_twist_keyboard on the Jetson.
# Use this alongside run_recorder_jetson.sh to collect a YOLO dataset:
#   Terminal 1: scripts/run_recorder_jetson.sh   (camera + calibration preview)
#   Terminal 2: scripts/teleop_jetson.sh          (drive the robot)
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"

echo "Connecting to ${JETSON_USER}@${JETSON_HOST} — teleop_twist_keyboard"
echo "Controls:  i=fwd  ,=back  j=left  l=right  k=STOP  Ctrl+C=emergency stop"
echo "Starting at low speed (0.2 m/s). Use w/x to adjust."
echo ""

# On exit (Ctrl+C or disconnect) send zero velocity
cleanup() {
  echo ""
  echo "Emergency stop — sending zero velocity..."
  ssh "${JETSON_USER}@${JETSON_HOST}" "source ~/.bashrc && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'" 2>/dev/null || true
  echo "Stopped."
}
trap cleanup EXIT

ssh -t "${JETSON_USER}@${JETSON_HOST}" "source /opt/ros/humble/setup.bash && source /home/puzzlebot/ros2_packages_ws/install/local_setup.bash && source /home/puzzlebot/ros2_ws/install/local_setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p linear_speed_start:=0.2 -p angular_speed_start:=0.5" 2>/dev/null || \
ssh -t "${JETSON_USER}@${JETSON_HOST}" "source /opt/ros/humble/setup.bash && source /home/puzzlebot/ros2_packages_ws/install/local_setup.bash && source /home/puzzlebot/ros2_ws/install/local_setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
