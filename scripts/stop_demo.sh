#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

SESSION="${SESSION:-line_follower_demo}"
ZERO_TWIST='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

safe_ssh() {
  timeout 5 ssh -o BatchMode=yes -o ConnectTimeout=2 "${JETSON_USER}@${JETSON_HOST}" "$1" 2>/dev/null || true
}

echo "[1/5] Killing local tmux session, H264 receivers, dashboard, tuner..."
tmux kill-session -t "${SESSION}" 2>/dev/null || true
pkill -f "run_line_calibrator_jetson.sh|run_line_follower_jetson.sh|view_h264_stream.sh|gst-launch-1.0 .*udpsrc port=${H264_PORT:-5000}|ffplay .*pb_h264|tools/dashboard.py|run_dashboard.sh|run_param_tuner_jetson.sh|run_control_panel_jetson.sh|tools/param_tuner.py|tools/control_panel.py" 2>/dev/null || true

echo "[2/5] Killing Jetson camera/perception processes..."
safe_ssh "pkill -f 'line_follower|line_detector|autonomous_racer|line_vision_calibrator|tools/recorder.py|cmd_vel_udp_bridge|sign_detector|illumination_calibrator|focus_assist|tilt_assistant|warp_calibrator|param_tuner|control_panel|calib_capture_checkerboard|nvarguscamerasrc|gst-launch|nvv4l2h264enc' 2>/dev/null || true"

echo "[3/5] Publishing zero /cmd_vel burst from Jetson..."
safe_ssh "
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_LOCALHOST_ONLY=0
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  timeout 3 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '${ZERO_TWIST}' 2>/dev/null || true
"

echo "[4/5] Stopping micro-ROS agent..."
safe_ssh "pkill -f micro_ros_agent 2>/dev/null || true"

echo "[5/5] Stop sequence complete. If the robot still moves, cut motor power physically."
