#!/usr/bin/env bash
set -u

SESSION="${SESSION:-line_follower_demo}"
JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
ZERO_TWIST='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

safe_ssh() {
  timeout 5 ssh -o BatchMode=yes -o ConnectTimeout=2 "${JETSON_USER}@${JETSON_HOST}" "$1" 2>/dev/null || true
}

echo "[1/4] Killing local tmux session..."
tmux kill-session -t "${SESSION}" 2>/dev/null || true

echo "[2/4] Killing Jetson line follower processes..."
safe_ssh "pkill -f 'line_follower|line_detector|autonomous_racer' 2>/dev/null || true"

echo "[3/4] Publishing zero /cmd_vel burst from Jetson..."
safe_ssh "
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_LOCALHOST_ONLY=0
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  timeout 3 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '${ZERO_TWIST}' 2>/dev/null || true
"

echo "[4/4] Stopping micro-ROS agent..."
safe_ssh "pkill -f micro_ros_agent 2>/dev/null || true"

echo "Stop sequence complete. If the robot still moves, cut motor power physically."
