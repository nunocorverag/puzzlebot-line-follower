#!/usr/bin/env bash
JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"

echo "Killing all robot processes..."
ssh "${JETSON_USER}@${JETSON_HOST}" '
  pkill -f "recorder.py"      2>/dev/null
  pkill -f "video_source"     2>/dev/null
  pkill -f "micro_ros_agent"  2>/dev/null
  pkill -f "camera_info"      2>/dev/null
  pkill -f "teleop_twist"     2>/dev/null
  # zero velocity just in case
  source /opt/ros/humble/setup.bash 2>/dev/null
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {x: 0.0}}" 2>/dev/null || true
  echo "done"
'
