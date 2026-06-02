#!/usr/bin/env bash
# Start the micro-ROS agent: the /cmd_vel -> motors bridge on the Jetson (this
# is what actually drives the wheels; the line follower / teleop only publish
# /cmd_vel). It also brings up the encoder topics (/VelocityEncL, /VelocityEncR,
# /robot_vel). Does NOT touch the CSI camera, so it runs alongside the follower.
# Run in its own terminal and leave it open; Ctrl-C stops it.
#
#   scripts/run_motor_agent_jetson.sh
#
# Uses the canonical launch (ros2 launch puzzlebot_ros micro_ros_agent.launch.py)
# via the Jetson's ~/start_robot.sh, which sources the right overlays.
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"

ssh -t "${JETSON_USER}@${JETSON_HOST}" "bash -lc 'exec bash ~/start_robot.sh'"
