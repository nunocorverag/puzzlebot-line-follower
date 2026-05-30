#!/usr/bin/env bash
# Starts robot + camera + recorder in the background on the Jetson.
# Usage:
#   scripts/start_all_jetson.sh        # start everything
#   scripts/start_all_jetson.sh stop   # kill everything
#   scripts/start_all_jetson.sh logs   # tail live logs
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
REMOTE_PKG="${REMOTE_WS}/src/puzzlebot_ros"
ROS="source /opt/ros/humble/setup.bash && source /home/puzzlebot/ros2_packages_ws/install/local_setup.bash && source ${REMOTE_WS}/install/local_setup.bash"

case "${1:-start}" in

  stop)
    echo "Killing all processes on the Jetson..."
    ssh "${JETSON_USER}@${JETSON_HOST}" '
      kill $(cat /tmp/robot.pid   2>/dev/null) 2>/dev/null || true
      kill $(cat /tmp/cam.pid     2>/dev/null) 2>/dev/null || true
      kill $(cat /tmp/rec.pid     2>/dev/null) 2>/dev/null || true
      pkill -f "micro_ros_agent"  2>/dev/null || true
      pkill -f "video_source"     2>/dev/null || true
      pkill -f "recorder.py"      2>/dev/null || true
      echo "done"
    '
    ;;

  logs)
    echo "=== ROBOT ===" && ssh "${JETSON_USER}@${JETSON_HOST}" 'tail -5 /tmp/robot.log 2>/dev/null || echo "no log"'
    echo "=== CAMERA ===" && ssh "${JETSON_USER}@${JETSON_HOST}" 'tail -5 /tmp/cam.log 2>/dev/null || echo "no log"'
    echo "=== RECORDER ===" && ssh "${JETSON_USER}@${JETSON_HOST}" 'tail -10 /tmp/rec.log 2>/dev/null || echo "no log"'
    ;;

  start|*)
    echo "Starting everything on the Jetson..."
    ssh "${JETSON_USER}@${JETSON_HOST}" "
      # Kill previous processes
      kill \$(cat /tmp/robot.pid 2>/dev/null) 2>/dev/null || true
      kill \$(cat /tmp/cam.pid   2>/dev/null) 2>/dev/null || true
      kill \$(cat /tmp/rec.pid   2>/dev/null) 2>/dev/null || true
      sleep 1

      # 1) Robot
      nohup bash -c '${ROS} && bash ~/start_robot.sh' > /tmp/robot.log 2>&1 &
      echo \$! > /tmp/robot.pid
      echo '[1/3] micro_ros_agent starting...'
      sleep 4

      # 2) Camera
      nohup bash -c '${ROS} && ros2 launch puzzlebot_ros camera_jetson.launch.py' > /tmp/cam.log 2>&1 &
      echo \$! > /tmp/cam.pid
      echo '[2/3] camera starting...'
      sleep 4

      # 3) Recorder
      nohup bash -c '${ROS} && cd ${REMOTE_PKG} && python3 tools/recorder.py --interval 0.5 --output-dir dataset' > /tmp/rec.log 2>&1 &
      echo \$! > /tmp/rec.pid
      echo '[3/3] recorder starting...'
    "

    echo ""
    echo "Everything running in the background. Useful commands:"
    echo "  scripts/start_all_jetson.sh logs   # see what is happening"
    echo "  scripts/start_all_jetson.sh stop   # kill everything"
    echo "  scripts/teleop_jetson.sh            # drive the robot"
    ;;
esac
