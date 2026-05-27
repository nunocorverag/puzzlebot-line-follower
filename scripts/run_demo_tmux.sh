#!/usr/bin/env bash
set -euo pipefail

SESSION="${SESSION:-line_follower_demo}"
JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
READY_FILE="/tmp/${SESSION}_build_ready"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed. Install it with: sudo apt install -y tmux" >&2
  exit 1
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  tmux kill-session -t "${SESSION}"
fi

rm -f "${READY_FILE}"

tmux new-session -d -s "${SESSION}" -n demo
P_SYNC=$(tmux display-message -p -t "${SESSION}:demo" '#{pane_id}')
P_MICRO=$(tmux split-window -h -t "${P_SYNC}" -P -F '#{pane_id}')
P_LINE=$(tmux split-window -v -t "${P_SYNC}" -P -F '#{pane_id}')
P_MONITOR=$(tmux split-window -v -t "${P_MICRO}" -P -F '#{pane_id}')
tmux select-layout -t "${SESSION}:demo" tiled

tmux send-keys -t "${P_SYNC}" \
  "scripts/sync_to_jetson.sh && scripts/build_on_jetson.sh && touch ${READY_FILE}; echo 'Jetson sync/build done. MJPEG: http://${JETSON_HOST}:8080'; exec bash" C-m

tmux send-keys -t "${P_MICRO}" \
  "ssh ${JETSON_USER}@${JETSON_HOST} 'bash -lc \"source /opt/ros/humble/setup.bash; source ${REMOTE_WS}/src/puzzlebot_ros/env_jetson.sh; source ~/ros2_packages_ws/install/setup.bash 2>/dev/null || true; ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyUSB0 -v 6\"'" C-m

tmux send-keys -t "${P_LINE}" \
  "while [ ! -f ${READY_FILE} ]; do echo 'Waiting for Jetson build...'; sleep 1; done; xhost +local: >/dev/null 2>&1 || true; ssh -X ${JETSON_USER}@${JETSON_HOST} 'bash -lc \"cd ${REMOTE_WS}; source /opt/ros/humble/setup.bash; source src/puzzlebot_ros/env_jetson.sh; source install/setup.bash; ros2 run puzzlebot_ros line_follower\"'" C-m

tmux send-keys -t "${P_MONITOR}" \
  "ssh ${JETSON_USER}@${JETSON_HOST} 'bash -lc \"source /opt/ros/humble/setup.bash; source ${REMOTE_WS}/src/puzzlebot_ros/env_jetson.sh; source ${REMOTE_WS}/install/setup.bash; watch -n 1 ros2 topic list\"'" C-m

tmux attach-session -t "${SESSION}"
