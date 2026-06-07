#!/usr/bin/env bash
# Launch the full robot test stack in tmux:
#   build/sync -> motor agent -> line follower -> dashboard -> control panel
#
# Usage:
#   scripts/run_demo_tmux.sh
#   SESSION=puzzlebot_test IGNORE_TRAFFIC_LIGHT=1 scripts/run_demo_tmux.sh
#   NO_BUILD=1 scripts/run_demo_tmux.sh       # skip colcon build after sync
#   STREAM=none scripts/run_demo_tmux.sh      # no local H264 receiver
#   IGNORE_TRAFFIC_LIGHT=0 scripts/run_demo_tmux.sh  # require the traffic light
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

SESSION="${SESSION:-line_follower_demo}"
READY_FILE="/tmp/${SESSION}_ready"
IGNORE_TRAFFIC_LIGHT="${IGNORE_TRAFFIC_LIGHT:-1}"
NO_BUILD="${NO_BUILD:-0}"
DASH="${DASH:-1}"            # DASH=0 -> no local dashboard (frees UDP 5055 so a
                            # co-pilot can capture the telemetry on the laptop)
MOTOR_BOOT_WAIT_S="${MOTOR_BOOT_WAIT_S:-4}"

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing '$1'. Install it first." >&2
    [ "$1" = tmux ] && echo "  sudo apt install -y tmux" >&2
    exit 1
  fi
}

safe_remote() {
  ssh -o BatchMode=yes -o ConnectTimeout=5 "${JETSON_USER}@${JETSON_HOST}" "$1"
}

need_cmd tmux
need_cmd ssh
need_cmd rsync

if ! safe_remote 'echo OK' >/dev/null 2>&1; then
  echo "Jetson not reachable at ${JETSON_USER}@${JETSON_HOST}." >&2
  exit 1
fi

# Own the whole local session. If an older tmux stack exists, kill it first; then
# stop remote robot processes so there is one camera owner and one /cmd_vel stack.
tmux kill-session -t "${SESSION}" 2>/dev/null || true
SESSION="${SESSION}" "${SCRIPT_DIR}/stop_demo.sh" || true
rm -f "${READY_FILE}"

# Window layout: one window per long-running process. This is more robust than
# packing many panes into one terminal; tmux may refuse tiny panes and silently
# leave the follower unstarted.
tmux new-session -d -s "${SESSION}" -n BUILD
P_BUILD=$(tmux display-message -p -t "${SESSION}:BUILD" '#{pane_id}')
tmux new-window -t "${SESSION}" -n MOTOR
P_MOTOR=$(tmux display-message -p -t "${SESSION}:MOTOR" '#{pane_id}')
tmux new-window -t "${SESSION}" -n FOLLOWER
tmux set-option -t "${SESSION}:FOLLOWER" remain-on-exit on >/dev/null
P_FOLLOWER=$(tmux display-message -p -t "${SESSION}:FOLLOWER" '#{pane_id}')
if [ "${DASH}" = "1" ]; then
  tmux new-window -t "${SESSION}" -n DASH
  P_DASH=$(tmux display-message -p -t "${SESSION}:DASH" '#{pane_id}')
fi
tmux new-window -t "${SESSION}" -n CONTROL_PRESS_D
P_PANEL=$(tmux display-message -p -t "${SESSION}:CONTROL_PRESS_D" '#{pane_id}')

# 1. Sync/build once. This is the only pane allowed to sync during startup.
tmux send-keys -t "${P_BUILD}" "set -e; echo '[build] sync -> Jetson'; scripts/sync_to_jetson.sh; if [ '${NO_BUILD}' != '1' ]; then echo '[build] colcon build'; scripts/build_on_jetson.sh; else echo '[build] skipped (NO_BUILD=1)'; fi; touch '${READY_FILE}'; echo '[build] READY'; exec bash" C-m

# 2. Motor agent. Run it in the foreground in its own window. Do NOT background it:
# run_motor_agent_jetson.sh uses ssh -t, and backgrounding an interactive ssh can
# leave the agent stopped before it creates the /cmd_vel bridge.
tmux send-keys -t "${P_MOTOR}" "set -e; while [ ! -f '${READY_FILE}' ]; do echo '[motor] waiting for build/sync...'; sleep 1; done; echo '[motor] starting micro-ROS agent (foreground)'; scripts/run_motor_agent_jetson.sh; exec bash" C-m

# 3. Dashboard can wait for UDP telemetry immediately (skipped when DASH=0).
if [ "${DASH}" = "1" ]; then
  tmux send-keys -t "${P_DASH}" "echo '[dashboard] waiting for telemetry'; scripts/run_dashboard.sh" C-m
fi

# 4. Follower waits for build, then gives the motor-agent window a short head
# start. Drive remains off until the control panel sends /drive_enable.
tmux send-keys -t "${P_FOLLOWER}" "while [ ! -f '${READY_FILE}' ]; do echo '[follower] waiting for build/sync...'; sleep 1; done; echo '[follower] waiting ${MOTOR_BOOT_WAIT_S}s for motor agent startup...'; sleep '${MOTOR_BOOT_WAIT_S}'; echo '[follower] starting'; IGNORE_TRAFFIC_LIGHT='${IGNORE_TRAFFIC_LIGHT}' STREAM='${STREAM}' scripts/run_line_follower_jetson.sh; rc=\$?; echo '[follower] exited rc='\$rc; exec bash" C-m

# 5. Control panel starts after follower parameter service exists. It owns session pull.
tmux send-keys -t "${P_PANEL}" "set -e; while [ ! -f '${READY_FILE}' ]; do echo '[panel] waiting for build/sync...'; sleep 1; done; echo '[panel] waiting for /autonomous_racer param service...'; until ssh -o BatchMode=yes -o ConnectTimeout=3 '${JETSON_USER}@${JETSON_HOST}' \"bash -lc 'source /opt/ros/humble/setup.bash; source ${REMOTE_WS}/install/setup.bash 2>/dev/null || true; ros2 service list 2>/dev/null | grep -q \\\"/autonomous_racer/get_parameters\\\"'\"; do sleep 1; done; echo '[panel] starting (quit q/Ctrl-C to pull session)'; SYNC=0 scripts/run_control_panel_jetson.sh; exec bash" C-m

tmux select-window -t "${SESSION}:CONTROL_PRESS_D"
echo "tmux session '${SESSION}' started. Attach with: tmux attach -t ${SESSION}"
echo "Windows: CONTROL_PRESS_D panel; BUILD/MOTOR/FOLLOWER/DASH logs (Ctrl-b n/p)."
echo "Follower default: IGNORE_TRAFFIC_LIGHT=${IGNORE_TRAFFIC_LIGHT} (use 0 to require semaphore)."
echo "Control panel owns session archival: quit that pane to pull events/CSV/snapshots."
tmux attach-session -t "${SESSION}"
