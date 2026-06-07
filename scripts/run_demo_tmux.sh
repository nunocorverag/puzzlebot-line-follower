#!/usr/bin/env bash
# Launch the full robot test stack in tmux, in TWO windows:
#   STACK   : build/sync, motor agent, follower, dashboard  (split panes = logs)
#   CONTROL : the control panel alone, full screen (curses needs the height)
#
# Quitting the control panel (q / Ctrl-C) STOPS EVERYTHING: it pulls the session
# and then tears down the whole stack (local tmux + remote Jetson processes).
#
# Usage:
#   scripts/run_demo_tmux.sh
#   NO_BUILD=1 scripts/run_demo_tmux.sh       # skip colcon build after sync
#   DASH=0 scripts/run_demo_tmux.sh           # no local dashboard (frees UDP 5055)
#   STREAM=none scripts/run_demo_tmux.sh      # no local H264 receiver
#   IGNORE_TRAFFIC_LIGHT=0 scripts/run_demo_tmux.sh  # require the traffic light
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

SESSION="${SESSION:-line_follower_demo}"
READY_FILE="/tmp/${SESSION}_ready"
IGNORE_TRAFFIC_LIGHT="${IGNORE_TRAFFIC_LIGHT:-1}"
NO_BUILD="${NO_BUILD:-0}"
DASH="${DASH:-1}"
MOTOR_BOOT_WAIT_S="${MOTOR_BOOT_WAIT_S:-4}"

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing '$1'. Install it first." >&2
    [ "$1" = tmux ] && echo "  sudo apt install -y tmux" >&2
    exit 1
  fi
}

safe_remote() { ssh -o BatchMode=yes -o ConnectTimeout=5 "${JETSON_USER}@${JETSON_HOST}" "$1"; }

need_cmd tmux; need_cmd ssh; need_cmd rsync

if ! safe_remote 'echo OK' >/dev/null 2>&1; then
  echo "Jetson not reachable at ${JETSON_USER}@${JETSON_HOST}." >&2
  exit 1
fi

# Own the whole local session: kill any old stack, then stop remote robot procs.
tmux kill-session -t "${SESSION}" 2>/dev/null || true
SESSION="${SESSION}" "${SCRIPT_DIR}/stop_demo.sh" || true
rm -f "${READY_FILE}"

# --- Window 1: LOGS (build/motor/follower as split panes) -----------------
tmux new-session -d -s "${SESSION}" -n LOGS
P_BUILD=$(tmux display-message -p -t "${SESSION}:LOGS" '#{pane_id}')
P_MOTOR=$(tmux split-window -t "${P_BUILD}" -P -F '#{pane_id}')
P_FOLLOWER=$(tmux split-window -t "${P_MOTOR}" -P -F '#{pane_id}')
tmux select-layout -t "${SESSION}:LOGS" tiled >/dev/null
tmux set-option -w -t "${SESSION}:LOGS" remain-on-exit on >/dev/null

# --- Window 2: MONITOR (state-machine dashboard, full screen) -------------
if [ "${DASH}" = "1" ]; then
  tmux new-window -t "${SESSION}" -n MONITOR
  P_DASH=$(tmux display-message -p -t "${SESSION}:MONITOR" '#{pane_id}')
fi

# --- Window 3: CONTROL (panel, full screen) -------------------------------
tmux new-window -t "${SESSION}" -n CONTROL
P_PANEL=$(tmux display-message -p -t "${SESSION}:CONTROL" '#{pane_id}')

# 1. Sync/build once (only this pane syncs at startup).
tmux send-keys -t "${P_BUILD}" "echo '[build] sync -> Jetson'; scripts/sync_to_jetson.sh; if [ '${NO_BUILD}' != '1' ]; then echo '[build] colcon build'; scripts/build_on_jetson.sh; else echo '[build] skipped (NO_BUILD=1)'; fi; touch '${READY_FILE}'; echo '[build] READY'; exec bash" C-m

# 2. Motor agent (foreground; ssh -t must not be backgrounded).
tmux send-keys -t "${P_MOTOR}" "while [ ! -f '${READY_FILE}' ]; do echo '[motor] waiting for build...'; sleep 1; done; echo '[motor] starting micro-ROS agent'; scripts/run_motor_agent_jetson.sh; exec bash" C-m

# 3. Follower (after build + a motor head start). Drive stays off until 'd'.
tmux send-keys -t "${P_FOLLOWER}" "while [ ! -f '${READY_FILE}' ]; do echo '[follower] waiting for build...'; sleep 1; done; echo '[follower] motor head start ${MOTOR_BOOT_WAIT_S}s'; sleep '${MOTOR_BOOT_WAIT_S}'; echo '[follower] starting'; IGNORE_TRAFFIC_LIGHT='${IGNORE_TRAFFIC_LIGHT}' STREAM='${STREAM}' scripts/run_line_follower_jetson.sh; rc=\$?; echo '[follower] exited rc='\$rc; exec bash" C-m

# 4. Dashboard (UDP telemetry; skipped when DASH=0).
if [ "${DASH}" = "1" ]; then
  tmux send-keys -t "${P_DASH}" "echo '[dashboard] waiting for telemetry'; scripts/run_dashboard.sh; exec bash" C-m
fi

# 5. Control panel: wait for the param service, run, and on EXIT tear the whole
# stack down. stop_demo is launched detached (setsid) so it survives the tmux
# kill it performs (it would otherwise kill its own pane mid-run).
tmux send-keys -t "${P_PANEL}" "while [ ! -f '${READY_FILE}' ]; do echo '[panel] waiting for build...'; sleep 1; done; echo '[panel] waiting for /autonomous_racer...'; until ssh -o BatchMode=yes -o ConnectTimeout=3 '${JETSON_USER}@${JETSON_HOST}' \"bash -lc 'source /opt/ros/humble/setup.bash; source ${REMOTE_WS}/install/setup.bash 2>/dev/null || true; ros2 service list 2>/dev/null | grep -q /autonomous_racer/get_parameters'\"; do sleep 1; done; echo '[panel] starting (q quits AND stops everything)'; SYNC=0 scripts/run_control_panel_jetson.sh; echo '[panel] exited -> stopping whole stack'; setsid bash -c 'SESSION=${SESSION} ${SCRIPT_DIR}/stop_demo.sh' </dev/null >/tmp/${SESSION}_stop.log 2>&1 &" C-m

tmux select-window -t "${SESSION}:CONTROL"
echo "tmux session '${SESSION}' started. Windows (Ctrl-b n/p):"
echo "  CONTROL = the panel (press d / 1,2,3 here)."
echo "  MONITOR = full-screen state-machine dashboard."
echo "  LOGS    = build/motor/follower panes."
echo "  Quitting the panel (q) stops EVERYTHING (local + Jetson)."
tmux attach-session -t "${SESSION}"
