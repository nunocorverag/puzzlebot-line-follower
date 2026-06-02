#!/usr/bin/env bash
# TRUE-combo WASD teleop. Reads the LAPTOP keyboard with real key state (pygame),
# so holding w+a together is a genuine curve. Velocity is sent over UDP to a tiny
# bridge on the Jetson that republishes /cmd_vel (no ROS needed on the laptop).
#
# Requires: motor agent running (scripts/run_motor_agent_jetson.sh) and NO other
# /cmd_vel publisher (stop the line follower first).
#
#   scripts/run_teleop_wasd_combo.sh
#   V_MAX=0.18 STEER_W=0.45 PIVOT_W=1.3 scripts/run_teleop_wasd_combo.sh
#
# Controls (focus the window): w/s fwd/back  a/d steer  q/e pivot  space stop
#                              - / = speed   ESC/close = quit
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"
UDP_PORT="${UDP_PORT:-5005}"

echo "Syncing repo + starting UDP->/cmd_vel bridge on the Jetson..."
sync_repo
# Heredoc + positional args (avoids nested-quote/escaping bugs over ssh).
ssh "${JETSON_USER}@${JETSON_HOST}" 'bash -s' \
    "${UDP_PORT}" "${PACKAGES_WS}" "${REMOTE_WS}" "${REMOTE_PKG}" <<'REMOTE'
set -e
UDP_PORT="$1"; PACKAGES_WS="$2"; REMOTE_WS="$3"; REMOTE_PKG="$4"
pkill -f cmd_vel_udp_bridge 2>/dev/null || true
sleep 0.5
source /opt/ros/humble/setup.bash
source "${PACKAGES_WS}/install/local_setup.bash" 2>/dev/null || true
source "${REMOTE_WS}/install/local_setup.bash" 2>/dev/null || true
cd "${REMOTE_PKG}"
setsid env UDP_PORT="${UDP_PORT}" python3 tools/cmd_vel_udp_bridge.py > /tmp/udp_bridge.log 2>&1 < /dev/null &
sleep 1.5
if pgrep -f cmd_vel_udp_bridge >/dev/null; then echo bridge-started; else echo BRIDGE-FAILED; cat /tmp/udp_bridge.log; exit 1; fi
REMOTE

cleanup() {
  echo ""
  echo "Stopping bridge (it zeroes /cmd_vel on exit)..."
  ssh "${JETSON_USER}@${JETSON_HOST}" 'pkill -f cmd_vel_udp_bridge 2>/dev/null || true' 2>/dev/null || true
}
trap cleanup EXIT

echo "Launching teleop window. FOCUS it to drive. ESC / close = quit."
JETSON_HOST="${JETSON_HOST}" UDP_PORT="${UDP_PORT}" \
  V_MAX="${V_MAX:-0.15}" STEER_W="${STEER_W:-0.5}" PIVOT_W="${PIVOT_W:-1.2}" \
  python3 "${SCRIPT_DIR}/../tools/teleop_wasd_gui.py"
