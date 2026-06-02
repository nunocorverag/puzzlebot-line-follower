#!/usr/bin/env bash
# Robust real-time WASD teleop. Keys are read from THIS terminal over ssh -t and
# published straight to /cmd_vel via rclpy (low latency). Needs the motor bridge
# running (scripts/run_motor_agent_jetson.sh) and NO other /cmd_vel publisher
# (stop the line follower first, or they will fight).
#
#   scripts/run_teleop_wasd_jetson.sh
#   V_MAX=0.18 STEER_W=0.5 PIVOT_W=1.2 scripts/run_teleop_wasd_jetson.sh
#
# Controls: w/s = fwd/back   a/d = STEER (curve)   q/e = PIVOT in place
#           x = stop linear  z = straighten   space = STOP
#           - / = = slower/faster   Ctrl-C = quit
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

echo "WASD  ->  w/s fwd/back | a/d steer(curve) | q/e pivot | space STOP | Ctrl-C quit"
echo "Wheels up / clear space first."
echo ""

sync_repo
run_remote_tool "V_MAX=${V_MAX:-0.15} STEER_W=${STEER_W:-0.5} PIVOT_W=${PIVOT_W:-1.2} python3 tools/teleop_wasd.py"
