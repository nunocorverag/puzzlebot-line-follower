#!/usr/bin/env bash
# Drive + record on the Jetson. Keys are read from THIS terminal (tty), so the
# preview can stream over H264 to the laptop.
# Controls: W=forward S=back A=left D=right Q=quit
#
#   scripts/run_teleop_recorder_jetson.sh               # H264 preview (default)
#   STREAM=local scripts/run_teleop_recorder_jetson.sh  # window via ssh -X
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

echo "Controls: W=forward  S=back  A=left  D=right  Q=quit"
echo ""

sync_repo
free_camera
start_stream
run_remote_tool "python3 tools/teleop_recorder.py"
