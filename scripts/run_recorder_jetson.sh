#!/usr/bin/env bash
# See the camera (H264 preview) and record CLEAN frames on the Jetson, with
# manual start/pause. Press Enter in THIS terminal to toggle recording (the
# preview shows RECORDING/PAUSED). On quit the session is pulled to the laptop
# and wiped on the robot:
#     datasets/<CATEGORY>/<SESSION>/        (CATEGORY defaults to "recordings")
#
# Pair it with scripts/run_teleop_wasd_combo.sh in another terminal to drive
# (WASD) while you watch the camera here and record only the parts you want.
#
#   CATEGORY=illumination scripts/run_recorder_jetson.sh   # -> datasets/illumination/<ts>/
#   INTERVAL=0.3 CATEGORY=signs scripts/run_recorder_jetson.sh
#   STREAM=none scripts/run_recorder_jetson.sh             # headless (no preview)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

INTERVAL="${INTERVAL:-0.5}"
CATEGORY="${CATEGORY:-recordings}"
SESSION="$(date +%Y%m%d_%H%M%S)"
REMOTE_DIR="/home/${JETSON_USER}/record_sessions/${SESSION}"
LOCAL_DEST="${REPO_DIR}/datasets/${CATEGORY}/${SESSION}"

echo "Recording -> datasets/${CATEGORY}/${SESSION}/   (Enter = start/pause, Ctrl+C = quit)"
sync_repo
free_camera
start_stream   # sets its own EXIT trap (kills the H264 receiver)

# Replace it with one that ALSO archives + wipes the session on exit.
trap 'pull_and_clean_session "${REMOTE_DIR}" "${LOCAL_DEST}"; [ -n "${H264_RX_PID:-}" ] && kill "${H264_RX_PID}" 2>/dev/null || true' EXIT

run_remote_tool "python3 tools/recorder.py --interval ${INTERVAL} --output-dir '${REMOTE_DIR}'"
