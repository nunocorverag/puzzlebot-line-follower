#!/usr/bin/env bash
# Live telemetry dashboard (runs on the LAPTOP). Shows the follower's state
# machine, zebra detection, lane metrics, live params and a rolling log.
#
# The follower broadcasts JSON over UDP to the laptop (it reuses the H264 host,
# so just run the follower normally with the default H264 stream). Open this in
# its own terminal -- it is independent of the video stream window.
#
#   scripts/run_dashboard.sh
#   PORT=5055 scripts/run_dashboard.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PORT="${PORT:-5055}"

echo "Telemetry dashboard on UDP :${PORT}  (Ctrl-C to quit)"
exec python3 "${REPO_DIR}/tools/dashboard.py" --port "${PORT}"
