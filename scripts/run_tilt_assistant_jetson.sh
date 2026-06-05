#!/usr/bin/env bash
# Live camera tilt (pitch) assistant on the Jetson. H264 dashboard shows the
# estimated pitch in degrees (0 = horizontal), the horizon, the ground/far
# bands and a quality score. Arm capture with `scripts/set_tilt_param.sh start 1`,
# then tilt the camera in small steps HOLDING at each angle; it snapshots each
# held pose to debug_dataset/tilt_session/ and, on quit, prints a ranking of the
# best tilts. `scripts/set_tilt_param.sh save 1` writes config/camera_pose.json.
#
#   scripts/run_tilt_assistant_jetson.sh
#   CAMERA_HEIGHT_CM=13 scripts/run_tilt_assistant_jetson.sh   # also show distances
#   RELEVEL=1 scripts/run_tilt_assistant_jetson.sh             # return to saved setpoint
set -euo pipefail

STREAM="${STREAM:-h264}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

EXTRA_ARGS=""
if [ -n "${CAMERA_HEIGHT_CM:-}" ]; then
  EXTRA_ARGS="${EXTRA_ARGS} --camera-height-cm ${CAMERA_HEIGHT_CM}"
fi
if [ "${RELEVEL:-0}" = "1" ]; then
  EXTRA_ARGS="${EXTRA_ARGS} --relevel"
fi

sync_repo
free_camera
start_stream   # sets its own EXIT trap (kills the H264 receiver)

# On exit, pull this session's tilt snapshots to the laptop and wipe the Jetson.
TILT_SESSION="$(date +%Y%m%d_%H%M%S)"
trap 'pull_and_clean_session "${REMOTE_PKG}/debug_dataset/tilt_session" "${REPO_DIR}/datasets/calibration/tilt_session/${TILT_SESSION}"; [ -n "${H264_RX_PID:-}" ] && kill "${H264_RX_PID}" 2>/dev/null || true' EXIT

run_remote_tool "python3 tools/tilt_assistant.py --gstreamer --camera-params config/camera_params.npz --illumination-params config/illumination_flatfield.npz ${EXTRA_ARGS}"
