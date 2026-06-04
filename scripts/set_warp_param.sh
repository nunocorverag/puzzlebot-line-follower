#!/usr/bin/env bash
# Push a warp/lane parameter to a running warp_calibrator.py over SSH.
#
#   scripts/set_warp_param.sh src_top_half_w_pct 16
#   scripts/set_warp_param.sh mask_method 1      # 0=Otsu, 1=adaptive
#   scripts/set_warp_param.sh save_lane 1        # -> config/lane_params.json
#   scripts/set_warp_param.sh q 1                # quit
#
# Any LaneParams field name is accepted; plus save_lane, q, p (pause),
# u (undistort), reset.
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 PARAM VALUE" >&2
  echo "Examples:
  $0 src_top_y_pct 55
  $0 save_lane 1" >&2
  exit 2
fi

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
COMMAND_FILE="${COMMAND_FILE:-${REMOTE_WS}/src/puzzlebot_ros/debug_dataset/warp_commands.txt}"
PARAM="$1"
VALUE="$2"

ssh "${JETSON_USER}@${JETSON_HOST}" "mkdir -p '$(dirname "${COMMAND_FILE}")' && printf '%s=%s\n' '${PARAM}' '${VALUE}' > '${COMMAND_FILE}'"
echo "Set ${PARAM}=${VALUE} via ${JETSON_USER}@${JETSON_HOST}:${COMMAND_FILE}"
