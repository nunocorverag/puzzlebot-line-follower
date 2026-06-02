#!/usr/bin/env bash
# Shared helpers for every run_*_jetson.sh script. Source it near the top:
#
#   SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#   source "${SCRIPT_DIR}/lib/common.sh"
#
# Then typically:
#   sync_repo
#   free_camera                       # optional, frees the CSI before starting
#   start_stream                      # launches the local H264 receiver if STREAM=h264
#   run_remote_tool "python3 tools/sign_detector.py --confidence 0.45"
#
# This centralises the SSH/sourcing boilerplate, laptop-IP autodetection and the
# unified STREAM (h264|local|none) handling that used to differ per script.

# --- Connection / paths (all overridable via env) ---
JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
REMOTE_PKG="${REMOTE_WS}/src/puzzlebot_ros"
PACKAGES_WS="${PACKAGES_WS:-/home/${JETSON_USER}/ros2_packages_ws}"

# --- Unified streaming knobs ---
STREAM="${STREAM:-h264}"            # h264 | local | none
H264_PORT="${H264_PORT:-5000}"
H264_BITRATE="${H264_BITRATE:-2000000}"

# --- Resolve repo dirs from this file's location ---
COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(cd "${COMMON_DIR}/.." && pwd)"
REPO_DIR="$(cd "${SCRIPTS_DIR}/.." && pwd)"

H264_RX_PID=""

sync_repo() {
  "${SCRIPTS_DIR}/sync_to_jetson.sh"
}

detect_laptop_ip() {
  ip route get "${JETSON_HOST}" 2>/dev/null | grep -oP 'src \K[0-9.]+' | head -1
}

# Pull an artifact from the Jetson package dir back to the laptop repo. The
# laptop is the source of truth (a later sync_repo re-pushes it idempotently).
#   fetch_from_jetson <path-relative-to-REMOTE_PKG> <local-dest>
fetch_from_jetson() {
  local remote_rel="$1" local_dest="$2"
  mkdir -p "$(dirname "${local_dest%/}")"
  rsync -az "${JETSON_USER}@${JETSON_HOST}:${REMOTE_PKG}/${remote_rel}" "${local_dest}" \
    && echo "fetched ${remote_rel} -> ${local_dest}"
}

# Free the CSI camera (single-owner) WITHOUT killing the micro-ROS agent, so
# motion tools (teleop, follower) keep their /cmd_vel bridge alive. Covers every
# camera consumer: ROS nodes, the standalone tools, AND the raw GStreamer
# pipeline (nvarguscamerasrc/gst-launch) left by run_camera_h264_jetson.sh.
free_camera() {
  ssh -o BatchMode=yes -o ConnectTimeout=3 "${JETSON_USER}@${JETSON_HOST}" \
    "pkill -f 'video_source|line_follower|autonomous_racer|sign_detector|tools/recorder.py|teleop_recorder|line_vision_calibrator|illumination_calibrator|focus_assist|calib_capture_checkerboard|nvarguscamerasrc|gst-launch' 2>/dev/null; sleep 1.5; true" \
    >/dev/null 2>&1 || true
}

# Launch the local H264 receiver when STREAM=h264 and arrange cleanup on exit.
start_stream() {
  case "${STREAM}" in
    h264)
      H264_HOST="${H264_HOST:-$(detect_laptop_ip)}"
      if [ -z "${H264_HOST}" ]; then
        echo "STREAM=h264 but could not detect laptop IP; set H264_HOST=<ip>." >&2
        exit 1
      fi
      echo "H264 stream <- Jetson  (receiver on ${H264_HOST}:${H264_PORT})"
      "${SCRIPTS_DIR}/view_h264_stream.sh" "${H264_PORT}" &
      H264_RX_PID=$!
      trap 'kill "${H264_RX_PID}" 2>/dev/null || true' EXIT
      ;;
    local|none)
      : ;;
    *)
      echo "Unknown STREAM='${STREAM}' (use: h264 | local | none)" >&2
      exit 1 ;;
  esac
}

# Env exports forwarded to the remote tool so Preview.from_env() picks the mode.
_stream_env_exports() {
  printf 'export STREAM=%q H264_PORT=%q H264_BITRATE=%q' \
    "${STREAM}" "${H264_PORT}" "${H264_BITRATE}"
  [ "${STREAM}" = "h264" ] && printf ' H264_HOST=%q' "${H264_HOST}"
}

# Run a tool on the Jetson with the canonical sourcing + stream env. Allocates a
# TTY (interactive tools) and X-forwards only when a local window is requested.
run_remote_tool() {
  local tool_cmd="$1"
  local flags="-t"
  if [ "${STREAM}" = "local" ]; then
    flags="-t -X"
    xhost +local: >/dev/null 2>&1 || true
  fi
  local remote="
    source /opt/ros/humble/setup.bash
    source ${PACKAGES_WS}/install/local_setup.bash
    source ${REMOTE_WS}/install/local_setup.bash
    [ -f ${REMOTE_PKG}/env_jetson.sh ] && source ${REMOTE_PKG}/env_jetson.sh
    cd ${REMOTE_PKG}
    export PYTHONNOUSERSITE=1
    $(_stream_env_exports)
    ${tool_cmd}
  "
  # shellcheck disable=SC2086
  ssh ${flags} "${JETSON_USER}@${JETSON_HOST}" "${remote}"
}
