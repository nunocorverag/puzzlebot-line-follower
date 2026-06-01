#!/usr/bin/env bash
# Raw CSI camera preview over hardware H264 -- NO line follower, NO overlays.
# Pure GStreamer on the Jetson (camera -> nvv4l2h264enc -> UDP) + local viewer.
#
# The CSI camera allows only ONE user at a time, so this stops any running
# follower first (via stop_demo.sh, in its own SSH session). The laptop IP is
# auto-detected. Ctrl+C stops both ends.
#
#   scripts/run_camera_h264_jetson.sh
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
H264_PORT="${H264_PORT:-5000}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-4000000}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
H264_HOST="${H264_HOST:-$(ip route get "${JETSON_HOST}" 2>/dev/null | grep -oP 'src \K[0-9.]+' | head -1)}"

if [ -z "${H264_HOST}" ]; then
  echo "Could not auto-detect laptop IP; set H264_HOST=<ip>." >&2; exit 1
fi
echo "Raw camera H264 -> ${H264_HOST}:${H264_PORT}"

# Free the camera: stop any running follower (separate SSH session; also sends a
# safety zero /cmd_vel). Doing this here avoids a pkill self-kill race.
echo "Freeing camera (stopping any follower)..."
"${SCRIPT_DIR}/stop_demo.sh" >/dev/null 2>&1 || true

# Start the receiver on the laptop.
"${SCRIPT_DIR}/view_h264_stream.sh" "${H264_PORT}" &
RX_PID=$!
trap 'kill "${RX_PID}" 2>/dev/null || true' EXIT

# Pure GStreamer pipeline on the Jetson: capture -> downscale -> HW H264 -> UDP.
# The leading sleep lets the camera fully release after the follower stopped.
ssh "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  sleep 1.5
  gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 ! \
    \"video/x-raw(memory:NVMM),width=1280,height=720,framerate=${FPS}/1\" ! \
    nvvidconv ! \"video/x-raw(memory:NVMM),width=${WIDTH},height=${HEIGHT}\" ! \
    nvv4l2h264enc insert-sps-pps=1 idrinterval=${FPS} bitrate=${BITRATE} maxperf-enable=1 ! \
    h264parse ! rtph264pay config-interval=1 pt=96 ! \
    udpsink host=${H264_HOST} port=${H264_PORT} sync=false async=false
'"
