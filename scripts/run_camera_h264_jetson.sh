#!/usr/bin/env bash
# Raw CSI camera preview over hardware H264 -- NO line follower, NO overlays.
# Pure GStreamer on the Jetson (camera -> nvv4l2h264enc -> UDP) + local viewer.
#
# The CSI camera allows only ONE user at a time, so this frees it first. The
# laptop IP is auto-detected. Ctrl+C stops both ends.
#
#   scripts/run_camera_h264_jetson.sh
set -euo pipefail

STREAM=h264   # this script is inherently an H264 stream

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-4000000}"

echo "Freeing camera..."
free_camera

start_stream   # auto-detects H264_HOST, launches the local receiver

# Pure GStreamer pipeline on the Jetson: capture -> downscale -> HW H264 -> UDP.
# The leading sleep lets the camera fully release after anything was stopped.
ssh "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
  sleep 1.5
  gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 ! \
    \"video/x-raw(memory:NVMM),width=1280,height=720,framerate=${FPS}/1\" ! \
    nvvidconv ! \"video/x-raw(memory:NVMM),width=${WIDTH},height=${HEIGHT}\" ! \
    nvv4l2h264enc insert-sps-pps=1 idrinterval=${FPS} bitrate=${BITRATE} maxperf-enable=1 ! \
    h264parse ! rtph264pay config-interval=1 pt=96 ! \
    udpsink host=${H264_HOST} port=${H264_PORT} sync=false async=false
'"
