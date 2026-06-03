#!/usr/bin/env bash
# Run the YOLO traffic-sign + traffic-light detector on the Jetson.
# Captures from CSI via GStreamer, overlays detections, and streams H264 UDP.
#
#   scripts/run_sign_detector_jetson.sh
#   CONFIDENCE=0.4 scripts/run_sign_detector_jetson.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

CONFIDENCE="${CONFIDENCE:-0.45}"
FPS="${FPS:-30}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
BITRATE="${BITRATE:-4000000}"

sync_repo

# Make sure ultralytics is available on the Jetson.
# ssh "${JETSON_USER}@${JETSON_HOST}" \
#  "python3 -c 'import ultralytics' 2>/dev/null || python3 -m pip install --user ultralytics --quiet" || true

echo "Freeing camera..."
free_camera

start_stream # Auto-detects H264_HOST and launches the local receiver

# Inject the necessary streaming variables explicitly into the remote env command
# Inject the necessary streaming variables explicitly into the remote env command
# Inject the necessary streaming variables explicitly into the remote env command
run_remote_tool "env -u PYTHONNOUSERSITE \
  H264_HOST=${H264_HOST} \
  H264_PORT=${H264_PORT:-5000} \
  FPS=${FPS} \
  WIDTH=${WIDTH} \
  HEIGHT=${HEIGHT} \
  BITRATE=${BITRATE} \
  LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 \
  python3 tools/sign_detector.py --confidence ${CONFIDENCE}"