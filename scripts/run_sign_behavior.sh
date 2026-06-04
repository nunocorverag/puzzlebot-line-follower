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

# Inyectamos las variables y corremos YOLO en el fondo (&) y el State Machine en primer plano.
# Cuando presiones Ctrl+C, el script matará el proceso de YOLO automáticamente.
run_remote_tool "env -u PYTHONNOUSERSITE \
  H264_HOST=${H264_HOST} \
  H264_PORT=${H264_PORT:-5000} \
  FPS=${FPS} \
  WIDTH=${WIDTH} \
  HEIGHT=${HEIGHT} \
  BITRATE=${BITRATE} \
  bash -c '
    LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 python3 tools/sign_detector.py --confidence ${CONFIDENCE} &
    YOLO_PID=\$!
    python3 tools/sign_state_machine.py
    kill \$YOLO_PID 2>/dev/null || true
  '"