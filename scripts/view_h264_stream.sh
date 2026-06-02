#!/usr/bin/env bash
# Receive the Jetson hardware-encoded H264/RTP stream on the laptop.
#
# Usually launched automatically by the run scripts (start_stream in
# scripts/lib/common.sh). To run it manually as a standalone receiver:
#   scripts/view_h264_stream.sh [port]
# paired with e.g.:
#   STREAM=h264 scripts/run_line_follower_jetson.sh
#
# Native Ubuntu can keep the default sink:
#   scripts/view_h264_stream.sh
#
# WSL/X11 users should select the X image sink explicitly:
#   VIDEO_SINK=ximagesink scripts/view_h264_stream.sh
#
# The GStreamer path is intentionally:
#   udpsrc -> rtpjitterbuffer -> rtph264depay -> avdec_h264 -> videoconvert -> ${VIDEO_SINK} sync=false
#
# Find this laptop's IP on the robot network with:  ip -4 addr show | grep 10.10
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/local.env" ]; then
  set -a
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/local.env"
  set +a
fi

PORT="${1:-${H264_PORT:-5000}}"
VIDEO_SINK="${VIDEO_SINK:-autovideosink}"
CAPS="application/x-rtp,media=video,encoding-name=H264,payload=96"

echo "Listening for H264/RTP on udp port ${PORT} (Ctrl+C to stop)..."
echo "Video sink: ${VIDEO_SINK} (override with VIDEO_SINK=ximagesink if needed)"

if command -v gst-launch-1.0 >/dev/null 2>&1; then
  exec gst-launch-1.0 -v \
    udpsrc port="${PORT}" caps="${CAPS}" ! \
    rtpjitterbuffer latency=50 ! rtph264depay ! avdec_h264 ! videoconvert ! \
    "${VIDEO_SINK}" sync=false
elif command -v ffplay >/dev/null 2>&1; then
  echo "gst-launch-1.0 not found, using ffplay (higher latency)."
  printf 'c=IN IP4 0.0.0.0\nm=video %s RTP/AVP 96\na=rtpmap:96 H264/90000\n' "${PORT}" > /tmp/pb_h264.sdp
  exec ffplay -protocol_whitelist file,udp,rtp -fflags nobuffer -flags low_delay -i /tmp/pb_h264.sdp
else
  echo "No gst-launch-1.0 or ffplay available to receive the stream." >&2
  exit 1
fi
