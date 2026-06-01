#!/usr/bin/env bash
# Receive the Jetson hardware-encoded H264/RTP stream on the laptop.
#
# Start the follower in H264 mode first (sends UDP to this laptop):
#   STREAM_MODE=h264 H264_HOST=<this-laptop-ip> scripts/run_line_follower_jetson.sh
# then run this receiver:
#   scripts/view_h264_stream.sh [port]
#
# Find this laptop's IP on the robot network with:  ip -4 addr show | grep 10.10
set -euo pipefail

PORT="${1:-5000}"
CAPS="application/x-rtp,media=video,encoding-name=H264,payload=96"

echo "Listening for H264/RTP on udp port ${PORT} (Ctrl+C to stop)..."

if command -v gst-launch-1.0 >/dev/null 2>&1; then
  exec gst-launch-1.0 -v \
    udpsrc port="${PORT}" caps="${CAPS}" ! \
    rtpjitterbuffer latency=50 ! rtph264depay ! avdec_h264 ! videoconvert ! \
    autovideosink sync=false
elif command -v ffplay >/dev/null 2>&1; then
  echo "gst-launch-1.0 not found, using ffplay (higher latency)."
  printf 'c=IN IP4 0.0.0.0\nm=video %s RTP/AVP 96\na=rtpmap:96 H264/90000\n' "${PORT}" > /tmp/pb_h264.sdp
  exec ffplay -protocol_whitelist file,udp,rtp -fflags nobuffer -flags low_delay -i /tmp/pb_h264.sdp
else
  echo "No gst-launch-1.0 or ffplay available to receive the stream." >&2
  exit 1
fi
