#!/usr/bin/env bash
# One command: start the follower in H264 mode on the Jetson AND open the
# local receiver window on this laptop. Ctrl+C stops both.
#
# The laptop IP is auto-detected from the route to the Jetson (on RoboNet it is
# always 10.10.0.1), so you normally just run:
#   scripts/run_line_follower_h264.sh
set -euo pipefail

JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
H264_PORT="${H264_PORT:-5000}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Start the receiver first so it is ready when frames begin to arrive.
"${SCRIPT_DIR}/view_h264_stream.sh" "${H264_PORT}" &
RX_PID=$!
trap 'kill "${RX_PID}" 2>/dev/null || true' EXIT

# Run the follower in H264 mode (blocks until Ctrl+C). H264_HOST auto-detected.
STREAM_MODE=h264 H264_PORT="${H264_PORT}" "${SCRIPT_DIR}/run_line_follower_jetson.sh"
