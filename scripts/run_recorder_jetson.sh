#!/usr/bin/env bash
# Record clean training frames on the Jetson. Opens the camera directly.
# Press Enter in this terminal to start/pause recording.
#
#   scripts/run_recorder_jetson.sh                      # H264 preview (default)
#   STREAM=none scripts/run_recorder_jetson.sh          # headless capture
#   INTERVAL=0.3 OUTPUT_DIR=dataset_signs scripts/run_recorder_jetson.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

INTERVAL="${INTERVAL:-0.5}"
OUTPUT_DIR="${OUTPUT_DIR:-dataset}"

sync_repo
free_camera
start_stream
run_remote_tool "python3 tools/recorder.py --interval ${INTERVAL} --output-dir ${OUTPUT_DIR}"
