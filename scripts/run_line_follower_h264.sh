#!/usr/bin/env bash
# Convenience shortcut: line follower with H264 streaming to the laptop.
# Equivalent to `STREAM=h264 scripts/run_line_follower_jetson.sh` (the receiver
# is launched automatically). Kept for backwards compatibility.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STREAM=h264 exec "${SCRIPT_DIR}/run_line_follower_jetson.sh" "$@"
