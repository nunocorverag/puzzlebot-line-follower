#!/usr/bin/env bash
# Pull the line follower's periodic snapshots (saved to
# debug_dataset/follower_session/ on the Jetson when recording is on -- tuner 'r')
# to the laptop for offline review / Claude analysis.
#
#   scripts/pull_follower_snapshots.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

REMOTE="${REMOTE_PKG}/debug_dataset/follower_session/"
LOCAL="${LOCAL:-datasets/follower_session/}"

mkdir -p "${LOCAL}"
rsync -az "${JETSON_USER}@${JETSON_HOST}:${REMOTE}" "${LOCAL}" 2>/dev/null \
  && echo "Pulled follower snapshots -> ${LOCAL}" \
  || echo "Nothing to pull yet (record with the tuner 'r' key first)."
