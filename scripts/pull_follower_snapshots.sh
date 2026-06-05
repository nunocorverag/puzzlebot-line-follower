#!/usr/bin/env bash
# Pull the line follower's periodic snapshots (saved to
# debug_dataset/follower_session/ on the Jetson when recording is on -- tuner 'r')
# into a per-session folder on the laptop, then wipe them on the Jetson so the
# robot stays clean and each pull is one tidy session.
#
#     datasets/follower_session/<SESSION>/
#
#   scripts/pull_follower_snapshots.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

REMOTE_DIR="${REMOTE_PKG}/debug_dataset/follower_session"
SESSION="$(date +%Y%m%d_%H%M%S)"
LOCAL_DEST="${REPO_DIR}/datasets/follower_session/${SESSION}"

pull_and_clean_session "${REMOTE_DIR}" "${LOCAL_DEST}"
