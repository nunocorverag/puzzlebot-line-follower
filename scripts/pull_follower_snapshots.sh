#!/usr/bin/env bash
# Compatibility wrapper. New name: scripts/pull_follower_session.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/pull_follower_session.sh" "$@"
