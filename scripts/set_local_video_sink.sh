#!/usr/bin/env bash
# Configure the laptop-side H264 viewer sink for every script that opens video.
#
# Native Ubuntu:
#   scripts/set_local_video_sink.sh autovideosink
#
# WSL/X11:
#   scripts/set_local_video_sink.sh ximagesink
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "Usage: scripts/set_local_video_sink.sh autovideosink|ximagesink|xvimagesink" >&2
  exit 2
fi

SINK="$1"
case "${SINK}" in
  autovideosink|ximagesink|xvimagesink)
    ;;
  *)
    echo "Unsupported sink '${SINK}'. Use autovideosink, ximagesink, or xvimagesink." >&2
    exit 2
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${SCRIPT_DIR}/local.env"

if [ ! -f "${ENV_FILE}" ]; then
  cp "${SCRIPT_DIR}/local.env.example" "${ENV_FILE}"
fi

if grep -q '^VIDEO_SINK=' "${ENV_FILE}"; then
  sed -i "s/^VIDEO_SINK=.*/VIDEO_SINK=${SINK}/" "${ENV_FILE}"
else
  printf '\nVIDEO_SINK=%s\n' "${SINK}" >> "${ENV_FILE}"
fi

echo "Configured VIDEO_SINK=${SINK} in ${ENV_FILE}"
