#!/usr/bin/env bash
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
REMOTE_SRC="${REMOTE_WS}/src"
REMOTE_PACKAGE="${REMOTE_SRC}/puzzlebot_ros"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ssh "${JETSON_USER}@${JETSON_HOST}" "mkdir -p '${REMOTE_PACKAGE}'"

rsync -az --delete \
  --exclude '.git/' \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude '.pytest_cache/' \
  --exclude 'puzzlebot_ros/line_debug/' \
  --exclude 'puzzlebot_ros/*_plot.png' \
  --exclude 'puzzlebot_ros/controller_data.csv' \
  --exclude 'debug_dataset/' \
  "${REPO_DIR}/" "${JETSON_USER}@${JETSON_HOST}:${REMOTE_PACKAGE}/"

ssh "${JETSON_USER}@${JETSON_HOST}" "chmod +x '${REMOTE_PACKAGE}/scripts/'*.sh '${REMOTE_PACKAGE}/env_jetson.sh' '${REMOTE_PACKAGE}/env_laptop.sh'"

echo "Synced ${REPO_DIR} -> ${JETSON_USER}@${JETSON_HOST}:${REMOTE_PACKAGE}"
