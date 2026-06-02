#!/usr/bin/env bash
# Install everything the LAPTOP needs to drive the robot and view its streams.
# This is independent of the network setup (scripts/setup_robonet.sh) — run
# either one on its own. No ROS2 is required on the laptop: the run-scripts SSH
# into the Jetson, which has its own ROS stack.
#
# Installs:
#   - System packages: GStreamer (H264 receiver), SSH/rsync, NetworkManager.
#   - Python packages from requirements-laptop.txt (pygame teleop, OpenCV, numpy).
#
#   scripts/setup_laptop.sh
#   NO_APT=1 scripts/setup_laptop.sh     # skip apt, only pip (no sudo)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

APT_PACKAGES=(
  # H264/RTP receiver pipeline for the camera streams.
  gstreamer1.0-tools
  gstreamer1.0-plugins-base
  gstreamer1.0-plugins-good
  gstreamer1.0-plugins-bad
  gstreamer1.0-plugins-ugly
  gstreamer1.0-libav
  # Remote workflow.
  openssh-client
  rsync
  # Hosts the RoboNet AP (used by scripts/setup_robonet.sh).
  network-manager
)

if [ "${NO_APT:-0}" != "1" ]; then
  echo "==> Installing system packages (apt)..."
  sudo apt-get update
  sudo apt-get install -y "${APT_PACKAGES[@]}"
else
  echo "==> NO_APT=1: skipping apt packages."
fi

echo "==> Installing Python packages (pip)..."
python3 -m pip install --user -r "${REPO_DIR}/requirements-laptop.txt"

# Seed the per-laptop overrides file if missing (video sink, optional IP/stream).
if [ ! -f "${SCRIPT_DIR}/local.env" ]; then
  cp "${SCRIPT_DIR}/local.env.example" "${SCRIPT_DIR}/local.env"
  echo "==> Created scripts/local.env (edit it if you are on WSL/X11)."
fi

echo ""
echo "Laptop dependencies installed. Next:"
echo "  - Network (optional, if you host the robot WiFi):  scripts/setup_robonet.sh"
echo "  - Passwordless SSH to the robot:                   ssh-copy-id puzzlebot@10.10.0.100"
echo "  - Verify everything:                               scripts/check_setup.sh"
