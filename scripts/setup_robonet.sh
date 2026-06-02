#!/usr/bin/env bash
# Recreate the "RoboNet" WiFi access point on this laptop, exactly as the robot
# expects it. The laptop hosts the AP (ipv4 internet-sharing), is always
# 10.10.0.1, and the Jetson joins it as a client at 10.10.0.100.
#
# The Jetson is pre-configured to join SSID "RoboNet" with this password, so the
# SSID and password here MUST match for it to connect automatically. Internet
# sharing means a SECOND interface (internal WiFi or ethernet) should provide
# the upstream connection; the AP runs on a separate WiFi adapter.
#
#   scripts/setup_robonet.sh                 # create + bring up the AP
#   WIFI_IFACE=wlan0 scripts/setup_robonet.sh
#   ROBONET_SSID=RoboNet ROBONET_PSK=puzzlebot123 scripts/setup_robonet.sh
set -euo pipefail

ROBONET_SSID="${ROBONET_SSID:-RoboNet}"
ROBONET_PSK="${ROBONET_PSK:-puzzlebot123}"
ROBONET_ADDR="${ROBONET_ADDR:-10.10.0.1/24}"
CON_NAME="${CON_NAME:-RoboNet}"

command -v nmcli >/dev/null 2>&1 || {
  echo "nmcli not found. Install NetworkManager: sudo apt install network-manager" >&2
  exit 1
}

# Pick the WiFi adapter for the AP: prefer a USB dongle (wlx*), else the first
# WiFi device. Override with WIFI_IFACE=<dev>.
if [ -z "${WIFI_IFACE:-}" ]; then
  WIFI_IFACE="$(nmcli -t -f DEVICE,TYPE device 2>/dev/null | awk -F: '$2=="wifi" && $1 ~ /^wlx/ {print $1; exit}')"
  [ -z "${WIFI_IFACE}" ] && WIFI_IFACE="$(nmcli -t -f DEVICE,TYPE device 2>/dev/null | awk -F: '$2=="wifi" {print $1; exit}')"
fi
if [ -z "${WIFI_IFACE}" ]; then
  echo "No WiFi device found. Plug in a WiFi adapter or set WIFI_IFACE=<dev>." >&2
  exit 1
fi

echo "Creating AP '${ROBONET_SSID}' on ${WIFI_IFACE} (laptop = ${ROBONET_ADDR})..."

# Idempotent: drop any previous definition so re-running gives a clean state.
nmcli connection delete "${CON_NAME}" >/dev/null 2>&1 || true

nmcli connection add type wifi ifname "${WIFI_IFACE}" con-name "${CON_NAME}" \
  autoconnect no ssid "${ROBONET_SSID}" \
  802-11-wireless.mode ap 802-11-wireless.band bg \
  ipv4.method shared ipv4.addresses "${ROBONET_ADDR}" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "${ROBONET_PSK}"

nmcli connection up "${CON_NAME}"

echo ""
echo "RoboNet is up. Laptop is ${ROBONET_ADDR%/*}; the Jetson should appear at 10.10.0.100."
echo "Verify with:  scripts/check_setup.sh"
