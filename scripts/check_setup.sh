#!/usr/bin/env bash
# Preflight checklist: reports what is ready and what is missing, in two
# independent blocks (LIBRARIES and NETWORK) so you can set up each on its own.
# Read-only: it changes nothing, just prints PASS/FAIL with the fix to run.
#
#   scripts/check_setup.sh
set -uo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fail=0

pass() { printf '  \033[32mOK\033[0m   %s\n' "$1"; }
warn() { printf '  \033[33mMISS\033[0m %s\n     -> %s\n' "$1" "$2"; fail=1; }

echo "== LIBRARIES (scripts/setup_laptop.sh) =="
for bin in ssh rsync gst-launch-1.0; do
  if command -v "$bin" >/dev/null 2>&1; then pass "$bin found"
  else warn "$bin missing" "scripts/setup_laptop.sh"; fi
done
for mod in cv2 numpy pygame; do
  if python3 -c "import $mod" >/dev/null 2>&1; then pass "python3 -c 'import $mod'"
  else warn "python module '$mod' missing" "scripts/setup_laptop.sh"; fi
done
if [ -f "${SCRIPT_DIR}/local.env" ]; then pass "scripts/local.env present"
else warn "scripts/local.env missing" "cp scripts/local.env.example scripts/local.env"; fi

echo ""
echo "== NETWORK (scripts/setup_robonet.sh + SSH) =="
if nmcli -t -f NAME connection show 2>/dev/null | grep -qx "RoboNet"; then
  pass "RoboNet connection defined"
  if nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx "RoboNet"; then
    pass "RoboNet is active (AP up)"
  else
    warn "RoboNet defined but not active" "nmcli connection up RoboNet"
  fi
else
  warn "RoboNet connection not defined" "scripts/setup_robonet.sh (only if you host the WiFi)"
fi
if ping -c1 -W2 "${JETSON_HOST}" >/dev/null 2>&1; then
  pass "Jetson reachable (ping ${JETSON_HOST})"
  if ssh -o BatchMode=yes -o ConnectTimeout=4 "${JETSON_USER}@${JETSON_HOST}" true >/dev/null 2>&1; then
    pass "Passwordless SSH to ${JETSON_USER}@${JETSON_HOST}"
  else
    warn "SSH needs a password / key" "ssh-copy-id ${JETSON_USER}@${JETSON_HOST}"
  fi
else
  warn "Jetson not reachable at ${JETSON_HOST}" "check RoboNet / robot powered on"
fi

echo ""
if [ "$fail" -eq 0 ]; then
  echo "All checks passed — you are ready to run the robot."
else
  echo "Some checks failed (see -> hints above). Each block is independent."
fi
exit "$fail"
