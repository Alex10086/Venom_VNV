#!/usr/bin/env bash
set -euo pipefail

CAN_PORT="${1:-${CAN_PORT:-can0}}"
BITRATE="${BITRATE:-1000000}"
CHECK_SECONDS="${CHECK_SECONDS:-4.0}"

check_feedback() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  python3 "${script_dir}/piper_feedback_preflight.py" \
    --can-port "$CAN_PORT" \
    --duration "$CHECK_SECONDS"
}

reset_can() {
  echo "Attempting software reset for ${CAN_PORT} at ${BITRATE} bps..."
  if ! sudo -n true 2>/dev/null; then
    echo "sudo without password is not available; cannot reset ${CAN_PORT} automatically." >&2
    echo "For competition, allow only these commands via sudoers or run this preflight before the run." >&2
    return 1
  fi

  sudo -n ip link set "$CAN_PORT" down || true
  sleep 0.5
  sudo -n ip link set "$CAN_PORT" type can bitrate "$BITRATE"
  sudo -n ip link set "$CAN_PORT" up
  sleep 1.0
}

echo "Checking Piper feedback on ${CAN_PORT}..."
if check_feedback; then
  exit 0
fi

reset_can || exit 2

echo "Rechecking Piper feedback on ${CAN_PORT}..."
check_feedback
