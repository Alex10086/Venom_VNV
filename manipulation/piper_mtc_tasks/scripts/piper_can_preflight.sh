#!/usr/bin/env bash
set -euo pipefail

CAN_PORT="${1:-${CAN_PORT:-can0}}"
BITRATE="${BITRATE:-1000000}"
RESTART_MS="${CAN_RESTART_MS:-100}"
CHECK_SECONDS="${CHECK_SECONDS:-4.0}"
RESTART_MS_REQUIRED=1

check_feedback() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  python3 "${script_dir}/piper_feedback_preflight.py" \
    --can-port "$CAN_PORT" \
    --duration "$CHECK_SECONDS"
}

can_configured() {
  local details
  details="$(ip -details link show "$CAN_PORT" 2>/dev/null || true)"
  if [[ "$details" != *"bitrate ${BITRATE}"* ]]; then
    return 1
  fi
  if [[ "$RESTART_MS_REQUIRED" -eq 0 ]]; then
    return 0
  fi
  [[ "$details" == *"restart-ms ${RESTART_MS}"* ]]
}

configure_can() {
  echo "Ensuring ${CAN_PORT} is up at ${BITRATE} bps with restart-ms ${RESTART_MS}..."
  if ! sudo -n true 2>/dev/null; then
    echo "sudo without password is not available; cannot configure ${CAN_PORT} automatically." >&2
    echo "For competition, allow only these commands via sudoers or run this preflight before the run." >&2
    return 1
  fi

  sudo -n ip link set "$CAN_PORT" down || true
  sleep 0.5
  if ! sudo -n ip link set "$CAN_PORT" type can bitrate "$BITRATE" restart-ms "$RESTART_MS"; then
    echo "${CAN_PORT} does not accept restart-ms ${RESTART_MS}; retrying with bitrate only." >&2
    RESTART_MS_REQUIRED=0
    sudo -n ip link set "$CAN_PORT" type can bitrate "$BITRATE"
  fi
  sudo -n ip link set "$CAN_PORT" up
  sleep 1.0
}

reset_can() {
  configure_can
}

if ! can_configured; then
  configure_can || exit 2
fi

if ! can_configured; then
  if [[ "$RESTART_MS_REQUIRED" -eq 0 ]]; then
    echo "${CAN_PORT} is still missing bitrate ${BITRATE} after configuration." >&2
  else
    echo "${CAN_PORT} is still missing bitrate ${BITRATE} or restart-ms ${RESTART_MS} after configuration." >&2
  fi
  exit 2
fi

echo "Checking Piper feedback on ${CAN_PORT}..."
if check_feedback; then
  exit 0
fi

reset_can || exit 2

echo "Rechecking Piper feedback on ${CAN_PORT}..."
check_feedback
