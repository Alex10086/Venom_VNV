#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$(cd "$ROOT/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
mode=${1:-bievr}
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in bievr|fastlio) ;; *) echo "Usage: $0 [bievr|fastlio] [launch args...]"; exit 2;; esac
if pgrep -u "$(id -u)" -f '/(fastlio_mapping|pointlio_mapping|process_topics|localPlanner|pathFollower|controller_server|async_slam_toolbox_node|scout_base_node)( |$)' >/dev/null; then
  echo 'Existing LIO/navigation/base process found. Stop the existing stack before TARE.' >&2
  exit 1
fi
# Match the last launch override; preview mode does not require a CAN device.
start_base=true
for arg in "$@"; do
  case "$arg" in start_base:=true) start_base=true;; start_base:=false) start_base=false;; esac
done
if [[ "$start_base" == true ]] && ! ip -details link show can0 2>/dev/null | grep -q 'state ERROR-ACTIVE'; then
  echo 'CAN0 is not ready. Before real chassis startup run: sudo ip link set can0 up type can bitrate 500000' >&2
  exit 1
fi
if [[ -z "${DISPLAY:-}" && -r "/run/user/$(id -u)/gdm/Xauthority" ]]; then
  export DISPLAY=:0
  export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
fi
printf 'TARE starts PAUSED. Enable with: ros2 service call /tare/set_enabled std_srvs/srv/SetBool "{data: true}"\n'
exec ros2 launch venom_bringup scout_mini_tare.launch.py "lio:=$mode" \
  start_driver:=true start_base:=true rviz:=true "$@"
