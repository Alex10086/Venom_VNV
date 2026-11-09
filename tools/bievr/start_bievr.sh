#!/usr/bin/env bash
set -eo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
if pgrep -u "$(id -u)" -f '/(fastlio_mapping|pointlio_mapping|process_topics)( |$)' >/dev/null; then
  echo 'Another LIO process is running. Stop it before starting BIEVR (shared body/odom TF).'
  exit 1
fi
if [ -z "${DISPLAY:-}" ] && [ -r "/run/user/$(id -u)/gdm/Xauthority" ]; then
  export DISPLAY=:0
  export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
fi
mode=${1:-lio}
if [ "$#" -gt 0 ]; then shift; fi
case "$mode" in
  lio) exec ros2 launch venom_bringup scout_mini_bievr.launch.py start_driver:=true "$@" ;;
  mapping) exec ros2 launch venom_bringup scout_mini_bievr_mapping.launch.py "$@" ;;
  localization) exec ros2 launch venom_bringup scout_mini_bievr_mapping.launch.py slam_mode:=localization "$@" ;;
  *) printf 'Usage: %s [lio|mapping|localization] [ROS launch arguments...]\n' "$0"; exit 2 ;;
esac
