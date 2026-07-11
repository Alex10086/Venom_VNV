#!/usr/bin/env bash
set -euo pipefail

WS="${VENOM_WS:-$HOME/venom_ws}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/$ROS_DISTRO/setup.bash}"
POINT_LIO_RVIZ="${POINT_LIO_RVIZ:-false}"
LIVOX_FRAME_ID="${LIVOX_FRAME_ID:-mid360_link}"
LIVOX_CONFIG="${LIVOX_CONFIG:-$WS/src/venom_vnv/venom_bringup/config/hunter_se/MID360_config.json}"
POINT_LIO_CFG="${POINT_LIO_CFG:-$WS/src/venom_vnv/venom_bringup/config/hunter_se/point_lio_mid360_light.yaml}"

# MID360 is mounted rear/front reversed and tilted. Point-LIO estimates the real
# tilted sensor frame; this static TF exposes a chassis-level base_link for checks.
MID360_TO_BASE_X="${MID360_TO_BASE_X:-0.0}"
MID360_TO_BASE_Y="${MID360_TO_BASE_Y:-0.0}"
MID360_TO_BASE_Z="${MID360_TO_BASE_Z:-0.0}"
MID360_TO_BASE_ROLL="${MID360_TO_BASE_ROLL:-0.0}"
MID360_TO_BASE_PITCH="${MID360_TO_BASE_PITCH:-0.5489}"
MID360_TO_BASE_YAW="${MID360_TO_BASE_YAW:-3.141592653589793}"

PIDS=()

start_process() {
    setsid "$@" &
    PIDS+=("$!")
}

cleanup() {
    trap - INT TERM EXIT
    local had_live_group=false
    echo
    echo "Stopping Hunter MID360 light LIO stack..."
    for pid in "${PIDS[@]}"; do
        if kill -0 -- "-$pid" 2>/dev/null; then
            kill -- "-$pid" 2>/dev/null || true
            had_live_group=true
        fi
    done
    if [ "$had_live_group" = true ]; then
        sleep 2
    fi
    for pid in "${PIDS[@]}"; do
        if kill -0 -- "-$pid" 2>/dev/null; then
            kill -KILL -- "-$pid" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
}

handle_signal() {
    cleanup
    exit 130
}

require_file() {
    local file_path="$1"
    local description="$2"

    if [ ! -f "$file_path" ]; then
        echo "$description not found: $file_path" >&2
        exit 1
    fi
}

setup_ros_env() {
    set +u
    source "$ROS_SETUP"
    source "$WS/install/setup.bash"
    set -u

    if ! command -v ros2 >/dev/null 2>&1; then
        echo "ros2 command not found after sourcing ROS and workspace setup files." >&2
        echo "ROS setup: $ROS_SETUP" >&2
        echo "Workspace setup: $WS/install/setup.bash" >&2
        exit 1
    fi
}

trap handle_signal INT TERM
trap cleanup EXIT

require_file "$ROS_SETUP" "ROS setup"
require_file "$WS/install/setup.bash" "Workspace setup"
require_file "$LIVOX_CONFIG" "Livox MID360 config"
require_file "$POINT_LIO_CFG" "Point-LIO light config"

setup_ros_env

echo "Starting MID360 + light Point-LIO only..."
start_process ros2 launch venom_bringup mid360_point_lio.launch.py \
    "rviz:=$POINT_LIO_RVIZ" \
    "livox_user_config:=$LIVOX_CONFIG" \
    "livox_frame_id:=$LIVOX_FRAME_ID" \
    "point_lio_cfg:=$POINT_LIO_CFG"

echo "Publishing MID360-to-base static TF..."
start_process ros2 run tf2_ros static_transform_publisher \
    --x "$MID360_TO_BASE_X" \
    --y "$MID360_TO_BASE_Y" \
    --z "$MID360_TO_BASE_Z" \
    --roll "$MID360_TO_BASE_ROLL" \
    --pitch "$MID360_TO_BASE_PITCH" \
    --yaw "$MID360_TO_BASE_YAW" \
    --frame-id "$LIVOX_FRAME_ID" \
    --child-frame-id base_link

echo
echo "Hunter MID360 light LIO stack is running."
echo "This entry intentionally starts no 2D scan conversion and no downstream mapper."
echo "Recommended checks:"
echo "  ros2 topic hz /livox/lidar"
echo "  ros2 topic hz /livox/imu"
echo "  ros2 topic hz /odom"
echo "  ros2 topic hz /cloud_registered"
echo "  ros2 run tf2_ros tf2_echo odom base_link"
echo "  ros2 topic echo /odom --once"
echo
echo "Expected: /odom stays near input rate; Point-LIO logs should not show sustained loop overruns."
echo "Override POINT_LIO_CFG to compare other configs without changing this script."
echo "Press Ctrl+C to stop all started processes."

set +e
wait -n
child_status=$?
set -e
if [ "$child_status" -eq 0 ]; then
    child_status=1
fi
echo "A required process exited; stopping the light LIO stack." >&2
exit "$child_status"
