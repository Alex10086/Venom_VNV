#!/usr/bin/env bash
set -euo pipefail

WS="${VENOM_WS:-$HOME/venom_ws}"
CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
POINT_LIO_RVIZ="${POINT_LIO_RVIZ:-false}"
LIVOX_FRAME_ID="${LIVOX_FRAME_ID:-mid360_link}"
LIVOX_CONFIG="${LIVOX_CONFIG:-$WS/src/venom_vnv/venom_bringup/config/hunter_se/MID360_config.json}"
POINT_LIO_CFG="${POINT_LIO_CFG:-$WS/src/venom_vnv/venom_bringup/config/hunter_se/point_lio_mid360_tilted.yaml}"
NAV2_PARAMS="${NAV2_PARAMS:-$WS/src/venom_vnv/venom_bringup/config/hunter_se/nav2_teb_params.yaml}"
DEFAULT_MAP="$WS/src/venom_vnv/venom_bringup/map/competition_10x6.yaml"
MAP="${MAP:-${1:-$DEFAULT_MAP}}"

# The MID360 is mounted front/back reversed, then tilted upward.
# Point-LIO publishes odom -> mid360_link, so this script publishes the inverse
# mount transform mid360_link -> base_link for Nav2 and pointcloud_to_laserscan.
# The effective ROS-frame pitch is calibrated from the live Point-LIO body pose.
MID360_TO_BASE_X="${MID360_TO_BASE_X:-0.0}"
MID360_TO_BASE_Y="${MID360_TO_BASE_Y:-0.0}"
MID360_TO_BASE_Z="${MID360_TO_BASE_Z:-0.0}"
MID360_TO_BASE_ROLL="${MID360_TO_BASE_ROLL:-0.0}"
MID360_TO_BASE_PITCH="${MID360_TO_BASE_PITCH:-0.5489}"
MID360_TO_BASE_YAW="${MID360_TO_BASE_YAW:-3.141592653589793}"

SCAN_MIN_HEIGHT="${SCAN_MIN_HEIGHT:-0.05}"
SCAN_MAX_HEIGHT="${SCAN_MAX_HEIGHT:-0.7}"
SCAN_RANGE_MIN="${SCAN_RANGE_MIN:-0.3}"
SCAN_RANGE_MAX="${SCAN_RANGE_MAX:-50.0}"

PIDS=()

cleanup() {
    echo
    echo "Stopping Hunter MID360 Nav2 TEB stack..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
}

require_file() {
    local file_path="$1"
    local description="$2"

    if [ ! -f "$file_path" ]; then
        echo "$description not found: $file_path" >&2
        exit 1
    fi
}

setup_can() {
    echo "Requesting sudo for CAN setup..."
    sudo -v

    echo "Loading gs_usb..."
    sudo modprobe gs_usb

    if ! ip link show "$CAN_IFACE" >/dev/null 2>&1; then
        echo "CAN interface not found: $CAN_IFACE" >&2
        echo "Install can-utils/iproute2 in the robot image and check the USB-CAN adapter." >&2
        exit 1
    fi

    echo "Bringing up $CAN_IFACE at $CAN_BITRATE bps..."
    sudo ip link set "$CAN_IFACE" down 2>/dev/null || true
    sudo ip link set "$CAN_IFACE" up type can bitrate "$CAN_BITRATE"
}

require_file "$WS/install/setup.bash" "Workspace setup"
require_file "$LIVOX_CONFIG" "Livox MID360 config"
require_file "$POINT_LIO_CFG" "Point-LIO config"
require_file "$NAV2_PARAMS" "Nav2 params"
require_file "$MAP" "Static map yaml; save one with: ros2 run nav2_map_server map_saver_cli -f ${DEFAULT_MAP%.yaml}"

setup_can

set +u
source "$WS/install/setup.bash"
set -u

echo "Starting MID360 + Point-LIO..."
ros2 launch venom_bringup mid360_point_lio.launch.py \
    "rviz:=$POINT_LIO_RVIZ" \
    "livox_user_config:=$LIVOX_CONFIG" \
    "livox_frame_id:=$LIVOX_FRAME_ID" \
    "point_lio_cfg:=$POINT_LIO_CFG" &
PIDS+=("$!")

echo "Publishing MID360-to-base static TF..."
ros2 run tf2_ros static_transform_publisher \
    --x "$MID360_TO_BASE_X" \
    --y "$MID360_TO_BASE_Y" \
    --z "$MID360_TO_BASE_Z" \
    --roll "$MID360_TO_BASE_ROLL" \
    --pitch "$MID360_TO_BASE_PITCH" \
    --yaw "$MID360_TO_BASE_YAW" \
    --frame-id "$LIVOX_FRAME_ID" \
    --child-frame-id base_link &
PIDS+=("$!")

sleep 8

echo "Starting pointcloud_to_laserscan..."
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args \
    -r cloud_in:=/cloud_registered \
    -r scan:=/scan \
    -p target_frame:=base_link \
    -p transform_tolerance:=0.2 \
    -p "min_height:=$SCAN_MIN_HEIGHT" \
    -p "max_height:=$SCAN_MAX_HEIGHT" \
    -p angle_min:=-3.14159 \
    -p angle_max:=3.14159 \
    -p angle_increment:=0.001 \
    -p scan_time:=0.1 \
    -p "range_min:=$SCAN_RANGE_MIN" \
    -p "range_max:=$SCAN_RANGE_MAX" \
    -p use_inf:=true \
    -p output_reliable:=true &
PIDS+=("$!")

sleep 2

echo "Starting Hunter base with isolated wheel odometry..."
ros2 launch hunter_base hunter_base.launch.py \
    "port_name:=$CAN_IFACE" \
    odom_frame:=hunter_odom \
    base_frame:=hunter_base_link \
    odom_topic_name:=hunter_odom &
PIDS+=("$!")

sleep 2

echo "Starting Nav2 localization + TEB with $NAV2_PARAMS..."
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=False \
    autostart:=True \
    "map:=$MAP" \
    "params_file:=$NAV2_PARAMS" &
PIDS+=("$!")

echo
echo "Hunter MID360 Nav2 TEB stack is running."
echo "Using static map: $MAP"
echo "Set the initial pose in RViz before mission_commander if AMCL has not converged."
echo "Check competition preflight with:"
echo "  ros2 lifecycle get /amcl"
echo "  ros2 lifecycle get /bt_navigator"
echo "  ros2 topic info /scan -v"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo "  ros2 topic info /cmd_vel -v"
echo
echo "Then run mission_commander with use_nav:=true and nav2_wait_mode:=bt_navigator."
echo "Set POINT_LIO_RVIZ=true if you need the LIO RViz view."
echo "Press Ctrl+C to stop all started processes."

wait
