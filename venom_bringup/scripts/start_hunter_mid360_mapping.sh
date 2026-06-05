#!/usr/bin/env bash
set -euo pipefail

WS="${VENOM_WS:-$HOME/venom_ws}"
CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
POINT_LIO_RVIZ="${POINT_LIO_RVIZ:-true}"
LIVOX_CONFIG="${LIVOX_CONFIG:-$WS/src/venom_vnv/venom_bringup/config/hunter_se/MID360_config.json}"
POINT_LIO_CFG="${POINT_LIO_CFG:-$WS/src/venom_vnv/venom_bringup/config/examples/point_lio_mapping.yaml}"
SLAM_PARAMS="${SLAM_PARAMS:-$WS/src/venom_vnv/venom_bringup/config/hunter_se/slam_toolbox_mapping.yaml}"
MAP_PREFIX="$WS/src/venom_vnv/venom_bringup/map/competition_10x6"

PIDS=()

cleanup() {
    echo
    echo "Stopping Hunter MID360 mapping stack..."
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
require_file "$SLAM_PARAMS" "slam_toolbox params"

setup_can

set +u
source "$WS/install/setup.bash"
set -u

echo "Starting MID360 + Point-LIO..."
ros2 launch venom_bringup mid360_point_lio.launch.py \
    "rviz:=$POINT_LIO_RVIZ" \
    "livox_user_config:=$LIVOX_CONFIG" \
    "point_lio_cfg:=$POINT_LIO_CFG" &
PIDS+=("$!")

sleep 8

echo "Starting pointcloud_to_laserscan..."
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args \
    -r cloud_in:=/cloud_registered \
    -r scan:=/scan \
    -p target_frame:=base_link \
    -p transform_tolerance:=0.2 \
    -p min_height:=0.05 \
    -p max_height:=0.7 \
    -p angle_min:=-3.14159 \
    -p angle_max:=3.14159 \
    -p angle_increment:=0.001 \
    -p scan_time:=0.1 \
    -p range_min:=0.3 \
    -p range_max:=50.0 \
    -p use_inf:=true \
    -p output_reliable:=true &
PIDS+=("$!")

sleep 2

echo "Starting slam_toolbox mapping with $SLAM_PARAMS..."
ros2 launch slam_toolbox online_async_launch.py "slam_params_file:=$SLAM_PARAMS" &
PIDS+=("$!")

echo
echo "Hunter MID360 mapping stack is running."
echo "Check mapping with:"
echo "  ros2 topic hz /cloud_registered"
echo "  ros2 topic info /scan -v"
echo "  ros2 topic hz /map"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo
echo "After mapping, save a static map before running mission_commander:"
echo "  ros2 run nav2_map_server map_saver_cli -f $MAP_PREFIX"
echo
echo "Set POINT_LIO_RVIZ=false to run without RViz."
echo "Press Ctrl+C to stop all started processes."

wait
