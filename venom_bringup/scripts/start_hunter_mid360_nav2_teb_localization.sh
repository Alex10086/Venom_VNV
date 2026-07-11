#!/usr/bin/env bash
set -euo pipefail

WS="${VENOM_WS:-$HOME/venom_ws}"
CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"

POINT_LIO_RVIZ="${POINT_LIO_RVIZ:-true}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/src/venom_vnv/venom_bringup/rviz_cfg/scout_mini_mapping.rviz}"

LIVOX_FRAME_ID="${LIVOX_FRAME_ID:-mid360_link}"
LIVOX_CONFIG="${LIVOX_CONFIG:-$WS/src/venom_vnv/venom_bringup/config/hunter_se/MID360_config.json}"
POINT_LIO_CFG="${POINT_LIO_CFG:-$WS/src/venom_vnv/venom_bringup/config/hunter_se/point_lio_mid360_tilted.yaml}"
NAV2_PARAMS="${NAV2_PARAMS:-$WS/src/venom_vnv/simulation/venom_nav_simulation/src/rm_nav_bringup/config/reality/nav2_params_real.yaml}"

DEFAULT_MAP="$WS/src/venom_vnv/venom_bringup/map/competition_10x6.yaml"
MAP="${MAP:-${1:-$DEFAULT_MAP}}"

# The MID360 is mounted front/back reversed, then tilted upward.
# Point-LIO publishes odom -> mid360_link. This script publishes the inverse
# mount transform mid360_link -> base_link for Nav2 and pointcloud_to_laserscan.
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

AUTO_INITIAL_POSE="${AUTO_INITIAL_POSE:-true}"
INITIAL_POSE_X="${INITIAL_POSE_X:-0.0}"
INITIAL_POSE_Y="${INITIAL_POSE_Y:-0.0}"
INITIAL_POSE_YAW="${INITIAL_POSE_YAW:-0.0}"

ENABLE_ROBOT_PATH="${ENABLE_ROBOT_PATH:-true}"
ROBOT_PATH_TOPIC="${ROBOT_PATH_TOPIC:-/robot_path}"
ROBOT_PATH_MIN_DISTANCE="${ROBOT_PATH_MIN_DISTANCE:-0.03}"
ROBOT_PATH_MAX_POSES="${ROBOT_PATH_MAX_POSES:-5000}"

# Optional mechanical mount TF. Disabled by default to keep this script focused
# on Hunter navigation; enable it when the arm bringup expects this mount frame.
ENABLE_PIPER_MOUNT_TF="${ENABLE_PIPER_MOUNT_TF:-false}"
PIPER_MOUNT_FRAME="${PIPER_MOUNT_FRAME:-hunter_piper_mount_link}"
PIPER_MOUNT_X="${PIPER_MOUNT_X:-0.10}"
PIPER_MOUNT_Y="${PIPER_MOUNT_Y:-0.0}"
PIPER_MOUNT_Z="${PIPER_MOUNT_Z:-0.0}"
PIPER_MOUNT_ROLL="${PIPER_MOUNT_ROLL:-0.0}"
PIPER_MOUNT_PITCH="${PIPER_MOUNT_PITCH:-0.0}"
PIPER_MOUNT_YAW="${PIPER_MOUNT_YAW:-0.0}"

PIDS=()

start_process() {
    setsid "$@" &
    PIDS+=("$!")
}

cleanup() {
    trap - INT TERM EXIT
    local had_live_group=false
    echo
    echo "Stopping Hunter MID360 Nav2 TEB localization stack..."
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

wait_for_lifecycle_active() {
    local node_name="$1"
    local timeout_sec="$2"
    local elapsed=0

    echo "Waiting for $node_name to become active..."
    while [ "$elapsed" -lt "$timeout_sec" ]; do
        if ros2 lifecycle get "$node_name" 2>/dev/null | grep -q "^active"; then
            echo "$node_name is active."
            return 0
        fi

        sleep 1
        elapsed=$((elapsed + 1))
    done

    echo "Timed out waiting for $node_name to become active." >&2
    return 1
}

publish_initial_pose() {
    echo "Publishing initial pose x=$INITIAL_POSE_X y=$INITIAL_POSE_Y yaw=$INITIAL_POSE_YAW..."
    local initial_pose_qz
    local initial_pose_qw
    initial_pose_qz="$(python3 -c "import math; print(math.sin(float('$INITIAL_POSE_YAW') / 2.0))")"
    initial_pose_qw="$(python3 -c "import math; print(math.cos(float('$INITIAL_POSE_YAW') / 2.0))")"

    timeout 10 ros2 topic pub -r 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: map}, pose: {pose: {position: {x: $INITIAL_POSE_X, y: $INITIAL_POSE_Y, z: 0.0}, orientation: {z: $initial_pose_qz, w: $initial_pose_qw}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}" || true
}

trap handle_signal INT TERM
trap cleanup EXIT

require_file "$WS/install/setup.bash" "Workspace setup"
if [ "$POINT_LIO_RVIZ" = "true" ]; then
    require_file "$RVIZ_CONFIG" "RViz config"
fi
require_file "$LIVOX_CONFIG" "Livox MID360 config"
require_file "$POINT_LIO_CFG" "Point-LIO config"
require_file "$NAV2_PARAMS" "Nav2 params"
require_file "$MAP" "Static map yaml; save one with: ros2 run nav2_map_server map_saver_cli -f ${DEFAULT_MAP%.yaml}"

setup_can

set +u
source "$WS/install/setup.bash"
set -u

echo "Starting MID360 + Point-LIO with Hunter tilted MID360 config..."
start_process ros2 launch venom_bringup mid360_point_lio.launch.py \
    "rviz:=$POINT_LIO_RVIZ" \
    "rviz_config:=$RVIZ_CONFIG" \
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

sleep 8

echo "Starting pointcloud_to_laserscan..."
start_process ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
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
    -p output_reliable:=true

if [ "$ENABLE_PIPER_MOUNT_TF" = "true" ]; then
    echo "Publishing base_link-to-$PIPER_MOUNT_FRAME static TF..."
    start_process ros2 run tf2_ros static_transform_publisher \
        --x "$PIPER_MOUNT_X" \
        --y "$PIPER_MOUNT_Y" \
        --z "$PIPER_MOUNT_Z" \
        --roll "$PIPER_MOUNT_ROLL" \
        --pitch "$PIPER_MOUNT_PITCH" \
        --yaw "$PIPER_MOUNT_YAW" \
        --frame-id base_link \
        --child-frame-id "$PIPER_MOUNT_FRAME"
fi

if [ "$ENABLE_ROBOT_PATH" = "true" ]; then
    echo "Starting robot path publisher on $ROBOT_PATH_TOPIC..."
    start_process ros2 run venom_bringup odom_path_publisher \
        --ros-args \
        -p odom_topic:=/odom \
        -p "path_topic:=$ROBOT_PATH_TOPIC" \
        -p "min_distance:=$ROBOT_PATH_MIN_DISTANCE" \
        -p "max_poses:=$ROBOT_PATH_MAX_POSES"
fi

sleep 2

echo "Starting Hunter base with isolated wheel odometry..."
start_process ros2 launch hunter_base hunter_base.launch.py \
    "port_name:=$CAN_IFACE" \
    odom_frame:=hunter_odom \
    base_frame:=hunter_base_link \
    odom_topic_name:=hunter_odom

sleep 2

echo "Starting Nav2 localization + TEB with $NAV2_PARAMS..."
start_process ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=False \
    autostart:=True \
    "map:=$MAP" \
    "params_file:=$NAV2_PARAMS"

if [ "$AUTO_INITIAL_POSE" = "true" ]; then
    if wait_for_lifecycle_active /amcl 60; then
        publish_initial_pose
    fi
fi

echo
echo "Hunter MID360 Nav2 TEB localization stack is running."
echo "Using static map: $MAP"
echo "Using RViz config: $RVIZ_CONFIG"
echo "Using Nav2 params: $NAV2_PARAMS"
echo "Initial pose auto publish: $AUTO_INITIAL_POSE (x=$INITIAL_POSE_X y=$INITIAL_POSE_Y yaw=$INITIAL_POSE_YAW)"
echo "Robot path publish: $ENABLE_ROBOT_PATH (topic=$ROBOT_PATH_TOPIC)"
echo "Piper mount TF: $ENABLE_PIPER_MOUNT_TF (frame=$PIPER_MOUNT_FRAME)"
echo
echo "Check status with:"
echo "  ros2 lifecycle get /amcl"
echo "  ros2 topic echo /amcl_pose --once"
echo "  ros2 lifecycle get /bt_navigator"
echo "  ros2 topic info /scan -v"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo "  ros2 topic info /cmd_vel -v"
echo
echo "Set POINT_LIO_RVIZ=false to run without RViz."
echo "Set AUTO_INITIAL_POSE=false to require manual RViz 2D Pose Estimate."
echo "Press Ctrl+C to stop all started processes."

set +e
wait -n
child_status=$?
set -e
if [ "$child_status" -eq 0 ]; then
    child_status=1
fi
echo "A required process exited; stopping the localization stack." >&2
exit "$child_status"
