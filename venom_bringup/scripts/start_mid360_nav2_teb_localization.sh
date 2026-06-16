#!/usr/bin/env bash
set -euo pipefail

WS="${VENOM_WS:-$HOME/venom_ws}"
CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
POINT_LIO_RVIZ="${POINT_LIO_RVIZ:-true}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/src/venom_vnv/venom_bringup/rviz_cfg/scout_mini_mapping.rviz}"
LIVOX_CONFIG="${LIVOX_CONFIG:-$WS/src/venom_vnv/venom_bringup/config/hunter_se/MID360_config.json}"
POINT_LIO_CFG="${POINT_LIO_CFG:-$WS/src/venom_vnv/venom_bringup/config/scout_mini/point_lio_mapping.yaml}"
NAV2_PARAMS="${NAV2_PARAMS:-$WS/src/venom_vnv/venom_bringup/config/scout_mini/nav2_teb_localization_params.yaml}"
AUTO_INITIAL_POSE="${AUTO_INITIAL_POSE:-true}"
INITIAL_POSE_X="${INITIAL_POSE_X:-0.0}"
INITIAL_POSE_Y="${INITIAL_POSE_Y:-0.0}"
INITIAL_POSE_YAW="${INITIAL_POSE_YAW:-0.0}"
STATIC_MAP_ODOM_TF="${STATIC_MAP_ODOM_TF:-true}"
MAP_ODOM_X="${MAP_ODOM_X:-$INITIAL_POSE_X}"
MAP_ODOM_Y="${MAP_ODOM_Y:-$INITIAL_POSE_Y}"
MAP_ODOM_YAW="${MAP_ODOM_YAW:-$INITIAL_POSE_YAW}"
ENABLE_ROBOT_PATH="${ENABLE_ROBOT_PATH:-true}"
ROBOT_PATH_TOPIC="${ROBOT_PATH_TOPIC:-/robot_path}"
ROBOT_PATH_MIN_DISTANCE="${ROBOT_PATH_MIN_DISTANCE:-0.03}"
ROBOT_PATH_MAX_POSES="${ROBOT_PATH_MAX_POSES:-5000}"
DEFAULT_MAP="$WS/src/venom_vnv/venom_bringup/map/competition_10x6.yaml"
MAP="${MAP:-${1:-$DEFAULT_MAP}}"

PIDS=()

cleanup() {
    echo
    echo "Stopping MID360 Nav2 TEB localization stack..."
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

trap cleanup INT TERM EXIT

require_file "$WS/install/setup.bash" "Workspace setup"
if [ "$POINT_LIO_RVIZ" = "true" ]; then
    require_file "$RVIZ_CONFIG" "RViz config"
fi
require_file "$LIVOX_CONFIG" "Livox MID360 config"
require_file "$POINT_LIO_CFG" "Point-LIO config"
require_file "$NAV2_PARAMS" "Nav2 params"
require_file "$MAP" "Static map yaml; save one with: ros2 run nav2_map_server map_saver_cli -f ${DEFAULT_MAP%.yaml}"

echo "Requesting sudo for CAN setup..."
sudo -v

if ! command -v candump >/dev/null 2>&1; then
    echo "Installing can-utils..."
    sudo apt install -y can-utils
fi

echo "Loading gs_usb..."
sudo modprobe gs_usb

if ! ip link show "$CAN_IFACE" >/dev/null 2>&1; then
    echo "CAN interface not found: $CAN_IFACE" >&2
    exit 1
fi

echo "Bringing up $CAN_IFACE at $CAN_BITRATE bps..."
sudo ip link set "$CAN_IFACE" down 2>/dev/null || true
sudo ip link set "$CAN_IFACE" up type can bitrate "$CAN_BITRATE"

set +u
source "$WS/install/setup.bash"
set -u

echo "Starting MID360 + Point-LIO..."
ros2 launch venom_bringup mid360_point_lio.launch.py \
    "rviz:=$POINT_LIO_RVIZ" \
    "rviz_config:=$RVIZ_CONFIG" \
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

echo "Starting robot static TF publishers..."
ros2 run tf2_ros static_transform_publisher \
    --x 0.10 --y 0.0 --z 0.0 \
    --roll 0.0 --pitch 0.0 --yaw 0.0 \
    --frame-id base_link \
    --child-frame-id scoutmini_piper_mount_link &
PIDS+=("$!")

if [ "$STATIC_MAP_ODOM_TF" = "true" ]; then
    echo "Publishing fixed map->odom TF x=$MAP_ODOM_X y=$MAP_ODOM_Y yaw=$MAP_ODOM_YAW..."
    ros2 run tf2_ros static_transform_publisher \
        --x "$MAP_ODOM_X" --y "$MAP_ODOM_Y" --z 0.0 \
        --roll 0.0 --pitch 0.0 --yaw "$MAP_ODOM_YAW" \
        --frame-id map \
        --child-frame-id odom &
    PIDS+=("$!")
fi

if [ "$ENABLE_ROBOT_PATH" = "true" ]; then
    echo "Starting robot path publisher on $ROBOT_PATH_TOPIC..."
    ros2 run venom_bringup odom_path_publisher \
        --ros-args \
        -p odom_topic:=/odom \
        -p "path_topic:=$ROBOT_PATH_TOPIC" \
        -p "min_distance:=$ROBOT_PATH_MIN_DISTANCE" \
        -p "max_poses:=$ROBOT_PATH_MAX_POSES" &
    PIDS+=("$!")
fi

sleep 1

echo "Starting Hunter base..."
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

if [ "$AUTO_INITIAL_POSE" = "true" ]; then
    if wait_for_lifecycle_active /amcl 60; then
        echo "Publishing initial pose x=$INITIAL_POSE_X y=$INITIAL_POSE_Y yaw=$INITIAL_POSE_YAW..."
        INITIAL_POSE_QZ="$(python3 -c "import math; print(math.sin(float('$INITIAL_POSE_YAW') / 2.0))")"
        INITIAL_POSE_QW="$(python3 -c "import math; print(math.cos(float('$INITIAL_POSE_YAW') / 2.0))")"
        timeout 10 ros2 topic pub -r 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
            "{header: {frame_id: map}, pose: {pose: {position: {x: $INITIAL_POSE_X, y: $INITIAL_POSE_Y, z: 0.0}, orientation: {z: $INITIAL_POSE_QZ, w: $INITIAL_POSE_QW}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}" || true
    fi
fi

echo
echo "MID360 Nav2 TEB localization stack is running."
echo "Using static map: $MAP"
echo "Using RViz config: $RVIZ_CONFIG"
echo "Initial pose auto publish: $AUTO_INITIAL_POSE (x=$INITIAL_POSE_X y=$INITIAL_POSE_Y yaw=$INITIAL_POSE_YAW)"
echo "Fixed map->odom TF: $STATIC_MAP_ODOM_TF (x=$MAP_ODOM_X y=$MAP_ODOM_Y yaw=$MAP_ODOM_YAW)"
echo "Robot path publish: $ENABLE_ROBOT_PATH (topic=$ROBOT_PATH_TOPIC)"
echo "Set the initial pose in RViz if AMCL has not converged or the auto pose is inaccurate."
echo "AMCL TF broadcasting is disabled in $NAV2_PARAMS; fixed map->odom is used to keep the map frame available."
echo "Check status with:"
echo "  ros2 lifecycle get /amcl"
echo "  ros2 topic echo /amcl_pose --once"
echo "  ros2 lifecycle get /bt_navigator"
echo "  ros2 topic info /scan -v"
echo "  ros2 topic echo $ROBOT_PATH_TOPIC --once"
echo "  ros2 run tf2_ros tf2_echo map odom"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo "  ros2 topic info /cmd_vel -v"
echo
echo "Set POINT_LIO_RVIZ=false to run without RViz."
echo "Set AUTO_INITIAL_POSE=false to require manual RViz 2D Pose Estimate."
echo "Set STATIC_MAP_ODOM_TF=false only if another node publishes map->odom."
echo "Set ENABLE_ROBOT_PATH=false to disable /odom to Path publishing."
echo "Press Ctrl+C to stop all started processes."

wait
