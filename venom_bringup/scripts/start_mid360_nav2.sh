#!/usr/bin/env bash
set -euo pipefail

WS="${VENOM_WS:-$HOME/venom_ws}"
CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
POINT_LIO_RVIZ="${POINT_LIO_RVIZ:-true}"
SLAM_PARAMS="${SLAM_PARAMS:-$WS/src/venom_vnv/venom_bringup/config/sentry/slam_toolbox_mapping.yaml}"
NAV2_PARAMS="${NAV2_PARAMS:-$WS/src/venom_vnv/venom_bringup/config/scout_mini/nav2_params.yaml}"

PIDS=()

# 退出脚本或按 Ctrl+C 时，停止本脚本启动的后台进程。
cleanup() {
    echo
    echo "Stopping MID360 Nav2 stack..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
}

trap cleanup INT TERM EXIT

# 检查工作空间和参数文件。
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "Workspace setup not found: $WS/install/setup.bash" >&2
    echo "Run colcon build first, or set VENOM_WS to the correct workspace." >&2
    exit 1
fi

if [ ! -f "$SLAM_PARAMS" ]; then
    echo "slam_toolbox params not found: $SLAM_PARAMS" >&2
    exit 1
fi

if [ ! -f "$NAV2_PARAMS" ]; then
    echo "Nav2 params not found: $NAV2_PARAMS" >&2
    exit 1
fi

echo "Requesting sudo for CAN setup..."
sudo -v

# can-utils 没装时自动安装，便于后续用 candump 等命令排查 CAN。
if ! command -v candump >/dev/null 2>&1; then
    echo "Installing can-utils..."
    sudo apt install -y can-utils
fi

# 加载 gs_usb 驱动并拉起 can0。
echo "Loading gs_usb..."
sudo modprobe gs_usb

if ! ip link show "$CAN_IFACE" >/dev/null 2>&1; then
    echo "CAN interface not found: $CAN_IFACE" >&2
    exit 1
fi

echo "Bringing up $CAN_IFACE at $CAN_BITRATE bps..."
sudo ip link set "$CAN_IFACE" down 2>/dev/null || true
sudo ip link set "$CAN_IFACE" up type can bitrate "$CAN_BITRATE"

# colcon 的 setup.bash 会读取未定义变量，临时关闭 nounset。
set +u
source "$WS/install/setup.bash"
set -u

# 1. MID360 + Point-LIO。这个 launch 会按 rviz 参数打开 lio.rviz。
echo "Starting MID360 + Point-LIO..."
ros2 launch venom_bringup mid360_point_lio.launch.py "rviz:=$POINT_LIO_RVIZ" &
PIDS+=("$!")

sleep 8
# 临时修正：将 base_link 旋转 180° 得到 base_link_corrected，使红色箭头指向车头
echo "Publishing correction TF from base_link to base_link_corrected..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 3.14159 base_link base_link_corrected &
PIDS+=("$!")
sleep 1

# 2. /cloud_registered 转 /scan。
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

# 3. slam_toolbox 消费 /scan，发布 /map 和 map->odom。
echo "Starting slam_toolbox with $SLAM_PARAMS..."
ros2 launch slam_toolbox online_async_launch.py "slam_params_file:=$SLAM_PARAMS" &
PIDS+=("$!")

sleep 4

# 4. Hunter 底盘驱动。隔离底盘里程计，避免和 Point-LIO 的 odom->base_link 冲突。
echo "Starting Hunter base..."
ros2 launch hunter_base hunter_base.launch.py \
    "port_name:=$CAN_IFACE" \
    odom_frame:=hunter_odom \
    base_frame:=hunter_base_link \
    odom_topic_name:=hunter_odom &
PIDS+=("$!")

sleep 2

# 5. Nav2。controller 输出经 velocity_smoother 发布到 /cmd_vel，Hunter 订阅 /cmd_vel。

echo "Starting Nav2 with $NAV2_PARAMS..."
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=False \
    autostart:=True \
    "params_file:=$NAV2_PARAMS" &
PIDS+=("$!")

echo
echo "MID360 Nav2 stack is running."
echo "Check status with:"
echo "  ros2 topic hz /map"
echo "  ros2 topic info /scan -v"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo "  ros2 topic info /cmd_vel -v"
echo "  ros2 lifecycle get /bt_navigator"
echo
echo "Send a small test goal with:"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'"
echo
echo "Set POINT_LIO_RVIZ=false to run without RViz."
echo "Press Ctrl+C to stop all started processes."

wait
