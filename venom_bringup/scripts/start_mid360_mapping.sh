#!/usr/bin/env bash
set -euo pipefail

WS="${VENOM_WS:-$HOME/venom_ws}"
CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
SLAM_PARAMS="${SLAM_PARAMS:-$WS/src/venom_vnv/venom_bringup/config/sentry/slam_toolbox_mapping.yaml}"

PIDS=()

# 退出脚本或按 Ctrl+C 时，统一停止本脚本拉起的 ROS 进程。
cleanup() {
    echo
    echo "Stopping mapping stack..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
}

trap cleanup INT TERM EXIT

# 基础环境检查：工作空间和 slam_toolbox 参数文件必须存在。
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "Workspace setup not found: $WS/install/setup.bash" >&2
    echo "Run colcon build first, or set VENOM_WS to the correct workspace." >&2
    exit 1
fi

if [ ! -f "$SLAM_PARAMS" ]; then
    echo "slam_toolbox params not found: $SLAM_PARAMS" >&2
    exit 1
fi

echo "Requesting sudo for CAN setup..."
sudo -v

# can-utils 用于 CAN 调试；如果系统里没有就自动安装。
if ! command -v candump >/dev/null 2>&1; then
    echo "Installing can-utils..."
    sudo apt install -y can-utils
fi

# 加载广成/gs_usb CAN 适配器驱动并拉起 can0。
echo "Loading gs_usb..."
sudo modprobe gs_usb

if ! ip link show "$CAN_IFACE" >/dev/null 2>&1; then
    echo "CAN interface not found: $CAN_IFACE" >&2
    exit 1
fi

echo "Bringing up $CAN_IFACE at $CAN_BITRATE bps..."
sudo ip link set "$CAN_IFACE" down 2>/dev/null || true
sudo ip link set "$CAN_IFACE" up type can bitrate "$CAN_BITRATE"

# 加载工作空间环境，后续 ros2 命令使用当前编译产物。
source "$WS/install/setup.bash"

# 启动原本的 MID360 + Point-LIO 在线里程计入口。
echo "Starting MID360 + Point-LIO odometry..."
ros2 launch venom_bringup mid360_point_lio_odom.launch.py &
PIDS+=("$!")

sleep 6

# 将 Point-LIO 输出的 /cloud_registered 转为 2D /scan。
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

# slam_toolbox 消费 /scan，发布 /map 和 map->odom。
echo "Starting slam_toolbox with $SLAM_PARAMS..."
ros2 launch slam_toolbox online_async_launch.py "slam_params_file:=$SLAM_PARAMS" &
PIDS+=("$!")

echo
echo "Mapping stack is running."
echo "Check topics with:"
echo "  ros2 topic hz /scan"
echo "  ros2 topic hz /map"
echo "  ros2 run tf2_ros tf2_echo map odom"
echo
echo "Press Ctrl+C to stop all started processes."

wait
