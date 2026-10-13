# venom_bringup

系统启动配置包

## 功能
- 建图模式启动
- 重定位模式启动
- 集成多传感器驱动
- PX4 DDS 探测示例启动

## 启动文件
- `mapping_bringup.launch.py` - 3D+2D 建图
- `relocalization_bringup.launch.py` - 重定位
- `examples/px4_agent_probe.launch.py` - PX4 uXRCE-DDS 连通性探测与桥接状态检查

## 哨兵极简导航入口

`launch/sentry/sentry_nav_v1.launch.py` 面向 ROS 2 Jazzy、全向轮底盘，
只编排 MID360、Point-LIO、静态 TF、点云转扫描、Nav2 和串口驱动。
不包含固定地图、AMCL、重定位、自瞄、任务调度或内置假传感器。

### 配置

配置集中在 `config/sentry_v1/`：

| 文件 | 职责 |
|---|---|
| `mid360.json` | 雷达和主机网络；当前沿用旧哨兵配置，接雷达前修改实际 IP |
| `point_lio.yaml` | 雷达/IMU 输入、LIO 参数、MID360 内部外参、输出话题与坐标系 |
| `nav2.yaml` | 全向 DWB、滚动代价地图、规划、行为树与速度平滑 |
| `robot.yaml` | 普通 YAML，包含 `transforms`、`lidar`、`scan`、`serial` 四组配置 |

`robot.yaml` 由 launch 分组读取，不能作为整份 ROS 参数文件加载。
雷达安装位置未确定，所以 `transforms: []`，不发布虚构的雷达安装 TF。
`base_link` 暂按 Point-LIO 的 IMU/body 坐标系用于台架联调，
不是已标定的实车底盘原点。`extrinsic_T/R` 沿用 Point-LIO 的 MID360
内部 LiDAR–IMU 配置，不是雷达到地面的安装高度。

### 编译和无硬件启动

在已有运行依赖的工作区中构建。独立构建目录避免旧 build 缓存影响：

```bash
cd ~/venom_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon --log-base log_sentry_v1 build \
  --build-base build_sentry_v1 --install-base install_sentry_v1 \
  --packages-select venom_serial_driver venom_robot_description venom_bringup \
  --symlink-install
source install_sentry_v1/setup.bash
ros2 launch venom_bringup sentry_nav_v1.launch.py
```

默认启动点云转扫描和 Nav2 进程，但关闭雷达、LIO、串口和 Nav2 自动激活。
没有输入时不会生成里程计、扫描或速度，也不会因等待 TF 自动激活超时。
离线联调需要外部仿真或 bag 提供 `/odom`、`/cloud_registered` 和
`odom -> base_link` TF；不要同时启动第二个里程计/TF 发布者。

| 参数 | 默认值 | 作用 |
|---|---|---|
| `config_dir` | 包内 `config/sentry_v1` | 四份配置所在目录 |
| `lidar_enabled` | `false` | 启动 MID360 |
| `lio_enabled` | `false` | 启动 Point-LIO |
| `serial_enabled` | `false` | 允许向 C 板发送控制帧 |
| `nav_enabled` | `true` | 启动 Nav2 和 lifecycle manager |
| `autostart` | `false` | 自动激活 Nav2；仅在输入已就绪时开启 |
| `use_sim_time` | `false` | 使用 bag/仿真的 `/clock` |
| `rviz` | `false` | 启动 RViz，固定坐标系为 `odom` |
| `port_name` | `/dev/ttyACM0` | 串口设备，仅串口启用时使用 |

先确认 `/odom`、`/scan` 与 `odom -> base_link` TF，再激活 Nav2：

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'
```

目标使用 `odom` 坐标系，例如：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: odom}, pose: {position: {x: 0.0, y: 0.8}, orientation: {w: 1.0}}}}'
```

### 逐步接入

1. 外部仿真/处理后的 bag：使用默认入口；输入就绪后手动激活 Nav2。
   仿真或 bag 带 `/clock` 时加 `use_sim_time:=true`。
2. 原始 MID360 bag：加 `lio_enabled:=true use_sim_time:=true`，保持雷达和串口关闭，
   回放时用 `ros2 bag play <bag_path> --clock`；回放输入不要同时提供另一份 odom/TF。
3. 实际雷达：修改网络配置后加 `lidar_enabled:=true lio_enabled:=true`，
   先保持 `serial_enabled:=false`，检查 LIO、点云和扫描。
4. 实车底盘：完成坐标标定、尺寸与速度限制配置、失联停车检查后，
   才加 `serial_enabled:=true port_name:=<实际设备>`。

### 接口和边界

- 输入：`/livox/lidar` 为 Livox CustomMsg，`/livox/imu` 为 IMU。
- LIO 输出：`/odom`、`/cloud_registered`（`odom` 坐标系），唯一动态 TF 为
  `odom -> base_link`；点云转换目标为 `base_link`，输出 `/scan`。
- 控制：controller/behavior 输出 `/cmd_vel_nav`，velocity smoother 输出
  `/cmd_vel`（`geometry_msgs/Twist`），串口使用 `linear.x/y` 和 `angular.z`。
- 全局/局部代价地图均为 `odom` 下的滚动窗口，没有 `map -> odom` 和静态地图。
  全局窗口为 20 m × 20 m，仅用于窗口内近距离目标，LIO 漂移不会被校正。
- 全向平移速度上限为 0.3 m/s，旋转上限为 0.8 rad/s；平滑器为 OPEN_LOOP，
  因当前 Point-LIO 不填写平面里程计速度。DWB 和目标检查器的位置容差均为 0.1 m。
- 未观测区域按自由空间处理，0.3 m 圆形包络和扫描高度区间仅用于联调。
  实车运行前必须替换为实际 footprint、安装坐标和障碍物高度范围。
- 串口 heartbeat 默认关闭。当前串口驱动不对缓存底盘速度做过期清零，
  且无自瞄输入时会发送零云台角度；启用实车前确认电控行为和失联停车。

### 已执行的离线验证

临时输入提供已知几何点云和闭环里程计，运行真实点云转换和 Nav2 节点：
六个 lifecycle 节点激活，横移目标和倒车目标均成功，到点后 `/cmd_vel` 回零。
另用伪终端启动真实串口驱动，确认横向速度、倒车速度和角速度的控制帧及 CRC，
并验证显式零速度帧和正常退出。Point-LIO 已验证配置加载和无输入启动，
未验证真实雷达里程计、C 板反馈或实车运动。
