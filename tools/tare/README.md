# Scout Mini / MID360：TARE 自主探索（ROS2 Humble）

已部署到 `/home/hyq/venom_ws/src/venom_vnv`。Ubuntu 22.04 / ROS2 Humble / x86_64。
原 FAST-LIO、BIEVR、Point-LIO、Nav2 入口和开机服务保持原状；TARE 使用单独启动入口。

## 上游版本

- [官方 TARE](https://github.com/caochao39/tare_planner/tree/humble-jazzy)：`humble-jazzy`，固定 `b6760adb4280b379567c76f4e684ce628e28ef71`。
- [官方开发环境](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/humble)：`humble`，固定 `158e67b31b644ed1e8b06eb1d7f70e183cc62591`。
- 只构建 `tare_planner`、`terrain_analysis`、`terrain_analysis_ext`、`local_planner` 和 Venom 适配。
  不需要 ROS1、ros1_bridge、Gazebo 或仿真场景资源。
- `tools/tare/patches` 保存可重复应用的两处上游补丁：安装 OR-Tools 并使用相对 RPATH；路径跟踪器防止空路径越界。
- 开发环境内不用于实车的包由构建脚本添加 `COLCON_IGNORE`，不改动原 `planning/COLCON_IGNORE`。

## 编译

```bash
cd /home/hyq/venom_ws/src/venom_vnv
bash tools/tare/build_tare.sh
```

脚本固定子模块版本、应用补丁、顺序构建，默认每个 CMake 包最多两路编译；不替换系统库。
BIEVR 首次部署依赖原 `tools/bievr/build_bievr.sh`，此机器已经编译完成。

## 实车启动与暂停

先在原终端结束正在运行的 Point-LIO / FAST-LIO / BIEVR / Nav2 / SLAM Toolbox 整套程序。
检查开机服务 `systemctl status scout-full-stack.service`；如果它正在运行，先 `sudo systemctl stop scout-full-stack.service`。
不要与 TARE 同时启动 Nav2：二者的底盘控制链不同。

实车启动前检查 `ip -details link show can0`，应为 `state ERROR-ACTIVE`。
如果接口关闭，按本车原配置执行 `sudo ip link set can0 up type can bitrate 500000`。
TARE 启动脚本会在底盘模式下检查 CAN 状态；预览模式 `start_base:=false` 不要求 CAN。


```bash
cd /home/hyq/venom_ws/src/venom_vnv
bash tools/tare/start_tare.sh bievr
```

上述命令启动 MID360、BIEVR、底盘、TARE、地形分析和 RViz，**初始暂停**。
脚本检测已有 LIO/导航/底盘进程并拒绝重复启动。进入探索前让车静止，等待 IMU 初始化和地形点云稳定。
在另一个 SSH 终端：

```bash
source /opt/ros/humble/setup.bash
source /home/hyq/venom_ws/install/setup.bash
# 状态
ros2 topic echo /tare/status
# 开始/恢复：只有定位、点云和地形数据新鲜且无其他速度发布器时才接受
ros2 service call /tare/set_enabled std_srvs/srv/SetBool '{data: true}'
# 暂停：立即输出零速度，保留当前探索地图
ros2 service call /tare/set_enabled std_srvs/srv/SetBool '{data: false}'
```

退出用启动终端的 Ctrl+C。覆盖完成后 TARE 原生规划返航，抵达初始位置 0.5 m 内后停止；
再次执行全新探索需要重启本次探索程序。重启后不会恢复之前的覆盖图。
首轮实车应在平整、可控区域低速验收，并保留底盘物理急停操作条件。

不接底盘、仅观察真实雷达规划：

```bash
bash tools/tare/start_tare.sh bievr start_base:=false cmd_vel_topic:=/tare/preview_cmd_vel
```

此模式仍需要调用启用服务才能开始 TARE 规划，控制输出只到预览话题。
单纯调用 `ros2 launch venom_bringup scout_mini_tare.launch.py` 默认不启动雷达驱动和底盘，便于回放/调试。

## FAST-LIO 对照

停止当前整套 TARE 后再启动另一种 LIO，不能两种同时运行：

```bash
bash tools/tare/start_tare.sh fastlio
```

FAST-LIO 使用与已有实车部署一致的 `lio_init -> body`，通过独立的
`config/scout_mini/tare/fastlio_alignment.json` 完成 `odom -> lio_init` 重力对齐。
该文件复制了本机原 FAST-LIO 标定，包含当时底盘约 5.56° 俯仰；换装雷达、修改安装外参后须更新标定。
可用 `fastlio_alignment_file:=/绝对路径/alignment.json` 指定新文件。
BIEVR 自身对齐重力，不使用 FAST-LIO 这份变换。
两种 LIO 共享相同 TARE、地形和底盘限速设置。

建议录制同一段原始数据分别回放比较：

```bash
# 每次单独启动一个 LIO；不启动真实驱动/底盘，并让所有节点使用 bag 时钟
ros2 launch venom_bringup scout_mini_tare.launch.py lio:=bievr \
  start_driver:=false start_base:=false use_sim_time:=true \
  cmd_vel_topic:=/tare/preview_cmd_vel rviz:=true
# 另一终端；bag 应只回放原始传感器，避免旧 TF/定位/控制混入
ros2 bag play /数据目录 --clock --topics /livox/lidar /livox/imu
```

回放期间调用启用服务。下一次完整重启后把 `lio:=bievr` 改为 `lio:=fastlio`。
如果旧 bag 的传感器发布 QoS 是 best-effort，FAST-LIO 的 reliable 订阅需要在 bag 回放时指定 reliable QoS override。

## 适配说明

数据链：

```text
MID360 -> BIEVR 或 FAST-LIO -> tare_lio_adapter
       -> 地形分析/扩展地形 -> TARE 探索航点
       -> CMU localPlanner -> pathFollower (TwistStamped)
       -> tare_command_gate -> /cmd_vel (Twist) -> Scout
```

- 根据实际 `body -> base_link` 静态外参消除 MID360 的约 53.6° 安装倾角，保留底盘真实坡度。
  `/tare/state_estimation[_at_scan]` 位姿位于 IMU 原点、朝向使用底盘坐标；局部规划器补偿传感器相对底盘的水平偏移。
  本机现有外参对应约 0.274 m 水平偏移、0.202 m 高度；不是把 0.34 m 斜向距离误当垂直高度。
- 点云按时间戳转换至重力对齐探索坐标，保留 XYZI，剔除车体和 15 m 外点；同步扫描和定位。
- CMU 上游固定使用 `map` / `vehicle` 坐标。因此本入口定义连续 `map -> odom` 恒等变换，
  不启动 SLAM Toolbox 的另一套 `map -> odom`。本模式没有回环校正，长距离结果受 LIO 漂移影响。
- 所有探索数据/控制中间话题位于 `/tare/*`，只有门控器输出实车 `/cmd_vel`。
  本入口不启动 Nav2；现有导航启动文件没有修改。
- 实际底盘约 0.55 m 方形，局部规划使用 0.60 m 尺寸，开启旋转障碍检查。
  地形障碍高度阈值 0.12 m，启用未知地形阻挡及落差检测。
  TARE 视点碰撞半径 0.40 m、平面单层视点、地形高度跟随。
- 线速度限制 0.30 m/s，角速度硬限制 0.80 rad/s，跟踪器内部上限 45°/s，加速度 0.30 m/s²。
- 定位/点云/地形超过 0.8 s 未更新会停车并解除启用，数据恢复后必须再次启用。
  局部路径超过 0.6 s、原始控制超过 0.4 s、探索航点超过 3 s 会保持零速度。
  同时检查消息时间戳，避免重复的旧数据维持运动。
- 发现其他 `/cmd_vel` 发布器（包括同名重复门控器）则拒绝启用/解除启用；这不替代停止旧导航程序。
  关键规划/适配进程异常退出时本启动入口整体退出。
- 任务暂停通过 `/tare/set_enabled`，原 CMU `/joy` 隔离在 `/tare/joy`，不直接接入现有手柄话题。

可调参数：`venom_bringup/config/scout_mini/tare/{planner,ground_stack}.yaml`。
TARE 区域约束接口：`/tare/navigation_boundary`，类型 `geometry_msgs/msg/PolygonStamped`、坐标 `map`；
禁行区为 `/tare/sensor_coverage_planner/nogo_boundary`。默认没有限定探索边界，按可达空间规划。
修改配置后重跑构建脚本；也可用 `planner_params:=/绝对路径/planner.yaml` 覆盖 TARE 参数。

## 验证

```bash
source /opt/ros/humble/setup.bash
source /home/hyq/venom_ws/install/setup.bash
cd /home/hyq/venom_ws/src/venom_vnv
python3 tools/tare/smoke_test.py --lio bievr
python3 tools/tare/smoke_test.py --lio fastlio
```

测试使用 localhost 独立 DDS 域（默认 178，可通过 `--domain` 指定）、合成倾斜 MID360 点云/IMU，
启动真实 LIO、TARE、地形分析、局部规划和门控器；不启动真实驱动/底盘，速度输出到 `/tare/test_cmd_vel`。
FAST-LIO 合成测试使用与合成传感器安装一致的临时对齐标定，不改实车标定。
测试覆盖：实际探索航点、局部路径、非零且受限的控制、初始暂停、暂停恢复、数据断流停车、
恢复后保持暂停、第二个控制发布器冲突时拒绝启用。
结果和完整日志保存在 `/home/hyq/venom_ws/runtime/tare_smoke/`。
这些是软件链路验证；真实环境的避障距离、地形适应性和探索覆盖效率仍需实车验收。

### 本机验证记录（2026-11-09）

已完成编译、Python/Bash 语法检查和两种 LIO 的端到端合成数据验证：

| LIO | 检查结果 | 探索航点 | 局部路径消息 | 修正后合成平地俯仰 |
| --- | --- | --- | --- | --- |
| BIEVR-LIO | 13/13 通过 | 54 | 577 | -0.0029° |
| FAST-LIO | 13/13 通过 | 54 | 590 | -0.0314° |

该俯仰值只验证安装倾角的坐标修正，不代表实车定位精度；合成测试没有车辆动力学闭环。
结果文件：

- `/home/hyq/venom_ws/runtime/tare_smoke/bievr_20261109_113159/result.json`
- `/home/hyq/venom_ws/runtime/tare_smoke/fastlio_20261109_113159/result.json`
- 编译日志：`/home/hyq/venom_ws/runtime/tare_build/final_build.log`

运行阶段没有 ERROR/FATAL。上述测试日志在结束时可见 SIGINT 退出异常：测试向进程组发送
SIGINT 后，launch 又向子进程发送 SIGINT；上游轮询节点在 ROS context 关闭后退出时会打印
`failed to create guard condition`。这发生在测试结束阶段，全部测试进程已退出，无底盘或雷达驱动被启动。
实车探索、实际障碍物/台阶避让、返航精度和覆盖率尚未验收；首次运行先采用预览模式检查。
