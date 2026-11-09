# Venom：FAST-LIO / BIEVR-LIO 对比

BIEVR-LIO 是新增选项。原 FAST-LIO、Point-LIO、开机服务及原 FAST-LIO runtime 均未替换。
上游：https://github.com/ethz-asl/BIEVR-LIO
本次固定版本：`2306022e341e98ce84244f92ba7d26f047b41dbc`。

## 已接入内容

- ROS2 Humble + MID360 原生 `livox_ros_driver2/msg/CustomMsg`。
- `/livox/lidar`、`/livox/imu` 输入；沿用本车 FAST-LIO 的 LiDAR→IMU 外参：
  平移 `[-0.011, -0.02329, 0.04412]`，旋转单位阵。
- BIEVR 自身重力初始化，静止等待至少 2 秒；自动识别 Livox 加速度是否以 g 为单位。
- TF：`map -> odom` 由 SLAM Toolbox 发布，`odom -> body` 由 BIEVR 发布，
  `body -> base_link` 沿用车上静态标定。没有复制 FAST-LIO 的初始对齐矩阵。
- 当前帧全点云 `/bievr_lio/points/registered` 转 `/scan`；保留原高度过滤和导航限速。
- SLAM Toolbox 二维建图、Nav2、RViz 均有独立入口。
- 累计点云 `/bievr_lio/map` 和 PCD 快照；每次启动创建新的输出目录。

## 编译

在远端 Ubuntu：

```bash
cd /home/hyq/venom_ws/src/venom_vnv
git submodule update --init localization/lio/BIEVR-LIO
bash tools/bievr/build_bievr.sh
```

Ceres 2.2 静态库安装在 `/home/hyq/venom_ws/third_party/bievr/ceres-2.2.0-install`，
仅供新增 BIEVR 包编译使用，不替换系统 Ceres 2.0。脚本默认以 2 线程编译，可用
`BIEVR_BUILD_JOBS=4 bash tools/bievr/build_bievr.sh` 调整。
上游同时带 ROS1 接口；构建脚本为其创建本地 `COLCON_IGNORE`，避免混入 ROS2 的全工作空间构建。
Livox 支持必须在编译时启用；脚本先加载已有 workspace overlay，再编译 4 个指定包。
依赖 Eigen、TBB、yaml-cpp、glog、SuiteSparse 及 ROS2 Humble；这台机器均已具备。

## 启动

先在原终端 Ctrl+C 停止原算法。如开机服务正在运行，先执行：

```bash
systemctl --user stop scout-full-stack.service
```

不要同时启动两个算法的实车全栈：它们共享 `body`、`odom`、`/scan` 和导航控制话题。
辅助启动脚本会检查常见 LIO 进程；直接使用 ros2 launch 时仍须自行保证没有重复发布者。

只运行 BIEVR 雷达里程计（启动雷达，不启动底盘/导航）：

```bash
cd /home/hyq/venom_ws/src/venom_vnv
bash tools/bievr/start_bievr.sh lio rviz:=true
```

完整建图和导航（启动 MID360、Scout can0 底盘、SLAM Toolbox、Nav2）：

```bash
bash tools/bievr/start_bievr.sh mapping
```

仅建图、不启动 Nav2：

```bash
bash tools/bievr/start_bievr.sh mapping navigation:=false
```

无图形界面时加 `rviz:=false`。启动时保持车辆静止至少 2 秒。
底盘和 can0 配置与旧 FAST-LIO runtime 相同；脚本不自动配置 CAN 或修改开机服务。

原 FAST-LIO 入口仍是：

```bash
bash /home/hyq/venom_ws/runtime/fastlio_nav_20261106/start_fastlio.sh
```

原默认开机入口仍为 Point-LIO；要恢复原服务：

```bash
systemctl --user start scout-full-stack.service
```

## 结果和限制

新结果在 `/home/hyq/venom_ws/runtime/bievr/<启动时间>/`：

- `trajectory.tum`：时间戳、位置、四元数。
- `sensor.yaml`、`bievr_params.yaml`：本次运行实际使用的配置。
- `maps/bievr.pcd`：每 60 秒及正常 Ctrl+C 退出时原子保存。

PCD 是注册点云的可视化快照，并非 BIEVR 内部图像/体素地图的完整序列化：
每秒采一帧、0.1m 体素、最多 30 万点，达到上限后淘汰最久未更新的体素；只保留 XYZ。
轨迹与 PCD 位于 `odom` 下，不含 SLAM Toolbox 回环校正。二维地图需另外保存：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/hyq/venom_ws/runtime/bievr/my_2d_map
```

这次沿用现有物理外参，没有重新做标定。默认最多使用 4 个 TBB 线程，其余算法参数
接近上游；是否优于 FAST-LIO 必须用同一段实测数据比较漂移、丢跟踪、CPU和延迟。
目前未找到可直接使用的 MID360 实测 bag；合成数据只能验证数据链与重力/TF初始化。

## 无硬件验证与同 bag 对比

```bash
source /opt/ros/humble/setup.bash
source /home/hyq/venom_ws/install/setup.bash
python3 /home/hyq/venom_ws/src/venom_vnv/tools/bievr/smoke_test.py
```

脚本固定使用隔离域 177、localhost，关闭雷达/底盘/Nav2/RViz，模拟倾斜 53.583° 的静态
MID360 和 IMU，验证里程计、TF、注册点云、LaserScan、二维地图和 PCD/TUM 保存。
结果和日志在 `/home/hyq/venom_ws/runtime/bievr_smoke/<时间>/`。
运行隔离测试时不要在域 177 使用其他程序。加 `--navigation` 可额外验证 Nav2 七个生命周期节点均为 active，仍不启动底盘。

录制同一段真实输入（先确认录包终端加载了工作空间）：

```bash
ros2 bag record -o /home/hyq/venom_ws/runtime/mid360_compare /livox/lidar /livox/imu
```

仅回放原始传感器话题，避免旧 TF/odom 干扰。两个回放终端都设置同一隔离域，例如
`export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1`，并加载 ROS 和 workspace。
先启动 BIEVR（不启驱动、底盘）：

```bash
ros2 launch venom_bringup scout_mini_bievr.launch.py use_sim_time:=true
```

另一个终端回放：

```bash
ros2 bag play /home/hyq/venom_ws/runtime/mid360_compare --clock
```

FAST-LIO 的原 `start_fastlio.sh` 会启动实车驱动，不能用于离线 bag 对比。
离线 FAST-LIO 可单独启动算法并记录其 `/odom`（与 BIEVR 输出不同）：

```bash
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file /home/hyq/venom_ws/src/venom_vnv/localization/lio/Fast-LIO/config/mid360.yaml \
  -p use_sim_time:=true -p mapping.extrinsic_est_en:=false \
  -p frame.base_frame_id:=body -p pcd_save.pcd_save_en:=false
```

每轮先停止上一轮算法及回放，再从头播放同一 bag；启动段必须静止。
进行精度比较前，应对齐两条轨迹的初始坐标系；有参考真值时再计算 ATE/RPE。

## 本机验证记录（2026-11-09）

- Ceres 2.2 静态编译；BIEVR 核心、ROS 公共接口、ROS2 接口及 bringup 共 4 包编译通过。
- 构建日志确认 Livox 支持开启；无缺失动态库；系统 Ceres 未替换。
- 最终 30 秒合成测试：278 帧 odom / registered cloud / scan，5 次累计点云发布，21 次二维地图发布。
- Nav2 7 个生命周期节点全部 active；PCD 和 TUM 正常保存；测试结束全部退出。
- 最终测试日志无 ERROR/FATAL，结果：
  `/home/hyq/venom_ws/runtime/bievr_smoke/20261109_110649/result.json`。
- 未启动实车雷达或底盘，也没有发送导航目标；实测精度、丢跟踪及 CPU 负载仍需真实 bag 验证。
