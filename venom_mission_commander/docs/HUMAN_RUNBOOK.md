# CRAIC2026 Mission Human Runbook

这份文档给现场调试人员和比赛操作人员使用，目标是回答：**现在怎么跑、跑前检查什么、出问题先看哪里。**

参数字段和插件接口不要在这里重复维护，统一查 [`MISSION_SCHEMA.md`](MISSION_SCHEMA.md)。源码结构和 AI 改代码边界查 [`AI_CONTEXT.md`](AI_CONTEXT.md)。

## 1. 当前比赛边界

CRAIC2026 规则描述两圈逻辑：第一圈探索未知环境、识别减速带/障碍物/任务点并构建循环赛道地图；第二圈基于已构建地图执行自主巡检。

当前 `config/competition_10x6_arm_mission.yaml` 对应的是**第二圈自主巡检执行链**：

- 地图、初始定位、任务点坐标和导航点 yaw 应在赛前完成标定。
- `MissionCommander` 不负责建图，也不动态发现任务点。
- 现场运行时它只按 YAML 顺序导航并触发对应任务模块。

## 2. 比赛任务顺序

```text
起停区
→ 下方减速带
→ 一号作业点：识别并抓取目标物体
→ 上方减速带
→ 二号作业点：电表读取、图像回传、语音播报
→ 二号到三号移动段：提前开启火焰动态追踪
→ 三号作业点：停止追踪并确认火焰识别
→ 四号作业点：识别并放置指定目标物体
→ 返回停车区
```

## 3. 当前 waypoint 表

主配置：`config/competition_10x6_arm_mission.yaml`。

| 顺序 | waypoint | 规则含义 | 坐标 `(x, y, yaw)` | 到点任务 |
| --- | --- | --- | --- | --- |
| 1 | `start_area` | 起停区 | `(1.10, 1.10, 0.0)` | `wait 0.5s`，跳过导航 |
| 2 | `pass_speed_bump_lower` | 第一处减速带 | `(6.00, 1.50, 0.0)` | 无 |
| 3 | `task_point_1_pick` | 一号作业点抓取 | `(7.85, 1.50, 0.0)` | `grasp_item` |
| 4 | `pass_speed_bump_upper` | 第二处减速带 | `(6.00, 4.50, 0.0)` | 无 |
| 5 | `task_point_2_meter_voice` | 二号作业点读表/回传/播报 | `(7.85, 4.50, 0.0)` | `read_meter` → `host_report` → `voice_report` → `track_flame start` → `wait` |
| 6 | `task_point_3_flame_tracking` | 三号作业点火焰识别/追踪确认 | `(5.00, 4.15, 1.5708)` | `track_flame stop` → `detect_flame` |
| 7 | `task_point_4_classify_place` | 四号作业点分类放置 | `(2.10, 4.50, 3.1416)` | `classify_place` |
| 8 | `return_start_area` | 返回停车区 | `(1.10, 1.10, 3.1416)` | `wait 0.5s` |

赛前必须复核：

1. `start_area` / `return_start_area` 是否能让四个车轮完全进入有效区。
2. 两个减速带 waypoint 是否保证所有车轮完整通过，不只是车体中心通过。
3. 四个作业点 yaw 是否让相机/机械臂在可工作姿态。
4. 二号点开启火焰追踪后，到三号点的导航和机械臂追踪是否互不干扰。

## 4. 外部系统启动顺序

`mission_commander.launch.py` 只启动总控节点，不会启动底层系统。建议按“底层先 ready，总控最后启动”的顺序：

```text
1. 底盘、雷达、相机、机械臂、声卡供电和急停检查
2. 启动地图/定位/Nav2，并在 RViz 确认定位稳定
3. 启动读表 YOLO 和 printed_number_reader 服务
4. 启动机械臂 action server / MTC 任务链
5. 启动火焰检测和 flame_arm_tracker
6. 手动检查关键 service/action/topic 均存在
7. 启动 mission_commander，指定 competition_10x6_arm_mission.yaml
```

## 5. 关键接口检查

| 功能 | 必要接口 | 快速检查 |
| --- | --- | --- |
| Nav2 导航 | Nav2 `BasicNavigator.goToPose()` 背后的 action/services | RViz 手动发目标；看 `[STARTUP] navigator_ready` |
| 读表服务 | `/perception/read_printed_number` | `ros2 service list \| grep read_printed_number` |
| 读表图像 | `/tmp/venom_meter_images` 下成功图片 | 识别成功后检查 `*_success.jpg` |
| 图像回传 | `/tmp/venom_host_reports` | 目录可写，生成 `metadata.json` / `receipt.json` |
| 语音播报 | `/home/alex/venom_ws/scripts/speak_meter_wav.sh` | `scripts/speak_meter_wav.sh 1234` |
| 机械臂任务 | `/manipulation/execute_task` | `ros2 action list \| grep execute_task` |
| 火焰检测 | `/perception/detections_2d_array` | `ros2 topic echo /perception/detections_2d_array` |
| 火焰追踪 | `/flame_arm_tracker/set_enabled`、`/flame_arm_tracker/status` | `ros2 service list` 和 status echo |

## 6. 启动 commander

```bash
cd /home/alex/venom_ws
source install/setup.bash

ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=true \
  use_sim_time:=false \
  nav2_wait_mode:=bt_navigator \
  mission_config:=/home/alex/venom_ws/src/venom_vnv/venom_mission_commander/config/competition_10x6_arm_mission.yaml
```

模块逐个联调时，不建议每个模块都新增一个 launch。推荐统一使用
`mission_commander.launch.py`，只切换 skip-navigation 验证 YAML：

```bash
ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=false \
  mock_nav_delay_sec:=0.0 \
  mission_config:=/home/alex/venom_ws/src/venom_vnv/venom_mission_commander/config/verify_point2_meter_host_voice.yaml
```

| 要测的链路 | 推荐 YAML | 运行前先启动 |
| --- | --- | --- |
| 一号点抓取 | `config/verify_point1_grasp.yaml` | 机械臂 action server、抓取视觉/目标融合 |
| 二号点读表 + 播报 | `config/verify_point2_meter_voice.yaml` | `/perception/read_printed_number`、WAV/声卡 |
| 二号点读表 + 图像回传 + 播报 | `config/verify_point2_meter_host_voice.yaml` | `/perception/read_printed_number`，且 response 带 `image_path` |
| 三号点火焰追踪/识别 | `config/verify_point3_flame_tracking.yaml` | flame detector、`/flame_arm_tracker/set_enabled`、`/flame_arm_tracker/status` |
| 四号点分类投放 | `config/verify_point4_classify_place.yaml` | 机械臂 action server，且物体/夹爪前置状态正确 |

旧的 `meter_digit_voice*_verification.launch.py` 仍可作为“一键启动相机 + YOLO + reader + commander”的兼容入口；新的日常模块测试入口以上表为准。

## 7. 现场日志怎么看

建议 2 号机位固定看 commander 终端输出。

| 前缀 | 含义 |
| --- | --- |
| `[STARTUP]` | mission 配置、插件、真实后端和 navigator ready 检查 |
| `[STATUS]` | 当前 waypoint、task、导航尝试、最近任务结果 |
| `[NAV2]` | 目标下发、周期反馈、超时、取消和恢复 |
| `[SUMMARY]` | 程序退出前的最终状态、进度和失败原因 |

默认不打印底层状态迁移；排查状态机时临时加：

```bash
ros2 launch venom_mission_commander mission_commander.launch.py \
  log_state_transitions:=true
```

## 8. 常见故障优先排查

| 现象 | 先看什么 | 常见处理 |
| --- | --- | --- |
| 启动后停在 preflight | `[STARTUP]` 哪一项失败 | 对应启动缺失的 service/topic/action/Nav2 |
| Nav2 不 ready | RViz 手动目标、Nav2 lifecycle、TF/map | 先让 RViz 目标能跑通，再启动 commander |
| 减速带/停车压线 | waypoint 坐标和 yaw | 重新标定 `x/y/yaw`，不要在代码里修 |
| 读表失败 | `/perception/read_printed_number`、数字 detections、成功图片目录 | 降低联调阈值或检查相机/模型/光照 |
| 图像回传失败 | `meter_reading.image_path`、图片大小、`/tmp/venom_host_reports` 权限 | 确认成功图片存在且小于 `max_image_bytes` |
| 语音没声音 | `aplay`、声卡、`voice_assets/` | 手动执行 WAV 脚本，不先查 commander |
| 火焰追踪 start 超时 | `/flame_arm_tracker/status` 字段 | 检查 `target_class_name=fire`、相机 info、joint state、检测 topic |
| 机械臂 action 超时 | `/manipulation/execute_task` action server 和 MoveIt/MTC 日志 | 单独调用 action 验证，再跑整条 mission |

## 9. 失败策略

当前比赛配置使用：

```yaml
stop_on_task_failure: true
```

因此任一关键 task 失败都会让 mission 进入 `FAILED`。这是保守策略，便于现场尽快定位问题。

安全相关行为：

- 导航失败、任务失败、mission 完成或 shutdown 时，commander 会尝试关闭 service-backed 火焰追踪。
- 执行 `grasp_item` / `classify_place` 前也会先尝试停止火焰追踪，避免多个模块同时控制机械臂关节。

## 10. 清理原则

不要只因为文件名包含 `test`、`demo`、`sample` 就删除源码。建议：

| 类型 | 处理 |
| --- | --- |
| `.pytest_cache/`、`__pycache__/`、`.cache/clangd/` | 可安全清理 |
| `*.cmake~` 等编辑器备份 | 可删除 |
| `build/`、`install/`、`log/` | 可删除后重建，但会影响当前运行环境 |
| `src/.../test/test_*.py`、CMake 注册 gtest | 保留 |
| SDK/sample/demo 源码 | 默认保留，除非确认不再需要 |
| `.codex_backups/` | 删除前复核，可能保存历史改动 |
