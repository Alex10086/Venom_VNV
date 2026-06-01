# venom_mission_commander

`venom_mission_commander` 是 CRAIC2026 智慧巡检任务编排包，负责把“导航到 waypoint → 到点执行 tasks → 记录状态 → 进入下一点”这条链路稳定跑起来。

它的核心原则是：**流程由 mission YAML 描述，任务能力由插件适配，底层硬件/感知/机械臂系统由外部 bringup 提供。**

## 它负责什么

- 读取 mission YAML，并解析为 `MissionConfig` / `WaypointSpec` / `TaskSpec`。
- 调用 mock navigator 或 Nav2 navigator 前往每个 waypoint。
- 到点后按 YAML 顺序执行 `read_meter`、`host_report`、`voice_report`、`track_flame`、`grasp_item` 等 task plugin。
- 通过 `blackboard` 在任务之间传递读表结果、图片路径、火焰追踪状态和机械臂结果。
- 记录 `[STARTUP]`、`[STATUS]`、`[NAV2]`、`[SUMMARY]` 日志，便于现场 2 号机位观察。

## 它不负责什么

- 不负责建图、第一圈探索或动态发现任务点。
- 不负责启动底盘、雷达、相机、Nav2、RViz、机械臂 action server、YOLO 或语音设备。
- 不负责感知算法、机械臂规划、模型训练或声卡配置。
- 不把比赛业务逻辑写死在 `mission_commander.py` 主循环里。

## 常用入口

| 内容 | 路径 |
| --- | --- |
| 主 executable | `mission_commander = venom_mission_commander.mission_commander:main` |
| 通用 launch | `launch/mission_commander.launch.py` |
| Nav2 仿真快捷 launch | `launch/mission_commander_nav2_sim.launch.py` |
| CRAIC2026 主配置 | `config/competition_10x6_arm_mission.yaml` |
| 任务插件分发 | `venom_mission_commander/task_plugins.py` |
| 主节点 | `venom_mission_commander/mission_commander.py` |

## 文档入口

文档按用途分成两类：给人看、给 AI 看；参数规范单独维护，避免重复。

| 读者 | 文档 | 用途 |
| --- | --- | --- |
| 人 | [`docs/HUMAN_RUNBOOK.md`](docs/HUMAN_RUNBOOK.md) | 比赛/联调怎么启动、怎么检查、常见故障怎么排查 |
| AI | [`docs/AI_CONTEXT.md`](docs/AI_CONTEXT.md) | 改代码前理解架构边界、文件职责、插件/blackboard 约定 |
| 人 + AI | [`docs/MISSION_SCHEMA.md`](docs/MISSION_SCHEMA.md) | mission YAML、ROS 参数、task 参数和 blackboard key 的唯一规范来源 |
| 历史设计 | [`docs/archive/`](docs/archive/) | 阶段路线和远期动态 mission 方案，非当前比赛主线 |

旧的长文档仍保留为跳转页，避免已有链接失效。

## 当前 CRAIC2026 执行链

```text
start_area
→ pass_speed_bump_lower
→ task_point_1_pick
→ pass_speed_bump_upper
→ task_point_2_meter_voice
→ task_point_3_flame_tracking
→ task_point_4_classify_place
→ return_start_area
```

对应比赛任务：启动、两个减速带、一号点抓取、二号点读表/图像回传/语音播报、三号点火焰识别/动态追踪、四号点分类放置、返回停车区。

## 快速启动示例

全任务 mock，导航也 mock，只测 mission_commander 流程：

```bash
cd "$HOME/venom_ws"
source install/setup.bash

ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=false \
  mock_nav_delay_sec:=0.0 \
  mission_config:=$HOME/venom_ws/src/venom_vnv/venom_mission_commander/config/competition_10x6_mission.yaml
```

全任务 mock，只测 Nav2 导航：

```bash
ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=true \
  use_sim_time:=false \
  nav2_wait_mode:=bt_navigator \
  mission_config:=$HOME/venom_ws/src/venom_vnv/venom_mission_commander/config/competition_10x6_mission.yaml
```

单模块 skip-navigation 验证也复用同一个 launch，只切换 `verify_point*.yaml`：

```bash
ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=false \
  mock_nav_delay_sec:=0.0 \
  mission_config:=$HOME/venom_ws/src/venom_vnv/venom_mission_commander/config/verify_point2_meter_host_voice.yaml
```

现有模块验证配置见 `docs/HUMAN_RUNBOOK.md`。

CRAIC2026 真机/仿真 Nav2 任务编排：

```bash
cd "$HOME/venom_ws"
source install/setup.bash

ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=true \
  use_sim_time:=false \
  nav2_wait_mode:=bt_navigator \
  mission_config:=$HOME/venom_ws/src/venom_vnv/venom_mission_commander/config/competition_10x6_arm_mission.yaml
```

运行比赛配置前，应先启动 Nav2、读表服务、机械臂 action server、火焰检测/追踪和语音设备；完整顺序见 `docs/HUMAN_RUNBOOK.md`。

## 修改原则

- 改导航点、任务顺序、超时和后端接口：优先改 `config/*.yaml`。
- 新增任务能力：优先新增/修改 task plugin 或对应 helper 文件。
- 不要把具体读表、语音、机械臂或火焰算法塞进 `mission_commander.py`。
- 任务参数只更新 `docs/MISSION_SCHEMA.md`，不要在多个文档重复维护。
