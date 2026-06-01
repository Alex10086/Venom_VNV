---
title: 任务层
description: 任务编排、行为树、状态监听、目标下发与任务执行相关模块的统一入口。
---

## 层级职责

任务层负责回答两个问题：

1. 当前系统要做什么
2. 这个任务由谁下发、谁监听、谁推进

它本身不负责底层轨迹优化，也不直接承担硬件接入。

## 推荐目录结构

任务层推荐统一使用：

```text
mission/
├── navigation/
└── manipulation/
```

其中当前最推荐的组织方式是：

```text
mission/
├── navigation/
│   ├── venom_waypoint/
│   ├── venom_nav_bt/
│   ├── venom_global_monitor/
│   └── venom_mission_manager/
└── manipulation/
    ├── venom_grasp_mission/
    └── venom_pick_place_manager/
```

## 与规划层的边界

- `planning/` 负责“怎么规划路径、轨迹、控制量”
- `mission/` 负责“什么时候发目标、如何切任务、如何根据状态推进流程”

这两个层级不要混在一起。

例如：

- `ego-planner-swarm`、`venom_teb_controller` 应进入 `planning/navigation/`
- `venom_waypoint`、`venom_nav_bt`、`venom_global_monitor` 应进入 `mission/navigation/`
- 机械臂运动规划包应进入 `planning/manipulation/`
- 机械臂抓取任务流程包应进入 `mission/manipulation/`

## 推荐包命名

- 导航任务入口：`venom_waypoint`
- 导航行为树：`venom_nav_bt`
- 全局状态监听：`venom_global_monitor`
- 任务调度与管理：`venom_mission_manager`
- 抓取任务流程：后续可按具体任务命名，例如 `venom_grasp_mission`

## 当前状态

当前主工作区已经创建 `mission/` 目录，并落地了 `navigation/` 与 `manipulation/` 两个预留子目录。

这两个目录当前只用于承接后续独立任务包，仓库里已经可运行的任务控制实现主要有两条：通用导航/健康管理仍在 `venom_bringup` 内；CRAIC2026 智慧巡检比赛的“导航点 + 作业点任务插件”编排已经沉淀到 `venom_mission_commander` 包。

| 当前实现 | 路径 | 作用 |
| --- | --- | --- |
| CRAIC2026 比赛任务编排 | `venom_mission_commander/venom_mission_commander/mission_commander.py` | 读取 mission YAML，按 waypoint 导航，到点后触发读表、回传、语音、火焰追踪和机械臂任务 |
| CRAIC2026 比赛配置 | `venom_mission_commander/config/competition_10x6_arm_mission.yaml` | 记录比赛地图、导航点、任务点和真实后端参数 |
| CRAIC2026 人类运行手册 | `venom_mission_commander/docs/HUMAN_RUNBOOK.md` | 梳理比赛启动顺序、检查项、现场日志和常见故障 |
| mission commander AI 上下文 | `venom_mission_commander/docs/AI_CONTEXT.md` | 梳理架构边界、代码职责、插件/黑板约定和安全规则 |
| mission YAML/task 参数规范 | `venom_mission_commander/docs/MISSION_SCHEMA.md` | 维护 ROS 参数、YAML 字段、task 参数和 blackboard key |
| 多航点导航入口 | `venom_bringup/venom_bringup/multi_waypoint_commander.py` | 读取 `waypoints.yaml`，调用 Nav2 Simple Commander 的 `followWaypoints()` |
| 健康状态感知入口 | `venom_bringup/venom_bringup/health_aware_commander.py` | 在多航点任务基础上接入状态监听、返航与恢复逻辑 |
| 任务控制核心 | `venom_bringup/venom_bringup/mission_controller/` | 提供状态监控、任务状态管理和行为插件抽象 |
| 任务插件 | `venom_bringup/venom_bringup/plugins/` | 当前包含健康状态插件与导航任务插件 |
| 参数文件 | `venom_bringup/config/scout_mini/mission_config.yaml`、`venom_bringup/config/scout_mini/waypoints.yaml` | 当前 Scout Mini 任务控制示例配置 |

因此当前判断规则是：

1. 跑 CRAIC2026 智慧巡检比赛流程时，从 `venom_mission_commander` 和对应 mission YAML 进入。
2. 使用通用多航点/健康感知任务控制功能时，从 `venom_bringup` 的 launch 或 `multi_waypoint_commander` 进入。
3. 新增独立任务包时，放进 `mission/navigation/` 或 `mission/manipulation/`。
4. 如果未来把已有任务控制代码迁出 `venom_bringup`，必须同步迁移 launch、参数路径和文档链接。

## 当前可用入口

```bash
cd ~/venom_ws
source install/setup.bash
ros2 launch venom_bringup health_aware_navigation.launch.py
```

或者直接运行 console script：

```bash
cd ~/venom_ws
source install/setup.bash
ros2 run venom_bringup multi_waypoint_commander
```

CRAIC2026 比赛任务编排入口：

```bash
cd ~/venom_ws
source install/setup.bash
ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=true \
  mission_config:=/home/alex/venom_ws/src/venom_vnv/venom_mission_commander/config/competition_10x6_arm_mission.yaml
```

## 相关页面

- [总体架构](../architecture.md)
- [规划层](../planning/index.md)
- [系统层](../integration/index.md)
- [CRAIC2026 人类运行手册](../../../venom_mission_commander/docs/HUMAN_RUNBOOK.md)
- [mission commander AI 上下文](../../../venom_mission_commander/docs/AI_CONTEXT.md)
- [mission YAML/task 参数规范](../../../venom_mission_commander/docs/MISSION_SCHEMA.md)
