# Mission Commander AI Context

这份文档给 AI agent 和后续维护者使用，目标是快速建立代码上下文，避免把职责改乱。人类现场运行请看 [`HUMAN_RUNBOOK.md`](HUMAN_RUNBOOK.md)，参数规范请看 [`MISSION_SCHEMA.md`](MISSION_SCHEMA.md)。

## 0. Non-negotiable boundaries

- `MissionCommander` 只负责编排，不实现具体读表、图像识别、语音播报、机械臂规划或火焰检测算法。
- mission flow 由 YAML 驱动；改点位、顺序、超时、service/action/topic 名称时优先改 `config/*.yaml`。
- 新任务能力应通过 task plugin 或 helper 文件接入，不要把业务逻辑塞进 `mission_commander.py`。
- `blackboard` 是 task 之间的运行时数据交换机制；跨任务依赖必须明确读写 key。
- 外部系统由 bringup/launch 负责启动；commander 只做最小 preflight 和运行时调用。
- 当前 CRAIC2026 主线是固定 waypoint + 顺序 task，不做动态 mission 生成；远期想法见 [`archive/FUTURE_DYNAMIC_MISSION_BRANCH.md`](archive/FUTURE_DYNAMIC_MISSION_BRANCH.md)。

## 1. Main execution flow

```text
launch / ros2 run
→ MissionCommander node
→ MissionLoader.load(mission_config)
→ TaskPluginRegistry.register_default_plugins()
→ create MockWaypointNavigator or Nav2WaypointNavigator
→ MissionManager.create_mission()
→ StartupChecker.run()
→ for waypoint in MissionConfig.waypoints
   → mark_waypoint_started()
   → navigate_to_waypoint() or skip_navigation
   → build TaskContext(..., blackboard)
   → WaypointTaskRunner.run_tasks()
      → registry.get(task_spec.task_type).execute(context, task_spec)
      → save last_task_* state
   → mark_waypoint_done()
→ mark_mission_completed() / fail()
→ shutdown cleanup, including flame tracking stop when needed
```

## 2. Important files

| File | Responsibility | Edit when |
| --- | --- | --- |
| `venom_mission_commander/mission_commander.py` | Node lifecycle, config loading, navigator creation, mission loop, navigation failure handling, safety cleanup | Orchestration model itself changes |
| `venom_mission_commander/navigator.py` | Mock/Nav2 navigation adapter; converts `WaypointSpec` to `PoseStamped` and calls Nav2 | Navigation integration changes |
| `venom_mission_commander/mission_loader.py` | YAML → `MissionConfig` / `WaypointSpec` / `TaskSpec` | YAML schema changes |
| `venom_mission_commander/models.py` | Dataclasses/enums/result/context definitions | Shared model contract changes |
| `venom_mission_commander/mission_manager.py` | State machine and state data/history | State semantics or progress recording changes |
| `venom_mission_commander/task_runner.py` | Sequential task execution and fail-fast policy | Task execution policy changes |
| `venom_mission_commander/task_plugins.py` | `task_type` → plugin dispatch | Adding/removing task type |
| `venom_mission_commander/read_meter_task.py` | `read_meter` mock/service backend | Printed-number service integration changes |
| `venom_mission_commander/host_report_task.py` | `host_report` mock/file backend | Image/report persistence changes |
| `venom_mission_commander/voice_report_task.py` | `voice_report` mock/command backend | Speech command/text formatting changes |
| `venom_mission_commander/arm_task_client.py` | Manipulation action client, flame detection topic wait, flame tracking service/status handling | Arm/flame integration changes |
| `venom_mission_commander/startup_checks.py` | Config/plugin/backend/Nav2 preflight | New real backend needs startup graph checks |
| `config/competition_10x6_arm_mission.yaml` | Current CRAIC2026 route and real backend parameters | Competition route/task config changes |

## 3. Current CRAIC2026 route

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

Task chain highlights:

- Point 1: `grasp_item` → `/manipulation/execute_task` → `PICK_AND_PLACE_LATEST_TARGET` → `blackboard["grasped_object"]`.
- Point 2: `read_meter` → `/perception/read_printed_number` in competition/standalone configs, or `/perception/verification/read_printed_number` in one-shot meter verification launches → `meter_reading`; then `host_report`; then `voice_report`; then `track_flame mode=start`.
- Point 3: `track_flame mode=stop`; then `detect_flame` waits on `/perception/flame/detections_2d_array`.
- Point 4: `classify_place` → `/manipulation/execute_task` → `CLASSIFY_PLATFORM_TO_COLOR_BOXES` → `blackboard["last_placement"]`.

## 4. Task plugin contract

All task plugins implement:

```python
class BaseTaskPlugin:
    task_type = 'base'

    def configure(self, node) -> None:
        self.node = node

    def execute(self, context: TaskContext, spec: TaskSpec) -> TaskExecutionResult:
        raise NotImplementedError

    def cancel(self) -> None:
        return None
```

Rules:

- `task_type` must match YAML `type`.
- `spec.params` contains all YAML task fields except `name` and `type`.
- `execute()` must return `TaskExecutionResult(success, message, data)`.
- Timeouts, unavailable service/action, rejected goals and invalid input should return `success=False`, not block forever.
- If `mission.stop_on_task_failure: true`, the first failed task stops the mission.

## 5. Blackboard contract

Current important keys:

| Key | Writer | Reader / use |
| --- | --- | --- |
| `detected_item` | `detect_item` mock/legacy | `grasp_item` mock/legacy |
| `grasped_object` | `grasp_item` | `classify_place` mock/debug summary |
| `meter_reading` | `read_meter` | `host_report`, `voice_report` |
| `last_host_report` | `host_report` | Summary/debug |
| `last_voice_report` | `voice_report` | Summary/debug |
| `flame_detection` | `detect_flame` | Summary/future decisions |
| `flame_tracking_active` | `track_flame` | Safety stop before manipulation/shutdown |
| `flame_tracking_state` | `track_flame` and cleanup | Safety state; `unknown` should be treated conservatively |
| `flame_tracking_ready` | `track_flame mode=start` | Debug/status |
| `flame_tracking_service_name` | `track_flame` | Safety stop service target |
| `last_flame_tracking` | `track_flame` | Summary/debug |
| `last_placement` | `classify_place` | Summary/debug |

Prefer serializable dicts for new keys unless the immediate next task must consume a ROS message object.

## 6. Real backend integrations

| Task | Plugin/helper | External interface |
| --- | --- | --- |
| `grasp_item` | `arm_task_client.execute_manipulation_action_task()` | action `/manipulation/execute_task`, goal constant `PICK_AND_PLACE_LATEST_TARGET` |
| `classify_place` | `arm_task_client.execute_manipulation_action_task()` | action `/manipulation/execute_task`, goal constant `CLASSIFY_PLATFORM_TO_COLOR_BOXES` |
| `read_meter` | `read_meter_task.execute_service_read_meter()` | YAML `service_name`; competition/standalone uses `/perception/read_printed_number`, one-shot meter verification launches use `/perception/verification/read_printed_number` (`printed_number_interfaces/srv/ReadPrintedNumber`) |
| `host_report` | `host_report_task.execute_file_host_report()` | local files under `/tmp/venom_host_reports` |
| `voice_report` | `voice_report_task.execute_command_voice_report()` | local command, CRAIC config uses `$HOME/venom_ws/scripts/speak_meter_wav.sh`; command paths expand `$HOME`/environment variables at runtime |
| `track_flame` | `arm_task_client.execute_flame_tracking_task()` | `SetBool` service `/flame_arm_tracker/set_enabled` + status topic `/flame_arm_tracker/status` |
| `detect_flame` | `arm_task_client.wait_for_flame_detection_task()` | topic `/perception/flame/detections_2d_array` |

Related external packages:

- Digit detection/reading: `perception/yolo_detector`, `perception/printed_number_reader`, `perception/printed_number_interfaces`.
- Pick/place vision and arm work: `perception/grasp_target_fusion`, `manipulation/piper_mtc_tasks`, `manipulation/venom_manipulation`.
- Flame detection/tracking: `manipulation/flame_arm_tracker`, `manipulation/venom_manipulation_interfaces`.

## 7. Startup and safety behavior

`StartupChecker.run()` performs:

```text
mission_config semantic check
→ task_plugins_registered
→ real_backend_preflight for selected real backends
→ navigator_ready
```

Current real backend preflight focuses on flame tracking service/status topic and flame detection topic. It does not prove target acquisition or full backend health; task execution still enforces its own timeouts.

Hunter SE real-robot navigation is an external bringup concern. Use
`venom_bringup/scripts/start_hunter_mid360_mapping.sh` only to create/save a
static map, and `venom_bringup/scripts/start_hunter_mid360_nav2_teb.sh` to run
static-map AMCL + Nav2/TEB before commander starts. Do not move this startup
logic into `mission_commander.py` or `navigator.py`.

Flame tracking safety:

- If the mission uses service-backed `track_flame`, commander tries to stop tracking on startup, navigation failure, task failure, mission completion and shutdown.
- Manipulation tasks call `stop_active_flame_tracking_if_needed()` before sending `/manipulation/execute_task` goals.

## 8. Where to make common changes

| Need | Change here | Avoid |
| --- | --- | --- |
| Adjust competition coordinates/yaw | `config/competition_10x6_arm_mission.yaml` | Hard-coding poses in Python |
| Adjust task timeout/retry/service/topic/action | mission YAML | Hidden constants in plugins |
| Add new backend for existing task | corresponding helper file, then schema docs | Changing mission loop |
| Add new task type | `task_plugins.py` + helper file + `MISSION_SCHEMA.md` | Overloading unrelated task type |
| Add startup graph check | `startup_checks.py` | Blocking inside `MissionCommander.configure()` |
| Change blackboard key | YAML + helper + `MISSION_SCHEMA.md` | Implicit key changes without docs |
| Adjust Hunter navigation bringup | `venom_bringup/config/hunter_se/*.yaml` and `venom_bringup/scripts/start_hunter_mid360_*.sh` | Starting chassis/LiDAR/Nav2 inside commander |

## 9. Test and cleanup guidance

- Keep real package tests under `src/.../test/` and CMake-registered gtests.
- Safe cleanup targets are caches: `.pytest_cache/`, `__pycache__/`, `.cache/clangd/`, editor backup files.
- Do not delete SDK/sample/demo/vendor files without explicit confirmation; many are upstream bringup/debug assets.
- After doc-only edits, at minimum run markdown/link-oriented checks plus existing Python tests if task docs or config examples changed.
