# Mission Commander Schema Reference

这份文档是 `venom_mission_commander` 参数规范的唯一维护入口：ROS 参数、mission YAML 字段、task 参数和 blackboard key 都放在这里。运行手册不要重复参数表，只链接本文件。

## 1. ROS parameters

这些参数由 `MissionCommander.__init__()` 声明，并由 `launch/mission_commander.launch.py` 透传。

| Parameter | Type | Default | Meaning |
| --- | --- | --- | --- |
| `mission_config` | string | package `config/simple_mission.yaml` | mission YAML 路径 |
| `use_nav` | bool | `false` | `false` 使用 mock navigator；`true` 使用 Nav2 |
| `mock_nav_delay_sec` | float | `0.5` | mock navigation 每个 waypoint 等待时间 |
| `nav2_wait_mode` | string | `bt_navigator` | Nav2 ready 等待方式：`bt_navigator` 或 `full` |
| `navigator_ready_timeout_sec` | float | `30.0` | StartupChecker 等待 navigator ready 的最长时间 |
| `nav_feedback_log_interval_sec` | float | `5.0` | Nav2 周期反馈日志间隔；`0` 表示关闭周期反馈 |
| `log_state_transitions` | bool | `false` | 是否打印 `MissionManager` 底层状态迁移 |
| `use_sim_time` | bool | `false` | Gazebo 仿真设 `true`，真机设 `false` |

## 2. YAML top-level structure

```yaml
map:
  name: competition_10x6
  frame_id: map
  source: rm_nav_bringup/map/competition_10x6.yaml

mission:
  id: competition_10x6_arm_mission_commander
  loop: false
  stop_on_task_failure: true
  nav_timeout_sec: 35.0
  retry_count: 1

waypoints:
  - name: task_point_1_pick
    description: 一号作业点
    kind: operation_stop
    frame_id: map
    x: 7.85
    y: 1.50
    yaw: 0.0
    skip_navigation: false
    nav_timeout_sec: 35.0
    retry_count: 1
    tasks:
      - name: grasp_item_at_point_1
        type: grasp_item
        backend: action
```

### `map`

`map` 当前是说明性 metadata，loader 不解析，不影响运行逻辑。建议记录地图名、frame、地图文件来源和标定备注。

### `mission`

| Field | Required | Default | Meaning |
| --- | --- | --- | --- |
| `id` | no | `venom_mission_commander` | mission 运行 ID |
| `loop` | no | `false` | 是否循环执行 waypoint 列表 |
| `stop_on_task_failure` | no | `true` | task 失败时是否立即终止 mission |
| `nav_timeout_sec` | no | `null` | 默认导航超时；设置时必须为正数 |
| `retry_count` | no | `0` | 默认导航重试次数；必须为非负整数 |

### `waypoints[]`

| Field | Required | Default | Meaning |
| --- | --- | --- | --- |
| `name` | yes | none | waypoint 名称，建议稳定可读 |
| `x` | yes | none | `frame_id` 坐标系下目标 x |
| `y` | yes | none | `frame_id` 坐标系下目标 y |
| `frame_id` | no | `map` | Nav2 目标 pose frame |
| `yaw` | no | `0.0` | 目标朝向，单位 rad |
| `kind` | no | `operation_stop` | `pass_through` / `operation_stop` / `return_park` |
| `tasks` | no | `[]` | 到点后按顺序执行的任务列表 |
| `skip_navigation` | no | `false` | 是否跳过导航，常用于起点或纯验证 waypoint |
| `description` | no | `""` | 人类可读说明 |
| `nav_timeout_sec` | no | inherit mission default | 覆盖该 waypoint 的导航超时 |
| `retry_count` | no | inherit mission default | 覆盖该 waypoint 的导航重试次数 |

### `tasks[]`

| Field | Required | Default | Meaning |
| --- | --- | --- | --- |
| `type` | yes | none | 在 `TaskPluginRegistry` 中查找的 task type |
| `name` | no | same as `type` | task 实例名，用于日志和状态记录 |
| other fields | no | none | 全部进入 `TaskSpec.params`，由对应 plugin 解释 |

默认注册 task types：

```text
detect_item, grasp_item, read_meter, host_report, voice_report,
detect_flame, track_flame, classify_place, wait
```

## 3. Task parameters

### `wait`

| Parameter | Default | Meaning |
| --- | --- | --- |
| `seconds` | `1.0` | 等待秒数 |

### `detect_item`

当前主要是 mock/legacy 物品识别任务。

| Parameter | Default | Meaning |
| --- | --- | --- |
| `target` | `unknown_item` | 目标类别/名称 |
| `mock_delay_sec` | `0.5` | mock 等待时间 |
| `mock_confidence` | `0.92` | mock 置信度 |
| `mock_pose_frame` | `camera_link` | mock pose frame |
| `mock_pose_hint` | `mock_center` | mock pose hint |

`detect_item` 当前固定写入 `blackboard["detected_item"]`。

### `grasp_item`

支持 mock 和 manipulation action 后端。CRAIC2026 主配置使用 action 后端。

| Parameter | Default | Backend | Meaning |
| --- | --- | --- | --- |
| `backend` | `mock` | all | `mock` / `action` / `execute_task` / `manipulation` |
| `source` | `detected_item` | mock | 读取 blackboard 的目标 key |
| `action_name` | `/manipulation/execute_task` | action | `ExecuteTask` action 名 |
| `task_type_name` | plugin default | action | action goal 常量名；`grasp_item` 默认为 `PICK_AND_PLACE_LATEST_TARGET` |
| `task_type_value` | unset | action | 直接指定 action goal uint8 值，优先级低于 name 解析逻辑 |
| `timeout_sec` | `60.0` | action | action 总超时 |
| `server_wait_timeout_sec` | `2.0` | action | 等待 action server 可用的时间 |
| `retry_count` | `0` | action | action 调用重试次数 |
| `retry_backoff_sec` | `0.5` | action | 重试间隔 |
| `output_key` | `grasped_object` | all | 写入 blackboard 的 key |

CRAIC2026 point 1 uses:

```yaml
type: grasp_item
backend: action
action_name: /manipulation/execute_task
task_type_name: PICK_AND_PLACE_LATEST_TARGET
timeout_sec: 90.0
output_key: grasped_object
```

### `classify_place`

参数与 `grasp_item` action 后端一致，但 `task_type_name` plugin default 为 `CLASSIFY_PLATFORM_TO_COLOR_BOXES`、`output_key` plugin default 为 `last_placement`。CRAIC2026 point 4 uses:

```yaml
type: classify_place
backend: action
action_name: /manipulation/execute_task
task_type_name: CLASSIFY_PLATFORM_TO_COLOR_BOXES
timeout_sec: 120.0
output_key: last_placement
```

### `read_meter`

| Parameter | Default | Backend | Meaning |
| --- | --- | --- | --- |
| `backend` | `mock` | all | `mock` / `service` |
| `meter_id` | `meter_1` | all | 目标电表 ID |
| `mock_value` | `220.0V` | mock | mock 返回值 |
| `mock_confidence` | `0.9` | mock | mock 置信度 |
| `mock_delay_sec` | `0.5` | mock | mock 等待时间 |
| `service_name` | `/perception/read_printed_number` | service | `ReadPrintedNumber` service 名 |
| `timeout_sec` | `5.0` | service | service 总等待时间 |
| `service_wait_timeout_sec` | `1.0` | service | 等待 service 可用的时间 |
| `expected_digits` | `0` | service | 期望数字位数，`0` 表示不限制 |
| `min_confidence` | `0.0` | service | 插件侧最低置信度校验 |
| `output_key` | `meter_reading` | all | 写入 blackboard 的 key |

Service contract: `printed_number_interfaces/srv/ReadPrintedNumber`。

```srv
string target_id
float32 timeout_sec
uint32 expected_digits
float32 min_confidence
---
bool success
string value
float32 confidence
string message
string image_path
```

Service response 的 `image_path` 非空时会写入 `blackboard[output_key]["image_path"]`，供 `host_report` 使用。

### `host_report`

| Parameter | Default | Meaning |
| --- | --- | --- |
| `backend` | `mock` | `mock` / `file` |
| `report_kind` | `meter_reading` | 回传类型 |
| `reading_key` | `meter_reading` | 从 blackboard 读取的读数 key |
| `image_path_key` | `image_path` | 从读数 dict 中读取图像路径的字段 |
| `image_path` | unset | 显式图像路径，优先于 blackboard |
| `output_key` | `last_host_report` | 回传结果写回 blackboard 的 key |
| `report_dir` | `/tmp/venom_host_reports` | `file` backend 输出目录 |
| `copy_image` | `true` | 是否复制图像到回传目录 |
| `required` | `false` | 失败时是否让 task 失败 |
| `timeout_sec` | `3.0` | `file` backend 重试循环总超时 |
| `retry_count` | `0` | 文件写入失败重试次数 |
| `retry_backoff_sec` | `0.3` | 重试间隔 |
| `max_image_bytes` | `5242880` | 最大图像大小 |

`file` backend output:

```text
/tmp/venom_host_reports/<report_id>/
  metadata.json
  <copied meter image>
  receipt.json
```

Image path resolution order:

```text
YAML image_path
> blackboard[reading_key][image_path_key]
> blackboard[image_path_key]
```

### `voice_report`

| Parameter | Default | Backend | Meaning |
| --- | --- | --- | --- |
| `backend` | `mock` | all | `mock` / `command` |
| `mock_delay_sec` | `0.2` | mock | mock 等待时间 |
| `text` | unset | all | 直接指定播报文本，优先级最高 |
| `template` | unset | all | 使用 `meter_reading` 字段渲染文本 |
| `command` | `spd-say -w` | command | 本机语音命令，会追加播报文本作为最后参数 |
| `timeout_sec` | `4.0` | command | 命令执行超时时间 |
| `required` | `false` | command | command 失败时是否让 task 失败 |

CRAIC2026 uses:

```yaml
type: voice_report
backend: command
template: 电表 {meter_id} 读数 {value}
command: /home/alex/venom_ws/scripts/speak_meter_wav.sh
timeout_sec: 12.0
required: true
```

### `track_flame`

| Parameter | Default | Meaning |
| --- | --- | --- |
| `backend` | `mock` | `mock` / `service` / `tracker` |
| `mode` | `hold` | `start` / `stop` / `hold` |
| `require_detection` | `true` when `mode: hold`, otherwise `false` | 是否要求已有 `flame_detection` |
| `service_name` | `/flame_arm_tracker/set_enabled` | `std_srvs/SetBool` service |
| `status_topic` | `/flame_arm_tracker/status` | tracker status topic |
| `service_wait_timeout_sec` | `1.0` | 等待 service 可用时间 |
| `wait_until_tracking` | `false` | `start` 后是否等待 status ready |
| `require_target_acquired` | `false` | ready 条件是否要求 `target_acquired` |
| `ready_timeout_sec` | `5.0` | 等待 status ready 的超时 |
| `target_class` | `fire` | 目标类别，CRAIC2026 为 `fire` |
| `timeout_sec` / `call_timeout_sec` | `3.0` | service/status 操作总超时 |
| `tracking_duration_sec` | `3.0` | `hold` 模式保持追踪时间 |
| `disable_on_exit` | `true` | `hold` 结束后是否关闭追踪 |
| `output_key` | `last_flame_tracking` | 写入 blackboard 的 key |

`wait_until_tracking: true` 的 ready 条件来自 `venom_manipulation_interfaces/msg/FlameTrackerStatus`：

- `enabled == true`
- `mode == "tracking"`
- `target_class_name == target_class`
- `joint_state_ok == true`
- `camera_info_ok == true`
- `command_output_ok == true`
- 如果 `require_target_acquired: true`，还要求 `target_acquired == true`

### `detect_flame`

| Parameter | Default | Backend | Meaning |
| --- | --- | --- | --- |
| `backend` | `mock` | all | `mock` / `topic` |
| `detection_topic` | `/perception/detections_2d_array` | topic | flame detector 输出 topic |
| `target_class` | `fire` | topic | 目标类别，CRAIC2026 为 `fire` |
| `min_confidence` | `0.45` | topic | 最低置信度 |
| `min_consecutive_detections` | `1` | topic | 连续命中帧数 |
| `timeout_sec` | `5.0` | topic | 等待检测超时 |
| `output_key` | `flame_detection` | all | 写入 blackboard 的 key |

## 4. Blackboard keys

| Key | Writer | Reader / Use | Typical content |
| --- | --- | --- | --- |
| `detected_item` | `detect_item` | `grasp_item` mock/legacy | target/confidence/pose hint |
| `grasped_object` | `grasp_item` | `classify_place` mock/debug | manipulation action result summary |
| `meter_reading` | `read_meter` | `host_report`, `voice_report` | `{meter_id, value, confidence, source, image_path?}` |
| `last_host_report` | `host_report` | summary/debug | report dir, metadata, receipt, copied image/hash, status |
| `last_voice_report` | `voice_report` | summary/debug | text, command/source, duration, status |
| `flame_detection` | `detect_flame` | summary/future tasks | best detection, class, confidence, bbox |
| `flame_tracking_active` | `track_flame` | manipulation safety/shutdown | bool indicating tracker may be active |
| `flame_tracking_state` | `track_flame` / cleanup | manipulation safety/shutdown | `inactive` / `starting` / `active` / `stopping` / `unknown` |
| `flame_tracking_ready` | `track_flame` | summary/debug | bool from latest start wait |
| `flame_tracking_service_name` | `track_flame` | safety stop | service name to stop |
| `last_flame_tracking` | `track_flame` | summary/debug | start/stop result and ready status |
| `last_placement` | `classify_place` | summary/debug | classification/place action result summary |

## 5. Existing mission configs

| Config | Purpose |
| --- | --- |
| `config/simple_mission.yaml` | mock-first 最小验证路线 |
| `config/rmul_sim_mission.yaml` | RMUL Gazebo/Nav2 仿真路线 |
| `config/competition_10x6_arm_mission.yaml` | CRAIC2026 10×6 场地真实后端任务链 |
| `config/competition_mission_template.yaml` | 比赛/真机 mission 模板 |
| `config/competition_10x6_mission.yaml` | 10×6 比赛仿真地图近似路线 |
| `config/verify_point1_grasp.yaml` | skip-navigation，验证一号点抓取 action |
| `config/verify_point2_meter_voice.yaml` | skip-navigation，验证二号点读表 service + WAV 语音 |
| `config/verify_point2_meter_host_voice.yaml` | skip-navigation，验证二号点读表 + 图片回传 + WAV 语音 |
| `config/verify_point3_flame_tracking.yaml` | skip-navigation，验证三号点火焰 tracker service/status + detection topic |
| `config/verify_point4_classify_place.yaml` | skip-navigation，验证四号点分类投放 action |
| `config/printed_number_service_mission.yaml` | 跳过导航，验证读表 service + voice |
| `config/meter_digit_voice_verification_mission.yaml` | 跳过导航，验证数字识别 + WAV 语音 |
| `config/meter_digit_voice_host_report_verification_mission.yaml` | 跳过导航，验证数字识别 + 成功图片 + host report + WAV 语音 |
| `config/meter_host_report_mission.yaml` | 读表、图像回传、语音播报样例 |
