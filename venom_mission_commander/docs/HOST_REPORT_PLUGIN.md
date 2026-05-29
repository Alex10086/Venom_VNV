# Host Report Plugin

`host_report` 是 `venom_mission_commander` 的任务级主机回传插件，用于在 waypoint task 链中显式把电表读数和电表图像回传到上位机/主控终端。

该插件面向 CRAIC2026 智慧巡检二号作业点：小车到达二号作业点后，需要读取电表、回传电表图像，并执行语音播报。规则中图像未回传会导致该项不得分，因此比赛配置应把 `required` 设为 `true`。

## Integration Boundary

```text
MissionCommander
→ waypoint tasks
→ read_meter
→ blackboard["meter_reading"]
→ host_report
→ blackboard["last_host_report"]
→ voice_report
```

`MissionCommander` 只负责任务编排；`host_report` 只负责回传动作，不修改 commander 主循环。

## Plugin Location

| Item | Value |
| --- | --- |
| class | `HostReportTaskPlugin` |
| plugin entry | `venom_mission_commander/task_plugins.py` |
| implementation | `venom_mission_commander/host_report_task.py` |
| task type | `host_report` |
| default input key | `meter_reading` |
| default output key | `last_host_report` |

## Backends

### `backend: mock`

仿真/单元测试用，不访问文件系统、不要求图像文件存在。

```yaml
- name: mock_host_report
  type: host_report
  backend: mock
  report_kind: meter_reading
```

### `backend: file`

推荐比赛现场优先使用。插件会在上位机本地生成可见、可留痕的回传目录：

```text
/tmp/venom_host_reports/<report_id>/
  metadata.json
  <meter image>
  receipt.json
```

成功条件：

- `blackboard[reading_key]` 存在。
- 能解析到图像路径。
- 图像文件存在且大小不超过 `max_image_bytes`。
- `copy_image: true` 时图像复制成功；`copy_image: false` 时源图像路径会被记录到回传结果。
- `metadata.json` 和 `receipt.json` 写入成功。

## YAML Parameters

| Parameter | Default | Notes |
| --- | --- | --- |
| `backend` | `mock` | `mock` / `file` |
| `report_kind` | `meter_reading` | 回传类型 |
| `reading_key` | `meter_reading` | 从 blackboard 读取的读数 key |
| `image_path_key` | `image_path` | 从读数 dict 中读取图像路径的字段 |
| `image_path` | empty | 显式图像路径，配置后优先于 blackboard |
| `output_key` | `last_host_report` | 回传结果写回 blackboard 的 key |
| `report_dir` | `/tmp/venom_host_reports` | `file` backend 输出目录 |
| `copy_image` | `true` | 是否复制图像到回传目录 |
| `required` | `false` | 失败时是否让 task 失败 |
| `timeout_sec` | `3.0` | `file` backend 重试循环的总超时 |
| `retry_count` | `0` | 文件写入失败时重试次数 |
| `retry_backoff_sec` | `0.3` | 重试间隔 |
| `max_image_bytes` | `5242880` | 最大图像大小 |

## Blackboard Input

推荐由 `read_meter` 写入：

```python
blackboard["meter_reading"] = {
    "meter_id": "meter_2",
    "value": "1234",
    "confidence": 0.93,
    "source": "printed_number_service",
    "image_path": "/tmp/venom_meter_images/meter_2_1234_success.jpg",
}
```

`host_report` 解析图像路径的优先级：

```text
YAML image_path
> blackboard[reading_key][image_path_key]
> blackboard[image_path_key]
```

## Blackboard Output

成功后写入：

```python
blackboard["last_host_report"] = {
    "success": True,
    "backend": "file",
    "report_kind": "meter_reading",
    "report_id": "...",
    "report_dir": "/tmp/venom_host_reports/...",
    "metadata_path": "/tmp/venom_host_reports/.../metadata.json",
    "receipt_path": "/tmp/venom_host_reports/.../receipt.json",
    "image_path": "/tmp/venom_host_reports/.../image_<sha256-prefix>.jpg",
    "image_sha256": "...",
    "message": "host report saved",
}
```

失败也会写入 `last_host_report`，其中 `success` 为 `false`，`message` 记录原因。

## Required Failure Semantics

沿用 `voice_report` 的风格：

- `required: true`：回传失败时返回 `TaskExecutionResult(False, "host report failed: ...")`。
- `required: false`：回传失败时返回 `TaskExecutionResult(True, "host report skipped: ...")`，mission 可继续执行。

CRAIC2026 二号作业点建议：

```yaml
required: true
```

## Example Mission

已提供配置：

```text
config/meter_host_report_mission.yaml
```

核心任务链：

```yaml
tasks:
  - name: read_meter_at_wp2
    type: read_meter
    backend: service
    output_key: meter_reading

  - name: report_meter_image_to_host
    type: host_report
    backend: file
    reading_key: meter_reading
    image_path_key: image_path
    # 不写 image_path，默认使用 read_meter 返回的 meter_reading.image_path
    required: true

  - name: voice_meter_reading
    type: voice_report
    required: true
```

运行：

```bash
ros2 run venom_mission_commander mission_commander --ros-args \
  -p mission_config:=/home/venom/venom_ws/src/venom_vnv/venom_mission_commander/config/meter_host_report_mission.yaml
```

## Troubleshooting

常见失败信息：

| Message | Meaning |
| --- | --- |
| `missing reading data` | `read_meter` 未写入 `blackboard[reading_key]` |
| `image_path missing` | 读数结果中没有图像路径 |
| `image file not found` | 图像路径不存在 |
| `image too large` | 图像超过 `max_image_bytes` |
| `timeout_sec must be positive` | YAML 参数非法 |

`ReadPrintedNumber.srv` 响应包含 `image_path` 后，`read_meter` 会自动把成功识别图片路径写入 `blackboard["meter_reading"]["image_path"]`。因此新配置可以不写显式 `image_path`，让 `host_report` 使用稳定识别结果对应的图片。

如果使用旧感知节点、外部感知程序，或确实希望固定读取某个稳定路径，也可以显式指定 `image_path`。注意：显式 `image_path` 会优先于 `read_meter` 返回的路径。

使用当前 `printed_number_reader` 生成的 latest 别名时，路径应是：

```yaml
image_path: /tmp/venom_meter_images/meter_2_latest_success.jpg
```

真机部署时需要保证感知节点在调用 `host_report` 前把电表图像保存到该路径。没有稳定外部路径时，推荐不写 `image_path`，直接使用 `meter_reading.image_path`。

## Integrated Recognition + Voice + Host-Report Verification

为了同时测试“数字识别、语音播报、图像回传”三项能力，仓库还提供了集成验证资源：

```text
config/meter_digit_voice_host_report_verification_mission.yaml
launch/meter_digit_voice_host_report_verification.launch.py
```

它与 `config/meter_host_report_mission.yaml` 的区别是：前者是三功能联调验证配置，会配套 launch 一起启动 camera、YOLO、`printed_number_reader` 和 commander，读 `/perception/verification/read_printed_number`，等待/超时更宽松、置信度阈值更低；后者是 CRAIC2026 二号作业点/常规任务样例，只描述 mission，假设外部已提供 `/perception/read_printed_number` service，参数更接近现场任务。两者的 `host_report` 部分保持一致：不写死 `image_path`，优先使用 `read_meter` 返回的 `meter_reading.image_path`。

该 launch 会启动：

- `v4l2_camera`：发布 `/perception/verification/image_raw`。
- `yolo_detector`：读取电表数字模型，发布 `/perception/verification/digit_detections` 和带框调试图 `/perception/verification/digit_yolo_result`。
- `printed_number_reader`：提供 `/perception/verification/read_printed_number` service，订阅带框图像，并只在 service 稳定识别成功时保存图片到 `/tmp/venom_meter_images`。
- `mission_commander`：执行识别 → 回传 → 播报任务链。

运行：

```bash
ros2 launch venom_mission_commander meter_digit_voice_host_report_verification.launch.py
```

可按现场设备覆盖参数：

```bash
ros2 launch venom_mission_commander meter_digit_voice_host_report_verification.launch.py \
  video_device:=/dev/video0 \
  yolo_device:=cpu \
  success_image_dir:=/tmp/venom_meter_images
```

成功识别图片示例：

```text
/tmp/venom_meter_images/meter_digit_voice_host_report_verification_1234_success.jpg
/tmp/venom_meter_images/meter_digit_voice_host_report_verification_latest_success.jpg
```

回传结果默认写入：

```text
/tmp/venom_host_reports/<report_id>/
  metadata.json
  image_<sha256-prefix>.jpg
  receipt.json
```
