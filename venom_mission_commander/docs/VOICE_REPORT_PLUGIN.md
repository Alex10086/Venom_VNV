# Voice Report Plugin

本文档说明 `venom_mission_commander` 中 `voice_report` 任务插件的 mock 和本机命令播报方式。

核心边界：`MissionCommander` 只负责任务编排；`VoiceReportTaskPlugin` 根据 `blackboard["meter_reading"]` 生成播报文本；真实语音输出可以先通过本机命令接入。

```text
read_meter
→ blackboard["meter_reading"]
→ voice_report plugin
→ blackboard["last_voice_report"]
```

## Plugin Location

| Item | Value |
| --- | --- |
| class | `VoiceReportTaskPlugin` |
| plugin entry | `venom_mission_commander/task_plugins.py` |
| implementation | `venom_mission_commander/voice_report_task.py` |
| task type | `voice_report` |
| input key | `meter_reading` |
| output key | `last_voice_report` |

`task_type` 保持为 `voice_report`，所以现有 mission YAML 不需要改任务类型。

## Backends

### `backend: mock`

默认行为。未配置 `backend` 时仍走 mock，保证旧 mission 不受影响：

```yaml
- name: voice_report_meter
  type: voice_report
  mock_delay_sec: 0.2
```

输出写入：

```python
blackboard["last_voice_report"] = {
    "text": "电表 meter_1 读数 1234，执行播报操作",
    "success": True,
    "source": "mock_tts",
    "message": "mock",
}
```

### `backend: command`

需要临时调用本机 TTS 命令时使用。插件会把生成的播报文本追加到命令参数末尾：

```yaml
- name: voice_report_meter
  type: voice_report
  backend: command
  command: "spd-say -w"
  timeout_sec: 4.0
  required: false
```

实际执行形式类似：

```bash
spd-say -w "电表 meter_1 读数 1234，执行播报操作"
```

成功后写入：

```python
blackboard["last_voice_report"] = {
    "text": "电表 meter_1 读数 1234，执行播报操作",
    "success": True,
    "source": "command_tts",
    "command": "spd-say",
    "message": "speech command completed",
    "duration_sec": 0.42,
}
```

## Text Selection

`voice_report` 默认读取：

```python
blackboard["meter_reading"] = {
    "meter_id": "meter_1",
    "value": "1234",
    "confidence": 1.0,
}
```

默认播报文本为：

```text
电表 meter_1 读数 1234，执行播报操作
```

可以用 `text` 直接覆盖：

```yaml
- name: voice_report_meter
  type: voice_report
  text: "读数完成"
```

也可以用 `template` 从 `meter_reading` 里取值：

```yaml
- name: voice_report_meter
  type: voice_report
  template: "电表 {meter_id} 当前读数为 {value}"
```

如果 template 缺少字段或格式错误，插件会记录 warning，并回退默认文本。

## YAML Parameters

| Parameter | Default | Backend | Notes |
| --- | --- | --- | --- |
| `backend` | `mock` | all | 支持 `mock` / `command` |
| `mock_delay_sec` | `0.2` | mock | mock 等待时间 |
| `text` | unset | all | 直接指定播报文本，优先级最高 |
| `template` | unset | all | 使用 `meter_reading` 字段渲染文本 |
| `command` | `spd-say -w` | command | 本机语音命令，会追加播报文本作为最后一个参数 |
| `timeout_sec` | `4.0` | command | 命令执行超时时间 |
| `required` | `false` | command | command 失败时是否让任务失败 |

## Failure Behavior

`backend: command` 会处理这些情况：

- command 为空：`empty speech command`
- command 不存在：`speech command not found: <command>`
- `timeout_sec <= 0`：`timeout_sec must be positive`
- 命令超时：`speech command timeout after Ns`
- 命令返回非 0：优先使用 stderr，其次 stdout，否则使用 exit code

当 `required: false` 时，命令失败不会中断 mission：

```text
TaskExecutionResult(True, "voice report skipped: ...", report)
```

当 `required: true` 时，命令失败会让任务失败：

```text
TaskExecutionResult(False, "voice report failed: ...", report)
```

mock backend 始终返回成功。

## Quick Checks

mock 默认链路：

```bash
ros2 run venom_mission_commander mission_commander \
  --ros-args \
  -p mission_config:=/home/venom/venom_ws/src/venom_vnv/venom_mission_commander/config/simple_mission.yaml
```

如果本机安装了 `spd-say`，可以在 YAML 中配置 `backend: command` 做临时语音测试；如果只是验证 command backend 的成功路径，也可以临时用 `command: "true"`。
