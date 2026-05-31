# Task Plugin Integration Guide

本文档说明后续视觉、机械臂、语音、火焰追踪等任务模块如何接入 `venom_mission_commander`。

核心原则：`MissionCommander` 只负责编排流程，不直接绑定具体业务模块。真实任务模块应通过任务插件接入，保持主流程稳定：

```text
导航到路点
→ 根据 YAML 的 tasks 顺序执行任务插件
→ 插件内部调用真实 ROS service / action / topic
→ 插件返回 TaskExecutionResult
→ MissionCommander 决定继续、失败或结束
```

## 接入层次

推荐优先级如下：

1. **替换现有 mock 插件内部实现**：如果任务语义不变，例如 `detect_item` 仍然是识别物品，就直接把 `DetectItemTaskPlugin.execute()` 里的 mock 逻辑换成真实视觉接口。
2. **新增任务插件类型**：如果任务语义变了，例如新增 `scan_qr_code`、`open_valve`、`dock_charger`，就新增一个 `BaseTaskPlugin` 子类。
3. **不要改 `MissionCommander.run()` 主流程**：除非任务编排模型本身变了，比如需要并行任务、条件分支、恢复点持久化。

## 任务 YAML 格式

每个路点的 `tasks` 是任务列表，执行顺序就是 YAML 中的顺序：

```yaml
waypoints:
  - name: task_point_1_pick
    frame_id: map
    x: 1.2
    y: 0.8
    yaw: 0.5404
    tasks:
      - name: detect_item_at_point_1
        type: detect_item
        target: example_object
        timeout_sec: 5.0
        output_key: detected_item

      - name: grasp_item_at_point_1
        type: grasp_item
        source: detected_item
        timeout_sec: 10.0
```

字段约定：

- `name`：任务实例名，用于日志、状态记录和失败定位。
- `type`：任务插件类型，必须和插件类的 `task_type` 一致。
- 其他字段：全部进入 `TaskSpec.params`，插件自己解释。

`MissionLoader` 只保留两个保留字段：

```text
name
type
```

除此之外的所有字段都会成为插件参数，例如：

```yaml
target: example_object
timeout_sec: 5.0
output_key: detected_item
```

会变成：

```python
spec.params = {
    'target': 'example_object',
    'timeout_sec': 5.0,
    'output_key': 'detected_item',
}
```

## 插件接口格式

所有任务插件都继承 `BaseTaskPlugin`：

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

必须实现的内容：

- `task_type`：字符串，和 YAML 里的 `type` 对应。
- `execute(context, spec)`：执行任务，返回 `TaskExecutionResult`。

可选实现的内容：

- `configure(node)`：创建 service client、action client、publisher、subscriber、参数等。
- `cancel()`：后续接急停、暂停、超时时用于取消任务。

## 输入对象

插件执行时有两个输入：

```python
execute(context: TaskContext, spec: TaskSpec) -> TaskExecutionResult
```

### `TaskSpec`

`TaskSpec` 来自 YAML 中的一条 task：

```python
TaskSpec(
    name='detect_item_at_point_1',
    task_type='detect_item',
    params={
        'target': 'example_object',
        'timeout_sec': 5.0,
    },
)
```

插件通常从 `spec.params` 读取配置：

```python
target = str(spec.params.get('target', 'unknown_item'))
timeout_sec = float(spec.params.get('timeout_sec', 5.0))
```

### `TaskContext`

`TaskContext` 是运行时上下文：

```python
TaskContext(
    node=MissionCommander,
    mission_id='rmul_nav2_mission_commander',
    waypoint=WaypointSpec(...),
    waypoint_index=1,
    task_index=0,
    mission_manager=MissionManager,
    blackboard={},
)
```

常用字段：

- `context.node`：ROS node，用来创建 service/action client、打印日志、读取 clock。
- `context.waypoint`：当前路点信息，例如 `name/x/y/yaw/frame_id`。
- `context.waypoint_index`：当前路点索引。
- `context.task_index`：当前任务索引。
- `context.mission_manager`：保存状态或读取任务状态。
- `context.blackboard`：任务之间共享数据。

## 输出对象

插件必须返回 `TaskExecutionResult`：

```python
return TaskExecutionResult(
    success=True,
    message='detected item: box',
    data={
        'target': 'box',
        'confidence': 0.94,
    },
)
```

字段含义：

- `success`：任务是否成功。
- `message`：日志和失败原因。
- `data`：任务结果数据，会被 `MissionManager.save_state()` 记录到 `last_task_data`。

失败时返回：

```python
return TaskExecutionResult(False, 'missing grasp target: detected_item')
```

如果 YAML 中 `stop_on_task_failure: true`，任意任务返回 `success=False` 会导致整个 mission 进入 `FAILED`。

## 数据传递方式

`venom_mission_commander` 内部有三种数据传递方式。

### 1. YAML 参数传入插件

适合静态配置：目标类别、超时时间、服务名、输出 key、放置区域。

```yaml
- name: detect_item_at_point_1
  type: detect_item
  target: example_object
  timeout_sec: 5.0
  output_key: detected_item
```

插件读取：

```python
target = spec.params.get('target')
output_key = spec.params.get('output_key', 'detected_item')
```

### 2. `blackboard` 在任务之间传数据

适合运行时结果：识别结果、抓取目标、读表结果、火焰检测框。

```python
context.blackboard['detected_item'] = detection
```

后续任务读取：

```python
target = context.blackboard.get('detected_item')
```

### 3. `TaskExecutionResult.data` 给状态机记录结果

适合最终状态记录和调试 summary：

```python
return TaskExecutionResult(True, 'grasp succeeded', grasped_object)
```

`WaypointTaskRunner` 会把结果保存成：

```text
last_task_name
last_task_type
last_task_success
last_task_message
last_task_data
```

## 推荐 blackboard key

为方便多个模块协作，建议先统一这些 key。

| Key | 写入者 | 读取者 | 推荐内容 |
| --- | --- | --- | --- |
| `detected_item` | `detect_item` | `grasp_item` | `{target, confidence, pose_frame, pose_hint 或 pose}` |
| `grasped_object` | `grasp_item` | `classify_place` | `{source, object, gripper}` |
| `meter_reading` | `read_meter` | `voice_report`, `host_report` | `{meter_id, value, confidence, source, image_path?}` |
| `last_host_report` | `host_report` | 调试/状态记录 | `{success, backend, report_kind, report_id, report_dir, metadata_path, receipt_path, image_path, image_sha256, message, duration_sec}` |
| `last_voice_report` | `voice_report` | 调试/状态记录 | `{text, success, source, message, duration_sec}` |
| `flame_detection` | `detect_flame` | `track_flame` | `{class, confidence, bbox 或 pose}` |
| `last_flame_tracking` | `track_flame` | 调试/状态记录 | `{backend, mode, service_name, status_topic, success, message, ready_status?, ready_wait_error?, stop_after_start_failure_success?}` |
| `flame_tracking_active` | `track_flame` | `grasp_item`, `classify_place`, mission shutdown | `true/false`，表示 `/flame_arm_tracker/set_enabled` 是否可能仍处于 active |
| `flame_tracking_state` | `track_flame`, mission cleanup | `grasp_item`, `classify_place`, mission shutdown | `inactive/starting/active/stopping/unknown`；`unknown` 必须先停用确认，不能直接执行 MoveIt/MTC action |
| `flame_tracking_ready` | `track_flame` | 调试/状态记录 | `true/false`，表示最近一次 `mode: start` 是否等到了 tracker ready 状态 |
| `last_placement` | `classify_place` | 调试/状态记录 | `{source, category, place_zone, object}` |

如果真实模块需要传 `PoseStamped`、图像框、点云目标等 ROS message，建议在 `blackboard` 里放可直接使用的 Python 对象，或者放一个可序列化 dict。长期建议优先用可序列化 dict，方便最终 mission summary、日志和恢复。

## CRAIC 机械臂接入约定

针对 CRAIC 2026 的三类机械臂任务，推荐直接复用当前仓库已经存在的能力边界：

- `grasp_item` → `backend: action` → `/manipulation/execute_task` → `PICK_AND_PLACE_LATEST_TARGET`
- `classify_place` → `backend: action` → `/manipulation/execute_task` → `CLASSIFY_PLATFORM_TO_COLOR_BOXES`
- `detect_flame` → `backend: topic` → `/perception/detections_2d_array` → 产出 `flame_detection`
- `track_flame` → `backend: service` → `/flame_arm_tracker/set_enabled` → 只做对准追踪，不做夹取

`track_flame` 支持三种 service 模式：

- `mode: start`：调用 `SetBool(True)` 开启追踪，`flame_arm_tracker` 在后续导航过程中独立持续对准火焰；移动追踪场景建议设置 `require_detection: false`，因为实时检测由 tracker 自己订阅 `/perception/detections_2d_array`。如果设置 `wait_until_tracking: true`，插件会继续等待 `/flame_arm_tracker/status` 表示 tracker 已进入 ready 状态后才返回成功。启动失败、service 超时或 ready 超时都会进入 `flame_tracking_state: unknown` 并立刻尝试强制关闭。
- `mode: stop`：调用 `SetBool(False)` 关闭追踪；后续 `grasp_item` / `classify_place` action 前会强制调用一次关闭，即使本地 blackboard 没有 active 标记也不直接相信本地状态，避免多个模块同时控制机械臂。
- `mode: hold`：保留“开启→等待 `tracking_duration_sec`→按 `disable_on_exit` 关闭”的停车短时追踪语义。

实赛配置必须统一火焰类别名：mission YAML、检测器输出和 `flame_arm_tracker` 的 `target_class_name` 应一致。当前 CRAIC arm mission 使用 `fire`，因此启动 tracker 时也应显式使用 `target_class_name:=fire`。

移动中追踪建议使用下面的 `start` 参数组合：

```yaml
- name: start_flame_tracking_for_moving_segment
  type: track_flame
  backend: service
  mode: start
  service_name: /flame_arm_tracker/set_enabled
  status_topic: /flame_arm_tracker/status
  wait_until_tracking: true
  require_target_acquired: true
  ready_timeout_sec: 4.0
  target_class: fire
  require_detection: false
```

`wait_until_tracking: true` 的 ready 条件来自 `venom_manipulation_interfaces/msg/FlameTrackerStatus`，当前要求：

- `enabled == true`
- `mode == "tracking"`
- `target_class_name == target_class`
- `joint_state_ok == true`
- `camera_info_ok == true`
- `command_output_ok == true`
- 如果 `require_target_acquired: true`，还要求 `target_acquired == true`

这样 `SetBool(True)` 不再被误认为“已经对准”。只有 tracker 完成 observe 过渡、具备相机/关节/命令输出条件，并且按配置确认目标后，mission 才继续进入下一段导航。

`FlameTrackerStatus` 当前字段约定如下：

```text
std_msgs/Header header
bool enabled
string mode
bool target_acquired
float32 target_age_sec
float32 yaw_error_rad
float32 pitch_error_rad
bool joint_state_ok
bool camera_info_ok
bool command_output_ok
bool command_saturated
string target_class_name
string message
```

`target_age_sec` 在没有目标时为 `-1.0`。`message` 用于记录当前等待原因或最近状态，例如 waiting joint state、waiting detection、tracking 等。

### 启动预检

`StartupChecker` 会根据 mission YAML 中的真实后端做最小预检：

- `track_flame` 的 `backend: service` / `backend: tracker` 需要能在 ROS graph 中看到 `/flame_arm_tracker/set_enabled`。
- `track_flame` 还需要能看到 `status_topic`，默认 `/flame_arm_tracker/status`。
- `detect_flame` 的 `backend: topic` 需要能看到检测 topic，默认 `/perception/detections_2d_array`。

预检只检查接口是否已在 ROS graph 中出现，不保证 tracker 已经 ready 或目标已对准；ready 和目标获取仍由 `track_flame mode: start` 的 status wait 负责。

这样 mission YAML 只负责编排“到点→识别/抓取→开启持续追踪→导航中对准→关闭追踪→分类放置”，复杂的机械臂规划、视觉融合和火焰追踪细节都留在现有的 `piper_mtc_tasks` 与 `flame_arm_tracker` 里，`MissionCommander` 不需要知道底层实现。

## ROS 消息怎么传

`MissionCommander` 本身不规定固定 ROS message 类型。它和任务模块之间的边界是插件接口：

```text
MissionCommander
→ TaskPlugin.execute(context, spec)
→ 插件内部调用真实 ROS service/action/topic
→ 插件把结果写入 blackboard + TaskExecutionResult
```

也就是说，真实 ROS 消息由每个插件自己适配。

推荐接口形式：

| 任务 | 推荐 ROS 接口 | 请求/目标 | 响应/结果 |
| --- | --- | --- | --- |
| 物品识别 | Service 或 Action | `target`, `timeout_sec`, 可选相机 topic | `success`, `class`, `confidence`, `pose` |
| 机械臂夹取 | Action | `target_pose` 或 `object_id`, `gripper` | `success`, `error_code`, `final_state` |
| 电表识别 | Service 或 Action | `meter_id`, 可选图像 topic | `success`, `value`, `confidence`, 可选 `image_path` |
| 图像/读数回传 | 本地 file backend（简单）或 Service（扩展） | `reading_key`, `image_path_key`, 可选 `image_path` | `success`, `report_dir`, `metadata_path`, `receipt_path`, `image_path` |
| 语音播报 | 本地 command（简单）或 Service（扩展） | `text` | `success`, `message` |
| 火焰检测 | Service 或 Action | 可选图像 topic / target | `success`, `bbox`, `confidence` |
| 火焰追踪 | Service + Status Topic（当前）或 Action（长期） | 当前用 `/flame_arm_tracker/set_enabled` 的 `start/stop/hold`，并用 `/flame_arm_tracker/status` 等待 ready；长期可升级为带反馈 Action | 当前返回启停和 ready 状态；长期建议返回 `target_acquired`, `aligned`, `final_error` |
| 分类放置 | Action | `object`, `category`, `place_zone` | `success`, `placed_pose` |

如果现有模块已经有自己的消息类型，不需要为了 `venom_mission_commander` 改消息。只需要在插件里做适配：

```text
YAML/spec.params + blackboard
→ 转成真实模块的 Request/Goal
→ 等待真实模块返回
→ 转成 blackboard dict + TaskExecutionResult
```

## Service 接入示例

下面是伪代码，展示一个识别插件如何调用真实 service。

```python
import rclpy

from venom_mission_commander.models import TaskContext, TaskExecutionResult, TaskSpec
from venom_mission_commander.task_plugins import BaseTaskPlugin


class RealDetectItemTaskPlugin(BaseTaskPlugin):
    task_type = 'detect_item'

    def configure(self, node) -> None:
        super().configure(node)
        self.client = node.create_client(DetectItem, '/detect_item')

    def execute(self, context: TaskContext, spec: TaskSpec) -> TaskExecutionResult:
        target = str(spec.params.get('target', 'unknown_item'))
        timeout_sec = float(spec.params.get('timeout_sec', 5.0))
        output_key = str(spec.params.get('output_key', 'detected_item'))

        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            return TaskExecutionResult(False, 'detect service unavailable')

        request = DetectItem.Request()
        request.target = target

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(context.node, future, timeout_sec=timeout_sec)

        if not future.done() or future.result() is None:
            return TaskExecutionResult(False, f'detect timeout: {target}')

        response = future.result()
        if not response.success:
            return TaskExecutionResult(False, response.message)

        detection = {
            'target': target,
            'confidence': response.confidence,
            'pose': response.pose,
        }
        context.blackboard[output_key] = detection
        return TaskExecutionResult(True, f'detected item: {target}', detection)
```

注意事项：

- `execute()` 当前是同步接口，内部可以用 async service/action，但要等待结果后再返回。
- 超时必须返回 `TaskExecutionResult(False, ...)`，不要无限等。
- 如果需要新消息包，记得在 `package.xml` 增加对应 `exec_depend`。

## Action 接入示例

机械臂、火焰追踪这类耗时任务更适合 Action。

```python
import rclpy
from rclpy.action import ActionClient

from venom_mission_commander.models import TaskExecutionResult
from venom_mission_commander.task_plugins import BaseTaskPlugin


class RealGraspItemTaskPlugin(BaseTaskPlugin):
    task_type = 'grasp_item'

    def configure(self, node) -> None:
        super().configure(node)
        self.action_client = ActionClient(node, GraspObject, '/grasp_object')

    def execute(self, context, spec) -> TaskExecutionResult:
        source_key = str(spec.params.get('source', 'detected_item'))
        timeout_sec = float(spec.params.get('timeout_sec', 10.0))
        target = context.blackboard.get(source_key)
        if target is None:
            return TaskExecutionResult(False, f'missing grasp target: {source_key}')

        if not self.action_client.wait_for_server(timeout_sec=timeout_sec):
            return TaskExecutionResult(False, 'grasp action server unavailable')

        goal = GraspObject.Goal()
        goal.target_pose = target['pose']
        goal.gripper = str(spec.params.get('gripper', 'piper'))

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(context.node, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return TaskExecutionResult(False, 'grasp goal rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(context.node, result_future, timeout_sec=timeout_sec)
        if not result_future.done() or result_future.result() is None:
            return TaskExecutionResult(False, 'grasp action timeout')

        result = result_future.result().result
        if not result.success:
            return TaskExecutionResult(False, result.message)

        grasped_object = {'source': source_key, 'object': target, 'gripper': goal.gripper}
        context.blackboard['grasped_object'] = grasped_object
        return TaskExecutionResult(True, 'grasp succeeded', grasped_object)
```

## 注册插件

当前最简单的注册方式是在 `TaskPluginRegistry.register_default_plugins()` 中加入插件实例：

```python
def register_default_plugins(self, node) -> None:
    for plugin in [
        DetectItemTaskPlugin(),
        GraspItemTaskPlugin(),
        RealDetectItemTaskPlugin(),
    ]:
        plugin.configure(node)
        self.register(plugin)
```

如果新插件使用同一个 `task_type`，后注册的插件会覆盖先注册的插件。因此真实接入时可以用真实插件替换 mock 插件，但要有意识地控制注册顺序。

推荐后续演进成按参数选择：

```text
task_backend:=mock
task_backend:=real
```

第一版可以先直接改注册表，保持实现简单。

## 失败语义

插件失败有两种方式：

1. 返回 `TaskExecutionResult(False, 'reason')`
2. 抛异常

`WaypointTaskRunner` 会捕获异常并转换成任务失败。

如果 mission YAML 中：

```yaml
mission:
  stop_on_task_failure: true
```

则任意任务失败都会导致：

```text
MissionManager.mark_task_failed()
→ MissionManager.fail()
→ mission state = failed
→ commander 退出码 = 1
```

如果设置为 `false`，任务失败会被记录，但会继续执行后续任务，最终可能进入 `COMPLETED_WITH_ERRORS`。

## 插件开发检查清单

新增或替换一个真实任务插件时，按下面顺序检查：

- 定义清楚 YAML `type`，例如 `detect_item`。
- 明确插件读取哪些 `spec.params`。
- 明确插件读取哪些 `blackboard` key。
- 明确插件写入哪些 `blackboard` key。
- 所有超时、服务不可用、goal rejected 都返回 `TaskExecutionResult(False, ...)`。
- 成功时返回 `TaskExecutionResult(True, message, data)`。
- 如果引入新 ROS message/service/action 包，更新 `package.xml`。
- 在 `TaskPluginRegistry.register_default_plugins()` 注册插件。
- 在 mission YAML 中增加或修改对应 task。
- 真实后端要能通过 startup preflight：关键 service/topic/status topic 已经在 ROS graph 中出现。
- 先用 mock 导航验证插件，再用 Nav2/Gazebo 验证完整流程。

## 建议保持稳定的边界

后续真实模块接入时，尽量保持这些边界不变：

```text
MissionCommander
→ 不关心具体视觉/机械臂/语音消息

TaskPlugin
→ 负责把 YAML 参数 + blackboard 转成真实 ROS 请求

真实任务模块
→ 只提供 service/action/topic，不直接控制 mission 流程
```

这样后续换比赛地图时主要改 mission YAML；换硬件或算法模块时主要改对应 task plugin；主流程代码可以保持稳定。
