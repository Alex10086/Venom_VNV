# printed_number_reader

`printed_number_reader` 是一个轻量 ROS 2 service 节点，用来把“纸上单行纯数字”读取成字符串。

它只负责数字读取，不负责任务编排：`mission_commander` 通过 service 调用它，底层识别方式可以先用 mock，后续切到 YOLO 数字检测。

## Package Layout

| Path | Role |
| --- | --- |
| `printed_number_reader/printed_number_reader_node.py` | service 节点实现 |
| `config/printed_number_reader.yaml` | 默认运行参数 |
| `launch/printed_number_reader.launch.py` | 启动入口 |
| `../printed_number_interfaces/srv/ReadPrintedNumber.srv` | service 接口定义 |

## Runtime Contract

### Service

默认提供：

| Direction | Name | Type |
| --- | --- | --- |
| provide | `/perception/read_printed_number` | `printed_number_interfaces/srv/ReadPrintedNumber` |

接口字段：

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

约定：

- `value` 必须是纯数字字符串，例如 `"1234"`。
- `expected_digits=0` 表示不限制位数。
- 失败时 `success=false`，`message` 给出原因，节点不应因为单次识别失败崩溃。
- `image_path` 只在 `reader_mode: yolo`、`save_success_image: true`、图像缓存能匹配本次检测 header 且图片成功写盘时填写；它指向本次稳定识别对应的成功图片。图片保存失败不会让 service 读数结果失败，此时 `success=true` 但 `image_path` 为空。

### Optional YOLO Input

YOLO 模式下订阅：

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| subscribe | `/perception/digit_detections` | `yolo_interfaces/msg/YoloDetections` | 每个框代表一个数字 |
| subscribe | `/perception/debug/yolo_result` | `sensor_msgs/msg/Image` | 推荐订阅 YOLO 带框图，用于保存成功识别图片 |

每个 detection 需要满足：

- `hypothesis.class_name` 是单字符数字：`"0"` ~ `"9"`
- `hypothesis.score >= min_confidence`
- `bbox.center_x` 可用于从左到右排序

## Modes

### `reader_mode: mock`

默认模式，不依赖摄像头和 YOLO，用来验证 service 和 mission 链路。

```yaml
reader_mode: mock
mock_value: "1234"
mock_confidence: 1.0
```

mock 返回前仍会校验：纯数字、期望位数、最低置信度。

### `reader_mode: yolo`

真实检测模式。节点缓存最近的 `YoloDetections`，service 被调用时在超时时间内寻找稳定结果：

```text
YoloDetections
→ 过滤低置信度框
→ 只保留 class_name 为 0~9 的框
→ 按 bbox.center_x 从左到右排序
→ 拼成 value
→ 校验 expected_digits / stable_frames
→ 只在 service 成功返回时保存稳定识别对应图片
→ 返回 service response
```

## Parameters

| Parameter | Default | Notes |
| --- | --- | --- |
| `service_name` | `/perception/read_printed_number` | service 名称 |
| `detections_topic` | `/perception/digit_detections` | YOLO 数字检测输入 |
| `image_topic` | `/perception/debug/yolo_result` | 成功图片输入，建议用 YOLO 带框图 |
| `reader_mode` | `mock` | 支持 `mock` / `yolo` |
| `mock_value` | `"1234"` | mock 模式返回值 |
| `mock_confidence` | `1.0` | mock 模式置信度 |
| `default_timeout_sec` | `3.0` | request 未指定超时时使用 |
| `min_confidence` | `0.7` | 默认最低置信度 |
| `expected_digits` | `4` | 默认期望数字位数 |
| `stable_frames` | `1` | 要求相同结果出现的最近帧数 |
| `max_detection_age_sec` | `1.0` | 缓存检测帧最大有效时长 |
| `max_image_age_sec` | `2.0` | 可保存图片最大缓存时长 |
| `poll_interval_sec` | `0.05` | service 等待检测结果的轮询间隔 |
| `save_success_image` | `true` | 是否保存成功识别图片 |
| `success_image_dir` | `/tmp/venom_meter_images` | 成功图片保存目录 |
| `success_image_encoding` | `bgr8` | `cv_bridge` 转换编码 |
| `success_image_wait_sec` | `0.5` | 等待同 header 图像到达的最长时间 |
| `image_cache_size` | `10` | 图像缓存帧数 |

成功图片命名示例：

```text
/tmp/venom_meter_images/meter_2_1234_success.jpg
/tmp/venom_meter_images/meter_2_latest_success.jpg
```

其中第一张是本次成功结果文件，第二张是便于人工查看的最新成功图片别名。

## Build

From workspace root:

```bash
cd ~/venom_ws
colcon build --packages-select printed_number_interfaces printed_number_reader
source install/setup.bash
```

## Run

默认 mock 模式：

```bash
ros2 launch printed_number_reader printed_number_reader.launch.py
```

临时切到 YOLO 模式：

```bash
ros2 run printed_number_reader printed_number_reader_node --ros-args \
  -p reader_mode:=yolo \
  -p detections_topic:=/perception/digit_detections \
  -p image_topic:=/perception/debug/yolo_result \
  -p success_image_dir:=/tmp/venom_meter_images \
  -p expected_digits:=4 \
  -p min_confidence:=0.7 \
  -p stable_frames:=1
```

## Verification

mock service 快速测试：

```bash
ros2 service call /perception/read_printed_number printed_number_interfaces/srv/ReadPrintedNumber \
"{target_id: meter_1, timeout_sec: 3.0, expected_digits: 4, min_confidence: 0.7}"
```

期望返回类似：

```text
success=True, value='1234', confidence=1.0, message='mock read ok'
```

## Notes

- 当前范围是“单行纯数字”读取，不处理小数点、单位、负号或多行文本。
- YOLO 模式假设画面中目标数字框已经由上游模型检测出来。
- 如果模型类别名不是 `"0"` ~ `"9"`，需要在 reader 中增加类别名映射。
- 如果现场背景复杂，优先通过 `expected_digits`、`min_confidence`、`stable_frames` 和 ROI/模型侧约束提高稳定性。
