# flame_arm_tracker Launch Guide

本文档记录 CRAIC 机械臂火焰追踪节点的启动约定。它要和 `venom_mission_commander/config/competition_10x6_arm_mission.yaml` 中的 `track_flame` 任务保持一致。

## 基本启动

默认使用 YOLO/OpenVINO 火焰检测器，并启动机械臂追踪节点：

```bash
ros2 launch flame_arm_tracker flame_tracking.launch.py use_yolo:=true
```

使用自定义参数文件：

```bash
ros2 launch flame_arm_tracker flame_tracking.launch.py \
  use_yolo:=true \
  params_file:=/path/to/flame_tracking.yaml
```

默认参数文件是 `config/flame_tracking.yaml`。

## Mission Commander 依赖的接口

`competition_10x6_arm_mission.yaml` 会在二号点开启持续追踪，并在三号点关闭追踪。启动 tracker 前需要确认这些接口已存在：

| 接口 | 类型 | 作用 |
| --- | --- | --- |
| `/flame_arm_tracker/set_enabled` | `std_srvs/srv/SetBool` | `track_flame mode: start/stop` 启停追踪 |
| `/flame_arm_tracker/status` | `venom_manipulation_interfaces/msg/FlameTrackerStatus` | mission 等待 tracker ready |
| `/perception/detections_2d_array` | `venom_manipulation_interfaces/msg/Detection2DArray` | 火焰检测框输入 |
| `/camera/d435i/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 计算图像中心和角度误差 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 获取当前机械臂关节状态 |
| `/joint_command` | `sensor_msgs/msg/JointState` | 输出机械臂追踪命令 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 移动底盘速度输入，用于移动追踪补偿 |

快速检查：

```bash
ros2 service list | grep /flame_arm_tracker/set_enabled
ros2 topic list | grep /flame_arm_tracker/status
ros2 topic list | grep /perception/detections_2d_array
```

查看 tracker 状态：

```bash
ros2 topic echo /flame_arm_tracker/status
```

手动启停追踪：

```bash
ros2 service call /flame_arm_tracker/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /flame_arm_tracker/set_enabled std_srvs/srv/SetBool "{data: false}"
```

## 火焰类别名约定

当前 CRAIC arm mission 使用 `target_class: fire`。因此 tracker 和检测器也必须使用同一个类别名：

```yaml
flame_yolo_detector:
  ros__parameters:
    class_names:
      - fire

flame_arm_tracker:
  ros__parameters:
    target_class_name: fire
```

`use_yolo:=true` 时默认配置已经满足这个约定。若改用 `use_yolo:=false` 的 HSV 检测器，需要同步修改 `flame_color_detector.class_name` 和 `flame_arm_tracker.target_class_name`，不要让一个发布 `flame_picture`、另一个等待 `fire`。

## Ready 状态语义

Mission Commander 中建议的启动任务是：

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

`wait_until_tracking: true` 不只检查 `SetBool(True)` 成功，还会等待 `/flame_arm_tracker/status` 满足：

- `enabled == true`
- `mode == "tracking"`
- `target_class_name == "fire"`
- `joint_state_ok == true`
- `camera_info_ok == true`
- `command_output_ok == true`
- `target_acquired == true`（当 `require_target_acquired: true` 时）

这样车辆开始移动前，mission 可以确认 tracker 已完成 observe 过渡并具备基本闭环条件。

## 联调顺序建议

1. 启动相机、机械臂驱动、`/joint_states`、`/joint_command` 桥接和底盘 `/cmd_vel`。
2. 启动 `flame_tracking.launch.py`。
3. 确认 `/flame_arm_tracker/status` 中 `joint_state_ok`、`camera_info_ok`、`command_output_ok` 为 `true`。
4. 用手动 service call 测试 `enabled`、`mode` 和 `target_acquired` 是否按预期变化。
5. 再启动 `mission_commander` 的 CRAIC arm mission。

如果 mission startup preflight 失败，先修缺失的 service/topic；不要先绕过预检进入实车任务。
