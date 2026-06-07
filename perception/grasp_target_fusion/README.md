# grasp_target_fusion

当前实现主要提供三部分能力：

- `Detection2D + D435i aligned depth + camera_info + TF` 融合为标准 `GraspTarget`
- 通过 `/perception/pick/target_valid`、`/perception/classify/target_valid` 发布目标有效信号
- 通过 `real_pick_vision.launch.py` 统一带起 D435i、Piper、MoveIt/MTC 和手眼外参

## 当前输入接口

- 抓取检测输入：`/perception/pick/detections_2d`
- 抓取检测数组输入：`/perception/pick/detections_2d_array`
- 分类检测输入：`/perception/classify/detections_2d`
- 分类检测数组输入：`/perception/classify/detections_2d_array`
- 深度输入：`/camera/d435i/aligned_depth_to_color/image_raw`
- 内参输入：`/camera/d435i/color/camera_info`
- 相机 frame 默认：`handeye_d435i_color_optical_frame`

> 以上 D435i 话题和 frame 名称基于本地 `realsense2_camera` 包与现有 launch 核实后填写。

## 当前输出接口

- 抓取目标：`/perception/pick/grasp_target`
- 抓取有效信号：`/perception/pick/target_valid`
- 分类目标：`/perception/classify/grasp_target`
- 分类有效信号：`/perception/classify/target_valid`

## `real_pick_vision.launch.py` 默认值

- 真机 CAN 默认使用 `can_piper`
- 默认开启抓取 YOLO，默认关闭分类 YOLO
- 如果要启用分类 YOLO，必须显式传入 `classification_yolo_model_path:=/abs/path/to/model`
- 当前默认启用 `require_single_target=true`，视野里同类目标不止一个时会把目标标记为 invalid

## 无 YOLO 时的联调方式

可以先启动一个假的 2D 检测发布器：

```bash
ros2 run grasp_target_fusion fake_detection_publisher
```

默认它会持续往 `/perception/detections_2d` 发布一个中心位于 `320,240` 的小方块检测框。
同时也会往 `/perception/detections_2d_array` 发布只包含这一个检测框的数组消息。

如果你想直接喂给当前抓取链路，建议改成：

```bash
ros2 run grasp_target_fusion fake_detection_publisher --ros-args \
  -p topic:=/perception/pick/detections_2d
```

这样可以直接验证：

- `Detection2D -> GraspTarget`
- `/perception/pick/target_valid`
- `pick_place_server` 读取最新视觉目标

## 单目标约束

当前默认配置里 `require_single_target=true` 已经在融合节点里生效：

- 如果检测数组里恰好只有 1 个目标，则继续生成 `GraspTarget`
- 如果是 0 个目标或多于 1 个目标，则 `/perception/pick/target_valid` 或 `/perception/classify/target_valid` 会变为 `false`

如果上游暂时只有单条 `/perception/detections_2d`，融合节点仍可兼容，但严格的“单目标/多目标”判断需要同时提供检测数组输入。

## 当前限制

- 只支持单个小方块目标
- 不做多目标排序
- 不估计视觉 yaw，默认 `has_yaw=false`
- 现已提供基于棋盘格的 eye-in-hand 采样与求解工具

## 手眼标定入口

- `ros2 run grasp_target_fusion handeye_sample_collector`
- `ros2 run grasp_target_fusion handeye_solver`
