# 比赛任务 1-4 启动命令

本文按比赛时任务 1、2、3、4 的顺序整理单点验证启动命令。

所有终端默认先执行：

```bash
cd "$HOME/venom_ws"
source install/setup.bash
```

Piper 机械臂当前按 `can1` 启动。比赛前先跑预检脚本，它会直接按接口名配置
`can1` 为 1 Mbps，并启用 bus-off 自动恢复 `restart-ms 100`：

```bash
$HOME/venom_ws/src/venom_vnv/manipulation/piper_mtc_tasks/scripts/piper_can_preflight.sh can1
```

确认配置：

```bash
ip -details link show can1 | grep -E 'bitrate|restart-ms'
```

## 任务 1：抓取 / 双目标装载

启动 D435i、抓取 YOLO、目标融合、Piper 控制、MoveIt/MTC 和
`/manipulation/execute_task`：

```bash
ros2 launch grasp_target_fusion real_pick_vision.launch.py \
  can_port:=can1 \
  launch_flame_tracking:=false \
  launch_color_box_detector:=false \
  launch_classification_yolo_detector:=false \
  launch_classification_yolo_bridge:=false \
  launch_repeat_visual_pick:=false \
  target_class:=black_block \
  yolo_allowed_classes:=black_block,golden_block \
  yolo_min_confidence:=0.7
```

如果抓取模型不是默认 `block_best.pt`，在上面命令后追加：

```bash
yolo_model_path:=$HOME/venom_ws/models/yolo/<your_pick_model>.pt
```

运行任务 1 verify：

```bash
ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=false \
  mock_nav_delay_sec:=0.0 \
  mission_config:=$HOME/venom_ws/src/venom_vnv/venom_mission_commander/config/verify_point1_grasp.yaml
```

## 任务 2：读表 + 回传 + 语音播报

如果还没有启动相机，先启动 D435i：

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera \
  camera_name:=d435i \
  enable_color:=true \
  enable_depth:=false \
  rgb_camera.color_profile:=640x480x15
```

启动 digit YOLO：

```bash
ros2 launch yolo_detector yolo_detector.launch.py \
  model_path:=$HOME/venom_ws/models/yolo/yolo_26_detect_digit.pt \
  image_topic:=/camera/d435i/color/image_raw \
  output_topic:=/perception/digit_detections
```

启动 standalone reader service：

```bash
ros2 run printed_number_reader printed_number_reader_node --ros-args \
  --params-file $HOME/venom_ws/src/venom_vnv/perception/printed_number_reader/config/printed_number_reader.yaml \
  -p reader_mode:=yolo \
  -p service_name:=/perception/read_printed_number \
  -p detections_topic:=/perception/digit_detections \
  -p image_topic:=/perception/debug/yolo_result \
  -p expected_digits:=4 \
  -p min_confidence:=0.25 \
  -p save_success_image:=true \
  -p success_image_dir:=/tmp/venom_meter_images
```

可选：先手动确认读表和播报：

```bash
ros2 service call /perception/read_printed_number printed_number_interfaces/srv/ReadPrintedNumber \
  "{target_id: competition_meter, timeout_sec: 3.0, expected_digits: 4, min_confidence: 0.25}"

$HOME/venom_ws/scripts/speak_meter_wav.sh "电表 competition_meter 读数 1234"
```

运行任务 2 完整链路 verify：

```bash
mkdir -p /tmp/venom_host_reports
ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=false \
  mock_nav_delay_sec:=0.0 \
  mission_config:=$HOME/venom_ws/src/venom_vnv/venom_mission_commander/config/verify_point2_meter_host_voice.yaml
```

如果只需要读表 + 播报，不需要图像回传，把 mission config 换成：

```bash
mission_config:=$HOME/venom_ws/src/venom_vnv/venom_mission_commander/config/verify_point2_meter_voice.yaml
```

## 任务 3：火焰识别 / 追踪

启动 D435i、Piper joint state/command、flame detector 和 flame tracker：

```bash
ros2 launch grasp_target_fusion real_pick_vision.launch.py \
  can_port:=can1 \
  launch_yolo_detector:=false \
  launch_yolo_bridge:=false \
  launch_classification_yolo_detector:=false \
  launch_classification_yolo_bridge:=false \
  launch_flame_tracking:=true \
  flame_use_yolo:=true
```

运行任务 3 verify：

```bash
ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=false \
  mock_nav_delay_sec:=0.0 \
  mission_config:=$HOME/venom_ws/src/venom_vnv/venom_mission_commander/config/verify_point3_flame_tracking.yaml
```

## 任务 4：分类投放

启动 Piper 控制、MoveIt/MTC、颜色框检测、分类 YOLO 和目标融合：

```bash
ros2 launch grasp_target_fusion real_pick_vision.launch.py \
  can_port:=can1 \
  launch_yolo_detector:=false \
  launch_yolo_bridge:=false \
  launch_color_box_detector:=true \
  launch_flame_tracking:=false \
  launch_classification_yolo_detector:=true \
  launch_classification_yolo_bridge:=true \
  classification_yolo_model_path:=$HOME/venom_ws/models/yolo/<your_classify_model>.pt
```

运行任务 4 verify：

```bash
ros2 launch venom_mission_commander mission_commander.launch.py \
  use_nav:=false \
  mock_nav_delay_sec:=0.0 \
  mission_config:=$HOME/venom_ws/src/venom_vnv/venom_mission_commander/config/verify_point4_classify_place.yaml
```

注意：任务 4 当前需要显式传入 `classification_yolo_model_path`。

## 全任务比赛入口

Nav2 和语音设备 ready 后，使用整场 runtime 入口。它会一次性拉起相机、
Piper/MTC、读表 reader、火焰追踪和各 YOLO 节点；YOLO 节点默认不加载模型，
由 mission 在各任务前后自动启停：

```bash
ros2 launch venom_mission_commander competition_10x6_arm_runtime.launch.py \
  can_port:=can1 \
  use_nav:=true \
  use_sim_time:=false \
  classification_yolo_model_path:=$HOME/venom_ws/models/yolo/<your_classify_model>.pt
```
