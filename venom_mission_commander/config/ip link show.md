ip link show

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
ip -details link show can1

cd "$HOME/venom_ws"
source install/setup.bash

ros2 launch grasp_target_fusion real_pick_vision.launch.py \
  can_port:=can1 \
  launch_piper_control:=true \
  launch_moveit_stack:=true \
  launch_pick_yolo_detector:=true \
  launch_pick_yolo_bridge:=true \
  launch_classification_yolo_detector:=true \
  launch_classification_yolo_bridge:=true \
  launch_color_box_detector:=false \
  launch_flame_tracking:=true \
  flame_use_yolo:=true \
  yolo_model_path:=$HOME/venom_ws/src/venom_vnv/perception/grasp_target_fusion/models/block_best.pt \
  yolo_allowed_classes:=black_block,golden_block \
  yolo_min_confidence:=0.3 \
  classification_yolo_model_path:=$HOME/venom_ws/src/venom_vnv/perception/grasp_target_fusion/models/box_best.pt
