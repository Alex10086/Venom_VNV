import rclpy

from flame_arm_tracker.flame_arm_tracker_node import FlameArmTracker
from flame_arm_tracker import flame_yolo_detector
from flame_arm_tracker.flame_yolo_detector import FlameYoloDetector


def test_enable_service_uses_flame_tracker_private_name():
    rclpy.init()
    node = FlameArmTracker()
    try:
        assert node.enable_service.service_name == "/flame_arm_tracker/set_enabled"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_flame_yolo_model_path_expands_home_environment(monkeypatch):
    monkeypatch.setenv("HOME", "/home/venom")
    monkeypatch.setattr(
        flame_yolo_detector,
        "get_package_share_directory",
        lambda package_name: "/opt/ros/share/" + package_name,
    )

    detector = object.__new__(FlameYoloDetector)

    assert detector._resolve_model_path(
        "$HOME/venom_ws/models/yolo/exp.pt"
    ).as_posix() == "/home/venom/venom_ws/models/yolo/exp.pt"
    assert detector._resolve_model_path(
        "~/venom_ws/models/yolo/exp.pt"
    ).as_posix() == "/home/venom/venom_ws/models/yolo/exp.pt"
