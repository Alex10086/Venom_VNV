import rclpy

from flame_arm_tracker.flame_arm_tracker_node import FlameArmTracker


def test_enable_service_uses_flame_tracker_private_name():
    rclpy.init()
    node = FlameArmTracker()
    try:
        assert node.enable_service.service_name == "/flame_arm_tracker/set_enabled"
    finally:
        node.destroy_node()
        rclpy.shutdown()
