import rclpy
from rclpy.parameter import Parameter
from venom_manipulation_interfaces.msg import Detection2D, Detection2DArray

from grasp_target_fusion.grasp_target_fusion_node import GraspTargetFusionNode


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


def make_matching_detection():
    detection = Detection2D()
    detection.class_name = "black_box"
    detection.confidence = 0.9
    return detection


def test_empty_array_honors_target_loss_grace_period():
    rclpy.init()
    node = GraspTargetFusionNode(
        parameter_overrides=[
            Parameter("target_class", value="black_box"),
            Parameter("target_loss_grace_sec", value=0.75),
        ]
    )
    publisher = CapturingPublisher()
    node.target_valid_publisher = publisher
    try:
        node.detection_array_callback(
            Detection2DArray(detections=[make_matching_detection()])
        )
        publisher.messages.clear()

        node.detection_array_callback(Detection2DArray())

        assert node.latest_detection is not None
        assert publisher.messages == []

        node.latest_matching_detection_received_time = (
            node.get_clock().now() - rclpy.duration.Duration(seconds=0.76)
        )
        node.refresh_target_status()

        assert node.latest_detection is None
        assert publisher.messages == [False]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_silent_upstream_expires_target_after_grace_period():
    rclpy.init()
    node = GraspTargetFusionNode(
        parameter_overrides=[
            Parameter("target_class", value="black_box"),
            Parameter("target_loss_grace_sec", value=0.75),
        ]
    )
    publisher = CapturingPublisher()
    node.target_valid_publisher = publisher
    try:
        node.detection_array_callback(
            Detection2DArray(detections=[make_matching_detection()])
        )
        publisher.messages.clear()

        assert node.target_visible_in_current_frame is True
        node.latest_matching_detection_received_time = (
            node.get_clock().now() - rclpy.duration.Duration(seconds=0.76)
        )
        node.refresh_target_status()

        assert node.latest_detection is None
        assert publisher.messages == [False]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_class_change_clears_existing_target_immediately():
    rclpy.init()
    node = GraspTargetFusionNode(
        parameter_overrides=[
            Parameter("target_class", value="black_box"),
            Parameter("target_loss_grace_sec", value=0.75),
        ]
    )
    publisher = CapturingPublisher()
    node.target_valid_publisher = publisher
    try:
        node.detection_array_callback(
            Detection2DArray(detections=[make_matching_detection()])
        )
        publisher.messages.clear()

        results = node.set_parameters([Parameter("target_class", value="golden_box")])

        assert results[0].successful is True
        assert node.latest_detection is None
        assert node.latest_detection_count == 0
        assert node.latest_detection_stamp is None
        assert node.latest_matching_detection_received_time is None
        assert node.target_visible_in_current_frame is False
        assert publisher.messages == [False]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_single_detection_subscription_can_be_disabled():
    rclpy.init()
    node = GraspTargetFusionNode(
        parameter_overrides=[
            Parameter("subscribe_single_detection_topic", value=False),
        ]
    )
    try:
        assert node.single_detection_subscription is None
        assert node.detection_array_subscription is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_single_detection_subscription_defaults_enabled():
    rclpy.init()
    node = GraspTargetFusionNode()
    try:
        assert node.single_detection_subscription is not None
        assert node.detection_array_subscription is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()
