"""Save the latest ROS image message to a stable file path."""

import time
from pathlib import Path


def normalize_save_every_n(value):
    """Return a positive frame-save interval."""
    try:
        interval = int(value)
    except (TypeError, ValueError):
        return 1
    return max(interval, 1)


def prepare_output_path(path):
    """Create the output parent directory and return a Path."""
    output_path = Path(path).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def should_save_frame(frame_count, save_every_n):
    """Return True when this frame should be written."""
    return frame_count % normalize_save_every_n(save_every_n) == 0


def temporary_image_path(output_path):
    """Build a collision-resistant temporary path with the same image suffix."""
    return output_path.with_name(
        f'.{output_path.stem}.{time.time_ns()}{output_path.suffix}'
    )


def main(args=None):
    """Run the image saver ROS node."""
    import cv2
    from cv_bridge import CvBridge
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    class LatestImageSaverNode(Node):
        """Subscribe to an image topic and atomically replace a file."""

        def __init__(self):
            super().__init__('latest_image_saver')
            self.declare_parameter('image_topic', '/image_raw')
            self.declare_parameter('output_path', '/tmp/venom_meter_images/latest.jpg')
            self.declare_parameter('desired_encoding', 'bgr8')
            self.declare_parameter('save_every_n', 1)
            self.declare_parameter('log_every_n', 30)

            self.image_topic = str(self.get_parameter('image_topic').value)
            self.output_path = prepare_output_path(
                self.get_parameter('output_path').value
            )
            self.desired_encoding = str(
                self.get_parameter('desired_encoding').value
            )
            self.save_every_n = normalize_save_every_n(
                self.get_parameter('save_every_n').value
            )
            self.log_every_n = normalize_save_every_n(
                self.get_parameter('log_every_n').value
            )
            self.frame_count = 0
            self.bridge = CvBridge()
            self.subscription = self.create_subscription(
                Image,
                self.image_topic,
                self._on_image,
                10,
            )

            self.get_logger().info(
                '[IMAGE SAVER] Saving %s to %s every %d frame(s)'
                % (self.image_topic, self.output_path, self.save_every_n)
            )

        def _on_image(self, msg):
            self.frame_count += 1
            if not should_save_frame(self.frame_count, self.save_every_n):
                return

            tmp_path = temporary_image_path(self.output_path)
            try:
                image = self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding=self.desired_encoding,
                )
                if not cv2.imwrite(str(tmp_path), image):
                    raise RuntimeError(f'cv2.imwrite failed: {tmp_path}')
                tmp_path.replace(self.output_path)
            except Exception as exc:  # keep the subscriber alive on bad frames/IO
                tmp_path.unlink(missing_ok=True)
                self.get_logger().warning(
                    f'[IMAGE SAVER] Failed to save image: {exc}'
                )
                return

            if should_save_frame(self.frame_count, self.log_every_n):
                self.get_logger().info(
                    f'[IMAGE SAVER] Saved latest image: {self.output_path}'
                )

    rclpy.init(args=args)
    node = LatestImageSaverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
