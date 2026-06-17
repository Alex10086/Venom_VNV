import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class OdomPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__("odom_path_publisher")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/robot_path")
        self.declare_parameter("min_distance", 0.03)
        self.declare_parameter("max_poses", 5000)

        self.min_distance = float(self.get_parameter("min_distance").value)
        self.max_poses = int(self.get_parameter("max_poses").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, path_topic, path_qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)

        self.path = Path()
        self.last_x = None
        self.last_y = None

        self.get_logger().info(
            f"Publishing robot path from {odom_topic} to {path_topic} "
            f"(min_distance={self.min_distance}, max_poses={self.max_poses})"
        )

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is not None and self.last_y is not None:
            distance = math.hypot(x - self.last_x, y - self.last_y)
            if distance < self.min_distance:
                return

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header = msg.header
        self.path.poses.append(pose)
        if self.max_poses > 0 and len(self.path.poses) > self.max_poses:
            self.path.poses = self.path.poses[-self.max_poses :]

        self.last_x = x
        self.last_y = y
        self.path_pub.publish(self.path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomPathPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

