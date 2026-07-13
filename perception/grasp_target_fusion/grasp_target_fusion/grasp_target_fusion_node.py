import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PointStamped, Vector3
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from venom_manipulation_interfaces.msg import Detection2D, Detection2DArray, GraspTarget


class GraspTargetFusionNode(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__("grasp_target_fusion", parameter_overrides=parameter_overrides)

        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("aligned_depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("detection_topic", "/perception/detections_2d")
        self.declare_parameter("detection_array_topic", "/perception/detections_2d_array")
        self.declare_parameter("subscribe_single_detection_topic", True)
        self.declare_parameter("grasp_target_topic", "/perception/grasp_target")
        self.declare_parameter("target_valid_topic", "/perception/target_valid")
        self.declare_parameter("debug_topic", "/perception/grasp_target_debug")
        self.declare_parameter("min_confidence", 0.7)
        self.declare_parameter("target_class", "block")
        self.declare_parameter("use_detection_header_stamp", False)
        self.declare_parameter("require_single_target", True)
        self.declare_parameter("default_target_size_xyz", [0.045, 0.045, 0.06])
        self.declare_parameter("depth_roi_half_width_px", 2)
        self.declare_parameter("max_detection_age_sec", 0.5)
        self.declare_parameter("target_loss_grace_sec", 0.0)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("target_center_offset_px", [0.0, 0.0])
        self.declare_parameter("target_center_offset_scale_xy", [0.0, 0.0])
        self.declare_parameter("use_bbox_depth_sample", False)
        self.declare_parameter("bbox_depth_sample_percentile", 50.0)
        self.declare_parameter("bbox_depth_sample_region_scale_xy", [1.0, 1.0])
        self.declare_parameter("bbox_depth_sample_center_offset_scale_xy", [0.0, 0.0])
        self.declare_parameter("target_ray_depth_offset_m", 0.0)
        self.declare_parameter("camera_target_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("front_center_base_x_bias_m", 0.0)
        self.declare_parameter("front_center_max_abs_y_m", 0.0)
        self.declare_parameter("front_center_min_x_m", 0.0)
        self.declare_parameter("front_center_max_x_m", 10.0)

        self.planning_frame = self.get_parameter("planning_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.target_classes = set()
        self.target_class_label = "*"
        self.update_target_classes(str(self.get_parameter("target_class").value))
        self.use_detection_header_stamp = bool(
            self.get_parameter("use_detection_header_stamp").value
        )
        self.require_single_target = bool(self.get_parameter("require_single_target").value)
        self.subscribe_single_detection_topic = bool(
            self.get_parameter("subscribe_single_detection_topic").value
        )
        self.default_target_size = self.get_parameter("default_target_size_xyz").value
        self.depth_roi_half_width_px = int(self.get_parameter("depth_roi_half_width_px").value)
        self.max_detection_age_sec = float(self.get_parameter("max_detection_age_sec").value)
        self.target_loss_grace_sec = max(
            0.0, float(self.get_parameter("target_loss_grace_sec").value)
        )
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.target_center_offset_px = self.read_float_pair_parameter("target_center_offset_px")
        self.target_center_offset_scale_xy = self.read_float_pair_parameter(
            "target_center_offset_scale_xy"
        )
        self.use_bbox_depth_sample = bool(
            self.get_parameter("use_bbox_depth_sample").value
        )
        self.bbox_depth_sample_percentile = max(
            0.0,
            min(100.0, float(self.get_parameter("bbox_depth_sample_percentile").value)),
        )
        self.bbox_depth_sample_region_scale_xy = self.read_float_pair_parameter(
            "bbox_depth_sample_region_scale_xy"
        )
        self.bbox_depth_sample_center_offset_scale_xy = self.read_float_pair_parameter(
            "bbox_depth_sample_center_offset_scale_xy"
        )
        self.target_ray_depth_offset_m = float(
            self.get_parameter("target_ray_depth_offset_m").value
        )
        self.camera_target_offset_xyz = self.read_float_triplet_parameter(
            "camera_target_offset_xyz"
        )
        self.front_center_base_x_bias_m = float(
            self.get_parameter("front_center_base_x_bias_m").value
        )
        self.front_center_max_abs_y_m = max(
            0.0, float(self.get_parameter("front_center_max_abs_y_m").value)
        )
        self.front_center_min_x_m = float(
            self.get_parameter("front_center_min_x_m").value
        )
        self.front_center_max_x_m = float(
            self.get_parameter("front_center_max_x_m").value
        )

        self.camera_info: Optional[CameraInfo] = None
        self.depth_image: Optional[np.ndarray] = None
        self.depth_encoding: Optional[str] = None
        self.latest_detection: Optional[Detection2D] = None
        self.latest_detection_stamp = None
        self.latest_detection_count = 0
        self.latest_matching_detection_received_time = None
        self.target_visible_in_current_frame = False
        self._warned_camera_info_frame_mismatch = False
        self._warned_depth_frame_mismatch = False
        self._warned_detection_frame_mismatch = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.grasp_target_publisher = self.create_publisher(
            GraspTarget,
            self.get_parameter("grasp_target_topic").value,
            10,
        )
        self.target_valid_publisher = self.create_publisher(
            Bool,
            self.get_parameter("target_valid_topic").value,
            10,
        )
        self.debug_publisher = self.create_publisher(
            String,
            self.get_parameter("debug_topic").value,
            10,
        )

        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self.camera_info_callback,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("aligned_depth_topic").value,
            self.depth_callback,
            10,
        )
        self.single_detection_subscription = None
        if self.subscribe_single_detection_topic:
            self.single_detection_subscription = self.create_subscription(
                Detection2D,
                self.get_parameter("detection_topic").value,
                self.detection_callback,
                10,
            )
        self.detection_array_subscription = self.create_subscription(
            Detection2DArray,
            self.get_parameter("detection_array_topic").value,
            self.detection_array_callback,
            10,
        )
        self.status_timer = self.create_timer(0.1, self.refresh_target_status)
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == "target_class":
                self.update_target_classes(str(parameter.value))
                self.clear_latest_detection()
                self.publish_target_valid(False)
                self.get_logger().info(
                    "Updated grasp target class filter to '%s'" % self.target_class_label
                )
            elif parameter.name == "require_single_target":
                self.require_single_target = bool(parameter.value)
                self.get_logger().info(
                    "Updated require_single_target to %s" % self.require_single_target
                )
            elif parameter.name == "max_detection_age_sec":
                self.max_detection_age_sec = float(parameter.value)
                self.get_logger().info(
                    "Updated max_detection_age_sec to %.3f" % self.max_detection_age_sec
                )
            elif parameter.name == "target_loss_grace_sec":
                self.target_loss_grace_sec = max(0.0, float(parameter.value))
                self.get_logger().info(
                    "Updated target_loss_grace_sec to %.3f"
                    % self.target_loss_grace_sec
                )
            elif parameter.name == "min_confidence":
                self.min_confidence = float(parameter.value)
                self.get_logger().info(
                    "Updated min_confidence to %.3f" % self.min_confidence
                )
        return SetParametersResult(successful=True)

    def update_target_classes(self, target_class_param: str) -> None:
        self.target_classes = {
            name.strip().lower()
            for name in target_class_param.split(",")
            if name.strip()
        }
        self.target_class_label = ",".join(sorted(self.target_classes)) or "*"

    def camera_info_callback(self, message: CameraInfo) -> None:
        self.warn_if_frame_mismatch_once(
            message.header.frame_id,
            "camera_info",
            "_warned_camera_info_frame_mismatch",
        )
        self.camera_info = message

    def depth_callback(self, message: Image) -> None:
        self.warn_if_frame_mismatch_once(
            message.header.frame_id,
            "aligned_depth",
            "_warned_depth_frame_mismatch",
        )
        if message.encoding not in ("16UC1", "32FC1"):
            self.get_logger().warn(
                f"Unsupported depth encoding '{message.encoding}', expected 16UC1 or 32FC1"
            )
            return

        if message.encoding == "16UC1":
            depth = np.frombuffer(message.data, dtype=np.uint16).reshape(
                message.height,
                message.width,
            )
        else:
            depth = np.frombuffer(message.data, dtype=np.float32).reshape(
                message.height,
                message.width,
            )

        self.depth_image = depth.copy()
        self.depth_encoding = message.encoding

    def detection_callback(self, message: Detection2D) -> None:
        self.warn_if_frame_mismatch_once(
            message.header.frame_id,
            "detection",
            "_warned_detection_frame_mismatch",
        )
        if not self.detection_matches_target_class(message):
            self.get_logger().info(
                "Grasp target debug: ignoring single detection class %s not in %s"
                % (message.class_name.lower(), self.target_class_label)
            )
            return
        self.target_visible_in_current_frame = True
        self.latest_matching_detection_received_time = self.get_clock().now()
        self.latest_detection = message
        if (
            not self.use_detection_header_stamp
            or (message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0)
        ):
            self.latest_detection_stamp = self.get_clock().now()
        else:
            self.latest_detection_stamp = rclpy.time.Time.from_msg(message.header.stamp)
        if self.latest_detection_count == 0:
            self.latest_detection_count = 1
        self.try_publish_grasp_target()

    def detection_array_callback(self, message: Detection2DArray) -> None:
        matching_detections = [
            detection
            for detection in message.detections
            if self.detection_matches_target_class(detection)
        ]
        self.latest_detection_count = len(matching_detections)
        self.target_visible_in_current_frame = bool(matching_detections)
        if len(matching_detections) >= 1:
            selected = max(matching_detections, key=lambda detection: detection.confidence)
            self.detection_callback(selected)
        elif self.is_within_target_loss_grace():
            return
        else:
            self.clear_latest_detection()
            self.publish_target_valid(False)

    def try_publish_grasp_target(self) -> None:
        if not self.has_valid_detection():
            return

        target_time = self.latest_detection_stamp
        if target_time is None:
            target_time = self.get_clock().now()

        projection_center_x, projection_center_y = self.compute_projection_center(
            self.latest_detection
        )
        depth_m, depth_debug = self.lookup_depth_meters(self.latest_detection)
        if depth_m is None or not math.isfinite(depth_m) or depth_m <= 0.0:
            self.get_logger().info(
                "Grasp target debug: invalid depth at raw_px=(%.1f, %.1f) proj_px=(%.1f, %.1f), depth=%s %s"
                % (
                    self.latest_detection.center_x,
                    self.latest_detection.center_y,
                    projection_center_x,
                    projection_center_y,
                    "None" if depth_m is None else f"{depth_m:.4f}",
                    depth_debug,
                )
            )
            self.publish_target_valid(False)
            return

        projected_depth_m = depth_m + self.target_ray_depth_offset_m
        if not math.isfinite(projected_depth_m) or projected_depth_m <= 0.0:
            self.get_logger().warn(
                "Grasp target debug: target_ray_depth_offset_m=%.4f produced invalid projected depth %.4f"
                % (
                    self.target_ray_depth_offset_m,
                    projected_depth_m,
                )
            )
            self.publish_target_valid(False)
            return

        point_in_camera = self.project_to_camera(
            projection_center_x,
            projection_center_y,
            projected_depth_m,
        )
        if point_in_camera is None:
            self.get_logger().info(
                "Grasp target debug: project_to_camera failed at raw_px=(%.1f, %.1f) proj_px=(%.1f, %.1f) depth=%.4f projected_depth=%.4f"
                % (
                    self.latest_detection.center_x,
                    self.latest_detection.center_y,
                    projection_center_x,
                    projection_center_y,
                    depth_m,
                    projected_depth_m,
                )
            )
            self.publish_target_valid(False)
            return

        point_in_camera.point.x += self.camera_target_offset_xyz[0]
        point_in_camera.point.y += self.camera_target_offset_xyz[1]
        point_in_camera.point.z += self.camera_target_offset_xyz[2]
        if not math.isfinite(point_in_camera.point.z) or point_in_camera.point.z <= 0.0:
            self.get_logger().warn(
                "Grasp target debug: camera_target_offset_xyz=(%.4f, %.4f, %.4f) produced invalid camera z=%.4f"
                % (
                    self.camera_target_offset_xyz[0],
                    self.camera_target_offset_xyz[1],
                    self.camera_target_offset_xyz[2],
                    point_in_camera.point.z,
                )
            )
            self.publish_target_valid(False)
            return

        # When detections are stamped at receive time, robot_state_publisher can lag
        # slightly behind this node's clock. Use the latest available TF in that mode
        # to avoid future extrapolation while still freshness-checking by receive time.
        lookup_time = target_time if self.use_detection_header_stamp else rclpy.time.Time()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.camera_frame,
                lookup_time,
            )
        except TransformException as exc:
            self.get_logger().warn(f"Grasp target debug: TF lookup failed: {exc}")
            self.publish_target_valid(False)
            return

        point_in_base = do_transform_point(point_in_camera, transform)
        base_point_before_region_bias = (
            float(point_in_base.point.x),
            float(point_in_base.point.y),
            float(point_in_base.point.z),
        )
        applied_front_center_bias = False
        if (
            abs(point_in_base.point.y) <= self.front_center_max_abs_y_m
            and self.front_center_min_x_m <= point_in_base.point.x <= self.front_center_max_x_m
            and abs(self.front_center_base_x_bias_m) > 1e-9
        ):
            point_in_base.point.x += self.front_center_base_x_bias_m
            applied_front_center_bias = True

        debug_text = (
            "Grasp target debug: cls=%s conf=%.3f raw_px=(%.1f, %.1f) proj_px=(%.1f, %.1f) "
            "size_px=(%.1f, %.1f) depth=%.4f projected_depth=%.4f %s ray_depth_offset=%.4f "
            "cam_offset=(%.4f, %.4f, %.4f) front_center_x_bias=%.4f applied=%s "
            "cam=(%.4f, %.4f, %.4f) base_pre_bias=(%.4f, %.4f, %.4f) base=(%.4f, %.4f, %.4f)"
            % (
                self.latest_detection.class_name,
                self.latest_detection.confidence,
                self.latest_detection.center_x,
                self.latest_detection.center_y,
                projection_center_x,
                projection_center_y,
                self.latest_detection.size_x,
                self.latest_detection.size_y,
                depth_m,
                projected_depth_m,
                depth_debug,
                self.target_ray_depth_offset_m,
                self.camera_target_offset_xyz[0],
                self.camera_target_offset_xyz[1],
                self.camera_target_offset_xyz[2],
                self.front_center_base_x_bias_m,
                str(applied_front_center_bias).lower(),
                point_in_camera.point.x,
                point_in_camera.point.y,
                point_in_camera.point.z,
                base_point_before_region_bias[0],
                base_point_before_region_bias[1],
                base_point_before_region_bias[2],
                point_in_base.point.x,
                point_in_base.point.y,
                point_in_base.point.z,
            )
        )
        self.get_logger().info(debug_text)
        debug_message = String()
        debug_message.data = debug_text
        self.debug_publisher.publish(debug_message)

        target = GraspTarget()
        target.header.stamp = self.to_msg_time(target_time)
        target.header.frame_id = self.planning_frame
        target.class_name = self.latest_detection.class_name
        target.confidence = self.latest_detection.confidence
        target.pose.position = point_in_base.point
        target.pose.orientation.w = 1.0
        target.size = Vector3(
            x=float(self.default_target_size[0]),
            y=float(self.default_target_size[1]),
            z=float(self.default_target_size[2]),
        )
        target.has_yaw = False
        target.yaw = 0.0

        self.grasp_target_publisher.publish(target)
        self.publish_target_valid(True)

    def has_valid_detection(self) -> bool:
        if self.camera_info is None or self.depth_image is None or self.latest_detection is None:
            missing = []
            if self.camera_info is None:
                missing.append("camera_info")
            if self.depth_image is None:
                missing.append("depth_image")
            if self.latest_detection is None:
                missing.append("latest_detection")
            self.get_logger().info(
                "Grasp target debug: waiting for %s" % ",".join(missing)
            )
            self.publish_target_valid(False)
            return False

        if self.latest_detection.confidence < self.min_confidence:
            self.get_logger().info(
                "Grasp target debug: confidence too low %.3f < %.3f"
                % (self.latest_detection.confidence, self.min_confidence)
            )
            self.publish_target_valid(False)
            return False

        if not self.detection_matches_target_class(self.latest_detection):
            self.get_logger().info(
                "Grasp target debug: class mismatch %s not in %s"
                % (self.latest_detection.class_name.lower(), self.target_class_label)
            )
            self.publish_target_valid(False)
            return False

        if self.require_single_target and self.latest_detection_count != 1:
            self.get_logger().info(
                "Grasp target debug: target count invalid %d"
                % self.latest_detection_count
            )
            self.publish_target_valid(False)
            return False

        if self.latest_detection_stamp is None:
            self.get_logger().info("Grasp target debug: detection stamp missing")
            self.publish_target_valid(False)
            return False

        age = (self.get_clock().now() - self.latest_detection_stamp).nanoseconds / 1e9
        if age > self.max_detection_age_sec:
            self.get_logger().info(
                "Grasp target debug: detection too old %.3fs > %.3fs"
                % (age, self.max_detection_age_sec)
            )
            self.publish_target_valid(False)
            return False

        return True

    def detection_matches_target_class(self, detection: Detection2D) -> bool:
        if not self.target_classes:
            return True
        return detection.class_name.strip().lower() in self.target_classes

    def refresh_target_status(self) -> None:
        if self.target_loss_grace_sec > 0.0 and self.latest_detection is not None:
            if self.is_within_target_loss_grace():
                if not self.target_visible_in_current_frame:
                    return
            else:
                self.clear_latest_detection()
                self.publish_target_valid(False)
                return
        self.has_valid_detection()

    def is_within_target_loss_grace(self) -> bool:
        if (
            self.target_loss_grace_sec <= 0.0
            or self.latest_detection is None
            or self.latest_matching_detection_received_time is None
        ):
            return False
        elapsed = (
            self.get_clock().now() - self.latest_matching_detection_received_time
        ).nanoseconds / 1e9
        return elapsed <= self.target_loss_grace_sec

    def clear_latest_detection(self) -> None:
        self.latest_detection = None
        self.latest_detection_stamp = None
        self.latest_detection_count = 0
        self.latest_matching_detection_received_time = None
        self.target_visible_in_current_frame = False

    def compute_projection_center(self, detection: Detection2D) -> Tuple[float, float]:
        return (
            float(
                detection.center_x
                + self.target_center_offset_px[0]
                + detection.size_x * self.target_center_offset_scale_xy[0]
            ),
            float(
                detection.center_y
                + self.target_center_offset_px[1]
                + detection.size_y * self.target_center_offset_scale_xy[1]
            ),
        )

    def lookup_depth_meters(self, detection: Detection2D) -> Tuple[Optional[float], str]:
        if self.depth_image is None:
            return None, "depth_mode=none"

        if self.use_bbox_depth_sample:
            return self.lookup_depth_from_bbox(detection)
        return self.lookup_depth_from_center(detection.center_x, detection.center_y)

    def lookup_depth_from_center(
        self,
        center_x: float,
        center_y: float,
    ) -> Tuple[Optional[float], str]:
        x = int(round(center_x))
        y = int(round(center_y))
        if (
            x < 0
            or y < 0
            or x >= self.depth_image.shape[1]
            or y >= self.depth_image.shape[0]
        ):
            return None, "depth_mode=center_roi out_of_bounds"

        roi = self.depth_image[
            max(0, y - self.depth_roi_half_width_px): min(self.depth_image.shape[0], y + self.depth_roi_half_width_px + 1),
            max(0, x - self.depth_roi_half_width_px): min(self.depth_image.shape[1], x + self.depth_roi_half_width_px + 1),
        ]
        valid = roi[np.isfinite(roi) & (roi > 0)]
        if valid.size == 0:
            return None, (
                "depth_mode=center_roi center_px=(%.1f, %.1f) window=%dx%d empty"
                % (
                    center_x,
                    center_y,
                    roi.shape[1],
                    roi.shape[0],
                )
            )

        depth = float(np.median(valid))
        return self.convert_depth_value(depth), (
            "depth_mode=center_roi center_px=(%.1f, %.1f) window=%dx%d samples=%d"
            % (
                center_x,
                center_y,
                roi.shape[1],
                roi.shape[0],
                valid.size,
            )
        )

    def lookup_depth_from_bbox(self, detection: Detection2D) -> Tuple[Optional[float], str]:
        if self.depth_image is None:
            return None, "depth_mode=bbox_roi no_depth_image"

        sample_center_x = (
            detection.center_x
            + detection.size_x * self.bbox_depth_sample_center_offset_scale_xy[0]
        )
        sample_center_y = (
            detection.center_y
            + detection.size_y * self.bbox_depth_sample_center_offset_scale_xy[1]
        )
        roi_width = max(
            1,
            int(round(abs(detection.size_x) * self.bbox_depth_sample_region_scale_xy[0])),
        )
        roi_height = max(
            1,
            int(round(abs(detection.size_y) * self.bbox_depth_sample_region_scale_xy[1])),
        )

        x1 = int(round(sample_center_x - roi_width * 0.5))
        y1 = int(round(sample_center_y - roi_height * 0.5))
        x2 = int(round(sample_center_x + roi_width * 0.5))
        y2 = int(round(sample_center_y + roi_height * 0.5))

        x1 = max(0, min(self.depth_image.shape[1] - 1, x1))
        y1 = max(0, min(self.depth_image.shape[0] - 1, y1))
        x2 = max(x1 + 1, min(self.depth_image.shape[1], x2))
        y2 = max(y1 + 1, min(self.depth_image.shape[0], y2))

        roi = self.depth_image[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi) & (roi > 0)]
        if valid.size == 0:
            center_depth, center_debug = self.lookup_depth_from_center(
                detection.center_x,
                detection.center_y,
            )
            return center_depth, (
                "depth_mode=bbox_roi center_px=(%.1f, %.1f) roi=(%d:%d,%d:%d) empty fallback={%s}"
                % (
                    sample_center_x,
                    sample_center_y,
                    x1,
                    x2,
                    y1,
                    y2,
                    center_debug,
                )
            )

        depth = float(np.percentile(valid, self.bbox_depth_sample_percentile))
        return self.convert_depth_value(depth), (
            "depth_mode=bbox_roi center_px=(%.1f, %.1f) roi=(%d:%d,%d:%d) p=%.1f samples=%d"
            % (
                sample_center_x,
                sample_center_y,
                x1,
                x2,
                y1,
                y2,
                self.bbox_depth_sample_percentile,
                valid.size,
            )
        )

    def convert_depth_value(self, depth: float) -> float:
        if self.depth_encoding == "16UC1":
            return depth * self.depth_scale
        return depth

    def project_to_camera(self, center_x: float, center_y: float, depth_m: float) -> Optional[PointStamped]:
        if self.camera_info is None:
            return None

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        if fx <= 0.0 or fy <= 0.0:
            return None

        x = (center_x - cx) * depth_m / fx
        y = (center_y - cy) * depth_m / fy

        point_time = self.latest_detection_stamp
        if point_time is None:
            point_time = self.get_clock().now()

        point = PointStamped()
        point.header.stamp = self.to_msg_time(point_time)
        point.header.frame_id = self.camera_frame
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(depth_m)
        return point

    def warn_if_frame_mismatch_once(
        self,
        message_frame: str,
        source_name: str,
        warned_attr_name: str,
    ) -> None:
        if not message_frame or getattr(self, warned_attr_name):
            return
        if message_frame == self.camera_frame:
            return
        self.get_logger().warn(
            "Configured camera_frame is '%s', but %s is arriving in frame '%s'. "
            "This pipeline will continue to use the configured camera_frame."
            % (self.camera_frame, source_name, message_frame)
        )
        setattr(self, warned_attr_name, True)

    @staticmethod
    def to_msg_time(time_value: rclpy.time.Time) -> TimeMsg:
        return time_value.to_msg()

    def read_float_pair_parameter(self, name: str) -> Tuple[float, float]:
        values = self.get_parameter(name).value
        if len(values) != 2:
            raise ValueError(f"Parameter '{name}' must contain exactly 2 values")
        return float(values[0]), float(values[1])

    def read_float_triplet_parameter(self, name: str) -> Tuple[float, float, float]:
        values = self.get_parameter(name).value
        if len(values) != 3:
            raise ValueError(f"Parameter '{name}' must contain exactly 3 values")
        return float(values[0]), float(values[1]), float(values[2])

    def publish_target_valid(self, value: bool) -> None:
        message = Bool()
        message.data = value
        self.target_valid_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = GraspTargetFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
