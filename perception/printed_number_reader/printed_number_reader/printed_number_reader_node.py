import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
import re
from threading import Lock

import rclpy
from printed_number_interfaces.srv import ReadPrintedNumber
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


@dataclass(frozen=True)
class CachedImage:
    received_at: float
    msg: object


class ImageCache:
    def __init__(self, maxlen=10):
        self._images = deque(maxlen=max(1, int(maxlen)))
        self._lock = Lock()

    def append(self, msg, received_at=None):
        stamp = time.monotonic() if received_at is None else float(received_at)
        with self._lock:
            self._images.append(CachedImage(stamp, msg))

    def matching(self, header, now_sec, max_image_age_sec):
        with self._lock:
            images = list(self._images)
        if not images:
            return None

        fresh = [
            image for image in images
            if max_image_age_sec <= 0.0 or now_sec - image.received_at <= max_image_age_sec
        ]
        if not fresh:
            return None

        target_stamp = header_stamp_ns(header)
        if target_stamp is None:
            return fresh[-1].msg

        exact = [
            image for image in fresh
            if header_stamp_ns(getattr(image.msg, 'header', None)) == target_stamp
        ]
        if exact:
            return exact[-1].msg
        return None


def header_stamp_ns(header):
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    return int(getattr(stamp, 'sec', 0)) * 1_000_000_000 + int(getattr(stamp, 'nanosec', 0))


def sanitize_filename_part(value, fallback):
    sanitized = re.sub(r'[^A-Za-z0-9_.-]+', '_', str(value)).strip('._')
    return sanitized or fallback


def save_success_image(
    *,
    image_cache,
    detection_header,
    output_dir,
    target_id,
    value,
    bridge,
    cv2_module,
    desired_encoding,
    now_sec,
    max_image_age_sec,
):
    image_msg = image_cache.matching(detection_header, now_sec, max_image_age_sec)
    if image_msg is None:
        return ''

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    safe_target = sanitize_filename_part(target_id, 'meter')
    safe_value = sanitize_filename_part(value, 'value')
    image_path = output_dir / f'{safe_target}_{safe_value}_success.jpg'
    latest_path = output_dir / f'{safe_target}_latest_success.jpg'
    temp_path = image_path.with_name(f'.{image_path.stem}.{time.time_ns()}{image_path.suffix}')

    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding=desired_encoding)
    if not cv2_module.imwrite(str(temp_path), image):
        temp_path.unlink(missing_ok=True)
        return ''
    temp_path.replace(image_path)
    latest_path.write_bytes(image_path.read_bytes())
    return str(image_path)


class PrintedNumberReader(Node):
    def __init__(self):
        super().__init__('printed_number_reader')
        self.declare_parameter('service_name', '/perception/read_printed_number')
        self.declare_parameter('detections_topic', '/perception/digit_detections')
        self.declare_parameter('image_topic', '/perception/debug/yolo_result')
        self.declare_parameter('reader_mode', 'mock')
        self.declare_parameter('mock_value', '12345')
        self.declare_parameter('mock_confidence', 1.0)
        self.declare_parameter('default_timeout_sec', 3.0)
        self.declare_parameter('min_confidence', 0.7)
        self.declare_parameter('expected_digits', 0)
        self.declare_parameter('stable_frames', 1)
        self.declare_parameter('max_detection_age_sec', 1.0)
        self.declare_parameter('max_image_age_sec', 2.0)
        self.declare_parameter('poll_interval_sec', 0.05)
        self.declare_parameter('save_success_image', True)
        self.declare_parameter('success_image_dir', '/tmp/venom_meter_images')
        self.declare_parameter('success_image_encoding', 'bgr8')
        self.declare_parameter('success_image_wait_sec', 0.5)
        self.declare_parameter('image_cache_size', 10)

        self._cb_group = ReentrantCallbackGroup()
        self._frames = deque(maxlen=max(1, int(self.get_parameter('stable_frames').value) * 5))
        self._frames_lock = Lock()
        self._image_cache = ImageCache(maxlen=int(self.get_parameter('image_cache_size').value))
        self._yolo_available = False
        self._image_saving_available = False
        self._bridge = None
        self._cv2 = None

        try:
            from yolo_interfaces.msg import YoloDetections
        except ImportError as exc:
            self.get_logger().warning(
                f'yolo_interfaces unavailable; yolo reader_mode will fail: {exc}'
            )
        else:
            self._yolo_available = True
            self.create_subscription(
                YoloDetections,
                str(self.get_parameter('detections_topic').value),
                self._on_detections,
                10,
                callback_group=self._cb_group,
            )
        self._configure_image_subscription()
        self.create_service(
            ReadPrintedNumber,
            str(self.get_parameter('service_name').value),
            self._handle_read,
            callback_group=self._cb_group,
        )

    def _on_detections(self, msg):
        with self._frames_lock:
            self._frames.append((time.monotonic(), msg))

    def _configure_image_subscription(self):
        if not bool(self.get_parameter('save_success_image').value):
            return
        try:
            import cv2
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image
        except ImportError as exc:
            self.get_logger().warning(
                f'image saving unavailable; install cv_bridge/sensor_msgs/opencv: {exc}'
            )
            return

        self._bridge = CvBridge()
        self._cv2 = cv2
        self._image_saving_available = True
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            10,
            callback_group=self._cb_group,
        )

    def _on_image(self, msg):
        self._image_cache.append(msg)

    def _handle_read(self, request, response):
        try:
            mode = str(self.get_parameter('reader_mode').value).strip().lower()
            if mode == 'mock':
                return self._handle_mock(request, response)
            if mode == 'yolo':
                return self._handle_yolo(request, response)
            return self._fail(response, f'invalid reader_mode: {mode}')
        except Exception as exc:  # keep service robust
            self.get_logger().warning(f'read request failed: {exc}')
            return self._fail(response, f'read failed: {exc}')

    def _handle_mock(self, request, response):
        value = str(self.get_parameter('mock_value').value)
        confidence = float(self.get_parameter('mock_confidence').value)
        return self._validate_and_fill(request, response, value, confidence, 'mock read ok')

    def _handle_yolo(self, request, response):
        if not self._yolo_available:
            return self._fail(response, 'yolo_interfaces unavailable; cannot use yolo reader_mode')

        timeout = float(request.timeout_sec) if request.timeout_sec > 0.0 else float(
            self.get_parameter('default_timeout_sec').value
        )
        deadline = time.monotonic() + max(timeout, 0.0)
        poll = max(float(self.get_parameter('poll_interval_sec').value), 0.01)

        last_error = 'no detection frame received'
        while time.monotonic() <= deadline and rclpy.ok():
            result = self._stable_result(request)
            if result[0]:
                value, confidence, detection_header = result[1], result[2], result[4]
                response = self._validate_and_fill(
                    request,
                    response,
                    value,
                    confidence,
                    'yolo read ok',
                )
                if response.success:
                    image_path = self._save_success_image(request, value, detection_header)
                    self._set_image_path(response, image_path)
                    if image_path:
                        self.get_logger().info(
                            f'[PRINTED NUMBER] Saved successful image: {image_path}'
                        )
                return response
            last_error = result[3]
            time.sleep(poll)
        return self._fail(response, f'timeout waiting for printed number: {last_error}')

    def _stable_result(self, request):
        min_conf = self._request_min_confidence(request)
        expected = self._request_expected_digits(request)
        stable_frames = max(int(self.get_parameter('stable_frames').value), 1)
        max_age = max(float(self.get_parameter('max_detection_age_sec').value), 0.0)
        now = time.monotonic()
        with self._frames_lock:
            frames = list(self._frames)

        parsed = []
        for stamp, msg in reversed(frames):
            if max_age > 0.0 and now - stamp > max_age:
                continue
            value, confidence, error = self._parse_frame(msg, min_conf, expected)
            if value:
                parsed.append((value, confidence, getattr(msg, 'header', None)))
            elif not parsed:
                last_error = error
        if not parsed:
            return False, '', 0.0, locals().get('last_error', 'no fresh valid digit detections'), None

        value = parsed[0][0]
        matches = [(conf, header) for val, conf, header in parsed if val == value]
        if len(matches) < stable_frames:
            return False, '', 0.0, f'value not stable for {stable_frames} frame(s)', None
        confidence = sum(conf for conf, _header in matches[:stable_frames]) / stable_frames
        return True, value, confidence, 'ok', matches[0][1]

    def _parse_frame(self, msg, min_confidence, expected_digits):
        digits = []
        for det in msg.detections:
            score = float(det.hypothesis.score)
            name = str(det.hypothesis.class_name)
            if score < min_confidence or len(name) != 1 or not name.isdigit():
                continue
            digits.append((float(det.bbox.center_x), name, score))
        if not digits:
            return '', 0.0, 'no digit detections above confidence threshold'
        digits.sort(key=lambda item: item[0])
        value = ''.join(item[1] for item in digits)
        if expected_digits > 0 and len(value) != expected_digits:
            return '', 0.0, f'expected {expected_digits} digits, got {len(value)}'
        confidence = sum(item[2] for item in digits) / len(digits)
        return value, confidence, 'ok'

    def _validate_and_fill(self, request, response, value, confidence, message):
        expected = self._request_expected_digits(request)
        min_conf = self._request_min_confidence(request)
        if not value.isdigit():
            return self._fail(response, f'value is not pure digits: {value}')
        if expected > 0 and len(value) != expected:
            return self._fail(response, f'expected {expected} digits, got {len(value)}')
        if confidence < min_conf:
            return self._fail(response, f'confidence {confidence:.3f} below {min_conf:.3f}')
        response.success = True
        response.value = value
        response.confidence = float(confidence)
        response.message = message
        self._set_image_path(response, '')
        return response

    def _save_success_image(self, request, value, detection_header):
        if not self._image_saving_available:
            return ''

        output_dir = self.get_parameter('success_image_dir').value
        encoding = str(self.get_parameter('success_image_encoding').value)
        max_age = max(float(self.get_parameter('max_image_age_sec').value), 0.0)
        wait_sec = max(float(self.get_parameter('success_image_wait_sec').value), 0.0)
        deadline = time.monotonic() + wait_sec

        while True:
            try:
                image_path = save_success_image(
                    image_cache=self._image_cache,
                    detection_header=detection_header,
                    output_dir=output_dir,
                    target_id=str(getattr(request, 'target_id', 'meter')),
                    value=value,
                    bridge=self._bridge,
                    cv2_module=self._cv2,
                    desired_encoding=encoding,
                    now_sec=time.monotonic(),
                    max_image_age_sec=max_age,
                )
            except Exception as exc:
                self.get_logger().warning(f'failed to save successful image: {exc}')
                return ''
            if image_path or time.monotonic() >= deadline:
                return image_path
            time.sleep(min(0.05, max(deadline - time.monotonic(), 0.0)))

    def _request_expected_digits(self, request):
        return int(request.expected_digits) if request.expected_digits > 0 else int(
            self.get_parameter('expected_digits').value
        )

    def _request_min_confidence(self, request):
        return float(request.min_confidence) if request.min_confidence > 0.0 else float(
            self.get_parameter('min_confidence').value
        )

    @staticmethod
    def _fail(response, message):
        response.success = False
        response.value = ''
        response.confidence = 0.0
        response.message = message
        PrintedNumberReader._set_image_path(response, '')
        return response

    @staticmethod
    def _set_image_path(response, image_path):
        if hasattr(response, 'image_path'):
            response.image_path = image_path


def main(args=None):
    rclpy.init(args=args)
    node = PrintedNumberReader()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
