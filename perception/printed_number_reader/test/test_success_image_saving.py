"""Contracts for saving the successful printed-number image."""

from pathlib import Path
import sys
import types


interface_module = types.ModuleType('printed_number_interfaces')
srv_module = types.ModuleType('printed_number_interfaces.srv')
setattr(srv_module, 'ReadPrintedNumber', object)
setattr(interface_module, 'srv', srv_module)
sys.modules.setdefault('printed_number_interfaces', interface_module)
sys.modules.setdefault('printed_number_interfaces.srv', srv_module)

from printed_number_reader import printed_number_reader_node as reader


PACKAGE_DIR = Path(__file__).resolve().parents[1]
INTERFACE_DIR = PACKAGE_DIR.parent / 'printed_number_interfaces'
SERVICE_PATH = INTERFACE_DIR / 'srv' / 'ReadPrintedNumber.srv'


class Header:
    def __init__(self, sec=0, nanosec=0):
        self.stamp = type('Stamp', (), {'sec': sec, 'nanosec': nanosec})()


class FakeImage:
    def __init__(self, sec, nanosec, payload):
        self.header = Header(sec, nanosec)
        self.payload = payload


class FakeBridge:
    def imgmsg_to_cv2(self, msg, desired_encoding='bgr8'):
        return msg.payload


class FakeCv2:
    def __init__(self):
        self.writes = []

    def imwrite(self, path, image):
        Path(path).write_bytes(image)
        self.writes.append((path, image))
        return True


def test_service_response_includes_success_image_path_field():
    service = SERVICE_PATH.read_text(encoding='utf-8')

    assert 'string image_path' in service


def test_success_image_helpers_save_matching_cached_image(tmp_path):
    cache = reader.ImageCache(maxlen=3)
    cv2_module = FakeCv2()
    old_image = FakeImage(10, 1, b'old-image')
    successful_image = FakeImage(10, 2, b'successful-image')
    cache.append(old_image, received_at=100.0)
    cache.append(successful_image, received_at=101.0)

    output_path = reader.save_success_image(
        image_cache=cache,
        detection_header=Header(10, 2),
        output_dir=tmp_path,
        target_id='meter_2',
        value='1234',
        bridge=FakeBridge(),
        cv2_module=cv2_module,
        desired_encoding='bgr8',
        now_sec=101.5,
        max_image_age_sec=10.0,
    )

    saved_path = Path(output_path)
    assert saved_path.exists()
    assert saved_path.name == 'meter_2_1234_success.jpg'
    assert saved_path.read_bytes() == b'successful-image'
    assert (tmp_path / 'meter_2_latest_success.jpg').read_bytes() == b'successful-image'
    assert cv2_module.writes[0][0].endswith('.jpg')


def test_success_image_helpers_return_empty_path_when_cache_is_stale(tmp_path):
    cache = reader.ImageCache(maxlen=3)
    cache.append(FakeImage(10, 2, b'stale-image'), received_at=100.0)

    output_path = reader.save_success_image(
        image_cache=cache,
        detection_header=Header(10, 2),
        output_dir=tmp_path,
        target_id='meter_2',
        value='1234',
        bridge=FakeBridge(),
        cv2_module=FakeCv2(),
        desired_encoding='bgr8',
        now_sec=105.5,
        max_image_age_sec=1.0,
    )

    assert output_path == ''
    assert list(tmp_path.iterdir()) == []


def test_success_image_helpers_wait_for_matching_stamp(tmp_path):
    cache = reader.ImageCache(maxlen=3)
    cache.append(FakeImage(10, 1, b'fresh-but-wrong-frame'), received_at=101.0)

    output_path = reader.save_success_image(
        image_cache=cache,
        detection_header=Header(10, 2),
        output_dir=tmp_path,
        target_id='meter_2',
        value='1234',
        bridge=FakeBridge(),
        cv2_module=FakeCv2(),
        desired_encoding='bgr8',
        now_sec=101.5,
        max_image_age_sec=10.0,
    )

    assert output_path == ''
    assert list(tmp_path.iterdir()) == []


def test_success_image_helpers_reject_path_traversal_target_id(tmp_path):
    cache = reader.ImageCache(maxlen=1)
    cache.append(FakeImage(1, 1, b'image'), received_at=1.0)

    output_path = reader.save_success_image(
        image_cache=cache,
        detection_header=Header(1, 1),
        output_dir=tmp_path,
        target_id='../meter 2',
        value='12/34',
        bridge=FakeBridge(),
        cv2_module=FakeCv2(),
        desired_encoding='bgr8',
        now_sec=1.1,
        max_image_age_sec=2.0,
    )

    saved_path = Path(output_path)
    assert saved_path.parent == tmp_path
    assert saved_path.name == 'meter_2_12_34_success.jpg'
    assert saved_path.exists()
