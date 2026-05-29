"""Tests for host report mission task integration."""

import hashlib
import json
from pathlib import Path

from venom_mission_commander.models import TaskContext, TaskSpec, WaypointSpec
from venom_mission_commander.read_meter_task import (
    ReadMeterServiceConfig,
    build_service_reading,
)
from venom_mission_commander.task_plugins import (
    BaseTaskPlugin,
    HostReportTaskPlugin,
    TaskPluginRegistry,
)

class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warning(self, message):
        self.messages.append(('warning', message))

    def error(self, message):
        self.messages.append(('error', message))


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


class FakeResponse:
    value = '1234'
    confidence = 0.93
    image_path = '/tmp/meter_2.jpg'


def make_context(blackboard=None):
    return TaskContext(
        node=FakeNode(),
        mission_id='craic2026_demo',
        waypoint=WaypointSpec(
            name='wp_2_meter',
            frame_id='map',
            x=1.0,
            y=2.0,
            yaw=0.0,
        ),
        waypoint_index=1,
        task_index=2,
        mission_manager=None,
        blackboard=blackboard or {},
    )


def make_spec(params):
    return TaskSpec(
        name='report_meter_to_host',
        task_type='host_report',
        params=params,
    )


def execute_host_report(context, params):
    plugin = HostReportTaskPlugin()
    plugin.configure(context.node)
    return plugin.execute(context, make_spec(params))


def test_mock_backend_writes_success_report_without_image_file():
    context = make_context({'meter_reading': {'meter_id': 'meter_2', 'value': '1234'}})

    result = execute_host_report(
        context,
        {
            'backend': 'mock',
            'report_kind': 'meter_reading',
            'output_key': 'last_host_report',
        },
    )

    assert result.success is True
    assert result.message == 'host report completed'
    assert context.blackboard['last_host_report']['success'] is True
    assert context.blackboard['last_host_report']['backend'] == 'mock'
    assert context.blackboard['last_host_report']['report_kind'] == 'meter_reading'
    assert context.blackboard['last_host_report']['mission_id'] == 'craic2026_demo'


def test_file_backend_copies_image_and_writes_metadata_and_receipt(tmp_path):
    source_image = tmp_path / 'meter_source.jpg'
    image_bytes = b'meter-image-bytes'
    source_image.write_bytes(image_bytes)
    expected_sha256 = hashlib.sha256(image_bytes).hexdigest()
    context = make_context(
        {
            'meter_reading': {
                'meter_id': 'meter_2',
                'value': '1234',
                'confidence': 0.93,
                'image_path': str(source_image),
            }
        }
    )

    result = execute_host_report(
        context,
        {
            'backend': 'file',
            'report_kind': 'meter_reading',
            'report_dir': str(tmp_path / 'reports'),
            'required': True,
        },
    )

    assert result.success is True
    report = context.blackboard['last_host_report']
    assert report['success'] is True
    assert report['backend'] == 'file'
    assert report['image_sha256'] == expected_sha256

    copied_image = Path(report['image_path'])
    metadata_path = Path(report['metadata_path'])
    receipt_path = Path(report['receipt_path'])
    assert copied_image.read_bytes() == image_bytes
    assert metadata_path.exists()
    assert receipt_path.exists()

    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    assert metadata['mission_id'] == 'craic2026_demo'
    assert metadata['waypoint_name'] == 'wp_2_meter'
    assert metadata['task_name'] == 'report_meter_to_host'
    assert metadata['meter_id'] == 'meter_2'
    assert metadata['value'] == '1234'
    assert metadata['image_sha256'] == expected_sha256

    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    assert receipt['success'] is True
    assert receipt['backend'] == 'file'


def test_file_backend_fails_required_when_image_path_missing():
    context = make_context({'meter_reading': {'meter_id': 'meter_2', 'value': '1234'}})

    result = execute_host_report(context, {'backend': 'file', 'required': True})

    assert result.success is False
    assert 'image_path missing' in result.message
    assert context.blackboard['last_host_report']['success'] is False


def test_file_backend_downgrades_failure_when_not_required():
    context = make_context({})

    result = execute_host_report(context, {'backend': 'file', 'required': False})

    assert result.success is True
    assert result.message.startswith('host report skipped:')
    assert context.blackboard['last_host_report']['success'] is False


def test_file_backend_rejects_image_larger_than_limit(tmp_path):
    source_image = tmp_path / 'large_meter.jpg'
    source_image.write_bytes(b'123456')
    context = make_context({'meter_reading': {'image_path': str(source_image)}})

    result = execute_host_report(
        context,
        {
            'backend': 'file',
            'required': True,
            'max_image_bytes': 5,
        },
    )

    assert result.success is False
    assert 'image too large' in result.message


def test_file_backend_uses_controlled_image_name_for_reserved_source(tmp_path):
    source_image = tmp_path / 'metadata.json'
    image_bytes = b'image-that-should-not-be-overwritten'
    source_image.write_bytes(image_bytes)
    context = make_context({'meter_reading': {'image_path': str(source_image)}})

    result = execute_host_report(
        context,
        {
            'backend': 'file',
            'required': True,
            'report_dir': str(tmp_path / 'reports'),
        },
    )

    assert result.success is True
    copied_image = Path(context.blackboard['last_host_report']['image_path'])
    assert copied_image.name.startswith('image_')
    assert copied_image.name != 'metadata.json'
    assert copied_image.read_bytes() == image_bytes


def test_invalid_numeric_parameter_returns_task_result_without_raising():
    context = make_context({})

    result = execute_host_report(
        context,
        {'backend': 'file', 'retry_count': 'two', 'required': False},
    )

    assert result.success is True
    assert result.message.startswith('host report skipped: invalid host_report parameter')
    assert context.blackboard['last_host_report']['success'] is False


def test_registry_registers_host_report_plugin():
    registry = TaskPluginRegistry()

    registry.register_default_plugins(FakeNode())

    assert registry.has('host_report') is True
    assert isinstance(registry.get('host_report'), BaseTaskPlugin)


def test_service_reading_includes_optional_image_path():
    config = ReadMeterServiceConfig(
        meter_id='meter_2',
        service_name='/perception/read_printed_number',
        timeout_sec=5.0,
        service_wait_timeout_sec=1.0,
        expected_digits=4,
        min_confidence=0.6,
        output_key='meter_reading',
    )

    reading = build_service_reading(config, FakeResponse())

    assert reading['image_path'] == '/tmp/meter_2.jpg'
