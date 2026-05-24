"""Resource contract tests for meter digit voice verification."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
MISSION_PATH = (
    PACKAGE_DIR / 'config' / 'meter_digit_voice_verification_mission.yaml'
)
LAUNCH_PATH = (
    PACKAGE_DIR / 'launch' / 'meter_digit_voice_verification.launch.py'
)
SPEAK_SCRIPT = '/home/alex/venom_ws/scripts/speak_meter_wav.sh'
STABLE_VIDEO_DEVICE = (
    '/dev/v4l/by-id/'
    'usb-SunplusIT_Inc_FHD_Webcam_01.00.00-video-index0'
)


def test_meter_digit_voice_mission_contract():
    """Verify the mission uses isolated 4-digit read and voice command."""
    assert MISSION_PATH.exists()

    mission = yaml.safe_load(MISSION_PATH.read_text(encoding='utf-8'))
    assert mission['mission']['loop'] is False

    tasks = mission['waypoints'][0]['tasks']
    read_meter = next(task for task in tasks if task['type'] == 'read_meter')
    assert read_meter['backend'] == 'service'
    assert read_meter['service_name'] == (
        '/perception/verification/read_printed_number'
    )
    assert read_meter['expected_digits'] == 4
    assert read_meter['timeout_sec'] == 30.0
    assert read_meter['service_wait_timeout_sec'] == 10.0
    assert read_meter['output_key'] == 'meter_reading'

    voice_report = next(
        task for task in tasks if task['type'] == 'voice_report'
    )
    assert voice_report['backend'] == 'command'
    assert voice_report['command'] == SPEAK_SCRIPT
    assert voice_report['required'] is True


def test_meter_digit_voice_launch_contract():
    """Verify the launch file imports and includes the required packages."""
    assert LAUNCH_PATH.exists()

    source = LAUNCH_PATH.read_text(encoding='utf-8')
    assert 'v4l2_camera' in source
    assert 'yolo_detector' in source
    assert 'printed_number_reader' in source
    assert STABLE_VIDEO_DEVICE in source
    assert 'MJPG' in source
    assert 'bgr8' in source
    assert '/perception/verification/read_printed_number' in source
    assert 'meter_digit_voice_verification_mission.yaml' in source

    spec = importlib.util.spec_from_file_location(
        'meter_digit_voice_launch',
        LAUNCH_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    launch_description = module.generate_launch_description()
    assert isinstance(launch_description, LaunchDescription)
