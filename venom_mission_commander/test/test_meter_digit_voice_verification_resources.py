"""Lightweight smoke/config tests for meter digit verification resources."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
MISSION_PATH = PACKAGE_DIR / 'config' / 'meter_digit_voice_verification_mission.yaml'
INTEGRATED_MISSION_PATH = (
    PACKAGE_DIR
    / 'config'
    / 'meter_digit_voice_host_report_verification_mission.yaml'
)
HOST_REPORT_MISSION_PATH = PACKAGE_DIR / 'config' / 'meter_host_report_mission.yaml'
LAUNCH_PATH = PACKAGE_DIR / 'launch' / 'meter_digit_voice_verification.launch.py'
INTEGRATED_LAUNCH_PATH = (
    PACKAGE_DIR / 'launch' / 'meter_digit_voice_host_report_verification.launch.py'
)


def load_yaml(path):
    assert path.exists()
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def tasks_for(path):
    mission = load_yaml(path)
    return mission['waypoints'][0]['tasks']


def task_by_type(tasks, task_type):
    return next(task for task in tasks if task['type'] == task_type)


def assert_read_meter_outputs_meter_reading(task):
    assert task['type'] == 'read_meter'
    assert task['backend'] == 'service'
    assert task['output_key'] == 'meter_reading'


def assert_host_report_uses_reading_image(task):
    assert task['type'] == 'host_report'
    assert task['reading_key'] == 'meter_reading'
    assert task['image_path_key'] == 'image_path'
    assert 'image_path' not in task


def assert_voice_report_requires_command_backend(task):
    assert task['type'] == 'voice_report'
    assert task['backend'] == 'command'
    assert task['required'] is True


def load_launch_description(path, module_name):
    assert path.exists()
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    launch_description = module.generate_launch_description()

    assert isinstance(launch_description, LaunchDescription)


def test_meter_digit_voice_mission_config_chain():
    tasks = tasks_for(MISSION_PATH)

    assert [task['type'] for task in tasks] == ['wait', 'read_meter', 'voice_report']
    assert_read_meter_outputs_meter_reading(tasks[1])
    assert_voice_report_requires_command_backend(tasks[2])


def test_integrated_host_report_mission_config_chain():
    tasks = tasks_for(INTEGRATED_MISSION_PATH)

    assert [task['type'] for task in tasks] == [
        'wait',
        'read_meter',
        'host_report',
        'voice_report',
    ]
    assert_read_meter_outputs_meter_reading(tasks[1])
    assert_host_report_uses_reading_image(tasks[2])
    assert_voice_report_requires_command_backend(tasks[3])


def test_meter_host_report_mission_uses_read_meter_image_path():
    tasks = tasks_for(HOST_REPORT_MISSION_PATH)

    read_meter = task_by_type(tasks, 'read_meter')
    host_report = task_by_type(tasks, 'host_report')

    assert_read_meter_outputs_meter_reading(read_meter)
    assert_host_report_uses_reading_image(host_report)


def test_meter_digit_voice_launch_imports_and_generates_description():
    load_launch_description(LAUNCH_PATH, 'meter_digit_voice_launch')


def test_integrated_launch_imports_and_generates_description():
    load_launch_description(
        INTEGRATED_LAUNCH_PATH,
        'meter_digit_voice_host_report_launch',
    )
