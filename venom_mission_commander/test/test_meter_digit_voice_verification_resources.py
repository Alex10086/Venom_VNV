"""Lightweight smoke/config tests for meter digit verification resources."""

import ast
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
COMPETITION_RUNTIME_LAUNCH_PATH = PACKAGE_DIR / 'launch' / 'competition_10x6_arm_runtime.launch.py'
D435I_COLOR_IMAGE_TOPIC = '/camera/camera/color/image_raw'
METER_VERIFICATION_LAUNCHES = (
    LAUNCH_PATH,
    INTEGRATED_LAUNCH_PATH,
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


def launch_source(path):
    assert path.exists()
    return path.read_text(encoding='utf-8')


def competition_runtime_launch_arguments():
    source = launch_source(COMPETITION_RUNTIME_LAUNCH_PATH)
    module = ast.parse(source)
    include = next(
        node
        for node in ast.walk(module)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'IncludeLaunchDescription'
    )
    launch_arguments = next(
        keyword.value
        for keyword in include.keywords
        if keyword.arg == 'launch_arguments'
    )
    assert isinstance(launch_arguments, ast.Call)
    assert isinstance(launch_arguments.func, ast.Attribute)
    assert launch_arguments.func.attr == 'items'
    assert isinstance(launch_arguments.func.value, ast.Dict)

    return {
        key.value: value.value
        for key, value in zip(
            launch_arguments.func.value.keys,
            launch_arguments.func.value.values,
        )
        if isinstance(key, ast.Constant)
        and isinstance(key.value, str)
        and isinstance(value, ast.Constant)
        and isinstance(value.value, str)
    }


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


def test_competition_runtime_launch_imports_and_generates_description():
    load_launch_description(
        COMPETITION_RUNTIME_LAUNCH_PATH,
        'competition_10x6_arm_runtime_launch',
    )


def test_competition_runtime_launch_starts_yolo_nodes_disabled():
    source = launch_source(COMPETITION_RUNTIME_LAUNCH_PATH)

    assert "'pick_yolo_enabled': 'false'" in source
    assert "'classification_yolo_enabled': 'false'" in source
    assert "'flame_yolo_enabled': 'false'" in source
    assert "'enabled': False" in source


def test_competition_runtime_uses_only_classification_yolo_for_box_detections():
    launch_arguments = competition_runtime_launch_arguments()

    assert launch_arguments['launch_color_box_detector'] == 'false'
    assert launch_arguments['launch_classification_yolo_detector'] == 'true'
    assert launch_arguments['launch_classification_yolo_bridge'] == 'true'


def test_meter_verification_launches_use_realsense_camera_driver():
    for path in METER_VERIFICATION_LAUNCHES:
        source = launch_source(path)

        assert "FindPackageShare('realsense2_camera')" in source
        assert "package='v4l2_camera'" not in source
        assert "executable='v4l2_camera_node'" not in source


def test_meter_verification_yolo_defaults_to_d435i_color_image_topic():
    for path in METER_VERIFICATION_LAUNCHES:
        source = launch_source(path)

        assert "'image_topic'" in source
        assert f"D435I_COLOR_IMAGE_TOPIC = '{D435I_COLOR_IMAGE_TOPIC}'" in source
        assert 'default_value=D435I_COLOR_IMAGE_TOPIC' in source


def test_meter_verification_model_path_uses_home_environment():
    for path in METER_VERIFICATION_LAUNCHES:
        source = launch_source(path)

        assert '/home/alex/venom_ws' not in source
        assert "EnvironmentVariable('HOME')" in source
        assert "'venom_ws'" in source
        assert "'yolo_26_detect_digit.pt'" in source


def test_meter_voice_reader_uses_annotated_image_topic_for_success_images():
    source = launch_source(LAUNCH_PATH)

    assert "'image_topic': annotated_image_topic" in source


def test_meter_verification_realsense_launch_does_not_inherit_yolo_args():
    for path in METER_VERIFICATION_LAUNCHES:
        source = launch_source(path)

        assert 'GroupAction' in source
        assert 'forwarding=False' in source


def test_meter_verification_realsense_defaults_to_usb2_safe_color_profile():
    for path in METER_VERIFICATION_LAUNCHES:
        source = launch_source(path)

        assert "'rgb_camera.color_profile'" in source
        assert "default_value='640x480x15'" in source
