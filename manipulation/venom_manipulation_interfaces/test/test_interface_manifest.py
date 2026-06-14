from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_flame_tracker_status_msg_is_generated_for_tracker_ready_checks():
    status_msg_path = PACKAGE_DIR / 'msg' / 'FlameTrackerStatus.msg'
    cmake_path = PACKAGE_DIR / 'CMakeLists.txt'

    assert status_msg_path.exists()
    assert '"msg/FlameTrackerStatus.msg"' in cmake_path.read_text(encoding='utf-8')

    status_fields = status_msg_path.read_text(encoding='utf-8').splitlines()
    for required_field in [
        'std_msgs/Header header',
        'bool enabled',
        'string mode',
        'bool target_acquired',
        'float32 target_age_sec',
        'float32 yaw_error_rad',
        'float32 pitch_error_rad',
        'bool joint_state_ok',
        'bool camera_info_ok',
        'bool command_output_ok',
        'bool command_saturated',
        'string target_class_name',
        'string message',
    ]:
        assert required_field in status_fields
