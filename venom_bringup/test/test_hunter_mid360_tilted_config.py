from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_mid360_launch_accepts_real_livox_frame_id():
    launch_text = read_text("launch/examples/mid360_point_lio.launch.py")

    assert 'LaunchConfiguration("livox_frame_id")' in launch_text
    assert "DeclareLaunchArgument" in launch_text
    assert '"livox_frame_id"' in launch_text
    assert '{"frame_id": livox_frame_id}' in launch_text


def test_hunter_tilted_point_lio_config_uses_sensor_frame_for_lio_body():
    config_path = (
        PACKAGE_ROOT / "config/hunter_se/point_lio_mid360_tilted.yaml"
    )
    assert config_path.is_file()

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["/**"]
    config = config["ros__parameters"]

    assert config["common"]["lid_topic"] == "livox/lidar"
    assert config["common"]["imu_topic"] == "livox/imu"
    assert config["mapping"]["extrinsic_est_en"] is False
    assert config["mapping"]["extrinsic_R"] == [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    assert config["frame"]["base_frame_id"] == "mid360_link"
    assert config["frame"]["cloud_registered_body_frame_id"] == "mid360_link"


def test_hunter_scripts_publish_tilted_tf_and_scan_in_base_link():
    for script in (
        "scripts/start_hunter_mid360_mapping.sh",
        "scripts/start_hunter_mid360_nav2_teb.sh",
    ):
        script_text = read_text(script)

        assert "config/hunter_se/point_lio_mid360_tilted.yaml" in script_text
        assert 'LIVOX_FRAME_ID="${LIVOX_FRAME_ID:-mid360_link}"' in script_text
        expected_pitch = (
            'MID360_TO_BASE_PITCH="${MID360_TO_BASE_PITCH:-'
            '0.5489}"'
        )
        expected_yaw = (
            'MID360_TO_BASE_YAW="${MID360_TO_BASE_YAW:-'
            '3.141592653589793}"'
        )
        assert expected_pitch in script_text
        assert expected_yaw in script_text
        assert '"livox_frame_id:=$LIVOX_FRAME_ID"' in script_text
        assert "static_transform_publisher" in script_text
        assert '--frame-id "$LIVOX_FRAME_ID"' in script_text
        assert "--child-frame-id base_link" in script_text
        assert "-p target_frame:=base_link" in script_text
        assert 'SCAN_MIN_HEIGHT="${SCAN_MIN_HEIGHT:-0.05}"' in script_text
        assert 'SCAN_MAX_HEIGHT="${SCAN_MAX_HEIGHT:-0.7}"' in script_text
        assert 'SCAN_RANGE_MIN="${SCAN_RANGE_MIN:-0.3}"' in script_text
        assert 'SCAN_RANGE_MAX="${SCAN_RANGE_MAX:-50.0}"' in script_text
        assert '"min_height:=$SCAN_MIN_HEIGHT"' in script_text
        assert '"max_height:=$SCAN_MAX_HEIGHT"' in script_text
        assert '"range_min:=$SCAN_RANGE_MIN"' in script_text
        assert '"range_max:=$SCAN_RANGE_MAX"' in script_text
