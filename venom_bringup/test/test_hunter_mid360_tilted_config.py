from pathlib import Path
import re

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def read_yaml(relative_path: str):
    config_path = PACKAGE_ROOT / relative_path
    assert config_path.is_file()

    return yaml.safe_load(config_path.read_text(encoding="utf-8"))


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
        "scripts/start_hunter_mid360_nav2_smac_teb.sh",
    ):
        script_text = read_text(script)

        expected_point_lio_config = (
            'POINT_LIO_CFG="${POINT_LIO_CFG:-$WS/src/venom_vnv/'
            'venom_bringup/config/hunter_se/point_lio_mid360_light.yaml}"'
            if script == "scripts/start_hunter_mid360_mapping.sh"
            else "config/hunter_se/point_lio_mid360_tilted.yaml"
        )
        assert expected_point_lio_config in script_text
        assert re.search(r"(?m)^\s*trap\s+\w+\s+INT\s+TERM\s*$", script_text)
        assert re.search(r"(?m)^\s*trap\s+cleanup\s+EXIT\s*$", script_text)
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


def test_hunter_mid360_pid_managed_scripts_stop_process_groups():
    scripts = sorted(
        path
        for path in (PACKAGE_ROOT / "scripts").glob("start_hunter_mid360*.sh")
        if "PIDS=()" in path.read_text(encoding="utf-8")
    )
    assert scripts

    for script_path in scripts:
        script_text = script_path.read_text(encoding="utf-8")
        helpers = list(re.finditer(
            r"(?ms)^(?P<name>[A-Za-z_][A-Za-z0-9_]*)\(\) \{\n"
            r"(?P<body>.*?)^\}\n",
            script_text,
        ))
        background_helper = next(
            (
                helper
                for helper in helpers
                if 'setsid "$@" &' in helper["body"]
                and 'PIDS+=("$!")' in helper["body"]
            ),
            None,
        )

        assert background_helper, f"{script_path.name} must launch PID-tracked commands with setsid"
        assert script_text.count('PIDS+=("$!")') == 1
        assert 'kill -- "-$pid"' in script_text

        cleanup_helper = next(
            (helper for helper in helpers if helper["name"] == "cleanup"),
            None,
        )
        assert cleanup_helper, f"{script_path.name} must define cleanup"
        cleanup_body = cleanup_helper["body"]

        if "ros2 topic pub" in cleanup_body:
            zero_velocity_publishes = [
                line.strip()
                for line in cleanup_body.splitlines()
                if "ros2 topic pub" in line and "/cmd_vel" in line
            ]
            assert zero_velocity_publishes, (
                f"{script_path.name} must publish zero velocity during cleanup"
            )
            assert all(
                re.match(
                    r"timeout\s+--kill-after=1\s+2\s+ros2\s+topic\s+pub\b",
                    publish,
                )
                for publish in zero_velocity_publishes
            ), (
                f"{script_path.name} must use timeout --kill-after=1 2 for "
                "cleanup zero-velocity publishes"
            )

        signal_handler = next(
            (
                helper
                for helper in helpers
                if helper["name"] != "cleanup"
                and re.search(r"\bcleanup\b.*\bexit(?:\s+\d+)?\b", helper["body"], re.DOTALL)
            ),
            None,
        )
        assert signal_handler, (
            f"{script_path.name} must use a signal handler that cleans up and exits"
        )
        signal_handler_name = signal_handler["name"]
        assert re.search(
            rf"(?m)^\s*trap\s+{re.escape(signal_handler_name)}\s+(?:INT\s+TERM|TERM\s+INT)\s*$",
            script_text,
        ), f"{script_path.name} must route INT and TERM to its signal handler"
        assert re.search(r"(?m)^\s*trap\s+cleanup\s+EXIT\s*$", script_text), (
            f"{script_path.name} must retain cleanup on EXIT"
        )

        term_kill = cleanup_body.find('kill -- "-$pid"')
        kill_escalation = cleanup_body.find('kill -KILL -- "-$pid"')
        assert term_kill >= 0 and kill_escalation > term_kill, (
            f"{script_path.name} must escalate its process group from TERM to KILL"
        )
        assert re.search(r"\b(?:sleep|timeout)\s+\d", cleanup_body), (
            f"{script_path.name} must bound the TERM-to-KILL escalation"
        )

        standalone_waits = list(
            re.finditer(r"(?m)^[ \t]*wait[ \t]+-n(?:[ \t]+\S+)*[ \t]*$", script_text)
        )
        assert standalone_waits, f"{script_path.name} must have a standalone wait -n"
        final_wait = standalone_waits[-1]

        child_status_capture = re.search(
            r"(?m)^\s*child_status=\$\?\s*$", script_text[final_wait.end():]
        )
        assert child_status_capture, (
            f"{script_path.name} must capture the final wait -n status"
        )

        zero_to_one_normalization = re.search(
            r"(?ms)^\s*if\s+\[\s+\"\$child_status\"\s+-eq\s+0\s+\];\s*then\s*"
            r".*?^\s*child_status=1\s*$.*?^\s*fi\s*$",
            script_text[final_wait.end() + child_status_capture.end():],
        )
        assert zero_to_one_normalization, (
            f"{script_path.name} must convert an unexpected zero child status to failure"
        )

        child_status_exit = re.search(
            r'(?m)^\s*exit\s+"\$child_status"\s*$',
            script_text[
                final_wait.end()
                + child_status_capture.end()
                + zero_to_one_normalization.end():
            ],
        )
        assert child_status_exit, (
            f"{script_path.name} must exit with the normalized child status"
        )


def test_hunter_smac_teb_nav2_params_use_ackermann_smac_with_teb():
    config = read_yaml("config/hunter_se/nav2_smac_teb_params.yaml")

    amcl = config["amcl"]["ros__parameters"]
    assert amcl["global_frame_id"] == "map"
    assert amcl["odom_frame_id"] == "odom"
    assert amcl["base_frame_id"] == "base_link"
    assert amcl["tf_broadcast"] is True
    assert amcl["scan_topic"] == "/scan"
    assert amcl["update_min_a"] == 0.10
    assert amcl["update_min_d"] == 0.10

    planner = config["planner_server"]["ros__parameters"]
    grid_based = planner["GridBased"]
    assert planner["planner_plugins"] == ["GridBased"]
    assert planner["expected_planner_frequency"] == 5.0
    assert grid_based["plugin"] == "nav2_smac_planner/SmacPlannerHybrid"
    assert grid_based["motion_model_for_search"] == "REEDS_SHEPP"
    assert grid_based["minimum_turning_radius"] == 1.2
    assert grid_based["analytic_expansion_max_length"] == 10.0

    follow_path = config["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow_path["plugin"] == "teb_local_planner::TebLocalPlannerROS"
    assert follow_path["min_turning_radius"] == 1.2
    assert follow_path["max_vel_theta"] == 0.45
    assert follow_path["acc_lim_theta"] == 0.8
    assert follow_path["allow_init_with_backwards_motion"] is True

    behavior = config["behavior_server"]["ros__parameters"]
    assert behavior["max_rotational_vel"] == 0.45
    assert behavior["rotational_acc_lim"] == 0.8

    velocity_smoother = config["velocity_smoother"]["ros__parameters"]
    assert velocity_smoother["max_velocity"][2] == 0.45
    assert velocity_smoother["min_velocity"][2] == -0.45
    assert velocity_smoother["max_accel"][2] == 0.8
    assert velocity_smoother["max_decel"][2] == -0.8


def test_hunter_smac_teb_script_launches_nav2_with_expected_config():
    script_text = read_text("scripts/start_hunter_mid360_nav2_smac_teb.sh")

    assert "config/hunter_se/point_lio_mid360_tilted.yaml" in script_text
    assert "config/hunter_se/nav2_smac_teb_params.yaml" in script_text
    assert 'NAV_RVIZ="${NAV_RVIZ:-true}"' in script_text
    assert (
        'NAV_RVIZ_CONFIG="${NAV_RVIZ_CONFIG:-$WS/src/venom_vnv/'
        'venom_bringup/rviz_cfg/hunter_mid360_nav2.rviz}"'
        in script_text
    )
    assert 'MID360_TO_BASE_PITCH="${MID360_TO_BASE_PITCH:-0.5489}"' in script_text
    assert 'MID360_TO_BASE_YAW="${MID360_TO_BASE_YAW:-3.141592653589793}"' in script_text
    assert 'require_file "$NAV_RVIZ_CONFIG" "Nav2 RViz config"' in script_text
    assert "Starting Hunter base with isolated wheel odometry" in script_text
    assert "odom_frame:=hunter_odom" in script_text
    assert "base_frame:=hunter_base_link" in script_text
    assert "odom_topic_name:=hunter_odom" in script_text
    assert "nav2_bringup bringup_launch.py" in script_text
    assert "slam:=False" in script_text
    assert '"map:=$MAP"' in script_text
    assert '"params_file:=$NAV2_PARAMS"' in script_text
    assert 'AUTO_INITIAL_POSE="${AUTO_INITIAL_POSE:-true}"' in script_text
    assert 'INITIAL_POSE_X="${INITIAL_POSE_X:-0.0}"' in script_text
    assert 'INITIAL_POSE_Y="${INITIAL_POSE_Y:-0.0}"' in script_text
    assert 'INITIAL_POSE_YAW="${INITIAL_POSE_YAW:-0.0}"' in script_text
    assert "wait_for_lifecycle_active /amcl" in script_text
    assert "ros2 service call /set_initial_pose" in script_text
    assert "nav2_msgs/srv/SetInitialPose" in script_text
    assert "topic pub --once /initialpose" not in script_text
    assert '"rviz:=$NAV_RVIZ"' in script_text
    assert '"rviz_config:=$NAV_RVIZ_CONFIG"' in script_text
    assert (
        "Set the initial pose in RViz with 2D Pose Estimate before "
        "mission_commander."
        not in script_text
    )
    assert "ros2 pkg prefix teb_local_planner" in script_text

    assert "wait -n" in script_text
    assert "/cmd_vel" in script_text
    assert "geometry_msgs/msg/Twist" in script_text
    assert "linear:" in script_text
    assert "angular:" in script_text


def test_hunter_smac_teb_initial_pose_timeout_accepts_available_amcl_tf():
    script_text = read_text("scripts/start_hunter_mid360_nav2_smac_teb.sh")

    tf_helper = re.search(
        r"(?ms)^wait_for_amcl_map_to_odom_tf\(\) \{\n(?P<body>.*?)^\}\n",
        script_text,
    )
    assert tf_helper, "initial-pose recovery must use a bounded AMCL TF check"
    assert 'timeout 2 ros2 run tf2_ros tf2_echo map odom' in tf_helper["body"]
    assert 'grep -q "At time"' in tf_helper["body"]
    assert "while" in tf_helper["body"]

    initial_pose_block = re.search(
        r"(?ms)^\s*if ! publish_initial_pose; then\n(?P<body>.*?)^\s*fi$",
        script_text,
    )
    assert initial_pose_block
    assert "wait_for_amcl_map_to_odom_tf" in initial_pose_block["body"]
    assert "Failed to set initial pose via service" in initial_pose_block["body"]


def test_hunter_smac_teb_script_reports_unexpected_child_exit_diagnostics():
    script_text = read_text("scripts/start_hunter_mid360_nav2_smac_teb.sh")

    lifecycle_helper = re.search(
        r"(?ms)^wait_for_lifecycle_active\(\) \{\n(?P<body>.*?)^\}\n",
        script_text,
    )
    assert lifecycle_helper
    assert 'ros2 lifecycle get --no-daemon "$node_name"' in lifecycle_helper["body"]

    assert re.search(r"(?m)^\s*declare\s+-A\s+PID_COMMANDS=\(\)\s*$", script_text), (
        "the script must retain each tracked PID's command for diagnostics"
    )
    start_process = re.search(
        r"(?ms)^start_process\(\) \{\n(?P<body>.*?)^\}\n",
        script_text,
    )
    assert start_process
    pid_recording = start_process["body"].find('PIDS+=("$!")')
    command_recording = start_process["body"].find('PID_COMMANDS["$!"]="$*"')
    assert pid_recording >= 0 and command_recording > pid_recording, (
        "start_process must record the command for every tracked PID"
    )

    final_wait = re.search(
        r'(?m)^\s*wait\s+-n\s+-p\s+exited_pid\s+"\$\{PIDS\[@\]\}"\s*$',
        script_text,
    )
    assert final_wait, "the final wait must identify the exited tracked PID"
    remaining_text = script_text[final_wait.end():]
    assert re.search(r"(?m)^\s*child_status=\$\?\s*$", remaining_text)
    assert re.search(
        r'(?m)^\s*echo\b.*\$exited_pid.*\$child_status.*\$\{PID_COMMANDS\[\$exited_pid\]\}',
        remaining_text,
    ), "unexpected-process output must include PID, status, and command"

    signal_handler = re.search(
        r"(?ms)^handle_signal\(\) \{\n(?P<body>.*?)^\}\n",
        script_text,
    )
    assert signal_handler
    signal_body = signal_handler["body"]
    signal_log = re.search(
        r"(?im)^\s*echo\b.*external.*(?:INT.*TERM|TERM.*INT).*?$",
        signal_body,
    )
    assert signal_log, "handle_signal must log the external INT/TERM before cleanup"
    assert signal_log.start() < signal_body.index("cleanup")


def test_hunter_mid360_nav2_rviz_config_matches_nav2_debug_needs():
    rviz_path = PACKAGE_ROOT / "rviz_cfg/hunter_mid360_nav2.rviz"
    assert rviz_path.is_file()

    rviz_text = rviz_path.read_text(encoding="utf-8")

    assert "Class: nav2_rviz_plugins/Navigation 2" in rviz_text
    assert "Name: Navigation 2" in rviz_text
    assert "Fixed Frame: map" in rviz_text
    rviz_config = yaml.safe_load(rviz_text)
    displays = rviz_config["Visualization Manager"]["Displays"]
    cloud_registered_displays = [
        display
        for display in displays
        if display["Name"] == "Cloud Registered"
    ]
    assert len(cloud_registered_displays) == 1
    cloud_registered = cloud_registered_displays[0]
    assert cloud_registered["Class"] == "rviz_default_plugins/PointCloud2"
    assert cloud_registered["Enabled"] is True
    assert cloud_registered["Topic"]["Value"] == "/cloud_registered"
    assert cloud_registered["Topic"]["Reliability Policy"] == "Reliable"
    assert cloud_registered["Use Fixed Frame"] is True
    for expected_text in (
        "Class: rviz_default_plugins/LaserScan",
        "Value: /scan",
        "Class: rviz_default_plugins/Map",
        "Value: /map",
        "Class: rviz_default_plugins/Path",
        "Value: /plan",
        "Name: Local Plan",
        "Value: /local_plan",
        "Name: Global Costmap",
        "Value: /global_costmap/costmap",
        "Name: Local Costmap",
        "Value: /local_costmap/costmap",
        "Class: nav2_rviz_plugins/ParticleCloud",
        "Value: /particle_cloud",
    ):
        assert expected_text in rviz_text


def test_human_runbook_documents_mid360_as_main_point_lio_tf_body():
    runbook_text = (
        PACKAGE_ROOT.parent
        / "venom_mission_commander/docs/HUMAN_RUNBOOK.md"
    ).read_text(encoding="utf-8")

    assert "odom -> mid360_link" in runbook_text
    assert "mid360_link -> base_link" in runbook_text
    assert "Point-LIO 发布\n`odom -> base_link`" not in runbook_text
    assert "Point-LIO 发布 `odom -> base_link`" not in runbook_text
    assert "AUTO_INITIAL_POSE" in runbook_text
    assert "不需要" in runbook_text and "RViz" in runbook_text and "手动" in runbook_text
    assert "必须先在 RViz 使用 2D Pose Estimate" not in runbook_text
    assert "默认" in runbook_text and "RViz" in runbook_text and "打开" in runbook_text
    assert "地图" in runbook_text
    assert "全局路径" in runbook_text
    assert "局部路径" in runbook_text
    assert "全局代价地图" in runbook_text
    assert "局部代价地图" in runbook_text
    assert "scan" in runbook_text
    assert "/set_initial_pose" in runbook_text
    assert "nav2_msgs/srv/SetInitialPose" in runbook_text
    assert "/cloud_registered" in runbook_text
    assert "点云" in runbook_text
    assert "NAV_RVIZ=false" in runbook_text
    assert "关闭" in runbook_text


def test_hunter_light_point_lio_config_reduces_runtime_load():
    config_path = (
        PACKAGE_ROOT / "config/hunter_se/point_lio_mid360_light.yaml"
    )
    assert config_path.is_file()

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["/**"]
    config = config["ros__parameters"]

    assert config["lio"]["operation_mode"] == "online_odom_async_map"
    assert config["point_filter_num"] == 8
    assert config["filter_size_surf"] == 0.15
    assert config["filter_size_map_internal"] == 0.5
    assert config["filter_size_map_publish"] == 0.5
    assert config["odometry"]["enable_2d_mode"] is False
    assert config["publish"]["path_en"] is False
    assert config["publish"]["scan_publish_en"] is True
    assert config["publish"]["map_publish_en"] is False
    assert config["frame"]["base_frame_id"] == "mid360_link"


def test_hunter_light_lio_script_runs_lio_only_with_tilted_static_tf():
    script_text = read_text("scripts/start_hunter_mid360_lio_light.sh")

    assert "config/hunter_se/point_lio_mid360_light.yaml" in script_text
    assert 'POINT_LIO_RVIZ="${POINT_LIO_RVIZ:-false}"' in script_text
    assert 'LIVOX_FRAME_ID="${LIVOX_FRAME_ID:-mid360_link}"' in script_text
    assert 'MID360_TO_BASE_PITCH="${MID360_TO_BASE_PITCH:-0.5489}"' in script_text
    assert 'MID360_TO_BASE_YAW="${MID360_TO_BASE_YAW:-3.141592653589793}"' in script_text
    assert '"livox_frame_id:=$LIVOX_FRAME_ID"' in script_text
    assert '"point_lio_cfg:=$POINT_LIO_CFG"' in script_text
    assert "static_transform_publisher" in script_text
    assert "pointcloud_to_laserscan" not in script_text
    assert "slam_toolbox" not in script_text
    assert "nav2_bringup" not in script_text
