"""Tests for CRAIC arm/flame mission task integration."""

import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
import yaml

from venom_mission_commander.mission_manager import MissionManager
from venom_mission_commander.models import (
    MissionConfig,
    MissionState,
    TaskContext,
    TaskExecutionResult,
    TaskSpec,
    WaypointSpec,
)
from venom_mission_commander.task_plugins import (
    ClassifyPlaceTaskPlugin,
    DetectFlameTaskPlugin,
    GraspItemTaskPlugin,
    TaskPluginRegistry,
    TrackFlameTaskPlugin,
)


PACKAGE_DIR = Path(__file__).resolve().parents[1]
VENOM_VNV_DIR = PACKAGE_DIR.parent
CRAIC_ARM_MISSION_PATH = PACKAGE_DIR / 'config' / 'competition_10x6_arm_mission.yaml'


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


class FakeStatusReporter:
    def __init__(self):
        self.snapshots = []

    def log_snapshot(self, label, mission_manager):
        self.snapshots.append((label, mission_manager.state))


def make_context(blackboard=None):
    return TaskContext(
        node=FakeNode(),
        mission_id='craic2026_arm_demo',
        waypoint=WaypointSpec(
            name='task_point_3_flame_tracking',
            frame_id='map',
            x=5.0,
            y=4.15,
            yaw=1.5708,
        ),
        waypoint_index=2,
        task_index=0,
        mission_manager=None,
        blackboard=blackboard or {},
    )


def make_test_commander(mission_config=None):
    from venom_mission_commander.mission_commander import MissionCommander

    if mission_config is None:
        mission_config = MissionConfig(
            mission_id='test_mission',
            loop=False,
            stop_on_task_failure=True,
            waypoints=[],
        )

    commander = object.__new__(MissionCommander)
    logger = FakeLogger()
    setattr(commander, 'get_logger', lambda: logger)
    setattr(commander, 'mission_config', mission_config)
    setattr(commander, 'navigator', SimpleNamespace())
    setattr(commander, 'blackboard', {})
    setattr(commander, 'mission_manager', MissionManager(FakeLogger()))
    commander.mission_manager.create_mission(mission_config)
    setattr(commander, 'status_reporter', FakeStatusReporter())
    return commander


def make_spec(name, task_type, params):
    return TaskSpec(name=name, task_type=task_type, params=params)


def configure(plugin, node):
    plugin.configure(node)
    return plugin


def load_yaml(path):
    assert path.exists()
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def all_tasks(mission):
    return [task for waypoint in mission['waypoints'] for task in waypoint.get('tasks', [])]


def task_named(mission, task_name):
    return next(task for task in all_tasks(mission) if task['name'] == task_name)


def waypoint_named(mission, waypoint_name):
    return next(waypoint for waypoint in mission['waypoints'] if waypoint['name'] == waypoint_name)


class FakeFuture:
    def __init__(self, done, result=None):
        self._done = done
        self._result = result

    def done(self):
        return self._done

    def result(self):
        return self._result


class FakeReadyNavigator:
    def is_ready(self):
        return True


class FakeRegistry(TaskPluginRegistry):
    def has(self, task_type):
        return True

    def available_types(self):
        return [
            'classify_place',
            'detect_flame',
            'grasp_item',
            'track_flame',
            'wait',
        ]


class FakeGraphNode(FakeNode):
    def __init__(self, services=None, topics=None):
        super().__init__()
        self.services = services or []
        self.topics = topics or []

    def get_service_names_and_types(self):
        return [(name, ['std_srvs/srv/SetBool']) for name in self.services]

    def get_topic_names_and_types(self):
        return [(name, ['venom_manipulation_interfaces/msg/FlameTrackerStatus']) for name in self.topics]


def make_flame_preflight_mission():
    return MissionConfig(
        mission_id='flame_preflight_mission',
        loop=False,
        stop_on_task_failure=True,
        waypoints=[
            WaypointSpec(
                name='start_flame_tracking',
                frame_id='map',
                x=0.0,
                y=0.0,
                yaw=0.0,
                skip_navigation=True,
                tasks=[
                    TaskSpec(
                        name='start_flame_tracking_for_moving_segment',
                        task_type='track_flame',
                        params={
                            'backend': 'service',
                            'mode': 'start',
                            'service_name': '/flame_arm_tracker/set_enabled',
                            'wait_until_tracking': True,
                            'status_topic': '/flame_arm_tracker/status',
                        },
                    ),
                    TaskSpec(
                        name='detect_flame_at_point_3',
                        task_type='detect_flame',
                        params={
                            'backend': 'topic',
                            'detection_topic': '/perception/detections_2d_array',
                        },
                    ),
                ],
            )
        ],
    )


def test_resolve_execute_task_value_accepts_constant_names_and_numeric_values():
    from venom_mission_commander.arm_task_client import resolve_execute_task_value

    fake_execute_task = SimpleNamespace(
        Goal=SimpleNamespace(
            PICK_AND_PLACE_LATEST_TARGET=4,
            CLASSIFY_PLATFORM_TO_COLOR_BOXES=5,
            START_FLAME_TRACKING=7,
        )
    )

    assert resolve_execute_task_value('PICK_AND_PLACE_LATEST_TARGET', fake_execute_task) == 4
    assert resolve_execute_task_value(7, fake_execute_task) == 7

    with pytest.raises(ValueError, match='unknown ExecuteTask goal constant'):
        resolve_execute_task_value('UNKNOWN_TASK', fake_execute_task)


def test_best_detection_selects_highest_confidence_matching_class():
    from venom_mission_commander.arm_task_client import best_detection_dict

    detections = [
        SimpleNamespace(class_name='person', confidence=0.99),
        SimpleNamespace(
            class_name='fire',
            confidence=0.61,
            center_x=100.0,
            center_y=120.0,
            size_x=30.0,
            size_y=40.0,
        ),
        SimpleNamespace(
            class_name='fire',
            confidence=0.82,
            center_x=220.0,
            center_y=130.0,
            size_x=50.0,
            size_y=60.0,
        ),
    ]

    detection = best_detection_dict(detections, target_class='fire', min_confidence=0.7)

    assert detection == {
        'class': 'fire',
        'confidence': 0.82,
        'bbox': [220.0, 130.0, 50.0, 60.0],
    }


def test_tracker_status_ready_requires_tracking_and_requested_target():
    from venom_mission_commander.arm_task_client import tracker_status_is_ready

    ready_status = SimpleNamespace(
        enabled=True,
        mode='tracking',
        target_acquired=True,
        target_class_name='fire',
    )

    assert tracker_status_is_ready(
        ready_status,
        target_class='fire',
        require_target_acquired=True,
    )
    assert tracker_status_is_ready(
        {
            'enabled': True,
            'mode': 'tracking',
            'target_acquired': False,
            'target_class_name': 'fire',
        },
        target_class='fire',
        require_target_acquired=False,
    )
    assert not tracker_status_is_ready(
        SimpleNamespace(
            enabled=True,
            mode='observe',
            target_acquired=True,
            target_class_name='fire',
        ),
        target_class='fire',
        require_target_acquired=True,
    )
    assert not tracker_status_is_ready(
        SimpleNamespace(
            enabled=True,
            mode='tracking',
            target_acquired=True,
            target_class_name='flame_picture',
        ),
        target_class='fire',
        require_target_acquired=True,
    )


def test_flame_tracking_disables_tracker_when_tracking_wait_fails(monkeypatch):
    from venom_mission_commander import arm_task_client

    calls = []

    def fake_set_bool(node, service_name, enabled, service_wait_timeout_sec, call_timeout_sec):
        calls.append(enabled)
        return SimpleNamespace(success=True, message=f'enabled={enabled}'), None

    def failing_wait(node, seconds):
        raise RuntimeError('tracking loop failed')

    monkeypatch.setattr(arm_task_client, 'call_set_bool_service', fake_set_bool)
    monkeypatch.setattr(arm_task_client, 'wait_with_spin', failing_wait)
    context = make_context({'flame_detection': {'class': 'fire', 'confidence': 0.91}})

    result = arm_task_client.execute_flame_tracking_task(
        context.node,
        context,
        make_spec(
            'track_flame_at_point_3',
            'track_flame',
            {
                'service_name': '/flame_arm_tracker/set_enabled',
                'tracking_duration_sec': 3.0,
                'disable_on_exit': True,
            },
        ),
    )

    assert calls == [True, False]
    assert result.success is False
    assert result.message == 'flame tracking interrupted: tracking loop failed'
    assert context.blackboard['last_flame_tracking']['success'] is False


def test_flame_tracking_start_mode_enables_tracker_without_wait_or_disable(monkeypatch):
    from venom_mission_commander import arm_task_client

    calls = []

    def fake_set_bool(node, service_name, enabled, service_wait_timeout_sec, call_timeout_sec):
        calls.append(enabled)
        return SimpleNamespace(success=True, message=f'enabled={enabled}'), None

    def unexpected_wait(node, seconds):
        raise AssertionError('start mode should not block on tracking wait')

    monkeypatch.setattr(arm_task_client, 'call_set_bool_service', fake_set_bool)
    monkeypatch.setattr(arm_task_client, 'wait_with_spin', unexpected_wait)
    context = make_context()

    result = arm_task_client.execute_flame_tracking_task(
        context.node,
        context,
        make_spec(
            'start_flame_tracking_for_moving_segment',
            'track_flame',
            {
                'service_name': '/flame_arm_tracker/set_enabled',
                'mode': 'start',
                'require_detection': False,
            },
        ),
    )

    assert calls == [True]
    assert result.success is True
    assert result.message == 'flame tracking started'
    assert context.blackboard['flame_tracking_active'] is True
    assert context.blackboard['flame_tracking_state'] == 'active'
    assert context.blackboard['flame_tracking_service_name'] == '/flame_arm_tracker/set_enabled'
    assert context.blackboard['last_flame_tracking']['mode'] == 'start'


def test_flame_tracking_start_waits_for_ready_status_before_returning(monkeypatch):
    from venom_mission_commander import arm_task_client

    events = []
    ready_status = {
        'enabled': True,
        'mode': 'tracking',
        'target_acquired': True,
        'target_class_name': 'fire',
    }

    def fake_set_bool(node, service_name, enabled, service_wait_timeout_sec, call_timeout_sec):
        events.append(('set_bool', enabled))
        return SimpleNamespace(success=True, message=f'enabled={enabled}'), None

    def fake_wait_ready(node, config):
        events.append(
            (
                'wait_ready',
                config.status_topic,
                config.ready_timeout_sec,
                config.require_target_acquired,
                config.target_class,
            )
        )
        return ready_status, None

    monkeypatch.setattr(arm_task_client, 'call_set_bool_service', fake_set_bool)
    monkeypatch.setattr(arm_task_client, 'wait_for_flame_tracker_ready', fake_wait_ready)
    context = make_context()

    result = arm_task_client.execute_flame_tracking_task(
        context.node,
        context,
        make_spec(
            'start_flame_tracking_for_moving_segment',
            'track_flame',
            {
                'service_name': '/flame_arm_tracker/set_enabled',
                'mode': 'start',
                'wait_until_tracking': True,
                'require_target_acquired': True,
                'ready_timeout_sec': 2.0,
                'status_topic': '/flame_arm_tracker/status',
                'target_class': 'fire',
            },
        ),
    )

    assert result.success is True
    assert events == [
        ('set_bool', True),
        ('wait_ready', '/flame_arm_tracker/status', 2.0, True, 'fire'),
    ]
    assert context.blackboard['flame_tracking_ready'] is True
    assert context.blackboard['last_flame_tracking']['ready_status'] == ready_status


def test_flame_tracking_start_ready_timeout_forces_stop(monkeypatch):
    from venom_mission_commander import arm_task_client

    events = []

    def fake_set_bool(node, service_name, enabled, service_wait_timeout_sec, call_timeout_sec):
        events.append(('set_bool', enabled))
        return SimpleNamespace(success=True, message=f'enabled={enabled}'), None

    def fake_wait_ready(node, config):
        events.append(('wait_ready', config.status_topic))
        return None, 'flame tracker ready timeout on /flame_arm_tracker/status'

    monkeypatch.setattr(arm_task_client, 'call_set_bool_service', fake_set_bool)
    monkeypatch.setattr(arm_task_client, 'wait_for_flame_tracker_ready', fake_wait_ready)
    context = make_context()

    result = arm_task_client.execute_flame_tracking_task(
        context.node,
        context,
        make_spec(
            'start_flame_tracking_for_moving_segment',
            'track_flame',
            {
                'service_name': '/flame_arm_tracker/set_enabled',
                'mode': 'start',
                'wait_until_tracking': True,
                'require_target_acquired': True,
                'ready_timeout_sec': 2.0,
                'status_topic': '/flame_arm_tracker/status',
                'target_class': 'fire',
            },
        ),
    )

    assert result.success is False
    assert result.message == 'flame tracker ready timeout on /flame_arm_tracker/status'
    assert events == [
        ('set_bool', True),
        ('wait_ready', '/flame_arm_tracker/status'),
        ('set_bool', False),
    ]
    assert context.blackboard['flame_tracking_ready'] is False
    assert context.blackboard['flame_tracking_state'] == 'inactive'
    assert context.blackboard['last_flame_tracking']['ready_wait_error'] == result.message


def test_flame_tracker_interface_and_config_declare_status_topic_and_message():
    interfaces_dir = VENOM_VNV_DIR / 'manipulation' / 'venom_manipulation_interfaces'
    status_msg = interfaces_dir / 'msg' / 'FlameTrackerStatus.msg'
    cmake_path = interfaces_dir / 'CMakeLists.txt'
    flame_config_path = VENOM_VNV_DIR / 'manipulation' / 'flame_arm_tracker' / 'config' / 'flame_tracking.yaml'
    tracker_source_path = (
        VENOM_VNV_DIR
        / 'manipulation'
        / 'flame_arm_tracker'
        / 'flame_arm_tracker'
        / 'flame_arm_tracker_node.py'
    )

    assert status_msg.exists()
    assert 'enabled' in status_msg.read_text(encoding='utf-8')
    assert 'target_acquired' in status_msg.read_text(encoding='utf-8')
    assert 'FlameTrackerStatus.msg' in cmake_path.read_text(encoding='utf-8')
    assert 'status_topic: /flame_arm_tracker/status' in flame_config_path.read_text(
        encoding='utf-8'
    )
    assert 'FlameTrackerStatus' in tracker_source_path.read_text(encoding='utf-8')


def test_startup_checker_preflights_flame_tracking_endpoints():
    from venom_mission_commander.startup_checks import StartupChecker

    checker = StartupChecker(
        mission_config=make_flame_preflight_mission(),
        registry=FakeRegistry(),
        navigator=FakeReadyNavigator(),
        navigator_ready_timeout_sec=1.0,
        node=FakeGraphNode(
            services=['/flame_arm_tracker/set_enabled'],
            topics=['/flame_arm_tracker/status', '/perception/detections_2d_array'],
        ),
    )

    results = checker.run()
    preflight = next(result for result in results if result.name == 'real_backend_preflight')

    assert preflight.success is True
    assert preflight.data['missing_services'] == []
    assert preflight.data['missing_topics'] == []


def test_startup_checker_fails_when_flame_tracking_endpoints_are_missing():
    from venom_mission_commander.startup_checks import StartupChecker

    checker = StartupChecker(
        mission_config=make_flame_preflight_mission(),
        registry=FakeRegistry(),
        navigator=FakeReadyNavigator(),
        navigator_ready_timeout_sec=1.0,
        node=FakeGraphNode(
            services=[],
            topics=['/perception/detections_2d_array'],
        ),
    )

    results = checker.run()
    preflight = next(result for result in results if result.name == 'real_backend_preflight')

    assert preflight.success is False
    assert '/flame_arm_tracker/set_enabled' in preflight.message
    assert '/flame_arm_tracker/status' in preflight.message
    assert preflight.data['missing_services'] == ['/flame_arm_tracker/set_enabled']
    assert preflight.data['missing_topics'] == ['/flame_arm_tracker/status']


def test_flame_tracking_start_failure_attempts_forced_stop_and_leaves_unknown(monkeypatch):
    from venom_mission_commander import arm_task_client

    calls = []

    def fake_set_bool(node, service_name, enabled, service_wait_timeout_sec, call_timeout_sec):
        calls.append(enabled)
        if enabled:
            return None, 'service call timeout: /flame_arm_tracker/set_enabled'
        return None, 'service unavailable: /flame_arm_tracker/set_enabled'

    monkeypatch.setattr(arm_task_client, 'call_set_bool_service', fake_set_bool)
    context = make_context()

    result = arm_task_client.execute_flame_tracking_task(
        context.node,
        context,
        make_spec(
            'start_flame_tracking_for_moving_segment',
            'track_flame',
            {
                'service_name': '/flame_arm_tracker/set_enabled',
                'mode': 'start',
            },
        ),
    )

    assert calls == [True, False]
    assert result.success is False
    assert result.message == 'service call timeout: /flame_arm_tracker/set_enabled'
    assert context.blackboard['flame_tracking_state'] == 'unknown'
    assert context.blackboard['last_flame_tracking']['stop_after_start_failure_success'] is False


def test_flame_tracking_stop_mode_disables_tracker_without_detection(monkeypatch):
    from venom_mission_commander import arm_task_client

    calls = []

    def fake_set_bool(node, service_name, enabled, service_wait_timeout_sec, call_timeout_sec):
        calls.append(enabled)
        return SimpleNamespace(success=True, message=f'enabled={enabled}'), None

    def unexpected_wait(node, seconds):
        raise AssertionError('stop mode should not block on tracking wait')

    monkeypatch.setattr(arm_task_client, 'call_set_bool_service', fake_set_bool)
    monkeypatch.setattr(arm_task_client, 'wait_with_spin', unexpected_wait)
    context = make_context({'flame_tracking_active': True})

    result = arm_task_client.execute_flame_tracking_task(
        context.node,
        context,
        make_spec(
            'stop_flame_tracking_at_point_3',
            'track_flame',
            {
                'service_name': '/flame_arm_tracker/set_enabled',
                'mode': 'stop',
            },
        ),
    )

    assert calls == [False]
    assert result.success is True
    assert result.message == 'flame tracking stopped'
    assert context.blackboard['flame_tracking_active'] is False
    assert context.blackboard['flame_tracking_state'] == 'inactive'
    assert context.blackboard['last_flame_tracking']['mode'] == 'stop'


def test_execute_task_action_reports_unconfirmed_cancel_after_result_timeout(monkeypatch):
    from venom_mission_commander import arm_task_client

    class FakeExecuteTask:
        class Goal:
            PICK_AND_PLACE_LATEST_TARGET = 4

            def __init__(self):
                self.task_type = 0

    action_module = ModuleType('venom_manipulation_interfaces.action')
    setattr(action_module, 'ExecuteTask', FakeExecuteTask)
    monkeypatch.setitem(sys.modules, 'venom_manipulation_interfaces', ModuleType('venom_manipulation_interfaces'))
    monkeypatch.setitem(sys.modules, 'venom_manipulation_interfaces.action', action_module)

    class FakeGoalHandle:
        accepted = True

        def __init__(self):
            self.cancel_requested = False
            self.result_future = FakeFuture(False)

        def get_result_async(self):
            return self.result_future

        def cancel_goal_async(self):
            self.cancel_requested = True
            return FakeFuture(True, SimpleNamespace(goals_canceling=[SimpleNamespace()]))

    goal_handle = FakeGoalHandle()

    class FakeActionClient:
        def __init__(self, node, action_type, action_name):
            self.action_name = action_name

        def wait_for_server(self, timeout_sec):
            return True

        def send_goal_async(self, goal):
            return FakeFuture(True, goal_handle)

        def destroy(self):
            return None

    monkeypatch.setattr(arm_task_client, 'ActionClient', FakeActionClient)
    monkeypatch.setattr(
        arm_task_client,
        'spin_until_future_done',
        lambda node, future, deadline: future.done(),
    )
    config = arm_task_client.ManipulationActionConfig(
        action_name='/manipulation/execute_task',
        task_type_name='PICK_AND_PLACE_LATEST_TARGET',
        task_type_value=None,
        timeout_sec=0.1,
        server_wait_timeout_sec=0.1,
        retry_count=0,
        retry_backoff_sec=0.0,
        output_key='grasped_object',
    )

    context = make_context()
    result, error = arm_task_client.call_execute_task_action(context.node, config)

    assert result is None
    assert error == 'action result timeout: /manipulation/execute_task; cancel not confirmed'
    assert goal_handle.cancel_requested is True


def test_manipulation_action_does_not_retry_when_cancel_is_unconfirmed(monkeypatch):
    from venom_mission_commander import arm_task_client

    calls = []

    def fake_call(node, config):
        calls.append(config.action_name)
        return None, 'action result timeout: /manipulation/execute_task; cancel not confirmed'

    monkeypatch.setattr(arm_task_client, 'call_execute_task_action', fake_call)
    monkeypatch.setattr(
        arm_task_client,
        'stop_active_flame_tracking_if_needed',
        lambda node, blackboard, reason, force=False: (True, None),
    )
    monkeypatch.setattr(arm_task_client, 'sleep_non_negative', lambda seconds: None)
    context = make_context()
    spec = make_spec(
        'grasp_item_at_point_1',
        'grasp_item',
        {
            'backend': 'action',
            'retry_count': 2,
            'retry_backoff_sec': 0.0,
        },
    )

    result = arm_task_client.execute_manipulation_action_task(
        context.node,
        context,
        spec,
        'PICK_AND_PLACE_LATEST_TARGET',
        'grasped_object',
    )

    assert result.success is False
    assert result.message == 'action result timeout: /manipulation/execute_task; cancel not confirmed'
    assert calls == ['/manipulation/execute_task']


def test_manipulation_action_stops_active_flame_tracking_before_sending_goal(monkeypatch):
    from venom_mission_commander import arm_task_client

    events = []

    def fake_stop(node, blackboard, reason, force=False):
        events.append(('stop', reason, force))
        blackboard['flame_tracking_active'] = False
        return True, None

    def fake_call(node, config):
        events.append(('action', config.task_type_name))
        result = SimpleNamespace(success=True, stage_reached=8, error_code=0, message='done')
        return result, None

    monkeypatch.setattr(arm_task_client, 'stop_active_flame_tracking_if_needed', fake_stop)
    monkeypatch.setattr(arm_task_client, 'call_execute_task_action', fake_call)
    context = make_context({'flame_tracking_active': True})

    result = arm_task_client.execute_manipulation_action_task(
        context.node,
        context,
        make_spec('classify_and_place_object', 'classify_place', {'backend': 'action'}),
        'CLASSIFY_PLATFORM_TO_COLOR_BOXES',
        'last_placement',
    )

    assert result.success is True
    assert events == [
        ('stop', 'before manipulation action', True),
        ('action', 'CLASSIFY_PLATFORM_TO_COLOR_BOXES'),
    ]


def test_manipulation_action_forces_flame_tracking_stop_even_when_blackboard_inactive(monkeypatch):
    from venom_mission_commander import arm_task_client

    events = []

    def fake_stop(node, blackboard, reason, force=False):
        events.append(('stop', reason, force, dict(blackboard)))
        return True, None

    def fake_call(node, config):
        events.append(('action', config.task_type_name))
        result = SimpleNamespace(success=True, stage_reached=8, error_code=0, message='done')
        return result, None

    monkeypatch.setattr(arm_task_client, 'stop_active_flame_tracking_if_needed', fake_stop)
    monkeypatch.setattr(arm_task_client, 'call_execute_task_action', fake_call)
    context = make_context({})

    result = arm_task_client.execute_manipulation_action_task(
        context.node,
        context,
        make_spec('grasp_item_at_point_1', 'grasp_item', {'backend': 'action'}),
        'PICK_AND_PLACE_LATEST_TARGET',
        'grasped_object',
    )

    assert result.success is True
    assert events == [
        ('stop', 'before manipulation action', True, {}),
        ('action', 'PICK_AND_PLACE_LATEST_TARGET'),
    ]


def test_manipulation_action_fails_when_forced_flame_stop_fails(monkeypatch):
    from venom_mission_commander import arm_task_client

    def fake_stop(node, blackboard, reason, force=False):
        assert force is True
        return False, 'service call timeout: /flame_arm_tracker/set_enabled'

    def unexpected_action(node, config):
        raise AssertionError('manipulation action must not start if tracker stop fails')

    monkeypatch.setattr(arm_task_client, 'stop_active_flame_tracking_if_needed', fake_stop)
    monkeypatch.setattr(arm_task_client, 'call_execute_task_action', unexpected_action)
    context = make_context({})

    result = arm_task_client.execute_manipulation_action_task(
        context.node,
        context,
        make_spec('classify_and_place_object', 'classify_place', {'backend': 'action'}),
        'CLASSIFY_PLATFORM_TO_COLOR_BOXES',
        'last_placement',
    )

    assert result.success is False
    assert result.message == (
        'active flame tracking stop failed before manipulation action: '
        'service call timeout: /flame_arm_tracker/set_enabled'
    )


def test_grasp_action_backend_calls_manipulation_action_without_detected_item(monkeypatch):
    from venom_mission_commander import task_plugins

    calls = []

    def fake_execute(node, context, spec, default_task_type_name, output_key):
        calls.append((default_task_type_name, output_key, dict(spec.params)))
        context.blackboard[output_key] = {'backend': 'action', 'task_type': default_task_type_name}
        return TaskExecutionResult(True, 'manipulation task succeeded', context.blackboard[output_key])

    monkeypatch.setattr(task_plugins, 'execute_manipulation_action_task', fake_execute)
    context = make_context()
    plugin = configure(GraspItemTaskPlugin(), context.node)

    result = plugin.execute(
        context,
        make_spec(
            'grasp_item_at_point_1',
            'grasp_item',
            {'backend': 'action'},
        ),
    )

    assert result.success is True
    assert calls == [('PICK_AND_PLACE_LATEST_TARGET', 'grasped_object', {'backend': 'action'})]
    assert context.blackboard['grasped_object']['task_type'] == 'PICK_AND_PLACE_LATEST_TARGET'


def test_classify_place_action_backend_calls_classification_action_without_source(monkeypatch):
    from venom_mission_commander import task_plugins

    calls = []

    def fake_execute(node, context, spec, default_task_type_name, output_key):
        calls.append((default_task_type_name, output_key, dict(spec.params)))
        context.blackboard[output_key] = {'backend': 'action', 'task_type': default_task_type_name}
        return TaskExecutionResult(True, 'classification task succeeded', context.blackboard[output_key])

    monkeypatch.setattr(task_plugins, 'execute_manipulation_action_task', fake_execute)
    context = make_context()
    plugin = configure(ClassifyPlaceTaskPlugin(), context.node)

    result = plugin.execute(
        context,
        make_spec(
            'classify_and_place_object',
            'classify_place',
            {'backend': 'action'},
        ),
    )

    assert result.success is True
    assert calls == [
        ('CLASSIFY_PLATFORM_TO_COLOR_BOXES', 'last_placement', {'backend': 'action'})
    ]
    assert context.blackboard['last_placement']['task_type'] == 'CLASSIFY_PLATFORM_TO_COLOR_BOXES'


def test_detect_flame_topic_backend_writes_detection_to_blackboard(monkeypatch):
    from venom_mission_commander import task_plugins

    detection = {'class': 'fire', 'confidence': 0.91, 'bbox': [320.0, 180.0, 80.0, 120.0]}

    def fake_wait(node, params):
        assert params['backend'] == 'topic'
        return TaskExecutionResult(True, 'flame detected', detection)

    monkeypatch.setattr(task_plugins, 'wait_for_flame_detection_task', fake_wait)
    context = make_context()
    plugin = configure(DetectFlameTaskPlugin(), context.node)

    result = plugin.execute(
        context,
        make_spec(
            'detect_flame_at_point_3',
            'detect_flame',
            {'backend': 'topic', 'target_class': 'fire'},
        ),
    )

    assert result.success is True
    assert context.blackboard['flame_detection'] == detection


def test_track_flame_service_backend_requires_detection_before_alignment(monkeypatch):
    from venom_mission_commander import task_plugins

    called = False

    def fake_track(node, context, spec):
        nonlocal called
        called = True
        return TaskExecutionResult(True, 'tracking completed')

    monkeypatch.setattr(task_plugins, 'execute_flame_tracking_task', fake_track)
    context = make_context()
    plugin = configure(TrackFlameTaskPlugin(), context.node)

    result = plugin.execute(
        context,
        make_spec(
            'track_flame_at_point_3',
            'track_flame',
            {'backend': 'service', 'source': 'flame_detection'},
        ),
    )

    assert result.success is False
    assert result.message == 'missing flame detection: flame_detection'
    assert called is False


def test_track_flame_start_backend_can_enable_without_static_detection(monkeypatch):
    from venom_mission_commander import task_plugins

    calls = []

    def fake_track(node, context, spec):
        calls.append(dict(spec.params))
        context.blackboard['flame_tracking_active'] = True
        return TaskExecutionResult(True, 'flame tracking started')

    monkeypatch.setattr(task_plugins, 'execute_flame_tracking_task', fake_track)
    context = make_context()
    plugin = configure(TrackFlameTaskPlugin(), context.node)

    result = plugin.execute(
        context,
        make_spec(
            'start_flame_tracking_for_moving_segment',
            'track_flame',
            {
                'backend': 'service',
                'mode': 'start',
                'require_detection': False,
            },
        ),
    )

    assert result.success is True
    assert calls == [{'backend': 'service', 'mode': 'start', 'require_detection': False}]


def test_track_flame_service_backend_aligns_without_grasping(monkeypatch):
    from venom_mission_commander import task_plugins
    calls = []

    def fake_track(node, context, spec):
        calls.append(dict(spec.params))
        tracking = {
            'backend': 'service',
            'service_name': spec.params['service_name'],
            'tracking_duration_sec': spec.params['tracking_duration_sec'],
            'disable_on_exit': spec.params['disable_on_exit'],
        }
        context.blackboard['last_flame_tracking'] = tracking
        return TaskExecutionResult(True, 'flame tracking completed', tracking)

    monkeypatch.setattr(task_plugins, 'execute_flame_tracking_task', fake_track)
    context = make_context({'flame_detection': {'class': 'fire', 'confidence': 0.91}})
    plugin = configure(TrackFlameTaskPlugin(), context.node)

    result = plugin.execute(
        context,
        make_spec(
            'track_flame_at_point_3',
            'track_flame',
            {
                'backend': 'service',
                'service_name': '/flame_arm_tracker/set_enabled',
                'tracking_duration_sec': 3.0,
                'disable_on_exit': True,
            },
        ),
    )

    assert result.success is True
    assert calls[0]['service_name'] == '/flame_arm_tracker/set_enabled'
    assert 'task_type_name' not in calls[0]
    assert 'gripper' not in calls[0]
    assert context.blackboard['last_flame_tracking']['disable_on_exit'] is True


def test_mission_startup_forces_tracker_stop_before_startup_checks(monkeypatch):
    from venom_mission_commander import mission_commander

    events = []
    mission_config = MissionConfig(
        mission_id='tracks_fire_during_motion',
        loop=False,
        stop_on_task_failure=True,
        waypoints=[
            WaypointSpec(
                name='start_tracking',
                frame_id='map',
                x=0.0,
                y=0.0,
                yaw=0.0,
                skip_navigation=True,
                tasks=[
                    TaskSpec(
                        name='start_flame_tracking',
                        task_type='track_flame',
                        params={'backend': 'service', 'mode': 'start'},
                    )
                ],
            )
        ],
    )
    commander = make_test_commander(mission_config)
    monkeypatch.setattr(mission_commander.rclpy, 'ok', lambda: True)
    commander.stop_active_flame_tracking = lambda reason, force=False: events.append(
        ('stop', reason, force)
    ) or True
    commander.run_startup_checks = lambda: events.append(('startup_checks',)) or True
    commander.run_once = lambda: events.append(('run_once',)) or False

    result = commander.run()

    assert result is False
    assert events[:2] == [
        ('stop', 'mission startup', True),
        ('startup_checks',),
    ]


def test_mission_completion_fails_when_tracker_stop_fails(monkeypatch):
    from venom_mission_commander import mission_commander

    commander = make_test_commander()
    monkeypatch.setattr(mission_commander.rclpy, 'ok', lambda: True)
    commander.run_startup_checks = lambda: True
    commander.run_once = lambda: True
    commander.stop_active_flame_tracking = lambda reason, force=False: False

    result = commander.run()

    assert result is False
    assert commander.mission_manager.state == MissionState.FAILED
    assert commander.mission_manager.state_data['failure_reason'] == (
        'failed to stop flame tracking before mission completion'
    )


def test_navigation_attempt_failure_stops_tracker_before_recovery():
    events = []

    class FakeNavigator:
        def go_to_waypoint(self, waypoint):
            events.append(('go', waypoint.name))

        def wait_until_done(self, timeout_sec):
            events.append(('wait', timeout_sec))
            return False

        def cancel(self):
            events.append(('cancel',))
            return True

        def recover(self):
            events.append(('recover',))
            return True

    waypoint = WaypointSpec(
        name='moving_with_flame_tracking',
        frame_id='map',
        x=1.0,
        y=2.0,
        yaw=0.0,
        nav_timeout_sec=4.0,
        retry_count=1,
    )
    commander = make_test_commander()
    setattr(commander, 'navigator', FakeNavigator())
    commander.stop_active_flame_tracking = lambda reason, force=False: events.append(
        ('stop', reason, force)
    ) or True
    setattr(commander, 'mission_uses_service_flame_tracking', lambda: True)

    result = commander.navigate_to_waypoint(waypoint)

    assert result is False
    assert events[:5] == [
        ('go', 'moving_with_flame_tracking'),
        ('wait', 4.0),
        ('stop', 'navigation attempt failed', True),
        ('cancel',),
        ('recover',),
    ]


def test_craic_arm_mission_uses_real_arm_backends_and_no_flame_grasp():
    mission = load_yaml(CRAIC_ARM_MISSION_PATH)

    grasp_task = task_named(mission, 'grasp_item_at_point_1')
    detect_flame_task = task_named(mission, 'detect_flame_at_point_3')
    start_flame_task = task_named(mission, 'start_flame_tracking_for_moving_segment')
    observe_wait_task = task_named(mission, 'wait_flame_tracker_observe_pose')
    stop_flame_task = task_named(mission, 'stop_flame_tracking_at_point_3')
    classify_task = task_named(mission, 'classify_and_place_object')
    task_point_2 = waypoint_named(mission, 'task_point_2_meter_voice')
    task_point_3 = waypoint_named(mission, 'task_point_3_flame_tracking')

    assert grasp_task['type'] == 'grasp_item'
    assert grasp_task['backend'] == 'action'
    assert grasp_task['task_type_name'] == 'PICK_AND_PLACE_LATEST_TARGET'

    assert detect_flame_task['type'] == 'detect_flame'
    assert detect_flame_task['backend'] == 'topic'
    assert detect_flame_task['target_class'] == 'fire'

    assert start_flame_task in task_point_2['tasks']
    assert start_flame_task['type'] == 'track_flame'
    assert start_flame_task['backend'] == 'service'
    assert start_flame_task['mode'] == 'start'
    assert start_flame_task['require_detection'] is False
    assert start_flame_task['service_name'] == '/flame_arm_tracker/set_enabled'
    assert start_flame_task['status_topic'] == '/flame_arm_tracker/status'
    assert start_flame_task['wait_until_tracking'] is True
    assert start_flame_task['require_target_acquired'] is True
    assert start_flame_task['ready_timeout_sec'] == 4.0
    assert start_flame_task['target_class'] == 'fire'
    assert observe_wait_task['seconds'] >= 2.5
    assert 'target_class_name:=fire' in CRAIC_ARM_MISSION_PATH.read_text(encoding='utf-8')

    assert stop_flame_task in task_point_3['tasks']
    assert stop_flame_task['type'] == 'track_flame'
    assert stop_flame_task['backend'] == 'service'
    assert stop_flame_task['mode'] == 'stop'
    assert stop_flame_task['service_name'] == '/flame_arm_tracker/set_enabled'
    assert 'task_type_name' not in start_flame_task
    assert 'task_type_name' not in stop_flame_task
    assert 'gripper' not in start_flame_task
    assert 'gripper' not in stop_flame_task

    assert classify_task['type'] == 'classify_place'
    assert classify_task['backend'] == 'action'
    assert classify_task['task_type_name'] == 'CLASSIFY_PLATFORM_TO_COLOR_BOXES'
