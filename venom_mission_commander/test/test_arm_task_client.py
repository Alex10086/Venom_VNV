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
VERIFY_POINT1_GRASP_MISSION_PATH = PACKAGE_DIR / 'config' / 'verify_point1_grasp.yaml'
VERIFY_POINT3_FLAME_MISSION_PATH = PACKAGE_DIR / 'config' / 'verify_point3_flame_tracking.yaml'
VERIFY_POINT4_CLASSIFY_PLACE_MISSION_PATH = PACKAGE_DIR / 'config' / 'verify_point4_classify_place.yaml'
FLAME_DETECTION_ARRAY_TOPIC = '/perception/flame/detections_2d_array'
DUAL_TARGET_PAYLOAD_TASK = 'REPEAT_VISUAL_PICK_TO_PAYLOAD'


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


def test_terminal_navigation_failure_cleans_up_task_plugins():
    commander = make_test_commander()
    events = []
    commander.registry = SimpleNamespace(cleanup=lambda: events.append('cleanup') or True)
    commander.navigator.cancel = lambda: True
    commander.stop_active_flame_tracking = lambda *args, **kwargs: True
    commander.mission_uses_service_flame_tracking = lambda: False

    commander.handle_navigation_failure(WaypointSpec('point4', 'map', 0.0, 0.0, 0.0))

    assert events == ['cleanup']


def test_shutdown_cleans_up_task_plugins():
    commander = make_test_commander()
    events = []
    commander.registry = SimpleNamespace(cleanup=lambda: events.append('cleanup') or True)
    commander.stop_active_flame_tracking = lambda *args, **kwargs: True
    commander.mission_uses_service_flame_tracking = lambda: False
    commander.cleanup_perception = lambda reason: True
    commander.navigator.shutdown = lambda: None
    commander.destroy_node = lambda: None

    commander.shutdown()

    assert events == ['cleanup']


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


def assert_dual_target_payload_grasp_task(task):
    assert task['type'] == 'grasp_item'
    assert task['backend'] == 'action'
    assert task['action_name'] == '/manipulation/execute_task'
    assert task['task_type_name'] == DUAL_TARGET_PAYLOAD_TASK
    assert task['timeout_sec'] >= 240.0
    assert task['retry_count'] == 0
    assert task['output_key'] == 'grasped_object'


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


class FakeDoneFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class CallbackGroupNode(FakeNode):
    def __init__(self, callback_group):
        super().__init__()
        self.communication_callback_group = callback_group
        self.clients = []
        self.subscriptions = []

    def create_client(self, service_type, service_name, callback_group=None):
        client = SimpleNamespace(
            service_type=service_type,
            service_name=service_name,
            callback_group=callback_group,
            wait_for_service=lambda timeout_sec: True,
            call_async=lambda request: FakeDoneFuture(SimpleNamespace(success=True, message='ok')),
        )
        self.clients.append(client)
        return client

    def create_subscription(self, msg_type, topic, callback, qos, callback_group=None):
        subscription = SimpleNamespace(
            msg_type=msg_type,
            topic=topic,
            callback=callback,
            qos=qos,
            callback_group=callback_group,
        )
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        return None


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
                            'detection_topic': FLAME_DETECTION_ARRAY_TOPIC,
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
            REPEAT_VISUAL_PICK_TO_PAYLOAD=6,
            START_FLAME_TRACKING=7,
        )
    )

    assert resolve_execute_task_value('PICK_AND_PLACE_LATEST_TARGET', fake_execute_task) == 4
    assert resolve_execute_task_value(DUAL_TARGET_PAYLOAD_TASK, fake_execute_task) == 6
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


def test_parse_flame_detection_config_defaults_to_flame_pipeline_topic():
    from venom_mission_commander.arm_task_client import parse_flame_detection_config

    config = parse_flame_detection_config({})

    assert config.detection_topic == FLAME_DETECTION_ARRAY_TOPIC


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


def test_arm_task_client_uses_node_callback_group_for_ros_entities(monkeypatch):
    from venom_mission_commander import arm_task_client

    callback_group = object()
    node = CallbackGroupNode(callback_group)
    action_client_calls = []

    fake_interfaces = ModuleType('venom_manipulation_interfaces')
    fake_action_module = ModuleType('venom_manipulation_interfaces.action')
    fake_msg_module = ModuleType('venom_manipulation_interfaces.msg')
    fake_service_module = ModuleType('std_srvs.srv')

    class FakeExecuteTask:
        class Goal:
            PICK_AND_PLACE_LATEST_TARGET = 4

            def __init__(self):
                self.task_type = 0

    class FakeSetBool:
        class Request:
            def __init__(self):
                self.data = False

    class FakeActionClient:
        def __init__(self, node_arg, action_type, action_name, callback_group=None):
            action_client_calls.append(
                {
                    'node': node_arg,
                    'action_type': action_type,
                    'action_name': action_name,
                    'callback_group': callback_group,
                }
            )

        def wait_for_server(self, timeout_sec):
            return False

    setattr(fake_action_module, 'ExecuteTask', FakeExecuteTask)
    setattr(fake_msg_module, 'FlameTrackerStatus', type('FlameTrackerStatus', (), {}))
    setattr(fake_msg_module, 'Detection2DArray', type('Detection2DArray', (), {}))
    setattr(fake_service_module, 'SetBool', FakeSetBool)
    monkeypatch.setitem(sys.modules, 'venom_manipulation_interfaces', fake_interfaces)
    monkeypatch.setitem(sys.modules, 'venom_manipulation_interfaces.action', fake_action_module)
    monkeypatch.setitem(sys.modules, 'venom_manipulation_interfaces.msg', fake_msg_module)
    monkeypatch.setitem(sys.modules, 'std_srvs.srv', fake_service_module)
    monkeypatch.setattr(arm_task_client, 'ActionClient', FakeActionClient)
    monkeypatch.setattr(arm_task_client, 'ros_ok', lambda: False)

    action_config = arm_task_client.ManipulationActionConfig(
        action_name='/manipulation/execute_task',
        task_type_name='PICK_AND_PLACE_LATEST_TARGET',
        task_type_value=None,
        timeout_sec=1.0,
        server_wait_timeout_sec=0.1,
        retry_count=0,
        retry_backoff_sec=0.0,
        output_key='grasped_object',
    )
    tracker_config = arm_task_client.FlameTrackingConfig(
        mode='start',
        service_name='/flame_arm_tracker/set_enabled',
        service_wait_timeout_sec=0.1,
        call_timeout_sec=0.1,
        tracking_duration_sec=0.0,
        disable_on_exit=True,
        output_key='last_flame_tracking',
        status_topic='/flame_arm_tracker/status',
        wait_until_tracking=True,
        require_target_acquired=True,
        ready_timeout_sec=0.1,
        target_class='fire',
    )

    arm_task_client.call_execute_task_action(node, action_config)
    arm_task_client.call_set_bool_service(
        node,
        '/flame_arm_tracker/set_enabled',
        True,
        0.1,
        0.1,
    )
    arm_task_client.wait_for_flame_tracker_ready(node, tracker_config)
    arm_task_client.wait_for_flame_detection_task(node, {'timeout_sec': 0.1})

    assert action_client_calls[0]['callback_group'] is callback_group
    assert node.clients[0].callback_group is callback_group
    assert [subscription.callback_group for subscription in node.subscriptions] == [
        callback_group,
        callback_group,
    ]


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
            topics=['/flame_arm_tracker/status', FLAME_DETECTION_ARRAY_TOPIC],
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
            topics=[FLAME_DETECTION_ARRAY_TOPIC],
        ),
    )

    results = checker.run()
    preflight = next(result for result in results if result.name == 'real_backend_preflight')

    assert preflight.success is False
    assert '/flame_arm_tracker/set_enabled' in preflight.message
    assert '/flame_arm_tracker/status' in preflight.message
    assert preflight.data['missing_services'] == ['/flame_arm_tracker/set_enabled']
    assert preflight.data['missing_topics'] == ['/flame_arm_tracker/status']


def test_startup_checker_defaults_flame_detection_to_flame_pipeline_topic():
    from venom_mission_commander.startup_checks import StartupChecker

    mission = MissionConfig(
        mission_id='default_flame_topic_mission',
        loop=False,
        stop_on_task_failure=True,
        waypoints=[
            WaypointSpec(
                name='detect_flame',
                frame_id='map',
                x=0.0,
                y=0.0,
                yaw=0.0,
                skip_navigation=True,
                tasks=[
                    TaskSpec(
                        name='detect_flame_at_point_3',
                        task_type='detect_flame',
                        params={'backend': 'topic'},
                    )
                ],
            )
        ],
    )
    checker = StartupChecker(
        mission_config=mission,
        registry=FakeRegistry(),
        navigator=FakeReadyNavigator(),
        navigator_ready_timeout_sec=1.0,
        node=FakeGraphNode(topics=[FLAME_DETECTION_ARRAY_TOPIC]),
    )

    results = checker.run()
    preflight = next(result for result in results if result.name == 'real_backend_preflight')

    assert preflight.success is True
    assert preflight.data['missing_topics'] == []


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
        def __init__(self, node, action_type, action_name, callback_group=None):
            self.action_name = action_name
            self.callback_group = callback_group

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
        ('stop', 'before manipulation action', False),
        ('action', 'CLASSIFY_PLATFORM_TO_COLOR_BOXES'),
    ]


def test_manipulation_action_skips_flame_tracking_stop_when_blackboard_inactive(monkeypatch):
    from venom_mission_commander import arm_task_client

    events = []

    def unexpected_set_bool(node, service_name, enabled, service_wait_timeout_sec, call_timeout_sec):
        raise AssertionError('inactive flame tracking must not require the flame tracker service')

    def fake_call(node, config):
        events.append(('action', config.task_type_name))
        result = SimpleNamespace(success=True, stage_reached=8, error_code=0, message='done')
        return result, None

    monkeypatch.setattr(arm_task_client, 'call_set_bool_service', unexpected_set_bool)
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
    assert events == [('action', 'PICK_AND_PLACE_LATEST_TARGET')]


def test_manipulation_action_fails_when_active_flame_stop_fails(monkeypatch):
    from venom_mission_commander import arm_task_client

    def fake_stop(node, blackboard, reason, force=False):
        assert blackboard['flame_tracking_active'] is True
        assert force is False
        return False, 'service call timeout: /flame_arm_tracker/set_enabled'

    def unexpected_action(node, config):
        raise AssertionError('manipulation action must not start if tracker stop fails')

    monkeypatch.setattr(arm_task_client, 'stop_active_flame_tracking_if_needed', fake_stop)
    monkeypatch.setattr(arm_task_client, 'call_execute_task_action', unexpected_action)
    context = make_context({'flame_tracking_active': True})

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


def test_main_runs_mission_with_background_multithreaded_executor(monkeypatch):
    from venom_mission_commander import mission_commander

    events = []

    class FakeSummaryReporter:
        def log_final_summary(self, mission_manager):
            events.append(('summary', mission_manager))

    class FakeCommander:
        def __init__(self):
            self.status_reporter = FakeSummaryReporter()
            self.mission_manager = 'mission-manager'
            self._venom_background_executor_active = False

        def configure(self):
            events.append(('configure', self._venom_background_executor_active))
            return True

        def run(self):
            events.append(('run', self._venom_background_executor_active))
            return True

        def shutdown(self):
            events.append(('commander_shutdown', self._venom_background_executor_active))

        def get_logger(self):
            return FakeLogger()

    commander = FakeCommander()

    class FakeExecutor:
        def __init__(self, num_threads=None):
            events.append(('executor_init', num_threads))

        def add_node(self, node):
            events.append(('executor_add_node', node is commander))

        def spin(self):
            events.append(('executor_spin', commander._venom_background_executor_active))

        def shutdown(self):
            events.append(('executor_shutdown', commander._venom_background_executor_active))

    class FakeThread:
        def __init__(self, target, daemon=False):
            self.target = target
            events.append(('thread_init', daemon))

        def start(self):
            events.append(('thread_start', commander._venom_background_executor_active))
            self.target()

        def join(self, timeout=None):
            events.append(('thread_join', timeout))

    monkeypatch.setattr(mission_commander.rclpy, 'init', lambda args=None: events.append(('init', args)))
    monkeypatch.setattr(mission_commander.rclpy, 'shutdown', lambda: events.append(('rclpy_shutdown',)))
    monkeypatch.setattr(mission_commander, 'MissionCommander', lambda: commander)
    monkeypatch.setattr(mission_commander, 'MultiThreadedExecutor', FakeExecutor, raising=False)
    monkeypatch.setattr(mission_commander, 'Thread', FakeThread, raising=False)

    with pytest.raises(SystemExit) as exc_info:
        mission_commander.main([])

    assert exc_info.value.code == 0
    assert events[:5] == [
        ('init', []),
        ('executor_init', 4),
        ('executor_add_node', True),
        ('thread_init', True),
        ('thread_start', True),
    ]
    assert ('executor_spin', True) in events
    assert ('run', True) in events
    assert ('commander_shutdown', True) in events
    assert ('executor_shutdown', False) in events
    assert ('rclpy_shutdown',) in events


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
    enable_pick_yolo_task = task_named(mission, 'enable_pick_yolo_for_point_1')
    disable_pick_yolo_task = task_named(mission, 'disable_pick_yolo_after_point_1')
    enable_digit_yolo_task = task_named(mission, 'enable_digit_yolo_for_point_2')
    disable_digit_yolo_task = task_named(mission, 'disable_digit_yolo_after_point_2')
    enable_flame_yolo_task = task_named(mission, 'enable_flame_yolo_for_moving_segment')
    disable_flame_yolo_task = task_named(mission, 'disable_flame_yolo_after_point_3')
    enable_classify_yolo_task = task_named(mission, 'enable_classification_yolo_for_point_4')
    disable_classify_yolo_task = task_named(mission, 'disable_classification_yolo_after_point_4')
    task_point_2 = waypoint_named(mission, 'task_point_2_meter_voice')
    task_point_3 = waypoint_named(mission, 'task_point_3_flame_tracking')

    assert_dual_target_payload_grasp_task(grasp_task)

    assert detect_flame_task['type'] == 'detect_flame'
    assert detect_flame_task['backend'] == 'topic'
    assert detect_flame_task['detection_topic'] == FLAME_DETECTION_ARRAY_TOPIC
    assert detect_flame_task['target_class'] == 'fire'

    assert start_flame_task in task_point_2['tasks']
    assert start_flame_task['type'] == 'track_flame'
    assert start_flame_task['backend'] == 'service'
    assert start_flame_task['mode'] == 'start'
    assert start_flame_task['require_detection'] is False
    assert start_flame_task['service_name'] == '/flame_arm_tracker/set_enabled'
    assert start_flame_task['status_topic'] == '/flame_arm_tracker/status'
    assert start_flame_task['wait_until_tracking'] is False
    assert start_flame_task['require_target_acquired'] is False
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

    assert_perception_control_task(
        enable_pick_yolo_task,
        '/pick_yolo_detector/set_enabled',
        'start',
    )
    assert_perception_control_task(
        disable_pick_yolo_task,
        '/pick_yolo_detector/set_enabled',
        'stop',
    )
    assert_perception_control_task(
        enable_digit_yolo_task,
        '/digit_yolo_detector/set_enabled',
        'start',
    )
    assert_perception_control_task(
        disable_digit_yolo_task,
        '/digit_yolo_detector/set_enabled',
        'stop',
    )
    assert_perception_control_task(
        enable_flame_yolo_task,
        '/flame_yolo_detector/set_enabled',
        'start',
    )
    assert_perception_control_task(
        disable_flame_yolo_task,
        '/flame_yolo_detector/set_enabled',
        'stop',
    )
    assert_perception_control_task(
        enable_classify_yolo_task,
        '/classification_yolo_detector/set_enabled',
        'start',
    )
    assert_perception_control_task(
        disable_classify_yolo_task,
        '/classification_yolo_detector/set_enabled',
        'stop',
    )


def assert_perception_control_task(task, service_name, mode):
    assert task['type'] == 'perception_control'
    assert task['service_name'] == service_name
    assert task['mode'] == mode


def test_verify_point1_grasp_uses_dual_target_payload_action():
    mission = load_yaml(VERIFY_POINT1_GRASP_MISSION_PATH)
    craic_arm_mission = load_yaml(CRAIC_ARM_MISSION_PATH)

    assert len(mission['waypoints']) == 1
    verify_waypoint = mission['waypoints'][0]
    task_point_1_pick = waypoint_named(craic_arm_mission, 'task_point_1_pick')

    assert verify_waypoint['x'] == task_point_1_pick['x']
    assert verify_waypoint['y'] == task_point_1_pick['y']
    assert verify_waypoint['yaw'] == task_point_1_pick['yaw']
    assert verify_waypoint['skip_navigation'] is True
    assert verify_waypoint['kind'] == task_point_1_pick['kind']
    assert verify_waypoint['frame_id'] == task_point_1_pick['frame_id']
    assert verify_waypoint['tasks'] == task_point_1_pick['tasks']

    grasp_task = task_named(mission, 'grasp_item_at_point_1')

    assert_dual_target_payload_grasp_task(grasp_task)


def test_verify_point4_classify_place_matches_craic_arm_mission():
    mission = load_yaml(VERIFY_POINT4_CLASSIFY_PLACE_MISSION_PATH)
    craic_arm_mission = load_yaml(CRAIC_ARM_MISSION_PATH)

    assert len(mission['waypoints']) == 1
    verify_waypoint = mission['waypoints'][0]
    task_point_4_classify_place = waypoint_named(
        craic_arm_mission,
        'task_point_4_classify_place',
    )

    assert verify_waypoint['x'] == task_point_4_classify_place['x']
    assert verify_waypoint['y'] == task_point_4_classify_place['y']
    assert verify_waypoint['yaw'] == task_point_4_classify_place['yaw']
    assert verify_waypoint['skip_navigation'] is True
    assert verify_waypoint['kind'] == task_point_4_classify_place['kind']
    assert verify_waypoint['frame_id'] == task_point_4_classify_place['frame_id']
    assert verify_waypoint['tasks'] == task_point_4_classify_place['tasks']

    classify_task = task_named(mission, 'classify_and_place_object')

    assert classify_task['type'] == 'classify_place'
    assert classify_task['backend'] == 'action'
    assert classify_task['task_type_name'] == 'CLASSIFY_PLATFORM_TO_COLOR_BOXES'


def test_verify_point3_flame_tracking_uses_flame_pipeline_topic():
    mission = load_yaml(VERIFY_POINT3_FLAME_MISSION_PATH)

    detect_flame_task = task_named(mission, 'detect_flame_at_point_3')

    assert detect_flame_task['detection_topic'] == FLAME_DETECTION_ARRAY_TOPIC
