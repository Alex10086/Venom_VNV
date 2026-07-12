"""Tests for transactional ROS parameter mission tasks."""

from types import SimpleNamespace

import pytest

from rcl_interfaces.msg import ParameterType

from venom_mission_commander.models import TaskContext, TaskSpec, WaypointSpec
from venom_mission_commander.ros_parameter_task import (
    RosParameterTaskPlugin,
    parameter_value_from_python,
)
from venom_mission_commander.task_plugins import BaseTaskPlugin, TaskPluginRegistry


class FakeLogger:
    def error(self, message):
        pass


class FakeFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class FakeClient:
    def __init__(self, response):
        self.response = response
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture(self.response)


class FakeNode:
    def __init__(self, get_response, set_responses):
        self.get_client = FakeClient(get_response)
        self.set_clients = [FakeClient(response) for response in set_responses]
        self.created_set_clients = []

    def create_client(self, service_type, service_name, callback_group=None):
        if service_type.__name__ == 'GetParameters':
            return self.get_client
        client = self.set_clients.pop(0)
        self.created_set_clients.append(client)
        return client

    def destroy_client(self, client):
        pass

    def get_logger(self):
        return FakeLogger()


def make_context(node):
    return TaskContext(
        node=node,
        mission_id='test',
        waypoint=WaypointSpec('point4', 'map', 0.0, 0.0, 0.0),
        waypoint_index=0,
        task_index=0,
        mission_manager=None,
        blackboard={},
    )


def parameter_value(value):
    return parameter_value_from_python(value)


def set_response(successful=True, reason=''):
    return SimpleNamespace(result=SimpleNamespace(successful=successful, reason=reason))


def test_parameter_value_conversion_preserves_ros_types():
    assert parameter_value_from_python(True).type == ParameterType.PARAMETER_BOOL
    assert parameter_value_from_python(3).type == ParameterType.PARAMETER_INTEGER
    assert parameter_value_from_python(0.25).type == ParameterType.PARAMETER_DOUBLE
    assert parameter_value_from_python('teb').type == ParameterType.PARAMETER_STRING
    assert parameter_value_from_python([1.0, 2.0]).type == ParameterType.PARAMETER_DOUBLE_ARRAY


@pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf'), [1.0, float('nan')]])
def test_parameter_value_conversion_rejects_non_finite_floats(value):
    with pytest.raises(ValueError, match='finite'):
        parameter_value_from_python(value)


def test_set_snapshots_current_values_then_sets_atomically():
    node = FakeNode(
        SimpleNamespace(values=[parameter_value(0.25), parameter_value(0.60)]),
        [set_response()],
    )
    plugin = RosParameterTaskPlugin()
    plugin.configure(node)

    result = plugin.execute(
        make_context(node),
        TaskSpec(
            'tighten_teb',
            'ros_parameters',
            {
                'mode': 'set',
                'node_name': '/controller_server',
                'snapshot_key': 'point4_teb_defaults',
                'parameters': {
                    'FollowPath.min_obstacle_dist': 0.10,
                    'FollowPath.inflation_dist': 0.25,
                },
            },
        ),
    )

    assert result.success is True
    snapshot = plugin.snapshots['point4_teb_defaults']
    assert snapshot['parameters']['FollowPath.min_obstacle_dist'] == 0.25
    request = node.created_set_clients[0].requests[0]
    assert [parameter.name for parameter in request.parameters] == [
        'FollowPath.min_obstacle_dist',
        'FollowPath.inflation_dist',
    ]
    assert request.parameters[0].value.double_value == 0.10
    assert request.parameters[1].value.double_value == 0.25


def test_restore_failure_keeps_snapshot_for_cleanup():
    node = FakeNode(
        SimpleNamespace(values=[]),
        [set_response(successful=False, reason='rejected')],
    )
    plugin = RosParameterTaskPlugin()
    plugin.configure(node)
    plugin.snapshots['point4_teb_defaults'] = {
        'node_name': '/controller_server',
        'parameters': {'FollowPath.min_obstacle_dist': 0.25},
    }

    result = plugin.execute(
        make_context(node),
        TaskSpec(
            'restore_teb',
            'ros_parameters',
            {'mode': 'restore', 'restore_snapshot_key': 'point4_teb_defaults'},
        ),
    )

    assert result.success is False
    assert 'point4_teb_defaults' in plugin.snapshots


def test_restore_atomically_removes_snapshot_after_success():
    node = FakeNode(SimpleNamespace(values=[]), [set_response()])
    plugin = RosParameterTaskPlugin()
    plugin.configure(node)
    plugin.snapshots['point4_teb_defaults'] = {
        'node_name': '/controller_server',
        'parameters': {'FollowPath.min_obstacle_dist': 0.25},
    }

    result = plugin.execute(
        make_context(node),
        TaskSpec(
            'restore_teb',
            'ros_parameters',
            {'mode': 'restore', 'restore_snapshot_key': 'point4_teb_defaults'},
        ),
    )

    assert result.success is True
    assert plugin.snapshots == {}


def test_registry_cleanup_restores_active_parameter_snapshots():
    node = FakeNode(SimpleNamespace(values=[]), [set_response()])
    plugin = RosParameterTaskPlugin()
    plugin.configure(node)
    plugin.snapshots['point4_teb_defaults'] = {
        'node_name': '/controller_server',
        'parameters': {'FollowPath.inflation_dist': 0.60},
    }
    registry = TaskPluginRegistry()
    registry.register(plugin)

    assert registry.cleanup() is True
    assert plugin.snapshots == {}


def test_registry_cleanup_continues_after_plugin_failure():
    events = []

    class CleanupPlugin(BaseTaskPlugin):
        def __init__(self, task_type, cleanup_success):
            self.task_type = task_type
            self.cleanup_success = cleanup_success

        def execute(self, context, spec):
            raise NotImplementedError

        def cleanup(self):
            events.append(self.task_type)
            return self.cleanup_success

    registry = TaskPluginRegistry()
    registry.register(CleanupPlugin('first', False))
    registry.register(CleanupPlugin('second', True))

    assert registry.cleanup() is False
    assert events == ['first', 'second']


def test_registry_cleanup_continues_after_plugin_exception_and_logs_error():
    events = []

    class Logger:
        def __init__(self):
            self.errors = []

        def error(self, message):
            self.errors.append(message)

    class Node:
        def __init__(self):
            self.logger = Logger()

        def get_logger(self):
            return self.logger

    class CleanupPlugin(BaseTaskPlugin):
        def __init__(self, task_type, raises=False):
            self.task_type = task_type
            self.node = Node()
            self.raises = raises

        def execute(self, context, spec):
            raise NotImplementedError

        def cleanup(self):
            events.append(self.task_type)
            if self.raises:
                raise RuntimeError('cleanup failed')
            return True

    failing_plugin = CleanupPlugin('first', raises=True)
    registry = TaskPluginRegistry()
    registry.register(failing_plugin)
    registry.register(CleanupPlugin('second'))

    assert registry.cleanup() is False
    assert events == ['first', 'second']
    assert failing_plugin.node.logger.errors == ['Task plugin cleanup failed (first): cleanup failed']
