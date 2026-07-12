"""Transactional remote ROS parameter mission task."""

import math
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically

from venom_mission_commander.arm_task_client import communication_callback_group, wait_for_callbacks
from venom_mission_commander.models import TaskContext, TaskExecutionResult, TaskSpec


@dataclass(frozen=True)
class RosParameterTaskConfig:
    mode: str
    node_name: str | None
    snapshot_key: str | None
    restore_snapshot_key: str | None
    parameters: dict[str, Any]
    timeout_sec: float
    service_wait_timeout_sec: float


def parameter_value_from_python(value: Any) -> ParameterValue:
    parameter_value = ParameterValue()
    if isinstance(value, bool):
        parameter_value.type = ParameterType.PARAMETER_BOOL
        parameter_value.bool_value = value
    elif isinstance(value, int):
        parameter_value.type = ParameterType.PARAMETER_INTEGER
        parameter_value.integer_value = value
    elif isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError('floating-point parameter values must be finite')
        parameter_value.type = ParameterType.PARAMETER_DOUBLE
        parameter_value.double_value = value
    elif isinstance(value, str):
        parameter_value.type = ParameterType.PARAMETER_STRING
        parameter_value.string_value = value
    elif isinstance(value, list):
        _set_array_parameter_value(parameter_value, value)
    else:
        raise ValueError(f'unsupported parameter value type: {type(value).__name__}')
    return parameter_value


def _set_array_parameter_value(parameter_value: ParameterValue, values: list[Any]) -> None:
    if not values or all(isinstance(value, bool) for value in values):
        parameter_value.type = ParameterType.PARAMETER_BOOL_ARRAY
        parameter_value.bool_array_value = values
    elif all(isinstance(value, int) and not isinstance(value, bool) for value in values):
        parameter_value.type = ParameterType.PARAMETER_INTEGER_ARRAY
        parameter_value.integer_array_value = values
    elif all(isinstance(value, (int, float)) and not isinstance(value, bool) for value in values):
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('floating-point parameter values must be finite')
        parameter_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        parameter_value.double_array_value = [float(value) for value in values]
    elif all(isinstance(value, str) for value in values):
        parameter_value.type = ParameterType.PARAMETER_STRING_ARRAY
        parameter_value.string_array_value = values
    else:
        raise ValueError('parameter arrays must contain one supported type')


def parameter_value_to_python(value: ParameterValue) -> Any:
    value_type = value.type
    values = {
        ParameterType.PARAMETER_BOOL: value.bool_value,
        ParameterType.PARAMETER_INTEGER: value.integer_value,
        ParameterType.PARAMETER_DOUBLE: value.double_value,
        ParameterType.PARAMETER_STRING: value.string_value,
        ParameterType.PARAMETER_BYTE_ARRAY: list(value.byte_array_value),
        ParameterType.PARAMETER_BOOL_ARRAY: list(value.bool_array_value),
        ParameterType.PARAMETER_INTEGER_ARRAY: list(value.integer_array_value),
        ParameterType.PARAMETER_DOUBLE_ARRAY: list(value.double_array_value),
        ParameterType.PARAMETER_STRING_ARRAY: list(value.string_array_value),
    }
    if value_type not in values:
        raise ValueError(f'cannot snapshot unset parameter type: {value_type}')
    return values[value_type]


def parse_ros_parameter_task_config(params: dict[str, Any]) -> RosParameterTaskConfig:
    mode = str(params.get('mode', '')).strip().lower()
    if mode not in {'set', 'restore'}:
        raise ValueError('mode must be set or restore')
    timeout_sec = float(params.get('timeout_sec', 5.0))
    service_wait_timeout_sec = min(float(params.get('service_wait_timeout_sec', 2.0)), timeout_sec)
    if timeout_sec <= 0.0 or service_wait_timeout_sec <= 0.0:
        raise ValueError('timeout_sec and service_wait_timeout_sec must be positive')
    if mode == 'set':
        node_name = str(params.get('node_name', '')).strip()
        snapshot_key = str(params.get('snapshot_key', '')).strip()
        parameters = params.get('parameters')
        if not node_name or not snapshot_key or not isinstance(parameters, dict) or not parameters:
            raise ValueError('set requires node_name, snapshot_key, and non-empty parameters')
        return RosParameterTaskConfig(
            mode, node_name, snapshot_key, None, parameters, timeout_sec, service_wait_timeout_sec
        )
    restore_snapshot_key = str(params.get('restore_snapshot_key', '')).strip()
    if not restore_snapshot_key:
        raise ValueError('restore requires restore_snapshot_key')
    return RosParameterTaskConfig(
        mode, None, None, restore_snapshot_key, {}, timeout_sec, service_wait_timeout_sec
    )


class RosParameterTaskPlugin:
    task_type = 'ros_parameters'

    def __init__(self):
        self.snapshots: dict[str, dict[str, Any]] = {}
        self.node: Any = None

    def configure(self, node: Any) -> None:
        self.node = node

    def cancel(self) -> None:
        return None

    def execute(self, context: TaskContext, spec: TaskSpec) -> TaskExecutionResult:
        try:
            config = parse_ros_parameter_task_config(spec.params)
        except Exception as exc:
            return TaskExecutionResult(False, f'invalid ros_parameters parameter: {exc}')
        if config.mode == 'set':
            return self._set(config)
        return self._restore(config.restore_snapshot_key, config.timeout_sec, config.service_wait_timeout_sec)

    def cleanup(self) -> bool:
        success = True
        for snapshot_key in list(self.snapshots):
            result = self._restore(snapshot_key, 5.0, 2.0)
            success = result.success and success
        return success

    def _set(self, config: RosParameterTaskConfig) -> TaskExecutionResult:
        assert config.node_name is not None and config.snapshot_key is not None
        if config.snapshot_key not in self.snapshots:
            values, error = self._get_parameters(
                config.node_name,
                list(config.parameters),
                config.timeout_sec,
                config.service_wait_timeout_sec,
            )
            if error is not None:
                return TaskExecutionResult(False, error)
            try:
                self.snapshots[config.snapshot_key] = {
                    'node_name': config.node_name,
                    'parameters': dict(zip(config.parameters, map(parameter_value_to_python, values))),
                }
            except Exception as exc:
                return TaskExecutionResult(False, f'failed to snapshot parameters: {exc}')
        return self._set_parameters(
            config.node_name,
            config.parameters,
            config.timeout_sec,
            config.service_wait_timeout_sec,
        )

    def _restore(
        self, snapshot_key: str | None, timeout_sec: float, service_wait_timeout_sec: float
    ) -> TaskExecutionResult:
        if snapshot_key not in self.snapshots:
            return TaskExecutionResult(False, f'unknown parameter snapshot: {snapshot_key}')
        snapshot = self.snapshots[snapshot_key]
        result = self._set_parameters(
            snapshot['node_name'], snapshot['parameters'], timeout_sec, service_wait_timeout_sec
        )
        if result.success:
            del self.snapshots[snapshot_key]
        return result

    def _get_parameters(self, node_name, names, timeout_sec, service_wait_timeout_sec):
        request = GetParameters.Request()
        request.names = names
        response, error = self._call_service(
            GetParameters, f'{node_name}/get_parameters', request, timeout_sec, service_wait_timeout_sec
        )
        if error is not None:
            return [], error
        if len(response.values) != len(names):
            return [], 'get parameters returned an unexpected number of values'
        return response.values, None

    def _set_parameters(self, node_name, parameters, timeout_sec, service_wait_timeout_sec):
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter(name=name, value=parameter_value_from_python(value))
            for name, value in parameters.items()
        ]
        response, error = self._call_service(
            SetParametersAtomically,
            f'{node_name}/set_parameters_atomically',
            request,
            timeout_sec,
            service_wait_timeout_sec,
        )
        if error is not None:
            return TaskExecutionResult(False, error)
        if not response.result.successful:
            return TaskExecutionResult(False, f'parameter update rejected: {response.result.reason}')
        return TaskExecutionResult(True, 'parameters updated atomically')

    def _call_service(self, service_type, service_name, request, timeout_sec, service_wait_timeout_sec):
        client = self.node.create_client(
            service_type, service_name, callback_group=communication_callback_group(self.node)
        )
        try:
            start_time = time.monotonic()
            if not client.wait_for_service(timeout_sec=service_wait_timeout_sec):
                return None, f'service unavailable: {service_name}'
            remaining = timeout_sec - (time.monotonic() - start_time)
            if remaining <= 0.0:
                return None, f'service wait timeout: {service_name}'
            future = client.call_async(request)
            deadline = time.monotonic() + remaining + 0.2
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                wait_for_callbacks(self.node, 0.05)
            if not future.done():
                return None, f'service call timeout: {service_name}'
            return future.result(), None
        except Exception as exc:
            return None, f'service call error ({service_name}): {exc}'
        finally:
            self.node.destroy_client(client)
