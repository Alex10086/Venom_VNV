import time
from dataclasses import dataclass
from typing import Any

import rclpy
from std_srvs.srv import SetBool

from venom_mission_commander.arm_task_client import communication_callback_group, wait_for_callbacks
from venom_mission_commander.models import TaskContext, TaskExecutionResult, TaskSpec


@dataclass(frozen=True)
class PerceptionControlConfig:
    service_name: str
    enabled: bool
    timeout_sec: float
    service_wait_timeout_sec: float
    settle_sec: float
    output_key: str


def execute_perception_control_task(
    node: Any,
    context: TaskContext,
    spec: TaskSpec,
) -> TaskExecutionResult:
    try:
        config = parse_perception_control_config(spec.params)
    except Exception as exc:
        return TaskExecutionResult(False, f'invalid perception_control parameter: {exc}')

    response, error = call_set_enabled_service(node, config)
    data = {
        'service_name': config.service_name,
        'enabled': config.enabled,
        'success': error is None and bool(getattr(response, 'success', False)),
        'message': error or str(getattr(response, 'message', '')),
    }
    context.blackboard[config.output_key] = data

    if error is not None:
        return TaskExecutionResult(False, error, data)
    if not response.success:
        return TaskExecutionResult(False, response.message, data)

    sleep_non_negative(config.settle_sec)
    return TaskExecutionResult(True, response.message, data)


def parse_perception_control_config(params: dict[str, Any]) -> PerceptionControlConfig:
    mode = str(params.get('mode', '')).strip().lower()
    if 'enabled' in params:
        enabled = as_bool(params['enabled'])
    elif mode in {'start', 'enable', 'enabled', 'on'}:
        enabled = True
    elif mode in {'stop', 'disable', 'disabled', 'off'}:
        enabled = False
    else:
        raise ValueError('set enabled or mode=start/stop')

    timeout_sec = float(params.get('timeout_sec', 5.0))
    service_wait_timeout_sec = min(
        float(params.get('service_wait_timeout_sec', 2.0)),
        timeout_sec,
    )
    if timeout_sec <= 0.0:
        raise ValueError('timeout_sec must be positive')
    if service_wait_timeout_sec <= 0.0:
        raise ValueError('service_wait_timeout_sec must be positive')

    return PerceptionControlConfig(
        service_name=str(params['service_name']),
        enabled=enabled,
        timeout_sec=timeout_sec,
        service_wait_timeout_sec=service_wait_timeout_sec,
        settle_sec=float(params.get('settle_sec', 0.2)),
        output_key=str(params.get('output_key', 'last_perception_control')),
    )


def call_set_enabled_service(
    node: Any,
    config: PerceptionControlConfig,
) -> tuple[Any | None, str | None]:
    return call_set_enabled(
        node,
        service_name=config.service_name,
        enabled=config.enabled,
        timeout_sec=config.timeout_sec,
        service_wait_timeout_sec=config.service_wait_timeout_sec,
    )


def call_set_enabled(
    node: Any,
    service_name: str,
    enabled: bool,
    timeout_sec: float,
    service_wait_timeout_sec: float,
) -> tuple[Any | None, str | None]:
    client = node.create_client(
        SetBool,
        service_name,
        callback_group=communication_callback_group(node),
    )
    try:
        start_time = time.monotonic()
        if not client.wait_for_service(timeout_sec=service_wait_timeout_sec):
            return None, f'service unavailable: {service_name}'

        remaining_timeout_sec = timeout_sec - (time.monotonic() - start_time)
        if remaining_timeout_sec <= 0.0:
            return None, f'service wait timeout: {service_name}'

        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        deadline = time.monotonic() + max(remaining_timeout_sec, 0.0) + 0.2
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            wait_for_callbacks(node, 0.05)

        if not future.done():
            return None, f'perception control service timeout: {service_name}'
        return future.result(), None
    except Exception as exc:
        return None, f'perception control service error: {exc}'
    finally:
        node.destroy_client(client)


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'y', 'on', 'start', 'enable'}


def sleep_non_negative(seconds: float) -> None:
    time.sleep(max(float(seconds), 0.0))
