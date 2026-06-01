import time
from dataclasses import dataclass
from typing import Any

import rclpy

from venom_mission_commander.arm_task_client import communication_callback_group, wait_for_callbacks
from venom_mission_commander.models import TaskContext, TaskExecutionResult, TaskSpec


@dataclass(frozen=True)
class ReadMeterMockConfig:
    meter_id: str
    delay_sec: float
    value: Any
    confidence: float


@dataclass(frozen=True)
class ReadMeterServiceConfig:
    meter_id: str
    service_name: str
    timeout_sec: float
    service_wait_timeout_sec: float
    expected_digits: int
    min_confidence: float
    output_key: str


def execute_read_meter_task(node: Any, context: TaskContext, spec: TaskSpec) -> TaskExecutionResult:
    backend = read_meter_backend(spec.params)
    if backend in ('mock', ''):
        return execute_mock_read_meter(node, context, parse_mock_config(spec.params))
    if backend == 'service':
        return execute_service_read_meter(node, context, spec.params)
    return TaskExecutionResult(False, f'unknown read_meter backend: {backend}')


def read_meter_backend(params: dict[str, Any]) -> str:
    return str(params.get('backend', 'mock')).strip().lower()


def parse_mock_config(params: dict[str, Any]) -> ReadMeterMockConfig:
    return ReadMeterMockConfig(
        meter_id=str(params.get('meter_id', 'meter_1')),
        delay_sec=float(params.get('mock_delay_sec', 0.5)),
        value=params.get('mock_value', '220.0V'),
        confidence=float(params.get('mock_confidence', 0.9)),
    )


def execute_mock_read_meter(
    node: Any,
    context: TaskContext,
    config: ReadMeterMockConfig,
) -> TaskExecutionResult:
    node.get_logger().info(f'[MOCK TASK] Reading meter image: {config.meter_id}')
    sleep_non_negative(config.delay_sec)

    reading = build_mock_reading(config)
    context.blackboard['meter_reading'] = reading
    return TaskExecutionResult(True, f'meter {config.meter_id}={config.value}', reading)


def build_mock_reading(config: ReadMeterMockConfig) -> dict[str, Any]:
    return {
        'meter_id': config.meter_id,
        'value': config.value,
        'confidence': config.confidence,
    }


def execute_service_read_meter(
    node: Any,
    context: TaskContext,
    params: dict[str, Any],
) -> TaskExecutionResult:
    try:
        from printed_number_interfaces.srv import ReadPrintedNumber
    except Exception as exc:
        return TaskExecutionResult(False, f'ReadPrintedNumber import failed: {exc}')

    config = parse_service_config(params)
    if config.timeout_sec <= 0.0:
        return TaskExecutionResult(False, 'timeout_sec must be positive for service backend')

    response, error = call_read_printed_number_service(node, ReadPrintedNumber, config)
    if error is not None:
        return TaskExecutionResult(False, error)

    try:
        error = validate_read_printed_number_response(response, config)
        if error is not None:
            return TaskExecutionResult(False, error)

        reading = build_service_reading(config, response)
        context.blackboard[config.output_key] = reading
        return TaskExecutionResult(True, f'meter {config.meter_id}={reading["value"]}', reading)
    except Exception as exc:
        return TaskExecutionResult(False, f'read meter service error: {exc}')


def parse_service_config(params: dict[str, Any]) -> ReadMeterServiceConfig:
    timeout_sec = float(params.get('timeout_sec', 5.0))
    service_wait_timeout_sec = min(
        float(params.get('service_wait_timeout_sec', 1.0)),
        timeout_sec,
    )
    return ReadMeterServiceConfig(
        meter_id=str(params.get('meter_id', 'meter_1')),
        service_name=str(params.get('service_name', '/perception/read_printed_number')),
        timeout_sec=timeout_sec,
        service_wait_timeout_sec=service_wait_timeout_sec,
        expected_digits=int(params.get('expected_digits', 0)),
        min_confidence=float(params.get('min_confidence', 0.0)),
        output_key=str(params.get('output_key', 'meter_reading')),
    )


def call_read_printed_number_service(
    node: Any,
    service_type: Any,
    config: ReadMeterServiceConfig,
) -> tuple[Any | None, str | None]:
    client = node.create_client(
        service_type,
        config.service_name,
        callback_group=communication_callback_group(node),
    )
    try:
        start_time = time.monotonic()
        if not client.wait_for_service(timeout_sec=max(config.service_wait_timeout_sec, 0.0)):
            return None, f'service unavailable: {config.service_name}'

        remaining_timeout_sec = config.timeout_sec - (time.monotonic() - start_time)
        if remaining_timeout_sec <= 0.0:
            return None, f'service wait timeout: {config.service_name}'

        request = build_read_printed_number_request(
            service_type,
            config,
            remaining_timeout_sec,
        )
        future = client.call_async(request)
        deadline = time.monotonic() + max(remaining_timeout_sec, 0.0) + 0.2
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            wait_for_callbacks(node, 0.05)

        if not future.done():
            return None, f'read meter service timeout: {config.service_name}'
        return future.result(), None
    except Exception as exc:
        return None, f'read meter service error: {exc}'
    finally:
        node.destroy_client(client)


def build_read_printed_number_request(
    service_type: Any,
    config: ReadMeterServiceConfig,
    timeout_sec: float,
) -> Any:
    request = service_type.Request()
    request.target_id = config.meter_id
    request.timeout_sec = float(timeout_sec)
    request.expected_digits = max(config.expected_digits, 0)
    request.min_confidence = max(config.min_confidence, 0.0)
    return request


def validate_read_printed_number_response(
    response: Any,
    config: ReadMeterServiceConfig,
) -> str | None:
    if response is None:
        return 'read meter service returned no response'
    if not response.success:
        return f'read meter failed: {response.message}'

    value = str(response.value)
    confidence = float(response.confidence)
    if not value.isdigit():
        return f'read value is not pure digits: {value}'
    if config.expected_digits > 0 and len(value) != config.expected_digits:
        return f'expected {config.expected_digits} digits, got {len(value)}'
    if confidence < config.min_confidence:
        return f'confidence {confidence:.3f} below {config.min_confidence:.3f}'
    return None


def build_service_reading(config: ReadMeterServiceConfig, response: Any) -> dict[str, Any]:
    reading = {
        'meter_id': config.meter_id,
        'value': str(response.value),
        'confidence': float(response.confidence),
        'source': 'printed_number_service',
    }
    if hasattr(response, 'image_path') and response.image_path:
        reading['image_path'] = str(response.image_path)
    return reading


def sleep_non_negative(seconds: float) -> None:
    time.sleep(max(float(seconds), 0.0))
