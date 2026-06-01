import math
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from rclpy.action.client import ActionClient

from venom_mission_commander.models import TaskContext, TaskExecutionResult, TaskSpec


EXECUTE_TASK_GOAL_CONSTANTS = {
    'PICK_AND_PLACE_FIXED': 1,
    'MOVE_HOME': 2,
    'MOVE_OBSERVE': 3,
    'PICK_AND_PLACE_LATEST_TARGET': 4,
    'CLASSIFY_PLATFORM_TO_COLOR_BOXES': 5,
    'REPEAT_VISUAL_PICK_TO_PAYLOAD': 6,
    'START_FLAME_TRACKING': 7,
    'STOP_FLAME_TRACKING': 8,
}

FLAME_TRACKING_INACTIVE_STATES = {'inactive', ''}
FLAME_TRACKING_UNSAFE_STATES = {'starting', 'active', 'stopping', 'unknown'}


@dataclass(frozen=True)
class ManipulationActionConfig:
    action_name: str
    task_type_name: str
    task_type_value: int | None
    timeout_sec: float
    server_wait_timeout_sec: float
    retry_count: int
    retry_backoff_sec: float
    output_key: str


@dataclass(frozen=True)
class FlameDetectionConfig:
    detection_topic: str
    target_class: str
    min_confidence: float
    min_consecutive_detections: int
    timeout_sec: float


@dataclass(frozen=True)
class FlameTrackingConfig:
    mode: str
    service_name: str
    service_wait_timeout_sec: float
    call_timeout_sec: float
    tracking_duration_sec: float
    disable_on_exit: bool
    output_key: str
    status_topic: str
    wait_until_tracking: bool
    require_target_acquired: bool
    ready_timeout_sec: float
    target_class: str


@dataclass(frozen=True)
class GoalCancelStatus:
    requested: bool
    confirmed: bool
    message: str


def status_field_value(status: Any, field_name: str, default: Any = None) -> Any:
    if isinstance(status, dict):
        return status.get(field_name, default)
    return getattr(status, field_name, default)


def flame_tracker_status_dict(status: Any) -> dict[str, Any]:
    target_age_sec = float(status_field_value(status, 'target_age_sec', -1.0))
    if not math.isfinite(target_age_sec):
        target_age_sec = -1.0
    return {
        'enabled': bool(status_field_value(status, 'enabled', False)),
        'mode': str(status_field_value(status, 'mode', '')),
        'target_acquired': bool(status_field_value(status, 'target_acquired', False)),
        'target_age_sec': target_age_sec,
        'yaw_error_rad': float(status_field_value(status, 'yaw_error_rad', 0.0)),
        'pitch_error_rad': float(status_field_value(status, 'pitch_error_rad', 0.0)),
        'joint_state_ok': bool(status_field_value(status, 'joint_state_ok', False)),
        'camera_info_ok': bool(status_field_value(status, 'camera_info_ok', False)),
        'command_output_ok': bool(status_field_value(status, 'command_output_ok', False)),
        'command_saturated': bool(status_field_value(status, 'command_saturated', False)),
        'target_class_name': str(status_field_value(status, 'target_class_name', '')),
        'message': str(status_field_value(status, 'message', '')),
    }


def tracker_status_is_ready(
    status: Any,
    target_class: str,
    require_target_acquired: bool,
) -> bool:
    enabled = as_bool(status_field_value(status, 'enabled', False))
    if not enabled:
        return False

    mode = str(status_field_value(status, 'mode', '')).strip().lower()
    if mode != 'tracking':
        return False

    status_target_class = str(status_field_value(status, 'target_class_name', '')).strip()
    if target_class and status_target_class and status_target_class != target_class:
        return False

    if not as_bool(status_field_value(status, 'camera_info_ok', True)):
        return False

    if not as_bool(status_field_value(status, 'joint_state_ok', True)):
        return False

    if not as_bool(status_field_value(status, 'command_output_ok', True)):
        return False

    if require_target_acquired and not as_bool(status_field_value(status, 'target_acquired', False)):
        return False

    return True


def wait_for_flame_tracker_ready(
    node: Any,
    config: FlameTrackingConfig,
) -> tuple[dict[str, Any] | None, str | None]:
    try:
        from venom_manipulation_interfaces.msg import FlameTrackerStatus  # type: ignore[import-not-found]
    except Exception as exc:
        return None, f'FlameTrackerStatus import failed: {exc}'

    latest_status: dict[str, Any] | None = None
    ready_status: dict[str, Any] | None = None

    def callback(message: Any) -> None:
        nonlocal latest_status, ready_status
        latest_status = flame_tracker_status_dict(message)
        if tracker_status_is_ready(
            latest_status,
            target_class=config.target_class,
            require_target_acquired=config.require_target_acquired,
        ):
            ready_status = latest_status

    subscription = node.create_subscription(
        FlameTrackerStatus,
        config.status_topic,
        callback,
        10,
        callback_group=communication_callback_group(node),
    )
    try:
        deadline = time.monotonic() + config.ready_timeout_sec
        while ros_ok() and time.monotonic() < deadline:
            if ready_status is not None:
                return ready_status, None
            wait_for_callbacks(node, 0.05)

        if ready_status is not None:
            return ready_status, None
        if latest_status is None:
            return None, f'flame tracker ready timeout on {config.status_topic}'
        return latest_status, f'flame tracker ready timeout on {config.status_topic}'
    finally:
        if hasattr(node, 'destroy_subscription'):
            node.destroy_subscription(subscription)


def execute_manipulation_action_task(
    node: Any,
    context: TaskContext,
    spec: TaskSpec,
    default_task_type_name: str,
    output_key: str,
) -> TaskExecutionResult:
    try:
        config = parse_manipulation_action_config(
            spec.params,
            default_task_type_name,
            output_key,
        )
    except Exception as exc:
        return TaskExecutionResult(False, f'invalid manipulation action parameter: {exc}')

    stop_ok, stop_error = stop_active_flame_tracking_if_needed(
        node,
        context.blackboard,
        'before manipulation action',
        force=True,
    )
    if not stop_ok:
        message = f'active flame tracking stop failed before manipulation action: {stop_error}'
        data = {
            'backend': 'action',
            'action_name': config.action_name,
            'task_type': config.task_type_name,
            'task_type_value': config.task_type_value,
            'success': False,
            'message': message,
        }
        context.blackboard[config.output_key] = data
        return TaskExecutionResult(False, message, data)

    attempts = max(config.retry_count, 0) + 1
    last_error = ''
    last_data: dict[str, Any] = {}
    for attempt_index in range(attempts):
        result, error = call_execute_task_action(node, config)
        data = build_manipulation_action_data(config, result, attempt_index + 1)
        last_data = data
        if error is None and data['success']:
            context.blackboard[config.output_key] = data
            return TaskExecutionResult(True, 'manipulation task succeeded', data)

        last_error = error or str(data.get('message') or 'manipulation task failed')
        if not should_retry_manipulation_error(last_error):
            break
        if attempt_index + 1 < attempts:
            sleep_non_negative(config.retry_backoff_sec)

    if last_data:
        context.blackboard[config.output_key] = last_data
    return TaskExecutionResult(False, last_error, last_data)


def parse_manipulation_action_config(
    params: dict[str, Any],
    default_task_type_name: str,
    default_output_key: str,
) -> ManipulationActionConfig:
    timeout_sec = float(params.get('timeout_sec', 60.0))
    server_wait_timeout_sec = float(params.get('server_wait_timeout_sec', 2.0))
    if timeout_sec <= 0.0:
        raise ValueError('timeout_sec must be positive')
    if server_wait_timeout_sec <= 0.0:
        raise ValueError('server_wait_timeout_sec must be positive')

    task_type_name = str(params.get('task_type_name', default_task_type_name)).strip()
    task_type_value = params.get('task_type_value')
    return ManipulationActionConfig(
        action_name=str(params.get('action_name', '/manipulation/execute_task')),
        task_type_name=task_type_name,
        task_type_value=int(task_type_value) if task_type_value is not None else None,
        timeout_sec=timeout_sec,
        server_wait_timeout_sec=server_wait_timeout_sec,
        retry_count=int(params.get('retry_count', 0)),
        retry_backoff_sec=float(params.get('retry_backoff_sec', 0.5)),
        output_key=str(params.get('output_key', default_output_key)),
    )


def call_execute_task_action(
    node: Any,
    config: ManipulationActionConfig,
) -> tuple[Any | None, str | None]:
    try:
        from venom_manipulation_interfaces.action import ExecuteTask  # type: ignore[import-not-found]
    except Exception as exc:
        return None, f'ExecuteTask action import failed: {exc}'

    client = ActionClient(
        node,
        ExecuteTask,
        config.action_name,
        callback_group=communication_callback_group(node),
    )
    goal_handle = None
    result_future = None
    try:
        if not client.wait_for_server(timeout_sec=config.server_wait_timeout_sec):
            return None, f'action server unavailable: {config.action_name}'

        task_type_value = config.task_type_value
        if task_type_value is None:
            task_type_value = resolve_execute_task_value(config.task_type_name, ExecuteTask)
        goal = ExecuteTask.Goal()
        goal.task_type = int(task_type_value)

        deadline = time.monotonic() + config.timeout_sec
        send_future = client.send_goal_async(goal)
        if not spin_until_future_done(node, send_future, deadline):
            return None, f'action goal send timeout: {config.action_name}'

        goal_handle = send_future.result()
        if goal_handle is None or not getattr(goal_handle, 'accepted', False):
            return None, f'action goal rejected: {config.task_type_name}'

        result_future = goal_handle.get_result_async()
        if not spin_until_future_done(node, result_future, deadline):
            cancel_status = cancel_goal_and_wait(goal_handle, node, result_future)
            cancel_message = 'cancel confirmed' if cancel_status.confirmed else 'cancel not confirmed'
            return None, f'action result timeout: {config.action_name}; {cancel_message}'

        action_result = result_future.result()
        return getattr(action_result, 'result', action_result), None
    except Exception as exc:
        if goal_handle is not None:
            cancel_status = cancel_goal_and_wait(goal_handle, node, result_future)
            cancel_message = 'cancel confirmed' if cancel_status.confirmed else 'cancel not confirmed'
            return None, f'manipulation action error: {exc}; {cancel_message}'
        return None, f'manipulation action error: {exc}'
    finally:
        destroy_ros_entity(client)


def resolve_execute_task_value(value: Any, execute_task_type: Any) -> int:
    if isinstance(value, int):
        return int(value)
    if isinstance(value, str) and value.strip().isdigit():
        return int(value.strip())

    constant_name = str(value).strip()
    goal_type = getattr(execute_task_type, 'Goal', execute_task_type)
    if hasattr(goal_type, constant_name):
        return int(getattr(goal_type, constant_name))
    if constant_name in EXECUTE_TASK_GOAL_CONSTANTS:
        return EXECUTE_TASK_GOAL_CONSTANTS[constant_name]
    raise ValueError(f'unknown ExecuteTask goal constant: {constant_name}')


def build_manipulation_action_data(
    config: ManipulationActionConfig,
    result: Any,
    attempt: int,
) -> dict[str, Any]:
    success = bool(getattr(result, 'success', False)) if result is not None else False
    return {
        'backend': 'action',
        'action_name': config.action_name,
        'task_type': config.task_type_name,
        'task_type_value': config.task_type_value,
        'success': success,
        'stage_reached': int(getattr(result, 'stage_reached', 0)) if result is not None else None,
        'error_code': int(getattr(result, 'error_code', -1)) if result is not None else None,
        'message': str(getattr(result, 'message', '')) if result is not None else '',
        'attempt': attempt,
    }


def wait_for_flame_detection_task(node: Any, params: dict[str, Any]) -> TaskExecutionResult:
    try:
        from venom_manipulation_interfaces.msg import Detection2DArray  # type: ignore[import-not-found]
    except Exception as exc:
        return TaskExecutionResult(False, f'Detection2DArray import failed: {exc}')

    try:
        config = parse_flame_detection_config(params)
    except Exception as exc:
        return TaskExecutionResult(False, f'invalid flame detection parameter: {exc}')

    latest_detection: dict[str, Any] | None = None
    consecutive_matches = 0

    def callback(message: Any) -> None:
        nonlocal latest_detection, consecutive_matches
        detection = best_detection_dict(
            getattr(message, 'detections', []),
            target_class=config.target_class,
            min_confidence=config.min_confidence,
        )
        if detection is None:
            consecutive_matches = 0
            return
        latest_detection = detection
        consecutive_matches += 1

    subscription = node.create_subscription(
        Detection2DArray,
        config.detection_topic,
        callback,
        10,
        callback_group=communication_callback_group(node),
    )
    try:
        deadline = time.monotonic() + config.timeout_sec
        while ros_ok() and time.monotonic() < deadline:
            if latest_detection is not None and consecutive_matches >= config.min_consecutive_detections:
                return TaskExecutionResult(True, 'flame detected', latest_detection)
            wait_for_callbacks(node, 0.05)
        return TaskExecutionResult(
            False,
            f'flame detection timeout on {config.detection_topic}',
        )
    finally:
        if hasattr(node, 'destroy_subscription'):
            node.destroy_subscription(subscription)


def parse_flame_detection_config(params: dict[str, Any]) -> FlameDetectionConfig:
    timeout_sec = float(params.get('timeout_sec', 5.0))
    if timeout_sec <= 0.0:
        raise ValueError('timeout_sec must be positive')
    return FlameDetectionConfig(
        detection_topic=str(params.get('detection_topic', '/perception/flame/detections_2d_array')),
        target_class=str(params.get('target_class', 'fire')),
        min_confidence=float(params.get('min_confidence', 0.45)),
        min_consecutive_detections=max(1, int(params.get('min_consecutive_detections', 1))),
        timeout_sec=timeout_sec,
    )


def best_detection_dict(
    detections: list[Any],
    target_class: str,
    min_confidence: float,
) -> dict[str, Any] | None:
    best = None
    best_confidence = float('-inf')
    for detection in detections:
        class_name = str(getattr(detection, 'class_name', ''))
        confidence = float(getattr(detection, 'confidence', 0.0))
        if class_name != target_class or confidence < min_confidence:
            continue
        if confidence <= best_confidence:
            continue
        best_confidence = confidence
        best = {
            'class': class_name,
            'confidence': confidence,
            'bbox': [
                float(getattr(detection, 'center_x', 0.0)),
                float(getattr(detection, 'center_y', 0.0)),
                float(getattr(detection, 'size_x', 0.0)),
                float(getattr(detection, 'size_y', 0.0)),
            ],
        }
    return best


def execute_flame_tracking_task(
    node: Any,
    context: TaskContext,
    spec: TaskSpec,
) -> TaskExecutionResult:
    try:
        config = parse_flame_tracking_config(spec.params)
    except Exception as exc:
        return TaskExecutionResult(False, f'invalid flame tracking parameter: {exc}')

    source_key = str(spec.params.get('source', 'flame_detection'))
    detection = context.blackboard.get(source_key)
    tracking = {
        'backend': 'service',
        'mode': config.mode,
        'source': source_key,
        'detection': detection,
        'service_name': config.service_name,
        'tracking_duration_sec': config.tracking_duration_sec,
        'disable_on_exit': config.disable_on_exit,
        'status_topic': config.status_topic,
        'wait_until_tracking': config.wait_until_tracking,
        'require_target_acquired': config.require_target_acquired,
        'ready_timeout_sec': config.ready_timeout_sec,
        'target_class': config.target_class,
    }

    if config.mode == 'stop':
        return stop_flame_tracking_task(node, context, config, tracking)

    enabled = False
    wait_error = None
    disable_error = None
    context.blackboard['flame_tracking_state'] = 'starting'
    context.blackboard['flame_tracking_service_name'] = config.service_name
    context.blackboard['flame_tracking_service_wait_timeout_sec'] = config.service_wait_timeout_sec
    context.blackboard['flame_tracking_call_timeout_sec'] = config.call_timeout_sec
    enable_response, enable_error = call_set_bool_service(
        node,
        config.service_name,
        True,
        config.service_wait_timeout_sec,
        config.call_timeout_sec,
    )
    if enable_error is not None:
        tracking['success'] = False
        tracking['message'] = enable_error
        context.blackboard['flame_tracking_state'] = 'unknown'
        context.blackboard['flame_tracking_ready'] = False
        stop_ok, stop_error = stop_active_flame_tracking_if_needed(
            node,
            context.blackboard,
            'after flame tracking start failure',
            force=True,
        )
        tracking['stop_after_start_failure_success'] = stop_ok
        if stop_error is not None:
            tracking['stop_after_start_failure_error'] = stop_error
        context.blackboard[config.output_key] = tracking
        return TaskExecutionResult(False, enable_error, tracking)

    enabled = True
    context.blackboard['flame_tracking_active'] = True
    context.blackboard['flame_tracking_state'] = 'active'
    context.blackboard['flame_tracking_ready'] = False
    tracking['enable_message'] = str(getattr(enable_response, 'message', ''))

    if config.mode == 'start':
        if config.wait_until_tracking:
            ready_status, ready_error = wait_for_flame_tracker_ready(node, config)
            if ready_status is not None:
                tracking['ready_status'] = ready_status
            if ready_error is not None:
                tracking['success'] = False
                tracking['message'] = ready_error
                tracking['ready_wait_error'] = ready_error
                context.blackboard['flame_tracking_ready'] = False
                context.blackboard['flame_tracking_state'] = 'unknown'
                stop_ok, stop_error = stop_active_flame_tracking_if_needed(
                    node,
                    context.blackboard,
                    'after flame tracking ready wait failure',
                    force=True,
                )
                tracking['stop_after_ready_failure_success'] = stop_ok
                if stop_error is not None:
                    tracking['stop_after_ready_failure_error'] = stop_error
                context.blackboard[config.output_key] = tracking
                return TaskExecutionResult(False, ready_error, tracking)
            context.blackboard['flame_tracking_ready'] = True
        tracking['success'] = True
        tracking['message'] = 'flame tracking started'
        context.blackboard[config.output_key] = tracking
        return TaskExecutionResult(True, 'flame tracking started', tracking)

    try:
        wait_with_spin(node, config.tracking_duration_sec)
    except Exception as exc:
        wait_error = f'flame tracking interrupted: {exc}'
    finally:
        if enabled and config.disable_on_exit:
            disable_response, disable_error = call_set_bool_service(
                node,
                config.service_name,
                False,
                config.service_wait_timeout_sec,
                config.call_timeout_sec,
            )
            tracking['disable_message'] = str(getattr(disable_response, 'message', ''))
            if disable_error is None:
                context.blackboard['flame_tracking_active'] = False
                context.blackboard['flame_tracking_state'] = 'inactive'
            else:
                context.blackboard['flame_tracking_state'] = 'unknown'

    if wait_error is not None:
        tracking['success'] = False
        tracking['message'] = wait_error
        if disable_error is not None:
            tracking['disable_error'] = disable_error
        context.blackboard[config.output_key] = tracking
        return TaskExecutionResult(False, wait_error, tracking)

    if disable_error is not None:
        tracking['success'] = False
        tracking['message'] = f'flame tracking disable failed: {disable_error}'
        context.blackboard[config.output_key] = tracking
        return TaskExecutionResult(False, tracking['message'], tracking)

    tracking['success'] = True
    tracking['message'] = 'flame tracking completed'
    context.blackboard[config.output_key] = tracking
    return TaskExecutionResult(True, 'flame tracking completed', tracking)


def parse_flame_tracking_config(params: dict[str, Any]) -> FlameTrackingConfig:
    mode = str(params.get('mode', 'hold')).strip().lower()
    if mode not in {'start', 'stop', 'hold'}:
        raise ValueError('mode must be one of: start, stop, hold')
    service_wait_timeout_sec = float(params.get('service_wait_timeout_sec', 1.0))
    call_timeout_sec = float(params.get('timeout_sec', params.get('call_timeout_sec', 3.0)))
    tracking_duration_sec = float(params.get('tracking_duration_sec', 3.0))
    ready_timeout_sec = float(params.get('ready_timeout_sec', 5.0))
    if service_wait_timeout_sec <= 0.0:
        raise ValueError('service_wait_timeout_sec must be positive')
    if call_timeout_sec <= 0.0:
        raise ValueError('timeout_sec must be positive')
    if tracking_duration_sec < 0.0:
        raise ValueError('tracking_duration_sec must be non-negative')
    if ready_timeout_sec <= 0.0:
        raise ValueError('ready_timeout_sec must be positive')
    return FlameTrackingConfig(
        mode=mode,
        service_name=str(params.get('service_name', '/flame_arm_tracker/set_enabled')),
        service_wait_timeout_sec=service_wait_timeout_sec,
        call_timeout_sec=call_timeout_sec,
        tracking_duration_sec=tracking_duration_sec,
        disable_on_exit=as_bool(params.get('disable_on_exit', True)),
        output_key=str(params.get('output_key', 'last_flame_tracking')),
        status_topic=str(params.get('status_topic', '/flame_arm_tracker/status')),
        wait_until_tracking=as_bool(params.get('wait_until_tracking', False)),
        require_target_acquired=as_bool(params.get('require_target_acquired', False)),
        ready_timeout_sec=ready_timeout_sec,
        target_class=str(params.get('target_class', 'fire')),
    )


def stop_flame_tracking_task(
    node: Any,
    context: TaskContext,
    config: FlameTrackingConfig,
    tracking: dict[str, Any],
) -> TaskExecutionResult:
    context.blackboard['flame_tracking_state'] = 'stopping'
    disable_response, disable_error = call_set_bool_service(
        node,
        config.service_name,
        False,
        config.service_wait_timeout_sec,
        config.call_timeout_sec,
    )
    tracking['disable_message'] = str(getattr(disable_response, 'message', ''))
    if disable_error is not None:
        context.blackboard['flame_tracking_active'] = True
        context.blackboard['flame_tracking_state'] = 'unknown'
        context.blackboard['flame_tracking_ready'] = False
        tracking['success'] = False
        tracking['message'] = f'flame tracking stop failed: {disable_error}'
        context.blackboard[config.output_key] = tracking
        return TaskExecutionResult(False, tracking['message'], tracking)

    context.blackboard['flame_tracking_active'] = False
    context.blackboard['flame_tracking_state'] = 'inactive'
    context.blackboard['flame_tracking_ready'] = False
    context.blackboard['flame_tracking_service_name'] = config.service_name
    context.blackboard['flame_tracking_service_wait_timeout_sec'] = config.service_wait_timeout_sec
    context.blackboard['flame_tracking_call_timeout_sec'] = config.call_timeout_sec
    tracking['success'] = True
    tracking['message'] = 'flame tracking stopped'
    context.blackboard[config.output_key] = tracking
    return TaskExecutionResult(True, 'flame tracking stopped', tracking)


def call_set_bool_service(
    node: Any,
    service_name: str,
    enabled: bool,
    service_wait_timeout_sec: float,
    call_timeout_sec: float,
) -> tuple[Any | None, str | None]:
    try:
        from std_srvs.srv import SetBool
    except Exception as exc:
        return None, f'SetBool import failed: {exc}'

    client = node.create_client(
        SetBool,
        service_name,
        callback_group=communication_callback_group(node),
    )
    try:
        if not client.wait_for_service(timeout_sec=service_wait_timeout_sec):
            return None, f'service unavailable: {service_name}'
        request = SetBool.Request()
        request.data = bool(enabled)
        future = client.call_async(request)
        if not spin_until_future_done(node, future, time.monotonic() + call_timeout_sec):
            return None, f'service call timeout: {service_name}'
        response = future.result()
        if not getattr(response, 'success', False):
            return response, str(getattr(response, 'message', 'service request rejected'))
        return response, None
    except Exception as exc:
        return None, f'service call error: {exc}'
    finally:
        destroy_ros_entity(client)


def stop_active_flame_tracking_if_needed(
    node: Any,
    blackboard: dict[str, Any],
    reason: str,
    force: bool = False,
) -> tuple[bool, str | None]:
    state = str(blackboard.get('flame_tracking_state', '')).strip().lower()
    may_be_active = bool(blackboard.get('flame_tracking_active', False)) or (
        state in FLAME_TRACKING_UNSAFE_STATES
    )
    if not force and not may_be_active:
        return True, None

    service_name = str(blackboard.get('flame_tracking_service_name', '/flame_arm_tracker/set_enabled'))
    service_wait_timeout_sec = float(blackboard.get('flame_tracking_service_wait_timeout_sec', 1.0))
    call_timeout_sec = float(blackboard.get('flame_tracking_call_timeout_sec', 3.0))
    previous_state = state or 'unknown'
    blackboard['flame_tracking_state'] = 'stopping'
    response, error = call_set_bool_service(
        node,
        service_name,
        False,
        service_wait_timeout_sec,
        call_timeout_sec,
    )
    stop_record = {
        'backend': 'service',
        'mode': 'stop',
        'reason': reason,
        'force': force,
        'previous_state': previous_state,
        'service_name': service_name,
        'disable_message': str(getattr(response, 'message', '')),
        'success': error is None,
    }
    if error is not None:
        blackboard['flame_tracking_active'] = True
        blackboard['flame_tracking_state'] = 'unknown'
        blackboard['flame_tracking_ready'] = False
        stop_record['message'] = f'flame tracking stop failed: {error}'
        blackboard['last_flame_tracking'] = stop_record
        return False, error

    blackboard['flame_tracking_active'] = False
    blackboard['flame_tracking_state'] = 'inactive'
    blackboard['flame_tracking_ready'] = False
    blackboard['flame_tracking_service_name'] = service_name
    blackboard['flame_tracking_service_wait_timeout_sec'] = service_wait_timeout_sec
    blackboard['flame_tracking_call_timeout_sec'] = call_timeout_sec
    stop_record['message'] = 'flame tracking stopped'
    blackboard['last_flame_tracking'] = stop_record
    return True, None


def cancel_goal_and_wait(
    goal_handle: Any,
    node: Any,
    result_future: Any | None,
    timeout_sec: float = 1.0,
) -> GoalCancelStatus:
    if not hasattr(goal_handle, 'cancel_goal_async'):
        return GoalCancelStatus(False, False, 'cancel not supported')

    try:
        cancel_future = goal_handle.cancel_goal_async()
        if not spin_until_future_done(node, cancel_future, time.monotonic() + timeout_sec):
            return GoalCancelStatus(True, False, 'cancel request timeout')

        cancel_response = cancel_future.result()
        canceling_goals = getattr(cancel_response, 'goals_canceling', []) or []
        if not canceling_goals:
            return GoalCancelStatus(True, False, 'cancel request rejected')

        if result_future is None and hasattr(goal_handle, 'get_result_async'):
            result_future = goal_handle.get_result_async()

        if result_future is not None:
            if spin_until_future_done(node, result_future, time.monotonic() + timeout_sec):
                return GoalCancelStatus(True, True, 'cancel confirmed')
            return GoalCancelStatus(True, False, 'cancel request accepted but result not confirmed')

        return GoalCancelStatus(True, True, 'cancel confirmed')
    except Exception as exc:
        return GoalCancelStatus(True, False, f'cancel error: {exc}')


def spin_until_future_done(node: Any, future: Any, deadline: float) -> bool:
    while ros_ok() and not future.done() and time.monotonic() < deadline:
        timeout_sec = min(0.05, max(deadline - time.monotonic(), 0.0))
        wait_for_callbacks(node, timeout_sec)
    return future.done()


def wait_with_spin(node: Any, seconds: float) -> None:
    deadline = time.monotonic() + max(float(seconds), 0.0)
    while ros_ok() and time.monotonic() < deadline:
        timeout_sec = min(0.05, max(deadline - time.monotonic(), 0.0))
        wait_for_callbacks(node, timeout_sec)


def try_cancel_goal(goal_handle: Any, node: Any) -> None:
    if not hasattr(goal_handle, 'cancel_goal_async'):
        return
    try:
        cancel_future = goal_handle.cancel_goal_async()
        spin_until_future_done(node, cancel_future, time.monotonic() + 1.0)
    except Exception:
        return


def should_retry_manipulation_error(error: str) -> bool:
    normalized = error.lower()
    if 'cancel not confirmed' in normalized:
        return False
    if 'cancel request timeout' in normalized:
        return False
    if 'cancel request rejected' in normalized:
        return False
    if 'cancel error' in normalized:
        return False
    return True


def destroy_ros_entity(entity: Any) -> None:
    if hasattr(entity, 'destroy'):
        entity.destroy()


def sleep_non_negative(seconds: float) -> None:
    time.sleep(max(float(seconds), 0.0))


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def ros_ok() -> bool:
    ok_function = getattr(rclpy, 'ok', None)
    if ok_function is None:
        return True
    return bool(ok_function())


def communication_callback_group(node: Any) -> Any:
    return getattr(node, 'communication_callback_group', None)


def node_uses_background_executor(node: Any) -> bool:
    return bool(getattr(node, '_venom_background_executor_active', False))


def wait_for_callbacks(node: Any, timeout_sec: float) -> None:
    if node_uses_background_executor(node):
        time.sleep(max(float(timeout_sec), 0.0))
        return
    rclpy.spin_once(node, timeout_sec=max(float(timeout_sec), 0.0))
