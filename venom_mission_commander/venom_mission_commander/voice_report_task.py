import os
import shlex
import shutil
import subprocess
import time
from dataclasses import dataclass
from typing import Any

from venom_mission_commander.models import TaskContext, TaskExecutionResult, TaskSpec


@dataclass(frozen=True)
class VoiceReportMockConfig:
    delay_sec: float


@dataclass(frozen=True)
class VoiceReportCommandConfig:
    command: list[str]
    timeout_sec: float
    required: bool


def execute_voice_report_task(node: Any, context: TaskContext, spec: TaskSpec) -> TaskExecutionResult:
    backend = voice_report_backend(spec.params)
    if backend in ('mock', ''):
        return execute_mock_voice_report(node, context, spec)
    if backend == 'command':
        return execute_command_voice_report(node, context, spec)
    return TaskExecutionResult(False, f'unknown voice_report backend: {backend}')


def voice_report_backend(params: dict[str, Any]) -> str:
    return str(params.get('backend', 'mock')).strip().lower()


def parse_mock_config(params: dict[str, Any]) -> VoiceReportMockConfig:
    return VoiceReportMockConfig(
        delay_sec=float(params.get('mock_delay_sec', 0.2)),
    )


def execute_mock_voice_report(
    node: Any,
    context: TaskContext,
    spec: TaskSpec,
) -> TaskExecutionResult:
    reading = context.blackboard.get('meter_reading', {})
    text = build_voice_report_text(node, reading, spec)
    config = parse_mock_config(spec.params)

    node.get_logger().info(f'[MOCK TASK] Voice report: {text}')
    sleep_non_negative(config.delay_sec)

    report = build_mock_voice_report(text)
    context.blackboard['last_voice_report'] = report
    return TaskExecutionResult(True, 'voice report completed', report)


def build_mock_voice_report(text: str) -> dict[str, Any]:
    return {'text': text, 'success': True, 'source': 'mock_tts', 'message': 'mock'}


def parse_command_config(params: dict[str, Any]) -> VoiceReportCommandConfig:
    raw_command = str(params.get('command', 'spd-say -w')).strip()
    expanded_command = os.path.expanduser(os.path.expandvars(raw_command))
    return VoiceReportCommandConfig(
        command=shlex.split(expanded_command),
        timeout_sec=float(params.get('timeout_sec', 4.0)),
        required=str(params.get('required', False)).lower() in {'1', 'true', 'yes', 'on'},
    )


def execute_command_voice_report(
    node: Any,
    context: TaskContext,
    spec: TaskSpec,
) -> TaskExecutionResult:
    reading = context.blackboard.get('meter_reading', {})
    text = build_voice_report_text(node, reading, spec)
    config = parse_command_config(spec.params)

    started_at = time.monotonic()
    success, message = run_speech_command(config.command, text, config.timeout_sec)
    report = build_command_voice_report(
        text,
        success,
        config.command,
        message,
        time.monotonic() - started_at,
    )
    context.blackboard['last_voice_report'] = report

    if success:
        return TaskExecutionResult(True, 'voice report completed', report)
    if config.required:
        return TaskExecutionResult(False, f'voice report failed: {message}', report)
    return TaskExecutionResult(True, f'voice report skipped: {message}', report)


def run_speech_command(command: list[str], text: str, timeout_sec: float) -> tuple[bool, str]:
    if not command:
        return False, 'empty speech command'
    if shutil.which(command[0]) is None:
        return False, f'speech command not found: {command[0]}'
    if timeout_sec <= 0.0:
        return False, 'timeout_sec must be positive'

    try:
        completed = subprocess.run(
            [*command, text],
            capture_output=True,
            check=False,
            text=True,
            timeout=timeout_sec,
        )
        if completed.returncode == 0:
            return True, 'speech command completed'

        message = completed.stderr.strip() or completed.stdout.strip()
        return False, message or f'speech command exit code {completed.returncode}'
    except subprocess.TimeoutExpired:
        return False, f'speech command timeout after {timeout_sec:.1f}s'
    except OSError as exc:
        return False, str(exc)


def build_command_voice_report(
    text: str,
    success: bool,
    command: list[str],
    message: str,
    duration_sec: float,
) -> dict[str, Any]:
    return {
        'text': text,
        'success': success,
        'source': 'command_tts',
        'command': command[0] if command else '',
        'message': message,
        'duration_sec': duration_sec,
    }


def build_voice_report_text(node: Any, reading: Any, spec: TaskSpec) -> str:
    reading_data = reading if isinstance(reading, dict) else {}
    default_text = (
        f"电表 {reading_data.get('meter_id', 'unknown')} 读数 "
        f"{reading_data.get('value', 'unknown')}，执行播报操作"
    )
    if 'text' in spec.params:
        return str(spec.params['text'])

    template = spec.params.get('template')
    if template is None:
        return default_text

    try:
        return str(template).format(**reading_data)
    except (KeyError, ValueError, IndexError, AttributeError, TypeError) as exc:
        node.get_logger().warning(f'Voice template render failed: {exc}')
        return default_text


def sleep_non_negative(seconds: float) -> None:
    time.sleep(max(float(seconds), 0.0))
