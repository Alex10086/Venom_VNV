import hashlib
import json
import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from venom_mission_commander.models import TaskContext, TaskExecutionResult, TaskSpec


DEFAULT_MAX_IMAGE_BYTES = 5 * 1024 * 1024


@dataclass(frozen=True)
class HostReportConfig:
    backend: str
    report_kind: str
    reading_key: str
    image_path_key: str
    image_path: str
    output_key: str
    report_dir: str
    copy_image: bool
    required: bool
    timeout_sec: float
    retry_count: int
    retry_backoff_sec: float
    max_image_bytes: int


def execute_host_report_task(node: Any, context: TaskContext, spec: TaskSpec) -> TaskExecutionResult:
    started_at = time.monotonic()
    try:
        config = parse_host_report_config(spec.params)
    except (TypeError, ValueError) as exc:
        config = fallback_host_report_config(spec.params)
        return finish_host_report_failure(
            context,
            spec,
            config,
            f'invalid host_report parameter: {exc}',
            started_at,
        )

    validation_error = validate_config(config)
    if validation_error is not None:
        return finish_host_report_failure(context, spec, config, validation_error, started_at)

    if config.backend in ('mock', ''):
        report = build_base_report(context, spec, config, started_at)
        report.update(
            {
                'success': True,
                'message': 'mock host report completed',
                'duration_sec': time.monotonic() - started_at,
            }
        )
        context.blackboard[config.output_key] = report
        node.get_logger().info(f'[MOCK TASK] Host report: {report["report_id"]}')
        return TaskExecutionResult(True, 'host report completed', report)

    if config.backend == 'file':
        return execute_file_host_report(node, context, spec, config, started_at)

    return finish_host_report_failure(
        context,
        spec,
        config,
        f'unknown host_report backend: {config.backend}',
        started_at,
    )


def parse_host_report_config(params: dict[str, Any]) -> HostReportConfig:
    return HostReportConfig(
        backend=str(params.get('backend', 'mock')).strip().lower(),
        report_kind=str(params.get('report_kind', 'meter_reading')),
        reading_key=str(params.get('reading_key', 'meter_reading')),
        image_path_key=str(params.get('image_path_key', 'image_path')),
        image_path=str(params.get('image_path', '') or ''),
        output_key=str(params.get('output_key', 'last_host_report')),
        report_dir=str(params.get('report_dir', '/tmp/venom_host_reports')),
        copy_image=parse_bool(params.get('copy_image', True)),
        required=parse_bool(params.get('required', False)),
        timeout_sec=float(params.get('timeout_sec', 3.0)),
        retry_count=int(params.get('retry_count', 0)),
        retry_backoff_sec=float(params.get('retry_backoff_sec', 0.3)),
        max_image_bytes=int(params.get('max_image_bytes', DEFAULT_MAX_IMAGE_BYTES)),
    )


def fallback_host_report_config(params: dict[str, Any]) -> HostReportConfig:
    return HostReportConfig(
        backend=str(params.get('backend', 'mock')).strip().lower(),
        report_kind=str(params.get('report_kind', 'meter_reading')),
        reading_key=str(params.get('reading_key', 'meter_reading')),
        image_path_key=str(params.get('image_path_key', 'image_path')),
        image_path=str(params.get('image_path', '') or ''),
        output_key=str(params.get('output_key', 'last_host_report')),
        report_dir=str(params.get('report_dir', '/tmp/venom_host_reports')),
        copy_image=parse_bool(params.get('copy_image', True)),
        required=parse_bool(params.get('required', False)),
        timeout_sec=3.0,
        retry_count=0,
        retry_backoff_sec=0.3,
        max_image_bytes=DEFAULT_MAX_IMAGE_BYTES,
    )


def parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def validate_config(config: HostReportConfig) -> str | None:
    if config.timeout_sec <= 0.0:
        return 'timeout_sec must be positive'
    if config.retry_count < 0:
        return 'retry_count must be non-negative'
    if config.retry_backoff_sec < 0.0:
        return 'retry_backoff_sec must be non-negative'
    if config.max_image_bytes <= 0:
        return 'max_image_bytes must be positive'
    if config.backend == 'file' and not config.report_dir.strip():
        return 'report_dir must not be empty for file backend'
    return None


def execute_file_host_report(
    node: Any,
    context: TaskContext,
    spec: TaskSpec,
    config: HostReportConfig,
    started_at: float,
) -> TaskExecutionResult:
    reading, error = resolve_reading(context, config)
    if error is not None:
        return finish_host_report_failure(context, spec, config, error, started_at)

    image_path, error = resolve_image_path(context, config, reading)
    if error is not None:
        return finish_host_report_failure(context, spec, config, error, started_at)

    error = validate_image_file(image_path, config)
    if error is not None:
        return finish_host_report_failure(context, spec, config, error, started_at)

    attempts = config.retry_count + 1
    last_error = ''
    for attempt_index in range(attempts):
        if timed_out(started_at, config):
            return finish_host_report_failure(
                context,
                spec,
                config,
                f'file backend timeout after {config.timeout_sec:.1f}s',
                started_at,
            )
        try:
            report = write_file_host_report(context, spec, config, reading, image_path, started_at)
            context.blackboard[config.output_key] = report
            node.get_logger().info(
                f'[HOST REPORT] Saved {config.report_kind} report: {report["report_dir"]}'
            )
            return TaskExecutionResult(True, 'host report completed', report)
        except OSError as exc:
            last_error = str(exc)
        except TypeError as exc:
            last_error = str(exc)
        if attempt_index < attempts - 1:
            time.sleep(retry_sleep_sec(started_at, config))

    return finish_host_report_failure(
        context,
        spec,
        config,
        f'file backend error: {last_error}',
        started_at,
    )


def resolve_reading(
    context: TaskContext,
    config: HostReportConfig,
) -> tuple[dict[str, Any], str | None]:
    reading = context.blackboard.get(config.reading_key)
    if not isinstance(reading, dict):
        return {}, f"missing reading data: blackboard['{config.reading_key}']"
    return reading, None


def resolve_image_path(
    context: TaskContext,
    config: HostReportConfig,
    reading: dict[str, Any],
) -> tuple[Path, str | None]:
    if config.image_path:
        return Path(config.image_path).expanduser(), None

    reading_image_path = reading.get(config.image_path_key)
    if reading_image_path:
        return Path(str(reading_image_path)).expanduser(), None

    blackboard_image_path = context.blackboard.get(config.image_path_key)
    if blackboard_image_path:
        return Path(str(blackboard_image_path)).expanduser(), None

    return Path(), f"image_path missing in blackboard['{config.reading_key}']"


def validate_image_file(image_path: Path, config: HostReportConfig) -> str | None:
    if not image_path.exists():
        return f'image file not found: {image_path}'
    if not image_path.is_file():
        return f'image path is not a file: {image_path}'
    image_size = image_path.stat().st_size
    if image_size > config.max_image_bytes:
        return f'image too large: {image_size} > {config.max_image_bytes}'
    return None


def write_file_host_report(
    context: TaskContext,
    spec: TaskSpec,
    config: HostReportConfig,
    reading: dict[str, Any],
    source_image_path: Path,
    started_at: float,
) -> dict[str, Any]:
    report_id = build_report_id(context, spec, config)
    report_dir = Path(config.report_dir).expanduser() / sanitize_filename(report_id)
    report_dir.mkdir(parents=True, exist_ok=False)

    image_sha256 = sha256_file(source_image_path)
    image_path = source_image_path
    if config.copy_image:
        image_path = copied_image_path(report_dir, source_image_path, image_sha256)
        shutil.copy2(source_image_path, image_path)

    metadata_path = report_dir / 'metadata.json'
    receipt_path = report_dir / 'receipt.json'
    metadata = build_metadata(
        context,
        spec,
        config,
        reading,
        report_id,
        image_path,
        image_sha256,
    )
    write_json(metadata_path, metadata)

    report = build_base_report(context, spec, config, started_at, report_id=report_id)
    report.update(
        {
            'success': True,
            'message': 'host report saved',
            'report_dir': str(report_dir),
            'metadata_path': str(metadata_path),
            'receipt_path': str(receipt_path),
            'image_path': str(image_path),
            'image_sha256': image_sha256,
            'duration_sec': time.monotonic() - started_at,
        }
    )
    write_json(receipt_path, report)
    return report


def build_metadata(
    context: TaskContext,
    spec: TaskSpec,
    config: HostReportConfig,
    reading: dict[str, Any],
    report_id: str,
    image_path: Path,
    image_sha256: str,
) -> dict[str, Any]:
    return {
        'report_id': report_id,
        'mission_id': context.mission_id,
        'waypoint_name': context.waypoint.name,
        'waypoint_index': context.waypoint_index,
        'task_name': spec.name,
        'task_index': context.task_index,
        'report_kind': config.report_kind,
        'meter_id': reading.get('meter_id', ''),
        'value': reading.get('value', ''),
        'confidence': reading.get('confidence', 0.0),
        'source': reading.get('source', ''),
        'image_filename': image_path.name,
        'image_sha256': image_sha256,
        'timestamp_sec': time.time(),
        'reading': reading,
    }


def build_base_report(
    context: TaskContext,
    spec: TaskSpec,
    config: HostReportConfig,
    started_at: float,
    report_id: str | None = None,
) -> dict[str, Any]:
    return {
        'success': False,
        'backend': config.backend,
        'report_kind': config.report_kind,
        'report_id': report_id or build_report_id(context, spec, config),
        'mission_id': context.mission_id,
        'waypoint_name': context.waypoint.name,
        'waypoint_index': context.waypoint_index,
        'task_name': spec.name,
        'task_index': context.task_index,
        'duration_sec': time.monotonic() - started_at,
    }


def build_report_id(context: TaskContext, spec: TaskSpec, config: HostReportConfig) -> str:
    timestamp = time.strftime('%Y%m%d_%H%M%S', time.localtime())
    return '_'.join(
        [
            context.mission_id,
            context.waypoint.name,
            spec.name,
            config.report_kind,
            f'{timestamp}_{time.time_ns()}',
        ]
    )


def finish_host_report_failure(
    context: TaskContext,
    spec: TaskSpec,
    config: HostReportConfig,
    message: str,
    started_at: float,
) -> TaskExecutionResult:
    report = build_base_report(context, spec, config, started_at)
    report.update(
        {
            'success': False,
            'message': message,
            'duration_sec': time.monotonic() - started_at,
        }
    )
    context.blackboard[config.output_key] = report
    if config.required:
        return TaskExecutionResult(False, f'host report failed: {message}', report)
    return TaskExecutionResult(True, f'host report skipped: {message}', report)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as image_file:
        for chunk in iter(lambda: image_file.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, data: dict[str, Any]) -> None:
    temp_path = path.with_name(f'.{path.name}.{time.time_ns()}.tmp')
    temp_path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True, default=str),
        encoding='utf-8',
    )
    temp_path.replace(path)


def copied_image_path(report_dir: Path, source_image_path: Path, image_sha256: str) -> Path:
    suffix = source_image_path.suffix or '.bin'
    return report_dir / f'image_{image_sha256[:12]}{suffix}'


def timed_out(started_at: float, config: HostReportConfig) -> bool:
    return time.monotonic() - started_at > config.timeout_sec


def retry_sleep_sec(started_at: float, config: HostReportConfig) -> float:
    remaining_sec = config.timeout_sec - (time.monotonic() - started_at)
    return max(min(config.retry_backoff_sec, remaining_sec), 0.0)


def sanitize_filename(value: str) -> str:
    safe_chars = []
    for char in value:
        if char.isalnum() or char in {'-', '_'}:
            safe_chars.append(char)
        else:
            safe_chars.append('_')
    return ''.join(safe_chars).strip('_') or 'host_report'
