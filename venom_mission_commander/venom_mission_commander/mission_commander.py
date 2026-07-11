import sys
from pathlib import Path
from threading import Thread

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from venom_mission_commander.mission_loader import MissionLoader
from venom_mission_commander.mission_manager import MissionManager
from venom_mission_commander.models import MissionState, TaskContext, WaypointSpec
from venom_mission_commander.navigator import MockWaypointNavigator, Nav2WaypointNavigator
from venom_mission_commander.status_reporter import MissionStatusReporter
from venom_mission_commander.startup_checks import StartupChecker
from venom_mission_commander.task_plugins import TaskPluginRegistry
from venom_mission_commander.task_runner import WaypointTaskRunner
from venom_mission_commander.arm_task_client import stop_active_flame_tracking_if_needed
from venom_mission_commander.perception_control_task import call_set_enabled


class MissionCommander(Node):
    def __init__(self):
        super().__init__('mission_commander')

        default_config = self._default_config_path()
        self._declare_parameter_if_needed(
            'mission_config',
            default_config,
            ParameterDescriptor(description='Absolute path to mission YAML.'),
        )
        self._declare_parameter_if_needed(
            'use_nav',
            False,
            ParameterDescriptor(description='Use Nav2 instead of mock navigation.'),
        )
        self._declare_parameter_if_needed(
            'mock_nav_delay_sec',
            0.5,
            ParameterDescriptor(description='Mock navigation delay per waypoint.'),
        )
        self._declare_parameter_if_needed(
            'nav2_wait_mode',
            'bt_navigator',
            ParameterDescriptor(description='Nav2 wait mode: bt_navigator or full.'),
        )
        self._declare_parameter_if_needed(
            'navigator_ready_timeout_sec',
            30.0,
            ParameterDescriptor(description='Startup wait timeout for mock/Nav2 navigator readiness.'),
        )
        self._declare_parameter_if_needed(
            'nav_feedback_log_interval_sec',
            5.0,
            ParameterDescriptor(description='Seconds between periodic Nav2 feedback logs; 0 disables.'),
        )
        self._declare_parameter_if_needed(
            'log_state_transitions',
            False,
            ParameterDescriptor(description='Print low-level MissionManager state transitions.'),
        )
        self._declare_parameter_if_needed(
            'use_sim_time',
            False,
            ParameterDescriptor(description='Use Gazebo /clock for simulation runs.'),
        )
        self._declare_parameter_if_needed(
            'perception_cleanup_services',
            [
                '/pick_yolo_detector/set_enabled',
                '/digit_yolo_detector/set_enabled',
                '/flame_yolo_detector/set_enabled',
                '/classification_yolo_detector/set_enabled',
            ],
            ParameterDescriptor(description='SetBool services to disable at mission exit/failure.'),
        )

        self.loader = MissionLoader()
        self.mission_manager = MissionManager(self.get_logger())
        self.registry = TaskPluginRegistry()
        self.status_reporter = MissionStatusReporter(self.get_logger())
        self.task_runner = WaypointTaskRunner(
            self.registry,
            self.mission_manager,
            self.status_reporter,
        )
        self.communication_callback_group = ReentrantCallbackGroup()
        self.blackboard = {}

        self.mission_config = None
        self.navigator = None
        self._venom_background_executor_active = False

    def configure(self) -> bool:
        config_path = self.get_parameter('mission_config').value
        use_nav = bool(self.get_parameter('use_nav').value)
        mock_nav_delay_sec = float(self.get_parameter('mock_nav_delay_sec').value)
        if mock_nav_delay_sec < 0.0:
            raise ValueError('mock_nav_delay_sec must be non-negative')
        nav2_wait_mode = str(self.get_parameter('nav2_wait_mode').value)
        if nav2_wait_mode not in {'bt_navigator', 'full'}:
            raise ValueError('nav2_wait_mode must be one of: bt_navigator, full')
        self.navigator_ready_timeout_sec = float(
            self.get_parameter('navigator_ready_timeout_sec').value
        )
        if self.navigator_ready_timeout_sec <= 0.0:
            raise ValueError('navigator_ready_timeout_sec must be positive')
        nav_feedback_log_interval_sec = float(
            self.get_parameter('nav_feedback_log_interval_sec').value
        )
        if nav_feedback_log_interval_sec < 0.0:
            raise ValueError('nav_feedback_log_interval_sec must be non-negative')
        self.mission_manager.set_transition_logging(
            bool(self.get_parameter('log_state_transitions').value)
        )
        use_sim_time = bool(self.get_parameter('use_sim_time').value)

        self.get_logger().info(f'Loading mission config: {config_path}')
        self.mission_config = self.loader.load(config_path)
        self.registry.register_default_plugins(self)
        self.navigator = self._create_navigator(
            use_nav,
            mock_nav_delay_sec,
            nav2_wait_mode,
            use_sim_time,
            nav_feedback_log_interval_sec,
        )
        self.mission_manager.create_mission(self.mission_config)

        self.get_logger().info(
            f'Configured mission {self.mission_config.mission_id} with '
            f'{len(self.mission_config.waypoints)} waypoint(s).'
        )
        return True

    def run(self) -> bool:
        if self.mission_config is None or self.navigator is None:
            raise RuntimeError('MissionCommander.configure() must be called before run().')

        uses_flame_tracking = self.mission_uses_service_flame_tracking()
        if uses_flame_tracking:
            if not self.stop_active_flame_tracking('mission startup', force=True):
                self.mission_manager.fail('failed to stop flame tracking before mission startup')
                self.status_reporter.log_snapshot('mission_failed', self.mission_manager)
                return False

        if not self.run_startup_checks():
            return False

        loop_count = 0
        self.mission_manager.save_state(phase='running')
        self.mission_manager.transition_to(MissionState.RUNNING, 'mission started')
        self.status_reporter.log_snapshot('mission_started', self.mission_manager)

        while rclpy.ok():
            loop_count += 1
            self.mission_manager.save_state(loop_count=loop_count)
            success = self.run_once()

            if not success:
                self.stop_active_flame_tracking('mission failure', force=uses_flame_tracking)
                self.cleanup_perception('mission failure')
                self.status_reporter.log_snapshot('mission_failed', self.mission_manager)
                return False

            if not self.mission_config.loop:
                if not self.stop_active_flame_tracking(
                    'mission completed',
                    force=uses_flame_tracking,
                ):
                    self.mission_manager.fail(
                        'failed to stop flame tracking before mission completion'
                    )
                    self.status_reporter.log_snapshot('mission_failed', self.mission_manager)
                    return False
                self.cleanup_perception('mission completed')
                self.mission_manager.mark_mission_completed()
                self.status_reporter.log_snapshot('mission_completed', self.mission_manager)
                return self.mission_manager.state == MissionState.COMPLETED

            self.get_logger().info('Mission loop enabled; restarting route.')

        self.mission_manager.fail('rclpy shutdown requested')
        self.status_reporter.log_snapshot('mission_failed', self.mission_manager)
        return False

    def cleanup_perception(self, reason: str) -> bool:
        services = [
            str(service).strip()
            for service in self.get_parameter('perception_cleanup_services').value
            if str(service).strip()
        ]
        ok = True
        for service_name in services:
            response, error = call_set_enabled(
                self,
                service_name=service_name,
                enabled=False,
                timeout_sec=2.0,
                service_wait_timeout_sec=0.2,
            )
            if error is not None:
                self.get_logger().debug(
                    f'Perception cleanup skipped {service_name} during {reason}: {error}'
                )
                continue
            if not response.success:
                ok = False
                self.get_logger().warn(
                    f'Perception cleanup failed for {service_name}: {response.message}'
                )
        return ok

    def run_startup_checks(self) -> bool:
        if self.mission_config is None or self.navigator is None:
            raise RuntimeError('MissionCommander.configure() must be called before startup checks.')

        self.mission_manager.mark_startup_checks_started()
        self.status_reporter.log_snapshot('startup_checks_started', self.mission_manager)

        checker = StartupChecker(
            mission_config=self.mission_config,
            registry=self.registry,
            navigator=self.navigator,
            navigator_ready_timeout_sec=self.navigator_ready_timeout_sec,
            node=self,
        )
        results = checker.run()
        result_records = [result.as_dict() for result in results]
        checks_passed = all(result.success for result in results)
        startup_status = 'passed' if checks_passed else 'failed'
        self.mission_manager.mark_startup_checks_completed(startup_status, result_records)
        self.status_reporter.log_startup_check_results(results)
        self.status_reporter.log_snapshot('startup_checks_completed', self.mission_manager)

        if checks_passed:
            return True

        failures = [result.name for result in results if not result.success]
        self.mission_manager.fail(f'startup checks failed: {", ".join(failures)}')
        self.status_reporter.log_snapshot('mission_failed', self.mission_manager)
        return False

    def run_once(self) -> bool:
        for waypoint_index, waypoint in enumerate(self.mission_config.waypoints):
            if not self.run_waypoint(waypoint_index, waypoint):
                return False
        return True

    def run_waypoint(self, waypoint_index: int, waypoint: WaypointSpec) -> bool:
        self.get_logger().debug(
            f'Waypoint {waypoint_index + 1}/{len(self.mission_config.waypoints)}: '
            f'{waypoint.name} ({waypoint.kind.value})'
        )
        self.mission_manager.mark_waypoint_started(
            waypoint_index,
            waypoint.name,
            waypoint.kind.value,
        )
        self.status_reporter.log_snapshot('waypoint_started', self.mission_manager)

        if waypoint.skip_navigation:
            self.get_logger().debug(f'Skipping navigation for waypoint: {waypoint.name}')
            self.mission_manager.save_state(phase='navigation_skipped')
            self.status_reporter.log_snapshot('navigation_skipped', self.mission_manager)
        elif not self.navigate_to_waypoint(waypoint):
            self.handle_navigation_failure(waypoint)
            return False

        context = TaskContext(
            node=self,
            mission_id=self.mission_config.mission_id,
            waypoint=waypoint,
            waypoint_index=waypoint_index,
            task_index=0,
            mission_manager=self.mission_manager,
            blackboard=self.blackboard,
        )
        tasks_ok = self.task_runner.run_tasks(
            context=context,
            tasks=waypoint.tasks,
            stop_on_failure=self.mission_config.stop_on_task_failure,
        )
        if not tasks_ok:
            return False

        self.mission_manager.mark_waypoint_done(waypoint_index, waypoint.name)
        self.status_reporter.log_snapshot('waypoint_completed', self.mission_manager)
        return True

    def navigate_to_waypoint(self, waypoint: WaypointSpec) -> bool:
        max_attempts = waypoint.retry_count + 1
        for attempt in range(1, max_attempts + 1):
            self.mission_manager.save_state(
                navigation_attempt=attempt,
                navigation_attempts=max_attempts,
                navigation_timeout_sec=waypoint.nav_timeout_sec,
                last_navigation_success=None,
            )
            self.status_reporter.log_snapshot('navigation_attempt_started', self.mission_manager)

            if attempt > 1:
                self.get_logger().warning(
                    f'Retrying navigation to {waypoint.name}: attempt {attempt}/{max_attempts}'
                )

            self.navigator.go_to_waypoint(waypoint)
            if self.navigator.wait_until_done(timeout_sec=waypoint.nav_timeout_sec):
                self.mission_manager.save_state(
                    last_navigation_success=True,
                    last_navigation_waypoint=waypoint.name,
                    last_navigation_attempt=attempt,
                )
                self.status_reporter.log_snapshot(
                    'navigation_attempt_succeeded',
                    self.mission_manager,
                )
                return True

            self.mission_manager.save_state(
                last_navigation_success=False,
                last_navigation_waypoint=waypoint.name,
                last_navigation_attempt=attempt,
            )
            self.status_reporter.log_snapshot('navigation_attempt_failed', self.mission_manager)
            if not self.stop_active_flame_tracking(
                'navigation attempt failed',
                force=self.mission_uses_service_flame_tracking(),
            ):
                return False

            if attempt < max_attempts:
                cancel_confirmed = self.navigator.cancel()
                recovery_performed = self.navigator.recover()
                self.mission_manager.save_state(
                    last_navigation_cancel_confirmed=cancel_confirmed,
                    last_navigation_recovery_performed=recovery_performed,
                )

        return False

    def handle_navigation_failure(self, waypoint: WaypointSpec) -> None:
        cancel_confirmed = self.navigator.cancel()
        self.stop_active_flame_tracking(
            'navigation failure',
            force=self.mission_uses_service_flame_tracking(),
        )
        self.mission_manager.save_state(
            last_navigation_cancel_confirmed=cancel_confirmed,
            last_navigation_recovery_performed=False,
        )
        self.mission_manager.fail(f'navigation failed at waypoint: {waypoint.name}')

    def stop_active_flame_tracking(self, reason: str, force: bool = False) -> bool:
        stopped, error = stop_active_flame_tracking_if_needed(
            self,
            self.blackboard,
            reason,
            force=force,
        )
        if not stopped:
            self.get_logger().error(f'Failed to stop flame tracking ({reason}): {error}')
        return stopped

    def mission_uses_service_flame_tracking(self) -> bool:
        if self.mission_config is None:
            return False
        for waypoint in self.mission_config.waypoints:
            for task in waypoint.tasks:
                if task.task_type != 'track_flame':
                    continue
                backend = str(task.params.get('backend', 'mock')).strip().lower()
                if backend in {'service', 'tracker'}:
                    return True
        return False

    def shutdown(self) -> None:
        self.stop_active_flame_tracking(
            'mission commander shutdown',
            force=self.mission_uses_service_flame_tracking(),
        )
        self.cleanup_perception('mission commander shutdown')
        if self.navigator is not None:
            self.navigator.shutdown()
        self.destroy_node()

    def _declare_parameter_if_needed(
        self,
        name: str,
        value,
        descriptor: ParameterDescriptor,
    ) -> None:
        if self.has_parameter(name):
            return
        self.declare_parameter(name, value, descriptor)

    def _create_navigator(
        self,
        use_nav: bool,
        mock_nav_delay_sec: float,
        nav2_wait_mode: str,
        use_sim_time: bool,
        nav_feedback_log_interval_sec: float,
    ):
        if use_nav:
            return Nav2WaypointNavigator(
                self,
                wait_mode=nav2_wait_mode,
                use_sim_time=use_sim_time,
                feedback_log_interval_sec=nav_feedback_log_interval_sec,
            )
        return MockWaypointNavigator(self, delay_sec=mock_nav_delay_sec)

    def _default_config_path(self) -> str:
        try:
            package_share = Path(get_package_share_directory('venom_mission_commander'))
            return str(package_share / 'config' / 'simple_mission.yaml')
        except PackageNotFoundError:
            package_root = Path(__file__).resolve().parents[1]
            return str(package_root / 'config' / 'simple_mission.yaml')


def main(args=None) -> None:
    rclpy.init(args=args)
    commander = MissionCommander()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(commander)
    commander._venom_background_executor_active = True
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    exit_code = 1

    try:
        if commander.configure():
            exit_code = 0 if commander.run() else 1
    except KeyboardInterrupt:
        commander.get_logger().warning('Interrupted by user.')
    except Exception as exc:
        commander.get_logger().error(f'Mission commander failed: {exc}')
    finally:
        commander.status_reporter.log_final_summary(commander.mission_manager)
        commander.shutdown()
        commander._venom_background_executor_active = False
        executor.shutdown()
        executor_thread.join(timeout=2.0)
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
