"""Regression tests for CRAIC2026 competition mission configs."""

from pathlib import Path

from venom_mission_commander.mission_loader import MissionLoader


PACKAGE_DIR = Path(__file__).resolve().parents[1]
ARM_COMPETITION_MISSION_PATH = PACKAGE_DIR / 'config' / 'competition_10x6_arm_mission.yaml'


def test_arm_competition_route_uses_latest_navigation_waypoints():
    mission = MissionLoader().load(str(ARM_COMPETITION_MISSION_PATH))
    actual_route = [
        (
            waypoint.name,
            waypoint.kind.value,
            waypoint.frame_id,
            waypoint.x,
            waypoint.y,
            waypoint.yaw,
            waypoint.nav_timeout_sec,
        )
        for waypoint in mission.waypoints
    ]

    assert actual_route == [
        ('start_area', 'operation_stop', 'map', 0.0, 0.0, 0.0, 350.0),
        ('pass_speed_bump_lower', 'pass_through', 'map', 2.28, 0.0, 0.0, 200.0),
        ('task_point_1_pick', 'operation_stop', 'map', 4.08, 0.02, 1.5708, 350.0),
        ('task_point_2_meter_voice', 'operation_stop', 'map', 4.51, 3.86, -3.1416, 350.0),
        ('task_point_3_flame_tracking', 'operation_stop', 'map', 2.12, 4.04, -3.1416, 350.0),
        ('pre_point_4_entry', 'operation_stop', 'map', 0.405, 2.0, -1.5708, 120.0),
        ('task_point_4_classify_place', 'operation_stop', 'map', 0.405, 4.04, -1.5708, 350.0),
        ('return_start_area', 'return_park', 'map', 0.0, 0.0, 0.0, 450.0),
    ]
