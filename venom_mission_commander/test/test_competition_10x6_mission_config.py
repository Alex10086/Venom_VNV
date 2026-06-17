"""Regression tests for CRAIC2026 competition mission configs."""

from pathlib import Path

from venom_mission_commander.mission_loader import MissionLoader


PACKAGE_DIR = Path(__file__).resolve().parents[1]
MOCK_COMPETITION_MISSION_PATH = PACKAGE_DIR / 'config' / 'competition_10x6_mission.yaml'


def test_mock_competition_route_keeps_flame_tracking_as_own_waypoint():
    mission = MissionLoader().load(str(MOCK_COMPETITION_MISSION_PATH))
    waypoint_names = [waypoint.name for waypoint in mission.waypoints]

    assert len(waypoint_names) == 8
    assert 'task_point_3_flame_tracking' in waypoint_names
    assert waypoint_names.index('task_point_3_flame_tracking') < waypoint_names.index(
        'task_point_4_classify_place'
    )
