"""Regression tests for the temporary point 4 TEB configuration."""

from pathlib import Path

from venom_mission_commander.mission_loader import MissionLoader


MISSION_PATH = Path(__file__).resolve().parents[1] / 'config' / 'competition_10x6_arm_mission.yaml'


def test_point4_teb_parameters_are_temporary_and_restored_before_yolo():
    mission = MissionLoader().load(str(MISSION_PATH))
    waypoints = {waypoint.name: waypoint for waypoint in mission.waypoints}

    set_task = waypoints['pre_point_4_entry'].tasks[0]
    assert set_task.task_type == 'ros_parameters'
    assert set_task.params == {
        'mode': 'set',
        'node_name': '/controller_server',
        'snapshot_key': 'point4_teb_defaults',
        'parameters': {
            'FollowPath.min_obstacle_dist': 0.10,
            'FollowPath.inflation_dist': 0.25,
        },
    }

    point4_tasks = waypoints['task_point_4_classify_place'].tasks
    assert point4_tasks[0].task_type == 'ros_parameters'
    assert point4_tasks[0].params == {
        'mode': 'restore',
        'restore_snapshot_key': 'point4_teb_defaults',
    }
    parameters = set_task.params['parameters']
    assert 0 < parameters['FollowPath.min_obstacle_dist'] < parameters['FollowPath.inflation_dist']
    assert not any('costmap' in name.lower() and 'enabled' in name.lower() for name in parameters)
    assert point4_tasks[1].name == 'enable_classification_yolo_for_point_4'
