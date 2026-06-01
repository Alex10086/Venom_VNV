"""Tests for voice report command configuration."""

from venom_mission_commander.voice_report_task import parse_command_config


def test_parse_command_config_expands_home_environment(monkeypatch):
    monkeypatch.setenv('HOME', '/home/alex')

    config = parse_command_config({
        'command': '$HOME/venom_ws/scripts/speak_meter_wav.sh --dry-run',
    })

    assert config.command == [
        '/home/alex/venom_ws/scripts/speak_meter_wav.sh',
        '--dry-run',
    ]
