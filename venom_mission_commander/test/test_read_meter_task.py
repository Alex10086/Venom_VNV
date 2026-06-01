"""Tests for read-meter task configuration defaults."""

from venom_mission_commander.read_meter_task import parse_mock_config


def test_mock_read_meter_default_matches_four_digit_printed_number_contract():
    config = parse_mock_config({})

    assert config.value == '1234'
    assert str(config.value).isdigit()
