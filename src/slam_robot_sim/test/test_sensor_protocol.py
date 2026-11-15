"""Tests for the sensor protocol parser."""

import json

import pytest

from slam_robot_sim.sensor_protocol import parse_sensor_line, SensorDataError


def valid_data():
    """Return a valid example of sensor data."""
    return {
        's1': [1500] * 64,
        's2': [1500] * 64,
        's3': [1500] * 64,
        'bno': {
            'ok': 1,
            'h': 120.5,
            'r': 2.3,
            'p': -1.5,
            'cal': [3, 3, 3, 3],
        },
    }


def test_valid_sensor_data():
    """Test parsing a valid measurement."""
    line = json.dumps(valid_data())

    result = parse_sensor_line(line)

    assert result['type'] == 'measurement'
    assert result['data']['bno']['ok'] == 1


def test_invalid_readings_are_allowed():
    """Test that -1 represents an invalid distance reading."""
    data = valid_data()
    data['s1'][0] = -1

    result = parse_sensor_line(json.dumps(data))

    assert result['data']['s1'][0] == -1


def test_wrong_grid_size():
    """Test rejection of a grid containing fewer than 64 readings."""
    data = valid_data()
    data['s1'] = [1500] * 63

    with pytest.raises(SensorDataError):
        parse_sensor_line(json.dumps(data))


def test_missing_sensor():
    """Test rejection when a distance sensor is missing."""
    data = valid_data()
    del data['s2']

    with pytest.raises(SensorDataError):
        parse_sensor_line(json.dumps(data))


def test_invalid_calibration():
    """Test rejection of calibration values outside the 0-3 range."""
    data = valid_data()
    data['bno']['cal'] = [3, 3, 3, 4]

    with pytest.raises(SensorDataError):
        parse_sensor_line(json.dumps(data))


def test_status_message():
    """Test parsing an ESP32 status message."""
    line = '{"status": "ALL_READY"}'

    result = parse_sensor_line(line)

    assert result['type'] == 'status'


def test_invalid_json():
    """Test rejection of malformed JSON."""
    line = '{invalid json'

    with pytest.raises(SensorDataError):
        parse_sensor_line(line)
