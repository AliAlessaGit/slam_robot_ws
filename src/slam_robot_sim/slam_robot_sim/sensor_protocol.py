
"""Validate and parse JSON messages received from the robot."""

import json
import math


class SensorDataError(ValueError):
    """Raise when sensor data does not match the expected format."""


def parse_sensor_line(line):
    """
    Parse and validate one JSON line from the ESP32 or simulator.

    Returns
    -------
    dict
        A dictionary with type 'measurement' or 'status'.

    """
    try:
        if isinstance(line, bytes):
            line = line.decode('utf-8', errors='strict')

        data = json.loads(line.strip())

    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise SensorDataError('Invalid JSON line') from exc

    if not isinstance(data, dict):
        raise SensorDataError('JSON root must be an object')

    # ESP32 sends status messages during initialization.
    if 'status' in data or 'error' in data:
        return {
            'type': 'status',
            'data': data,
        }

    # Validate the three distance sensors.
    for sensor in ('s1', 's2', 's3'):
        if sensor not in data:
            raise SensorDataError(f'Missing {sensor}')

        values = data[sensor]

        if not isinstance(values, list) or len(values) != 64:
            raise SensorDataError(
                f'{sensor} must contain exactly 64 readings'
            )

        for value in values:
            if type(value) is not int:
                raise SensorDataError(
                    f'{sensor} contains a non-integer reading'
                )

            if value != -1 and value <= 0:
                raise SensorDataError(
                    f'{sensor} contains an invalid distance'
                )

    # Validate BNO055.
    bno = data.get('bno')

    if not isinstance(bno, dict):
        raise SensorDataError('Missing or invalid BNO055 data')

    required = ('ok', 'h', 'r', 'p', 'cal')

    for key in required:
        if key not in bno:
            raise SensorDataError(
                f'Missing BNO055 field: {key}'
            )

    if type(bno['ok']) is not int or bno['ok'] not in (0, 1):
        raise SensorDataError('BNO055 ok must be 0 or 1')

    for key in ('h', 'r', 'p'):
        value = bno[key]

        if isinstance(value, bool) or not isinstance(
            value, (int, float)
        ):
            raise SensorDataError(
                f'Invalid BNO055 value: {key}'
            )

        if not math.isfinite(value):
            raise SensorDataError(
                f'Non-finite BNO055 value: {key}'
            )

    calibration = bno['cal']

    if not isinstance(calibration, list) or len(calibration) != 4:
        raise SensorDataError(
            'Calibration must contain 4 values'
        )

    for value in calibration:
        if type(value) is not int or value not in range(4):
            raise SensorDataError(
                'Calibration values must be 0-3'
            )

    return {
        'type': 'measurement',
        'data': data,
    }
