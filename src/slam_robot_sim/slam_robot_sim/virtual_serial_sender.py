"""Send representative newline-delimited sensor JSON to a serial port."""
import argparse
import json
import time

import serial


def sample(frame):
    """Return one valid example matching the ESP32 sensor protocol."""
    return {
        's1': [1000 + frame] * 64,
        's2': [1500 + frame] * 64,
        's3': [2000 + frame] * 64,
        'bno': {
            'ok': 1, 'h': float(frame % 360), 'r': 1.0, 'p': 2.0,
            'cal': [3, 3, 3, 3],
        },
    }


def main(args=None):
    """Write sample data to an existing virtual or physical serial port."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='PTY/serial device path')
    parser.add_argument('--baud', type=int, default=921600)
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--count', type=int, default=0, help='0 means forever')
    options = parser.parse_args(args)
    if options.rate <= 0 or options.count < 0:
        parser.error('rate must be positive and count cannot be negative')

    interval = 1.0 / options.rate
    with serial.Serial(options.port, options.baud, timeout=1) as port:
        frame = 0
        deadline = time.monotonic()
        while not options.count or frame < options.count:
            port.write((json.dumps(sample(frame)) + '\n').encode('utf-8'))
            port.flush()
            frame += 1
            deadline += interval
            time.sleep(max(0.0, deadline - time.monotonic()))


if __name__ == '__main__':
    main()
