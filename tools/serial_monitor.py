#!/usr/bin/env python3
"""Print newline-delimited data from the ESP32 USB serial port."""

# الوظيفة: قراءة وعرض الأسطر الواردة من منفذ ESP32 التسلسلي.
# الغرض: مراقبة JSON الحساسات مع إعادة الاتصال عند انقطاع المنفذ.

import argparse
import sys
import time

import serial


def parse_args():
    """Parse serial connection options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyACM0', help='serial device')
    parser.add_argument('--baud', type=int, default=921600, help='baud rate')
    parser.add_argument(
        '--reconnect-delay', type=float, default=2.0,
        help='seconds to wait before reconnecting')
    return parser.parse_args()


def main():
    """Read and print serial lines until interrupted."""
    args = parse_args()
    if args.baud <= 0 or args.reconnect_delay < 0:
        print('baud must be positive and reconnect delay non-negative', file=sys.stderr)
        return 2

    try:
        while True:
            try:
                print(
                    f'Opening {args.port} at {args.baud} baud...',
                    file=sys.stderr,
                    flush=True,
                )
                with serial.Serial(args.port, args.baud, timeout=1) as device:
                    print('Connected. Press Ctrl+C to stop.', file=sys.stderr, flush=True)
                    while True:
                        line = device.readline()
                        if line:
                            print(line.decode('utf-8', errors='replace').rstrip(), flush=True)
            except (serial.SerialException, OSError) as exc:
                print(f'Serial port unavailable: {exc}', file=sys.stderr, flush=True)
                time.sleep(args.reconnect_delay)
    except KeyboardInterrupt:
        print('\nStopped.', file=sys.stderr)
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
