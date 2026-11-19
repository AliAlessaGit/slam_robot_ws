"""Parse the JSON lines that the ESP32 receiver writes to its USB serial port."""

# الوظيفة: تحليل رسائل JSON والتحقق من بيانات الحساسات القادمة من ESP32.
# الغرض: تجهيز القياسات وحساب تحويل اتجاه BNO إلى رباعي ROS.

from dataclasses import dataclass
import json
import math

TOF_ZONES = 64
TOF_KEYS = ('s1', 's2', 's3')
INVALID_MM = -1
MAX_DISTANCE_MM = 2**31 - 1
MAX_LINE_BYTES = 1024 * 1024
BNO_KEYS = ('ok', 'h', 'r', 'p', 'cal')


class ProtocolError(ValueError):
    """Raised when a serial line is not a valid sensor message."""


@dataclass(frozen=True)
class BnoData:
    ok: int
    heading: float
    roll: float
    pitch: float
    cal: tuple


@dataclass(frozen=True)
class Message:
    # kind is 'frame', 'status' or 'error'.
    kind: str
    text: str = ''
    tof: tuple = ()
    bno: BnoData | None = None


def _as_int(value, what):
    if isinstance(value, bool):
        raise ProtocolError(f'{what} must be a number, not a boolean')
    if isinstance(value, int):
        return value
    if isinstance(value, float) and math.isfinite(value) and value.is_integer():
        return int(value)
    raise ProtocolError(f'{what} must be an integer, got {value!r}')


def _as_float(value, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f'{what} must be a number, got {value!r}')
    try:
        number = float(value)
    except OverflowError as exc:
        raise ProtocolError(f'{what} is too large') from exc
    if not math.isfinite(number):
        raise ProtocolError(f'{what} must be finite')
    return number


def _parse_tof(obj, key):
    values = obj.get(key)
    if not isinstance(values, list):
        raise ProtocolError(f'{key} is missing or is not a list')
    if len(values) != TOF_ZONES:
        raise ProtocolError(f'{key} has {len(values)} values, expected {TOF_ZONES}')
    result = []
    for index, raw in enumerate(values):
        mm = _as_int(raw, f'{key}[{index}]')
        if mm != INVALID_MM and not 0 < mm <= MAX_DISTANCE_MM:
            raise ProtocolError(f'{key}[{index}] = {mm} is out of range')
        result.append(mm)
    return tuple(result)


def _parse_bno(obj):
    bno = obj.get('bno')
    if not isinstance(bno, dict):
        raise ProtocolError('bno is missing or is not an object')
    missing = [key for key in BNO_KEYS if key not in bno]
    if missing:
        raise ProtocolError(f'bno is missing keys: {missing}')

    ok = _as_int(bno['ok'], 'bno.ok')
    if ok not in (0, 1):
        raise ProtocolError(f'bno.ok must be 0 or 1, got {ok}')

    cal_raw = bno['cal']
    if not isinstance(cal_raw, list) or len(cal_raw) != 4:
        raise ProtocolError('bno.cal must be a list of 4 values')
    cal = tuple(_as_int(level, f'bno.cal[{i}]') for i, level in enumerate(cal_raw))
    for i, level in enumerate(cal):
        if not 0 <= level <= 3:
            raise ProtocolError(f'bno.cal[{i}] = {level} must be between 0 and 3')

    return BnoData(
        ok=ok,
        heading=_as_float(bno['h'], 'bno.h'),
        roll=_as_float(bno['r'], 'bno.r'),
        pitch=_as_float(bno['p'], 'bno.p'),
        cal=cal,
    )


def parse_line(line):
    """Parse one serial line and return a Message, or raise ProtocolError."""
    if isinstance(line, (bytes, bytearray)):
        try:
            line = bytes(line).decode('utf-8')
        except UnicodeDecodeError as exc:
            raise ProtocolError(f'line is not valid UTF-8: {exc}') from exc
    text = line.strip()
    if not text:
        raise ProtocolError('empty line')
    try:
        obj = json.loads(text)
    except (ValueError, RecursionError) as exc:
        raise ProtocolError(f'invalid JSON: {exc}') from exc
    if not isinstance(obj, dict):
        raise ProtocolError('top-level JSON value must be an object')

    if 'error' in obj:
        return Message(kind='error', text=str(obj['error']))
    if 'status' in obj:
        return Message(kind='status', text=str(obj['status']))

    tof = tuple(_parse_tof(obj, key) for key in TOF_KEYS)
    return Message(kind='frame', tof=tof, bno=_parse_bno(obj))


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """Convert roll, pitch and yaw in degrees to a quaternion (x, y, z, w)."""
    roll = math.radians(roll_deg) / 2.0
    pitch = math.radians(pitch_deg) / 2.0
    yaw = math.radians(yaw_deg) / 2.0
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class LineBuffer:
    """Collect serial bytes and hand back complete lines."""

    def __init__(self, max_line_bytes=MAX_LINE_BYTES):
        self._buffer = bytearray()
        self._max_line_bytes = max_line_bytes
        self._discarding = False
        self.overflows = 0

    def feed(self, data):
        """Add bytes and return the list of complete, non-empty lines."""
        lines = []
        while data:
            index = data.find(b'\n')
            if index < 0:
                piece, data, terminated = data, b'', False
            else:
                piece, data, terminated = data[:index], data[index + 1:], True

            if self._discarding:
                # Skip the rest of an oversized line, up to its newline.
                self._discarding = not terminated
                continue

            self._buffer += piece
            if len(self._buffer) > self._max_line_bytes:
                self._buffer.clear()
                self.overflows += 1
                self._discarding = not terminated
                continue

            if terminated:
                line = bytes(self._buffer).rstrip(b'\r')
                self._buffer.clear()
                if line.strip():
                    lines.append(line)
        return lines
