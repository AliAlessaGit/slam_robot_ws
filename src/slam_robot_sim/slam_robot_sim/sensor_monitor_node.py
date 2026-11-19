"""Monitor ROS sensor topics and periodically report their health."""

# الوظيفة: متابعة صحة وتواتر قراءات الحساسات المنشورة على ROS.
# الغرض: اكتشاف القراءات غير الصالحة أو المتوقفة وإصدار تقرير الجودة.

import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Imu

from std_msgs.msg import Int32MultiArray, String, UInt8MultiArray

SEVERITY = {'OK': 0, 'WARN': 1, 'INVALID': 2, 'ERROR': 2, 'STALE': 2}


def worst(statuses):
    """Return the most severe status of a list."""
    return max(statuses, key=lambda status: SEVERITY[status])


class SensorMonitor(Node):
    """Track sensor rates, ToF validity, staleness, and BNO055 state."""

    def __init__(self, **kwargs):
        """Create subscriptions and configurable monitor timers."""
        super().__init__('sensor_monitor', **kwargs)
        for name, value in (
            ('report_period_sec', 1.0), ('stale_timeout_sec', 2.0),
            ('rate_window_sec', 5.0),
            ('tof_min_mm', 0),
            ('tof_max_mm', 0),
            ('jump_threshold_mm', 0),
            ('invalid_warning_ratio', 0.10),
            ('invalid_error_ratio', 0.50),
            ('stuck_min_repeats', 20),
            ('stuck_min_duration_sec', 2.0),
            ('imu_quaternion_tolerance', 0.05),
        ):
            self.declare_parameter(name, value)
        self.report_period = float(
            self.get_parameter('report_period_sec').value
        )
        self.stale_timeout = float(
            self.get_parameter('stale_timeout_sec').value
        )
        self.rate_window = float(
            self.get_parameter('rate_window_sec').value
        )
        self.tof_min = int(self.get_parameter('tof_min_mm').value)
        self.tof_max = int(self.get_parameter('tof_max_mm').value)
        self.jump_limit = int(self.get_parameter('jump_threshold_mm').value)
        self.invalid_warning = float(
            self.get_parameter('invalid_warning_ratio').value
        )
        self.invalid_error = float(
            self.get_parameter('invalid_error_ratio').value
        )
        self.stuck_repeats = int(
            self.get_parameter('stuck_min_repeats').value
        )
        self.stuck_duration = float(
            self.get_parameter('stuck_min_duration_sec').value
        )
        self.imu_tolerance = float(
            self.get_parameter('imu_quaternion_tolerance').value
        )
        intervals = (self.report_period, self.stale_timeout, self.rate_window)
        if min(intervals) <= 0 or self.stuck_duration <= 0:
            raise ValueError('Monitor periods must be positive')
        if self.tof_min < 0 or self.tof_max < 0 or self.jump_limit < 0:
            raise ValueError('ToF limits must be non-negative; zero disables')
        if not 0 <= self.invalid_warning < self.invalid_error <= 1:
            raise ValueError('Invalid ratios must satisfy 0 <= warning < error <= 1')
        if self.stuck_repeats < 2 or self.imu_tolerance < 0:
            raise ValueError('Stuck repeats must be >= 2; tolerance >= 0')
        if self.tof_min and self.tof_max and self.tof_min >= self.tof_max:
            raise ValueError('tof_min_mm must be less than tof_max_mm')
        self.tof_stats = {
            name: {'count': 0, 'invalid': 0, 'out_of_range': 0,
                   'jumps': 0, 'total': 0, 'valid': 0, 'times': [],
                   'last': None, 'previous': None, 'repeat_count': 0,
                   'repeat_since': None, 'stuck': False,
                   'current_findings': [],
                   'window_total': 0, 'window_invalid': 0,
                   'window_out_of_range': 0, 'window_jumps': 0}
            for name in ('sensor_1', 'sensor_2', 'sensor_3')
        }
        self.imu_count = self.cal_count = 0
        self.imu_times = []
        self.cal_times = []
        self.imu_last = self.cal_last = None
        self.imu_valid = False
        self.imu_norm = None
        self.imu_norm_ok = False
        self.calibration = None
        self.cal_valid = False
        self.imu_status = 'STALE'
        self.cal_status = 'STALE'
        self.stale_sensors = set()
        self.quality_pub = self.create_publisher(
            String, '/sensors/quality_report', 10
        )
        self.monitor_subscriptions = []
        for name in self.tof_stats:
            self.monitor_subscriptions.append(self.create_subscription(
                Int32MultiArray, f'/sensors/tof/{name}',
                lambda msg, key=name: self.on_tof(key, msg), 10))
        self.monitor_subscriptions.extend((
            self.create_subscription(
                Imu, '/sensors/bno055/imu', self.on_imu, 10
            ),
            self.create_subscription(
                UInt8MultiArray, '/sensors/bno055/calibration',
                self.on_calibration, 10
            ),
        ))
        self.timer = self.create_timer(self.report_period, self.report)
        self.get_logger().info(
            f'Monitor started: report={self.report_period}s, '
            f'stale={self.stale_timeout}s, window={self.rate_window}s'
        )

    def on_tof(self, sensor, message):
        """Count ToF messages, invalid -1 readings, range errors, jumps, and repeats."""
        now = time.monotonic()
        stats = self.tof_stats[sensor]
        values = list(message.data)
        bad = sum(value == -1 for value in values)
        stats['count'] += 1
        stats['total'] += len(values)
        stats['invalid'] += bad
        stats['valid'] += len(values) - bad
        stats['window_total'] += len(values)
        stats['window_invalid'] += bad
        stats['last'] = now
        stats['times'].append(now)
        cutoff = now - self.rate_window
        while len(stats['times']) > 1 and stats['times'][1] < cutoff:
            stats['times'].pop(0)
        self._check_range(stats, values)
        self._check_jumps(stats, values)
        self._check_stuck(stats, values, now)
        stats['previous'] = values

    def _check_range(self, stats, values):
        if not (self.tof_min or self.tof_max):
            return
        outside = 0
        for value in values:
            if value == -1:
                continue
            if self.tof_min and value < self.tof_min:
                outside += 1
            elif self.tof_max and value > self.tof_max:
                outside += 1
        stats['out_of_range'] += outside
        stats['window_out_of_range'] += outside

    def _check_jumps(self, stats, values):
        previous = stats['previous']
        if not self.jump_limit or previous is None or len(previous) != len(values):
            return
        jumps = sum(
            1 for old, new in zip(previous, values)
            if old != -1 and new != -1 and abs(new - old) > self.jump_limit
        )
        stats['jumps'] += jumps
        stats['window_jumps'] += jumps

    def _check_stuck(self, stats, values, now):
        if stats['previous'] is not None and values == stats['previous']:
            if stats['repeat_count'] == 0:
                stats['repeat_since'] = now
            stats['repeat_count'] += 1
            long_enough = now - stats['repeat_since'] >= self.stuck_duration
            stats['stuck'] = stats['repeat_count'] >= self.stuck_repeats and long_enough
        else:
            stats['repeat_count'] = 0
            stats['repeat_since'] = None
            stats['stuck'] = False

    def on_imu(self, message):
        """Record IMU arrival and validate the orientation quaternion."""
        q = message.orientation
        components = (q.x, q.y, q.z, q.w)
        self.imu_valid = all(math.isfinite(value) for value in components)
        if self.imu_valid:
            self.imu_norm = math.sqrt(sum(value * value for value in components))
            self.imu_norm_ok = abs(self.imu_norm - 1.0) <= self.imu_tolerance
        else:
            self.imu_norm = None
            self.imu_norm_ok = False
        self.imu_count += 1
        self.imu_last = time.monotonic()
        self.imu_times.append(self.imu_last)
        self._trim_times(self.imu_times, self.imu_last)

    def on_calibration(self, message):
        """Record four BNO055 calibration levels from zero through three."""
        values = list(message.data)
        self.cal_count += 1
        self.cal_last = time.monotonic()
        self.cal_times.append(self.cal_last)
        self._trim_times(self.cal_times, self.cal_last)
        self.calibration = values if len(values) == 4 and all(
            value in range(4) for value in values) else None
        self.cal_valid = self.calibration is not None
        if self.calibration is None:
            self.get_logger().warning('Invalid BNO055 calibration values')

    def _trim_times(self, times, now):
        cutoff = now - self.rate_window
        while len(times) > 1 and times[1] < cutoff:
            times.pop(0)

    @staticmethod
    def rate(stats):
        """Return timestamp-based message rate in hertz."""
        times = stats['times'] if isinstance(stats, dict) else stats
        if len(times) < 2 or times[-1] <= times[0]:
            return 0.0
        return (len(times) - 1) / (times[-1] - times[0])

    def _stale(self, name, last, now):
        stale = last is None or now - last > self.stale_timeout
        if stale and name not in self.stale_sensors:
            self.get_logger().warning(f'No recent data from {name}')
        elif not stale and name in self.stale_sensors:
            self.get_logger().info(f'Data resumed from {name}')
        (self.stale_sensors.add if stale else self.stale_sensors.discard)(name)
        return stale

    def _tof_status(self, stats, stale):
        if stale:
            return 'STALE', ['no recent data']
        status = 'OK'
        findings = []
        if stats['window_total']:
            ratio = stats['window_invalid'] / stats['window_total']
            if ratio >= self.invalid_error:
                status = 'ERROR'
                findings.append(f'{ratio:.0%} invalid readings')
            elif ratio >= self.invalid_warning:
                status = 'WARN'
                findings.append(f'{ratio:.0%} invalid readings')
        if stats['window_out_of_range']:
            status = worst([status, 'WARN'])
            findings.append(f'{stats["window_out_of_range"]} readings out of range')
        if stats['window_jumps']:
            status = worst([status, 'WARN'])
            findings.append(f'{stats["window_jumps"]} sudden jumps')
        if stats['stuck']:
            status = 'ERROR'
            findings.append('readings are stuck')
        return status, findings

    @staticmethod
    def _reset_window(stats):
        stats['window_total'] = 0
        stats['window_invalid'] = 0
        stats['window_out_of_range'] = 0
        stats['window_jumps'] = 0

    def report(self):
        """Log health, publish the JSON quality report, and restart the window."""
        now = time.monotonic()
        lines = ['Sensor health report:']
        statuses = []
        tof_report = {}
        for name, stats in self.tof_stats.items():
            self._trim_times(stats['times'], now)
            stale = self._stale(name, stats['last'], now)
            pct = (
                100.0 * stats['valid'] / stats['total']
                if stats['total'] else 0
            )
            status, findings = self._tof_status(stats, stale)
            statuses.append(status)
            tof_report[name] = {
                'status': status,
                'rate_hz': round(self.rate(stats), 2),
                'messages': stats['count'],
                'valid_percent': round(pct, 1),
                'invalid': stats['invalid'],
                'out_of_range': stats['out_of_range'],
                'jumps': stats['jumps'],
                'stuck': stats['stuck'],
                'findings': findings,
            }
            detail = f' ({", ".join(findings)})' if findings else ''
            lines.append(
                f'  {name}: {self.rate(stats):.1f} Hz, '
                f'{stats["count"]} msgs, {stats["invalid"]} invalid (-1), '
                f'{pct:.1f}% valid [{status}]{detail}'
            )
            self._reset_window(stats)

        self._trim_times(self.imu_times, now)
        stale = self._stale('bno055_imu', self.imu_last, now)
        if stale:
            self.imu_status = 'STALE'
        elif self.imu_valid and self.imu_norm_ok:
            self.imu_status = 'OK'
        else:
            self.imu_status = 'INVALID'
        statuses.append(self.imu_status)
        lines.append(
            f'  BNO055 IMU: {self.rate(self.imu_times):.1f} Hz, '
            f'{self.imu_count} msgs [{self.imu_status}]'
        )

        self._trim_times(self.cal_times, now)
        stale = self._stale('bno055_calibration', self.cal_last, now)
        if stale:
            self.cal_status = 'STALE'
        elif self.calibration is None:
            self.cal_status = 'INVALID'
        elif min(self.calibration) < 3:
            self.cal_status = 'WARN'
        else:
            self.cal_status = 'OK'
        statuses.append(self.cal_status)
        labels = ('SYS', 'GYRO', 'ACC', 'MAG')
        cal = 'unknown' if self.calibration is None else ', '.join(
            f'{key}={value}/3' for key, value in zip(labels, self.calibration))
        lines.append(
            f'  BNO055 calibration: {self.rate(self.cal_times):.1f} Hz, '
            f'{cal} ({self.cal_count} msgs) '
            f'[{self.cal_status}]'
        )
        self.get_logger().info('\n'.join(lines))

        severity = SEVERITY[worst(statuses)]
        payload = {
            'stamp': round(self.get_clock().now().nanoseconds / 1e9, 3),
            'overall': ('OK', 'WARN', 'ERROR')[severity],
            'tof': tof_report,
            'imu': {
                'status': self.imu_status,
                'rate_hz': round(self.rate(self.imu_times), 2),
                'messages': self.imu_count,
                'quaternion_norm': (
                    None if self.imu_norm is None else round(self.imu_norm, 4)),
            },
            'calibration': {
                'status': self.cal_status,
                'levels': self.calibration,
                'rate_hz': round(self.rate(self.cal_times), 2),
                'messages': self.cal_count,
            },
        }
        message = String()
        message.data = json.dumps(payload)
        self.quality_pub.publish(message)


def main(args=None):
    """Run the sensor monitor node."""
    rclpy.init(args=args)
    node = SensorMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
