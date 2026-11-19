# الاختبار: فحص قراءة المنفذ التسلسلي ونشر بيانات الحساسات على ROS.
# الغرض: التحقق من معالجة الإطارات والحالات وإعادة الاتصال بالمنفذ.

import json
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
from slam_robot_sim.serial_receiver_node import SerialReceiverNode
from std_msgs.msg import Int32MultiArray, String, UInt8MultiArray


class FakeSerial:

    def __init__(self, payload=b'', fail_on_read=False):
        self._data = bytearray(payload)
        self._fail_on_read = fail_on_read
        self.closed = False

    @property
    def in_waiting(self):
        if self._fail_on_read:
            raise OSError('device disconnected')
        return len(self._data)

    def read(self, size=1):
        chunk = bytes(self._data[:size])
        del self._data[:size]
        return chunk

    def close(self):
        self.closed = True


def frame_line(heading=90.0):
    return json.dumps({
        's1': [1000] * 64,
        's2': [1500] * 63 + [-1],
        's3': [2000] * 64,
        'bno': {'ok': 1, 'h': heading, 'r': 0.0, 'p': 0.0, 'cal': [3, 2, 1, 0]},
    }) + '\n'


class Harness:

    def __init__(self, serial_factory, parameter_overrides=None):
        self.context = Context()
        rclpy.init(context=self.context)
        # A very long poll period keeps the node's own timer out of the way,
        # so each test decides exactly when poll_once() runs.
        overrides = [Parameter('poll_period', Parameter.Type.DOUBLE, 3600.0)]
        overrides += list(parameter_overrides or [])
        self.receiver = SerialReceiverNode(
            serial_factory=serial_factory,
            context=self.context,
            parameter_overrides=overrides,
        )
        self.listener = Node('serial_receiver_test_listener', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.receiver)
        self.executor.add_node(self.listener)
        self.received = {}
        for topic, msg_type in [
            ('/sensors/raw_json', String),
            ('/sensors/status', String),
            ('/sensors/tof/sensor_1', Int32MultiArray),
            ('/sensors/tof/sensor_2', Int32MultiArray),
            ('/sensors/bno055/imu', Imu),
            ('/sensors/bno055/calibration', UInt8MultiArray),
        ]:
            self.listener.create_subscription(
                msg_type, topic, self._make_callback(topic), 10)
        self._wait_for_subscribers()

    def _make_callback(self, topic):
        def callback(msg):
            self.received.setdefault(topic, []).append(msg)
        return callback

    def _wait_for_subscribers(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            if all(self.receiver.count_subscribers(topic) > 0 for topic in [
                    '/sensors/raw_json', '/sensors/status', '/sensors/tof/sensor_1',
                    '/sensors/bno055/imu', '/sensors/bno055/calibration']):
                return
        pytest.fail('subscribers were not discovered in time')

    def spin(self, seconds=0.3):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def close(self):
        self.executor.shutdown()
        self.receiver.destroy_node()
        self.listener.destroy_node()
        self.context.try_shutdown()


@pytest.fixture
def make_harness():
    created = []

    def factory(serial_factory, parameter_overrides=None):
        harness = Harness(serial_factory, parameter_overrides)
        created.append(harness)
        return harness

    yield factory
    for harness in created:
        harness.close()


def test_valid_frame_is_published(make_harness):
    fake = FakeSerial(frame_line().encode())
    harness = make_harness(lambda port, baud: fake)

    harness.receiver.poll_once()
    harness.spin()

    assert harness.receiver.connected
    assert harness.receiver.frames_ok == 1
    assert len(harness.received['/sensors/raw_json']) == 1
    assert list(harness.received['/sensors/tof/sensor_1'][0].data) == [1000] * 64
    assert harness.received['/sensors/tof/sensor_2'][0].data[63] == -1
    assert list(harness.received['/sensors/bno055/calibration'][0].data) == [3, 2, 1, 0]

    imu = harness.received['/sensors/bno055/imu'][0]
    assert imu.header.frame_id == 'bno055_link'
    assert imu.orientation.z == pytest.approx(0.7071, abs=1e-3)
    assert imu.orientation.w == pytest.approx(0.7071, abs=1e-3)
    assert imu.angular_velocity_covariance[0] == -1.0
    assert imu.linear_acceleration_covariance[0] == -1.0


def test_heading_clockwise_flips_yaw(make_harness):
    fake = FakeSerial(frame_line().encode())
    overrides = [Parameter('heading_clockwise', Parameter.Type.BOOL, True)]
    harness = make_harness(lambda port, baud: fake, overrides)

    harness.receiver.poll_once()
    harness.spin()

    imu = harness.received['/sensors/bno055/imu'][0]
    assert imu.orientation.z == pytest.approx(-0.7071, abs=1e-3)


def test_invalid_line_only_publishes_raw(make_harness):
    fake = FakeSerial(b'this is not json\n')
    harness = make_harness(lambda port, baud: fake)

    harness.receiver.poll_once()
    harness.spin()

    assert harness.receiver.frames_invalid == 1
    assert len(harness.received['/sensors/raw_json']) == 1
    assert '/sensors/tof/sensor_1' not in harness.received
    assert '/sensors/bno055/imu' not in harness.received


def test_status_line_goes_to_status_topic(make_harness):
    fake = FakeSerial(b'{"status":"ALL_READY"}\n')
    harness = make_harness(lambda port, baud: fake)

    harness.receiver.poll_once()
    harness.spin()

    assert [msg.data for msg in harness.received['/sensors/status']] == ['ALL_READY']
    assert harness.receiver.frames_invalid == 0


def test_reconnects_after_open_failure(make_harness):
    attempts = []
    fake = FakeSerial(frame_line().encode())

    def factory(port, baud):
        attempts.append(port)
        if len(attempts) == 1:
            raise OSError('no such device')
        return fake

    overrides = [Parameter('reconnect_interval', Parameter.Type.DOUBLE, 0.0)]
    harness = make_harness(factory, overrides)

    harness.receiver.poll_once()
    assert not harness.receiver.connected

    harness.receiver.poll_once()
    harness.spin()
    assert harness.receiver.connected
    assert harness.receiver.frames_ok == 1


def test_serial_loss_closes_port(make_harness):
    fake = FakeSerial(fail_on_read=True)
    harness = make_harness(lambda port, baud: fake)

    harness.receiver.poll_once()

    assert not harness.receiver.connected
    assert fake.closed
