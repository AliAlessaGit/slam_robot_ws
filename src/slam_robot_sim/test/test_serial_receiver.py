"""End-to-end tests for the serial receiver over a local pseudo-terminal."""
import json
import os
import pty
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from sensor_msgs.msg import Imu

from slam_robot_sim.serial_receiver_node import SerialReceiver
from slam_robot_sim.virtual_serial_sender import sample

from std_msgs.msg import Int32MultiArray, String, UInt8MultiArray


def test_virtual_serial_to_ros_topics():
    """Verify PTY JSON validation, ROS topics, and the 20 Hz data path."""
    master_fd, slave_fd = pty.openpty()
    port = os.ttyname(slave_fd)
    rclpy.init()
    receiver = SerialReceiver(
        parameter_overrides=[Parameter('serial_port', value=port)]
    )
    observer = rclpy.create_node('serial_receiver_test_observer')
    received = {
        'raw': [], 'tof1': [], 'tof2': [], 'tof3': [], 'imu': [], 'cal': []
    }
    subscriptions = [
        observer.create_subscription(
            String, '/sensors/raw_json',
            lambda msg: received['raw'].append(msg), 10
        ),
        observer.create_subscription(
            Int32MultiArray, '/sensors/tof/sensor_1',
            lambda msg: received['tof1'].append(msg), 10
        ),
        observer.create_subscription(
            Int32MultiArray, '/sensors/tof/sensor_2',
            lambda msg: received['tof2'].append(msg), 10
        ),
        observer.create_subscription(
            Int32MultiArray, '/sensors/tof/sensor_3',
            lambda msg: received['tof3'].append(msg), 10
        ),
        observer.create_subscription(
            Imu, '/sensors/bno055/imu',
            lambda msg: received['imu'].append(msg), 10
        ),
        observer.create_subscription(
            UInt8MultiArray, '/sensors/bno055/calibration',
            lambda msg: received['cal'].append(msg), 10
        ),
    ]
    executor = SingleThreadedExecutor()
    executor.add_node(receiver)
    executor.add_node(observer)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        deadline = time.monotonic() + 5.0
        while receiver.connection is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert receiver.connection is not None, 'receiver did not open PTY'
        frame_count = 40
        start = time.monotonic()
        for frame in range(frame_count):
            os.write(master_fd, (json.dumps(sample(frame)) + '\n').encode())
            time.sleep(0.05)
        deadline = time.monotonic() + 3.0
        while (
            len(received['raw']) < frame_count
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        elapsed = time.monotonic() - start

        assert len(received['raw']) >= 38
        for key in ('tof1', 'tof2', 'tof3', 'imu', 'cal'):
            assert len(received[key]) >= 38, f'missing messages on {key}'
        assert len(received['raw']) / elapsed >= 18.0
        assert list(received['tof1'][0].data) == [1000] * 64
        assert list(received['cal'][0].data) == [3, 3, 3, 3]
        assert received['imu'][0].header.frame_id == 'bno055_link'

        before = len(received['raw'])
        os.write(master_fd, b'{invalid json}\n')
        deadline = time.monotonic() + 2.0
        while len(received['raw']) == before and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(received['raw']) == before + 1
        assert len(received['tof1']) >= 38
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        executor.remove_node(receiver)
        executor.remove_node(observer)
        receiver.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
        os.close(master_fd)
        os.close(slave_fd)
        _ = subscriptions
