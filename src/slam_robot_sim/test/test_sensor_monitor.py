"""Integration tests for the ROS sensor monitor subscriptions and report."""

# الاختبار: التحقق من متابعة حالة وتواتر قراءات الحساسات.
# الغرض: التأكد من اكتشاف الانقطاع والقراءات غير الصالحة في ROS.
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from sensor_msgs.msg import Imu

from slam_robot_sim.sensor_monitor_node import SensorMonitor

from std_msgs.msg import Int32MultiArray, UInt8MultiArray


def test_monitor_counts_rates_validity_and_staleness():
    """Publish ROS samples and verify rolling rates, invalids, and dropout."""
    rclpy.init()
    monitor = SensorMonitor(parameter_overrides=[
        Parameter('report_period_sec', value=10.0),
        Parameter('stale_timeout_sec', value=0.25),
        Parameter('rate_window_sec', value=2.0),
    ])
    sender = rclpy.create_node('sensor_monitor_test_sender')
    pubs = {
        name: sender.create_publisher(
            Int32MultiArray, f'/sensors/tof/{name}', 10
        ) for name in ('sensor_1', 'sensor_2', 'sensor_3')
    }
    imu_pub = sender.create_publisher(Imu, '/sensors/bno055/imu', 10)
    cal_pub = sender.create_publisher(
        UInt8MultiArray, '/sensors/bno055/calibration', 10
    )
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(sender)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if all(pub.get_subscription_count() for pub in pubs.values()):
                break
            time.sleep(0.01)
        assert all(pub.get_subscription_count() for pub in pubs.values())

        for _ in range(10):
            for name, publisher in pubs.items():
                message = Int32MultiArray()
                last = -1 if name == 'sensor_1' else 1000
                message.data = [1000] * 63 + [last]
                publisher.publish(message)
            imu = Imu()
            imu.orientation.w = 1.0
            imu_pub.publish(imu)
            calibration = UInt8MultiArray()
            calibration.data = [3, 3, 3, 3]
            cal_pub.publish(calibration)
            time.sleep(0.05)

        deadline = time.monotonic() + 2.0
        while (
            monitor.tof_stats['sensor_1']['count'] < 10
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        stats = monitor.tof_stats['sensor_1']
        assert stats['count'] == 10
        assert stats['invalid'] == 10
        assert stats['valid'] == 630
        assert stats['total'] == 640
        assert 15.0 <= monitor.rate(stats) <= 25.0
        assert monitor.imu_count >= 10
        assert monitor.imu_valid
        assert monitor.cal_count >= 10
        assert monitor.calibration == [3, 3, 3, 3]

        time.sleep(0.3)
        monitor.report()
        assert 'sensor_3' in monitor.stale_sensors
        assert 'bno055_imu' in monitor.stale_sensors
        assert 'bno055_calibration' in monitor.stale_sensors

        message = Int32MultiArray()
        message.data = [500] * 64
        pubs['sensor_3'].publish(message)
        deadline = time.monotonic() + 2.0
        while (
            monitor.tof_stats['sensor_3']['count'] < 11
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        monitor.report()
        assert 'sensor_3' not in monitor.stale_sensors
        assert monitor.tof_stats['sensor_3']['count'] == 11
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        executor.remove_node(monitor)
        executor.remove_node(sender)
        monitor.destroy_node()
        sender.destroy_node()
        rclpy.shutdown()
