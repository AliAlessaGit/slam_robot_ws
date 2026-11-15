"""Receive and publish sensor JSON from an ESP32 USB serial connection."""
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

import serial

from slam_robot_sim.sensor_protocol import parse_sensor_line, SensorDataError

from std_msgs.msg import Int32MultiArray, String, UInt8MultiArray


class SerialReceiver(Node):
    """Read and validate newline-delimited sensor JSON."""

    def __init__(self, **kwargs):
        """Initialize ROS publishers and serial connection parameters."""
        super().__init__('serial_receiver', **kwargs)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('reconnect_interval', 2.0)
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.retry = self.get_parameter('reconnect_interval').value
        self.connection = None
        self.last_attempt = 0.0
        self.buffer = bytearray()
        self.raw_pub = self.create_publisher(String, '/sensors/raw_json', 10)
        self.tof_pubs = {
            key: self.create_publisher(
                Int32MultiArray, f'/sensors/tof/sensor_{index}', 10
            ) for index, key in enumerate(('s1', 's2', 's3'), 1)
        }
        self.imu_pub = self.create_publisher(Imu, '/sensors/bno055/imu', 10)
        self.cal_pub = self.create_publisher(
            UInt8MultiArray, '/sensors/bno055/calibration', 10
        )
        self.status_pub = self.create_publisher(
            String, '/sensors/bno055/status', 10
        )
        self.timer = self.create_timer(0.005, self.poll_serial)
        self.get_logger().info(f'Waiting for {self.port} at {self.baud} baud')

    def connect(self):
        """Open the serial port, limiting retries when unavailable."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_attempt < self.retry:
            return
        self.last_attempt = now
        try:
            self.connection = serial.Serial(self.port, self.baud, timeout=0)
            self.buffer.clear()
            self.get_logger().info(f'Connected to {self.port}')
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f'Cannot open serial port: {exc}')

    def poll_serial(self):
        """Read available bytes, process complete lines, and reconnect."""
        if self.connection is None or not self.connection.is_open:
            self.connection = None
            self.connect()
            return
        try:
            self.buffer.extend(self.connection.read(4096))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f'Serial read failed: {exc}')
            self.close_serial()
            return
        if len(self.buffer) > 1024 * 1024:
            self.get_logger().warning('Discarded oversized serial line')
            self.buffer.clear()
        while b'\n' in self.buffer:
            line, _, rest = self.buffer.partition(b'\n')
            self.buffer = bytearray(rest)
            if line.strip():
                self.process_line(line.rstrip(b'\r'))

    def process_line(self, line):
        """Publish raw JSON and, if valid, decoded sensor measurements."""
        raw = String()
        raw.data = line.decode('utf-8', errors='replace')
        self.raw_pub.publish(raw)
        try:
            parsed = parse_sensor_line(line)
        except SensorDataError as exc:
            self.get_logger().warning(f'Invalid sensor JSON: {exc}')
            return
        if parsed['type'] == 'status':
            status = String()
            status.data = raw.data
            self.status_pub.publish(status)
            return
        data = parsed['data']
        for key, publisher in self.tof_pubs.items():
            msg = Int32MultiArray()
            msg.data = data[key]
            publisher.publish(msg)
        bno = data['bno']
        calibration = UInt8MultiArray()
        calibration.data = bno['cal']
        self.cal_pub.publish(calibration)
        status = String()
        status.data = 'ready' if bno['ok'] else 'unavailable'
        self.status_pub.publish(status)
        if bno['ok']:
            self.publish_imu(bno)

    def publish_imu(self, bno):
        """Publish BNO055 Euler angles as an IMU quaternion."""
        roll, pitch, yaw = map(math.radians, (bno['r'], bno['p'], bno['h']))
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'bno055_link'
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        self.imu_pub.publish(msg)

    def close_serial(self):
        """Close serial gracefully and discard any partial line."""
        if self.connection is not None:
            try:
                self.connection.close()
            except (serial.SerialException, OSError):
                pass
        self.connection = None
        self.buffer.clear()

    def destroy_node(self):
        """Close serial before destroying the ROS node."""
        self.close_serial()
        super().destroy_node()


def main(args=None):
    """Start the serial receiver node."""
    rclpy.init(args=args)
    node = SerialReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
