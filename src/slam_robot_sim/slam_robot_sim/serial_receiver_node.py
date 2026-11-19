"""Read the ESP32 USB serial stream and publish it as ROS 2 topics."""

# الوظيفة: قراءة JSON الحساسات من المنفذ التسلسلي ونشرها على ROS.
# الغرض: توفير بيانات ToF وBNO055 للعقد والواجهات اللاحقة.

import array
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
from slam_robot_sim import esp32_link
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, String, UInt8MultiArray

NUM_SENSORS = 3
GRID_SIDE = 8


def open_serial_port(port, baud_rate):
    return serial.Serial(port, baud_rate, timeout=0)


class SerialReceiverNode(Node):
    """Publish raw JSON, ToF grids, IMU and calibration read from a serial port."""

    def __init__(self, *, serial_factory=None, context=None, parameter_overrides=None):
        super().__init__(
            'serial_receiver', context=context, parameter_overrides=parameter_overrides)

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('poll_period', 0.005)
        self.declare_parameter('imu_frame_id', 'bno055_link')
        # BNO055 heading grows clockwise (compass style) while ROS yaw grows
        # counter-clockwise. Set to true to negate the heading.
        self.declare_parameter('heading_clockwise', False)

        self._port = self.get_parameter('serial_port').value
        self._baud_rate = self.get_parameter('baud_rate').value
        self._reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self._imu_frame_id = self.get_parameter('imu_frame_id').value
        self._heading_clockwise = bool(self.get_parameter('heading_clockwise').value)
        poll_period = float(self.get_parameter('poll_period').value)

        self._serial_factory = serial_factory or open_serial_port
        self._serial = None
        self._last_attempt = float('-inf')
        self._lines = esp32_link.LineBuffer()

        self.frames_ok = 0
        self.frames_invalid = 0

        self._raw_pub = self.create_publisher(String, '/sensors/raw_json', 10)
        self._status_pub = self.create_publisher(String, '/sensors/status', 10)
        self._tof_pubs = [
            self.create_publisher(Int32MultiArray, f'/sensors/tof/sensor_{i + 1}', 10)
            for i in range(NUM_SENSORS)
        ]
        self._imu_pub = self.create_publisher(Imu, '/sensors/bno055/imu', 10)
        self._cal_pub = self.create_publisher(
            UInt8MultiArray, '/sensors/bno055/calibration', 10)

        self._timer = self.create_timer(poll_period, self.poll_once)
        self.get_logger().info(
            f'Waiting for serial port {self._port} at {self._baud_rate} baud')

    @property
    def connected(self):
        return self._serial is not None

    def poll_once(self):
        if self._serial is None:
            now = time.monotonic()
            if now - self._last_attempt < self._reconnect_interval:
                return
            self._last_attempt = now
            if not self._open_serial():
                return

        try:
            waiting = self._serial.in_waiting
            data = self._serial.read(waiting) if waiting else b''
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Serial connection lost: {exc}')
            self._close_serial()
            return

        if data:
            self.process_bytes(data)

    def process_bytes(self, data):
        overflows = self._lines.overflows
        for line in self._lines.feed(data):
            self.process_line(line)
        if self._lines.overflows != overflows:
            self.get_logger().warning('Serial line exceeded the size limit and was dropped')

    def process_line(self, line):
        raw = String()
        raw.data = line.decode('utf-8', errors='replace')
        self._raw_pub.publish(raw)

        try:
            message = esp32_link.parse_line(line)
        except esp32_link.ProtocolError as exc:
            self.frames_invalid += 1
            self.get_logger().warning(f'Ignoring invalid line: {exc}', throttle_duration_sec=5.0)
            return

        if message.kind == 'frame':
            self.frames_ok += 1
            self._publish_frame(message)
            return

        status = String()
        status.data = message.text
        self._status_pub.publish(status)
        if message.kind == 'error':
            self.get_logger().error(f'ESP32 error: {message.text}')
        else:
            self.get_logger().info(f'ESP32 status: {message.text}')

    def _publish_frame(self, message):
        for publisher, values in zip(self._tof_pubs, message.tof):
            grid = Int32MultiArray()
            grid.layout.dim = [
                MultiArrayDimension(label='row', size=GRID_SIDE, stride=GRID_SIDE * GRID_SIDE),
                MultiArrayDimension(label='col', size=GRID_SIDE, stride=GRID_SIDE),
            ]
            grid.data = list(values)
            publisher.publish(grid)

        bno = message.bno
        calibration = UInt8MultiArray()
        calibration.data = array.array('B', bno.cal)
        self._cal_pub.publish(calibration)

        if not bno.ok:
            self.get_logger().warning(
                'BNO055 reports not ok, IMU message skipped', throttle_duration_sec=10.0)
            return

        yaw = -bno.heading if self._heading_clockwise else bno.heading
        x, y, z, w = esp32_link.euler_to_quaternion(bno.roll, bno.pitch, yaw)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self._imu_frame_id
        imu.orientation.x = x
        imu.orientation.y = y
        imu.orientation.z = z
        imu.orientation.w = w
        # Orientation covariance stays all zeros, which means "unknown".
        # The sensor stream has no angular velocity or acceleration: mark them unavailable.
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self._imu_pub.publish(imu)

    def _open_serial(self):
        try:
            self._serial = self._serial_factory(self._port, self._baud_rate)
        except (serial.SerialException, OSError) as exc:
            self._serial = None
            self.get_logger().warning(
                f'Cannot open {self._port}: {exc}', throttle_duration_sec=10.0)
            return False
        # Start from a clean buffer so a half line from the old connection is not reused.
        self._lines = esp32_link.LineBuffer()
        self.get_logger().info(f'Connected to {self._port}')
        return True

    def _close_serial(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
            self._serial = None

    def destroy_node(self):
        self._close_serial()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialReceiverNode()
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
