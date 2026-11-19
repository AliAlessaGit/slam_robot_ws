"""Simulate the robot's ToF and IMU sensor data."""

# الوظيفة: إنشاء قراءات افتراضية لحساسات المسافة ووحدة IMU.
# الغرض: اختبار تدفق بيانات ROS دون الحاجة إلى توصيل الحساسات.

import json
import math
import array

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32MultiArray, MultiArrayDimension


class SensorSimulator(Node):
    """Generate and publish simulated robot sensor readings."""

    NUM_SENSORS = 3
    GRID_SIDE = 8

    def __init__(self):
        """Initialize the simulator node and its publishers."""
        super().__init__('sensor_simulator')

        self.raw_publisher = self.create_publisher(
            String,
            '/sensors/raw_json',
            10
        )

        # Publish individual sensor grids for downstream nodes
        self.tof_publishers = [
            self.create_publisher(
                Int32MultiArray,
                f'/sensors/tof/sensor_{i + 1}',
                10
            )
            for i in range(self.NUM_SENSORS)
        ]

        self.frame = 0

        # Original Arduino sends JSON every 50 ms.
        self.timer = self.create_timer(
            0.05,
            self.publish_sensor_data
        )

        self.get_logger().info(
            'Sensor simulator started - publishing at 20 Hz'
        )

    def generate_grid(self, sensor_id):
        """Generate one simulated 8x8 distance grid (fixed, no noise)."""
        grid = []

        for y in range(8):
            for x in range(7, -1, -1):
                # جعل المسافة ثابتة بدون ضجيج
                distance = 1500

                # عقبة محاكاة ثابتة
                if 2 <= x <= 4 and 3 <= y <= 5:
                    distance -= 700

                grid.append(distance)

        return grid

    def generate_data(self):
        """Generate a complete simulated sensor data message."""
        heading = (self.frame * 2.0) % 360
        roll = 5.0 * math.sin(self.frame * 0.05)
        pitch = 3.0 * math.cos(self.frame * 0.04)

        data = {
            's1': self.generate_grid(1),
            's2': self.generate_grid(2),
            's3': self.generate_grid(3),
            'bno': {
                'ok': 1,
                'h': round(heading, 1),
                'r': round(roll, 1),
                'p': round(pitch, 1),
                'cal': [3, 3, 3, 3],
            },
        }

        return data

    def publish_sensor_data(self):
        """Publish the simulated sensor data as JSON and individual topics."""
        data = self.generate_data()

        # Publish raw JSON
        msg = String()
        msg.data = json.dumps(data)
        self.raw_publisher.publish(msg)

        # Publish individual sensor grids
        for i, key in enumerate(['s1', 's2', 's3']):
            grid = Int32MultiArray()
            grid.layout.dim = [
                MultiArrayDimension(label='row', size=self.GRID_SIDE,
                                    stride=self.GRID_SIDE * self.GRID_SIDE),
                MultiArrayDimension(label='col', size=self.GRID_SIDE,
                                    stride=self.GRID_SIDE),
            ]
            grid.data = data[key]
            self.tof_publishers[i].publish(grid)

        self.frame += 1


def main(args=None):
    """Start the sensor simulator node."""
    rclpy.init(args=args)

    node = SensorSimulator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
