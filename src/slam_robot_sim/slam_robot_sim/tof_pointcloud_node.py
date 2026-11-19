"""Publish a planar PointCloud2 from the three VL53L5CX sensor grids."""

# الوظيفة: تحويل قراءات الصف الأوسط للحساسات الثلاثة إلى نقاط ثنائية الأبعاد.
# الغرض: تجهيز بيانات مكانية لـRViz والخريطة دون افتراض وجود Odometry.

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField

from sensor_msgs_py import point_cloud2

from slam_robot_sim.mapping_utils import project_grid_points

from std_msgs.msg import Header, Int32MultiArray


class TofPointCloudNode(Node):
    """Convert ToF grids to a configurable planar point cloud."""

    def __init__(self, **kwargs):
        """Declare sensor geometry and set up ROS publishers/subscribers."""
        super().__init__('tof_pointcloud', **kwargs)
        defaults = (
            ('sensor_fov_deg', 45.0),
            ('sensor_yaw_deg', [0.0, 0.0, 0.0]),
            ('sensor_x_m', [0.0, 0.0, 0.0]),
            ('sensor_y_m', [-0.01, 0.0, 0.01]),
            ('grid_row', 3),
            ('reverse_columns', False),
            ('range_min_m', 0.05),
            ('range_max_m', 4.0),
            ('frame_id', 'base_link'),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)
        self.fov = math.radians(
            float(self.get_parameter('sensor_fov_deg').value))
        self.sensor_yaw = [
            math.radians(value)
            for value in self.get_parameter('sensor_yaw_deg').value
        ]
        self.sensor_x = list(self.get_parameter('sensor_x_m').value)
        self.sensor_y = list(self.get_parameter('sensor_y_m').value)
        self.row = int(self.get_parameter('grid_row').value)
        self.reverse_columns = bool(
            self.get_parameter('reverse_columns').value)
        self.range_min = float(self.get_parameter('range_min_m').value)
        self.range_max = float(self.get_parameter('range_max_m').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self._validate_parameters()

        self.grids = [None, None, None]
        self._cloud_pub = self.create_publisher(
            PointCloud2, '/sensors/tof_points', 10)
        self._tof_subs = [
            self.create_subscription(
                Int32MultiArray, f'/sensors/tof/sensor_{index + 1}',
                lambda message, i=index: self._on_grid(i, message), 10)
            for index in range(3)
        ]
        self.get_logger().info(
            'Point cloud ready on /sensors/tof_points; '
            'sensor positions default to 1 cm spacing.')

    def _validate_parameters(self):
        if not 0 < self.fov <= math.pi:
            raise ValueError('sensor_fov_deg must be between 0 and 180')
        lengths = (
            len(self.sensor_yaw), len(self.sensor_x), len(self.sensor_y))
        if lengths != (3, 3, 3):
            raise ValueError(
                'sensor yaw/x/y parameters must each have 3 values')
        if not 0 <= self.row < 8:
            raise ValueError('grid_row must be between 0 and 7')
        if self.range_min <= 0 or self.range_max <= self.range_min:
            raise ValueError(
                'range limits must be positive and ordered')

    def _on_grid(self, index, message):
        """Store a grid and publish when all three sensors are available."""
        if len(message.data) != 64:
            self.get_logger().warning(
                f'Sensor {index + 1} grid has {len(message.data)} values')
            return
        self.grids[index] = list(message.data)
        if any(grid is None for grid in self.grids):
            return
        self.publish_cloud()

    def publish_cloud(self):
        """Project valid ranges and publish them as XYZ points with z=0."""
        points = []
        for index, grid in enumerate(self.grids):
            points.extend(project_grid_points(
                grid,
                float(self.sensor_x[index]),
                float(self.sensor_y[index]),
                self.sensor_yaw[index],
                self.fov,
                self.row,
                self.reverse_columns,
                self.range_min,
                self.range_max,
            ))
        header = self.get_clock().now().to_msg()
        cloud_header = self._make_header(header)
        fields = [
            PointField(
                name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(cloud_header, fields, points)
        self._cloud_pub.publish(cloud)

    def _make_header(self, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id
        return header


def main(args=None):
    """Run the ToF point-cloud publisher."""
    rclpy.init(args=args)
    node = TofPointCloudNode()
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
