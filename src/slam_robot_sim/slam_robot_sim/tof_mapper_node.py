"""Subscribe to PointCloud2 and update an OccupancyGrid."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from slam_robot_sim.occupancy_map import OccupancyMap
from sensor_msgs_py import point_cloud2

class TofMapperNode(Node):
    def __init__(self):
        super().__init__('tof_mapper')
        self.map = OccupancyMap(width=200, height=200)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.create_subscription(PointCloud2, '/sensors/tof_points', self._on_cloud, 10)
        self.timer = self.create_timer(1.0, self._publish_map)

    def _on_cloud(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            # For brevity in this prototype, we transform points using the TF
            # In a real impl, iterate points and transform them
            points = list(point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True))
            # Just add them to the map assuming base_link == map for testing
            self.map.update_scan(0, 0, points)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')

    def _publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.map.resolution
        grid.info.width = self.map.width
        grid.info.height = self.map.height
        grid.data = self.map.to_occupancy_data()
        self.map_pub.publish(grid)

def main(args=None):
    rclpy.init(args=args)
    node = TofMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
