"""Subscribe to PointCloud2 and update an OccupancyGrid using robot pose from TF."""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException
from slam_robot_sim.occupancy_map import OccupancyMap
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped


class TofMapperNode(Node):
    def __init__(self):
        super().__init__('tof_mapper')
        
        # Map parameters
        self.declare_parameter('map_width', 300)
        self.declare_parameter('map_height', 300)
        self.declare_parameter('map_resolution', 0.04)
        self.declare_parameter('sensor_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('max_tf_age_sec', 0.1)
        
        width = self.get_parameter('map_width').value
        height = self.get_parameter('map_height').value
        resolution = self.get_parameter('map_resolution').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.max_tf_age = Duration(seconds=self.get_parameter('max_tf_age_sec').value)
        
        self.map = OccupancyMap(width=width, height=height, resolution_m=resolution)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.create_subscription(PointCloud2, '/sensors/tof_points', self._on_cloud, 10)
        self.timer = self.create_timer(1.0, self._publish_map)
        
        self.get_logger().info(
            f'ToF Mapper started: map_frame={self.map_frame}, '
            f'sensor_frame={self.sensor_frame}, map_size={width}x{height}@{resolution}m'
        )

    def _transform_points_to_map(self, points, transform):
        """Transform points from sensor frame to map frame using TF."""
        if not points:
            return []
        
        # Extract translation and rotation from transform
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        
        # Convert quaternion to yaw
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        transformed = []
        for px, py, pz in points:
            # Rotate and translate (2D, ignore z)
            mx = tx + px * cos_yaw - py * sin_yaw
            my = ty + px * sin_yaw + py * cos_yaw
            transformed.append((mx, my))
        
        return transformed

    def _on_cloud(self, msg):
        try:
            # Look up transform from sensor frame to map frame
            # Use rclpy.time.Time() to get the latest available transform
            # This avoids extrapolation errors when message time is slightly ahead of TF
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.sensor_frame,
                Time(),
                timeout=self.max_tf_age
            )
            
            # Read points from PointCloud2 (x, y only, z=0)
            points = list(point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            ))
            
            # Transform points to map frame
            map_points = self._transform_points_to_map(points, transform)
            
            # Get robot position in map frame (translation part of transform)
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Update occupancy map
            self.map.update_scan(robot_x, robot_y, map_points)
            
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f'Unexpected error in mapper: {e}')

    def _publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.map.resolution
        grid.info.width = self.map.width
        grid.info.height = self.map.height
        grid.info.origin.position.x = self.map.origin_x
        grid.info.origin.position.y = self.map.origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = self.map.to_occupancy_data()
        self.map_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = TofMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
