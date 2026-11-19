"""Publish TF and Odometry based on 2D Pose Estimate or Interactive Markers."""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration

class RvizPoseOdomNode(Node):
    def __init__(self):
        super().__init__('rviz_pose_odom')
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_pose, 10
        )
        self.timer = self.create_timer(0.05, self._broadcast_tf)

    def _on_pose(self, msg):
        pose = msg.pose.pose
        self.pose['x'] = pose.position.x
        self.pose['y'] = pose.position.y
        # Simple yaw extraction
        siny_cosp = 2 * (pose.orientation.w * pose.orientation.z + 
                         pose.orientation.x * pose.orientation.y)
        cosy_cosp = 1 - 2 * (pose.orientation.y * pose.orientation.y + 
                             pose.orientation.z * pose.orientation.z)
        self.pose['yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def _broadcast_tf(self):
        now = self.get_clock().now().to_msg()
        # map -> odom
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        self.tf_broadcaster.sendTransform(t)
        
        # odom -> base_link
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pose['x']
        t.transform.translation.y = self.pose['y']
        t.transform.rotation.z = math.sin(self.pose['yaw'] / 2)
        t.transform.rotation.w = math.cos(self.pose['yaw'] / 2)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RvizPoseOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()