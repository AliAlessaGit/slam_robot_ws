"""Publish TF and Odometry for manual robot pose control in RViz.



Supports:
- 2D Pose Estimate tool in RViz (/initialpose topic)
- Keyboard teleop (WASD + QE for rotation)
- Service to reset pose
- Continuous odometry publishing
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Empty
from std_msgs.msg import String


class RvizPoseOdomNode(Node):
    def __init__(self):
        super().__init__('rviz_pose_odom')
        
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.last_update_time = self.get_clock().now()
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.max_linear_vel = 0.3
        self.max_angular_vel = 1.0
        self.vel_decay = 0.95
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_initial_pose, 10
        )
        self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10
        )
        self.create_subscription(
            String, '/keyboard_cmd', self._on_keyboard_cmd, 10
        )
        
        self.create_service(Empty, 'reset_pose', self._on_reset_pose)
        
        self.timer = self.create_timer(0.05, self._timer_callback)
        
        self.get_logger().info(
            'RViz Pose/Odom node started.\n'
            '  - Use "2D Pose Estimate" in RViz to set pose\n'
            '  - Publish to /cmd_vel (Twist) for keyboard/joystick control\n'
            '  - Publish to /keyboard_cmd (String) with "w/a/s/d/q/e/space" for simple control\n'
            '  - Call /reset_pose service to reset to origin\n'
            '  - Publishes /odom and TF: map -> odom -> base_link'
        )

    def _on_initial_pose(self, msg):
        pose = msg.pose.pose
        self.pose['x'] = pose.position.x
        self.pose['y'] = pose.position.y
        siny_cosp = 2 * (pose.orientation.w * pose.orientation.z + 
                         pose.orientation.x * pose.orientation.y)
        cosy_cosp = 1 - 2 * (pose.orientation.y * pose.orientation.y + 
                              pose.orientation.z * pose.orientation.z)
        self.pose['yaw'] = math.atan2(siny_cosp, cosy_cosp)
        self.get_logger().info(
            f'Pose set via RViz: x={self.pose["x"]:.2f}, '
            f'y={self.pose["y"]:.2f}, yaw={math.degrees(self.pose["yaw"]):.1f}°'
        )

    def _on_cmd_vel(self, msg):
        self.linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        self.angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))

    def _on_keyboard_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'w':
            self.linear_vel = self.max_linear_vel
            self.angular_vel = 0.0
        elif cmd == 's':
            self.linear_vel = -self.max_linear_vel
            self.angular_vel = 0.0
        elif cmd == 'a':
            self.linear_vel = 0.0
            self.angular_vel = self.max_angular_vel
        elif cmd == 'd':
            self.linear_vel = 0.0
            self.angular_vel = -self.max_angular_vel
        elif cmd == 'q':
            self.linear_vel = self.max_linear_vel * 0.7
            self.angular_vel = self.max_angular_vel * 0.7
        elif cmd == 'e':
            self.linear_vel = self.max_linear_vel * 0.7
            self.angular_vel = -self.max_angular_vel * 0.7
        elif cmd in (' ', 'stop'):
            self.linear_vel = 0.0
            self.angular_vel = 0.0
        elif cmd == 'reset':
            self._reset_pose()

    def _on_reset_pose(self, request, response):
        self._reset_pose()
        self.get_logger().info('Pose reset to origin via service')
        return response

    def _reset_pose(self):
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def _timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        
        if abs(self.linear_vel) > 1e-3 or abs(self.angular_vel) > 1e-3:
            self.pose['x'] += self.linear_vel * math.cos(self.pose['yaw']) * dt
            self.pose['y'] += self.linear_vel * math.sin(self.pose['yaw']) * dt
            self.pose['yaw'] += self.angular_vel * dt
            self.pose['yaw'] = math.atan2(math.sin(self.pose['yaw']), math.cos(self.pose['yaw']))
        
        self.linear_vel *= self.vel_decay
        self.angular_vel *= self.vel_decay
        
        self._broadcast_tf(now.to_msg())
        self._publish_odometry(now.to_msg())

    def _broadcast_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pose['x']
        t.transform.translation.y = self.pose['y']
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.pose['yaw'] / 2)
        t.transform.rotation.w = math.cos(self.pose['yaw'] / 2)
        self.tf_broadcaster.sendTransform(t)

    def _publish_odometry(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.pose['x']
        odom.pose.pose.position.y = self.pose['y']
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.pose['yaw'] / 2)
        odom.pose.pose.orientation.w = math.cos(self.pose['yaw'] / 2)
        
        odom.pose.covariance = [999.0] * 36
        
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.angular.z = self.angular_vel
        odom.twist.covariance = [999.0] * 36
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = RvizPoseOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
