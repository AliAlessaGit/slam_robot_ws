from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    serial_port = LaunchConfiguration('serial_port')
    enable_monitor = LaunchConfiguration('enable_monitor')
    enable_web = LaunchConfiguration('enable_web')
    
    rviz_config = os.path.join(
        get_package_share_directory('slam_robot_sim'), 'launch', 'mapping.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true',
                              description='Use simulator instead of real hardware'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Serial port for ESP32 receiver'),
        DeclareLaunchArgument('enable_monitor', default_value='false',
                              description='Enable sensor monitor node'),
        DeclareLaunchArgument('enable_web', default_value='true',
                              description='Enable web dashboard node'),
        
        # Sensor data source (simulator OR serial receiver)
        Node(package='slam_robot_sim', executable='simulator', name='simulator',
             condition=IfCondition(use_sim)),
        Node(package='slam_robot_sim', executable='serial_receiver', name='serial_receiver',
             parameters=[{'serial_port': serial_port}],
             condition=UnlessCondition(use_sim)),
             
        # Processing pipeline
        Node(package='slam_robot_sim', executable='tof_pointcloud', name='tof_pointcloud'),
        Node(package='slam_robot_sim', executable='rviz_pose_odom', name='rviz_pose_odom'),
        Node(package='slam_robot_sim', executable='tof_mapper', name='tof_mapper',
             parameters=[{'map_width': 300, 'map_height': 300, 'map_resolution': 0.04}]),
        
        # Optional nodes
        Node(package='slam_robot_sim', executable='sensor_monitor', name='sensor_monitor',
             condition=IfCondition(enable_monitor)),
        Node(package='slam_robot_sim', executable='sensor_web', name='sensor_web',
             condition=IfCondition(enable_web)),
        
        # RViz
        Node(package='rviz2', executable='rviz2', name='rviz2',
             additional_env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/libpthread.so.0'},
             arguments=['-d', rviz_config])
    ])