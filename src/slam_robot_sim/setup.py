"""Setup configuration for the SLAM robot simulation package."""

# الإعداد: تعريف حزمة Python وأوامر تشغيل عقد ROS.
# الغرض: تثبيت ملفات الواجهة وتسجيل أدوات الحزمة التنفيذية.

from setuptools import find_packages, setup

package_name = 'slam_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['web/*.html']},
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/mapping.launch.py', 'launch/mapping.rviz'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali_alessa',
    maintainer_email='ali_alessa@todo.todo',
    description='SLAM robot sensor simulation package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simulator = slam_robot_sim.simulator_node:main',
            'serial_receiver = slam_robot_sim.serial_receiver_node:main',
            'sensor_web = slam_robot_sim.sensor_web_node:main',
            'tof_pointcloud = slam_robot_sim.tof_pointcloud_node:main',
            'sensor_monitor = slam_robot_sim.sensor_monitor_node:main',
            'virtual_serial_sender = slam_robot_sim.virtual_serial_sender:main',
            'rviz_pose_odom = slam_robot_sim.rviz_pose_odom_node:main',
            'tof_mapper = slam_robot_sim.tof_mapper_node:main',
        ],
    },
)
