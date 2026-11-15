
"""Setup configuration for the SLAM robot simulation package."""

from setuptools import find_packages, setup

package_name = 'slam_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
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
            'virtual_serial_sender = slam_robot_sim.virtual_serial_sender:main',
        ],
    },
)
