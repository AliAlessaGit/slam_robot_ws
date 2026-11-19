#!/bin/bash
# Force the system pthread library instead of snap
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libpthread.so.0
source /opt/ros/jazzy/setup.bash
source /home/ali_alessa/slam_robot_ws/install/setup.bash
rviz2 -d /home/ali_alessa/slam_robot_ws/install/slam_robot_sim/share/slam_robot_sim/launch/mapping.rviz
