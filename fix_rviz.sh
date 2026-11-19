#!/bin/bash
# Script to source ROS 2 and run Rviz with correct LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source /home/ali_alessa/slam_robot_ws/install/setup.bash
# We need to remove the problematic library if it exists in the path or override it
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
rviz2 -d /home/ali_alessa/slam_robot_ws/install/slam_robot_sim/share/slam_robot_sim/launch/mapping.rviz
