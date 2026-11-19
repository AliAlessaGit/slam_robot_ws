#!/bin/bash
source /opt/ros/jazzy/setup.bash
# Filter out any paths containing "/snap/"
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v "/snap/" | tr '\n' ':' | sed 's/:$//')
source /home/ali_alessa/slam_robot_ws/install/setup.bash
rviz2 -d /home/ali_alessa/slam_robot_ws/install/slam_robot_sim/share/slam_robot_sim/launch/mapping.rviz
