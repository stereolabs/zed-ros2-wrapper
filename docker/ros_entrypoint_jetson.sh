#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/install/setup.bash" --
source "/root/ros2_ws/install/local_setup.bash" --

# Welcome information
echo "ZED ROS2 Docker Image"
echo "---------------------"
echo 'ROS distro: ' $ROS_DISTRO
echo 'DDS middleware: ' $RMW_IMPLEMENTATION 
echo "---"  
echo 'Available ZED packages:'
ros2 pkg list | grep zed
echo "---------------------"    
exec "$@"
