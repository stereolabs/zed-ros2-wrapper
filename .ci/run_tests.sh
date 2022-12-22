#!/bin/bash
set -e

ttk='--->'
WORKDIR=$(pwd)


echo "${ttk} WORKDIR (${WORKDIR})content"
ls ${WORKDIR}

echo "${ttk} Check artifact presence"
ls ../ros2_ws/install

echo "${ttk} Initialize local ROS2 environment"
cd ${WORKDIR}
source ../ros2_ws/install/local_setup.bash
env | grep COLCON

echo "${ttk} Check ROS2 installation"
ros2 doctor -r

echo "${ttk} Check ZED ROS2 packages presence"
ros2 pkg list | grep zed

echo "${ttk} USB peripherals"
lsusb

echo "${ttk} Test node running for 20 seconds"
timeout --signal=SIGINT 20s ros2 launch zed_wrapper zed2i.launch.py

