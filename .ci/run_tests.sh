#!/bin/bash
set -e

ttk='--->'
WORKDIR=$(pwd)

echo "${ttk} Check artifact presence"
ls ros2_ws/install

echo "${ttk} Initialize local ROS2 environment"
cd ros2_ws/install
export COLCON_PREFIX_PATH=$(pwd)
source ros2_ws/install/local_setup.sh
cd ${WORKDIR}

echo "${ttk} Check ROS2 installation"
ros2 doctor -r

echo "${ttk} Check ZED ROS2 packages presence"
ros2 pkg list | grep zed

echo "${ttk} USB peripherals"
lsusb

echo "${ttk} Test node running for 20 seconds"
timeout --signal=SIGINT 20s ros2 launch zed_wrapper zed2i.launch.py

