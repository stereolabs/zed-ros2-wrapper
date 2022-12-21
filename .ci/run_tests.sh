#!/bin/bash
set -e

ttk='--->'

echo "${ttk} Check artifact presence"
ls ros2_ws/install

echo "${ttk} Initialize local ROS2 environment"
source ros2_ws/install/local_setup.sh

echo "${ttk} Check ROS2 installation"
ros2 doctor -r

echo "${ttk} USB peripherals"
lsusb

