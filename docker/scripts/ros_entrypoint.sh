#!/bin/bash
set -e

# Setup the ROS 2 environment. /opt/ros_env is written at build time by
# install_ros2.sh and sources the right setup.bash for either the APT or the
# from-source installation.
if [ -f /opt/ros_env ]; then
  # shellcheck disable=SC1091
  source /opt/ros_env
else
  # shellcheck disable=SC1091
  source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Setup the ZED ROS 2 Wrapper workspace overlay.
# shellcheck disable=SC1091
source "/root/ros2_ws/install/local_setup.bash"

export ROS_DOMAIN_ID=0

# Welcome information
echo "ZED ROS2 Docker Image"
echo "---------------------"
echo 'ROS distro: ' $ROS_DISTRO
echo 'ROS install mode: ' ${ROS_INSTALL_MODE:-apt}
echo 'DDS middleware: ' $RMW_IMPLEMENTATION
echo 'ROS 2 Workspaces:' $COLCON_PREFIX_PATH
echo 'ROS 2 Domain ID:' $ROS_DOMAIN_ID
echo ' * Note: Host and Docker image Domain ID must match to allow communication'
echo 'Local IPs:' $(hostname -I)
echo "---"
echo 'Available ZED packages:'
ros2 pkg list | grep zed
echo "---------------------"
echo 'To start a ZED camera node:'
echo '  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<zed|zedm|zed2|zed2i|zedx|zedxm|zedxonegs|zedxone4k|zedxonehdr|virtual>'
echo "---------------------"
exec "$@"
