#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "~/ros2_ws/install/local_setup.bash" --
exec "$@"
