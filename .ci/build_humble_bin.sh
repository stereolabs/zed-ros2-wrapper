#!/bin/bash

ttk="===>"

echo "${ttk} Building the ROS2 node in Humble installed from binaries."

echo "${ttk} Install ROS2 Humble"

echo "${ttk} Set Locale"
locale  # check for UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

echo "${ttk} Setup Sources"
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "${ttk} Install ROS 2 packages"
sudo apt update
sudo apt upgrade
sudo apt install -y ros-humble-ros-base ros-dev-tools

echo "${ttk} Sourcing the setup script"
source /opt/ros/humble/setup.bash

echo "${ttk} Check environment variables"
env | grep ROS

echo "${ttk} ROS2 Humble is ready"







