#!/bin/bash -ex

pwd_path="$(pwd)"
if [[ ${pwd_path:${#pwd_path}-3} == ".ci" ]] ; then cd .. && pwd_path="$(pwd)"; fi
ttk="===>"
root_path=${pwd_path}
repo_name=${PWD##*/}

echo "${ttk} Root repository folder: ${root_path}"
echo "${ttk} Repository name: ${repo_name}"

echo "${ttk} Building the ROS2 node in Foxy installed from binaries."

# Create the ROS 2 workspace
echo "${ttk} Create ROS2 workspace"
cd ..
ws_path="$(pwd)"/ros2_ws
mkdir -p ${ws_path}/src 
echo "${ttk} ROS2 Workspace: ${ws_path}"
#echo "${ttk} '${ws_path}' content"
#ls -lah ${ws_path}
cd ${root_path}
cd ..
#echo "${ttk} Current path: $(pwd)"
#ls -lah
cp -a ./${repo_name} ${ws_path}/src/
#echo "${ttk} '${ws_path}/src' content"
#ls -lha ${ws_path}/src
#echo "${ttk} '${ws_path}/src/${repo_name}' content"
#ls -lha ${ws_path}/src/${repo_name}

echo "${ttk} Install ROS2 Foxy"

echo "${ttk} Set Locale"
#locale  # check for UTF-8
apt-get update && apt-get install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

echo "${ttk} Setup Sources"
apt-get install -y software-properties-common
add-apt-repository universe
apt-get update && apt-get install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "${ttk} Install ROS 2 packages"
apt-get update && apt-get upgrade -y && apt-get autoremove && apt-get autoclean
apt-get install -y build-essential ros-foxy-ros-base python3-argcomplete python3-flake8-docstrings python3-pip python3-pytest-cov ros-dev-tools
pip3 install numpy
pip3 install empy
pip3 install lark

echo "${ttk} Sourcing the setup script"
source /opt/ros/foxy/setup.bash

echo "${ttk} Initialize rosdep"
rosdep init
rosdep update

echo "${ttk} Check environment variables"
env | grep ROS

echo "${ttk} ROS2 Foxy is ready"

echo "${ttk} Install Node dependencies"
cd ${ws_path}
rosdep install --from-paths src --ignore-src -r -y

echo "${ttk} Build the node"
# Fix not existing `/usr/bin/gmake` error
ln -s /usr/bin/make /usr/bin/gmake
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

echo "${ttk} Prepare 'install' artifact"
cd ${ws_path}
mkdir -p ${root_path}/ros2
cp -a ./install ${root_path}/ros2/

cd ${root_path}
