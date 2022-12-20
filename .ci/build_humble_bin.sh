#!/bin/bash -ex

pwd_path="$(pwd)"
if [[ ${pwd_path:${#pwd_path}-3} == ".ci" ]] ; then cd .. && pwd_path="$(pwd)"; fi
ttk="===>"
root_path=${pwd_path}
repo_name=${PWD##*/}

echo "${ttk} Root repository folder: ${root_path}"
echo "${ttk} Repository name: ${repo_name}"

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
echo "cp -a ./${repo_name} ${ws_path}/src/"
cp -a ./${repo_name} ${ws_path}/src/
#echo "${ttk} '${ws_path}/src' content"
#ls -lha ${ws_path}/src
#echo "${ttk} '${ws_path}/src/${repo_name}' content"
#ls -lha ${ws_path}/src/${repo_name}

echo "${ttk} Check environment variables"
env | grep ROS

echo "${ttk} Update rosdep"
rosdep update

echo "${ttk} Install ZED ROS2 Package dependencies"
cd ${ws_path}
rosdep install --from-paths src --ignore-src -r -y

echo "${ttk} Build the ZED ROS2 Package"
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

echo "${ttk} Prepare 'install' artifact"
cd ${ws_path}
mkdir -p ${root_path}/ros2
cp -a ./install ${root_path}/ros2/

cd ${root_path}
