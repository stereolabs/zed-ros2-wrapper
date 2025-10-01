#!/bin/bash -ex

pwd_path="$(pwd)"
if [[ ${pwd_path:${#pwd_path}-3} == ".ci" ]] ; then cd .. && pwd_path="$(pwd)"; fi
ttk="===>"
WORKDIR=${pwd_path}
PROJ_NAME=${PWD##*/}

echo "${ttk} Root repository folder: ${WORKDIR}"
echo "${ttk} Repository name: ${PROJ_NAME}"

# Create the ROS 2 workspace
echo "${ttk} Create ROS 2 workspace"
cd ..
WS_DIR="$(pwd)"/ros2_ws
rm -rf ${WS_DIR} # clean residual cache files
mkdir -p ${WS_DIR}/src 
echo "${ttk} ROS 2 Workspace: ${WS_DIR}"
cd ${WORKDIR}
cd ..
echo "cp -a ./${PROJ_NAME} ${WS_DIR}/src/"
cp -a ./${PROJ_NAME} ${WS_DIR}/src/

echo "${ttk} Check environment variables"
env | grep ROS

echo "${ttk} Update bin repositories"
apt-get update || true
apt-get upgrade --yes
rosdep update

echo "${ttk} Install ZED ROS2 Package dependencies"
cd ${WS_DIR}
rosdep install --from-paths src --ignore-src -r -y

echo "${ttk} Build the ZED ROS2 Package"
colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

echo "${ttk} Prepare 'install' artifact"
cd ${WS_DIR}
mkdir -p ${WORKDIR}/ros2_ws
cp -a ./install ${WORKDIR}/ros2_ws/

cd ${WORKDIR}
