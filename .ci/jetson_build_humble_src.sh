#!/bin/bash -ex

pwd_path="$(pwd)"
if [[ ${pwd_path:${#pwd_path}-3} == ".ci" ]] ; then cd .. && pwd_path="$(pwd)"; fi
ttk="===>"
WORKDIR=${pwd_path}
PROJ_NAME=${PWD##*/}

echo "${ttk} Root repository folder: ${WORKDIR}"
echo "${ttk} Repository name: ${PROJ_NAME}"
echo "${ttk} User: ${USER}"

# Create the ROS 2 workspace
echo "${ttk} Create ROS2 workspace"
cd ..
WS_DIR="$(pwd)"/ros2_ws
rm -rf ${WS_DIR} # clean residual cached files
mkdir -p ${WS_DIR}/src 
echo "${ttk} ROS2 Workspace: ${WS_DIR}"

echo "${ttk} Check environment variables"
env | grep ROS

echo "${ttk} Update bin repositories"
apt-get update || true
TZ="Europe/Paris" apt-get upgrade --yes
rosdep update

echo "${ttk} Install ZED ROS2 Package dependencies from the sources"
cd ${WS_DIR}/src
XACRO_VERSION=2.0.8
wget https://github.com/ros/xacro/archive/refs/tags/${XACRO_VERSION}.tar.gz -O - | tar -xvz && mv xacro-${XACRO_VERSION} xacro
DIAGNOSTICS_VERSION=3.0.0
wget https://github.com/ros/diagnostics/archive/refs/tags/${DIAGNOSTICS_VERSION}.tar.gz -O - | tar -xvz && mv diagnostics-${DIAGNOSTICS_VERSION} diagnostics
AMENT_LINT_VERSION=0.12.4
wget https://github.com/ament/ament_lint/archive/refs/tags/${AMENT_LINT_VERSION}.tar.gz -O - | tar -xvz && mv ament_lint-${AMENT_LINT_VERSION} ament-lint

echo "${ttk} Current folder"
pwd
echo "${ttk} Content"
ls -lah

echo "${ttk} Build the dependencies"
cd ..
colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

echo "${ttk} Copy the ZED ROS2 Package sources in the workspace"
cd ${WORKDIR}
cd ..
echo "cp -a ./${PROJ_NAME} ${WS_DIR}/src/"
cp -a ./${PROJ_NAME} ${WS_DIR}/src/

echo "${ttk} Check that all the dependencied of the ZED ROS2 Package are satisfied"
cd ${WS_DIR}
rosdep install --from-paths src --ignore-src -r -y

echo "${ttk} Build the ZED ROS2 Package"
cd ${WS_DIR}
colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

echo "${ttk} Prepare 'install' artifact"
cd ${WS_DIR}
mkdir -p ${WORKDIR}/ros2_ws
cp -a ./install ${WORKDIR}/ros2_ws/

cd ${WORKDIR}
