#!/bin/bash -e

ZED_SDK_MAJOR=4
ZED_SDK_MINOR=1

# Retrieve CUDA version from environment variable CUDA_VERSION
CUDA_MAJOR=`echo ${CUDA_VERSION} | cut -d. -f1`
CUDA_MINOR=`echo ${CUDA_VERSION} | cut -d. -f2`

pwd_path="$(pwd)"
if [[ ${pwd_path:${#pwd_path}-3} == ".ci" ]] ; then cd .. && pwd_path="$(pwd)"; fi
ttk="---> "
ROOT_PATH=${pwd_path}
REPO_NAME=${PWD##*/}

echo "${ttk} Root repository folder: ${ROOT_PATH}"
echo "${ttk} Repository name: ${REPO_NAME}"
#echo "${ttk} User: ${USER}"

sudocmd=""
if  [[ ! $(uname) == "MINGW"* ]]; then
	LINUX_OS=1
    if  [[ ! ${CI_RUNNER_TAGS} == *"docker-builder"* ]]; then
	    sudocmd="sudo "
    fi
fi

${sudocmd} chmod +x .ci/*.sh

#the . command.sh syntaxe allows env var to be accessible cross-scripts (needed for timers)

# Check Ubuntu version
ubuntu=$(lsb_release -r)
echo "${ttk} Ubuntu $ubuntu"
VER=$(cut -f2 <<< "$ubuntu")
echo "${ttk} Version: ${VER}"

# Build the node
cd "${ROOT_PATH}"
ARCH=$(uname -m)
echo "${ttk} Architecture: ${ARCH}"
if [[ $ARCH == "x86_64" ]]; then     
    if [[ $VER == "20.04" ]]; then 
        echo "${ttk} Install the ZED SDK for ${ARCH} under Ubuntu ${VER}"
        . .ci/download_and_install_sdk.sh 20 ${CUDA_MAJOR} ${CUDA_MINOR} ${ZED_SDK_MAJOR} ${ZED_SDK_MINOR}
        echo "${ttk} Build ROS 2 Humble from the source."    
        . .ci/build_humble_src.sh
    fi
    if [[ $VER == "22.04" ]]; then 
        echo "${ttk} Install the ZED SDK for ${ARCH} under Ubuntu ${VER}"
        . .ci/download_and_install_sdk.sh 22 ${CUDA_MAJOR} ${CUDA_MINOR} ${ZED_SDK_MAJOR} ${ZED_SDK_MINOR}
        echo "${ttk} Install ROS 2 Humble from the binaries."
        . .ci/build_humble_bin.sh
    fi
elif [[ $ARCH == "aarch64" ]]; then 
if [[ $VER == "20.04" ]]; then 
        JP_MAJOR=5
        JP_MINOR=0
        L4T_MAJOR=35
        L4T_MINOR=1
        echo "${ttk} Install the ZED SDK for ${ARCH} under Ubuntu ${VER}"
        . .ci/jetson_download_and_install_sdk.sh ${JP_MAJOR} ${JP_MINOR} ${L4T_MAJOR} ${L4T_MINOR} ${ZED_SDK_MAJOR} ${ZED_SDK_MINOR}
        echo "${ttk} Build ROS 2 Humble from the source."    
        . .ci/jetson_build_humble_src.sh    
    fi
else
    echo "${ttk} Architecture ${ARCH} is not supported."
    exit 1
fi
if [ $? -ne 0 ]; then echo "${ttk} ROS 2 Node build failed" > "$pwd_path/failure.txt" ; cat "$pwd_path/failure.txt" ; exit 1 ; fi
