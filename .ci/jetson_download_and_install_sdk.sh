#!/bin/bash
set -e

JETPACK_MAJOR=$1
JETPACK_MINOR=$2
L4T_MINOR_VERSION=$3
ZED_SDK_MAJOR=$4
ZED_SDK_MINOR=$5

ttk="***>"

echo "Europe/Paris" > /etc/localtime

apt-get update -y && apt-get install -y sudo apt-utils apt-transport-https lsb-release udev usbutils git 

#Install ZED SDK
echo "${ttk} Installing ZED SDK v${ZED_SDK_MAJOR}.${ZED_SDK_MINOR} for Jetpack ${JETPACK_MAJOR}.${JETPACK_MINOR}"
apt-get update -y && apt-get install -y --no-install-recommends wget less cmake curl gnupg2 \
    build-essential python3 python3-pip python3-dev python3-setuptools libusb-1.0-0-dev -y && \
    sudo pip install protobuf && \
    echo "# R35 (release), REVISION: ${L4T_MINOR_VERSION}" > /etc/nv_tegra_release ; \
    wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/jp${JETPACK_MAJOR}${JETPACK_MINOR}/jetsons && \
    chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_tools && \
    rm -rf /usr/local/zed/resources/* && \
    rm -rf ZED_SDK_Linux_JP.run && \
    rm -rf /var/lib/apt/lists/*

#This symbolic link is needed to use the streaming features on Jetson inside a container
ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so
