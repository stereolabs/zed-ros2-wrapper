#!/bin/bash
set -e

JETPACK_MAJOR=$1
JETPACK_MINOR=$2
L4T_MAJOR=$3
L4T_MINOR=$4
ZED_SDK_MAJOR=$5
ZED_SDK_MINOR=$6

ttk="***>"

echo "Europe/Paris" > /etc/timezone
echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release
    
#Install ZED SDK
echo "${ttk} Installing ZED SDK v${ZED_SDK_MAJOR}.${ZED_SDK_MINOR} for Jetpack ${JETPACK_MAJOR}.${JETPACK_MINOR} (L4T v${L4T_MAJOR}.${L4T_MINOR})"
apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 42D5A192B819C5DA
apt-get update -y || true
apt-get install -y --no-install-recommends zstd wget less cmake curl gnupg2 \
    build-essential python3 python3-pip python3-dev python3-setuptools libusb-1.0-0-dev -y && \
    pip install protobuf && \
    wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run \
    https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons && \
    chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_tools && \
    rm -rf /usr/local/zed/resources/* && \
    rm -rf ZED_SDK_Linux_JP.run && \
    rm -rf /var/lib/apt/lists/*
