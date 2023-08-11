#!/bin/bash
set -e

UBUNTU_RELEASE_YEAR=$1
CUDA_MAJOR=$2
CUDA_MINOR=$3
ZED_SDK_MAJOR=$4
ZED_SDK_MINOR=$5

ttk="***>"

echo "Europe/Paris" > /etc/localtime ; echo "CUDA Version ${CUDA_MAJOR}.${CUDA_MINOR}.0" > /usr/local/cuda/version.txt

# Setup the ZED SDK
echo "${ttk} Installing ZED SDK v${ZED_SDK_MAJOR}.${ZED_SDK_MINOR} for Ubuntu ${UBUNTU_RELEASE_YEAR}.04 CUDA ${CUDA_MAJOR}.${CUDA_MINOR}"
apt-get update -y || true
apt-get install --no-install-recommends lsb-release wget less udev sudo  build-essential cmake zstd python3 python3-pip libpng-dev libgomp1 -y && \
    python3 -m pip install --upgrade pip; python3 -m pip install numpy opencv-python-headless && \
    wget -q -O ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}${CUDA_MINOR%.*}/ubuntu${UBUNTU_RELEASE_YEAR} && \
    chmod +x ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run ; ./ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run -- silent skip_tools skip_cuda skip_python skip_hub && \
    ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so && \
    rm ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run && \
    rm -rf /var/lib/apt/lists/*
