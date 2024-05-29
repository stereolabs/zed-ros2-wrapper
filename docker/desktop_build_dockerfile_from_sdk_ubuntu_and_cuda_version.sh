#!/bin/bash
cd $(dirname $0)

if [ "$#" -lt 3 ]; then
    echo "Give Ubuntu version then CUDA version then ZED SDK version has parameters, like this:"
    echo "./desktop_build_dockerfile_from_sdk_ubuntu_and_cuda_version.sh ubuntu22.04 cuda12.1.0 zedsdk4.1.2"
    exit 1
fi

# Ubuntu version
# Verify the format (l4t-r digits.digits.digits)
if ! [[ $1 =~ ^ubuntu[0-9]+\.[0-9]+$ ]]; then
    echo "Invalid Ubuntu version format."
    exit 1
fi

ubuntu_version=$1
ubuntu_version_number="${ubuntu_version#ubuntu}"

# CUDA version
# Verify the format (l4t-r digits.digits.digits)
if ! [[ $2 =~ ^cuda[0-9]+\.[0-9]\.[0-9]$ ]]; then
    echo "Invalid CUDA version format."
    exit 1
fi

cuda_version=$2
cuda_version_number="${cuda_version#cuda}"


# Verify the ZED SDK format (digits.digits.digits)
if ! [[ $3 =~ ^zedsdk[0-9]\.[0-9]\.[0-9]$ ]]; then
    echo "Invalid ZED SDK version format."
    exit 1
fi

ZED_SDK_version=$3
# Remove the prefix 'zedsdk-'
zed_sdk_version_number="${ZED_SDK_version#zedsdk}"

# copy the wrapper content
rm -r ./tmp_sources
mkdir -p ./tmp_sources
cp -r ../zed* ./tmp_sources

# Split the string and assign to variables
IFS='.' read -r cuda_major cuda_minor cuda_patch <<< "$cuda_version_number"
echo "CUDA $cuda_major.$cuda_minor.$cuda_patch detected."

###########

# Split the string and assign to variables
IFS='.' read -r ubuntu_major ubuntu_minor <<< "$ubuntu_version_number"
echo "Ubuntu $ubuntu_major.$ubuntu_minor detected."
###########

# Split the string and assign to variables
IFS='.' read -r major minor patch <<< "$zed_sdk_version_number"
echo "ZED SDK $major.$minor.$patch detected."

echo "Building dockerfile for $1 and ZED SDK $2"
docker build -t zed_ros2_desktop_image \
--build-arg ZED_SDK_MAJOR=$major \
--build-arg ZED_SDK_MINOR=$minor \
--build-arg ZED_SDK_PATCH=$patch \
--build-arg UBUNTU_MAJOR=$ubuntu_major \
--build-arg UBUNTU_MINOR=$ubuntu_minor \
--build-arg CUDA_MAJOR=$cuda_major \
--build-arg CUDA_MINOR=$cuda_minor \
--build-arg CUDA_PATCH=$cuda_patch \
-f ./Dockerfile.desktop-humble .