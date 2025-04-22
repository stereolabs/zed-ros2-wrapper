#!/bin/bash
cd $(dirname $0)

if [ "$#" -lt 3 ]; then
    echo "Give Ubuntu version then CUDA version then ZED SDK version has parameters, like this:"
    echo "./desktop_build_dockerfile_from_sdk_ubuntu_and_cuda_version.sh ubuntu-22.04 cuda-12.6.3 zedsdk-4.2.3"
    exit 1
fi

# Ubuntu version
# Verify the format (l4t-r digits.digits.digits)
if ! [[ $1 =~ ^ubuntu-[0-9]+\.[0-9]+$ ]]; then
    echo "Invalid Ubuntu version format."
    exit 1
fi

ubuntu_version=$1
ubuntu_version_number="${ubuntu_version#ubuntu-}"

# Split the string and assign to variables
IFS='.' read -r ubuntu_major ubuntu_minor <<< "$ubuntu_version_number"
echo "Ubuntu $ubuntu_major.$ubuntu_minor detected."

# CUDA version
# Verify the format (l4t-r digits.digits.digits)
if ! [[ $2 =~ ^cuda-[0-9]+\.[0-9]\.[0-9]$ ]]; then
    echo "Invalid CUDA version format."
    exit 1
fi

cuda_version=$2
cuda_version_number="${cuda_version#cuda-}"

# Split the string and assign to variables
IFS='.' read -r cuda_major cuda_minor cuda_patch <<< "$cuda_version_number"
echo "CUDA $cuda_major.$cuda_minor.$cuda_patch detected."


ZED_SDK_version=$3

# copy the wrapper content
rm -r ./tmp_sources
mkdir -p ./tmp_sources
cp -r ../zed* ./tmp_sources

# Check if the third arg is a custom path
CUSTOM_ZED_SDK_URL=$3
# Use curl to check if the URL is valid (returns HTTP 200)
if [ "$(curl -L -I "${CUSTOM_ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}\n' | head -n 1)" = "200" ]; then
    echo "${ZED_SDK_version} detected as a valid custom installer URL"

    echo "Building dockerfile for $1, CUDA $2 and a custom ZED SDK"

    docker build -t zed_ros2_desktop_u${ubuntu_major}.${ubuntu_minor}_sdk_custom_cuda_${cuda_major}.${cuda_minor}.${cuda_patch} \
    --build-arg CUSTOM_ZED_SDK_URL=$CUSTOM_ZED_SDK_URL \
    --build-arg UBUNTU_MAJOR=$ubuntu_major \
    --build-arg UBUNTU_MINOR=$ubuntu_minor \
    --build-arg CUDA_MAJOR=$cuda_major \
    --build-arg CUDA_MINOR=$cuda_minor \
    --build-arg CUDA_PATCH=$cuda_patch \
    -f ./Dockerfile.desktop-humble .
else
    # Verify the ZED SDK format (digits.digits.digits)
    if ! [[ $3 =~ ^zedsdk-[0-9]\.[0-9]\.[0-9]$ ]]; then
        echo "Invalid ZED SDK version format."
        exit 1
    fi

    # Remove the prefix 'zedsdk-'
    zed_sdk_version_number="${ZED_SDK_version#zedsdk-}"

    # Split the string and assign to variables
    IFS='.' read -r sdk_major sdk_minor sdk_patch <<< "$zed_sdk_version_number"
    echo "ZED SDK $major.$minor.$patch detected."

    echo "Building dockerfile for $1, CUDA $2 and ZED SDK $3"

    docker build -t zed_ros2_desktop_u${ubuntu_major}.${ubuntu_minor}_sdk_${sdk_major}.${sdk_minor}.${sdk_patch}_cuda_${cuda_major}.${cuda_minor}.${cuda_patch} \
    --build-arg ZED_SDK_MAJOR=$sdk_major \
    --build-arg ZED_SDK_MINOR=$sdk_minor \
    --build-arg ZED_SDK_PATCH=$sdk_patch \
    --build-arg UBUNTU_MAJOR=$ubuntu_major \
    --build-arg UBUNTU_MINOR=$ubuntu_minor \
    --build-arg CUDA_MAJOR=$cuda_major \
    --build-arg CUDA_MINOR=$cuda_minor \
    --build-arg CUDA_PATCH=$cuda_patch \
    -f ./Dockerfile.desktop-humble .
fi

###########

# Remove the temporary folder
rm -r ./tmp_sources