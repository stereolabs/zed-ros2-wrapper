#!/bin/bash
cd $(dirname $0)

if [ "$#" -lt 2 ]; then
    echo "Please enter valid L4T version and ZED SDK version has parameters. For example:"
    echo "./jetson_build_dockerfile_from_sdk_and_l4T_version.sh l4t-r36.3.0 zedsdk-4.2.5"
    exit 1
fi

# L4T version
# Verify the format (l4t-r digits.digits.digits)
if ! [[ $1 =~ ^l4t-r[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Invalid L4T version format."
    exit 1
fi

L4T_version=$1
# Remove the prefix 'l4t-r'
l4t_version_number="${L4T_version#l4t-r}"


ZED_SDK_version=$2


# copy the wrapper content
rm -r ./tmp_sources
mkdir -p ./tmp_sources
cp -r ../zed* ./tmp_sources

# Split the string and assign to variables
IFS='.' read -r l4t_major l4t_minor l4t_patch <<< "$l4t_version_number"
###########

L4T_VERSION=${1:-l4t-r${l4t_major}.${l4t_minor}.${l4t_patch}}

# Determine the IMAGE_NAME based on the L4T_VERSION
if [[ "$L4T_VERSION" == *"l4t-r36.4"* ]]; then
    IMAGE_NAME="dustynv/ros:humble-desktop-${L4T_VERSION}"
else
    IMAGE_NAME="dustynv/ros:humble-ros-base-${L4T_VERSION}"
fi

# Check if the third arg is a custom path
CUSTOM_ZED_SDK_URL=$2
# Use curl to check if the URL is valid (returns HTTP 200)
if [ "$(curl -L -I "${CUSTOM_ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}\n' | head -n 1)" = "200" ]; then
    echo "${ZED_SDK_version} detected as a valid custom installer URL"

    echo "Building dockerfile for $1 and a custom ZED SDK"

    docker build -t zed_ros2_l4t_${l4t_major}.${l4t_minor}.${l4t_patch}_sdk_custom \
    --build-arg CUSTOM_ZED_SDK_URL=$CUSTOM_ZED_SDK_URL \
    --build-arg L4T_VERSION=$1 \
    --build-arg L4T_MAJOR=$l4t_major \
    --build-arg L4T_MINOR=$l4t_minor \
    --build-arg IMAGE_NAME=$IMAGE_NAME \
    -f ./Dockerfile.l4t-humble .
else
    # Verify the ZED SDK format (digits.digits.digits)
    if ! [[ $2 =~ ^zedsdk-[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        echo "Invalid ZED SDK version format."
        exit 1
    fi

    # Remove the prefix 'zedsdk-'
    zed_sdk_version_number="${ZED_SDK_version#zedsdk-}"

    # Split the string and assign to variables
    IFS='.' read -r sdk_major sdk_minor sdk_patch <<< "$zed_sdk_version_number"

    echo "Building dockerfile for $1 and ZED SDK $2"

    docker build -t zed_ros2_l4t_${l4t_major}.${l4t_minor}.${l4t_patch}_sdk_${sdk_major}.${sdk_minor}.${sdk_patch} \
    --build-arg ZED_SDK_MAJOR=$sdk_major \
    --build-arg ZED_SDK_MINOR=$sdk_minor \
    --build-arg ZED_SDK_PATCH=$sdk_patch \
    --build-arg L4T_VERSION=$1 \
    --build-arg L4T_MAJOR=$l4t_major \
    --build-arg L4T_MINOR=$l4t_minor \
    --build-arg IMAGE_NAME=$IMAGE_NAME \
    -f ./Dockerfile.l4t-humble .
fi

# Remove the temporary folder
rm -r ./tmp_sources