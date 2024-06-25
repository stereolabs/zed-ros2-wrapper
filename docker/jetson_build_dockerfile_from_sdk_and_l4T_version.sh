#!/bin/bash
cd $(dirname $0)

if [ "$#" -lt 2 ]; then
    echo "Give L4T version then ZED SDK version has parameters, like this:"
    echo "./jetson_build_dockerfile_from_sdk_and_l4T_version.sh l4t-r35.4.1 zedsdk4.1.2"
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

# Verify the ZED SDK format (digits.digits.digits)
if ! [[ $2 =~ ^zedsdk[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Invalid ZED SDK version format."
    exit 1
fi

ZED_SDK_version=$2
# Remove the prefix 'zedsdk-'
zed_sdk_version_number="${ZED_SDK_version#zedsdk}"

# copy the wrapper content
rm -r ./tmp_sources
mkdir -p ./tmp_sources
cp -r ../zed* ./tmp_sources

# Split the string and assign to variables
IFS='.' read -r l4t_major l4t_minor l4t_patch <<< "$l4t_version_number"
###########

# Split the string and assign to variables
IFS='.' read -r major minor patch <<< "$zed_sdk_version_number"

echo "Building dockerfile for $1 and ZED SDK $2"
docker build -t zed_ros2_l4t_image \
--build-arg ZED_SDK_MAJOR=$major \
--build-arg ZED_SDK_MINOR=$minor \
--build-arg ZED_SDK_PATCH=$patch \
--build-arg L4T_VERSION=$1 \
--build-arg L4T_MAJOR=$l4t_major \
--build-arg L4T_MINOR=$l4t_minor \
-f ./Dockerfile.l4t-humble .