#!/bin/bash
# =============================================================================
# build_wrapper.sh
#
# Builds the ZED ROS 2 Wrapper (already copied into /root/ros2_ws/src) on top of
# the ROS 2 installation produced by install_ros2.sh.
#
#   * APT mode:    wrapper ROS dependencies are pulled with rosdep from APT.
#   * Source mode: the wrapper ROS dependencies that are not part of ros_base
#                  are fetched from source (zed-wrapper-deps.repos) and built
#                  together with the wrapper.
#
# Usage: build_wrapper.sh <ros2_distro>
# =============================================================================
set -e

ROS_DISTRO="${1:?Usage: build_wrapper.sh <ros2_distro>}"
WS=/root/ros2_ws
DEPS_REPOS=/tmp/zed-wrapper-deps.repos

export DEBIAN_FRONTEND=noninteractive

# Source the ROS 2 environment created by install_ros2.sh.
# shellcheck disable=SC1091
source /opt/ros_env

echo "============================================================"
echo " Building the ZED ROS 2 Wrapper (mode: ${ROS_INSTALL_MODE})"
echo "============================================================"

apt-get update || true
rosdep update --rosdistro "${ROS_DISTRO}" || true

cd "${WS}/src"

if [ "${ROS_INSTALL_MODE}" = "source" ]; then
  # ROS 2 was built from source: the wrapper ROS dependencies are not available
  # as binaries either, so fetch the ones that are not part of ros_base.
  echo ">>> Fetching wrapper ROS dependencies from source."

  # robot_localization does not always cut a "<distro>-devel" branch right
  # away for a new ROS 2 distro (e.g. Lyrical, as of this writing); fall back
  # to its distro-agnostic "rolling-devel" branch in that case.
  case "${ROS_DISTRO}" in
    humble|jazzy|kilted)
      ROBOT_LOCALIZATION_VERSION="${ROS_DISTRO}-devel"
      ;;
    *)
      ROBOT_LOCALIZATION_VERSION="rolling-devel"
      ;;
  esac

  sed -e "s/\${ROS_DISTRO}/${ROS_DISTRO}/g" \
      -e "s/\${ROBOT_LOCALIZATION_VERSION}/${ROBOT_LOCALIZATION_VERSION}/g" \
      "${DEPS_REPOS}" > /tmp/deps.resolved.repos
  vcs import --input /tmp/deps.resolved.repos .
fi

# The base image already ships a compatible OpenCV (matching the ZED SDK / GPU
# build). On Jetson, its "libopencv-dev"/"nvidia-opencv-dev" packages collide
# by name with Ubuntu's own (unaccelerated) OpenCV dev packages, which various
# vcs-imported wrapper dependencies pull in via rosdep (not just the
# "libopencv-dev" key - individual per-module keys like
# "libopencv-imgproc-dev" resolve the same way). Skipping the umbrella key
# alone is not enough, so hold the packages too: apt then fails to swap them
# out instead of silently uninstalling the good OpenCV as a side effect.
apt-mark hold libopencv-dev nvidia-opencv-dev nvidia-opencv \
  libopencv-python libopencv-samples opencv-licenses opencv-samples-data \
  2>/dev/null || true

# Install system (and, in APT mode, ROS) dependencies. '-r' lets the build
# continue when a key cannot be resolved (e.g. zed_description, which is not
# released to the ROS index).
rosdep install --from-paths . --ignore-src -r -y --rosdistro "${ROS_DISTRO}" \
  --skip-keys "libopencv-dev" || true

# find_package(OpenCV) in cv_bridge needs a CMake config file; point CMake at
# it explicitly since it is not always on the default search path.
OPENCV_CMAKE_DIR="$(find /usr /opt -maxdepth 6 -iname 'OpenCVConfig.cmake' 2>/dev/null | head -n 1 | xargs -r dirname)"
CMAKE_ARGS=(
  " -DCMAKE_BUILD_TYPE=Release"
  " -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs"
  " -DCMAKE_CXX_FLAGS=-Wl,--allow-shlib-undefined"
)
if [ -n "${OPENCV_CMAKE_DIR}" ]; then
  echo ">>> Found OpenCVConfig.cmake in ${OPENCV_CMAKE_DIR}; passing it as OpenCV_DIR."
  CMAKE_ARGS+=(" -DOpenCV_DIR=${OPENCV_CMAKE_DIR}")
fi
CMAKE_ARGS+=(" --no-warn-unused-cli")

cd "${WS}"
colcon build \
  --parallel-workers "$(nproc)" \
  --symlink-install \
  --event-handlers console_direct+ \
  --base-paths src \
  --packages-up-to zed_ros2 zed_debug \
  --cmake-args "${CMAKE_ARGS[@]}"

rm -rf /var/lib/apt/lists/*
echo ">>> ZED ROS 2 Wrapper build complete."
