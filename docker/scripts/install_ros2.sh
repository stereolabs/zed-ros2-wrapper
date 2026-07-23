#!/bin/bash
# =============================================================================
# install_ros2.sh
#
# Installs ROS 2 inside a StereoLabs ZED base image.
#
#   * If the binary packages for the requested ROS 2 distribution are available
#     on APT for the base image Ubuntu release, ROS 2 is installed from APT.
#   * Otherwise ROS 2 (ros_base) is built from source with colcon.
#
# In both cases the resulting installation lives in /opt/ros/${ROS_DISTRO} and a
# small environment file (/opt/ros_env) is written so that the entry point and
# the wrapper build step can source the right setup.bash.
#
# Usage: install_ros2.sh <ros2_distro>
# =============================================================================
set -e

ROS_DISTRO="${1:?Usage: install_ros2.sh <ros2_distro>}"

# Ubuntu codename of the base image (e.g. jammy, noble). This is the actual OS
# of the StereoLabs image, regardless of how the tag is named.
UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME}")"

echo "============================================================"
echo " Installing ROS 2 '${ROS_DISTRO}' on Ubuntu '${UBUNTU_CODENAME}'"
echo "============================================================"

export DEBIAN_FRONTEND=noninteractive

# --- ROS 2 APT repository ----------------------------------------------------
# The repository is keyed by Ubuntu codename and provides both the per-distro
# ROS packages (when available) and the distro-independent 'ros-dev-tools'
# (colcon, vcstool, rosdep, ...) used by the from-source build.
apt-get update || true
apt-get install -y --no-install-recommends curl gnupg2 lsb-release software-properties-common
add-apt-repository -y universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
  > /etc/apt/sources.list.d/ros2.list
apt-get update

# --- Decide APT vs. source ---------------------------------------------------
if apt-cache show "ros-${ROS_DISTRO}-ros-base" > /dev/null 2>&1; then
  INSTALL_MODE="apt"
else
  INSTALL_MODE="source"
fi
echo ">>> ROS 2 '${ROS_DISTRO}' install mode: ${INSTALL_MODE}"

if [ "${INSTALL_MODE}" = "apt" ]; then
  # ---------------------------------------------------------------------------
  # Binary installation from APT
  # ---------------------------------------------------------------------------
  apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-ros-base" \
    ros-dev-tools \
    python3-flake8-docstrings \
    python3-pytest-cov
  rosdep init || true
  rosdep update --rosdistro "${ROS_DISTRO}"
else
  # ---------------------------------------------------------------------------
  # From-source installation (ros_base) for a supported distribution that has no
  # binaries for this Ubuntu release (e.g. Lyrical on Ubuntu 24.04 / L4T r38.x,
  # whose APT binaries are published for Ubuntu 26.04 only).
  # ---------------------------------------------------------------------------
  echo ">>> No binary packages for '${ROS_DISTRO}' on '${UBUNTU_CODENAME}': building ROS 2 from source."
  apt-get install -y --no-install-recommends ros-dev-tools

  ROS2_SRC_WS=/opt/ros2_build
  mkdir -p "${ROS2_SRC_WS}/src"
  cd "${ROS2_SRC_WS}"

  # Core ROS 2 sources for the requested distribution.
  vcs import --input "https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos" src

  # System (non-ROS) dependencies. ROS packages themselves are resolved from the
  # source tree, so only apt system packages are pulled here.
  rosdep init || true
  rosdep update --rosdistro "${ROS_DISTRO}"
  # Skip the proprietary, EULA-gated RTI Connext DDS keys (all known versions):
  # their .deb aborts on a non-interactive pre-install and breaks dpkg/rosdep.
  # Connext is an optional alternative RMW; Fast-DDS (the default) is built anyway.
  rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    --skip-keys "fastcdr rti-connext-dds-6.0.1 rti-connext-dds-7.3.0 rti-connext-dds-7.7.0 urdfdom_headers" || true

  # Build the ROS 2 core (the full ros2.repos set, as in the official
  # from-source install) directly into /opt/ros/${ROS_DISTRO}. This set already
  # provides image_transport (image_common), tf2_geometry_msgs (geometry2) and
  # rmw_cyclonedds, which the wrapper needs; the remaining wrapper dependencies
  # are added later from zed-wrapper-deps.repos.
  colcon build \
    --merge-install \
    --install-base "/opt/ros/${ROS_DISTRO}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

  # The build tree is large; keep only the install space.
  rm -rf "${ROS2_SRC_WS}/build" "${ROS2_SRC_WS}/log"
fi

# --- Environment file shared with the entry point and the build step ---------
{
  echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
  echo "export ROS_INSTALL_MODE=${INSTALL_MODE}"
} > /opt/ros_env

rm -rf /var/lib/apt/lists/*
echo ">>> ROS 2 '${ROS_DISTRO}' installation complete (${INSTALL_MODE})."
