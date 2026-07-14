# =============================================================================
# ZED ROS 2 Wrapper - Docker image
#
# Base image: a StereoLabs ZED image (https://hub.docker.com/r/stereolabs/zed/tags)
# These images already ship CUDA and the ZED SDK, so neither is installed here.
# This single Dockerfile is used for both desktop (CUDA/Ubuntu) and Jetson
# (L4T) targets; only the base image tag changes.
#
# Build it through ./build_desktop.sh or ./build_jetson.sh, which select the
# right StereoLabs base image and pass the build arguments below.
# =============================================================================

# Full StereoLabs base image tag, e.g.:
#   stereolabs/zed:5.4-devel-cuda12.8-ubuntu24.04   (desktop)
#   stereolabs/zed:5.3-devel-l4t-r36.5              (Jetson)
#
# When building against a custom/unreleased ZED SDK (build_*.sh --sdk-url), this
# is instead a plain CUDA / L4T image and the SDK is installed further below.
ARG BASE_IMAGE=stereolabs/zed:5.4-devel-cuda12.8-ubuntu24.04
FROM ${BASE_IMAGE}

# ROS 2 distribution to install (default: Jazzy)
ARG ROS2_DISTRO=jazzy
ENV ROS_DISTRO=${ROS2_DISTRO}

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris

ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility

# --- Common build/runtime tooling and locale --------------------------------
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    apt-get update || true && \
    apt-get install -y --no-install-recommends \
      apt-utils dialog locales \
      curl wget gnupg2 lsb-release software-properties-common \
      build-essential cmake git jq less udev sudo zstd usbutils \
      python3 python3-dev python3-pip python3-wheel \
      libopencv-dev libpq-dev && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --- Optional: install a custom/unreleased ZED SDK, either from a URL or
#     from a local '.run' file staged into the build context ----------------
# When CUSTOM_ZED_SDK_URL is set, the base image is a plain CUDA / L4T image
# (see build_*.sh --sdk-url) and the ZED SDK is fetched from that URL.
# CUSTOM_ZED_SDK_LOCAL_FILE is set instead when --sdk-url pointed to a local
# file: build_*.sh stages it into tmp_sdk/ since a Docker build cannot see the
# host filesystem otherwise. When both are empty (the default), the base
# image is a stereolabs/zed image that already ships the SDK and this step is
# a no-op.
ARG CUSTOM_ZED_SDK_URL=""
ARG CUSTOM_ZED_SDK_LOCAL_FILE=""
COPY tmp_sdk/ /tmp/sdk_installer/
COPY scripts/install_zed_sdk.sh /tmp/install_zed_sdk.sh
RUN chmod +x /tmp/install_zed_sdk.sh && /tmp/install_zed_sdk.sh "${CUSTOM_ZED_SDK_URL}" "${CUSTOM_ZED_SDK_LOCAL_FILE}"

# --- Install ROS 2 (APT if binaries exist for this Ubuntu release, else build
#     ros_base from source) -------------------------------------------------
COPY scripts/install_ros2.sh /tmp/install_ros2.sh
RUN chmod +x /tmp/install_ros2.sh && /tmp/install_ros2.sh "${ROS_DISTRO}"

# --- Copy the ZED ROS 2 Wrapper sources from the current checkout ------------
WORKDIR /root/ros2_ws/src
COPY tmp_sources/ ./

# --- Build the wrapper (and, in source mode, its source-only dependencies) ---
COPY scripts/zed-wrapper-deps.repos /tmp/zed-wrapper-deps.repos
COPY scripts/build_wrapper.sh /tmp/build_wrapper.sh
WORKDIR /root/ros2_ws
RUN chmod +x /tmp/build_wrapper.sh && /tmp/build_wrapper.sh "${ROS_DISTRO}"

# --- Entry point -------------------------------------------------------------
COPY scripts/ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN chmod 755 /sbin/ros_entrypoint.sh

ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]
