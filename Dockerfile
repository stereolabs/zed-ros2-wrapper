ARG IMAGE_NAME=nvcr.io/nvidia/cuda:12.1.0-devel-ubuntu22.04
FROM ${IMAGE_NAME}

ARG UBUNTU_MAJOR=22
#ARG UBUNTU_MINOR=04
ARG CUDA_MAJOR=12
ARG CUDA_MINOR=1
# ARG CUDA_PATCH=0

ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=1
ARG ZED_SDK_PATCH=0

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility

ENV DEBIAN_FRONTEND=noninteractive

########### ROS2 ###########

ENV ROS_DISTRO=humble
RUN apt update && apt install -y curl software-properties-common

# Setup ROS Sources
RUN add-apt-repository universe && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Development and Base packages
RUN apt update && apt install -y ros-dev-tools ros-humble-ros-core ros-humble-robot-state-publisher ros-${ROS_DISTRO}-sensor-msgs-py
RUN . /opt/ros/$ROS_DISTRO/setup.sh && rosdep init && rosdep update

########### ZED ###########

# Install Dependencies for the SDK installation
RUN apt update && apt install -y zstd
ENV ZED_SDK_URL="https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/ZED_SDK_Ubuntu${UBUNTU_MAJOR}_cuda${CUDA_MAJOR}.${CUDA_MINOR}_v${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}.zstd.run"

# Install the ZED SDK
RUN wget -q -O zed.run ${ZED_SDK_URL} && \
  chmod +x zed.run && \
  ./zed.run -- silent skip_cuda && \
  ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so && \
  rm -rf /usr/local/zed/resources/* zed.run

# Install extra dependencies. TODO remove these
#RUN apt update && apt install -y libpq-dev usbutils
#RUN pip3 install opencv-python-headless protobuf

###################### WS ######################
# Install Open3D system dependencies and pip
RUN apt-get update && apt-get install --no-install-recommends -y \
    libegl1 \
    libgl1 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir --upgrade open3d


########### COLCON BUILD ZED WRAPPER ###########
WORKDIR /root/ros2_ws/src
ADD . .

WORKDIR /root/ros2_ws

# # Install ROS dependencies
RUN apt update -y && rosdep update && \
  rosdep install --from-paths . --ignore-src -r -y && \
  rm -rf /var/lib/apt/lists/*

RUN /bin/sh -c ". /opt/ros/$ROS_DISTRO/setup.sh && \
  colcon build --symlink-install --event-handlers console_direct+ \
  --cmake-args '-DCMAKE_BUILD_TYPE=Release' \
  '-DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  '-DCMAKE_CXX_FLAGS=\"-Wl,--allow-shlib-undefined\"'"



# Set the default DDS middleware to FastRTPS for improved big data transmission
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENTRYPOINT ["/bin/sh", "-c", ". /opt/ros/$ROS_DISTRO/setup.sh && . /root/ros2_ws/install/setup.sh && \"$@\"", "-s" ]