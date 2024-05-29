# Docker

This folder contains a list of Dockerfile files to build Docker images ready to start the nodes of the *ZED ROS2 Wrapper*:

* `Dockerfile.desktop-humble`: development desktop image for ROS2 Humble, running on the specified Ubuntu and CUDA versions. The ZED Wrapper is copied from the source file of the current branch and compiled.
* `Dockerfile.l4t-humble`: Jetson image for ROS2 Humble, running on the given L4T version (L4T35.4 by default).

### Cross compilation
You can easily compile the image for jetson from your usual Desktop PC. For that you just need to run the following line before:
```
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

## Build the Docker images

We provide a script to build your image with the right L4T / ZED SDK version.
- Checkout the tag or commit of the ROS2 wrapper that you need.
```
git checkout 4.1.2
```
You can also modify the sources.
- Build the image
```
./jetson_build_dockerfile_from_sdk_and_l4T_version.sh <l4T version> <ZED SDK version>
```
or
```
./desktop_build_dockerfile_from_sdk_ubuntu_and_cuda_version.sh <Ubuntu version> <CUDA version> <ZED SDK version>
```
Examples:
```
./jetson_build_dockerfile_from_sdk_and_l4T_version.sh l4t-r35.4.1 zedsdk4.1.2
./desktop_build_dockerfile_from_sdk_ubuntu_and_cuda_version.sh ubuntu22.04 cuda12.1.0 zedsdk4.1.2
```
That will produce the image `zed_ros2_l4t_image` or `zed_ros2_desktop_image`.
Some configuration will not work (for example, if a specific ZED SDK does not exist for a given Ubuntu/CUDA/L4T version, or if the given ros2 wrapper is not compatible with this version)

## Run the Docker image

### NVIDIA runtime
NVIDIA drivers must be accessible from the Docker image to run the ZED SDK code on the GPU. You'll need :

- The `nvidia` container runtime installed, following [this guide](https://www.stereolabs.com/docs/docker/install-guide-linux/#nvidia-docker)
- A specific docker runtime environment with `-gpus all` or `-e NVIDIA_DRIVER_CAPABILITIES=all`
- Docker privileged mode with `--privileged`


### Volumes
A few volumes should also be shared with the host.
- `/usr/local/zed/settings:/usr/local/zed/settings` if you plan to use the robot in an Internet-negated area, and you previously downloaded the camera calibration files by following [this guide](https://support.stereolabs.com/hc/en-us/articles/21614848880791-How-can-I-use-the-ZED-with-Docker-on-a-robot-with-no-internet-connection). 
- `/usr/local/zed/resources:/usr/local/zed/resources` if you plan to use the AI module of the ZED SDK (Object Detection, Skeleton Tracking, NEURAL depth) we suggest binding mounting a folder to avoid downloading and optimizing the AI models each time the Docker image is restarted. The first time you use the AI model inside the Docker image, it will be downloaded and optimized in the local bound-mounted folder, and stored there for the next runs.
- `/dev:/dev` to share the video devices
- For GMSL cameras (ZED X) you'll also need
  - `/tmp:/tmp`
  - `/var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/`
  - `/etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service` 

### Start the Docker container

The following command starts an interactive session:

```bash
docker run --runtime nvidia -it --privileged --ipc=host --pid=host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY \
  -v /dev:/dev -v /tmp/.X11-unix/:/tmp/.X11-unix \
  -v ${HOME}/zed_docker_ai/:/usr/local/zed/resources/ \
  <image_tag>
```

For GMSL cameras

```bash
docker run --runtime nvidia -it --privileged --ipc=host --pid=host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY \
  -v /dev:/dev \
  -v /tmp:/tmp \
  -v /var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/ \
  -v /etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service \
  -v ${HOME}/zed_docker_ai/:/usr/local/zed/resources/ \
  <image_tag>
```
