# Docker

This folder contains a list of Dockerfile files to build Docker images ready to start the nodes of the *ZED ROS2 Wrapper*:

* `Dockerfile.desktop-humble`: development desktop image for ROS 2 Humble, running on the specified Ubuntu and CUDA versions. The ZED Wrapper is copied from the source file of the current branch and compiled.
* `Dockerfile.l4t-humble`: Jetson image for ROS 2 Humble, running on the given L4T version (L4T35.4 by default).

> :pushpin: **NOTE:** in the entrypoint files we set the value of the `ROS_DOMAIN_ID` environment
> variable to `0` that is the default value in ROS 2.
>
> If your setup requires a different value you can change it in the `ros_entrypoint_jetson.sh` and
> `ros_entrypoint.sh` file before building your image to set it automatically when starting your Docker image,
> or you can use the CLI command `export ROS_DOMAIN_ID=<new_value>` when each interactive session is started.
>
> You can get more details concerning the `ROS_DOMAIN_ID` usage on the [official ROS 2 documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html#the-ros-domain-id).

## Cross compilation

You can easily compile the image for Jetson from your usual Desktop PC.
For that you just need to run the following line before launching the build command:

```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

## Build the Docker images

We provide a script to build your image with the right L4T / ZED SDK version.

* Checkout the tag or commit of the ROS2 wrapper that you need.

 ```bash
 git checkout <build_branch>
 ```

  e.g. to build the master branch:

 ```bash
 git checkout master
 ```

* Build the image for **Jetson**:

 ```bash
 ./jetson_build_dockerfile_from_sdk_and_l4T_version.sh <l4T version> <ZED SDK version>
 ```

* Build the image for **Desktop**:

 ```bash
 ./desktop_build_dockerfile_from_sdk_ubuntu_and_cuda_version.sh <Ubuntu version> <CUDA version> <ZED SDK version>
 ```

Examples:

```bash
# Jetson with JP6.0 and ZED SDK v4.2.5
./jetson_build_dockerfile_from_sdk_and_l4T_version.sh l4t-r36.3.0 zedsdk-4.2.5
```

```bash
# Jetson with JP6.2 and ZED SDK v5.0.0
./jetson_build_dockerfile_from_sdk_and_l4T_version.sh l4t-r36.4.0 zedsdk-5.0.0
```

```bash
# Desktop on Ubuntu 22.04m CUDA 12.6.3 and ZED SDK v4.2.5
./desktop_build_dockerfile_from_sdk_ubuntu_and_cuda_version.sh ubuntu-22.04 cuda-12.6.3 zedsdk-4.2.5
```

> :warning: Some configurations will not work. For example, if a specific ZED SDK does not exist for a given Ubuntu/CUDA/L4T version, or if the given ROS 2 wrapper is not compatible with the selected Ubuntu version.

## Run the Docker image

### NVIDIA runtime

NVIDIA drivers must be accessible from the Docker image to run the ZED SDK code on the GPU. You'll need :

* The `nvidia` container runtime installed, following [this guide](https://www.stereolabs.com/docs/docker/install-guide-linux/#nvidia-docker)
* A specific docker runtime environment with `-gpus all` or `-e NVIDIA_DRIVER_CAPABILITIES=all`
* Docker privileged mode with `--privileged`

### Network

Setup the network configuration to enable the communication between the Docker image, other Docker images, and the host:

* `--network=host`: Remove network isolation between the container and the Docker host
* `--ipc=host`: Use the host system's Inter-Process Communication namespace
* `--pid=host`: Use the host system's namespace for process ID

### Display context to use CUDA based applications

Use the same host `DISPLAY` environment variable in every Docker image to enable CUDA-based applications with `-e DISPLAY=$DISPLAY`.

> :pushpin: **NOTE**: the shared volume `/tmp/.X11-unix/:/tmp/.X11-unix` is also required.

### Volumes

A few volumes should also be shared with the host.

* `/tmp/.X11-unix/:/tmp/.X11-unix` is required to enable X11 server communication for CUDA-based applications
* `/usr/local/zed/settings:/usr/local/zed/settings` if you plan to use the robot in an Internet-negated area, and you previously downloaded the camera calibration files by following [this guide](https://support.stereolabs.com/hc/en-us/articles/21614848880791-How-can-I-use-the-ZED-with-Docker-on-a-robot-with-no-internet-connection). 
* `/usr/local/zed/resources:/usr/local/zed/resources` if you plan to use the AI module of the ZED SDK (Object Detection, Skeleton Tracking, NEURAL depth) we suggest binding mounting a folder to avoid downloading and optimizing the AI models each time the Docker image is restarted. The first time you use the AI model inside the Docker image, it will be downloaded and optimized in the local bound-mounted folder, and stored there for the next runs.
  * If you plan to use different SDK versions in different Docker images it's preferred to use a different
    volume on the host for each of them: `/<specific_folder_name>/:/usr/local/zed/resources`
* `/dev:/dev` to share the video devices
* For GMSL2 cameras (ZED X, ZED X One) you'll also need
  * `/tmp:/tmp`
  * `/var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/`
  * `/etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service`
* `/dev:/dev`: to share the video and other required devices
  * `/dev/shm:/dev/shm`: to use ROS 2 with shared memory

### Start the Docker container

First of all, allow the container to access EGL display resources (required only once):

```bash
sudo xhost +si:localuser:root
```

then you can start an interactive session:

#### USB3 cameras

```bash
docker run --runtime nvidia -it --privileged --network=host --ipc=host --pid=host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix/:/tmp/.X11-unix \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  -v /usr/local/zed/resources/:/usr/local/zed/resources/ \
  -v /usr/local/zed/settings/:/usr/local/zed/settings/ \
  <docker_image_tag>
```

#### GMSL cameras

```bash
docker run --runtime nvidia -it --privileged --network=host --ipc=host --pid=host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY \
  -v /tmp:/tmp \
  -v /dev:/dev \
  -v /var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/ \
  -v /etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service \
  -v /usr/local/zed/resources/:/usr/local/zed/resources/ \
  -v /usr/local/zed/settings/:/usr/local/zed/settings/ \
  <docker_image_tag>
```
