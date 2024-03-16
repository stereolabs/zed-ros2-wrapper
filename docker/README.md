# Docker

This folder contains a list of Dockerfile files to build Docker images ready to start the nodes of the *ZED ROS2 Wrapper*:

* `Dockerfile.u22-cu117-humble-release`: desktop image for ROS2 Humble, running on Ubuntu 22.04, with CUDA 11.7. The ZED Wrapper is cloned from the master branch and compiled.
* `Dockerfile.u22-cu117-humble-devel`: development desktop image for ROS2 Humble, running on Ubuntu 22.04, with CUDA 11.7. The ZED Wrapper is copied from the source file of the current branch and compiled. This is useful to create a Docker image of a branch to be tested before merging it in the master branch.

## Build the Docker images

Choose a name for the image and replace `<image_tag>` with it, e.g. `zed-ubuntu22.04-cuda11.7-ros2-humble`

**Note:** You can find the Docker images pre-built for the latest version of the `master` branch in the Docker hub [stereolabs/zedbot](https://hub.docker.com/r/stereolabs/zedbot).

### Release image

The Release image internally clones the master branch of this repository to build the ZED ROS2 Wrapper code.

Build the image for desktop:
```bash
docker build -t "<image_tag>" -f Dockerfile.u22-cu117-humble-release .
```

or build the image for Jetson:

```bash
docker build -t "<image_tag>" -f Dockerfile.l4t35_1-humble-release .
```

### Devel image

The devel image internally includes the source code of the current branch, so you can modify it. For this reason we must first copy the source to a temporary folder reachable while building the Docker image. The folder can be removed when the Docker image is ready.

Create a temporary `tmp_sources` folder for the sources and copy the files:

```bash
mkdir -p ./tmp_sources
cp -r ../zed* ./tmp_sources
```

Build the image for desktop:

```bash
docker build -t "<image_tag>" -f Dockerfile.u22-cu117-humble-devel .
```

or build the image for Jetson:

```bash
docker build -t "<image_tag>" -f Dockerfile.l4t35_1-humble-devel .
```

Remove the temporary sources to avoid future compiling issues:

```bash
rm -r ./tmp_sources
```

**Note:** it is important that the name of the temporary folder is `tmp_sources` because it is used in the Dockerfile we provide.

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
