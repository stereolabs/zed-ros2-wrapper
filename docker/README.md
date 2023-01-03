# Docker

This folder contains a list of Dockerfile to build Docker images ready to start the ZED ROS2 Wrapper nodes:

* `Dockerfile.u22-cu117-humble-release`: desktop image for ROS2 Humble, running on Ubuntu 22.04, with CUDA 11.7. The ZED Wrapper is cloned from the master branch and compiled.
* `Dockerfile.u22-cu117-humble-devel`: desktop image for ROS2 Humble, running on Ubuntu 22.04, with CUDA 11.7. The ZED Wrapper is copied from the source file of the current branch and compiled. This is useful to create a Docker image of a branch to be tested before merging it in the master branch.

## Build the Docker images

Choose a name for the image and replace `<image_tag>` with it, e.g. `zed-ubuntu22.04-cuda11.7-ros2-humble`

### Release image

#### Build

```bash
docker build -t "<image_tag>" -f Dockerfile.u22-cu117-humble-release .
```
    
#### Run

```bash
docker run -it --gpus 'all,"capabilities=compute,utility,video"' \
 --runtime nvidia --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
 <image_tag>
```

### Devel image

#### Build

Create a folder for the sources and copy the files:

```bash
mkdir -p ./tmp_sources
cp -r ../zed* ./tmp_sources
```

Build the image:

```bash
docker build -t "<image_tag>" -f Dockerfile.u22-cu117-humble-devel .
```

Remove the sources:

```bash
rm -r ./tmp_sources
```

#### Run

```bash
docker run -it --gpus 'all,"capabilities=compute,utility,video"' \
 --runtime nvidia --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
 <image_tag>
```
