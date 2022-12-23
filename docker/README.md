This folder contains a list of Dockerfile to build Docker images ready to start the ZED ROS2 Wrapper nodes:
* `Dockerfile.u22-cu117-humble-release`: desktop image for ROS2 Humble, on Ubuntu 22.04, CUDA 11.7. The ZED Wrapper is cloned from the master branch and built inside the image
* `Dockerfile.u22-cu117-humble-devel`: desktop image for ROS2 Humble, on Ubuntu 22.04, CUDA 11.7. The ZED Wrapper is copied from the sources of the current branch and built inside the image. Useful to test development branches before merging them.

## Build images

### Release image

Build:

```bash
docker build -t "<image_tag>" -f Dockerfile.u22-cu117-humble-release .
```
    
Run:

```
docker run -it --gpus 'all,"capabilities=compute,utility,video"' \
 --runtime nvidia --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
 <image_tag>
```
