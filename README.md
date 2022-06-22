![](./images/Picto+STEREOLABS_Black.jpg)

# Stereolabs ZED Camera - ROS2 Foxy Fitzroy (Ubuntu 20.04)

#### **Note:** if you are searching for a version of the ROS2 wrapper running on an Nvidia Jetson based on Ubuntu 18.04 that does not require recompiling ROS2 from source, please check out the `eloquent` branch, the official ROS2 version running on Ubuntu 18.04.

This package lets you use the ZED stereo cameras with ROS2. It provides access to the following data:

  - Left and right rectified/unrectified images
  - Depth data
  - Colored 3D point cloud
  - Position and Mapping
  - Sensors data (not available with ZED)
  - Detected objects (not available with ZED)
  - Persons skeleton (not available with ZED)

[More information](https://www.stereolabs.com/docs/ros2/getting-started/)

![](https://cdn.stereolabs.com/docs/ros/images/PointCloud_Depth_ROS.jpg)

## Known issues

### Image Transport and topic subscriptions

There is an **IMPORTANT** issue with the function `CameraPublisher::getNumSubscribers` preventing the correct counting of the number of nodes subscribing one of the topics published by an `image_transport::CameraPublisher` object and hence stopping the correct publishing of the subscribed topics.

The only known solution is to install the exact version [v3.0.0](https://github.com/ros-perception/image_common/releases/tag/3.0.0) of the `image_transport` package, published on 2021-05-26, that contains the fix for this issue.

To install the working version from the sources:

    $ cd <colcon_workspace>/src # Access the source folder of your colcon workspace
    $ git clone https://github.com/ros-perception/image_common.git --branch 3.0.0 --single-branch # clone the "v3.0.0" branch of the "image_common" repository
    $ cd <colcon_workspace> # Go back to the root of your colcon workspace
    $ colcon build --symlink-install # Compile everything and install

Close the console and re-open it to apply the modifications.

### Image Transport Plugins and compressed topics

The `image_transport_plugins` package is not correctly working with ROS2 Foxy (see [here](https://github.com/stereolabs/zed-ros2-wrapper/issues/31), [here](https://github.com/ros-perception/image_common/issues/184), [here](https://github.com/stereolabs/zed-ros2-wrapper/issues/31), and [here](https://github.com/ros-perception/image_transport_plugins/pull/58)). We suggest you remove it to avoid many annoying warning messages until the ROS2 developers do not fix it or we find a workaround:

```
$ sudo apt remove ros-foxy-image-transport-plugins ros-foxy-compressed-depth-image-transport ros-foxy-compressed-image-transport
```

## Installation

### Prerequisites

- [Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/focal/)
- [ZED SDK](https://www.stereolabs.com/developers/release/latest/) v3.7
- [CUDA](https://developer.nvidia.com/cuda-downloads) dependency
- ROS2 Foxy Fitxroy: 
  - [Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)

### Build the package

The **zed_ros2_wrapper** is a [colcon](http://design.ros2.org/articles/build_tool.html) package. 

**Note:** If you haven’t set up your colcon workspace yet, please follow this short [tutorial](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/). 

To install the **zed_ros2_wrapper**, open a bash terminal, clone the package from Github, and build it:

```bash
$ cd ~/ros2_ws/src/ #use your current ros2 workspace folder
$ git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
$ cd ..
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
$ echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
$ source ~/.bashrc
```

**Note:** If `rosdep` is missing you can install it with:

  ```$ sudo apt-get install python-rosdep python-rosinstall-generator python-vcstool python-rosinstall build-essential```

**Note:** The option `--symlink-install` is very important, it allows to use symlinks instead of copying files to the ROS2 folders during the installation, where possible. Each package in ROS2 must be installed and all the files used by the nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without the needing to issue a new `colcon build` command. This is true only for all the files that don't need to be compiled (Python scripts, configurations, etc.).

**Note:** If you are using a different console interface like zsh, you have to change the `source` command as follows: `echo source $(pwd)/install/local_setup.zsh >> ~/.zshrc` and `source ~/.zshrc`.

#### Update the local repository

To update the repository to the latest release you must use the following command to retrieve the latest commits of `zed-ros2-wrapper` and of all the submodules:

    $ git checkout master # if you are not on the main branch  
    $ git pull --recurse-submodules # update recursively all the submodules

Remember to always clean the cache of your colcon workspace before compiling with the `colcon build` command to be sure that everything will work as expected:

    $ cd <catkin_workspace_root>
    $ rm -rf install
    $ rm -rf build
    $ rm -rf log
    $ colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

## Starting the ZED node

To start the ZED node, open a terminal and use the [CLI](https://index.ros.org/doc/ros2/Tutorials/Introspection-with-command-line-tools/) command `ros2 launch`:

ZED:
```bash
$ ros2 launch zed_wrapper zed.launch.py
```

ZED Mini:
```bash
$ ros2 launch zed_wrapper zedm.launch.py
```

ZED 2:
```bash
$ ros2 launch zed_wrapper zed2.launch.py
```

ZED 2i:
```bash
$ ros2 launch zed_wrapper zed2i.launch.py
```

The `zed.launch.py`, `zedm.launch.py`, `zed2.launch.py` and `zed2i.launch.py` are three Python scripts that automatically start the ZED node using ["manual composition"](https://index.ros.org/doc/ros2/Tutorials/Composition/), loading the parameters from the correct "YAML files" and creating the camera model from the correct "URDF file".

**Note:** You can set your own configurations modifying the parameters in the files **common.yaml**, **zed.yaml** **zedm.yaml**, **zed2.yaml** and **zed2i.yaml** available in the folder `zed_wrapper/config`.
For full descriptions of each parameter, follow the complete guide [here](https://www.stereolabs.com/docs/ros2/zed_node#configuration-parameters).

### Rviz visualization
Example launch files to start a pre-configured Rviz environment to visualize the data of ZED, ZED Mini, ZED2, and ZED2i cameras are provided in the [`zed-ros2-examples` repository](https://github.com/stereolabs/zed-ros2-examples/tree/master/zed_display_rviz2)
    
### SVO recording
[SVO recording](https://www.stereolabs.com/docs/video/recording/) can be started and stopped while the ZED node is running using the service `start_svo_recording` and the service `stop_svo_recording`.
[More information](https://www.stereolabs.com/docs/ros2/zed_node/#services)

### Object Detection
The SDK v3.0 introduces the Object Detection and Tracking module. **The Object Detection module is available only with a ZED 2 or ZED 2i camera**. 

The Object Detection can be enabled *automatically* when the node start setting the parameter `object_detection/od_enabled` to `true` in the file `zed2.yaml` or `zed2i.yaml`.

The Object Detection can be enabled/disabled *manually* calling the services `enable_obj_det`.

### Spatial Mapping
The Spatial Mapping can be enabled automatically when the node start setting the parameter `mapping/mapping_enabled` to `true` in the file `common.yaml`.
The Spatial Mapping can be enabled/disabled manually calling the services `enable_mapping`.

### 2D mode
For robots moving on a planar surface it is possible to activate the "2D mode" (parameter `pos_tracking/two_d_mode` in `common.yaml`). 
The value of the coordinate Z for odometry and pose will have a fixed value (parameter `pos_tracking/fixed_z_value` in `common.yaml`). 
Roll and pitch and relative velocities will be fixed to zero.

## Examples and Tutorials
Examples and tutorials are provided to better understand how to use the ZED wrapper and how to integrate it in the ROS2 framework.
See the [`zed-ros2-examples` repository](https://github.com/stereolabs/zed-ros2-examples)

### Rviz2 visualization examples

 - Example launch files to start a preconfigured instance of Rviz displaying all the ZED Wrapper node information: [zed_display_rviz2](https://github.com/stereolabs/zed-ros2-examples/tree/master/zed_display_rviz2)
 - ROS2 plugin for ZED2 to visualize the results of the Object Detection module (bounding boxes and skeletons): [rviz-plugin-zed-od](https://github.com/stereolabs/zed-ros2-examples/tree/master/rviz-plugin-zed-od)

### Tutorials

 - [Images subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_video_tutorial)
 - [Depth subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_depth_tutorial)
 - [Pose/Odometry subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_pose_tutorial)
 - [ROS2 Composition + BGRA2BGR conversion tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_rgb_convert)
