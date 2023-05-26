<h1 align="center">
  ZED SDK for ROS 2
  <br>
</h1>
<p align="center">
  ROS2 Foxy Fitzroy (Ubuntu 20.04)
  ·
  ROS2 Humble Hawksbill (Ubuntu 22.04)
</p>

This project provides a comprehensive **ROS2 wrapper for the ZED** stereo cameras from Stereolabs, enabling all of the camera's functionalities within the ROS2 framework.

![](https://cdn2.stereolabs.com/docs/ros/images/PointCloud_Depth_ROS.jpg)

The ZED SDK ROS 2 Wrapper offers a complete set of features that enable users to seamlessly integrate the ZED stereo camera with ROS 2.These features include:

 - Left and right rectified/unrectified images
 - Depth data
 - Colored 3D point cloud
 - Position and Mapping (with GNSS data fusion)
 - Sensors data
 - Object detection
 - Person skeleton detection and tracking

## Installation

### Prerequisites

- [Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/focal/) or [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)
- [ZED SDK](https://www.stereolabs.com/developers/release/latest/) v4.0
- [CUDA](https://developer.nvidia.com/cuda-downloads) dependency
- ROS2 Foxy Fitxroy or ROS2 Humble Hawksbill: 
  - [Foxy on Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)
  - [Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html)

### Build the package

The **zed_ros2_wrapper** is a [colcon](http://design.ros2.org/articles/build_tool.html) package. 

**Note:** If you haven’t set up your colcon workspace yet, please follow this short [tutorial](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/). 

To install the **zed_ros2_wrapper**, open a bash terminal, clone the package from Github, and build it:

```bash
$ cd ~/ros2_ws/src/ #use your current ros2 workspace folder
$ git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
$ cd ..
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
$ echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
$ source ~/.bashrc
```

<details>
<summary>Troubleshooting</summary>

<br>
<p><strong>Note:</strong> If <code>rosdep</code> is missing you can install it with:</p>
<pre><code>
$ [sudo apt]-get install python-rosdep python-rosinstall-generator python-vcstool python-rosinstall build-essential
</code></pre>

<p><strong>Note:</strong> The option <code>--symlink-install</code> is very important, it allows to use symlinks instead of copying files to the ROS2 folders during the installation, where possible. Each package in ROS2 must be installed and all the files used by the nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without the needing to issue a new <code>colcon build</code> command. This is true only for all the files that don't need to be compiled (Python scripts, configurations, etc.).</p>

<p><strong>Note:</strong> If you are using a different console interface like zsh, you have to change the <code>source</code> command as follows: <code>echo source $(pwd)/install/local_setup.zsh &gt;&gt; ~/.zshrc</code> and <code>source ~/.zshrc</code>.</p>
</details>

#### Update the local repository

To update the repository to the latest release you must use the following command to retrieve the latest commits of `zed-ros2-wrapper` and of all the submodules:

```bash
$ git checkout master # if you are not on the main branch  
$ git pull --recurse-submodules # update recursively all the submodules
```

Remember to always clean the cache of your colcon workspace before compiling with the `colcon build` command to be sure that everything will work as expected:

```bash
$ cd <catkin_workspace_root>
$ rm -rf install
$ rm -rf build
$ rm -rf log
$ colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
```

## Starting the ZED node

To start the ZED node, open a terminal and use the [CLI](https://index.ros.org/doc/ros2/Tutorials/Introspection-with-command-line-tools/) command `ros2 launch`:

```bash
$ ros2 launch zed_wrapper {zed-model}.launch.py
```

> **Note**: `zed-model` can be one of the following: `zed`, `zedm`, `zed2`, `zed2i`, `zedx`, `zedxm`

The `{zed-model}.launch.py` scripts are Python launch scripts that automatically start the ZED node using ["manual composition"](https://index.ros.org/doc/ros2/Tutorials/Composition/), loading the parameters from the correct "YAML files" and creating the camera model from the correct "URDF file".

**Note:** You can set your own configurations modifying the parameters in the files **common.yaml**, **zed.yaml** **zedm.yaml**, **zed2.yaml**, **zed2i.yaml**, **zedx.yaml**, and **zedxm.yaml** available in the folder `zed_wrapper/config`.

## Documentation

For detailed documentation on how to use the ZED SDK ROS 2 Wrapper, including a full list of topics and parameters exposed by the wrapper, please visit our documentation website [here](https://www.stereolabs.com/docs/ros2/zed-node/).

## Examples and Tutorials
To learn more about how to use the ZED SDK ROS 2 Wrapper and integrate it, you can check out the ZED ROS 2 [examples and tutorials](https://github.com/stereolabs/zed-ros2-examples).

### Rviz2 visualization examples

 - Example launch files to start a preconfigured instance of Rviz displaying all the ZED Wrapper node information: [zed_display_rviz2](https://github.com/stereolabs/zed-ros2-examples/tree/master/zed_display_rviz2)
 - ROS2 plugin for ZED2 to visualize the results of the Object Detection and Body Tracking modules (bounding boxes and skeletons): [rviz-plugin-zed-od](https://github.com/stereolabs/zed-ros2-examples/tree/master/rviz-plugin-zed-od)

### Tutorials

 - [Images subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_video_tutorial)
 - [Depth subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_depth_tutorial)
 - [Pose/Odometry subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_pose_tutorial)
 - [ROS2 Composition + BGRA2BGR conversion tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_rgb_convert)


## Main Features
### Rviz visualization
Example launch files to start a pre-configured Rviz environment to visualize the data of `ZED`, `ZED Mini`, `ZED2`, `ZED2i`, `ZED-X`, and `ZED-X Mini` cameras are provided in the [`zed-ros2-examples` repository](https://github.com/stereolabs/zed-ros2-examples/tree/master/zed_display_rviz2)
    
### SVO recording
[SVO recording](https://www.stereolabs.com/docs/video/recording/) can be started and stopped while the ZED node is running using the service `start_svo_recording` and the service `stop_svo_recording`.
[More information](https://www.stereolabs.com/docs/ros2/zed_node/#services)

### Object Detection
The SDK v3.0 introduced the Object Detection and Tracking module. **The Object Detection module is not available for the ZED Camera**. 

The Object Detection can be enabled *automatically* when the node start by setting the parameter `object_detection/od_enabled` to `true` in the file `common.yaml`.

The Object Detection can be enabled/disabled *manually* by calling the services `enable_obj_det`.

### Body Tracking
The SDK v4.0 introduced the Body Tracking module (now separated from the Object Detection module). **The Body Tracking module is not available for the ZED Camera**. 

The Body Tracking can be enabled *automatically* when the node start by setting the parameter `body_tracking/bt_enabled` to `true` in the file `common.yaml`.

### Spatial Mapping
The Spatial Mapping can be enabled automatically when the node start setting the parameter `mapping/mapping_enabled` to `true` in the file `common.yaml`.
The Spatial Mapping can be enabled/disabled manually calling the services `enable_mapping`.

### GNSS fusion
The SDK v4.0 introduced GNSS fusion. The ZED ROS2 Wrapper can subscribe to a `NavSatFix` topic and fuse GNSS data information
with Positional Tracking information to obtain a precise robot localization referred to Earth coordinates.
To enable GNSS fusion set the parameter `gnss_fusion.gnss_fusion_enabled` to `true`.
It is important that you set the correct `gnss_frame` parameter when launching the node, e.g. `gnss_frame:='gnss_link'`.
The services `toLL` and `fromLL` can be used to convert Latitude/Longitude coordinates to robot `map` coordinates.

### 2D mode
For robots moving on a planar surface it is possible to activate the "2D mode" (parameter `pos_tracking/two_d_mode` in `common.yaml`). 
The value of the coordinate Z for odometry and pose will have a fixed value (parameter `pos_tracking/fixed_z_value` in `common.yaml`). 
Roll and pitch and relative velocities will be fixed to zero.