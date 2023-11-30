<h1 align="center">
   <img src="./images/Picto+STEREOLABS_Black.jpg" alt="Stereolabs" title="Stereolabs" /><br \>
   ROS2 wrapper
</h1>

<p align="center">
  ROS2 packages for using Stereolabs ZED Camera cameras.<br>
  ROS2 Foxy Fitzroy (Ubuntu 20.04) - ROS2 Humble Hawksbill (Ubuntu 22.04)
</p>

<hr>

This package lets you use the ZED stereo cameras with ROS2. It provides access to the following data:

  - Left and right rectified/unrectified images
  - Depth data
  - Colored 3D point cloud
  - Position and Mapping (with GNSS data fusion)
  - Sensors data (not available with ZED)
  - Detected objects (not available with ZED)
  - Persons skeleton (not available with ZED)

[More information](https://www.stereolabs.com/docs/ros2/getting-started/)

![](https://cdn.stereolabs.com/docs/ros/images/PointCloud_Depth_ROS.jpg)

## Installation

### Prerequisites

- [Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/focal/) or [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)
- [ZED SDK](https://www.stereolabs.com/developers/release/latest/) v4.0.7 (for older versions support please check the [releases](https://github.com/stereolabs/zed-ros2-wrapper/releases))
- [CUDA](https://developer.nvidia.com/cuda-downloads) dependency
- ROS2 Foxy Fitxroy or ROS2 Humble Hawksbill: 
  - [Foxy on Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)
  - [Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html)

### Build the package

The **zed_ros2_wrapper** is a [colcon](http://design.ros2.org/articles/build_tool.html) package. 

**Note:** If you haven’t set up your colcon workspace yet, please follow this short [tutorial](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/). 

To install the **zed_ros2_wrapper**, open a bash terminal, clone the package from Github, and build it:

```bash
mkdir -p ~/ros2_ws/src/ # create your workspace if it does not exist
cd ~/ros2_ws/src/ #use your current ros2 workspace folder
git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
sudo apt update
rosdep install --from-paths src --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc # automatically source the installation in every new bash (optional)
source ~/.bashrc
```

**Note:** If `rosdep` is missing you can install it with:

  ```sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool python3-rosinstall build-essential```

**Note:** The option `--symlink-install` is very important, it allows to use symlinks instead of copying files to the ROS2 folders during the installation, where possible. Each package in ROS2 must be installed and all the files used by the nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without the needing to issue a new `colcon build` command. This is true only for all the files that don't need to be compiled (Python scripts, configurations, etc.).

**Note:** If you are using a different console interface like zsh, you have to change the `source` command as follows: `echo source $(pwd)/install/local_setup.zsh >> ~/.zshrc` and `source ~/.zshrc`.

## Starting the ZED node

To start the ZED node, open a bash terminal and use the [CLI](https://index.ros.org/doc/ros2/Tutorials/Introspection-with-command-line-tools/) command `ros2 launch`:

```bash
$ ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
```

Replace `<camera_model>` with the model of the camera that you are using: `'zed'`, `'zedm'`, `'zed2'`, `'zed2i'`, `'zedx'`, `'zedxm'`.

The `zed_camera.launch.py` is a Python launch scripts that automatically start the ZED node using ["manual composition"](https://index.ros.org/doc/ros2/Tutorials/Composition/). The parameters for the indicated camera model are loaded from the relative "YAML files".
A Robot State Publisher node is started to publish the camera static links and joints loaded from the URDF model associated with the camera model.

**Note:** You can set your own configurations modifying the parameters in the files **common.yaml**, **zed.yaml** **zedm.yaml**, **zed2.yaml**, **zed2i.yaml**, **zedx.yaml**, and **zedxm.yaml** available in the folder `zed_wrapper/config`.

You can get the list of all the available launch parameters by using the `-s` launch option:

```bash
$ ros2 launch zed_wrapper zed_camera.launch.py -s
$ ros2 launch zed_display_rviz2 display_zed_cam.launch.py -s
```

For full descriptions of each parameter, follow the complete guide [here](https://www.stereolabs.com/docs/ros2/zed_node#configuration-parameters).

### Rviz visualization

To start a pre-configured Rviz environment and visualize the data of all ZED cameras, we provide in the [`zed-ros2-examples` repository](https://github.com/stereolabs/zed-ros2-examples/tree/master/zed_display_rviz2). You'll see there more advanced examples and visualisation that demonstrate depth, point clouds, odometry, object detection, etc.

You can also quickly check that your depth data is correctly retrieved in rviz with `rviz2 -d ./zed_wrapper/config/rviz2/<your camera model>.rviz`. Be aware that rviz subscribes to numerous ROS topics, which can potentially impact the performance of your application compared to when it runs without rviz.

### Simulation mode

Launch a standalone ZED ROS 2 node with simulated ZED data as input by using the following command:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx sim_mode:=true
```

Launch options: 
* [Mandatory] `camera_model`: indicates the model of the simulated camera. It's required that this parameter matches the model of the simulated camera. In most case it will be a ZED X, since the first versions of the simulation plugins that we released are simulating this type of device.
* [Mandatory] `sim_mode`: start the ZED node in simulation mode if `true`.
* [Optional] `use_sim_time`: force the node to wait for valid messages on the topic `/clock`, and so use the simulation clock as time reference.
* [Optional] `sim_address`: set the address of the simulation server. Default is `127.0.0.1` and it's valid if the node runs on the same machine as the simulator.
* [Optional] `sim_port`: set the port of the simulation server. It must match the value of the field `Streaming Port` of the properties of the `ZED camera streamer` Action Graph node. A different `Streaming Port` value for each camera is required in multi-camera simulations.

You can also start a preconfigured instance of `rviz2` to visualize all the information available in simulation by using the command:

```bash
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx sim_mode:=true
```

the `display_zed_cam.launch.py` launch file includes the `zed_camera.launch.py` launch file, so it provides the same parameters.

Here's an example of `rviz2` running with the simulated information obtained by placing the ZED camera on a shelf of a simulated warehouse:

![](./images/sim_rviz.jpg)

![](./images/zed_shelves.jpg)

Supported simulation environments:

* [NVIDIA Omniverse Isaac Sim](https://www.stereolabs.com/docs/isaac-sim/)

## More features

### SVO recording

[SVO recording](https://www.stereolabs.com/docs/video/recording/) can be started and stopped while the ZED node is running using the service `start_svo_recording` and the service `stop_svo_recording`.
[More information](https://www.stereolabs.com/docs/ros2/zed_node/#services)

### Object Detection

The Object Detection can be enabled *automatically* when the node start by setting the parameter `object_detection/od_enabled` to `true` in the file `common.yaml`.
The Object Detection can be enabled/disabled *manually* by calling the services `enable_obj_det`.


### Body Tracking

The Body Tracking can be enabled *automatically* when the node start by setting the parameter `body_tracking/bt_enabled` to `true` in the file `common.yaml`.

*The Object Detection module is not available on the very first generation of ZED cameras.*

### Spatial Mapping

The Spatial Mapping can be enabled automatically when the node start setting the parameter `mapping/mapping_enabled` to `true` in the file `common.yaml`.
The Spatial Mapping can be enabled/disabled manually calling the services `enable_mapping`.

### GNSS fusion

The ZED ROS2 Wrapper can subscribe to a `NavSatFix` topic and fuse GNSS data information
with Positional Tracking information to obtain a precise robot localization referred to Earth coordinates.
To enable GNSS fusion set the parameter `gnss_fusion.gnss_fusion_enabled` to `true`.
It is important that you set the correct `gnss_frame` parameter when launching the node, e.g. `gnss_frame:='gnss_link'`.
The services `toLL` and `fromLL` can be used to convert Latitude/Longitude coordinates to robot `map` coordinates.

### 2D mode

For robots moving on a planar surface it is possible to activate the "2D mode" (parameter `pos_tracking/two_d_mode` in `common.yaml`). 
The value of the coordinate Z for odometry and pose will have a fixed value (parameter `pos_tracking/fixed_z_value` in `common.yaml`). 
Roll and pitch and relative velocities will be fixed to zero.

## Examples and Tutorials

Examples and tutorials are provided to better understand how to use the ZED wrapper and how to integrate it in the ROS2 framework.
See the [`zed-ros2-examples` repository](https://github.com/stereolabs/zed-ros2-examples)

### Rviz2 visualization examples

 - Example launch files to start a preconfigured instance of Rviz displaying all the ZED Wrapper node information: [zed_display_rviz2](https://github.com/stereolabs/zed-ros2-examples/tree/master/zed_display_rviz2)
 - ROS2 plugin for ZED2 to visualize the results of the Object Detection and Body Tracking modules (bounding boxes and skeletons): [rviz-plugin-zed-od](https://github.com/stereolabs/zed-ros2-examples/tree/master/rviz-plugin-zed-od)

### Tutorials

 - [Images subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_video_tutorial)
 - [Depth subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_depth_tutorial)
 - [Pose/Odometry subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_pose_tutorial)
 - [ROS2 Composition + BGRA2BGR conversion tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_rgb_convert)

## Update the local repository

To update the repository to the latest release, use the following command that will retrieve the latest commits of `zed-ros2-wrapper` and of all the submodules:

```bash
git checkout master # if you are not on the main branch  
git pull --recurse-submodules # update recursively all the submodules
```

Clean the cache of your colcon workspace before compiling with the `colcon build` command to be sure that everything will work as expected:

```bash
cd <ros2_workspace_root> # replace with your workspace folder, for example ~/ros2_ws/src/
rm -r install
rm -r build
rm -r log
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
```

## Known issues

### [ROS2 Foxy] Image Transport and topic subscriptions

There is an **IMPORTANT** issue in ROS2 Foxy with the function `CameraPublisher::getNumSubscribers` preventing the correct counting of the number of nodes subscribing one of the topics published by an `image_transport::CameraPublisher` object and hence stopping the correct publishing of the subscribed topics.

The only known solution is to install the exact version [v3.0.0](https://github.com/ros-perception/image_common/releases/tag/3.0.0) of the `image_transport` package, published on 2021-05-26, that contains the fix for this issue.

To install the working version from the sources:

```bash
cd <colcon_workspace>/src # Access the source folder of your colcon workspace
git clone https://github.com/ros-perception/image_common.git --branch 3.0.0 --single-branch # clone the "v3.0.0" branch of the "image_common" repository
cd <colcon_workspace> # Go back to the root of your colcon workspace
colcon build --symlink-install # Compile everything and install
```
Close the console and re-open it to apply the modifications.

### [ROS2 Foxy] Image Transport Plugins and compressed topics

The `image_transport_plugins` package is not correctly working with ROS2 Foxy (see [here](https://github.com/stereolabs/zed-ros2-wrapper/issues/31), [here](https://github.com/ros-perception/image_common/issues/184), [here](https://github.com/stereolabs/zed-ros2-wrapper/issues/31), and [here](https://github.com/ros-perception/image_transport_plugins/pull/58)). We suggest you remove it to avoid many annoying warning messages until the ROS2 developers do not fix it or we find a workaround:

```
sudo apt remove ros-foxy-image-transport-plugins ros-foxy-compressed-depth-image-transport ros-foxy-compressed-image-transport
```