![](./images/Picto+STEREOLABS_Black.jpg)

# Stereolabs ZED Camera - ROS2 Integration - Beta Version

This package lets you use the ZED stereo camera with ROS2. It provides access to the following data:

  - Left and right rectified/unrectified images
  - Depth data
  - Colored 3D point cloud
  - IMU data

[More information](https://www.stereolabs.com/docs/ros2/getting-started/)

![](https://cdn.stereolabs.com/docs/ros/images/PointCloud_Depth_ROS.jpg)

## Installation

### Prerequisites

- Ubuntu 16.04 or Ubuntu 18.04 (*support for Windows 10 will be provided soon*)
- [ZED SDK](https://www.stereolabs.com/developers/release/latest/) v2.6 or later
- [CUDA](https://developer.nvidia.com/cuda-downloads) dependency
- ROS2 Crystal: 
  - Ubuntu 16.04 [[source](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/)] 
  - Ubuntu 18.04 [[binaries](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/) - [source](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/)]

### Build the package

The **zed_ros2_wrapper** is a [colcon](http://design.ros2.org/articles/build_tool.html) package. It depends on the following ROS2 packages:

  - ament_cmake
  - ament_index_cpp
  - class_loader
  - lifecycle_msgs
  - rclcpp_lifecycle
  - sensor_msgs
  - tf2
  - tf2_ros
  - tf2_geometry_msgs
  - nav_msgs
  - stereo_msgs
  - urdf
  - robot_state_publisher
  - message_runtime
  - image_transport

**Note:** If you haven’t set up your colcon workspace yet, please follow this short [tutorial](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/). 

To install the **zed_ros2_wrapper**, open a bash terminal, clone the package from Github, and build it.

It is very important to follow the correct compiling order to guarantee that all the dependencies are respected:

```bash
$ cd ~/ros2_ws/src/ #use your current ros2 workspace folder
$ git clone https://github.com/stereolabs/zed-ros2-wrapper.git
$ cd ..
$ colcon build --symlink-install --packages-select stereolabs_zed_interfaces --cmake-args=-DCMAKE_BUILD_TYPE=Release
$ source ./install/local_setup.bash
$ colcon build --symlink-install --packages-select stereolabs_zed --cmake-args=-DCMAKE_BUILD_TYPE=Release
$ source ./install/local_setup.bash
$ colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
$ echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
$ source ~/.bashrc
```

**Note:** The option `--symlink-install` is very important, it allows to use symlinks instead of copying files to the ROS2 folders during the installation, where possible. Each package in ROS2 must be installed and all the files used by the nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without the needing to issue a new `colcon build` command. This is true only for all the files that don't need to be compiled (Python scripts, configurations, etc.).

**Note:** If you are using a different console interface like zsh, you have to change the `source` command as follows: `echo source $(pwd)/install/local_setup.zsh >> ~/.zshrc` and `source ~/.zshrc`.

**Error:** If an error mentioning `/usr/lib/x86_64-linux-gnu/libEGL.so` blocks compilation, use the following command to repair the libEGl symlink before restarting the `colcon` command:

```
#Only on libEGL error
$ sudo rm /usr/lib/x86_64-linux-gnu/libEGL.so; sudo ln /usr/lib/x86_64-linux-gnu/libEGL.so.1 /usr/lib/x86_64-linux-gnu/libEGL.so
```

## Starting the ZED node

The ZED is available in ROS2 as a [lifecycle managed node](https://index.ros.org/doc/ros2/Tutorials/Managed-Nodes/) that publishes its data to topics. You can get the full list of the available topics [here](https://www.stereolabs.com/docs/ros2/zed_node/#published-topics).

To start the ZED node, open a terminal and use the [CLI](https://index.ros.org/doc/ros2/Tutorials/Introspection-with-command-line-tools/) command `ros2 launch`:

```bash
$ ros2 launch stereolabs_zed zed.launch.py
```

The `zed.launch.py` is a Python launch script that automatically manages the lifecycle state transitions of the ZED ROS2 node. You can run the `zed_unmanaged.launch.py` launch script if you want to manually control the state of the node. For a full guide about manually managing the lifecycle states of the ZED ROS2 node, please follow the [lifecycle tutorial](https://www.stereolabs.com/docs/ros2/lifecycle/)

**Note:** You can set your own configurations modifying the parameters in the files **common.yaml**, **zed.yaml** and **zedm.yaml** available in the folder `zed_wrapper/config`.
For full descriptions of each parameter, follow the complete guide [here](https://www.stereolabs.com/docs/ros2/zed_node#configuration-parameters).

## Displaying ZED data

### Using RVIZ2

RVIZ2 is a useful visualization tool in ROS2. Using RVIZ2, you can visualize the ZED left and right images, the depth image and the 3D colored point cloud.

Launch the ZED wrapper along with RVIZ using the following command:

```bash
$ ros2 launch zed_rviz display_zed.launch.py
```
If you are using a ZED Mini camera:

```bash
$ roslaunch zed_rviz display_zedm.launch.py
```
**Note:** If you haven't yet configured your own RVIZ interface, you can find a detailed tutorial [here](https://www.stereolabs.com/docs/ros2/rviz2/).

### Displaying Images
The ZED node publishes both original and stereo rectified (aligned) left and right images. In RVIZ, select a topic and use the `image` preview mode. 

Here is the list of the available image topics:

  - **zed/zed_node/rgb/image_rect_color**: Color rectified image (left image by default)
  - **zed/zed_node/rgb/image_raw_color**: Color unrectified image (left image by default)
  - **zed/zed_node/right/image_rect_color**: Color rectified right image
  - **zed/zed_node/right/image_raw_color**: Color unrectified right image
  - **zed/zed_node/confidence/confidence_image**: Confidence map as image

**Note:** The Confidence Map is also available as a 32bit floating point image subscribing to the **/zed/zed_node/confidence/confidence_map** topic.

![](https://cdn.stereolabs.com/docs/ros/images/rgb.jpg)

### Displaying Depth

The depth map can be displayed in RVIZ with the following topic:

  - **zed/zed_node/depth/depth_registered**: 32-bit depth values in meters. RVIZ will normalize the depth map on 8-bit and display it as a grayscale depth image.

**Note:** An OpenNI compatibility mode is available in the `config/common.yaml` file. Set `depth.openni_depth_mode` to `1` to get depth in millimeters with 16-bit precision, then restart the ZED node.

![](https://cdn.stereolabs.com/docs/ros/images/depth.jpg)

### Displaying the Point cloud
A 3D colored point cloud can be displayed in RVIZ2 with the **zed/zed_node/point_cloud/cloud_registered** topic. 

Add it in RVIZ2 with `point_cloud` -> `cloud` -> `PointCloud2`. Note that displaying point clouds slows down RVIZ2, so open a new instance if you want to display other topics.

![](https://cdn.stereolabs.com/docs/ros/images/point_cloud.jpg)

### Displaying position and path

The ZED position and orientation in space over time is published to the following topics:

- **zed/zed_node/odom**: Odometry pose referred to odometry frame (only visual odometry is applied for ZED, visual-inertial for ZED-M)
- **zed/zed_node/pose**: Camera pose referred to Map frame (complete data fusion algorithm is applied)
- **zed/zed_node/pose_with_covariance**: Camera pose referred to Map frame with covariance (if spatial_memory is false in launch parameters)
- zed/zed_node/path_odom: The sequence of camera odometry poses in Map frame
- **zed/zed_node/path_map**: The sequence of camera poses in Map frame

**Important**: By default, RVIZ does not display odometry data correctly. Open the newly created Odometry object in the left list, and set Position Tolerance and Angle Tolerance to 0, and Keep to1.

## Launching with recorded SVO video

With the ZED, you can record and play back stereo video using Stereolabs' .SVO file format. To record a sequence, open the **ZED Explorer** app (`/usr/local/zed/tools`) and click on the **REC** button.

To launch the ROS wrapper with an SVO file, set the path of the SVO in the [launch parameter](https://www.stereolabs.com/docs/ros2/zed_node/#configuration-parameters) `general.svo_file` in the file `config/common.yaml`.

**Note:** add a `#` in front of the `general.svo_file` parameter to use an USB3 connected device, YAML does not allow to set an empty string parameter.

**Important:** Use only full paths to the SVO file. Relative paths are not allowed.

## Dynamic reconfigure
You can dynamically change many configuration parameters during the execution of the ZED node:

  - **general.mat_resize_factor**: Sets the scale factor of the output images and depth map. Note that the camera will acquire data at the dimension set by the *resolution* parameter; images are resized before being sent to the user
  - **video.auto_exposure**: Enables/disables automatic gain and exposure
  - **video.exposure**: Sets camera exposure only if *auto_exposure* is false
  - **video.gain**: Sets camera gain only if *auto_exposure* is false  
  - **depth.confidence**: Sets a threshold that filters the values of the depth or the point cloud. With a *confidence threshold* set to 100, all depth values will be written to the depth and the point cloud. This is set to 80 by default, which removes the least accurate values.
  - **depth.max_depth**: Sets the maximum depth range 

You can set the parameters using the [CLI](https://index.ros.org/doc/ros2/Tutorials/Introspection-with-command-line-tools/) command `ros2 param set`, e.g.:

```bash
$ ros2 param set /zed/zed_node depth.confidence 80
```
if the parameter is successfully set you will get a confirmation message:

```bash
Set parameter successful
```

If you try to set a parameter that's not dynamically reconfigurable, or if you provided an invalid value, you will get this error:

```bash
$ ros2 param set /zed/zed_node depth.confidence 150
Set parameter failed
```
and the ZED node will report a warning message explaining the error type:

```
1538556595.265117561: [zed.zed_node] [WARN]	The param 'depth.confidence' requires an INTEGER value in the range ]0,100]
```

## Limitations

The ROS2 wrapper is released as beta version. Many features available in the ROS wrapper are not yet introduced.
Due to problems with TF2 in ROS2 Crystal Clemmys the visualization in RVIZ2 cannot result fluid due to timestamp synchronization.
