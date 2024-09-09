# Stereolabs ZED Camera - ROS2 Display package

This package lets you visualize in the [ROS2 RViz application](https://github.com/ros2/rviz/tree/foxy) all the
possible information that can be acquired using a Stereolabs camera.
The package provides the launch files for ZED, ZED Mini and ZED 2 camera models.

**Note:** The main package [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
is required to correctly execute the ROS node to acquire data from a Stereolabs 3D camera.

## Getting started

   - First, be sure to have installed the main ROS package to integrate the ZED cameras in the ROS framework: [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper/#build-the-package)
   - [Install](#Installation) the package
   - Read the online documentation for [More information](https://www.stereolabs.com/docs/ros2/)

### Prerequisites

   - Ubuntu 20.04
   - [ZED SDK](https://www.stereolabs.com/developers/release/latest/) v3.5 or later
   - [ROS2 ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)

### Installation

The *zed_display_rviz2* is a colcon package. 

Install the [zed-ros2-wrapper](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html) package
following the [installation guide](https://github.com/stereolabs/zed-ros2-wrapper#build-the-package)

Install the [zed-ros2-examples](https://github.com/stereolabs/zed-ros2-examples) package following the [installation guide](https://github.com/stereolabs/zed-ros2-examples#build-the-package)

### Execution

Use the following launch command to start the ZED ROS2 Wrapper node and RVIZ2 with the default setting for the camera that you are using:

```bash
$ ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=<camera_model>
```

Replace `<camera_model>` with the model of the camera that you are using: `'zed'`, `'zedm'`, `'zed2'`, `'zed2i'`, `'zedx'`, `'zedxm'`.

![ZED rendering on Rviz](images/depthcloud-RGB.jpg)
![ZED rendering on Rviz](images/ZEDM-Rviz.jpg)
![ZED rendering on Rviz](images/ZED-Rviz.jpg)

[Detailed information](https://www.stereolabs.com/docs/ros2/rviz2/)
