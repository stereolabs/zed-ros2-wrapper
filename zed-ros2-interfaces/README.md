![](./images/Picto+STEREOLABS_Black.jpg)

# Stereolabs ZED Camera - ROS2 Interfaces

The `zed-ros2-interfaces` repository install the `zed_interfaces` ROS2 package which defines the custom topics, services and actions used by the [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper) to interface with ROS2.

If you already installed the [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper) or you plan to install it on this machine, this package is not required because it is automatically integrated by `zed-ros2-wrapper` as a git submodule to satisfy all the required dependencies.

You must instead install this package on a remote system that must retrieve the topics sent by the ZED Wrapper (e.g. the list of detected objects obtained with the Object Detection module) or call services and actions to control the status of the ZED Wrapper.

**Note:** this package does not require CUDA, hence it can be used to receive the ZED data also on machines not equipped with an Nvidia GPU.

### Prerequisites

- Ubuntu 20.04
  - [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- Ubuntu 22.04
  - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Build the repository

The `zed_interfaces` is a colcon package. It depends on the following ROS packages:

- ament_cmake
- builtin_interfaces
- std_msgs
- geometry_msgs
- rosidl_default_generators
- rosidl_default_runtime
- rosidl_interface_packages

Open a terminal, clone the repository, update the dependencies and build the packages:

```
cd ~/catkin_ws/src
git clone https://github.com/stereolabs/zed-ros2-interfaces.git
cd ../
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
source ~/.bashrc
```

**Note**: If rosdep is missing you can install it with:

`$ sudo apt-get install python-rosdep python-rosinstall-generator python-vcstool python-rosinstall build-essential`

'Note': The option `--symlink-install` is very important, it allows to use symlinks instead of copying files to the ROS2 folders during the installation, where possible. Each package in ROS2 must be installed and all the files used by the nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without the needing to issue a new colcon build command. This is true only for all the files that don't need to be compiled (Python scripts, configurations, etc.).

**Note**: If you are using a different console interface like zsh, you have to change the `source` command as follows: `echo source $(pwd)/install/local_setup.zsh >> ~/.zshrc and source ~/.zshrc`.

## Custom Topics

 - BoundingBox2Df
 - BoundingBox2Di
 - BoundingBox3D
 - Keypoint2Df
 - Keypoint2Di
 - Keypoint3D
 - Object
 - ObjectsStamped
 - Skeleton2D
 - Skeleton3D

You can get more information reading the [Stereolabs online documentation](https://www.stereolabs.com/docs/ros2/zed-node/)

## Custom Services

 - SetPose
 - StartSvoRec

You can get more information reading the [Stereolabs online documentation](https://www.stereolabs.com/docs/ros2/zed-node/#services)
