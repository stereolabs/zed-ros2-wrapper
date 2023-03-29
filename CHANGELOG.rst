LATEST CHANGES
==============

v3.8.2
----------
- Removed annoying build log messages. Only warning regarding unsupported ROS2 distributions will be displayed when required.
- Added a check on timestamp to not publish already published point cloud messages in the point cloud thread
- Added Docker files (`docker` folder) ready to create Docker images for desktop host devices: Ubuntu 22.04, ROS2 Foxy, ZED SDK v3.8.2.
- Changed the default depth mode to `PERFORMANCE` (1) for a faster node start in the case that the NEURAL depth AI model is not present.
- Change `general.grab_resolution` and `general.pub_resolution` from integer to string.
- Add new `LOW` value for `general.pub_resolution` (half the `MEDIUM` output resolution).
- Fix distortion model (see Issue [#128](https://github.com/stereolabs/zed-ros2-wrapper/issues/128))
- Added diagnostic warning if the frequency of the camera grabbing thread is minor than the selected `general.grab_frame_rate` value.
- Improved the code for Moving Average calculation for better node diagnostics.
- Temperature diagnostic is now always updated even if `sensors.sensors_image_sync` is true and no image topics are subscribed.
- Improve Grab thread and Video/Depth publishing thread elaboration time diagnostic.
- Improve thread synchronization when the frequency of the `grab` SDK function is minor of the expected camera frame rate setting because of a leaking of elaboration power.
- Removed node parameter `general.resolution`, replaced by `general.grab_resolution`.
- Added node parameter `general.pub_resolution` used to reduce node computation and message bandwidth.
  - Available output resolutions: `HD2K`, `HD1080`, `HD720`, `MEDIUM`, `VGA`. `MEDIUM` is an optimized output resolution to maximize throughput and minimize processing costs.
- Removed node parameters `video.img_downsample_factor` and `depth.depth_downsample_factor`. Use the new parameter `general.pub_resolution` instead.
- Set default DEPTH MODE to NEURAL (4)
- Improved the `zed_camera.launch.py`

  - Added support for `OpaqueFunction` in order to automatically configure the launch file according to the value of the launch parameter `cam_model`.
  - Changed parameters to set camera pose in launch files. From 6 separated parameters (`cam_pos_x`,`cam_pos_y`,`cam_pos_z`,`cam_roll`,`cam_pitch`,`cam_yaw`) to one single array (`cam_pose`).
  - Removed the workaround for empty `svo_path` launch parameter values thanks to `TextSubstitution`.
  - Modified the "display" launch files in [zed-ros2-examples](https://github.com/stereolabs/zed-ros2-examples) to match the new configuration.
  - Added `publish_tf` and `publish_map_tf` launch parameters useful for multi-camera configuretion or external odometry fusion.

- Added multi-camera example in `zed-ros2-examples` repository.
- Added `pos_tracking.set_as_static` parameters for applications with a static camera monitoring a robotics environment. See [PR #122](https://github.com/stereolabs/zed-ros2-wrapper/pull/122 ) Thx @gabor-kovacs
- Code lint and re-formatting according to [ROS2 code rules](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
- Add support for automatic lint tools to all the packages.
- Change LICENSE to Apache 2.0 to match ROS2 license.
- Added `zed_id` and `serial_number` launch parameters to open the correct camera in multi-camera configurations.

v3.8.x
------
- Fixed `set_pose` wrong behavior. Now initial odometry is coherent with the new starting point.
- Added Plane Detection.
- Fixed "NO DEPTH" mode. By setting `depth/quality` to `0` now the depth extraction and all the sub-modules depending on it are correctly disabled.
- Added `debug` sub-set of parameters with new parameters `debug_mode` and `debug_sensors`.
- Added support for ROS2 Humble. Thx @nakai-omer.
  The two ROS2 LTS releases are now supported simoultaneously.
- Set `read_only` flag in parameter descriptors for non-dynamic parameters. Thx @bjsowa.
- Enabled Intra Process Communication. The ZED node no longer publishes topics with `TRANSIENT LOCAL` durability.
- Improved TF broadcasting at grabbing frequency
- Improved IMU/Left Camera TF broadcasting at IMU frequency
- Fixed data grabbing frame rate when publishing is set to a lower value
- Added TF broadcasting diagnostic
- The parameter `general.sdk_verbose` is now an integer accepting different SDK verbose levels.
- Moved Object Detection parameters from cameras configuration files to `common.yaml`
- Moved Sensor Parameters from cameras configuration files to `common.yaml`
- New data thread configuration to maximize data publishing frequency
  - Sensor data publishing moved from timer to thread
  - RGB/Depth data publishing moved from timer to thread
- Fixed random errors when closing the node
- Fixed wrong timing when playing SVO in `real-time` mode
- Fixed units for atmospheric pressure data. Now pressure is published in `Pascals` according to the [definition of the topic](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/FluidPressure.msg).
- Add new parameter `pos_tracking.transform_time_offset` to fix odometry TF timestamp issues
- Added new parameter `pos_tracking.depth_min_range` for removing fixed zones of the robot in the FoV of the camerafrom the visual odometry evaluation
- Added new parameter `pos_tracking.sensor_world` to define the world type that the SDK can use to initialize the Positionnal Tracking module
- Added new parameter `object_detection.prediction_timeout` for setting the timeout time [sec] of object prediction when not detected.
- Added support for ZED SDK Regiorn of Interest:
  - Added parameter `general.region_of_interest` to set the region of interest for SDK processing.
  - Added the service `resetRoi` to reset the region of interest.
  - Added the service `setRoi` to set a new region of interest.

v3.7.x
----------
- Add support for sport-related OD objects
- Add `remove_saturated_areas` dynamic parameter to disable depth filtering when luminance >=255
- Add `sl::ObjectDetectionParameters::filtering_mode` parameter
- Publish `depth_info` topic with current min/max depth information
- Fix parameter override problem (Issue #71). Thx @kevinanschau
- Add default xacro path value in `zed_camera.launch.py`. Thx @sttobia
- Fix `zed-ros2-interfaces` sub-module url, changing from `ssh` to `https`.

v3.6.x (2021-12-03)
-------------------
- Moved the `zed_interfaces` package to the `zed-ros2-interfaces` repository to match the same configuration of the ROS1 wrapper
- The `zed-ros2-interfaces` repository has been added as a sub-module to this repository
- Add new <zed>_base_link frame on the base of the camera to easily handle camera positioning on robots. Thx @civerachb-cpr
- Improve URDF by adding 3° slope for ZED and ZED2, X-offset for optical frames to correctly match the CMOS sensors position on the PCB, X-offset for mounting screw on ZED2i
- Add `zed_macro.urdf.xacro` to be included by other xacro file to easily integrate ZED cameras in the robot descriptions. See ROS1 PR [#771](https://github.com/stereolabs/zed-ros-wrapper/pull/771) for details. Thx @civerachb-cpr
- Fix URDF `height` value for ZED, ZED2 and ZED2i
- Fix performances drop on slower platforms. Thx @roncapat
- Fix SVO LOOP wrong behavior. Thx @kevinanschau
- Add xacro support for automatic URDF configuration
- Reworked launch files to support xacro and launch parameters
    - Use `ros2 launch zed_wrapper <launch_file> -s` to retrieve all the available parameters
- Add `svo_path:=<full path to SVO file>` as input for all the launch files to start the node using an SVO as input without modifying 'common.yaml`
- Improved diagnostic information adding elaboration time on all the main tasks
- Improved diagnostic time and frequencies calculation
- Added StopWatch to sl_tools
- Enabled Diagnostic status publishing
- Changed the default values of the QoS parameter reliability for all topics from BEST_EFFORT to RELIABLE to guarantee compatibility with all ROS2 tools
- Fixed tab error in `zedm.yaml`
- Fixed compatibility issue with ZED SDK older than v3.5 - Thanks @PhilippPolterauer
- Migration to ROS2 Foxy Fitzroy

v3.5.x (2021-07-05)
-------------------
- Add support for SDK v3.5
- Add support for the new ZED 2i
- Add new parameter `pos_tracking/pos_tracking_enabled` to enable positional tracking from start even if not required by any subscribed topic. This is useful, for example, to keep the TF always updated.
- Add support for new AI models: `MULTI_CLASS_BOX_MEDIUM` and `HUMAN_BODY_MEDIUM`
- Depth advertising is disabled when depth is disabled (see `sl::DETH_MODE::NONE`)
