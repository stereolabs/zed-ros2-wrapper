LATEST CHANGES
==============

2022-06-13
----------
- Fix "NO DEPTH" mode. By setting `depth/quality` to `0` now the depth extraction and all the sub-modules depending on it are correctly disabled.
- Add `debug` sub-set of parameters with new parameters `debug_mode` and `debug_sensors`

2022-05-02
----------
- Add Plane Detection

2022-03-30
----------
- Fix `set_pose` wrong behavior. Now initial odometry is coherent with the new starting point.

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
