LATEST CHANGES
==============

2021-08-06
----------
- Fix SVO LOOP wrong behavior. Thx @kevinanschau

2021-07-21
----------
- Add xacro support for automatic URDF configuration
- Reworked launch files to support xacro and launch parameters
    - Use `ros2 launch zed_wrapper <launch_file> -s` to retrieve all the available parameters
- Add `svo_path:=<full path to SVO file>` as input for all the launch files to start the node using an SVO as input without modifying 'common.yaml`

2021-07-15
----------
- Improved diagnostic information adding elaboration time on all the main tasks
- Improved diagnostic time and frequencies calculation
- Added StopWatch to sl_tools

2021-07-14
----------
- Enabled Diagnostic status publishing
- Changed the default values of the QoS parameter reliability for all topics from BEST_EFFORT to RELIABLE to guarantee compatibility with all ROS2 tools

2021-07-12
----------
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
