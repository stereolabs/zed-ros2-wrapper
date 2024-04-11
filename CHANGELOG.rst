LATEST CHANGES
==============

v4.0.8
------
- The parameter `general.sdk_verbose` has been moved to `debug.sdk_verbose` and set to `0` as default.
- Add new parameter `general.optional_opencv_calibration_file` to use custom OpenCV camera calibrations.
- Add [new tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_robot_integration) to illustrate how to integrate one or more ZED cameras on a robot
- Add 'simulation.sim_enabled' parameter to enable the simulation mode
- Add 'simulation.sim_address' parameter to set the simulation server address
- Add 'simulation.sim_port' parameter to set the simulation server port
- Add `/clock` subscriber to check the presence of the required message when `use_sim_time` is true
- Force `grab_frame_rate` and `pub_frame_rate` to 60 Hz in simulation
- Force `grab_resolution` to `HD1080` in simulation
- Remove the `general.zed_id` parameter. Always use `general.serial_number` to distinguish between different cameras in a multi-camera configuration.
- The multi-camera example has been updated to match the new TF configuration
- The old launch files are now obsolete: 'ros2 launch zed_wrapper <camera_model>.launch.py' is replaced by 'ros2 
  launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>'
- The reference link for positional tracking is no longer 'base_link' but `<camera_name>_camera_link`. 
  This will allow an easier ZED integration in existing robot configuration because the transform `base_link` -> `camera_link` 
  is no longer published by the ZED ROS2 Wrapper. Thanks to @SteveMacenski for the advice
  - Remove `parent` and `origin` parameters from `zed_macro.urdf.xacro`
  - Remove launch argument `cam_pose` from `zed_camera.launch.py`
- Move parameter `publish_imu_tf` from `pos_tracking` to `sensors` to make it available also in "no depth" configurations of the node
- Add new parameter `pos_tracking.pos_tracking_mode` to exploit the new ZED SDK `QUALITY` mode for improved odometry and localization
- New Video/Depth processing throttling method by using the `grab_compute_capping_fps` ZED SDK parameter instead of a dedicated thread
- Advanced parameters to handle Thread scheduling policy and priorities (sudo required):`thread_sched_policy`,`thread_grab_priority`,
  `thread_sensor_priority`,`thread_pointcloud_priority`
- Add new GNSS calibration parameters: `enable_reinitialization`, `enable_rolling_calibration`, `enable_translation_uncertainty_target`, `gnss_vio_reinit_threshold`, `target_translation_uncertainty`, `target_yaw_uncertainty`
- Add new Plane Detection parameters: `pd_max_distance_threshold`, `pd_normal_similarity_threshold`

v4.0.5
----------
- The parameter `general.pub_resolution` can now take only `NATIVE` and `CUSTOM` values. 'NATIVE' to use the same `general.grab_resolution` - `CUSTOM` to apply the `general.pub_downscale_factor` downscale factory to reduce bandwidth in transmission
- Add new parameter `general.pub_downscale_factor` to be used with the new option `CUSTOM` for the parameter `general.pub_resolution`
- `ULTRA` is the new default value for `depth.depth_mode` (better performance for odometry and positional tracking)
- Add resolution `HD1080` for ZED X
- Fix issue with Body Tracking start/stop by service call. Now Body Tracking can be restarted multiple times
- Fix depth grab performance by removing a [not required `PNG Write` call](https://github.com/stereolabs/zed-ros2-wrapper/pull/164). Thank you Esteban Zamora @ezamoraa 
- Fix bug with `general.pub_resolution` value, not allowing to select the correct data publish resolution
- Add new launch parameter `ros_params_override_path` to provide the path to a custom YAML file to override the parameters of the ZED Node without modifying the original files in the `zed_wrapper/config` folder. Thank you David Lu @MetroRobots

v4.0.0
------
- Add support for ZED-X and ZED-X Mini

  - Move `general.grab_resolution` and `general.grab_frame_rate` to the yaml file specific for the relative camera model (i.e. `zed.yaml`, `zedm.yaml`, `zed2.yaml`, `zed2i.yaml`, `zedx.yaml`, `zedxm.yaml`)
  - Add `zedx.launch.py` for ZED-X
  - Add `zedxm.launch.py` for ZED-X Mini
  - Improve `zed_macro.urdf.xacro` with specific configuration for the new camera models
  - Add `display_zedx.launch.py` for ZED-X to ZED-ROS2-Examples
  - Add `display_zedxm.launch.py` for ZED-X Mini to ZED-ROS2-Examples
  - Add ZED-X and ZED-X Mini STL files to ZED-ROS2-Interfaces

- Positional Tracking

  - Add `pos_tracking.set_as_static` parameters for applications with a static camera monitoring a robotics environment. See [PR #122](https://github.com/stereolabs/zed-ros2-wrapper/pull/122 ) Thx @gabor-kovacs
  - Add custom message type `PosTrackStatus`
  - Publish message on topic `~/pose/status` with the current status of the pose from the ZED SDK
  - Publish message on topic `~/odom/status` with the current status of the odometry from the ZED SDK

- Body Tracking

  - Add Support for the new Body Tracking module
  - Add parameter `body_tracking.bt_enabled` to enable Body Tracking
  - Add parameter `body_tracking.model` to set the AI model to be used
  - Add parameter `body_tracking.body_format` to set the Body Format to be used
  - Add parameter `body_tracking.allow_reduced_precision_inference` to improve performances
  - Add parameter `body_tracking.max_range` to set the max range for Body Detection
  - Add parameter `body_tracking.body_kp_selection` to choose the Body key points to be used
  - Add parameter `body_tracking.enable_body_fitting` to enable body fitting
  - Add parameter `body_tracking.enable_tracking` to enable the tracking of the detected bodies
  - Add parameter `body_tracking.prediction_timeout_s` to set the timeout of the prediction phase while tracking
  - Add parameter `body_tracking.confidence_threshold` to set the detection confidence threshold
  - Add parameter `body_tracking.minimum_keypoints_threshold` to set the minimum number of detected key points to consider a body valid
  - Publish new message on topic `~/body_trk/skeletons`
  - Add service `enable_body_trk` to start/stop body tracking

- GNSS fusion integration

  - New param `gnss_fusion.gnss_fusion_enabled` to enable GNSS fusion
  - New param `gnss_fusion.gnss_fix_topic` name of the topic containing GNSS Fix data of type `sensor_msgs/NavSatFix`
  - Add `nmea_msgs` dependency
  - Add GNSS Fix Diagnostic
  - Add new launch parameter `gnss_frame` to enable the GNSS link in the ZED URDF
  - Add new node parameter `gnss_fusion.gnss_zero_altitude` to ignore GNSS altitude information
  - Add new node parameter `gnss_fusion.gnss_frame` to set the name of the frame link of the GNSS sensor
  - Disable Area Memory (loop closure) when GNSS fusion is enabled
  - Add services `toLL` and `fromLL` to use the ZED ROS2 Wrapper with the Nav2 Waypoint Navigation package
  - Add `geographic_msgs::msg::GeoPoseStamped` message publisher
  - Add parameter `gnss_fusion.publish_utm_tf`
  - Add parameter `gnss_fusion.broadcast_utm_transform_as_parent_frame`
  - Add parameter `gnss_fusion.gnss_init_distance`
  - Publish message on topic `~/geo_pose/status` with the current status of the GeoPose from the ZED SDK
  - Publish message on topic `~/pose/filtered` with the current GNSS filtered pose in `map` frame
  - Publish message on topic `~/pose/filtered/status` with the current status of the GNSS filtered pose from the ZED SDK

- Object Detection

  - Add `object_detection.allow_reduced_precision_inference` to allow inference to run at a lower precision to improve runtime and memory usage
  - Add `object_detection.max_range` to defines a upper depth range for detections
  - Remove `object_detection.body_format`

- Docker

  - Add Docker files (see `docker` folder) ready to create Docker images for desktop host devices

- Examples/Tutorials

  - Add multi-camera example in `zed-ros2-examples` repository.

- Add full Terrain Mapping (local obstacle detection) support [EXPERIMENTAL FEATURE AVAILABLE ONLY FOR BETA TESTERS]

  - ZED SDK Terrain Mapping published as GridMap message
  - Add parameter `local_mapping.terrain_mapping_enabled` to enable terrain mapping publishing a local obstacle map
  - Add parameter `local_mapping.data_pub_rate` to set the Local Map data publish frequency
  - Add parameter `local_mapping.grid_resolution` to set the Local Map resolution in meters [min: 0.01 - max: 1.0]
  - Add parameter `local_mapping.grid_range` to set the maximum depth range for local map generation [min: 1.0 - max: 8.0]
  - Add parameter `local_mapping.height_threshold` to set the maximum height for obstacles
  - Publish gridmap on topic `local_map/gridmap`
  - Publish elevation map image on topic `local_map/elev_img`
  - Publish obstacle color map image on topic `local_map/col_img`
  - Add traversability cost computation for Terrain Mapping (local_mapping)

    - Change parameter `local_mapping.height_threshold` to `local_mapping.robot_heigth`
    - Add parameter `local_mapping.robot_radius` to set radius of the robot
    - Add parameter `local_mapping.robot_max_step` to set max height of a step that the robot can overcome
    - Add parameter `local_mapping.robot_max_slope` to set max slope (degrees) that the robot can overcome
    - Add parameter `local_mapping.robot_max_roughness` to set max roughness of the terrain that the robot can overcome

- Add support for simulated data [EXPERIMENTAL FEATURE AVAILABLE ONLY FOR BETA TESTERS]

  - Add parameter `use_sim_time` to enable SIMULATION mode
  - Add parameter `sim_address` tos set the local address of the machine running the simulator
  - Change StopWatch to use ROS clock instead of System Clock. In this way diagnostic and time checking work also in simulation
  - Disable camera settings control in simulation

- Others

  - Remove `sensing_mode`, no more available in SDK v4.0
  - Remove `extrinsic_in_camera_frame`, no more available in SDK v4.0
  - Add `zed_id` and `serial_number` launch parameters to open the correct camera in multi-camera configurations.
  - Code lint and re-formatting according to [ROS2 code rules](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
  - Add support for automatic lint tools to all the packages.
  - Remove node parameter `general.resolution`, replaced by `general.grab_resolution`.
  - Add node parameter `general.pub_resolution` used to reduce node computation and message bandwidth.

    - Available output resolutions: `HD2K`, `HD1080`, `HD720`, `MEDIUM`, `VGA`. `MEDIUM` is an optimized output resolution to maximize throughput and minimize processing costs.
  
  - Remove node parameters `video.img_downsample_factor` and `depth.depth_downsample_factor`. Use the new parameter `general.pub_resolution` instead.
  - Change `general.grab_resolution` and `general.pub_resolution` from integer to string.
  - Add new `LOW` value for `general.pub_resolution` (half the `MEDIUM` output resolution).
  - Remove `depth.quality` parameter (replaced with `depth.depth_mode`)
  - Add `depth.depth_mode` parameter: a string reflecting the ZED SDK `DEPTH_MODE` available value names
  - The parameter `depth.depth_stabilization` is now an integer in [0,100] reflecting ZED SDK behavior
  - Fix distortion model (see Issue [#128](https://github.com/stereolabs/zed-ros2-wrapper/issues/128))
  - Improve the code for Moving Average calculation for better node diagnostics.
  - Temperature diagnostic is now always updated even if `sensors.sensors_image_sync` is true and no image topics are subscribed.
  - Improve Grab thread and Video/Depth publishing thread elaboration time diagnostic.
  - Add a check on timestamp to not publish already published point cloud messages in the point cloud thread
  - Improve thread synchronization when the frequency of the `grab` SDK function is minor of the expected camera frame rate setting because of a leaking of elaboration power.
  - Add diagnostic warning if the frequency of the camera grabbing thread is minor than the selected `general.grab_frame_rate` value.
  - Remove annoying build log messages. Only warning regarding unsupported ROS2 distributions will be displayed when required.
  - Convert `shared_ptr` to `unique_ptr` for IPC support
  - Improve the `zed_camera.launch.py`

    - Add support for `OpaqueFunction` in order to automatically configure the launch file according to the value of the launch parameter `cam_model`.
    - Change parameters to set camera pose in launch files. From 6 separated parameters (`cam_pos_x`,`cam_pos_y`,`cam_pos_z`,`cam_roll`,`cam_pitch`,`cam_yaw`) to one single array (`cam_pose`).
    - Remove the workaround for empty `svo_path` launch parameter values thanks to `TextSubstitution`.
    - Modify the "display" launch files in [zed-ros2-examples](https://github.com/stereolabs/zed-ros2-examples) to match the new configuration.
    - Add `publish_tf` and `publish_map_tf` launch parameters useful for multi-camera configuretion or external odometry fusion.
  
  - Change LICENSE to Apache 2.0 to match ROS2 license.

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
