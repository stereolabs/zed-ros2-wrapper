LATEST CHANGES
==============

v5.6.1-dev
----------

- Fixed the stereo node publishing, on a topic that a node starts subscribing to while other image or depth topics are published, one empty image or one image from a previous subscription with the current timestamp. All the image, depth, depth info, confidence and disparity topics were affected.

v5.6.0
------

New Features
~~~~~~~~~~~~

- Added the `~/reboot_camera` service (`std_srvs/srv/Trigger`, stereo and mono) to reboot a live camera without restarting the node: the node closes the camera, reboots it, reopens it and resumes publishing. The call returns when the camera is back, after 5 to 15 s. Available for the GMSL cameras, the ZED 2 and the ZED 2i, not with SVO, streaming or simulation input, nor with GNSS fusion enabled. The positional tracking restarts from the last valid pose (with `pos_tracking.floor_alignment`, the floor is not searched again because that pose is already aligned to it), object detection, body tracking, spatial mapping, the streaming server and the Region of Interest are enabled again, and an active SVO recording is stopped. The other GMSL cameras of the device can stop for a few seconds. If the camera does not come back within `general.camera_timeout_sec` x `general.camera_max_reconnect` seconds, the call fails and the node stops, as after a disconnection. The diagnostic reports the reboot in progress and, in the new `Reboots` entry, the number of reboots with the time and result of the last one.
- Full Isaac ROS integration with `rosidl::Buffer` support and CUDA zero-copy transport (ROS 2 Lyrical and newer, stereo and mono):

  - The images, depth maps, confidence maps and point clouds stay in GPU memory and are shared without copies with the subscribers on the same host and GPU that accept the `cuda` `rosidl::Buffer` backend.
  - All the other subscribers (RViz2, `ros2 bag`, Python nodes, other hosts, intra-process subscribers, compressed transports) automatically receive the standard CPU data, with no change on their side.
  - Enabled by default with the new `general.buffer_backend` parameter (`cuda` default, `cpu`). The node falls back to CPU, with a log message, when the `cuda_buffer` and `cuda_buffer_backend` packages are not installed or the RMW does not support buffer backends: `rmw_fastrtps_cpp` and `rmw_zenoh_cpp` v0.10.5 or newer are supported.
  - The topics published with NITROS are unchanged.
  - Measured with a ZED 2i (HD720 @ 60 FPS) on an AGX Thor: point cloud received at 10 Hz instead of 4-6 Hz by a CUDA subscriber in another process, image publishing with 7-22% less CPU and ~2 ms less latency. CPU-only subscribers are not affected.

- Added the motion prior to the positional tracking (`pos_tracking.motion_prior.*` parameters): the stereo node subscribes to a `nav_msgs/Odometry` topic, such as wheel odometry, and sends its motion to the `GEN_3` positional tracking, which then stays accurate in fast motion, low texture or occlusions, and follows the motion source while the images cannot be used. Its status is shown in the diagnostic. Requires ZED SDK >= 5.6.
- Added the `~/pose/motion_prior_links` topic (`visualization_msgs/Marker`), published while the motion prior is enabled: one segment between the two keyframes of each link the motion prior holds, green when used, yellow when deferred, red when rejected. Requires ZED SDK >= 5.6.
- Added the `tracking_fusion_status` and `motion_prior_status` fields to the `PosTrackStatus` message published on `~/pose/status`. Requires `zed_msgs` v5.6 (and ZED SDK >= 5.6 for `motion_prior_status`); with an older `zed_msgs` the node runs as before, without the fields.
- Added the `NEURAL_MAX` value to `depth.depth_mode`: the most accurate NEURAL depth mode, for high-end GPUs such as Jetson AGX Thor or for offline SVO processing. Requires ZED SDK >= 5.6.
- Added the `depth.depth_compute_device` parameter (`AUTO` default, `GPU`, `DLA`, `PVA`, experimental) to run the `NEURAL_LIGHT` depth on an accelerator of the Jetson and leave the GPU to the application. The diagnostic shows the selected device, or `GPU` when the ZED SDK runs the depth on the GPU instead, at start-up or during the session, and a warning says so. Requires ZED SDK >= 5.6.
- Added the `depth.publish_depth_image` parameter (default `false`) to publish the colorized depth visualization on `~/image/depth_image`, separate from the depth measurement published on `~/depth/depth_registered`. Like the other image topics, it follows `video.enable_24bit_output`, and it is only advertised if enabled and only published while a node subscribes to it.
- Added the `sensors.imu_sampling_rate`, `sensors.imu_accelerometer_range` and `sensors.imu_gyroscope_range` parameters (stereo and mono) to select a 400 Hz IMU and the IMU ranges on ZED X family cameras. The values in use are logged at start-up. Requires ZED SDK >= 5.6 and ZED X driver >= 1.5.0.
- Added the `-1` value to `video.exposure_compensation` and `video.ae_antibanding` (stereo and mono): the node does not set the setting and the camera keeps its own value. It is the default in `virtual.yaml`. ZED SDK 5.6 also fixes these settings being refused by the ZED X Mini and ZED X One, which made the node warn at every start.
- Added the `general.sync_pub_to_timestamp` parameter (stereo and mono, default `false`) for GMSL cameras connected to the same capture card: the frames to publish are selected on their image timestamp instead of each node's publishing timer, so with `general.pub_frame_rate` lower than `general.grab_frame_rate` all the cameras publish the frames captured at the same instant, ZED X and ZED X One cameras together. In the stereo node the point cloud then follows `general.pub_frame_rate`, `general.grab_compute_capping_fps` is ignored, and the depth is computed only for the published frames when no other module needs it. Thanks to @khaterma (#447).
- Added the `general.exit_on_failure` parameter (stereo and mono) to choose what happens when a node fails for good (camera that cannot be opened or that does not come back, invalid configuration, critical camera error): `true` stops the whole process, so that the launch ends and a supervisor can restart it; `false` stops this node only, and the other nodes of its container keep running. `zed_camera.launch.py` sets it to `true` when it creates the container and to `false` when the node is loaded in an existing container (`container_name`, as `zed_multi_camera.launch.py` does). The default value, used for example with `ros2 component load`, is `false`. A stopped node reports the reason in its diagnostic and can be unloaded with `ros2 component unload`.
- Added the `general.pub_frame_rate` parameter to the mono node (default `0.0`, publish every grabbed frame as before) to publish the images at a lower rate than `general.grab_frame_rate`.

Upgrades
~~~~~~~~

- Reduced the CPU usage of both nodes:

  - The sensors data are no longer read at the IMU rate when no node subscribes to the sensors topics.
  - The images, depth maps and disparity maps are copied only when they are published both raw and compressed, and the deprecated `~/disparity/disparity_image` topic is filled only when a node subscribes to it.
  - With intra-process communication, the images are copied only for the subscribers in the same process.
  - The mono node no longer subscribes to `/tf` and `/tf_static`.
  - Repeated errors in the data threads are logged at most once per second.
  - Measured with the container CPU usage, in % of a CPU core. AGX Orin with a ZED X (HD1200 @ 30 FPS): 37% instead of 40% with no subscriber, 115% instead of 130% with the color image, the depth map and the side-by-side image subscribed by another process. AGX Thor with a ZED 2i (HD720 @ 30 FPS): 18% instead of 24% with no subscriber; no measurable change with the same subscribers.

- Raised the minimum ZED SDK version to v5.0.7 (was v4.2): the build stops with a CMake error when an older version is installed.
- Changed how both nodes handle a camera disconnected while running: they wait `general.camera_timeout_sec` x `general.camera_max_reconnect` seconds (25 by default) for it to come back, then stop, as described for `general.exit_on_failure`. Previously they waited forever while the ZED SDK was reopening the camera, and were killed when it reported the camera as not detected. Both parameters are now also available in `common_mono.yaml`.
- The mono node now retries to open the camera at start-up every `general.camera_timeout_sec` seconds, for `general.camera_timeout_sec` x `general.camera_max_reconnect` seconds, as the stereo node does. Previously it stopped at the first failure, for example when another node of the same device was opening its camera at the same time.
- Added the camera disconnections to the diagnostic (stereo and mono): while the camera is disconnected the status reports how long the node has been waiting, and the new `Disconnections` entry counts the disconnections of the session with the time of the last reconnection.
- Changed `zed_camera.launch.py` and `zed_camera_debug.launch.py` to stop the whole launch (including `robot_state_publisher`) when the ZED node exits, so `ros2 launch` returns and a supervisor can restart it.
- The stereo node now stops with a clear error when the AI model of `depth.depth_mode` is missing and cannot be downloaded, instead of retrying forever and asking to check the camera connection. Requires ZED SDK >= 5.6.
- GNSS fusion now also runs with `depth.depth_mode: 'NONE'` and `GEN_3`, which saves the depth computation. Requires ZED SDK >= 5.6; with older versions the node disables GNSS fusion with a warning instead of failing silently.
- Note for the upgrade to ZED SDK 5.6: with `GEN_3`, the area files saved by the node (`pos_tracking.save_area_memory_on_closing`, `save_area_memory` service) can no longer be loaded by ZED SDK 5.5 or older, and an older file is converted when saved again. Keep a copy if an older ZED SDK still needs it.
- Removed the stereo node warning about `depth.depth_precision: 'INT8'` with ZED SDK >= 5.6: every NEURAL depth mode now has an INT8 model, and the ZED SDK itself warns when the depth mode or the GPU cannot use it.
- Changed the stereo node to refuse `depth.depth_mode: 'CUSTOM'` (depth provided by the application), which the ROS 2 Wrapper does not support: it now warns and uses `NEURAL`.
- Changed both nodes to report a video setting refused by the camera or the input (unsupported setting or refused value) once, naming the node parameter, instead of retrying it: the mono node retried it at every frame, the stereo node 20 times. Requires ZED SDK >= 5.6, which reports these errors.
- Changed both nodes to send the video parameters (`video.*`, also when changed at runtime) to a simulated camera, which applies the ones the simulator supports; the others are reported once and ignored. Previously the video parameters were ignored in simulation. Requires ZED SDK >= 5.6.
- Added the `simulation.self_calib` parameter (default `false`): in simulation it replaces `general.self_calib`, so the self-calibration is now disabled by default with a simulated camera, for reproducible results. Set it to `true` to restore the previous behavior.
- Added to the Docker README the `--ulimit rtprio=5` option for GMSL cameras, which lets ZED SDK >= 5.6 with ZED X driver >= 1.5.0 run the camera capture and sensors threads with real-time priority. The `advanced.*` comments now state the system requirements of `SCHED_FIFO` and `SCHED_RR`, and warn that higher node priorities can delay the camera capture.
- Changed the default camera configurations to match the camera specifications: `zedxone4k.yaml` now uses `HD1200` at 30 FPS (`QHDPLUS` only runs at 15 FPS and only in HDR mode, so the previous default could not run as configured), `zed2i.yaml` runs at 30 FPS like the other USB cameras (was 60 FPS), `zedxnano.yaml` limits `depth.max_depth` to 3 m, the range of the camera (was 5 m), and `zedxm.yaml` and `zedxnano.yaml` use the same manual exposure and gain values as `zedx.yaml` (`video.exposure_time: 16000`, `video.auto_exposure_time_range_max: 30000`, `video.analog_gain: 1255`, `video.digital_gain: 1`).
- Removed the `urdf` folder: the camera models now belong to the `zed_description` package.

Bug fixes
~~~~~~~~~

- Fixed the mono node crashing (memory corruption) when the `set_svo_frame` service is called while an SVO is playing.
- Fixed the "System overloaded" diagnostic warning never being reported (stereo and mono): it is now reported when the processing is more than 10% slower than the grab rate for 10 consecutive diagnostic updates.
- Fixed a ZED node that cannot open its camera, or that has an invalid configuration or a critical camera error, stopping every other node of its component container, usually with a crash: the cameras of the other nodes stopped publishing (stereo and mono). The node now stops alone, or stops the whole process cleanly when `general.exit_on_failure` is `true`. A node that cannot be created (unsupported ZED SDK version) now makes `ros2 component load` report the failure instead of ending the container.
- Fixed a possible crash when stopping a container from which a ZED node had been unloaded with `ros2 component unload` (stereo and mono).
- Fixed the mono node possibly never applying a video setting that was still waiting to be applied when another parameter, such as `sensors.sensors_pub_rate`, was changed.
- Fixed the build with ZED SDK 5.0 and 5.1: with these versions the `virtual` camera model (ZED SDK >= 5.1), `pos_tracking.enable_localization_only` (ZED SDK >= 5.1) and the custom object detection tracking parameters (ZED SDK >= 5.2) are not available, and the IMU of an SVO or a simulated camera is read at the camera frame rate.
- Fixed the positional tracking never starting with `depth.depth_mode: 'NONE'` and `pos_tracking.pos_tracking_mode: 'GEN_3'`, a combination supported since ZED SDK 5.2: it now starts, and with another tracking mode the node says why it is disabled.
- Fixed the positional tracking failing to start with ZED SDK 5.6 when `pos_tracking.area_file_path` is set and `pos_tracking.area_memory` is `false`: the file is now ignored with a warning, as older ZED SDK versions did.
- Fixed GNSS fusion with `GEN_3` flooding the log: the Area Memory (loop closure), which cannot be combined with GNSS fusion, is now really disabled, and the GNSS fusion status and reference initialization are logged on change only instead of at every frame.
- Fixed the deprecated `status` field of `PosTrackStatus` on `~/pose/status`, which was always published as `0` (read as `SEARCHING`) since the GEN_3 status fields were introduced. It now carries the positional tracking state returned by the ZED SDK (`SEARCHING`, `OK`, `OFF`, `FPS_TOO_LOW`, `SEARCHING_FLOOR_PLANE`, `UNAVAILABLE`) for code still relying on it. New code should use `odometry_status` and `spatial_memory_status`.
- Fixed `pose_confidence` on `~/pose/status` being able to wrap around in the `uint8` field: the value is now clamped to the documented [0,100] range.
- Fixed `~/imu/data_raw` (stereo and mono) not flagging its missing orientation as REP-145 requires: `orientation_covariance[0]` is now `-1`, so consumers such as `robot_localization` ignore the orientation field instead of reading the all-zero covariance as an exactly known orientation.
- Fixed the stereo node aborting with `can't compare times with different time sources` when `sensors.sensors_image_sync` is enabled on a non-ZED camera model: the exception took down the component container.
- Fixed a USB camera plugged back more than 10 seconds after a disconnection never resuming: the ZED SDK now keeps trying to reopen it for as long as the node waits for it.
- Fixed both nodes retrying to open a camera that the ZED SDK had already opened with a warning (such as `CONFIGURATION_FALLBACK`): the node now logs the warning and starts.
- Fixed `optical_offset_x` for the different camera models.
- Fixed SVO recordings started with `start_svo_rec` being empty with ZED SDK versions older than 5.4 when neither depth nor positional tracking is required (e.g. `depth.depth_mode: 'NONE'`): the node now grabs the frames while recording, without computing depth. With ZED SDK 5.4 and newer the recording was already correct. Thanks to @Parv-Maheshwari (#461).
- Fixed the positional tracking never starting when the ZED SDK enables it with a warning, such as `SENSORS_DATA_REQUIRED` with `GEN_3` and `pos_tracking.imu_fusion: true` on an input without high-frequency IMU data (a network stream or Isaac Sim): the node logged `Pos. Tracking not started` and retried, while the ZED SDK was tracking with the integrated orientation as rotation prior. The node now logs the warning and uses the positional tracking. Thanks to @akashshingha850 (#463).
- Fixed the stereo node flooding the log with `sl::getSensorsData error` warnings (about 100 per second) when the input does not provide the IMU data at the sensor rate, such as an SVO v1: the sensors thread now stops with a single warning, which suggests `sensors.sensors_image_sync: true` to publish the IMU data at the camera frame rate. The same applies to an input without IMU. The temperatures are now read with the image timestamp, which every input provides, so their warning at every second is also gone.
- Fixed the stereo node publishing the camera info topics at the grab rate instead of `general.pub_frame_rate` while no node subscribes to the images or to the depth.
- Fixed both nodes aborting, together with their container, when a parameter value set in a YAML file or in a launch override is out of range or of the wrong type (for example `5` instead of `5.0`, or `[0, 0, 0, 0, 0, 0]` for `pos_tracking.initial_base_pose`): the node now warns and uses a valid value instead: a number of the other numeric type is converted (`5` for `5.0`, `6.6` rounded to `7` for an integer), an out-of-range value is replaced by the nearest limit, and any other wrong value by the default value.
- Fixed the mono node accepting `general.grab_resolution: 'SVGA'` with a ZED X One 4K, which does not support it, so the camera failed to open: the node now warns and uses `AUTO`.
- Fixed the stereo node reporting `CAMERA NOT DETECTED` when `general.serial_number` is the serial number of a connected ZED X One. The node now stops at once with an error that names the ZED X One model and the `camera_model` value that starts the mono node.
- Fixed the mono node retrying for `general.camera_timeout_sec` x `general.camera_max_reconnect` seconds with `MODULE NOT COMPATIBLE WITH SELECTED CAMERA` when `general.serial_number` is the serial number of a connected stereo camera. The node now stops at once with an error that names the stereo camera model and the `camera_model` value that starts the stereo node.
- Fixed the stereo node not saying that it retries while waiting for a camera: it now logs `Error opening camera: <error>. Retrying in <N> sec...`, like the mono node.
- Fixed the mono node sending video settings that the camera does not have in its current mode, such as `video.gamma` on a ZED X One 4K with `video.enable_hdr: true`: they are no longer sent, so the node no longer warns about them.
- Fixed a looping SVO (`svo.svo_loop: true`) starting the positional tracking at the end of the recording when it is disabled. The node restarted the tracking to reset the pose, and without a `robot_state_publisher` providing the camera transforms (for instance the component composed in another application) it stopped publishing. The pose is now reset only when the positional tracking is running.
- Fixed the stereo node waiting 2.8 hours instead of 10 seconds for the camera static transforms when it starts the positional tracking and no node publishes them (no `robot_state_publisher`, for example `publish_urdf:=false` or the component composed in another application): the node published nothing and did not stop on Ctrl+C. The node now waits 10 seconds, measured in real time (also with `use_sim_time` while `/clock` is not published), and stops waiting at once when it is shut down or unloaded. After the timeout it warns and starts the positional tracking without the transform to the base frame (`<camera_name>_camera_link`): the pose and the odometry are then the ones of the camera center, `~/odom` carries `<camera_name>_camera_center` as `child_frame_id`, and the `odom` -> `<camera_name>_camera_link` TF is not published. As soon as the transform is available, the pose and the odometry move to the base frame without a jump of the camera position and the TF is published.

v5.5.0
------
- Fixed the large IMU publishing delay caused by the IMU being read in step with `grab()` (stereo and mono). With `general.grab_frame_rate: 15` and `general.grab_compute_capping_fps: 5` the samples arrived in 5 Hz bursts several hundred milliseconds late; the mono node, which has no compute capping, was delayed by the grab rate alone. Both nodes now read the sensors stream independently of `grab()`, polling well above the hardware ODR so that no sample is lost. Measured on a ZED X on an AGX Orin: median publish delay 411 ms -> 10.5 ms, output rate 92.6 -> 100.0 Hz, and the delay no longer depends on `general.grab_compute_capping_fps`. SVO and simulation are unchanged.
- Fixed `sensors.sensors_pub_rate` not being honored on cameras whose IMU does not run at the rate the ZED SDK advertises (stereo and mono). A ZED X One GS reports 400 Hz but delivers 200 Hz, so `sensors_pub_rate: 100` produced a steady 50 Hz. Both nodes now decimate against the measured sample interval instead of the advertised one. Cameras that report correctly now produce exactly the requested rate rather than the nearest sub-multiple of the real one (a ZED 2i publishes 100.0 Hz instead of 101.1 Hz).
- Reduced the CPU usage of both nodes by counting topic subscribers through the publisher instead of a ROS graph query, and by refreshing the sensors thread counts every 250 ms instead of on every poll. On an AGX Orin this cuts the sensors thread overhead from ~5% to ~2-3% of a CPU core, whether or not the IMU topics are subscribed. Two consequences: a new subscriber to a sensors topic can wait up to 250 ms for its first message, and a subscriber whose QoS is incompatible with the publisher no longer triggers publishing (it could never receive the data anyway).
- Added the `general.bus_type` parameter to the `ZedCameraOne` node (`AUTO` default, `USB`, `GMSL`, and with ZED SDK >= 5.5 `MIPI` and `HOLOSCAN`), used when the camera is selected with `general.camera_id`. The bus was hard-coded to `GMSL`, so a ZED X One Core on a MIPI capture card, or reached over Camera-over-Ethernet through a Holoscan sensor bridge, could not be opened by ID at all.
- Added the `pause_svo_rec` service (stereo and mono, `std_srvs/srv/SetBool`) to pause and resume an SVO recording without closing the file. It returns `success: false` when no recording is active, and the diagnostic reports `SVO Recording: PAUSED` while paused.
- Added the `general.max_system_clock_step_ms` parameter (stereo and mono) to set how fast the SDK follows a backward host-clock adjustment (NTP/PTP step, manual date change): `4.0` is the ZED SDK default and suits slew-only setups, a negative value makes step corrections land instantly as PTP deployments expect, and `0.0` freezes the offset captured on first use. Applied only when `general.sdk_timestamp_clock` is `SYSTEM_CLOCK`. Requires ZED SDK >= 5.3.
- Added the `general.sdk_timestamp_clock` parameter (stereo and mono) to select the clock behind every SDK timestamp: `SYSTEM_CLOCK` (default), `MONOTONIC_CLOCK`, or `MONOTONIC_RAW_CLOCK` (ZED SDK >= 5.4), which is immune to NTP/PTP frequency slewing as well as to clock steps. It replaces the boolean `general.sdk_use_monotonic_clock`, which is still read and still works, but now logs a deprecation warning.
- Added the `svo.encryption_key` parameter (stereo and mono) to encrypt the SVO files created by `start_svo_rec` (AES-256-CTR); playback was already supported through `svo.decryption_key`. It accepts a path to a 32-byte key file, a 64-character hex key or a passphrase, and is never written to the log. Requires ZED SDK >= 5.3 and OpenSSL at runtime.
- Added the positional tracking pose confidence, a [0,100] quality metric filled by `POSITIONAL_TRACKING_MODE::GEN_3` where 0 means tracking is lost. It is reported as `Pose confidence` in the diagnostic and published as the new `pose_confidence` field of `PosTrackStatus` on `~/pose/status`. Requires ZED SDK >= 5.4 and `zed_msgs` v5.5.
- Fixed the `distortion_model` and the distortion coefficients published in the `camera_info` topics. They were chosen from a hard-coded per-model table that the lens does not follow: a fisheye ZED X One was published as `rational_polynomial` instead of `equidistant`. Both nodes now read the model from the ZED SDK (>= 5.4) and fall back to the old heuristic on older versions. Rectified parameters are reported as `plumb_bob` with zero coefficients.
- Aligned the `zed_debug` configuration files with the `zed_wrapper` ones: five parameters were missing and `custom_object_detection.yaml` lagged four per-class tracking parameters behind. Both directories now declare the same parameters; only the debug-specific values differ.
- Fixed `general.sdk_use_monotonic_clock` sitting in the `debug` section of `zed_wrapper/config/common_mono.yaml` and `zed_debug/config/common_stereo.yaml`, where it was read under the wrong name and silently ignored.
- Added the missing `advanced.change_thread_priority` to `common_stereo.yaml` and to both `common_mono.yaml` files. It gates every other `advanced.*` parameter and defaults to `false`, so the thread scheduling policy and priorities listed next to it were never applied unless it was added by hand.
- Fixed the `sports ball` class in `custom_object_detection.yaml`, whose label was corrupted to `pesports ballrson` by a bad search and replace. The other 79 labels and every `model_class_id` were verified against the COCO 2017 order.
- Improved the per-class defaults of `custom_object_detection.yaml`: `is_grounded` is now set only for objects that really lie on the floor (it was set on tableware, food and laptops, whose 3D position was then projected onto the floor) and `is_static` only for objects fixed in the world, and the tracking timeout, 3D box bounds, velocity settings and confirmation and prediction times are now set per object type instead of being identical for all 80 classes.
- Fixed `object_detection.class_XXX.min_box_height_meters` being documented for every class but never read by the node, so the minimum 3D height was never applied.
- Added `CUSTOM_RFDETRLIKE_BOX_OBJECTS` and `CUSTOM_BOX_OBJECTS_AUTODETECT` as valid `object_detection.detection_model` values (ZED SDK >= 5.5). They were accepted by the parameter parser but treated as built-in models, so the ONNX file was never passed to the ZED SDK and enabling object detection failed.
- Fixed the ONNX input shape sent to the ZED SDK for custom object detection models: `object_detection.custom_onnx_input_size` produced a `size x 0` resolution instead of a square one, so the shape never matched the one a model had been optimized with and the engine had to be rebuilt.
- Added the camera health status to the diagnostic output: image quality, lighting, depth reliability, motion sensors reliability and, with ZED SDK >= 5.5 and `zed_msgs` v5.5, duplicated images. The diagnostic goes to WARN and names the issues while any of them is raised, and reports `DISABLED` when `general.enable_image_validity_check` is off.
- Added the `duplicated_image` field to the `HealthStatusStamped` message published on `~/status/health`, raised by the ZED SDK when a frame repeats the content of an earlier one under a new timestamp, i.e. a repeating or stalled stream. Requires ZED SDK >= 5.5 and `zed_msgs` v5.5; with an older `zed_msgs` the node runs as before, without the field.
- Added the `general.timestamp_reference` parameter (`IMAGE` default, `IMAGE_CENTER_OF_EXPOSURE`) to both nodes. `IMAGE_CENTER_OF_EXPOSURE` stamps the published data with the middle of the frame exposure instead of the start of the sensor readout, which is what aligns frames with sensors timestamped at the instant they measure (LiDAR, robot joints). It needs an input carrying a per-frame exposure (ZED X, ZED X Mini, ZED X One GS, ZED X One 4K): on USB models the node warns once and falls back to `IMAGE`. Requires ZED SDK >= 5.5.
- Changed the default `body_tracking.body_format` from `BODY_38` to `BODY_34`, and `body_tracking.enable_body_fitting` from `false` to `true`. Only `BODY_18` and `BODY_34` have a `GEN_2` network, so the previous default silently fell back to the older `GEN_1`. Skeletons published on `~/body_trk/skeletons` now carry 34 keypoints instead of 38: set `body_format` back to `BODY_38` to keep the previous topology, at the cost of running `GEN_1`.
- Added the `body_tracking.model_gen` parameter (`GEN_1`, `GEN_2`; default `GEN_2`) to select the body tracking network. `GEN_2` is more robust in crowded scenes and to unusual body orientations, detects people further from the camera, and is up to 18% faster and 14% more accurate than `GEN_1`. `body_tracking.allow_reduced_precision_inference` has no effect on `GEN_2`, which always runs in FP16. Requires ZED SDK >= 5.5.
- Added the `pos_tracking.compute_preference` parameter (`AUTO` default, `PREFER_CPU`, `PREFER_GPU`) to control how much GPU the positional tracking may use: `PREFER_GPU` makes tracking faster and shortens `grab()`, `PREFER_CPU` keeps the GPU free for your own workloads. `GEN_1` computes depth and so uses the GPU whatever the setting. Requires ZED SDK >= 5.5.
- Added the `depth.allow_depth_cuda_graph` parameter (default `false`): the ZED SDK records the depth computation once and replays it at every grab, lowering the CPU cost of launching it, which matters most on Jetson. The depth output is unchanged. It applies to the `NEURAL` depth mode family only, and the SDK reverts to the regular computation if the recording fails. Requires ZED SDK >= 5.5.
- Added the `depth.depth_precision` parameter to select the precision of the neural depth inference: `FP16` (default, unchanged behavior) or `INT8`, which is faster and uses less GPU memory for a small accuracy cost. `INT8` requires `depth.depth_mode: 'NEURAL'`; the ZED SDK falls back to `FP16` when the depth mode or the GPU does not support it. Requires ZED SDK >= 5.5.
- Fixed a `zed_msgs` version check that blocked `scene_illuminance` information in health status messages.
- Fixed `scene_illuminance` reporting a value frozen at start-up in both the health status message and the `Scene Illuminance` diagnostic. It is now refreshed while grabbing and tracks the actual scene light level.
- Improved simulation mode to match the features of the ZED extension for NVIDIA Isaac Sim:

  - The grab resolution and framerate are no longer forced to `HD1080` @ 60 Hz, but read from the simulated camera, so a simulation running at the extension default of 30 FPS is reported and published correctly. This also removes the bogus "`general.grab_frame_rate` value is not valid" warning printed at every start-up, and the capture diagnostics reporting half the expected rate.
  - `general.pub_frame_rate: 0` ("no limit") now follows the real grab rate even when it is only known after the input is open.
  - Fixed the unstable IMU publishing rate in simulation: the stereo node now drains the IMU FIFO as the mono node already did, instead of polling only the latest sample. (Live cameras no longer take this path: see the IMU delay entry above.)
  - Fixed the simulation connection timeout never expiring when `use_sim_time:=true` and no `/clock` publisher was running yet. The timeout error now reports the configured `simulation.sim_address` / `simulation.sim_port` to check.
  - A simulated Virtual Stereo Camera (`camera_model:=virtual`) no longer requires `serial_numbers` or `camera_ids`: the simulator streams the already paired stereo images.
  - The "system overloaded" diagnostic now points to the simulator `FPS` / `Resolution` fields instead of node parameters that have no effect in simulation.
  - Silenced the spurious `SENSORS_NOT_AVAILABLE` warnings produced by a simulated camera without an IMU.
- Added simulation mode support for the ZED X One monocular cameras: launch `ZedCameraOne` with `sim_mode:=true` (plus the optional `sim_address`, `sim_port` and `use_sim_time` options) to publish images and IMU data from the simulated camera. With `use_sim_time` the node waits for a valid `/clock` and stamps every topic with the simulation time.

v5.4.1
------
- Fixed camera video settings (exposure, gain, white balance, brightness, etc.) not being reliably applied at start-up. The node now always enforces the configured values when it starts, instead of keeping whatever was left in the camera by a previous application (e.g. ZED Explorer or an earlier run). This fixes two identically-configured cameras showing different images, and the case where the image did not match the configured exposure/gain after opening the node. The apply is also retried if the camera is not ready yet (e.g. during USB bandwidth contention when several cameras are opened at once).
- Fixed unstable IMU publishing rate (issues #249 and #445). Sensor data is now retrieved by draining the full IMU FIFO with `getSensorsDataBatch()` instead of polling only the latest sample, so no sample is dropped and every published sample keeps its hardware timestamp. The drained stream is decimated to the requested `sensors.sensors_pub_rate` with a fractional accumulator that selects samples uniformly, so the parameter is still honored while the inter-sample interval stays constant. Measured on a ZED 2i: `sensors_pub_rate` 100/200/400 Hz yields a steady 101/202/404 Hz output with std-dev ~0.07 ms (vs ~1 ms before, with frequent missed-sample gaps up to ~12.7 ms).
- Fixed camera video settings (exposure, gain, white balance, brightness, etc.) not being reliably applied at start-up. The node now always enforces the configured values when it starts, instead of keeping whatever was left in the camera by a previous application (e.g. ZED Explorer or an earlier run). This fixes two identically-configured cameras showing different images, and the case where the image did not match the configured exposure/gain after opening the node. The apply is also retried if the camera is not ready yet (e.g. during USB bandwidth contention when several cameras are opened at once).
- Fixed unstable IMU publishing rate (issues #249 and #445). Sensor data is now retrieved by draining the full IMU FIFO with `getSensorsDataBatch()` instead of polling only the latest sample, so no sample is dropped and every published sample keeps its hardware timestamp. The drained stream is decimated to the requested `sensors.sensors_pub_rate` with a fractional accumulator that selects samples uniformly, so the parameter is still honored while the inter-sample interval stays constant. Measured on a ZED 2i: `sensors_pub_rate` 100/200/400 Hz yields a steady 101/202/404 Hz output with std-dev ~0.07 ms (vs ~1 ms before, with frequent missed-sample gaps up to ~12.7 ms).
- Added support for ROS 2 Lyrical Luth
- Improved Docker support:

  - Single parameterized `Dockerfile` with new `build_desktop.sh`/`build_jetson.sh` scripts, replacing the previous per-variant Dockerfiles.
  - Added Jetson cross-compilation from an x86_64 host via QEMU.
  - `--sdk-url` now also accepts a local ZED SDK `.run` file, not just a URL.

v5.4.0
----------
- Improved point cloud publishing by eliminating one full-cloud copy per published frame (~44 MB at HD2K, proportional at lower res) and removing a second full-cloud CPU allocation.
- Fixed node lock when calling the `set_svo_frame` service with Positional Tracking enabled
- Fixed Disparity Map data (bad pixel value sign).
- Changed disparity map publishing policy:

  - The topic `~/disparity/disparity_image` of type `stereo_msgs/msg/disparity_image` is now obsolete.  
  - Added two new topics:

    - `~/disparity/map` of type `stereo_msgs/msg/disparity_image` containing the Disparity Map for depth processing. This replaces the obsolete `~/disparity/disparity_image` topic.
      Use `ros2 run image_view disparity_view --ros-args -r image:=/zed/zed_node/disparity/disparity_image` to visualize this topic.
    - `~/disparity/image` of type `sensor_msgs/msg/image` containing the Normalized Disparity Map as an image for visualization purposes. You can subscribe to this topic in RViz2.

v5.3
----------
- Added voxel point cloud support. When `depth.voxel_point_cloud` is enabled, the point cloud topic publishes voxel-decimated data using `retrieveVoxelMeasure()`. New parameters: `depth.voxel_size_mm` (voxel cell size in millimeters), `depth.voxel_resolution_mode` (`FIXED`, `STEREO_UNCERTAINTY`, `LINEAR`), and `depth.voxel_resolution_scale`. All voxel parameters are dynamically reconfigurable.
- Added handling of `ERROR_CODE::CAMERA_EXCEEDS_BANDWIDTH` during camera open in both stereo and mono components. When a GMSL PHY CSI bandwidth overflow is detected, the node logs an error and stops initialization.
- Added `XVGA` and `TXGA` as valid `grab_resolution` options.
- Added `XVGA` and `TXGA` resolution parsing in both stereo (`ZedCamera`) and mono (`ZedCameraOne`) components.
- Added support for ZED X Nano camera: new `zedxnano` model with dedicated configuration file, URDF macro, and launch file integration.
- Added `video.ae_antibanding` parameter for ZED X cameras to control auto-exposure anti-banding mode (0=OFF, 1=AUTO, 2=50Hz, 3=60Hz). Dynamically reconfigurable.
- Added `Scene Illuminance` field to the diagnostic output for ZED X cameras (read-only metric reflecting the level of light in the scene).
- Added `scene_illuminance` field (int32, units 0.1 lux) to the `HealthStatusStamped` message published on `~/status/health`. Value -1 indicates the metric is unsupported by the connected camera. Requires the matching `zed_msgs` v5.3.
- Added `svo.svo_encoding_preset` parameter (`DEFAULT`, `ULTRAFAST`, `FAST`, `MEDIUM`, `SLOW`) to choose the SVO encoder speed/quality preset when starting a recording.
- Added `svo.decryption_key` parameter (stereo and mono) to open encrypted SVO files. The key/passphrase is forwarded to the SDK via `InitParameters::svo_decryption_key`.
- Added `general.sdk_use_monotonic_clock` parameter that switches the SDK to `TIMESTAMP_CLOCK::MONOTONIC_CLOCK` so timestamps are immune to host clock step adjustments (NTP/PTP). When combined with `debug.use_pub_timestamps: false`, an offset captured at camera open is applied so published Header stamps remain ROS-epoch-shaped while inter-frame deltas stay monotonic. Process-wide setting; in a composition the first node to open wins (subsequent nodes log a warning and fall back to the active clock).
- IPC is now handled automatically disabling it when NITROS is enabled and enabling it when NITROS is disabled. The `debug.disable_nitros` parameter can be used to disable NITROS and enable IPC if needed.
- Added support to `rclcpp::TypeAdapter` for better handling of Image messages:

  - A `TypeAdapter` publisher handles the base "raw" topic. Intra-process subscribers receive `StampedSlMat` (wrapping `sl::Mat`) directly without serialization. Inter-process subscribers receive `sensor_msgs/msg/Image` via automatic `TypeAdapter` conversion.
  - An `image_transport` publisher handles the transport-specific topics (e.g., `compressed`, `theora`). The "raw" transport is disabled to avoid duplicate messages.
  - Image transport plugins are now filtered by topic type: visual topics (IMAGE) only advertise `compressed` and `theora`, while measurement topics (MEASURE) only advertise `compressedDepth`. This prevents silent data corruption from incompatible plugin/encoding combinations (e.g., JPEG on float depth).
  - When the package `isaac_ros_nitros` is installed and NITROS not disabled via the `debug.disable_nitros` parameter, NITROS publishers take priority, and neither `TypeAdapter` nor `image_transport` publishers are created.

v5.2.2
----------
- Default Positional Tracking mode changed back to `GEN_1` until the stability and reliability of `GEN_3` is improved. 
  Users can still select a specific mode by setting the `pos_tracking.pos_tracking_mode` parameter to `GEN_1`, `GEN_2`, or `GEN_3` according to their needs and preferences.
- Modified node behaviors when Positional Tracking is disabled [`pos_tracking.pos_tracking_enabled: false`]:

  - `publish_tf` is automatically disabled.
  - The `odom` related topics are no longer advertised.
  - The `pose` related topics are no longer advertised.
  - The GNSS fusion is automatically disabled.
  - The Plane Detection is automatically disabled.
  - The Positional Tracking services are no longer advertised.
  - Depth stability follows the ZED SDK behaviors.
  - Object Tracking follows the ZED SDK behaviors.
  - Body Tracking follows the ZED SDK behaviors.

- Add new parameter `debug.debug_dyn_params` to enable debug logs for dynamic parameters changes. 

  - Dynamic parameters related logs are now displayed only if the new debug parameter `debug.debug_dyn_params` is set to `true` to avoid log spam when changing dynamic parameters.

v5.2.1
------
- Added the parameter `general.grab_compute_capping_fps` to define a computation upper limit to the grab frequency.

  - This can be useful to get a known constant fixed rate or limit the computation load while keeping a short exposure time by setting a high camera capture framerate.
  - If set to 0, the grab compute capping will be disabled, and the ZED SDK will process data at the grab rate.
- URDF now belongs to the `zed_description` package, which is now a dependency of the `zed_wrapper` package. This allows to use the URDF files of the ZED ROS2 Wrapper in other packages without depending on the whole wrapper.

  - The `zed_description` is available in binary form for ROS 2 Humble, Jazzy, and Rolling and can be installed with `sudo apt install ros-$ROS_DISTRO-zed-description`

v5.2.0
------
- Removed the `zed_wrapper/urdf/include/materials.urdf.xacro` file and moved the material settings directly in the `zed_macro.urdf.xacro` file to avoid possible conflicts in multi-camera configurations. Thx @davesarmoury for the fix
- Added the `enable_localization_only` parameter to the configuration to allow the camera to localize in the loaded area memory without updating the map with new information.
- Added support for the ZED SDK Positional Tracking 2D mode if the SDK version is 5.1 or higher.
- Added the `zed_debug` package for debugging ZED Components by loading them in a single C++ process.
- Add `enable_depth` service to disable depth processing at runtime
- Positional Tracking `GEN_3` is now the default mode when using ZED SDK v5.2 or newer, providing improved stability and performance. The `GEN_2` mode is still available as an option for users who prefer it or need it for specific use cases.
- When using GEN_3 with ZED SDK v5.2 or newer, Positional Tracking continues to provide localization feedback even if depth is disabled at runtime or when the node starts by setting the `depth.depth_mode` parameter to `NONE`.
- New diagnostic information regarding Positional Tracking status: "Mode", "Odometry Status", "Spatial Memory Status", "Tracking Fusion Status".

v5.1.0
------
- Changed ZED Camera image topic names to match the cleaner convention used by ZED X One cameras:
  
  - **NOTE** THIS IS A BREAKING CHANGE.
  - Left sensor topics:

    - From `~/left/image_rect_color` to `~/left/color/rect/image`
    - From `~/left_raw/image_raw_color` to `~/left/color/raw/image`
    - From `~/left_gray/image_rect_gray` to `~/left/gray/rect/image`
    - From `~/left_raw_gray/image_raw_gray` to `~/left/gray/raw/image`

  - Right sensor topics:

    - From `~/right/image_rect_color` to `~/right/color/rect/image`
    - From `~/right_raw/image_raw_color` to `~/right/color/raw/image`
    - From `~/right_gray/image_rect_gray` to `~/right/gray/rect/image`
    - From `~/right_raw_gray/image_raw_gray` to `~/right/gray/raw/image`

  - RGB sensor topics (corresponding to the left sensor for the Stereo cameras):

    - From `~/rgb/image_rect_color` to `~/rgb/color/rect/image`
    - From `~/rgb_raw/image_raw_color` to `~/rgb/color/raw/image`
    - From `~/rgb_gray/image_rect_gray` to `~/rgb/gray/rect/image`
    - From `~/rgb_raw_gray/image_raw_gray` to `~/rgb/gray/raw/image`

- Added parameters to select what topics will be advertised when the node starts:

  - **NOTE** THIS IS A BREAKING CHANGE. TOPICS MAY NO LONGER BE AVAILABLE IF NOT ENABLED IN THE DEFAULT CONFIGURATION. Please check what topic you use and set the relevant parameter to `true`.
  - `general.publish_status`: Advertise the status topics that are published only if a node subscribes to them
  - `video.publish_rgb`: Advertise the RGB image topics that are published only if a node subscribes to them
  - `video.publish_left_right`:  Advertise the left and right image topics that are published only if a node subscribes to them
  - `video.publish_raw`: Advertise the raw image topics that are published only if a node subscribes to them
  - `video.publish_gray`: Advertise the gray image topics that are published only if a node subscribes to them
  - `video.publish_stereo`: Advertise the stereo image topic that is published only if a node subscribes to it
  - `sensors.publish_imu`: Advertise the IMU topic that is published only if a node subscribes to it
  - `sensors.publish_imu_raw`: Advertise the raw IMU topic that is published only if a node subscribes to it
  - `sensors.publish_cam_imu_transf`: Advertise the IMU transformation topic that is published only if a node subscribes to it
  - `sensors.publish_mag`: Advertise the magnetometer topic that is published only if a node subscribes to it
  - `sensors.publish_baro`: Advertise the barometer topic that is published only if a node subscribes to it
  - `sensors.publish_temp`: Advertise the temperature topics that are published only if a node subscribes to them
  - `region_of_interest.publish_roi_mask`: Advertise the ROI mask image topic that is published only if a node subscribes to it
  - `depth.publish_depth_map`: Advertise the depth map topics that are published only if a node subscribes to them
  - `depth.publish_depth_info`: Advertise the depth info topic that is published only if a node subscribes to it
  - `depth.publish_point_cloud`: Advertise the point cloud topic that is published only if a node subscribes to it
  - `depth.publish_depth_confidence`: Advertise the depth confidence topic that is published only if a node subscribes to it
  - `depth.publish_disparity`: Advertise the disparity topic that is published only if a node subscribes to it
  - `pos_tracking.publish_odom_pose`: Advertise the odometry and pose topics that are published only if a node subscribes to them
  - `pos_tracking.publish_pose_cov`: Advertise the pose with covariance topic that is published only if a node subscribes to it
  - `pos_tracking.publish_cam_path`: Advertise the camera odometry and pose path topics that are published only if a node subscribes to them
  - `mapping.publish_det_plane`: Advertise the plane detection topics that is published only if a node subscribes to it

- Added topic enabler feature to `sl::CameraOne`

  - **NOTE** THIS IS A BREAKING CHANGE. TOPICS MAY NO LONGER BE AVAILABLE IF NOT ENABLED IN THE DEFAULT CONFIGURATION. Please check what topic you use and set the relevant parameter to `true`.
  - Added parameter `video.publish_rgb` to enable/disable RGB image publishing
  - Added parameter `video.publish_raw` to enable/disable raw image publishing
  - Added parameter `video.publish_gray` to enable/disable gray image publishing
  - Added parameter `sensors.publish_imu`: Advertise the IMU topic that is published only if a node subscribes to it
  - Added parameter `sensors.publish_imu_raw`: Advertise the raw IMU topic that is published only if a node subscribes to it
  - Added parameter `sensors.publish_temp`: Advertise the temperature topics that are published only if a node subscribes to them

- Enabled Isaac ROS NITROS integration for ZED X One cameras
- Added debug parameter `debug.debug_nitros` to enable debug logs for NITROS-related operations.
- Added debug parameter `debug.use_pub_timestamps` to use the current ROS time for the message timestamp instead of the camera timestamp.
  This is useful to test data communication latency.
- Added `camera_info` in transport namespace to reflect `rviz2` requirements with the Camera plugin.

  - Added new `camInfoPubTrans` publisher for each image topic to publish the `camera_info` in the transport namespace.
  - Updated `publishImageWithInfo` method to handle the new `camInfoPubTrans` publisher.

- Added 3D visualization of the positional tracking landmarks as a point cloud on topic `~/pose/landmarks` (only with GEN_2 and GEN_3 positional tracking modes):

  - Added parameter `pos_tracking.publish_3d_landmarks` to enable/disable landmarks publishing
  - Added parameter `pos_tracking.publish_lm_skip_frame` to set the frequency of landmarks publishing (0 to publish every frame)

- Removed Point Cloud Transport as a required dependency. Point Cloud Transport is now only automatically enabled if the `point_cloud_transport` package is installed on the system.
- Removed FFMPEG Image Transport support because of a problem with the Humble distribution not allowing to set the transport parameters, and the lack of compatibility with NVIDIA® Jetson.
- Improved diagnostic information for 3D Mapping status in diagnostics
- Fixed random crash when stopping 3D Mapping
- Fixed a bug that forced the maximum publishing rate of the point cloud topic to 15 Hz with SVO files
- Set the default mode for positional tracking to `GEN_2` waiting for improvements in `GEN_3` stability
- Remapped `robot_description` topic to `<camera_name>_description` to allow multi-camera URDF integration
- Changed minimum depth value to 0.01 meters when using ZED SDK v5.1 or higher
- Added debug option for TF broadcasting

  - Improved TF debug logs to show frame transformations when enabled

- Static baseline information from URDF is now overwritten by the real baseline value retrieved from the camera calibration file.
- Removed mandatory `custom_baseline` launch argument for virtual stereo cameras made with two ZED X One cameras.
  The value is retrieved from the calibration file.
- IMU TF is now broadcast as static if IPC is disabled.
- IMU Transform topic is now published with TRANSIENT LOCAL durability if IPC is disabled.
- Fixed `camera_info` publishing when no image topics are subscribed
- Loop Closure log event is now displayed only in DEBUG mode to reduce log spam
- Renamed camera optical frames to comply with ROS conventions:

  - **NOTE** THIS IS A BREAKING CHANGE. Please update your TF references accordingly.
  - From `<camera_name>_left_camera_optical_frame` to `<camera_name>_left_camera_frame_optical`
  - From `<camera_name>_right_camera_optical_frame` to `<camera_name>_right_camera_frame_optical`
  - From `<camera_name>_camera_optical_frame` to `<camera_name>_rgb_camera_frame_optical`

- Added twist information to the `odom` topic
- Added support for the new Virtual Stereo API with SDK v5.1.

  - New launch arguments to setup the virtual camera: `serial_numbers` and `camera_ids`
  - New `ZedCamera` component parameters to setup the virtual camera: `general.virtual_serial_numbers` and `general.virtual_camera_ids`
  - **NOTE** ZED MEDIA SERVER IS NO LONGER REQUIRED to create a virtual Stereo camera using two ZED X One cameras.

- Added 24-bit BGR image mode

  - Added parameter `video.enable_24bit_output` to enable/disable 24-bit BGR image publishing to `common_stereo.yaml` and `common_mono.yaml` configuration files
  - **NOTE**: `video.enable_24bit_output` is disabled by default to maintain backward compatibility. Enabling this parameter will change the image message encoding from `BGRA8` to `BGR8`, which may affect existing applications that rely on the previous encoding.

- Enabled SVO support for ZED X One cameras (playback, recording, and diagnostic)
- Set thread names according to the thread function name for easier identification in debuggers and profilers
- Enable SVO for ZedCameraOne

  - add service to pause SVO playback
  - add service to set SVO frame ID
  - add services to start/stop SVO recording
  - publish SVO status
  - publish SVO clock on `/clock` topic

- Publish ZedCameraOne heartbeat status on `~/status/heartbeat` topic

v5.0.0
------
- Backward compatible with SDK v4.2
- Added official support for ROS 2 Jazzy Jalisco
- Note: requires the latest `zed_msgs` package v5.0.0
- Added SVO Status topic to monitor the current SVO status of type `zed_msgs::SvoStatus`
- Added fully integrated Health Status topic of type `zed_msgs::HealthStatusStamped`

  - Remove the single health status topics to simplicy health monitoring

- Remove `cob_srvs` dependency to use the custom `zed_msgs::SetSvoFrame` service
- Added Heartbeat status message at 1 Hz: `~/status/heartbeat`
- Improve performance with the default stereo configuration
- Fix Positional Tracking enabling when required by ZED SDK modules
- Fix realtime IMU data publishing when using SVO2
- Added parameter 'debug.sdk_verbose_log_file' to Stereo and Mono components to set the path of the SDK verbose log file
- Clean shutdown of ZED components using `pre_shutdown_callback`
- Added new parameter `svo.replay_rate` to set the replay rate for the SVO when not used in realtime mode (range [0.10-5.0])
- Improved diagnostic information for SVO playback
- Default SVO Recording Compression mode [`0`] is forced to `H265` replacing the old `LOSSLESS` mode

  - H265 is far superior as it uses hardware encoder, resulting in faster, lighter encoding, and dramatically smaller SVO2 files

- Added `/clock` publisher to be used in SVO Mode to synchronize other nodes with the SVO timestamp
- Added parameter `svo.publish_svo_clock` to enable the `/clock` publisher

  - The parameter 'svo.publish_svo_clock' is normally overridden by the `publish_svo_clock` launch option

- Moved `brightness`, `contrast`, and `hue` from `common_stereo.yaml` to `zed.yaml`, `zed2.yaml`, `zed2i.yaml`, and `zedm.yaml` files
- Add advanced handling of the Object Detection and Tracking module of the ZED SDK

  - Move the multi-box native object detection parameters to the `object_detection.yaml` file
  - Add specific parameters to set the confidence threshold for each of the includes object detection classes of the ZED SDK
  - Move the Custom Object Detection parameters to the `custom_object_detection.yaml` file
  - Support all the new parameters of the ZED SDK v5 separately for each of the custom object detection classes

- The usage of the new Object Detection support is fully described on the ZED ROS 2 online documentation:

  - Object Detection: https://docs.stereolabs.com/ros2/object-detection/
  - Custom Object Detection: https://docs.stereolabs.com/ros2/custom-object-detection/

- Separated Video/Depth data publishing into its own thread for more precise control over the publishing rate, 
  independent of the camera grab rate. This enables recording SVO files or processing positional tracking at 
  full grab rate, while publishing data at a reduced rate to optimize bandwidth usage.
- Added a new launch option 'node_log_type' to set the type of log to be used by the ZED Node.

  - The available options are `screen`, `log`, and `both`.

- Changed `pos_tracking.area_memory_db_path` to `pos_tracking.area_file_path` to match the ZED SDK parameter name
- Added parameter `pos_tracking.save_area_memory_on_closing` to save the Area Memory before closing the camera
- Fixed Area Mapping file handling according to the ZED SDK policies.
  
  - The Area Memory file is now saved only if the Area Memory is enabled, if the `pos_tracking.save_area_memory_on_closing` 
  parameter is set to `true`, if the `pos_tracking.area_file_path` is set and if the `pos_tracking.area_file_path` is valid.

- Added `save_area_memory` service
  
  - Set the filename as a parameter. If the filename is empty, it uses the value of the parameter `pos_tracking.area_file_path` if not empty.

- Added `enable_ipc` launch argument to enable intra-process communication (IPC) when using ROS 2 Composition. 
  
  - Note: NITROS requires IPC to be disabled to work properly.

- Fixed plane topic names, adding missing node name prefix
- Added camera_info to Confidence Map topic
- Enabled Isaac ROS integration and automatic NITROS usage: https://docs.stereolabs.com/isaac-ros/
  
  - Added the parameter `debug.disable_nitros` to disable NITROS usage. This is useful for debugging and testing purposes.

v4.2.5
------
- Add new parameter 'depth.point_cloud_res' to set a specific point cloud publishing resolution
  
  - 'COMPACT': Standard resolution. Optimizes processing and bandwidth
  - 'REDUCED': Half 'COMPACT' resolution. Low processing and low bandwidth requirements

- Add uptime and frame drop rate information to node diagnostics
- Add image validity check support [SDK 5 required]
  
  - Add new parameter 'general.enable_image_validity_check'
  - Add new topic 'health_status/low_image_quality' to publish image quality status
  - Add new topic 'health_status/low_lighting' to publish low light condition status
  - Add new topic 'health_status/low_depth_reliability' to publish low depth quality status
  - Add new topic 'health_status/low_motion_sensors_reliability' to publish low quality of inertial sensors status
  - Set the Node Disgnostic to WARNING if any of the above conditions are detected

- Add `general.camera_id` parameter to set the camera ID for the ZedCamera. 
- Add `general.camera_id` parameter to set the camera ID for the ZedCameraOne.
- Add `camera_id` argument to the `zed_camera.launch.py` launch file. Useful for GMSL2 multi-camera configurations where camera ID is estabilished by the GMSL2 wire.
- Improve Node Diagnostics information
- Add `pos_tracking.reset_pose_with_svo_loop` parameter to reset the camera pose the `initial_base_pose` when the SVO loop is enabled and the SVO playback reaches the end of the file.
- Add `svo.play_from_frame` parameter to set the starting frame when playing an SVO file.
- Add `set_svo_frame` service to set the current frame when playing an SVO file.
- Remove unused open timeout for ZED X One cameras
- Add parameter `svo.use_svo_timestamps` to use the SVO timestamps when publishing data (both stereo and mono components)

v4.2.x
------
- Add new `OPTIMIZED` mode for `general.pub_resolution`
- Add new parameter `general.async_image_retrieval` to enable/disable the asynchronous image retrieval to be used with SVO recording.
- Set the Positional Tracking Mode to `GEN_1` as default as wa orkaround for the random crash issue caused by `GEN_2` mode.
- Fixed a bug for raw gray image publisher on Zed One Component: raw gray images were not published when the rectified image topic was subscribed. Thx @Alex-Beh 
- Enabled grayscale output for ZED X One cameras (SDK v4.2.3 required)
- Enabled streaming input for ZED X One cameras (SDK v4.2.3 required)
- Fixed wrong range check for the `general.pub_downscale_factor` parameter
- Enhanced sensor thread rate due to an automatically adjusting sleep time
- Removed the `zed-ros2-interfaces` sub-module. The `zed_msgs` package is now included in ROS 2 Humble binaries and can be installed with `sudo apt install ros-humble-zed-msgs`.
- Fixed 4K resolution support for ZED X One 4K cameras
- Changed C++ version to 17 to follow ROS 2 Humble standard
- Renamed `common.yaml` to `common_stereo.yaml`
- Added `common_mono.yaml` for monocular cameras
- Added `video.enable_hdr` to `zedxone4k.yaml` for monocular 4K cameras
- Changed the name of the package `zed_interfaces` to `zed_msgs` to match the ROS 2 naming convention
- Added the new `stereolabs::ZedCameraOne` component to handle ZED X One cameras
- Removed the ZED Wrapper executable node.

  - Modified the launch file to create an isolated composable container that loads the `stereolabs:ZedCamera` or the `stereolabs:ZedCameraOne` component according to the camera model  

- Added support for custom ONNX detection engine (SDK v4.2 required)
  
  - Added value `CUSTOM_YOLOLIKE_BOX_OBJECTS` to the `object_detection.model` parameter
  - Added parameter `object_detection.custom_onnx_file` to set the full path of the custom ONNX file
  - Added parameter `object_detection.onnx_input_size` to set the size of the YOLO input tensor
  - Added parameter `object_detection.custom_label_yaml` to set the full path to custom YAML file storing class labels in [COCO format](https://docs.ultralytics.com/datasets/detect/coco/#dataset-yaml)

v4.1.x
------
- Updated the Docker files to the CUDA 12.4 (PC), L4T 35.4 (Jetson), SDK v4.1.4
- Added Local Streaming output

  - Added `enable_streaming` service to start/stop a streaming server
  - Added Streaming Server diagnostic
  - Added parameter 'stream_server.stream_enabled': enable the streaming server when the camera is open
  - Added parameter 'stream_server.codec': different encoding types for image streaming
  - Added parameter 'stream_server.port': Port used for streaming
  - Added parameter 'stream_server.bitrate': Streaming bitrate (in Kbits/s) used for streaming
  - Added parameter 'stream_server.gop_size': The GOP size determines the maximum distance between IDR/I-frames
  - Added parameter 'stream_server.adaptative_bitrate': Bitrate will be adjusted depending on the number of packets dropped during streaming
  - Added parameter 'stream_server.chunk_size': Stream buffers are divided into X number of chunks where each chunk is chunk_size bytes long
  - Added parameter 'stream_server.target_framerate': Framerate for the streaming output

- Added Local Streaming input

  - Added 'stream.stream_address' and 'stream.stream_port' parameter to configure the local streaming input

- GNSS Fusion temporarily disabled *(available with 4.1.1)*
- Moved parameter 'general.svo_file' to 'svo.svo_path'
- Moved parameter 'general.svo_loop' to 'svo.svo_loop'
- Moved parameter 'general.svo_realtime' to 'svo.svo_realtime'
- Removed obsolete launch files: 'zed.launch.pi','zed2.launch.pi', 'zed2i.launch.pi', 'zedm.launch.pi', 'zedx.launch.pi', 'zedxm.launch.pi'
- Removed obsolete display launch file: 'display_zed.launch.py', 'display_zed2.launch.py', 'display_zed2i.launch.py', 'display_zedm.launch.py', 'display_zedx.launch.py', 'display_zedxm.launch.py'
- Added support for custom virtual stereo cameras made with two calibrated ZED X One cameras *(available with 4.1.1)*
- Added parameter `pos_tracking.reset_odom_with_loop_closure` to automatically reset odometry when a loop closure is detected
- Added new positional tracking information to the `PosTrackStatus` message
- Added new `GnssFusionStatus` message with GNSS Fusion status information *(available with 4.1.1)*
- Added new parameters `gnss_fusion.h_covariance_mul` and `gnss_fusion.v_covariance_mul` to control the effects of the GNSS covariance
- Added support to Automatic ROI

  - Added ROI diagnostic
  - Added parameter `debug.debug_roi`
  - Publish ROI mask image on the topic `~/roi_mask` using image transport
  - Moved the parameter `general.region_of_interest` to `region_of_interest.manual_polygon`
  - Added automatic Region of Interest support
  - Added parameter `region_of_interest.automatic_roi`
  - Added parameter `region_of_interest.depth_far_threshold_meters`
  - Added parameter `region_of_interest.image_height_ratio_cutoff`
  - Added parameter `region_of_interest.apply_to_depth`
  - Added parameter `region_of_interest.apply_to_positional_tracking`
  - Added parameter `region_of_interest.apply_to_object_detection`
  - Added parameter `region_of_interest.apply_to_body_tracking`
  - Added parameter `region_of_interest.apply_to_spatial_mapping`

- Removed QoS parameters to use ROS 2 QoS overwrite -> https://design.ros2.org/articles/qos_configurability.html
- Added support for new `NEURAL_PLUS` depth mode
- Added new `<camera_name>_gnss_link` frame to URDF to set the position of the GNSS antenna with respect to the camera position
- New Docker configuration files allow to easily create "ZED ROS2 Wrapper" images based on specific tag versions. [Read more](./docker/README.md)
- Fixed a bug while playing a ZED X stream on a "not-Jetson" host device
- Add support for point cloud transport [only Humble, no Foxy]
- Add support for FFMPEG image transport
- Add new `ffmpeg.yaml` configuration file
- Fix `~/imu/data_raw` message not containing RAW IMU data

v4.0.8
------
- The parameter `general.sdk_verbose` has been moved to `debug.sdk_verbose` and set to `0` as default.
- Added new parameter `general.optional_opencv_calibration_file` to use custom OpenCV camera calibrations.
- Added [new tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_robot_integration) to illustrate how to integrate one or more ZED cameras on a robot
- Added 'simulation.sim_enabled' parameter to enable the simulation mode
- Added 'simulation.sim_address' parameter to set the simulation server address
- Added 'simulation.sim_port' parameter to set the simulation server port
- Added `/clock` subscriber to check the presence of the required message when `use_sim_time` is true
- Force `grab_frame_rate` and `pub_frame_rate` to 60 Hz in simulation
- Force `grab_resolution` to `HD1080` in simulation
- Removed the `general.zed_id` parameter. Always use `general.serial_number` to distinguish between different cameras in a multi-camera configuration.
- The multi-camera example has been updated to match the new TF configuration
- The old launch files are now obsolete: 'ros2 launch zed_wrapper <camera_model>.launch.py' is replaced by 'ros2 
  launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>'
- The reference link for positional tracking is no longer 'base_link' but `<camera_name>_camera_link`. 
  This will allow an easier ZED integration in existing robot configuration because the transform `base_link` -> `camera_link` 
  is no longer published by the ZED ROS2 Wrapper. Thanks to @SteveMacenski for the advice

  - Removed `parent` and `origin` parameters from `zed_macro.urdf.xacro`
  - Removed launch argument `cam_pose` from `zed_camera.launch.py`

- Moved parameter `publish_imu_tf` from `pos_tracking` to `sensors` to make it available also in "no depth" configurations of the node
- Added new parameter `pos_tracking.pos_tracking_mode` to exploit the new ZED SDK `QUALITY` mode for improved odometry and localization
- New Video/Depth processing throttling method by using the `grab_compute_capping_fps` ZED SDK parameter instead of a dedicated thread
- Advanced parameters to handle Thread scheduling policy and priorities (sudo required):`thread_sched_policy`,`thread_grab_priority`,
  `thread_sensor_priority`,`thread_pointcloud_priority`
- Added new GNSS calibration parameters: `enable_reinitialization`, `enable_rolling_calibration`, `enable_translation_uncertainty_target`, `gnss_vio_reinit_threshold`, `target_translation_uncertainty`, `target_yaw_uncertainty`
- Added new Plane Detection parameters: `pd_max_distance_threshold`, `pd_normal_similarity_threshold`

v4.0.5
----------
- The parameter `general.pub_resolution` can now take only `NATIVE` and `CUSTOM` values. 'NATIVE' to use the same `general.grab_resolution` - `CUSTOM` to apply the `general.pub_downscale_factor` downscale factory to reduce bandwidth in transmission
- Added new parameter `general.pub_downscale_factor` to be used with the new option `CUSTOM` for the parameter `general.pub_resolution`
- `ULTRA` is the new default value for `depth.depth_mode` (better performance for odometry and positional tracking)
- Added resolution `HD1080` for ZED X
- Fix issue with Body Tracking start/stop by service call. Now Body Tracking can be restarted multiple times
- Fix depth grab performance by removing a [not required `PNG Write` call](https://github.com/stereolabs/zed-ros2-wrapper/pull/164). Thank you Esteban Zamora @ezamoraa 
- Fix bug with `general.pub_resolution` value, not allowing to select the correct data publish resolution
- Added new launch parameter `ros_params_override_path` to provide the path to a custom YAML file to override the parameters of the ZED Node without modifying the original files in the `zed_wrapper/config` folder. Thank you David Lu @MetroRobots

v4.0.0
------
- Added support for ZED-X and ZED-X Mini

  - Moved `general.grab_resolution` and `general.grab_frame_rate` to the yaml file specific for the relative camera model (i.e. `zed.yaml`, `zedm.yaml`, `zed2.yaml`, `zed2i.yaml`, `zedx.yaml`, `zedxm.yaml`)

  - Added `zedx.launch.py` for ZED-X
  - Added `zedxm.launch.py` for ZED-X Mini
  - Improve `zed_macro.urdf.xacro` with specific configuration for the new camera models
  - Added `display_zedx.launch.py` for ZED-X to ZED-ROS2-Examples
  - Added `display_zedxm.launch.py` for ZED-X Mini to ZED-ROS2-Examples
  - Added ZED-X and ZED-X Mini STL files to ZED-ROS2-Interfaces

- Positional Tracking

  - Added `pos_tracking.set_as_static` parameters for applications with a static camera monitoring a robotics environment. See [PR #122](https://github.com/stereolabs/zed-ros2-wrapper/pull/122 ) Thx @gabor-kovacs
  - Added custom message type `PosTrackStatus`
  - Publish message on topic `~/pose/status` with the current status of the pose from the ZED SDK
  - Publish message on topic `~/odom/status` with the current status of the odometry from the ZED SDK

- Body Tracking

  - Added Support for the new Body Tracking module
  - Added parameter `body_tracking.bt_enabled` to enable Body Tracking
  - Added parameter `body_tracking.model` to set the AI model to be used
  - Added parameter `body_tracking.body_format` to set the Body Format to be used
  - Added parameter `body_tracking.allow_reduced_precision_inference` to improve performances
  - Added parameter `body_tracking.max_range` to set the max range for Body Detection
  - Added parameter `body_tracking.body_kp_selection` to choose the Body key points to be used
  - Added parameter `body_tracking.enable_body_fitting` to enable body fitting
  - Added parameter `body_tracking.enable_tracking` to enable the tracking of the detected bodies
  - Added parameter `body_tracking.prediction_timeout_s` to set the timeout of the prediction phase while tracking
  - Added parameter `body_tracking.confidence_threshold` to set the detection confidence threshold
  - Added parameter `body_tracking.minimum_keypoints_threshold` to set the minimum number of detected key points to consider a body valid
  - Publish new message on topic `~/body_trk/skeletons`
  - Added service `enable_body_trk` to start/stop body tracking

- GNSS fusion integration

  - New param `gnss_fusion.gnss_fusion_enabled` to enable GNSS fusion
  - New param `gnss_fusion.gnss_fix_topic` name of the topic containing GNSS Fix data of type `sensor_msgs/NavSatFix`
  - Added `nmea_msgs` dependency
  - Added GNSS Fix Diagnostic
  - Added new launch parameter `gnss_frame` to enable the GNSS link in the ZED URDF
  - Added new node parameter `gnss_fusion.gnss_zero_altitude` to ignore GNSS altitude information
  - Added new node parameter `gnss_fusion.gnss_frame` to set the name of the frame link of the GNSS sensor
  - Disable Area Memory (loop closure) when GNSS fusion is enabled
  - Added services `toLL` and `fromLL` to use the ZED ROS2 Wrapper with the Nav2 Waypoint Navigation package
  - Added `geographic_msgs::msg::GeoPoseStamped` message publisher
  - Added parameter `gnss_fusion.publish_utm_tf`
  - Added parameter `gnss_fusion.broadcast_utm_transform_as_parent_frame`
  - Added parameter `gnss_fusion.gnss_init_distance`
  - Publish message on topic `~/geo_pose/status` with the current status of the GeoPose from the ZED SDK
  - Publish message on topic `~/pose/filtered` with the current GNSS filtered pose in `map` frame
  - Publish message on topic `~/pose/filtered/status` with the current status of the GNSS filtered pose from the ZED SDK

- Object Detection

  - Added `object_detection.allow_reduced_precision_inference` to allow inference to run at a lower precision to improve runtime and memory usage
  - Added `object_detection.max_range` to defines a upper depth range for detections
  - Removed `object_detection.body_format`

- Docker

  - Added Docker files (see `docker` folder) ready to create Docker images for desktop host devices

- Examples/Tutorials

  - Added multi-camera example in `zed-ros2-examples` repository.

- Added full Terrain Mapping (local obstacle detection) support [EXPERIMENTAL FEATURE AVAILABLE ONLY FOR BETA TESTERS]

  - ZED SDK Terrain Mapping published as GridMap message
  - Added parameter `local_mapping.terrain_mapping_enabled` to enable terrain mapping publishing a local obstacle map
  - Added parameter `local_mapping.data_pub_rate` to set the Local Map data publish frequency
  - Added parameter `local_mapping.grid_resolution` to set the Local Map resolution in meters [min: 0.01 - max: 1.0]
  - Added parameter `local_mapping.grid_range` to set the maximum depth range for local map generation [min: 1.0 - max: 8.0]
  - Added parameter `local_mapping.height_threshold` to set the maximum height for obstacles
  - Publish gridmap on topic `local_map/gridmap`
  - Publish elevation map image on topic `local_map/elev_img`
  - Publish obstacle color map image on topic `local_map/col_img`
  - Added traversability cost computation for Terrain Mapping (local_mapping)

    - Change parameter `local_mapping.height_threshold` to `local_mapping.robot_heigth`
    - Added parameter `local_mapping.robot_radius` to set radius of the robot
    - Added parameter `local_mapping.robot_max_step` to set max height of a step that the robot can overcome
    - Added parameter `local_mapping.robot_max_slope` to set max slope (degrees) that the robot can overcome
    - Added parameter `local_mapping.robot_max_roughness` to set max roughness of the terrain that the robot can overcome

- Added support for simulated data [EXPERIMENTAL FEATURE AVAILABLE ONLY FOR BETA TESTERS]

  - Added parameter `use_sim_time` to enable SIMULATION mode
  - Added parameter `sim_address` tos set the local address of the machine running the simulator
  - Change StopWatch to use ROS clock instead of System Clock. In this way diagnostic and time checking work also in simulation
  - Disable camera settings control in simulation

- Others

  - Removed `sensing_mode`, no more available in SDK v4.0
  - Removed `extrinsic_in_camera_frame`, no more available in SDK v4.0
  - Added `zed_id` and `serial_number` launch parameters to open the correct camera in multi-camera configurations.
  - Code lint and re-formatting according to [ROS2 code rules](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
  - Added support for automatic lint tools to all the packages.
  - Removed node parameter `general.resolution`, replaced by `general.grab_resolution`.
  - Added node parameter `general.pub_resolution` used to reduce node computation and message bandwidth.

    - Available output resolutions: `HD2K`, `HD1080`, `HD720`, `MEDIUM`, `VGA`. `MEDIUM` is an optimized output resolution to maximize throughput and minimize processing costs.
  
  - Removed node parameters `video.img_downsample_factor` and `depth.depth_downsample_factor`. Use the new parameter `general.pub_resolution` instead.
  - Change `general.grab_resolution` and `general.pub_resolution` from integer to string.
  - Added new `LOW` value for `general.pub_resolution` (half the `MEDIUM` output resolution).
  - Removed `depth.quality` parameter (replaced with `depth.depth_mode`)
  - Added `depth.depth_mode` parameter: a string reflecting the ZED SDK `DEPTH_MODE` available value names
  - The parameter `depth.depth_stabilization` is now an integer in [0,100] reflecting ZED SDK behavior
  - Fix distortion model (see Issue [#128](https://github.com/stereolabs/zed-ros2-wrapper/issues/128))
  - Improve the code for Moving Average calculation for better node diagnostics.
  - Temperature diagnostic is now always updated even if `sensors.sensors_image_sync` is true and no image topics are subscribed.
  - Improve Grab thread and Video/Depth publishing thread elaboration time diagnostic.
  - Added a check on timestamp to not publish already published point cloud messages in the point cloud thread
  - Improve thread synchronization when the frequency of the `grab` SDK function is minor of the expected camera frame rate setting because of a leaking of elaboration power.
  - Added diagnostic warning if the frequency of the camera grabbing thread is minor than the selected `general.grab_frame_rate` value.
  - Removed annoying build log messages. Only warning regarding unsupported ROS 2 distributions will be displayed when required.
  - Convert `shared_ptr` to `unique_ptr` for IPC support
  - Improve the `zed_camera.launch.py`

    - Added support for `OpaqueFunction` in order to automatically configure the launch file according to the value of the launch parameter `cam_model`.
    - Change parameters to set camera pose in launch files. From 6 separated parameters (`cam_pos_x`,`cam_pos_y`,`cam_pos_z`,`cam_roll`,`cam_pitch`,`cam_yaw`) to one single array (`cam_pose`).
    - Removed the workaround for empty `svo_path` launch parameter values thanks to `TextSubstitution`.
    - Modify the "display" launch files in [zed-ros2-examples](https://github.com/stereolabs/zed-ros2-examples) to match the new configuration.
    - Added `publish_tf` and `publish_map_tf` launch parameters useful for multi-camera configuretion or external odometry fusion.
  
  - Change LICENSE to Apache 2.0 to match ROS 2 license.

v3.8.x
------
- Fixed `set_pose` wrong behavior. Now initial odometry is coherent with the new starting point.
- Added Plane Detection.
- Fixed "NO DEPTH" mode. By setting `depth/quality` to `0` now the depth extraction and all the sub-modules depending on it are correctly disabled.
- Added `debug` sub-set of parameters with new parameters `debug_mode` and `debug_sensors`.
- Added support for ROS 2 Humble. Thx @nakai-omer.
  The two ROS 2 LTS releases are now supported simoultaneously.
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
- Added new parameter `pos_tracking.transform_time_offset` to fix odometry TF timestamp issues
- Added new parameter `pos_tracking.depth_min_range` for removing fixed zones of the robot in the FoV of the camerafrom the visual odometry evaluation
- Added new parameter `pos_tracking.sensor_world` to define the world type that the SDK can use to initialize the Positionnal Tracking module
- Added new parameter `object_detection.prediction_timeout` for setting the timeout time [sec] of object prediction when not detected.
- Added support for ZED SDK Regiorn of Interest:

  - Added parameter `general.region_of_interest` to set the region of interest for SDK processing.
  - Added the service `resetRoi` to reset the region of interest.
  - Added the service `setRoi` to set a new region of interest.

v3.7.x
----------
- Added support for sport-related OD objects
- Added `remove_saturated_areas` dynamic parameter to disable depth filtering when luminance >=255
- Added `sl::ObjectDetectionParameters::filtering_mode` parameter
- Publish `depth_info` topic with current min/max depth information
- Fix parameter override problem (Issue #71). Thx @kevinanschau
- Added default xacro path value in `zed_camera.launch.py`. Thx @sttobia
- Fix `zed-ros2-interfaces` sub-module url, changing from `ssh` to `https`.

v3.6.x (2021-12-03)
-------------------
- Moved the `zed_interfaces` package to the `zed-ros2-interfaces` repository to match the same configuration of the ROS1 wrapper
- The `zed-ros2-interfaces` repository has been added as a sub-module to this repository
- Added new <zed>_base_link frame on the base of the camera to easily handle camera positioning on robots. Thx @civerachb-cpr
- Improve URDF by adding 3° slope for ZED and ZED2, X-offset for optical frames to correctly match the CMOS sensors position on the PCB, X-offset for mounting screw on ZED2i
- Added `zed_macro.urdf.xacro` to be included by other xacro file to easily integrate ZED cameras in the robot descriptions. See ROS1 PR [#771](https://github.com/stereolabs/zed-ros-wrapper/pull/771) for details. Thx @civerachb-cpr
- Fix URDF `height` value for ZED, ZED2 and ZED2i
- Fix performances drop on slower platforms. Thx @roncapat
- Fix SVO LOOP wrong behavior. Thx @kevinanschau
- Added xacro support for automatic URDF configuration
- Reworked launch files to support xacro and launch parameters

    - Use `ros2 launch zed_wrapper <launch_file> -s` to retrieve all the available parameters

- Added `svo_path:=<full path to SVO file>` as input for all the launch files to start the node using an SVO as input without modifying 'common.yaml`
- Improved diagnostic information adding elaboration time on all the main tasks
- Improved diagnostic time and frequencies calculation
- Added StopWatch to sl_tools
- Enabled Diagnostic status publishing
- Changed the default values of the QoS parameter reliability for all topics from BEST_EFFORT to RELIABLE to guarantee compatibility with all ROS 2 tools
- Fixed tab error in `zedm.yaml`
- Fixed compatibility issue with ZED SDK older than v3.5 - Thanks @PhilippPolterauer
- Migration to ROS 2 Foxy Fitzroy

v3.5.x (2021-07-05)
-------------------
- Added support for SDK v3.5
- Added support for the new ZED 2i
- Added new parameter `pos_tracking/pos_tracking_enabled` to enable positional tracking from start even if not required by any subscribed topic. This is useful, for example, to keep the TF always updated.
- Added support for new AI models: `MULTI_CLASS_BOX_MEDIUM` and `HUMAN_BODY_MEDIUM`
- Depth advertising is disabled when depth is disabled (see `sl::DETH_MODE::NONE`)
