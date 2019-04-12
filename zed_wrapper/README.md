![](../images/Picto+STEREOLABS_Black.jpg)
# ZED ROS2 wrapper

The easiest way to start a ZED ROS2 node is to use the command line:
```bash
$ ros2 launch stereolabs_zed zed.py.launch
```

## Published Topics
The ZED node publishes data to the following topics:

- **Left camera**

   - `zed/zed_node/rgb/image_rect_color`: Color rectified image (left RGB image by default) 
   - `zed/zed_node/rgb/image_rect_color/camera_info`: Color camera calibration data 
   - `zed/zed_node/rgb/image_raw_color`: Color unrectified image (left RGB image by default) 
   - `zed/zed_node/rgb/image_raw_color/camera_info`: Color camera calibration data 
   - `zed/zed_node/left/image_rect_color`: Left camera color rectified image 
   - `zed/zed_node/left/image_rect_color/camera_info`: Left camera calibration data 
   - `zed/zed_node/left/image_raw_color`: Left camera color unrectified image 
   - `zed/zed_node/left/image_raw_color/camera_info`: Left camera calibration data 

- **Right camera**

  - `zed/zed_node/right/image_rect_color`: Color rectified right image 
  - `zed/zed_node/right/image_rect_color/camera_info`: Right camera calibration data 
  - `zed/zed_node/right/image_raw_color`: Color unrectified right image 
  - `zed/zed_node/right/image_raw_color/camera_info`: Right camera calibration data 

- **Depth and point cloud**

   - `zed/zed_node/depth/depth_registered`: Depth map image registered on left image (**Normal mode** - 32-bit float in meters) 
   - `zed/zed_node/depth/depth_registered/camera_info`: Depth camera calibration data (**Normal mode**) 
   - `zed/zed_node/depth/depth_raw_registered`: Depth map image registered on left image (**OpenNI mode** - 16-bit unsigned in millimeters) 
   - `zed/zed_node/depth/depth_raw_registered/camera_info`: Depth camera calibration data (**OpenNI mode**) 
   - `zed/zed_node/point_cloud/cloud_registered`: Registered color point cloud 
   - `zed/zed_node/confidence/confidence_image`: Confidence image 
   - `zed/zed_node/confidence/confidence_image/camera_info`: Depth camera calibration data 
   - `zed/zed_node/confidence/confidence_map`: Confidence image (floating point values) 
   - `zed/zed_node/disparity/disparity_image`: Disparity image 
 
- **Positional tracking**
  - `/zed/zed_node/pose`: Absolute 3D position and orientation relative to the Map frame (Sensor Fusion algorithm + SLAM)
  - `zed/zed_node/pose_with_covariance`: Camera pose referred to Map frame with covariance
  - `/zed/zed_node/odom`: Absolute 3D position and orientation relative to the Odometry frame (pure visual odometry for ZED, visual-inertial odometry for ZED-M)
  - `/zed/zed_node/map2odom`: Current transform from the odometry frame to the map frame. Useful to publish the TF transform `map` -> `odom`
  - `/zed/zed_node/path_pose`: Sequence of camera poses in Map frame
  - `/zed/zed_node/path_odom`: Sequence of camera odometry poses in Map frame

- **Inertial data**

   - `zed/zed_node/imu/data`: Accelerometer, gyroscope, and orientation data in Earth frame 
   - `zed/zed_node/imu/data_raw`: Accelerometer and gyroscope data in Earth frame 

- **Lyfecycle transitions**

  - `zed/zed_node/transition_event`: notification of lifecycle state changing (see the [lifecycle tutorial](/integrations/ros2/lifecycle/))

The `transition_event` topic has default system settings.

### Image Transport

The ROS wrapper supports the stack [`image_transport`](http://wiki.ros.org/image_transport) introduced with ROS2 Crystal Clemmys.

The `rgb`, `left`, `right` and `depth` topics are republished using the `image_transport::CameraPublisher` object, that correctly associates the `sensor_msgs::msg::CameraInfo` message to the relative `sensor_msg::msg::Image` message and creates the compressed image streams.

The prefix `it_` is added to the root of each image stream to indicate that the images are published using `image_transport` (e.g. `/zed/zed_node/it_rgb/image_rect_color/compressed`).

### QoS profiles

All the topics are configured to use the default ROS2 [QoS profile](http://design.ros2.org/articles/qos.html):

* **History**: `KEEP_LAST`
* **Depth**: `10`
* **Reliability**: `RELIABLE`
* **Durability**: `VOLATILE`

The QoS settings can be modified changing the relative parameters in the YAML files, as described below.

When creating a subscriber, be sure to use a compatible QOS profile according to the following tables:

*Compatibility of QoS durability profiles:*

Publisher       | Subscriber      | Connection  | Result
----------------|-----------------|-------------|--------
Volatile	      | Volatile        |	Yes         |	Volatile
**Volatile**	  | **Transient local** | **No**  | **-**
Transient local | Volatile	      | Yes         | Volatile
Transient local |	Transient local	| Yes         | Transient local

*Compatibility of QoS reliability profiles:*

Publisher       | Subscriber      | Connection  | Result
----------------|-----------------|-------------|--------
Best effort	    | Best effort     | Yes	        | Best effort
**Best effort** | **Reliable**    | **No**      | **-**
Reliable	      | Best effort	    | Yes         |	Best effort
Reliable	      | Reliable	      | Yes	        | Reliable

In order for a connection to be made, *all* of the policies that affect compatibility must be compatible. For instance, if a publisher-subscriber pair has compatible reliability QoS profiles, but incompatible durability QoS profiles, the connection will not be made.

## Detailed guide
For a detailed description of the parameter settings and the different mode to execute a ZED ROS2 node please refer to the [online documentation](https://www.stereolabs.com/docs/ros2/zed_node/)