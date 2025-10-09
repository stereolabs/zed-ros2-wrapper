// Copyright 2025 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SL_TYPES_HPP_
#define SL_TYPES_HPP_

#include <sl/Camera.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rosgraph_msgs/msg/clock.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <robot_localization/srv/to_ll.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <zed_msgs/srv/set_svo_frame.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <zed_msgs/msg/depth_info_stamped.hpp>
#include <zed_msgs/msg/gnss_fusion_status.hpp>
#include <zed_msgs/msg/object.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>
#include <zed_msgs/msg/plane_stamped.hpp>
#include <zed_msgs/msg/pos_track_status.hpp>
#include <zed_msgs/msg/svo_status.hpp>
#include <zed_msgs/msg/health_status_stamped.hpp>
#include <zed_msgs/msg/heartbeat.hpp>
#include <zed_msgs/srv/set_pose.hpp>
#include <zed_msgs/srv/set_roi.hpp>
#include <zed_msgs/srv/start_svo_rec.hpp>
// #include <zed_msgs/srv/save_area_memory.hpp> TODO(Walter): Uncomment when
// available in `zed_msgs` package from APT

#ifdef FOUND_ISAAC_ROS_NITROS
  #include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
  #include "isaac_ros_nitros_image_type/nitros_image.hpp"
#endif

#ifdef FOUND_POINT_CLOUD_TRANSPORT
  #include <point_cloud_transport/point_cloud_transport.hpp>
#endif

#define TIMEZERO_ROS rclcpp::Time(0, 0, RCL_ROS_TIME)
#define TIMEZERO_SYS rclcpp::Time(0, 0, RCL_SYSTEM_TIME)

namespace stereolabs
{

// ----> Global constants
const double DEG2RAD = 0.017453293;
const double RAD2DEG = 57.295777937;

const sl::COORDINATE_SYSTEM ROS_COORDINATE_SYSTEM =
  sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
const sl::UNIT ROS_MEAS_UNITS = sl::UNIT::METER;

const int QOS_QUEUE_SIZE = 10;
// <---- Global constants

#ifdef _SL_JETSON_
const bool IS_JETSON = true;
#else
const bool IS_JETSON = false;
#endif

const float NOT_VALID_TEMP = -273.15f;

// ----> Typedefs to simplify declarations

#ifdef FOUND_ISAAC_ROS_NITROS
typedef std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosImage>> nitrosImgPub;
#endif

typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;

typedef rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoPub;
typedef rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPub;
typedef rclcpp::Publisher<zed_msgs::msg::SvoStatus>::SharedPtr svoStatusPub;
typedef rclcpp::Publisher<zed_msgs::msg::HealthStatusStamped>::SharedPtr healthStatusPub;
typedef rclcpp::Publisher<zed_msgs::msg::Heartbeat>::SharedPtr heartbeatStatusPub;

typedef rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub;
typedef rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr disparityPub;

typedef rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudPub;

typedef rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
typedef rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magPub;
typedef rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressPub;
typedef rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPub;

typedef rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub;
typedef rclcpp::Publisher<zed_msgs::msg::PosTrackStatus>::SharedPtr poseStatusPub;
typedef rclcpp::Publisher<zed_msgs::msg::GnssFusionStatus>::SharedPtr gnssFusionStatusPub;
typedef rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poseCovPub;
typedef rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transfPub;
typedef rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
typedef rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub;

typedef rclcpp::Publisher<zed_msgs::msg::ObjectsStamped>::SharedPtr objPub;
typedef rclcpp::Publisher<zed_msgs::msg::DepthInfoStamped>::SharedPtr depthInfoPub;

typedef rclcpp::Publisher<zed_msgs::msg::PlaneStamped>::SharedPtr planePub;
typedef rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;

typedef rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geoPosePub;
typedef rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnssFixPub;

typedef rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr healthPub;

typedef rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPtSub;
typedef rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnssFixSub;
typedef rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clockSub;

typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetOdomSrvPtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetPosTrkSrvPtr;
typedef rclcpp::Service<zed_msgs::srv::SetPose>::SharedPtr setPoseSrvPtr;
// typedef rclcpp::Service<zed_msgs::srv::SaveAreaMemory>::SharedPtr saveAreaMemorySrvPtr; TODO(Walter): Uncomment when available in `zed_msgs` package from APT
typedef rclcpp::Service<zed_msgs::srv::SetROI>::SharedPtr saveAreaMemorySrvPtr;

typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableObjDetPtr;
typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableBodyTrkPtr;
typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableMappingPtr;

typedef rclcpp::Service<zed_msgs::srv::StartSvoRec>::SharedPtr startSvoRecSrvPtr;
typedef rclcpp::Service<zed_msgs::srv::SetROI>::SharedPtr setRoiSrvPtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopSvoRecSrvPtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pauseSvoSrvPtr;
typedef rclcpp::Service<zed_msgs::srv::SetSvoFrame>::SharedPtr setSvoFramePtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetRoiSrvPtr;
typedef rclcpp::Service<robot_localization::srv::ToLL>::SharedPtr toLLSrvPtr;
typedef rclcpp::Service<robot_localization::srv::FromLL>::SharedPtr fromLLSrvPtr;
typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableStreamingPtr;


/*!
 * @brief Video/Depth topic resolution
 */
typedef enum
{
  NATIVE,  //!< Same camera grab resolution
  CUSTOM   //!< Custom Rescale Factor
} PubRes;

std::string toString(const PubRes & res);

typedef enum
{
  PUB,     //!< Same resolution as Color and Depth Map. [Old behavior for compatibility]
  FULL,    //!< Full resolution. Not recommended because slow processing and high bandwidth requirements
  COMPACT,  //!< Standard resolution. Optimizes processing and bandwidth
  REDUCED   //!< Half resolution. Low processing and bandwidth requirements
} PcRes;

std::string toString(const PcRes & res);

const int NEURAL_W = 896;
const int NEURAL_H = 512;
// <---- Typedefs to simplify declarations

}  // namespace stereolabs

#endif  // SL_TYPES_HPP_
