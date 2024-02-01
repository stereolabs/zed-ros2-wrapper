// Copyright 2023 Stereolabs
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

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <zed_interfaces/msg/depth_info_stamped.hpp>
#include <zed_interfaces/msg/object.hpp>
#include <zed_interfaces/msg/objects_stamped.hpp>
#include <zed_interfaces/msg/plane_stamped.hpp>
#include <zed_interfaces/srv/set_pose.hpp>
#include <zed_interfaces/srv/set_roi.hpp>
#include <zed_interfaces/srv/start_svo_rec.hpp>
#include <zed_interfaces/msg/pos_track_status.hpp>

#include <robot_localization/srv/from_ll.hpp>
#include <robot_localization/srv/to_ll.hpp>

#define TIMEZERO_ROS rclcpp::Time(0, 0, RCL_ROS_TIME)
#define TIMEZERO_SYS rclcpp::Time(0, 0, RCL_SYSTEM_TIME)

namespace stereolabs
{

#ifdef _SL_JETSON_
const bool IS_JETSON = true;
#else
const bool IS_JETSON = false;
#endif

const float NOT_VALID_TEMP = -273.15f;

// ----> Typedefs to simplify declarations

typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> imagePub;
typedef std::shared_ptr<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>> disparityPub;

typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloudPub;

typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imuPub;
typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::MagneticField>> magPub;
typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::FluidPressure>> pressPub;
typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Temperature>> tempPub;

typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> posePub;
typedef std::shared_ptr<rclcpp::Publisher<zed_interfaces::msg::PosTrackStatus>> poseStatusPub;
typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>>
  poseCovPub;
typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TransformStamped>> transfPub;
typedef std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPub;
typedef std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> pathPub;

typedef std::shared_ptr<rclcpp::Publisher<zed_interfaces::msg::ObjectsStamped>> objPub;
typedef std::shared_ptr<rclcpp::Publisher<zed_interfaces::msg::DepthInfoStamped>> depthInfoPub;

typedef std::shared_ptr<rclcpp::Publisher<zed_interfaces::msg::PlaneStamped>> planePub;
typedef std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> markerPub;

typedef std::shared_ptr<rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>> geoPosePub;

typedef std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PointStamped>> clickedPtSub;
typedef std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>> gnssFixSub;
typedef std::shared_ptr<rclcpp::Subscription<rosgraph_msgs::msg::Clock>> clockSub;

typedef std::unique_ptr<sensor_msgs::msg::Image> imageMsgPtr;
typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;
typedef std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloudMsgPtr;
typedef std::unique_ptr<sensor_msgs::msg::Imu> imuMsgPtr;
typedef std::unique_ptr<sensor_msgs::msg::FluidPressure> pressMsgPtr;
typedef std::unique_ptr<sensor_msgs::msg::Temperature> tempMsgPtr;
typedef std::unique_ptr<sensor_msgs::msg::MagneticField> magMsgPtr;
typedef std::unique_ptr<stereo_msgs::msg::DisparityImage> dispMsgPtr;

typedef std::unique_ptr<geometry_msgs::msg::PoseStamped> poseMsgPtr;
typedef std::unique_ptr<zed_interfaces::msg::PosTrackStatus> poseStatusMsgPtr;
typedef std::unique_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> poseCovMsgPtr;
typedef std::unique_ptr<geometry_msgs::msg::TransformStamped> transfMsgPtr;
typedef std::unique_ptr<nav_msgs::msg::Odometry> odomMsgPtr;
typedef std::unique_ptr<nav_msgs::msg::Path> pathMsgPtr;

typedef std::unique_ptr<geographic_msgs::msg::GeoPoseStamped> geoPoseMsgPtr;

typedef std::unique_ptr<zed_interfaces::msg::ObjectsStamped> objDetMsgPtr;
typedef std::unique_ptr<zed_interfaces::msg::DepthInfoStamped> depthInfoMsgPtr;
typedef std::unique_ptr<zed_interfaces::msg::PlaneStamped> planeMsgPtr;
typedef std::unique_ptr<visualization_msgs::msg::Marker> markerMsgPtr;

typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetOdomSrvPtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetPosTrkSrvPtr;
typedef rclcpp::Service<zed_interfaces::srv::SetPose>::SharedPtr setPoseSrvPtr;
typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableObjDetPtr;
typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableBodyTrkPtr;
typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableMappingPtr;
typedef rclcpp::Service<zed_interfaces::srv::StartSvoRec>::SharedPtr startSvoRecSrvPtr;
typedef rclcpp::Service<zed_interfaces::srv::SetROI>::SharedPtr setRoiSrvPtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopSvoRecSrvPtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pauseSvoSrvPtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetRoiSrvPtr;
typedef rclcpp::Service<robot_localization::srv::ToLL>::SharedPtr toLLSrvPtr;
typedef rclcpp::Service<robot_localization::srv::FromLL>::SharedPtr fromLLSrvPtr;

/*!
   * @brief Video/Depth topic resolution
   */
typedef enum
{
  NATIVE,  //!< Same camera grab resolution
  CUSTOM   //!< Custom Rescale Factor
} PubRes;
// <---- Typedefs to simplify declarations
}  // namespace stereolabs

#endif  // SL_TYPES_HPP_
