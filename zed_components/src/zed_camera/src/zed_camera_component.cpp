// Copyright 2022 Stereolabs
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

#include "zed_camera_component.hpp"

#include <sys/resource.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <limits>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include <sstream>

#include "sl_logging.hpp"

#ifdef FOUND_HUMBLE
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif defined FOUND_IRON
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif defined FOUND_FOXY
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#error Unsupported ROS2 distro
#endif

#include <sl/Camera.hpp>

#include "sl_tools.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

// Used for simulation data input
#define ZED_SDK_PORT 30000

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

ZedCamera::ZedCamera(const rclcpp::NodeOptions & options)
: Node("zed_node", options),
  mThreadStop(false),
  mQos(QOS_QUEUE_SIZE),
  mAiInstanceID(0),
  mDiagUpdater(this),
  mImuTfFreqTimer(get_clock()),
  mGrabFreqTimer(get_clock()),
  mImuFreqTimer(get_clock()),
  mBaroFreqTimer(get_clock()),
  mMagFreqTimer(get_clock()),
  mOdomFreqTimer(get_clock()),
  mPoseFreqTimer(get_clock()),
  mPcPubFreqTimer(get_clock()),
  mVdPubFreqTimer(get_clock()),
  mSensPubFreqTimer(get_clock()),
  mOdFreqTimer(get_clock()),
  mBtFreqTimer(get_clock()),
  mPcFreqTimer(get_clock()),
  mGnssFixFreqTimer(get_clock()),
  mFrameTimestamp(TIMEZERO_ROS),
  mGnssTimestamp(TIMEZERO_ROS),
  mLastTs_imu(TIMEZERO_ROS),
  mLastTs_baro(TIMEZERO_ROS),
  mLastTs_mag(TIMEZERO_ROS),
  mLastTs_odom(TIMEZERO_ROS),
  mLastTs_pose(TIMEZERO_ROS),
  mLastTs_pc(TIMEZERO_ROS),
  mPrevTs_pc(TIMEZERO_ROS),
  mLastClock(TIMEZERO_ROS),
  mStreamingServerRequired(false),
  mStreamingServerRunning(false)
{
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), "      ZED Camera Component ");
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "********************************");

  const size_t SDK_MAJOR_REQ = 4;
  const size_t SDK_MINOR_REQ = 1;

  if (ZED_SDK_MAJOR_VERSION < SDK_MAJOR_REQ ||
    (ZED_SDK_MAJOR_VERSION == SDK_MAJOR_REQ &&
    ZED_SDK_MINOR_VERSION < SDK_MINOR_REQ))
  {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "This version of the ZED ROS2 wrapper is designed to work with ZED SDK "
      "v" << static_cast<int>(SDK_MAJOR_REQ)
          << "." << static_cast<int>(SDK_MINOR_REQ) << " or newer.");
    RCLCPP_INFO_STREAM(
      get_logger(), "* Detected SDK v"
        << ZED_SDK_MAJOR_VERSION << "."
        << ZED_SDK_MINOR_VERSION << "."
        << ZED_SDK_PATCH_VERSION << "-"
        << ZED_SDK_BUILD_ID);
    RCLCPP_INFO(get_logger(), "Node stopped");
    exit(EXIT_FAILURE);
  }

  // ----> Publishers/Sbscribers options
  #ifndef FOUND_FOXY
  mPubOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  mSubOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  #endif
  // <---- Publishers/Sbscribers options

  // ----> Start a "one shot timer" to initialize the node and make `shared_from_this` available
  std::chrono::milliseconds init_msec(static_cast<int>(50.0));
  mInitTimer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(init_msec),
    std::bind(&ZedCamera::init, this));
  // <---- Start a "one shot timer" to initialize the node and make `shared_from_this` available
}

void ZedCamera::init()
{
  // Stop the timer for "one shot" initialization
  mInitTimer->cancel();

  // Parameters initialization
  initParameters();

  // ----> Diagnostic initialization
  mDiagUpdater.add(
    "ZED Diagnostic", this,
    &ZedCamera::callback_updateDiagnostic);
  std::string hw_id = std::string("Stereolabs camera: ") + mCameraName;
  mDiagUpdater.setHardwareID(hw_id);
  // <---- Diagnostic initialization

  // Services initialization
  initServices();

  // ----> Start camera
  if (!startCamera()) {
    exit(EXIT_FAILURE);
  }
  // <---- Start camera

  // Dynamic parameters callback
  mParamChangeCallbackHandle = add_on_set_parameters_callback(
    std::bind(&ZedCamera::callback_paramChange, this, _1));
}

ZedCamera::~ZedCamera()
{
  DEBUG_STREAM_COMM("Destroying node");

  // ----> Stop subscribers
  mClickedPtSub.reset();
  mGnssFixSub.reset();
  mClockSub.reset();
  // <---- Stop subscribers

  if (mObjDetRunning) {
    std::lock_guard<std::mutex> lock(mObjDetMutex);
    stopObjDetect();
  }

  if (mBodyTrkRunning) {
    std::lock_guard<std::mutex> lock(mBodyTrkMutex);
    stopBodyTracking();
  }

  if (mSpatialMappingRunning) {
    std::lock_guard<std::mutex> lock(mMappingMutex);
    stop3dMapping();
  }

  DEBUG_STREAM_PT("Stopping path timer");
  if (mPathTimer) {
    mPathTimer->cancel();
  }
  DEBUG_STREAM_MAP("Stopping fused cloud timer");
  if (mFusedPcTimer) {
    mFusedPcTimer->cancel();
  }

  DEBUG_STREAM_SENS("Stopping temperatures timer");
  if (mTempPubTimer) {
    mTempPubTimer->cancel();
  }

  // ----> Verify that all the threads are not active
  DEBUG_STREAM_COMM("Stopping running threads");
  if (!mThreadStop) {
    mThreadStop = true;
  }

  DEBUG_STREAM_COMM("Waiting for grab thread...");
  try {
    if (mGrabThread.joinable()) {
      mGrabThread.join();
    }
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Grab thread joining exception: " << e.what());
  }
  DEBUG_STREAM_COMM("... grab thread stopped");

  DEBUG_STREAM_SENS("Waiting for sensors thread...");
  try {
    if (mSensThread.joinable()) {
      mSensThread.join();
    }
  } catch (std::system_error & e) {
    DEBUG_STREAM_SENS("Sensors thread joining exception: " << e.what());
  }
  DEBUG_STREAM_SENS("... sensors thread stopped");

  DEBUG_STREAM_VD("Waiting for RGB/Depth thread...");
  try {
    if (mVideoDepthThread.joinable()) {
      mVideoDepthThread.join();
    }
  } catch (std::system_error & e) {
    DEBUG_STREAM_VD("RGB/Depth thread joining exception: " << e.what());
  }
  DEBUG_STREAM_VD("... RGB/Depth thread stopped");

  DEBUG_STREAM_PC("Waiting for Point Cloud thread...");
  try {
    if (mPcThread.joinable()) {
      mPcThread.join();
    }
  } catch (std::system_error & e) {
    DEBUG_STREAM_PC("Pointcloud thread joining exception: " << e.what());
  }
  DEBUG_STREAM_PC("... Point Cloud thread stopped");

  // <---- Verify that all the threads are not active
}

void ZedCamera::initServices()
{
  RCLCPP_INFO(get_logger(), "*** SERVICES ***");

  std::string srv_name;

  std::string srv_prefix = "~/";

  if (!mDepthDisabled) {
    // Reset Odometry
    srv_name = srv_prefix + mSrvResetOdomName;
    mResetOdomSrv = create_service<std_srvs::srv::Trigger>(
      srv_name,
      std::bind(&ZedCamera::callback_resetOdometry, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mResetOdomSrv->get_service_name());
    // Reset Pose
    srv_name = srv_prefix + mSrvResetPoseName;
    mResetPosTrkSrv = create_service<std_srvs::srv::Trigger>(
      srv_name,
      std::bind(&ZedCamera::callback_resetPosTracking, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mResetPosTrkSrv->get_service_name());
    // Set Pose
    srv_name = srv_prefix + mSrvSetPoseName;
    mSetPoseSrv = create_service<zed_interfaces::srv::SetPose>(
      srv_name, std::bind(&ZedCamera::callback_setPose, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mSetPoseSrv->get_service_name());
    // Enable Object Detection
    srv_name = srv_prefix + mSrvEnableObjDetName;
    mEnableObjDetSrv = create_service<std_srvs::srv::SetBool>(
      srv_name,
      std::bind(&ZedCamera::callback_enableObjDet, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mEnableObjDetSrv->get_service_name());
    // Enable BodyTracking
    srv_name = srv_prefix + mSrvEnableBodyTrkName;
    mEnableBodyTrkSrv = create_service<std_srvs::srv::SetBool>(
      srv_name,
      std::bind(&ZedCamera::callback_enableBodyTrk, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mEnableBodyTrkSrv->get_service_name());
    // Enable Mapping
    srv_name = srv_prefix + mSrvEnableMappingName;
    mEnableMappingSrv = create_service<std_srvs::srv::SetBool>(
      srv_name,
      std::bind(&ZedCamera::callback_enableMapping, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mEnableMappingSrv->get_service_name());
  }

  // Enable Streaming
  srv_name = srv_prefix + mSrvEnableStreamingName;
  mEnableStreamingSrv = create_service<std_srvs::srv::SetBool>(
    srv_name, std::bind(&ZedCamera::callback_enableStreaming, this, _1, _2, _3));
  RCLCPP_INFO(get_logger(), " * '%s'", mEnableStreamingSrv->get_service_name());
  // Start SVO Recording
  srv_name = srv_prefix + mSrvStartSvoRecName;
  mStartSvoRecSrv = create_service<zed_interfaces::srv::StartSvoRec>(
    srv_name, std::bind(&ZedCamera::callback_startSvoRec, this, _1, _2, _3));
  RCLCPP_INFO(get_logger(), " * '%s'", mStartSvoRecSrv->get_service_name());
  // Stop SVO Recording
  srv_name = srv_prefix + mSrvStopSvoRecName;
  mStopSvoRecSrv = create_service<std_srvs::srv::Trigger>(
    srv_name, std::bind(&ZedCamera::callback_stopSvoRec, this, _1, _2, _3));
  RCLCPP_INFO(get_logger(), " * '%s'", mStopSvoRecSrv->get_service_name());
  // Pause SVO
  if (mSvoMode && !mSvoRealtime) {
    srv_name = srv_prefix + mSrvToggleSvoPauseName;
    mPauseSvoSrv = create_service<std_srvs::srv::Trigger>(
      srv_name,
      std::bind(&ZedCamera::callback_pauseSvoInput, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mPauseSvoSrv->get_service_name());
  }
  // Set ROI
  srv_name = srv_prefix + mSrvSetRoiName;
  mSetRoiSrv = create_service<zed_interfaces::srv::SetROI>(
    srv_name, std::bind(&ZedCamera::callback_setRoi, this, _1, _2, _3));
  RCLCPP_INFO(get_logger(), " * '%s'", mSetRoiSrv->get_service_name());
  // Reset ROI
  srv_name = srv_prefix + mSrvResetRoiName;
  mResetRoiSrv = create_service<std_srvs::srv::Trigger>(
    srv_name, std::bind(&ZedCamera::callback_resetRoi, this, _1, _2, _3));
  RCLCPP_INFO(get_logger(), " * '%s'", mResetRoiSrv->get_service_name());

  if (mGnssFusionEnabled) {
    // To Latitude/Longitude
    srv_name = srv_prefix + mSrvToLlName;
    mToLlSrv = create_service<robot_localization::srv::ToLL>(
      srv_name, std::bind(&ZedCamera::callback_toLL, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mToLlSrv->get_service_name());
    // From Latitude/Longitude
    srv_name = srv_prefix + mSrvFromLlName;
    mFromLlSrv = create_service<robot_localization::srv::FromLL>(
      srv_name, std::bind(&ZedCamera::callback_fromLL, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mFromLlSrv->get_service_name());
  }
}

std::string ZedCamera::getParam(
  std::string paramName,
  std::vector<std::vector<float>> & outVal)
{
  outVal.clear();

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  declare_parameter(
    paramName, rclcpp::ParameterValue(std::string("[]")),
    descriptor);

  std::string out_str;

  if (!get_parameter(paramName, out_str)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default "
        "value: []");
  }

  if (out_str.empty()) {
    return std::string();
  }

  std::string error;
  outVal = sl_tools::parseStringVector(out_str, error);

  if (error != "") {
    RCLCPP_WARN_STREAM(
      get_logger(), "Error parsing "
        << paramName
        << " parameter: " << error.c_str());
    RCLCPP_WARN_STREAM(
      get_logger(),
      "   " << paramName << " string was " << out_str.c_str());

    outVal.clear();
  }

  return out_str;
}

template<typename T>
void ZedCamera::getParam(
  std::string paramName, T defValue, T & outVal,
  std::string log_info, bool dynamic)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = !dynamic;

  declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << defValue);
  }

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

void ZedCamera::initParameters()
{
  // DEBUG parameters
  getDebugParams();

  // SIMULATION parameters
  getSimParams();

  // GENERAL parameters
  getGeneralParams();

  // VIDEO parameters
  getVideoParams();

  // DEPTH parameters
  getDepthParams();

  // POS. TRACKING and GNSS FUSION parameters
  if (!mDepthDisabled) {
    // GNSS Fusion parameters
    getGnssFusionParams();

    // Positional Tracking parameters
    getPosTrackingParams();

    // Region of Interest parameters
    getRoiParams();
  } else {
    mGnssFusionEnabled = false;
    mPosTrackingEnabled = false;
    mRoyPolyParam.clear();
    mAutoRoiEnabled = false;
  }

  // SENSORS parameters
  if (!sl_tools::isZED(mCamUserModel)) {
    getSensorsParams();
  }

  if (!mDepthDisabled) {
    getMappingParams();
  } else {
    mMappingEnabled = false;
  }

  // AI PARAMETERS
  if (!mDepthDisabled) {
    if (sl_tools::isObjDetAvailable(mCamUserModel)) {
      getOdParams();
      getBodyTrkParams();
    }
  } else {
    mObjDetEnabled = false;
    mBodyTrkEnabled = false;
  }

  getStreamingServerParams();

  getAdvancedParams();
}

std::string ZedCamera::parseRoiPoly(
  const std::vector<std::vector<float>> & in_poly,
  std::vector<sl::float2> & out_poly)
{
  out_poly.clear();

  std::string ss;
  ss = "[";

  size_t poly_size = in_poly.size();

  if (poly_size < 3) {
    if (poly_size != 0) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "A vector with "
          << poly_size
          << " points is not enough to create a polygon to set a Region "
          "of Interest.");
      return std::string();
    }
  } else {
    for (size_t i = 0; i < poly_size; ++i) {
      if (in_poly[i].size() != 2) {
        RCLCPP_WARN_STREAM(
          get_logger(), "The component with index '"
            << i
            << "' of the ROI vector "
            "has not the correct size.");
        out_poly.clear();
        return std::string();
      } else if (in_poly[i][0] < 0.0 || in_poly[i][1] < 0.0 ||
        in_poly[i][0] > 1.0 || in_poly[i][1] > 1.0)
      {
        RCLCPP_WARN_STREAM(
          get_logger(), "The component with index '"
            << i
            << "' of the ROI vector "
            "is not a "
            "valid normalized point: ["
            << in_poly[i][0] << ","
            << in_poly[i][1] << "].");
        out_poly.clear();
        return std::string();
      } else {
        sl::float2 pt;
        pt.x = in_poly[i][0];
        pt.y = in_poly[i][1];
        out_poly.push_back(pt);
        ss += "[";
        ss += std::to_string(pt.x);
        ss += ",";
        ss += std::to_string(pt.y);
        ss += "]";
      }

      if (i != poly_size - 1) {
        ss += ",";
      }
    }
  }
  ss += "]";

  return ss;
}

void ZedCamera::getDebugParams()
{
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "*** DEBUG parameters ***");

  getParam("debug.sdk_verbose", mVerbose, mVerbose, " * SDK Verbose: ");

  getParam("debug.debug_common", mDebugCommon, mDebugCommon);
  RCLCPP_INFO(
    get_logger(), " * Debug Common: %s",
    mDebugCommon ? "TRUE" : "FALSE");

  getParam("debug.debug_sim", mDebugSim, mDebugSim);
  RCLCPP_INFO(
    get_logger(), " * Debug Simulation: %s",
    mDebugSim ? "TRUE" : "FALSE");

  getParam("debug.debug_video_depth", mDebugVideoDepth, mDebugVideoDepth);
  RCLCPP_INFO(
    get_logger(), " * Debug Video/Depth: %s",
    mDebugVideoDepth ? "TRUE" : "FALSE");

  getParam("debug.debug_camera_controls", mDebugCamCtrl, mDebugCamCtrl);
  RCLCPP_INFO(
    get_logger(), " * Debug Control settings: %s",
    mDebugCamCtrl ? "TRUE" : "FALSE");

  getParam("debug.debug_point_cloud", mDebugPointCloud, mDebugPointCloud);
  RCLCPP_INFO(
    get_logger(), " * Debug Point Cloud: %s",
    mDebugPointCloud ? "TRUE" : "FALSE");

  getParam("debug.debug_gnss", mDebugGnss, mDebugGnss);
  RCLCPP_INFO(get_logger(), " * Debug GNSS: %s", mDebugGnss ? "TRUE" : "FALSE");

  getParam(
    "debug.debug_positional_tracking", mDebugPosTracking,
    mDebugPosTracking);
  RCLCPP_INFO(
    get_logger(), " * Debug Positional Tracking: %s",
    mDebugPosTracking ? "TRUE" : "FALSE");

  getParam("debug.debug_sensors", mDebugSensors, mDebugSensors);
  RCLCPP_INFO(
    get_logger(), " * Debug sensors: %s",
    mDebugSensors ? "TRUE" : "FALSE");

  getParam("debug.debug_mapping", mDebugMapping, mDebugMapping);
  RCLCPP_INFO(
    get_logger(), " * Debug Mapping: %s",
    mDebugMapping ? "TRUE" : "FALSE");

  getParam("debug.debug_object_detection", mDebugObjectDet, mDebugObjectDet);
  RCLCPP_INFO(
    get_logger(), " * Debug Object Detection: %s",
    mDebugObjectDet ? "TRUE" : "FALSE");

  getParam("debug.debug_body_tracking", mDebugBodyTrk, mDebugBodyTrk);
  RCLCPP_INFO(
    get_logger(), " * Debug Body Tracking: %s",
    mDebugBodyTrk ? "TRUE" : "FALSE");

  getParam("debug.debug_streaming", mDebugStreaming, mDebugStreaming);
  RCLCPP_INFO(get_logger(), " * Debug Streaming: %s", mDebugStreaming ? "TRUE" : "FALSE");

  getParam("debug.debug_roi", mDebugRoi, mDebugRoi);
  RCLCPP_INFO(get_logger(), " * Debug ROI: %s", mDebugRoi ? "TRUE" : "FALSE");

  getParam("debug.debug_advanced", mDebugAdvanced, mDebugAdvanced);
  RCLCPP_INFO(
    get_logger(), " * Debug Advanced: %s",
    mDebugAdvanced ? "TRUE" : "FALSE");

  mDebugMode = mDebugCommon || mDebugSim || mDebugVideoDepth || mDebugCamCtrl ||
    mDebugPointCloud || mDebugPosTracking || mDebugGnss ||
    mDebugSensors || mDebugMapping || mDebugObjectDet ||
    mDebugBodyTrk || mDebugAdvanced || mDebugRoi || mDebugStreaming;

  if (mDebugMode) {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting DEBUG level for logger");
    } else {
      RCLCPP_INFO(get_logger(), " + Debug Mode enabled +");
    }
  } else {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting INFO level for logger");
    }
  }

  DEBUG_STREAM_COMM(
    "[ROS2] Using RMW_IMPLEMENTATION "
      << rmw_get_implementation_identifier());
}

void ZedCamera::getSimParams()
{
  // SIMULATION active?
  getParam("simulation.sim_enabled", mSimMode, mSimMode);

  if (!get_parameter("use_sim_time", mUseSimTime)) {
    RCLCPP_WARN(
      get_logger(),
      "The parameter 'use_sim_time' is not available or is not "
      "valid, using the default value.");
  }

  if (mSimMode) {
    RCLCPP_INFO(get_logger(), " *** SIMULATION MODE ACTIVE ***");
    getParam(
      "simulation.sim_address", mSimAddr, mSimAddr,
      " * Sim. server address: ");
    getParam(
      "simulation.sim_port", mSimPort, mSimPort,
      " * Sim. server port: ");

    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Use Sim Time: " << (mUseSimTime ? "TRUE" : "FALSE"));
  } else if (mUseSimTime) {
    RCLCPP_WARN(
      get_logger(),
      "The parameter 'use_sim_time' is set to 'true', but simulation "
      "mode is not enabled. UNPREDICTABLE BEHAVIORS EXPECTED.");
  }
}

void ZedCamera::getGeneralParams()
{
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "*** GENERAL parameters ***");

  getParam("svo.svo_path", std::string(), mSvoFilepath);
  if (mSvoFilepath.compare("live") == 0) {
    mSvoFilepath = "";
  }

  if (mSvoFilepath == "") {
    mSvoMode = false;
  } else {
    RCLCPP_INFO_STREAM(get_logger(), " * SVO: '" << mSvoFilepath.c_str() << "'");
    mSvoMode = true;
    getParam("svo.svo_loop", mSvoLoop, mSvoLoop);
    RCLCPP_INFO(get_logger(), " * SVO Loop: %s", mSvoLoop ? "TRUE" : "FALSE");
    getParam("svo.svo_realtime", mSvoRealtime, mSvoRealtime);
    RCLCPP_INFO(
      get_logger(), " * SVO Realtime: %s",
      mSvoRealtime ? "TRUE" : "FALSE");
  }

  mStreamMode = false;
  if (!mSvoMode) {
    getParam("stream.stream_address", std::string(), mStreamAddr);
    if (mStreamAddr != "") {
      mStreamMode = true;
      getParam("stream.stream_port", mStreamPort, mStreamPort);
      RCLCPP_INFO_STREAM(
        get_logger(), " * Local stream input: " << mStreamAddr << ":" << mStreamPort);
    }
  }

  std::string camera_model = "zed";
  getParam("general.camera_model", camera_model, camera_model);
  if (camera_model == "zed") {
    mCamUserModel = sl::MODEL::ZED;
  } else if (camera_model == "zedm") {
    mCamUserModel = sl::MODEL::ZED_M;
  } else if (camera_model == "zed2") {
    mCamUserModel = sl::MODEL::ZED2;
  } else if (camera_model == "zed2i") {
    mCamUserModel = sl::MODEL::ZED2i;
  } else if (camera_model == "zedx") {
    mCamUserModel = sl::MODEL::ZED_X;
    if (mSvoMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Playing an SVO for "
          << sl::toString(mCamUserModel)
          << " camera model.");
    } else if (mStreamMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Playing a network stream from a "
          << sl::toString(mCamUserModel)
          << " camera model.");
    } else if (mSimMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Simulating a "
          << sl::toString(mCamUserModel)
          << " camera model.");
    } else if (!IS_JETSON) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Camera model " << sl::toString(mCamUserModel).c_str()
                        << " is available only with NVIDIA Jetson devices.");
      exit(EXIT_FAILURE);
    }
  } else if (camera_model == "zedxm") {
    mCamUserModel = sl::MODEL::ZED_XM;
    if (mSvoMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Playing an SVO for "
          << sl::toString(mCamUserModel)
          << " camera model.");
    } else if (mStreamMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Playing a network stream from a "
          << sl::toString(mCamUserModel)
          << " camera model.");
    } else if (mSimMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Simulating a "
          << sl::toString(mCamUserModel)
          << " camera model.");
    } else if (!IS_JETSON) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Camera model " << sl::toString(mCamUserModel).c_str()
                        << " is available only with NVIDIA Jetson devices.");
      exit(EXIT_FAILURE);
    }
  } else if (camera_model == "virtual") {
    mCamUserModel = sl::MODEL::VIRTUAL_ZED_X;

    if (ZED_SDK_MAJOR_VERSION == 4 && ZED_SDK_MINOR_VERSION == 1 && ZED_SDK_PATCH_VERSION == 0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Camera model '" << sl::toString(mCamUserModel).c_str()
                         << "' is available only with ZED SDK 4.1.1 or newer");
      exit(EXIT_FAILURE);
    }

    if (mSvoMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Playing an SVO for "
          << sl::toString(mCamUserModel)
          << " camera model.");
    } else if (mStreamMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Playing a network stream from a "
          << sl::toString(mCamUserModel)
          << " camera model.");
    } else if (mSimMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Simulating a "
          << sl::toString(mCamUserModel)
          << " camera model.");
    } else if (!IS_JETSON) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Camera model " << sl::toString(mCamUserModel).c_str()
                        << " is available only with NVIDIA Jetson devices.");
      exit(EXIT_FAILURE);
    }
  } else {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Camera model not valid in parameter values: " << camera_model);
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Camera model: " << camera_model << " - "
                                      << mCamUserModel);

  getParam("general.camera_name", mCameraName, mCameraName, " * Camera name: ");
  getParam(
    "general.serial_number", mCamSerialNumber, mCamSerialNumber,
    " * Camera SN: ");
  getParam(
    "general.camera_timeout_sec", mCamTimeoutSec, mCamTimeoutSec,
    " * Camera timeout [sec]: ");
  getParam(
    "general.camera_max_reconnect", mMaxReconnectTemp, mMaxReconnectTemp,
    " * Camera reconnection temptatives: ");
  if (mSimMode) {
    RCLCPP_INFO(
      get_logger(),
      "* [Simulation mode] Camera framerate forced to 60 Hz");
    mCamGrabFrameRate = 60;
  } else {
    getParam(
      "general.grab_frame_rate", mCamGrabFrameRate, mCamGrabFrameRate,
      " * Camera framerate: ");
  }
  getParam("general.gpu_id", mGpuId, mGpuId, " * GPU ID: ");

  // TODO(walter) ADD SVO SAVE COMPRESSION PARAMETERS

  if (mSimMode) {
    RCLCPP_INFO(
      get_logger(),
      "* [Simulation mode] Camera resolution forced to 'HD1080'");
    mCamResol = sl::RESOLUTION::HD1080;
  } else {
    std::string resol = "AUTO";
    getParam("general.grab_resolution", resol, resol);
    if (resol == "AUTO") {
      mCamResol = sl::RESOLUTION::AUTO;
    } else if (sl_tools::isZEDX(mCamUserModel)) {
      if (resol == "HD1200") {
        mCamResol = sl::RESOLUTION::HD1200;
      } else if (resol == "HD1080") {
        mCamResol = sl::RESOLUTION::HD1080;
      } else if (resol == "SVGA") {
        mCamResol = sl::RESOLUTION::SVGA;
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Not valid 'general.grab_resolution' value: '%s'. Using "
          "'AUTO' setting.",
          resol.c_str());
        mCamResol = sl::RESOLUTION::AUTO;
      }
      RCLCPP_INFO_STREAM(
        get_logger(), " * Camera resolution: "
          << sl::toString(mCamResol).c_str());
    } else {
      if (resol == "HD2K") {
        mCamResol = sl::RESOLUTION::HD2K;
      } else if (resol == "HD1080") {
        mCamResol = sl::RESOLUTION::HD1080;
      } else if (resol == "HD720") {
        mCamResol = sl::RESOLUTION::HD720;
      } else if (resol == "VGA") {
        mCamResol = sl::RESOLUTION::VGA;
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Not valid 'general.grab_resolution' value: '%s'. Using "
          "'AUTO' setting.",
          resol.c_str());
        mCamResol = sl::RESOLUTION::AUTO;
      }
      RCLCPP_INFO_STREAM(
        get_logger(), " * Camera resolution: "
          << sl::toString(mCamResol).c_str());
    }
  }

  std::string out_resol = "MEDIUM";
  getParam("general.pub_resolution", out_resol, out_resol);
  if (out_resol == "NATIVE") {
    mPubResolution = PubRes::NATIVE;
  } else if (out_resol == "CUSTOM") {
    mPubResolution = PubRes::CUSTOM;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Not valid 'general.pub_resolution' value: '%s'. Using default "
      "setting instead.",
      out_resol.c_str());
    out_resol = "NATIVE";
    mPubResolution = PubRes::NATIVE;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Publishing resolution: " << out_resol.c_str());

  if (mPubResolution == PubRes::CUSTOM) {
    getParam(
      "general.pub_downscale_factor", mCustomDownscaleFactor,
      mCustomDownscaleFactor, " * Publishing downscale factor: ");
  } else {
    mCustomDownscaleFactor = 1.0;
  }

  getParam(
    "general.optional_opencv_calibration_file", mOpencvCalibFile,
    mOpencvCalibFile, " * OpenCV custom calibration: ");

  getParam("general.self_calib", mCameraSelfCalib, mCameraSelfCalib);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Camera self calibration: " << (mCameraSelfCalib ? "TRUE" : "FALSE"));
  getParam("general.camera_flip", mCameraFlip, mCameraFlip);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Camera flip: " << (mCameraFlip ? "TRUE" : "FALSE"));

  // Dynamic parameters

  if (mSimMode) {
    RCLCPP_INFO(
      get_logger(),
      "* [Simulation mode] Publish framerate forced to 60 Hz");
    mPubFrameRate = 60;
  } else {
    getParam("general.pub_frame_rate", mPubFrameRate, mPubFrameRate, "", false);
    if (mPubFrameRate > mCamGrabFrameRate) {
      RCLCPP_WARN(
        get_logger(),
        "'pub_frame_rate' cannot be bigger than 'grab_frame_rate'");
      mPubFrameRate = mCamGrabFrameRate;
    }
    if (mPubFrameRate < 0.1) {
      RCLCPP_WARN(
        get_logger(),
        "'pub_frame_rate' cannot be lower than 0.1 Hz or negative.");
      mPubFrameRate = mCamGrabFrameRate;
    }
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * [DYN] Publish framerate [Hz]:  " << mPubFrameRate);
  }
}

void ZedCamera::getVideoParams()
{
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "*** VIDEO parameters ***");

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  if (!sl_tools::isZEDX(mCamUserModel)) {
    getParam(
      "video.brightness", mCamBrightness, mCamBrightness,
      " * [DYN] Brightness: ", true);
    getParam(
      "video.contrast", mCamContrast, mCamContrast,
      " * [DYN] Contrast: ", true);
    getParam("video.hue", mCamHue, mCamHue, " * [DYN] Hue: ", true);
  }

  getParam(
    "video.saturation", mCamSaturation, mCamSaturation,
    " * [DYN] Saturation: ", true);
  getParam(
    "video.sharpness", mCamSharpness, mCamSharpness,
    " * [DYN] Sharpness: ", true);
  getParam("video.gamma", mCamGamma, mCamGamma, " * [DYN] Gamma: ", true);
  getParam(
    "video.auto_exposure_gain", mCamAutoExpGain, mCamAutoExpGain, "",
    true);
  RCLCPP_INFO(
    get_logger(), " * [DYN] Auto Exposure/Gain: %s",
    mCamAutoExpGain ? "TRUE" : "FALSE");
  if (mCamAutoExpGain) {
    mTriggerAutoExpGain = true;
  }
  getParam(
    "video.exposure", mCamExposure, mCamExposure,
    " * [DYN] Exposure: ", true);
  getParam("video.gain", mCamGain, mCamGain, " * [DYN] Gain: ", true);
  getParam("video.auto_whitebalance", mCamAutoWB, mCamAutoWB, "", true);
  RCLCPP_INFO(
    get_logger(), " * [DYN] Auto White Balance: %s",
    mCamAutoWB ? "TRUE" : "FALSE");
  if (mCamAutoWB) {
    mTriggerAutoWB = true;
  }
  int wb = 42;
  getParam(
    "video.whitebalance_temperature", wb, wb,
    " * [DYN] White Balance Temperature: ", true);
  mCamWBTemp = wb * 100;

  if (sl_tools::isZEDX(mCamUserModel)) {
    getParam(
      "video.exposure_time", mGmslExpTime, mGmslExpTime,
      " * [DYN] ZED X Exposure time: ", true);
    getParam(
      "video.auto_exposure_time_range_min", mGmslAutoExpTimeRangeMin,
      mGmslAutoExpTimeRangeMin,
      " * [DYN] ZED X Auto Exp. time range min: ", true);
    getParam(
      "video.auto_exposure_time_range_max", mGmslAutoExpTimeRangeMax,
      mGmslAutoExpTimeRangeMax,
      " * [DYN] ZED X Auto Exp. time range max: ", true);
    if (mGmslAutoExpTimeRangeMax > mCamGrabFrameRate * 1000 ||
      mGmslAutoExpTimeRangeMax > 30000)
    {
      mGmslAutoExpTimeRangeMax = std::max(mCamGrabFrameRate * 1000, 30000);
      RCLCPP_WARN_STREAM(
        get_logger(),
        "The values of 'video.auto_exposure_time_range_max' is clamped to "
        "max(30000,'general.grab_frame_rate'x1000): "
          << mGmslAutoExpTimeRangeMax);
    }
    getParam(
      "video.exposure_compensation", mGmslExposureComp,
      mGmslExposureComp, " * [DYN] ZED X Exposure comp.: ", true);
    getParam(
      "video.analog_gain", mGmslAnalogGain, mGmslAnalogGain,
      " * [DYN] ZED X Analog Gain: ", true);
    getParam(
      "video.auto_analog_gain_range_min", mGmslAnalogGainRangeMin,
      mGmslAnalogGainRangeMin,
      " * [DYN] ZED X Auto Analog Gain range min: ", true);
    getParam(
      "video.auto_analog_gain_range_max", mGmslAnalogGainRangeMax,
      mGmslAnalogGainRangeMax,
      " * [DYN] ZED X Auto Analog Gain range max: ", true);
    getParam(
      "video.digital_gain", mGmslDigitalGain, mGmslDigitalGain,
      " * [DYN] ZED X Digital Gain: ", true);
    getParam(
      "video.auto_digital_gain_range_min", mGmslAutoDigitalGainRangeMin,
      mGmslAutoDigitalGainRangeMin,
      " * [DYN] ZED X Auto Digital Gain range min: ", true);
    getParam(
      "video.auto_digital_gain_range_max", mGmslAutoDigitalGainRangeMax,
      mGmslAutoDigitalGainRangeMax,
      " * [DYN] ZED X Auto Digital Gain range max: ", true);
    getParam(
      "video.denoising", mGmslDenoising, mGmslDenoising,
      " * [DYN] ZED X Auto Digital Gain range max: ", true);
  }
}

void ZedCamera::getRoiParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** Region of Interest parameters ***");

  getParam(
    "region_of_interest.automatic_roi", mAutoRoiEnabled,
    mAutoRoiEnabled);
  RCLCPP_INFO(
    get_logger(), " * Automatic ROI generation: %s",
    mAutoRoiEnabled ? "TRUE" : "FALSE");

  if (mAutoRoiEnabled) {
    if (mPosTrkMode == sl::POSITIONAL_TRACKING_MODE::GEN_1) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Automatic ROI generation with '"
          << sl::toString(mPosTrkMode)
          << "' is not recommended. Please set the parameter "
          "'pos_tracking.pos_tracking_mode' to 'GEN_2' for "
          "improved results.");
    }

    getParam(
      "region_of_interest.depth_far_threshold_meters",
      mRoiDepthFarThresh, mRoiDepthFarThresh,
      " * Depth far threshold [m]: ");
    getParam(
      "region_of_interest.image_height_ratio_cutoff",
      mRoiImgHeightRationCutOff, mRoiImgHeightRationCutOff,
      " * Image Height Ratio Cut Off: ");
  } else {
    std::string parsed_str =
      getParam("region_of_interest.manual_polygon", mRoyPolyParam);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Manual ROI polygon: " << parsed_str.c_str());
  }

  if (mRoyPolyParam.size() > 0 || mAutoRoiEnabled) {
    mRoiModules.clear();
    bool apply = true;

    getParam("region_of_interest.apply_to_depth", apply, apply);
    RCLCPP_INFO(
      get_logger(), " * Apply to depth: %s",
      apply ? "TRUE" : "FALSE");
    if (apply) {
      mRoiModules.insert(sl::MODULE::DEPTH);
    }

    apply = true;
    getParam("region_of_interest.apply_to_positional_tracking", apply, apply);
    RCLCPP_INFO(
      get_logger(), " * Apply to positional tracking: %s",
      apply ? "TRUE" : "FALSE");
    if (apply) {
      mRoiModules.insert(sl::MODULE::POSITIONAL_TRACKING);
    }

    apply = true;
    getParam("region_of_interest.apply_to_object_detection", apply, apply);
    RCLCPP_INFO(
      get_logger(), " * Apply to object detection: %s",
      apply ? "TRUE" : "FALSE");
    if (apply) {
      mRoiModules.insert(sl::MODULE::OBJECT_DETECTION);
    }

    apply = true;
    getParam("region_of_interest.apply_to_body_tracking", apply, apply);
    RCLCPP_INFO(
      get_logger(), " * Apply to body tracking: %s",
      apply ? "TRUE" : "FALSE");
    if (apply) {
      mRoiModules.insert(sl::MODULE::BODY_TRACKING);
    }

    apply = true;
    getParam("region_of_interest.apply_to_spatial_mapping", apply, apply);
    RCLCPP_INFO(
      get_logger(), " * Apply to spatial mapping: %s",
      apply ? "TRUE" : "FALSE");
    if (apply) {
      mRoiModules.insert(sl::MODULE::SPATIAL_MAPPING);
    }
  }
}

void ZedCamera::getDepthParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** DEPTH parameters ***");

  std::string depth_mode_str = sl::toString(mDepthMode).c_str();
  getParam("depth.depth_mode", depth_mode_str, depth_mode_str);

  bool matched = false;
  for (int mode = static_cast<int>(sl::DEPTH_MODE::NONE);
    mode < static_cast<int>(sl::DEPTH_MODE::LAST); ++mode)
  {
    std::string test_str =
      sl::toString(static_cast<sl::DEPTH_MODE>(mode)).c_str();
    std::replace(
      test_str.begin(), test_str.end(), ' ',
      '_');    // Replace spaces with underscores to match the YAML setting
    if (test_str == depth_mode_str) {
      matched = true;
      mDepthMode = static_cast<sl::DEPTH_MODE>(mode);
      break;
    }
  }

  if (!matched) {
    RCLCPP_WARN(
      get_logger(),
      "The parameter 'depth.depth_mode' contains a not valid string. "
      "Please check it in 'common.yaml'.");
    RCLCPP_WARN(get_logger(), "Using default DEPTH_MODE.");
    mDepthMode = sl::DEPTH_MODE::PERFORMANCE;
  }

  if (mDepthMode == sl::DEPTH_MODE::NONE) {
    mDepthDisabled = true;
    mDepthStabilization = 0;
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Depth mode: " << sl::toString(mDepthMode).c_str()
                        << " - DEPTH DISABLED");
  } else {
    mDepthDisabled = false;
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Depth mode: " << sl::toString(mDepthMode).c_str()
                        << " [" << static_cast<int>(mDepthMode)
                        << "]");
  }

  if (!mDepthDisabled) {
    getParam(
      "depth.min_depth", mCamMinDepth, mCamMinDepth,
      " * Min depth [m]: ");
    getParam(
      "depth.max_depth", mCamMaxDepth, mCamMaxDepth,
      " * Max depth [m]: ");

    getParam(
      "depth.depth_stabilization", mDepthStabilization,
      mDepthStabilization, " * Depth Stabilization: ");
    if (mDepthStabilization < 0 || mDepthStabilization > 100) {
      mDepthStabilization = 1;
      RCLCPP_WARN_STREAM(
        get_logger(),
        "'depth.depth_stabilization' is not in the valid "
        "range [0,100]. Using the default value.");
    }

    getParam("depth.openni_depth_mode", mOpenniDepthMode, mOpenniDepthMode);
    RCLCPP_INFO(
      get_logger(), " * OpenNI mode (16bit point cloud): %s",
      mOpenniDepthMode ? "TRUE" : "FALSE");

    getParam("depth.point_cloud_freq", mPcPubRate, mPcPubRate, "", true);
    if (mPcPubRate > mPubFrameRate) {
      RCLCPP_WARN(
        get_logger(),
        "'point_cloud_freq' cannot be bigger than 'pub_frame_rate'");
      mPcPubRate = mPubFrameRate;
    }
    if (mPcPubRate < 0.1) {
      RCLCPP_WARN(
        get_logger(),
        "'point_cloud_freq' cannot be lower than 0.1 Hz or negative.");
      mPcPubRate = mPubFrameRate;
    }
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * [DYN] Point cloud rate [Hz]: " << mPcPubRate);

    getParam(
      "depth.depth_confidence", mDepthConf, mDepthConf,
      " * [DYN] Depth Confidence: ", true);
    getParam(
      "depth.depth_texture_conf", mDepthTextConf, mDepthTextConf,
      " * [DYN] Depth Texture Confidence: ", true);
    getParam(
      "depth.remove_saturated_areas", mRemoveSatAreas, mRemoveSatAreas,
      "", true);
    RCLCPP_INFO(
      get_logger(), " * [DYN] Remove saturated areas: %s",
      mRemoveSatAreas ? "TRUE" : "FALSE");
    // ------------------------------------------
  }
}

void ZedCamera::getSensorsParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** SENSORS STACK parameters ***");
  if (sl_tools::isZED(mCamUserModel)) {
    RCLCPP_WARN(
      get_logger(),
      "!!! SENSORS parameters are not used with ZED !!!");
    return;
  }

  getParam("sensors.publish_imu_tf", mPublishImuTF, mPublishImuTF);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Broadcast IMU TF [not for ZED]: "
      << (mPublishImuTF ? "TRUE" : "FALSE"));

  getParam("sensors.sensors_image_sync", mSensCameraSync, mSensCameraSync);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Sensors Camera Sync: "
      << (mSensCameraSync ? "TRUE" : "FALSE"));

  getParam("sensors.sensors_pub_rate", mSensPubRate, mSensPubRate);
  if (mSensPubRate < mCamGrabFrameRate) {
    mSensPubRate = mCamGrabFrameRate;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Sensors publishing rate: " << mSensPubRate << " Hz");
}

void ZedCamera::getMappingParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** Spatial Mapping parameters ***");

  getParam("mapping.mapping_enabled", mMappingEnabled, mMappingEnabled);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Spatial Mapping Enabled: "
      << (mMappingEnabled ? "TRUE" : "FALSE"));

  getParam(
    "mapping.resolution", mMappingRes, mMappingRes,
    " * Spatial Mapping resolution [m]: ");
  getParam("mapping.max_mapping_range", mMappingRangeMax, mMappingRangeMax);
  if (mMappingRangeMax == -1.0f) {
    RCLCPP_INFO(get_logger(), " * 3D Max Mapping range: AUTO");
  } else {
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * 3D Max Mapping range [m]: " << mMappingRangeMax);
  }
  getParam(
    "mapping.fused_pointcloud_freq", mFusedPcPubRate, mFusedPcPubRate,
    " * Map publishing rate [Hz]: ");

  getParam(
    "mapping.clicked_point_topic", mClickedPtTopic, mClickedPtTopic,
    " * Clicked point topic: ");

  getParam(
    "mapping.pd_max_distance_threshold", mPdMaxDistanceThreshold,
    mPdMaxDistanceThreshold, " * Plane Det. Max Dist. Thresh.: ");
  getParam(
    "mapping.pd_normal_similarity_threshold",
    mPdNormalSimilarityThreshold, mPdNormalSimilarityThreshold,
    " * Plane Det. Normals Sim. Thresh.: ");
}

void ZedCamera::getPosTrackingParams()
{
  rclcpp::Parameter paramVal;
  std::string paramName;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** POSITIONAL TRACKING parameters ***");

  getParam(
    "pos_tracking.pos_tracking_enabled", mPosTrackingEnabled,
    mPosTrackingEnabled);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Positional tracking enabled: "
      << (mPosTrackingEnabled ? "TRUE" : "FALSE"));

  std::string pos_trk_mode;
  getParam("pos_tracking.pos_tracking_mode", pos_trk_mode, pos_trk_mode);
  if (pos_trk_mode == "GEN_1") {
    mPosTrkMode = sl::POSITIONAL_TRACKING_MODE::GEN_1;
  } else if (pos_trk_mode == "GEN_2") {
    mPosTrkMode = sl::POSITIONAL_TRACKING_MODE::GEN_2;
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "'pos_tracking.pos_tracking_mode' not valid ('"
        << pos_trk_mode << "'). Using default value.");
    mPosTrkMode = sl::POSITIONAL_TRACKING_MODE::GEN_2;
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Positional tracking mode: "
      << sl::toString(mPosTrkMode).c_str());

  mBaseFrameId = mCameraName;
  mBaseFrameId += "_camera_link";

  getParam(
    "pos_tracking.map_frame", mMapFrameId, mMapFrameId,
    " * Map frame id: ");
  getParam(
    "pos_tracking.odometry_frame", mOdomFrameId, mOdomFrameId,
    " * Odometry frame id: ");

  getParam("pos_tracking.publish_tf", mPublishTF, mPublishTF);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Broadcast Odometry TF: "
      << (mPublishTF ? "TRUE" : "FALSE"));
  if (mPublishTF) {
    getParam("pos_tracking.publish_map_tf", mPublishMapTF, mPublishMapTF);
  } else {
    mPublishMapTF = false;
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Broadcast Pose TF: "
      << (mPublishMapTF ? "TRUE" : "FALSE"));

  getParam(
    "pos_tracking.depth_min_range", mPosTrackDepthMinRange,
    mPosTrackDepthMinRange, " * [DYN] Depth minimum range: ");
  getParam(
    "pos_tracking.transform_time_offset", mTfOffset, mTfOffset,
    " * [DYN] TF timestamp offset: ", true);
  getParam(
    "pos_tracking.path_pub_rate", mPathPubRate, mPathPubRate,
    " * [DYN] Path publishing rate: ", true);
  getParam("pos_tracking.path_max_count", mPathMaxCount, mPathMaxCount);
  if (mPathMaxCount < 2 && mPathMaxCount != -1) {
    mPathMaxCount = 2;
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Path history lenght: " << mPathMaxCount);

  paramName = "pos_tracking.initial_base_pose";
  declare_parameter(
    paramName, rclcpp::ParameterValue(mInitialBasePose),
    read_only_descriptor);
  if (!get_parameter(paramName, mInitialBasePose)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value");
    mInitialBasePose = std::vector<double>(6, 0.0);
  }
  if (mInitialBasePose.size() < 6) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' must be a vector of 6 values of double type");
    mInitialBasePose = std::vector<double>(6, 0.0);
  }
  RCLCPP_INFO(
    get_logger(), " * Initial pose: [%g,%g,%g,%g,%g,%g,]",
    mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2],
    mInitialBasePose[3], mInitialBasePose[4], mInitialBasePose[5]);

  // TODO(Walter) Fix this as soon as the `sl::Fusion` module will support loop
  // closure and odometry
  if (mGnssFusionEnabled) {
    mAreaMemory = false;
    RCLCPP_INFO(get_logger(), " * Area Memory: FALSE - Forced by GNSS usage");
    RCLCPP_INFO(
      get_logger(),
      "   Note: loop closure (Area Memory) is disabled when using "
      "GNSS fusion");
  } else {
    getParam("pos_tracking.area_memory", mAreaMemory, mAreaMemory);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Area Memory: " << (mAreaMemory ? "TRUE" : "FALSE"));
    getParam(
      "pos_tracking.area_memory_db_path", mAreaMemoryDbPath,
      mAreaMemoryDbPath, " * Area Memory DB: ");
  }
  getParam("pos_tracking.set_as_static", mSetAsStatic, mSetAsStatic);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Camera is static: "
      << (mSetAsStatic ? "TRUE" : "FALSE"));
  getParam(
    "pos_tracking.set_gravity_as_origin", mSetGravityAsOrigin,
    mSetGravityAsOrigin);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Gravity as origin [not for ZED]: "
      << (mSetGravityAsOrigin ? "TRUE" : "FALSE"));
  getParam("pos_tracking.imu_fusion", mImuFusion, mImuFusion);
  RCLCPP_INFO_STREAM(
    get_logger(), " * IMU Fusion [not for ZED]: "
      << (mImuFusion ? "TRUE" : "FALSE"));
  getParam("pos_tracking.floor_alignment", mFloorAlignment, mFloorAlignment);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Floor Alignment: "
      << (mFloorAlignment ? "TRUE" : "FALSE"));
  getParam(
    "pos_tracking.reset_odom_with_loop_closure",
    mResetOdomWhenLoopClosure, mResetOdomWhenLoopClosure);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Reset Odometry with Loop Closure: "
      << (mResetOdomWhenLoopClosure ? "TRUE" : "FALSE"));
  getParam("pos_tracking.two_d_mode", mTwoDMode, mTwoDMode);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * 2D mode: " << (mTwoDMode ? "TRUE" : "FALSE"));
  if (mTwoDMode) {
    getParam(
      "pos_tracking.fixed_z_value", mFixedZValue, mFixedZValue,
      " * Fixed Z value: ");
  }
}

void ZedCamera::getGnssFusionParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** GNSS FUSION parameters ***");
  if (sl_tools::isZED(mCamUserModel)) {
    RCLCPP_WARN(
      get_logger(),
      "!!! GNSS FUSION module cannot be enabled with ZED!!!");
    return;
  }

  getParam(
    "gnss_fusion.gnss_fusion_enabled", mGnssFusionEnabled,
    mGnssFusionEnabled);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * GNSS fusion enabled: " << (mGnssFusionEnabled ? "TRUE" : "FALSE"));

  if (ZED_SDK_MAJOR_VERSION == 4 && ZED_SDK_MINOR_VERSION == 1 && ZED_SDK_PATCH_VERSION == 0) {
    if (mGnssFusionEnabled) {
      RCLCPP_FATAL(
        get_logger(),
        "GNSS Fusion is temporarely disabled with ZED SDK v4.1. This module will be enabled in a future release of the ZED SDK.");
      exit(EXIT_FAILURE);
    }
  }

  if (mGnssFusionEnabled) {
    mGnssFrameId = mCameraName + "_gnss_link";

    getParam(
      "gnss_fusion.gnss_fix_topic", mGnssTopic, mGnssTopic,
      " * GNSS topic name: ");
    getParam(
      "gnss_fusion.enable_reinitialization", mGnssEnableReinitialization,
      mGnssEnableReinitialization);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * GNSS Reinitialization: "
        << (mGnssEnableReinitialization ? "TRUE" : "FALSE"));
    getParam(
      "gnss_fusion.enable_rolling_calibration",
      mGnssEnableRollingCalibration, mGnssEnableRollingCalibration);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * GNSS Rolling Calibration: "
        << (mGnssEnableRollingCalibration ? "TRUE" : "FALSE"));
    getParam(
      "gnss_fusion.enable_translation_uncertainty_target",
      mGnssEnableTranslationUncertaintyTarget,
      mGnssEnableTranslationUncertaintyTarget);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * GNSS Transl. Uncert. Target: "
        << (mGnssEnableTranslationUncertaintyTarget ? "TRUE" : "FALSE"));
    getParam(
      "gnss_fusion.gnss_vio_reinit_threshold", mGnssVioReinitThreshold,
      mGnssVioReinitThreshold, " * GNSS VIO Reinit. Thresh.: ");
    getParam(
      "gnss_fusion.target_translation_uncertainty",
      mGnssTargetTranslationUncertainty,
      mGnssTargetTranslationUncertainty,
      " * GNSS Target Transl. Uncert.: ");
    getParam(
      "gnss_fusion.target_yaw_uncertainty", mGnssTargetYawUncertainty,
      mGnssTargetYawUncertainty, " * GNSS Target Yaw Uncert.: ");
    getParam(
      "gnss_fusion.gnss_zero_altitude", mGnssZeroAltitude,
      mGnssZeroAltitude);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * GNSS Zero Altitude: " << (mGnssZeroAltitude ? "TRUE" : "FALSE"));

    getParam(
      "gnss_fusion.h_covariance_mul", mGnssHcovMul, mGnssHcovMul,
      " * Horiz. Covariance mult.: ");
    getParam(
      "gnss_fusion.v_covariance_mul", mGnssVcovMul, mGnssVcovMul,
      " * Vert. Covariance mult.: ");

    getParam("gnss_fusion.publish_utm_tf", mPublishUtmTf, mPublishUtmTf);
    RCLCPP_INFO_STREAM(
      get_logger(), " * Publish UTM TF: "
        << (mPublishUtmTf ? "TRUE" : "FALSE"));
    getParam(
      "gnss_fusion.broadcast_utm_transform_as_parent_frame",
      mUtmAsParent, mUtmAsParent);
    RCLCPP_INFO_STREAM(
      get_logger(), " * Publish UTM TF as parent of 'map': "
        << (mUtmAsParent ? "TRUE" : "FALSE"));
  }
}

void ZedCamera::getOdParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** Object Det. parameters ***");
  if (sl_tools::isZED(mCamUserModel)) {
    RCLCPP_WARN(get_logger(), "!!! OD parameters are not used with ZED!!!");
    return;
  }

  getParam("object_detection.od_enabled", mObjDetEnabled, mObjDetEnabled);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Object Det. enabled: "
      << (mObjDetEnabled ? "TRUE" : "FALSE"));

  std::string model_str;
  getParam("object_detection.model", model_str, model_str);

  DEBUG_STREAM_OD(" 'object_detection.model': " << model_str.c_str());

  bool matched = false;
  for (int idx =
    static_cast<int>(sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_FAST);
    idx < static_cast<int>(sl::OBJECT_DETECTION_MODEL::LAST);
    idx++)
  {
    sl::OBJECT_DETECTION_MODEL test_model =
      static_cast<sl::OBJECT_DETECTION_MODEL>(idx);
    std::string test_model_str = sl::toString(test_model).c_str();
    std::replace(
      test_model_str.begin(), test_model_str.end(), ' ',
      '_');    // Replace spaces with underscores to match the YAML setting
    // DEBUG_OD(" Comparing '%s' to '%s'", test_model_str.c_str(),
    // model_str.c_str());
    if (model_str == test_model_str) {
      mObjDetModel = test_model;
      matched = true;
      break;
    }
  }
  if (!matched) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The value of the parameter 'object_detection.model' is not valid: '"
        << model_str << "'. Using the default value.");
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Object Det. model: "
      << sl::toString(mObjDetModel).c_str());

  getParam(
    "object_detection.allow_reduced_precision_inference",
    mObjDetReducedPrecision, mObjDetReducedPrecision);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Object Det. allow reduced precision: "
      << (mObjDetReducedPrecision ? "TRUE" : "FALSE"));
  getParam(
    "object_detection.max_range", mObjDetMaxRange, mObjDetMaxRange,
    " * Object Det. maximum range [m]: ");
  getParam(
    "object_detection.confidence_threshold", mObjDetConfidence,
    mObjDetConfidence, " * Object Det. min. confidence: ", true);
  getParam(
    "object_detection.prediction_timeout", mObjDetPredTimeout,
    mObjDetPredTimeout, " * Object Det. prediction timeout [sec]: ");
  getParam(
    "object_detection.object_tracking_enabled", mObjDetTracking,
    mObjDetTracking);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Object Det. tracking: "
      << (mObjDetTracking ? "TRUE" : "FALSE"));

  int filtering_mode = static_cast<int>(mObjFilterMode);
  getParam("object_detection.filtering_mode", filtering_mode, filtering_mode);
  mObjFilterMode = static_cast<sl::OBJECT_FILTERING_MODE>(filtering_mode);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Object Filtering mode: "
      << filtering_mode << " - "
      << sl::toString(mObjFilterMode).c_str());
  getParam(
    "object_detection.mc_people", mObjDetPeopleEnable,
    mObjDetPeopleEnable, "", true);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * MultiClassBox people: " << (mObjDetPeopleEnable ? "TRUE" : "FALSE"));
  getParam(
    "object_detection.mc_vehicle", mObjDetVehiclesEnable,
    mObjDetVehiclesEnable, "", true);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * MultiClassBox vehicles: "
      << (mObjDetVehiclesEnable ? "TRUE" : "FALSE"));
  getParam(
    "object_detection.mc_bag", mObjDetBagsEnable, mObjDetBagsEnable, "",
    true);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * MultiClassBox bags: " << (mObjDetBagsEnable ? "TRUE" : "FALSE"));
  getParam(
    "object_detection.mc_animal", mObjDetAnimalsEnable,
    mObjDetAnimalsEnable, "", true);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * MultiClassBox animals: "
      << (mObjDetAnimalsEnable ? "TRUE" : "FALSE"));
  getParam(
    "object_detection.mc_electronics", mObjDetElectronicsEnable,
    mObjDetElectronicsEnable, "", true);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * MultiClassBox electronics: "
      << (mObjDetElectronicsEnable ? "TRUE" : "FALSE"));
  getParam(
    "object_detection.mc_fruit_vegetable", mObjDetFruitsEnable,
    mObjDetFruitsEnable, "", true);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * MultiClassBox fruits and vegetables: "
      << (mObjDetFruitsEnable ? "TRUE" : "FALSE"));
  getParam(
    "object_detection.mc_sport", mObjDetSportEnable, mObjDetSportEnable,
    "", true);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * MultiClassBox sport-related objects: "
      << (mObjDetSportEnable ? "TRUE" : "FALSE"));
}

void ZedCamera::getBodyTrkParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** Body Track. parameters ***");
  if (sl_tools::isZED(mCamUserModel)) {
    RCLCPP_WARN(
      get_logger(),
      "!!! Body Tracking parameters are not used with ZED!!!");
    return;
  }

  getParam("body_tracking.bt_enabled", mBodyTrkEnabled, mBodyTrkEnabled);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Body Track. enabled: "
      << (mBodyTrkEnabled ? "TRUE" : "FALSE"));

  bool matched = false;

  std::string model_str = "HUMAN_BODY_FAST";
  getParam("body_tracking.model", model_str, model_str);

  for (int idx = static_cast<int>(sl::BODY_TRACKING_MODEL::HUMAN_BODY_FAST);
    idx < static_cast<int>(sl::BODY_TRACKING_MODEL::LAST); idx++)
  {
    sl::BODY_TRACKING_MODEL test_model =
      static_cast<sl::BODY_TRACKING_MODEL>(idx);
    std::string test_model_str = sl::toString(test_model).c_str();
    std::replace(
      test_model_str.begin(), test_model_str.end(), ' ',
      '_');    // Replace spaces with underscores to match the YAML setting
    // DEBUG_BT(" Comparing '%s' to '%s'", test_model_str.c_str(),
    // model_str.c_str());
    if (model_str == test_model_str) {
      mBodyTrkModel = test_model;
      matched = true;
      break;
    }
  }
  if (!matched) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The value of the parameter 'body_tracking.model' is not valid: '"
        << model_str << "'. Using the default value.");
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Body Track. model: "
      << sl::toString(mBodyTrkModel).c_str());

  std::string fmt_str = "BODY_70";
  getParam("body_tracking.body_format", fmt_str, fmt_str);

  for (int idx = static_cast<int>(sl::BODY_FORMAT::BODY_18);
    idx < static_cast<int>(sl::BODY_FORMAT::LAST); idx++)
  {
    sl::BODY_FORMAT test_fmt = static_cast<sl::BODY_FORMAT>(idx);
    std::string test_fmt_str = sl::toString(test_fmt).c_str();
    std::replace(
      test_fmt_str.begin(), test_fmt_str.end(), ' ',
      '_');    // Replace spaces with underscores to match the YAML setting
    // DEBUG_BT(" Comparing '%s' to '%s'", test_fmt_str.c_str(),
    // test_fmt.c_str());
    if (fmt_str == test_fmt_str) {
      mBodyTrkFmt = test_fmt;
      matched = true;
      break;
    }
  }
  if (!matched) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The value of the parameter 'body_tracking.body_format' is not valid: '"
        << fmt_str << "'. Using the default value.");
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Body Track. format: "
      << sl::toString(mBodyTrkFmt).c_str());

  getParam(
    "body_tracking.allow_reduced_precision_inference",
    mBodyTrkReducedPrecision, mBodyTrkReducedPrecision);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Body Track. allow reduced precision: "
      << (mObjDetReducedPrecision ? "TRUE" : "FALSE"));
  getParam(
    "body_tracking.max_range", mBodyTrkMaxRange, mBodyTrkMaxRange,
    " * Body Track. maximum range [m]: ");

  std::string body_sel_str = "FULL";
  getParam(
    "body_tracking.body_kp_selection", body_sel_str, body_sel_str,
    " * Body Track. KP selection: ");

  DEBUG_BT("body_selection.body_kp_selection: %s", body_sel_str.c_str());

  for (int idx = static_cast<int>(sl::BODY_KEYPOINTS_SELECTION::FULL);
    idx < static_cast<int>(sl::BODY_KEYPOINTS_SELECTION::LAST); idx++)
  {
    sl::BODY_KEYPOINTS_SELECTION test_kp_sel =
      static_cast<sl::BODY_KEYPOINTS_SELECTION>(idx);
    std::string test_body_sel_str = sl::toString(test_kp_sel).c_str();
    std::replace(
      test_body_sel_str.begin(), test_body_sel_str.end(), ' ',
      '_');    // Replace spaces with underscores to match the YAML setting
    DEBUG_BT(
      " Comparing '%s' to '%s'", test_body_sel_str.c_str(),
      body_sel_str.c_str());
    if (body_sel_str == test_body_sel_str) {
      mBodyTrkKpSelection = test_kp_sel;
      matched = true;
      break;
    }
  }
  if (!matched) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The value of the parameter "
      "'body_tracking.body_kp_selection' is not valid: '"
        << body_sel_str << "'. Using the default value.");
  }

  getParam(
    "body_tracking.enable_body_fitting", mBodyTrkFitting,
    mBodyTrkFitting);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Body fitting: "
      << (mBodyTrkFitting ? "TRUE" : "FALSE"));

  getParam(
    "body_tracking.enable_tracking", mBodyTrkEnableTracking,
    mBodyTrkEnableTracking);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Body joints tracking: "
      << (mBodyTrkEnableTracking ? "TRUE" : "FALSE"));

  getParam(
    "body_tracking.prediction_timeout_s", mBodyTrkPredTimeout,
    mBodyTrkPredTimeout, " * Body Track. prediction timeout [sec]: ");

  getParam(
    "body_tracking.confidence_threshold", mBodyTrkConfThresh,
    mBodyTrkConfThresh, " * Body Track. confidence thresh.: ", true);

  getParam(
    "body_tracking.minimum_keypoints_threshold", mBodyTrkMinKp,
    mBodyTrkMinKp, " * Body Track. min. KP thresh.: ", true);
}

void ZedCamera::getStreamingServerParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** Streaming Server parameters ***");

  bool stream_server = false;
  getParam("stream_server.stream_enabled", stream_server, stream_server);
  mStreamingServerRequired = stream_server;
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Streaming Server enabled: "
      << (mStreamingServerRequired ? "TRUE" : "FALSE"));

  std::string codec = "H264";
  getParam("stream_server.codec", codec, codec);
  if (codec == "H264") {
    mStreamingServerCodec = sl::STREAMING_CODEC::H264;
    RCLCPP_INFO(get_logger(), " * Stream codec: H264");
  } else if (codec == "H265") {
    mStreamingServerCodec = sl::STREAMING_CODEC::H265;
    RCLCPP_INFO(get_logger(), " * Stream codec: H265");
  } else {
    mStreamingServerCodec = sl::STREAMING_CODEC::H264;
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.codec': " << codec <<
        ". Using the default value.");
    RCLCPP_INFO(get_logger(), " * Stream codec: H264");
  }

  getParam("stream_server.port", mStreamingServerPort, mStreamingServerPort, " * Stream port:");

  getParam("stream_server.bitrate", mStreamingServerBitrate, mStreamingServerBitrate);
  if (mStreamingServerBitrate < 1000) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.bitrate': " << codec <<
        ". The minimum allowed value is 1000");
    mStreamingServerBitrate = 1000;
  }
  if (mStreamingServerBitrate > 60000) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.bitrate': " << codec <<
        ". The maximum allowed value is 60000");
    mStreamingServerBitrate = 60000;
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Stream bitrate: " << mStreamingServerBitrate);

  getParam("stream_server.gop_size", mStreamingServerGopSize, mStreamingServerGopSize);
  if (mStreamingServerGopSize < -1) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.gop_size': " << codec <<
        ". The minimum allowed value is -1");
    mStreamingServerGopSize = -1;
  }
  if (mStreamingServerGopSize > 256) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.gop_size': " << codec <<
        ". The maximum allowed value is 256");
    mStreamingServerGopSize = 256;
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Stream GOP size: " << mStreamingServerGopSize);

  getParam("stream_server.chunk_size", mStreamingServerChunckSize, mStreamingServerChunckSize);
  if (mStreamingServerChunckSize < 1024) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.chunk_size': " << codec <<
        ". The minimum allowed value is 1024");
    mStreamingServerChunckSize = 1024;
  }
  if (mStreamingServerChunckSize > 65000) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.chunk_size': " << codec <<
        ". The maximum allowed value is 65000");
    mStreamingServerChunckSize = 65000;
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Stream Chunk size: " << mStreamingServerChunckSize);

  getParam(
    "stream_server.adaptative_bitrate", mStreamingServerAdaptiveBitrate,
    mStreamingServerAdaptiveBitrate);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Adaptive bitrate: " << (mStreamingServerAdaptiveBitrate ? "TRUE" : "FALSE"));

  getParam(
    "stream_server.target_framerate", mStreamingServerTargetFramerate,
    mStreamingServerTargetFramerate, " * Target frame rate:");
}

void ZedCamera::getAdvancedParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** Advanced parameters ***");

  getParam(
    "advanced.thread_sched_policy", mThreadSchedPolicy,
    mThreadSchedPolicy, " * Thread sched. policy: ");

  if (mThreadSchedPolicy == "SCHED_FIFO" || mThreadSchedPolicy == "SCHED_RR") {
    if (!sl_tools::checkRoot()) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "'sudo' permissions required to set "
          << mThreadSchedPolicy
          << " thread scheduling policy. Using Linux "
          "default [SCHED_OTHER]");
      mThreadSchedPolicy = "SCHED_OTHER";
    } else {
      getParam(
        "advanced.thread_grab_priority", mThreadPrioGrab,
        mThreadPrioGrab, " * Grab thread priority: ");
      getParam(
        "advanced.thread_sensor_priority", mThreadPrioSens,
        mThreadPrioSens, " * Sensors thread priority: ");
      getParam(
        "advanced.thread_pointcloud_priority", mThreadPrioPointCloud,
        mThreadPrioPointCloud, " * Point Cloud thread priority: ");
    }
  }
}

rcl_interfaces::msg::SetParametersResult ZedCamera::callback_paramChange(
  std::vector<rclcpp::Parameter> parameters)
{
  if (mDebugMode) {
    DEBUG_STREAM_COMM("Parameter change callback");
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Modifying " << parameters.size() << " parameters");

  int count = 0;

  for (const rclcpp::Parameter & param : parameters) {
    count++;

    if (mDebugMode) {
      DEBUG_STREAM_COMM("Param #" << count << ": " << param.get_name());
    }

    if (sl_tools::isZEDX(mCamRealModel)) {
      if (param.get_name() == "video.exposure_time") {
        rclcpp::ParameterType correctType =
          rclcpp::ParameterType::PARAMETER_INTEGER;
        if (param.get_type() != correctType) {
          result.successful = false;
          result.reason =
            param.get_name() + " must be a " + rclcpp::to_string(correctType);
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        int val = param.as_int();

        if ((val < 28) || (val > 30000)) {
          result.successful = false;
          result.reason = param.get_name() +
            " must be a positive integer in the range [28,30000]";
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        mGmslExpTime = val;
        mCamAutoExpGain = false;

        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << val);
      } else if (param.get_name() == "video.auto_exposure_time_range_min") {
        rclcpp::ParameterType correctType =
          rclcpp::ParameterType::PARAMETER_INTEGER;
        if (param.get_type() != correctType) {
          result.successful = false;
          result.reason =
            param.get_name() + " must be a " + rclcpp::to_string(correctType);
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        int val = param.as_int();

        if ((val < 28) || (val > mGmslAutoExpTimeRangeMax)) {
          result.successful = false;
          result.reason = param.get_name() +
            " must be a positive integer in the range "
            "[28,auto_exposure_time_range_max]";
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        mGmslAutoExpTimeRangeMin = val;

        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << val);
      } else if (param.get_name() == "video.auto_exposure_time_range_max") {
        rclcpp::ParameterType correctType =
          rclcpp::ParameterType::PARAMETER_INTEGER;
        if (param.get_type() != correctType) {
          result.successful = false;
          result.reason =
            param.get_name() + " must be a " + rclcpp::to_string(correctType);
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        int val = param.as_int();

        int max_val = std::max(mCamGrabFrameRate * 1000, 30000);

        if ((val < mGmslAutoExpTimeRangeMin) || (val > max_val)) {
          result.successful = false;
          result.reason = param.get_name() +
            " must be a positive integer in the range "
            "[auto_exposure_time_range_min,max(30000,'general."
            "grab_frame_rate'x1000)]";
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        mGmslAutoExpTimeRangeMax = val;

        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << val);
      } else if (param.get_name() == "video.exposure_compensation") {
        rclcpp::ParameterType correctType =
          rclcpp::ParameterType::PARAMETER_INTEGER;
        if (param.get_type() != correctType) {
          result.successful = false;
          result.reason =
            param.get_name() + " must be a " + rclcpp::to_string(correctType);
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        int val = param.as_int();

        if ((val < 0) || (val > 1000)) {
          result.successful = false;
          result.reason = param.get_name() +
            " must be a positive integer in the range [0,100]";
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        mGmslExposureComp = val;

        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << val);
      } else if (param.get_name() == "video.analog_gain") {
        rclcpp::ParameterType correctType =
          rclcpp::ParameterType::PARAMETER_INTEGER;
        if (param.get_type() != correctType) {
          result.successful = false;
          result.reason =
            param.get_name() + " must be a " + rclcpp::to_string(correctType);
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        int val = param.as_int();

        if ((val < 1000) || (val > 16000)) {
          result.successful = false;
          result.reason =
            param.get_name() +
            " must be a positive integer in the range [1000,16000]";
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        mGmslAnalogGain = val;
        mCamAutoExpGain = false;

        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << val);
      } else if (param.get_name() == "video.auto_analog_gain_range_min") {
        rclcpp::ParameterType correctType =
          rclcpp::ParameterType::PARAMETER_INTEGER;
        if (param.get_type() != correctType) {
          result.successful = false;
          result.reason =
            param.get_name() + " must be a " + rclcpp::to_string(correctType);
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        int val = param.as_int();

        if ((val < 1000) || (val > mGmslAnalogGainRangeMax)) {
          result.successful = false;
          result.reason = param.get_name() +
            " must be a positive integer in the range "
            "[1000,auto_analog_gain_range_max]";
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        mGmslAnalogGainRangeMin = val;

        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << val);
      } else if (param.get_name() == "video.auto_analog_gain_range_max") {
        rclcpp::ParameterType correctType =
          rclcpp::ParameterType::PARAMETER_INTEGER;
        if (param.get_type() != correctType) {
          result.successful = false;
          result.reason =
            param.get_name() + " must be a " + rclcpp::to_string(correctType);
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        int val = param.as_int();

        if ((val < mGmslAnalogGainRangeMin) || (val > 16000)) {
          result.successful = false;
          result.reason = param.get_name() +
            " must be a positive integer in the range "
            "[auto_analog_gain_range_min,16000]";
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        mGmslAnalogGainRangeMax = val;

        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << val);
      } else if (param.get_name() == "video.digital_gain") {
        rclcpp::ParameterType correctType =
          rclcpp::ParameterType::PARAMETER_INTEGER;
        if (param.get_type() != correctType) {
          result.successful = false;
          result.reason =
            param.get_name() + " must be a " + rclcpp::to_string(correctType);
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        int val = param.as_int();

        if ((val < 1) || (val > 256)) {
          result.successful = false;
          result.reason = param.get_name() +
            " must be a positive integer in the range [1,256]";
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        mGmslDigitalGain = val;
        mCamAutoExpGain = false;

        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << val);
      } else if (param.get_name() == "video.auto_digital_gain_range_min") {
        rclcpp::ParameterType correctType =
          rclcpp::ParameterType::PARAMETER_INTEGER;
        if (param.get_type() != correctType) {
          result.successful = false;
          result.reason =
            param.get_name() + " must be a " + rclcpp::to_string(correctType);
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        int val = param.as_int();

        if ((val < 1) || (val > mGmslAutoDigitalGainRangeMax)) {
          result.successful = false;
          result.reason = param.get_name() +
            " must be a positive integer in the range "
            "[1,auto_digital_gain_range_max]";
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        mGmslAutoDigitalGainRangeMin = val;

        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << val);
      } else if (param.get_name() == "video.auto_digital_gain_range_max") {
        rclcpp::ParameterType correctType =
          rclcpp::ParameterType::PARAMETER_INTEGER;
        if (param.get_type() != correctType) {
          result.successful = false;
          result.reason =
            param.get_name() + " must be a " + rclcpp::to_string(correctType);
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        int val = param.as_int();

        if ((val < mGmslAutoDigitalGainRangeMin) || (val > 256)) {
          result.successful = false;
          result.reason = param.get_name() +
            " must be a positive integer in the range "
            "[auto_digital_gain_range_min,256]";
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        mGmslAutoDigitalGainRangeMax = val;

        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << val);
      } else if (param.get_name() == "video.denoising") {
        rclcpp::ParameterType correctType =
          rclcpp::ParameterType::PARAMETER_INTEGER;
        if (param.get_type() != correctType) {
          result.successful = false;
          result.reason =
            param.get_name() + " must be a " + rclcpp::to_string(correctType);
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        int val = param.as_int();

        if ((val < 0) || (val > 100)) {
          result.successful = false;
          result.reason = param.get_name() +
            " must be a positive integer in the range [0,100]";
          RCLCPP_WARN_STREAM(get_logger(), result.reason);
          break;
        }

        mGmslDenoising = val;

        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << val);
      }
    }

    if (param.get_name() == "general.pub_frame_rate") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      double val = param.as_double();

      if ((val <= 0.0) || (val > mCamGrabFrameRate)) {
        result.successful = false;
        result.reason =
          param.get_name() +
          " must be positive and minor or equal to `grab_frame_rate`";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mPubFrameRate = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "video.brightness") {
      if (sl_tools::isZEDX(mCamRealModel)) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' not available for "
                                      << sl::toString(mCamRealModel).c_str());
        break;
      }

      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      int val = param.as_int();

      if ((val < 0) || (val > 8)) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a positive integer in the range [0,8]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mCamBrightness = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "video.contrast") {
      if (sl_tools::isZEDX(mCamRealModel)) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' not available for "
                                      << sl::toString(mCamRealModel).c_str());
        break;
      }

      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      int val = param.as_int();

      if ((val < 0) || (val > 8)) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a positive integer in the range [0,8]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mCamContrast = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "video.hue") {
      if (sl_tools::isZEDX(mCamRealModel)) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' not available for "
                                      << sl::toString(mCamRealModel).c_str());
        break;
      }

      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      int val = param.as_int();

      if ((val < 0) || (val > 11)) {
        result.successful = false;
        result.reason = param.get_name() +
          " must be a positive integer in the range [0,11]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mCamHue = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "video.saturation") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      int val = param.as_int();

      if ((val < 0) || (val > 8)) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a positive integer in the range [0,8]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mCamSaturation = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "video.sharpness") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      int val = param.as_int();

      if ((val < 0) || (val > 8)) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a positive integer in the range [0,8]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mCamSharpness = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "video.gamma") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      int val = param.as_int();

      if ((val < 0) || (val > 8)) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a positive integer in the range [0,8]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mCamGamma = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "video.auto_exposure_gain") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      bool val = param.as_bool();

      if (val != mCamAutoExpGain) {
        mTriggerAutoExpGain = true;
      }

      mCamAutoExpGain = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "video.exposure") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      int val = param.as_int();

      if ((val < 0) || (val > 100)) {
        result.successful = false;
        result.reason = param.get_name() +
          " must be a positive integer in the range [0,100]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mCamExposure = val;
      mCamAutoExpGain = false;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "video.gain") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      int val = param.as_int();

      if ((val < 0) || (val > 100)) {
        result.successful = false;
        result.reason = param.get_name() +
          " must be a positive integer in the range [0,100]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mCamGain = val;
      mCamAutoExpGain = false;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "video.auto_whitebalance") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      bool val = param.as_bool();

      if (val != mCamAutoWB) {
        mTriggerAutoWB = true;
      }

      mCamAutoWB = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "video.whitebalance_temperature") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      int val = param.as_int();

      if ((val < 28) || (val > 65)) {
        result.successful = false;
        result.reason = param.get_name() +
          " must be a positive integer in the range [28,65]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mCamWBTemp = val * 100;
      mCamAutoWB = false;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "depth.point_cloud_freq") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      double val = param.as_double();

      if ((val <= 0.0) || (val > mCamGrabFrameRate)) {
        result.successful = false;
        result.reason = param.get_name() +
          " must be positive and minor of `grab_frame_rate`";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mPcPubRate = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "depth.depth_confidence") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      int val = param.as_int();

      if ((val < 0) || (val > 100)) {
        result.successful = false;
        result.reason = param.get_name() +
          " must be a positive integer in the range [0,100]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mDepthConf = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "depth.depth_texture_conf") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      int val = param.as_int();

      if ((val < 0) || (val > 100)) {
        result.successful = false;
        result.reason = param.get_name() +
          " must be a positive integer in the range [0,100]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mDepthTextConf = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "depth.remove_saturated_areas") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mRemoveSatAreas = param.as_bool();

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Parameter '" << param.get_name()
                      << "' correctly set to "
                      << (mRemoveSatAreas ? "TRUE" : "FALSE"));
    } else if (param.get_name() == "pos_tracking.transform_time_offset") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      double val = param.as_double();
      mTfOffset = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "pos_tracking.path_pub_rate") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      double val = param.as_double();

      if ((val <= 0.0) || (val > mCamGrabFrameRate)) {
        result.successful = false;
        result.reason =
          param.get_name() +
          " must be positive and minor of `general.grab_frame_rate`";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mPathPubRate = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "mapping.fused_pointcloud_freq") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      double val = param.as_double();

      if ((val <= 0.0) || (val > mPcPubRate)) {
        result.successful = false;
        result.reason = param.get_name() +
          " must be positive and minor of `point_cloud_freq`";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mFusedPcPubRate = val;
      startFusedPcTimer(mFusedPcPubRate);  // Reset publishing timer

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "object_detection.confidence_threshold") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      double val = param.as_double();

      if ((val < 0.0) || (val > 100.0)) {
        result.successful = false;
        result.reason =
          param.get_name() +
          " must be positive double value in the range [0.0,100.0]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mObjDetConfidence = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() == "object_detection.mc_people") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mObjDetPeopleEnable = param.as_bool();

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Parameter '"
          << param.get_name() << "' correctly set to "
          << (mObjDetPeopleEnable ? "TRUE" : "FALSE"));
    } else if (param.get_name() == "object_detection.mc_vehicle") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mObjDetVehiclesEnable = param.as_bool();

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Parameter '"
          << param.get_name() << "' correctly set to "
          << (mObjDetVehiclesEnable ? "TRUE" : "FALSE"));
    } else if (param.get_name() == "object_detection.mc_bag") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mObjDetBagsEnable = param.as_bool();

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Parameter '"
          << param.get_name() << "' correctly set to "
          << (mObjDetBagsEnable ? "TRUE" : "FALSE"));
    } else if (param.get_name() == "object_detection.mc_animal") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mObjDetAnimalsEnable = param.as_bool();

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Parameter '"
          << param.get_name() << "' correctly set to "
          << (mObjDetAnimalsEnable ? "TRUE" : "FALSE"));
    } else if (param.get_name() == "object_detection.mc_electronics") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mObjDetElectronicsEnable = param.as_bool();

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Parameter '"
          << param.get_name() << "' correctly set to "
          << (mObjDetElectronicsEnable ? "TRUE" : "FALSE"));
    } else if (param.get_name() == "object_detection.mc_fruit_vegetable") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mObjDetFruitsEnable = param.as_bool();

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Parameter '"
          << param.get_name() << "' correctly set to "
          << (mObjDetFruitsEnable ? "TRUE" : "FALSE"));
    } else if (param.get_name() == "object_detection.mc_sport") {
      rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mObjDetSportEnable = param.as_bool();

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Parameter '"
          << param.get_name() << "' correctly set to "
          << (mObjDetSportEnable ? "TRUE" : "FALSE"));
    } else if (param.get_name() == "body_tracking.confidence_threshold") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      double val = param.as_double();

      if ((val < 0.0) || (val >= 100.0)) {
        result.successful = false;
        result.reason =
          param.get_name() +
          " must be positive double value in the range [0.0,100.0[";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mBodyTrkConfThresh = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    } else if (param.get_name() ==
      "body_tracking.minimum_keypoints_threshold")
    {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_INTEGER;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      double val = param.as_int();

      if ((val < 0) || (val > 70)) {
        result.successful = false;
        result.reason = param.get_name() +
          " must be positive double value in the range [0,70]";
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }

      mBodyTrkMinKp = val;

      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << val);
    }
  }

  if (result.successful) {
    RCLCPP_INFO_STREAM(
      get_logger(), "Correctly set " << count << "/"
                                     << parameters.size()
                                     << " parameters");
  } else {
    RCLCPP_INFO_STREAM(
      get_logger(), "Correctly set " << count - 1 << "/"
                                     << parameters.size()
                                     << " parameters");
  }

  return result;
}

void ZedCamera::setTFCoordFrameNames()
{
  // ----> Coordinate frames
  mCameraFrameId = mCameraName + "_camera_center";
  mLeftCamFrameId = mCameraName + "_left_camera_frame";
  mLeftCamOptFrameId = mCameraName + "_left_camera_optical_frame";
  mRightCamFrameId = mCameraName + "_right_camera_frame";
  mRightCamOptFrameId = mCameraName + "_right_camera_optical_frame";

  mImuFrameId = mCameraName + "_imu_link";
  mBaroFrameId = mCameraFrameId;         // mCameraName + "_baro_link";
  mMagFrameId = mImuFrameId;             // mCameraName + "_mag_link";
  mTempLeftFrameId = mLeftCamFrameId;    // mCameraName + "_temp_left_link";
  mTempRightFrameId = mRightCamFrameId;  // mCameraName + "_temp_right_link";

  mDepthFrameId = mLeftCamFrameId;
  mDepthOptFrameId = mLeftCamOptFrameId;
  mPointCloudFrameId = mDepthFrameId;

  // Note: Depth image frame id must match color image frame id
  mCloudFrameId = mDepthOptFrameId;
  mRgbFrameId = mDepthFrameId;
  mRgbOptFrameId = mDepthOptFrameId;
  mDisparityFrameId = mDepthFrameId;
  mDisparityOptFrameId = mDepthOptFrameId;
  mConfidenceFrameId = mDepthFrameId;
  mConfidenceOptFrameId = mDepthOptFrameId;

  // Print TF frames
  RCLCPP_INFO_STREAM(get_logger(), "*** TF FRAMES ***");
  RCLCPP_INFO_STREAM(get_logger(), " * Map\t\t\t-> " << mMapFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Odometry\t\t-> " << mOdomFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Base\t\t\t-> " << mBaseFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Camera\t\t\t-> " << mCameraFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Left\t\t\t-> " << mLeftCamFrameId);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Left Optical\t\t-> " << mLeftCamOptFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * RGB\t\t\t-> " << mRgbFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * RGB Optical\t\t-> " << mRgbOptFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Right\t\t\t-> " << mRightCamFrameId);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Right Optical\t\t-> " << mRightCamOptFrameId);
  if (!mDepthDisabled) {
    RCLCPP_INFO_STREAM(get_logger(), " * Depth\t\t\t-> " << mDepthFrameId);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Depth Optical\t\t-> " << mDepthOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Point Cloud\t\t-> " << mCloudFrameId);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Disparity\t\t-> " << mDisparityFrameId);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Disparity Optical\t-> " << mDisparityOptFrameId);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Confidence\t\t-> " << mConfidenceFrameId);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Confidence Optical\t-> " << mConfidenceOptFrameId);
  }

  if (!sl_tools::isZED(mCamRealModel)) {
    RCLCPP_INFO_STREAM(get_logger(), " * IMU\t\t\t-> " << mImuFrameId);

    if (sl_tools::isZED2OrZED2i(mCamUserModel)) {
      RCLCPP_INFO_STREAM(get_logger(), " * Barometer\t\t-> " << mBaroFrameId);
      RCLCPP_INFO_STREAM(get_logger(), " * Magnetometer\t\t-> " << mMagFrameId);
      RCLCPP_INFO_STREAM(
        get_logger(),
        " * Left Temperature\t-> " << mTempLeftFrameId);
      RCLCPP_INFO_STREAM(
        get_logger(),
        " * Right Temperature\t-> " << mTempRightFrameId);
    }
  }
  // <---- Coordinate frames
}

void ZedCamera::fillCamInfo(
  const std::shared_ptr<sl::Camera> zed,
  const std::shared_ptr<sensor_msgs::msg::CameraInfo> & leftCamInfoMsg,
  const std::shared_ptr<sensor_msgs::msg::CameraInfo> & rightCamInfoMsg,
  const std::string & leftFrameId, const std::string & rightFrameId,
  bool rawParam /*= false*/)
{
  sl::CalibrationParameters zedParam;

  if (rawParam) {
    zedParam = zed->getCameraInformation(mMatResol)
      .camera_configuration.calibration_parameters_raw;
  } else {
    zedParam = zed->getCameraInformation(mMatResol)
      .camera_configuration.calibration_parameters;
  }

  float baseline = zedParam.getCameraBaseline();

  // ----> Distortion models
  // ZED SDK params order: [ k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]
  // Radial (k1, k2, k3, k4, k5, k6), Tangential (p1,p2) and Prism (s1, s2, s3,
  // s4) distortion. Prism not currently used.

  // ROS2 order (OpenCV) -> k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
  switch (mCamRealModel) {
    case sl::MODEL::ZED:  // PLUMB_BOB
      leftCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::PLUMB_BOB;
      rightCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::PLUMB_BOB;
      leftCamInfoMsg->d.resize(5);
      rightCamInfoMsg->d.resize(5);
      leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
      leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
      leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
      leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
      leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
      rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
      rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
      rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
      rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
      rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
      break;

    case sl::MODEL::ZED2:    // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED2i:   // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED_X:   // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED_XM:  // RATIONAL_POLYNOMIAL
    case sl::MODEL::VIRTUAL_ZED_X:  // RATIONAL_POLYNOMIAL
      leftCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
      rightCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
      leftCamInfoMsg->d.resize(8);
      rightCamInfoMsg->d.resize(8);
      leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
      leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
      leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
      leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
      leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
      leftCamInfoMsg->d[5] = zedParam.left_cam.disto[5];    // k4
      leftCamInfoMsg->d[6] = zedParam.left_cam.disto[6];    // k5
      leftCamInfoMsg->d[7] = zedParam.left_cam.disto[7];    // k6
      rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
      rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
      rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
      rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
      rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
      rightCamInfoMsg->d[5] = zedParam.right_cam.disto[5];  // k4
      rightCamInfoMsg->d[6] = zedParam.right_cam.disto[6];  // k5
      rightCamInfoMsg->d[7] = zedParam.right_cam.disto[7];  // k6
      break;

    case sl::MODEL::ZED_M:
      if (zedParam.left_cam.disto[5] != 0 &&   // k4!=0
        zedParam.right_cam.disto[2] == 0 &&    // p1==0
        zedParam.right_cam.disto[3] == 0)      // p2==0
      {
        leftCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::EQUIDISTANT;
        rightCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::EQUIDISTANT;

        leftCamInfoMsg->d.resize(4);
        rightCamInfoMsg->d.resize(4);
        leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
        leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
        leftCamInfoMsg->d[2] = zedParam.left_cam.disto[4];    // k3
        leftCamInfoMsg->d[3] = zedParam.left_cam.disto[5];    // k4
        rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
        rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
        rightCamInfoMsg->d[2] = zedParam.right_cam.disto[4];  // k3
        rightCamInfoMsg->d[3] = zedParam.right_cam.disto[5];  // k4
      } else {
        leftCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::PLUMB_BOB;
        rightCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::PLUMB_BOB;
        leftCamInfoMsg->d.resize(5);
        rightCamInfoMsg->d.resize(5);
        leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
        leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
        leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
        leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
        leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
        rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
        rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
        rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
        rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
        rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
      }
  }

  leftCamInfoMsg->k.fill(0.0);
  rightCamInfoMsg->k.fill(0.0);
  leftCamInfoMsg->k[0] = static_cast<double>(zedParam.left_cam.fx);
  leftCamInfoMsg->k[2] = static_cast<double>(zedParam.left_cam.cx);
  leftCamInfoMsg->k[4] = static_cast<double>(zedParam.left_cam.fy);
  leftCamInfoMsg->k[5] = static_cast<double>(zedParam.left_cam.cy);
  leftCamInfoMsg->k[8] = 1.0;
  rightCamInfoMsg->k[0] = static_cast<double>(zedParam.right_cam.fx);
  rightCamInfoMsg->k[2] = static_cast<double>(zedParam.right_cam.cx);
  rightCamInfoMsg->k[4] = static_cast<double>(zedParam.right_cam.fy);
  rightCamInfoMsg->k[5] = static_cast<double>(zedParam.right_cam.cy);
  rightCamInfoMsg->k[8] = 1.0;
  leftCamInfoMsg->r.fill(0.0);
  rightCamInfoMsg->r.fill(0.0);

  for (size_t i = 0; i < 3; i++) {
    // identity
    rightCamInfoMsg->r[i + i * 3] = 1;
    leftCamInfoMsg->r[i + i * 3] = 1;
  }

  if (rawParam) {
    // ROS frame (X forward, Z up, Y left)
    for (int i = 0; i < 9; i++) {
      rightCamInfoMsg->r[i] =
        zedParam.stereo_transform.getRotationMatrix().r[i];
    }
  }

  leftCamInfoMsg->p.fill(0.0);
  rightCamInfoMsg->p.fill(0.0);
  leftCamInfoMsg->p[0] = static_cast<double>(zedParam.left_cam.fx);
  leftCamInfoMsg->p[2] = static_cast<double>(zedParam.left_cam.cx);
  leftCamInfoMsg->p[5] = static_cast<double>(zedParam.left_cam.fy);
  leftCamInfoMsg->p[6] = static_cast<double>(zedParam.left_cam.cy);
  leftCamInfoMsg->p[10] = 1.0;
  // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  rightCamInfoMsg->p[3] =
    static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
  rightCamInfoMsg->p[0] = static_cast<double>(zedParam.right_cam.fx);
  rightCamInfoMsg->p[2] = static_cast<double>(zedParam.right_cam.cx);
  rightCamInfoMsg->p[5] = static_cast<double>(zedParam.right_cam.fy);
  rightCamInfoMsg->p[6] = static_cast<double>(zedParam.right_cam.cy);
  rightCamInfoMsg->p[10] = 1.0;
  leftCamInfoMsg->width = rightCamInfoMsg->width =
    static_cast<uint32_t>(mMatResol.width);
  leftCamInfoMsg->height = rightCamInfoMsg->height =
    static_cast<uint32_t>(mMatResol.height);
  leftCamInfoMsg->header.frame_id = leftFrameId;
  rightCamInfoMsg->header.frame_id = rightFrameId;
}

void ZedCamera::initPublishers()
{
  RCLCPP_INFO(get_logger(), "*** PUBLISHED TOPICS ***");

  // ----> Topics names definition
  std::string rgbTopicRoot = "rgb";
  std::string rightTopicRoot = "right";
  std::string leftTopicRoot = "left";
  std::string stereoTopicRoot = "stereo";
  std::string img_topic = "/image_rect_color";
  std::string img_raw_topic = "/image_raw_color";
  std::string img_gray_topic = "_gray/image_rect_gray";
  std::string img_raw_gray_topic_ = "_gray/image_raw_gray";
  std::string raw_suffix = "_raw";
  std::string left_topic = mTopicRoot + leftTopicRoot + img_topic;
  std::string left_raw_topic =
    mTopicRoot + leftTopicRoot + raw_suffix + img_raw_topic;
  std::string right_topic = mTopicRoot + rightTopicRoot + img_topic;
  std::string right_raw_topic =
    mTopicRoot + rightTopicRoot + raw_suffix + img_raw_topic;
  std::string rgb_topic = mTopicRoot + rgbTopicRoot + img_topic;
  std::string rgb_raw_topic =
    mTopicRoot + rgbTopicRoot + raw_suffix + img_raw_topic;
  std::string stereo_topic = mTopicRoot + stereoTopicRoot + img_topic;
  std::string stereo_raw_topic =
    mTopicRoot + stereoTopicRoot + raw_suffix + img_raw_topic;
  std::string left_gray_topic = mTopicRoot + leftTopicRoot + img_gray_topic;
  std::string left_raw_gray_topic =
    mTopicRoot + leftTopicRoot + raw_suffix + img_raw_gray_topic_;
  std::string right_gray_topic = mTopicRoot + rightTopicRoot + img_gray_topic;
  std::string right_raw_gray_topic =
    mTopicRoot + rightTopicRoot + raw_suffix + img_raw_gray_topic_;
  std::string rgb_gray_topic = mTopicRoot + rgbTopicRoot + img_gray_topic;
  std::string rgb_raw_gray_topic =
    mTopicRoot + rgbTopicRoot + raw_suffix + img_raw_gray_topic_;

  // Set the disparity topic name
  std::string disparity_topic = mTopicRoot + "disparity/disparity_image";

  // Set the depth topic names
  std::string depth_topic_root = "depth";

  if (mOpenniDepthMode) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Openni depth mode activated -> Units: mm, Encoding: MONO16");
  }
  std::string depth_topic = mTopicRoot + depth_topic_root + "/depth_registered";
  std::string depth_info_topic = mTopicRoot + depth_topic_root + "/depth_info";

  mRoiMaskTopic = mTopicRoot + "roi_mask";

  std::string pointcloud_topic = mTopicRoot + "point_cloud/cloud_registered";
  mPointcloudFusedTopic = mTopicRoot + "mapping/fused_cloud";

  std::string object_det_topic_root = "obj_det";
  mObjectDetTopic = mTopicRoot + object_det_topic_root + "/objects";

  std::string body_trk_topic_root = "body_trk";
  mBodyTrkTopic = mTopicRoot + body_trk_topic_root + "/skeletons";

  std::string confImgRoot = "confidence";
  std::string conf_map_topic_name = "confidence_map";
  std::string conf_map_topic =
    mTopicRoot + confImgRoot + "/" + conf_map_topic_name;

  // Set the positional tracking topic names
  mPoseTopic = mTopicRoot + "pose";
  mPoseStatusTopic = mPoseTopic + "/status";
  mPoseCovTopic = mPoseTopic + "_with_covariance";
  mGnssPoseTopic = mPoseTopic + "/filtered";
  mGnssPoseStatusTopic = mGnssPoseTopic + "/status";
  mGeoPoseTopic = mTopicRoot + "geo_pose";
  mGeoPoseStatusTopic = mGeoPoseTopic + "/status";
  mFusedFixTopic = mPoseTopic + "/fused_fix";
  mOriginFixTopic = mPoseTopic + "/origin_fix";

  mOdomTopic = mTopicRoot + "odom";
  mOdomPathTopic = mTopicRoot + "path_odom";
  mPosePathTopic = mTopicRoot + "path_map";

  // Set the Sensors topic names
  std::string temp_topic_root = "temperature";
  std::string imuTopicRoot = "imu";
  std::string imu_topic_name = "data";
  std::string imu_topic_raw_name = "data_raw";
  std::string imu_topic_mag_name = "mag";
  // std::string imu_topic_mag_raw_name = "mag_raw";
  std::string pressure_topic_name = "atm_press";

  std::string imu_topic = mTopicRoot + imuTopicRoot + "/" + imu_topic_name;
  std::string imu_topic_raw =
    mTopicRoot + imuTopicRoot + "/" + imu_topic_raw_name;
  std::string imu_temp_topic =
    mTopicRoot + temp_topic_root + "/" + imuTopicRoot;
  std::string imu_mag_topic =
    mTopicRoot + imuTopicRoot + "/" + imu_topic_mag_name;
  // std::string imu_mag_topic_raw = imuTopicRoot + "/" +
  // imu_topic_mag_raw_name;
  std::string pressure_topic =
    mTopicRoot + /*imuTopicRoot + "/" +*/ pressure_topic_name;
  std::string temp_topic_left = mTopicRoot + temp_topic_root + "/left";
  std::string temp_topic_right = mTopicRoot + temp_topic_root + "/right";
  // <---- Topics names definition

  // ----> Camera publishers
  mPubRgb = image_transport::create_camera_publisher(
    this, rgb_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRgb.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRgb.getInfoTopic());
  mPubRgbGray = image_transport::create_camera_publisher(
    this, rgb_gray_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRgbGray.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRgbGray.getInfoTopic());
  mPubRawRgb = image_transport::create_camera_publisher(
    this, rgb_raw_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawRgb.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawRgb.getInfoTopic());
  mPubRawRgbGray = image_transport::create_camera_publisher(
    this, rgb_raw_gray_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawRgbGray.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawRgbGray.getInfoTopic());
  mPubLeft = image_transport::create_camera_publisher(
    this, left_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubLeft.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubLeft.getInfoTopic());
  mPubLeftGray = image_transport::create_camera_publisher(
    this, left_gray_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubLeftGray.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubLeftGray.getInfoTopic());
  mPubRawLeft = image_transport::create_camera_publisher(
    this, left_raw_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawLeft.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawLeft.getInfoTopic());
  mPubRawLeftGray = image_transport::create_camera_publisher(
    this, left_raw_gray_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawLeftGray.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawLeftGray.getInfoTopic());
  mPubRight = image_transport::create_camera_publisher(
    this, right_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRight.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRight.getInfoTopic());
  mPubRightGray = image_transport::create_camera_publisher(
    this, right_gray_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRightGray.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRightGray.getInfoTopic());
  mPubRawRight = image_transport::create_camera_publisher(
    this, right_raw_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawRight.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawRight.getInfoTopic());
  mPubRawRightGray = image_transport::create_camera_publisher(
    this, right_raw_gray_topic, mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawRightGray.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(), "Advertised on topic: " << mPubRawRightGray.getInfoTopic());

  if (!mDepthDisabled) {
    mPubDepth = image_transport::create_camera_publisher(
      this, depth_topic, mQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubDepth.getTopic());
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubDepth.getInfoTopic());
    mPubDepthInfo = create_publisher<zed_interfaces::msg::DepthInfoStamped>(
      depth_info_topic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(), "Advertised on topic: "
        << mPubDepthInfo->get_topic_name());

    if (mAutoRoiEnabled || mManualRoiEnabled) {
      mPubRoiMask = image_transport::create_camera_publisher(
        this, mRoiMaskTopic, mQos.get_rmw_qos_profile());
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Advertised on topic: " << mPubRoiMask.getTopic());
    }
  }

  mPubStereo = image_transport::create_publisher(
    this, stereo_topic,
    mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubStereo.getTopic());
  mPubRawStereo = image_transport::create_publisher(
    this, stereo_raw_topic,
    mQos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubRawStereo.getTopic());
  // <---- Camera publishers

  if (!mDepthDisabled) {
    // ----> Depth publishers
    mPubConfMap =
      create_publisher<sensor_msgs::msg::Image>(conf_map_topic, mQos);
    RCLCPP_INFO_STREAM(
      get_logger(), "Advertised on topic: " << mPubConfMap->get_topic_name());
    mPubDisparity = create_publisher<stereo_msgs::msg::DisparityImage>(
      disparity_topic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(), "Advertised on topic: "
        << mPubDisparity->get_topic_name());
#ifndef FOUND_FOXY
    mPubCloud = point_cloud_transport::create_publisher(
      this->shared_from_this(),
      pointcloud_topic, mQos.get_rmw_qos_profile(), mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubCloud.getTopic());
#else
    mPubCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubCloud->get_topic_name());
#endif
    // <---- Depth publishers

    // ----> Pos Tracking
    mPubPose = create_publisher<geometry_msgs::msg::PoseStamped>(
      mPoseTopic,
      mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubPose->get_topic_name());
    mPubPoseStatus = create_publisher<zed_interfaces::msg::PosTrackStatus>(
      mPoseStatusTopic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(), "Advertised on topic: "
        << mPubPoseStatus->get_topic_name());
    mPubPoseCov =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      mPoseCovTopic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(), "Advertised on topic: " << mPubPoseCov->get_topic_name());
    mPubOdom =
      create_publisher<nav_msgs::msg::Odometry>(mOdomTopic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubOdom->get_topic_name());
    mPubPosePath =
      create_publisher<nav_msgs::msg::Path>(mPosePathTopic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(), "Advertised on topic: "
        << mPubPosePath->get_topic_name());
    mPubOdomPath =
      create_publisher<nav_msgs::msg::Path>(mOdomPathTopic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(), "Advertised on topic: "
        << mPubOdomPath->get_topic_name());
    if (mGnssFusionEnabled) {
      mPubGnssPose = create_publisher<nav_msgs::msg::Odometry>(
        mGnssPoseTopic,
        mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic (GNSS): "
          << mPubGnssPose->get_topic_name());
      mPubGnssPoseStatus =
        create_publisher<zed_interfaces::msg::GnssFusionStatus>(
        mGnssPoseStatusTopic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Advertised on topic: " << mPubGnssPoseStatus->get_topic_name());
      mPubGeoPose = create_publisher<geographic_msgs::msg::GeoPoseStamped>(
        mGeoPoseTopic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic (GNSS): "
          << mPubGeoPose->get_topic_name());
      mPubGeoPoseStatus =
        create_publisher<zed_interfaces::msg::GnssFusionStatus>(
        mGeoPoseStatusTopic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Advertised on topic (GNSS): "
          << mPubGeoPoseStatus->get_topic_name());
      mPubFusedFix = create_publisher<sensor_msgs::msg::NavSatFix>(
        mFusedFixTopic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic (GNSS): "
          << mPubFusedFix->get_topic_name());

      mPubOriginFix = create_publisher<sensor_msgs::msg::NavSatFix>(
        mOriginFixTopic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic (GNSS origin): "
          << mPubOriginFix->get_topic_name());
    }
    // <---- Pos Tracking

    // ----> Mapping
    if (mMappingEnabled) {
#ifndef FOUND_FOXY
      mPubFusedCloud = point_cloud_transport::create_publisher(
        this->shared_from_this(), mPointcloudFusedTopic, mQos.get_rmw_qos_profile(), mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic "
          << mPubFusedCloud.getTopic()
          << " @ " << mFusedPcPubRate
          << " Hz");
#else
      mPubFusedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
        mPointcloudFusedTopic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic "
          << mPubFusedCloud->get_topic_name()
          << " @ " << mFusedPcPubRate
          << " Hz");
#endif
    }

    std::string marker_topic = "plane_marker";
    std::string plane_topic = "plane";
    // Rviz markers publisher
    mPubMarker = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubMarker->get_topic_name());
    // Detected planes publisher
    mPubPlane = create_publisher<zed_interfaces::msg::PlaneStamped>(
      plane_topic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubPlane->get_topic_name());
    // <---- Mapping
  }

  // ----> Sensors
  if (!sl_tools::isZED(mCamRealModel)) {
    mPubImu = create_publisher<sensor_msgs::msg::Imu>(imu_topic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubImu->get_topic_name());
    mPubImuRaw =
      create_publisher<sensor_msgs::msg::Imu>(imu_topic_raw, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubImuRaw->get_topic_name());

    if (sl_tools::isZED2OrZED2i(mCamRealModel) ||
      sl_tools::isZEDX(mCamRealModel))
    {
      mPubImuTemp = create_publisher<sensor_msgs::msg::Temperature>(
        imu_temp_topic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic: "
          << mPubImuTemp->get_topic_name());
    }

    if (sl_tools::isZED2OrZED2i(mCamRealModel)) {
      mPubImuMag = create_publisher<sensor_msgs::msg::MagneticField>(
        imu_mag_topic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic: "
          << mPubImuMag->get_topic_name());
      mPubPressure = create_publisher<sensor_msgs::msg::FluidPressure>(
        pressure_topic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic: "
          << mPubPressure->get_topic_name());
      mPubTempL = create_publisher<sensor_msgs::msg::Temperature>(
        temp_topic_left, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic: " << mPubTempL->get_topic_name());
      mPubTempR = create_publisher<sensor_msgs::msg::Temperature>(
        temp_topic_right, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic: " << mPubTempR->get_topic_name());
    }

    // ----> Camera/imu transform message
    std::string cam_imu_tr_topic = mTopicRoot + "left_cam_imu_transform";
    mPubCamImuTransf = create_publisher<geometry_msgs::msg::TransformStamped>(
      cam_imu_tr_topic, mQos, mPubOpt);

    RCLCPP_INFO_STREAM(
      get_logger(), "Advertised on topic: "
        << mPubCamImuTransf->get_topic_name());

    sl::Orientation sl_rot = mSlCamImuTransf.getOrientation();
    sl::Translation sl_tr = mSlCamImuTransf.getTranslation();
    RCLCPP_INFO(
      get_logger(), "Camera-IMU Translation: \n %g %g %g", sl_tr.x,
      sl_tr.y, sl_tr.z);
    RCLCPP_INFO(
      get_logger(), "Camera-IMU Rotation:\n%s",
      sl_rot.getRotationMatrix().getInfos().c_str());

    // publishImuFrameAndTopic();
    // <---- Camera/imu transform message
  }
  // <---- Sensors
}

void ZedCamera::initSubscribers()
{
  RCLCPP_INFO(get_logger(), "*** Subscribers ***");
  // ----> Clicked Point Subscriber
  /* From `$ ros2 topic info /clicked_point -v`
      QoS profile:
          Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
          Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
          Lifespan: 2147483651294967295 nanoseconds
          Deadline: 2147483651294967295 nanoseconds
          Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
          Liveliness lease duration: 2147483651294967295 nanoseconds
  */
  if (!mDepthDisabled) {
    mClickedPtSub = create_subscription<geometry_msgs::msg::PointStamped>(
      mClickedPtTopic, mQos,
      std::bind(&ZedCamera::callback_clickedPoint, this, _1), mSubOpt);

    RCLCPP_INFO_STREAM(
      get_logger(), " * Plane detection: '"
        << mClickedPtSub->get_topic_name()
        << "'");
  }
  // <---- Clicked Point Subscriber

  // ----> GNSS Fix Subscriber
  /* From `$ ros2 topic info /fix -v`
    QoS profile:
        Reliability: RELIABLE
        History (Depth): KEEP_LAST (10)
        Durability: VOLATILE
        Lifespan: Infinite
        Deadline: Infinite
        Liveliness: AUTOMATIC
        Liveliness lease duration: Infinite
  */
  if (mGnssFusionEnabled && !mSvoMode) {
    mGnssMsgReceived = false;
    mGnssFixValid = false;

    mGnssFixSub = create_subscription<sensor_msgs::msg::NavSatFix>(
      mGnssTopic, mQos, std::bind(&ZedCamera::callback_gnssFix, this, _1),
      mSubOpt);

    RCLCPP_INFO_STREAM(
      get_logger(), " * GNSS Fix: '" << mGnssFixSub->get_topic_name() << "'");
  }
  // <---- GNSS Fix Subscriber

  // ----> Clock Subscriber
  /* From `$ ros2 topic info /clock -v`

    QoS profile:
      Reliability: RELIABLE
      History (Depth): KEEP_LAST (10)
      Durability: VOLATILE
      Lifespan: Infinite
      Deadline: Infinite
      Liveliness: AUTOMATIC
      Liveliness lease duration: Infinite
  */

  if (mUseSimTime) {
    mClockAvailable = false;

    mClockSub = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", mQos, std::bind(&ZedCamera::callback_clock, this, _1),
      mSubOpt);

    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Sim Clock: '" << mClockSub->get_topic_name() << "'");
  }
}

bool ZedCamera::startCamera()
{
  RCLCPP_INFO(get_logger(), "***** STARTING CAMERA *****");

  // Create a ZED object
  mZed = std::make_shared<sl::Camera>();

  // ----> SDK version
  RCLCPP_INFO(
    get_logger(), "ZED SDK Version: %d.%d.%d - Build %s",
    ZED_SDK_MAJOR_VERSION, ZED_SDK_MINOR_VERSION,
    ZED_SDK_PATCH_VERSION, ZED_SDK_BUILD_ID);
  // <---- SDK version

  // ----> TF2 Transform
  mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  mTfListener = std::make_unique<tf2_ros::TransformListener>(
    *mTfBuffer);    // Start TF Listener thread
  mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  // <---- TF2 Transform

  // ----> ZED configuration
  if (mSimMode) {  // Simulation?
    RCLCPP_INFO_STREAM(
      get_logger(), "*** CONNECTING TO THE SIMULATION SERVER ["
        << mSimAddr.c_str() << ":" << mSimPort
        << "] ***");

    mInitParams.input.setFromStream(mSimAddr.c_str(), mSimPort);
  } else if (!mSvoFilepath.empty()) {
    RCLCPP_INFO(get_logger(), "*** SVO OPENING ***");

    mInitParams.input.setFromSVOFile(mSvoFilepath.c_str());
    mInitParams.svo_real_time_mode = mSvoRealtime;
  } else if (!mStreamAddr.empty()) {
    RCLCPP_INFO(get_logger(), "*** LOCAL STREAMING OPENING ***");

    mInitParams.input.setFromStream(mStreamAddr.c_str(), static_cast<unsigned short>(mStreamPort));
  } else {
    RCLCPP_INFO(get_logger(), "*** CAMERA OPENING ***");

    mInitParams.camera_fps = mCamGrabFrameRate;
    mInitParams.grab_compute_capping_fps = static_cast<float>(mPubFrameRate);
    mInitParams.camera_resolution = static_cast<sl::RESOLUTION>(mCamResol);

    if (mCamSerialNumber > 0) {
      mInitParams.input.setFromSerialNumber(mCamSerialNumber);
    }
  }

  mInitParams.coordinate_system = ROS_COORDINATE_SYSTEM;
  mInitParams.coordinate_units = ROS_MEAS_UNITS;
  mInitParams.depth_mode = mDepthMode;
  mInitParams.sdk_verbose = mVerbose;
  mInitParams.sdk_gpu_id = mGpuId;
  mInitParams.depth_stabilization = mDepthStabilization;
  mInitParams.camera_image_flip = mCameraFlip;
  mInitParams.depth_minimum_distance = mCamMinDepth;
  mInitParams.depth_maximum_distance = mCamMaxDepth;

  if (!mOpencvCalibFile.empty()) {
    mInitParams.optional_opencv_calibration_file = mOpencvCalibFile.c_str();
  }

  mInitParams.camera_disable_self_calib = !mCameraSelfCalib;
  mInitParams.enable_image_enhancement = true;
  mInitParams.enable_right_side_measure = false;

  mInitParams.async_grab_camera_recovery =
    true;    // Camera recovery is handled asynchronously to provide information
             // about this status

  // NOTE: this is a temp fix to GMSL2 camera close issues
  // TODO: check if this issue has been fixed in the SDK
  if (sl_tools::isZEDX(mCamUserModel)) {
    RCLCPP_INFO(get_logger(), "Disable async recovery for GMSL2 cameras");
    mInitParams.async_grab_camera_recovery = false;
  }
  // <---- ZED configuration

  // ----> Try to connect to a camera, to a stream, or to load an SVO
  sl_tools::StopWatch connectTimer(get_clock());

  mThreadStop = false;
  mGrabStatus = sl::ERROR_CODE::LAST;

  if (!mSvoMode && !mSimMode && !mStreamMode) {
    if (mCamSerialNumber > 0) {
      mInitParams.input.setFromSerialNumber(mCamSerialNumber);
    }
  }

  while (1) {
    rclcpp::sleep_for(500ms);

    mConnStatus = mZed->open(mInitParams);

    if (mConnStatus == sl::ERROR_CODE::SUCCESS) {
      DEBUG_STREAM_COMM("Opening successfull");
      break;
    }

    if (mConnStatus == sl::ERROR_CODE::INVALID_CALIBRATION_FILE) {
      if (mOpencvCalibFile.empty()) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Calibration file error: "
            << sl::toVerbose(mConnStatus));
      } else {
        RCLCPP_ERROR_STREAM(
          get_logger(),
          "If you are using a custom OpenCV calibration file, please check "
          "the correctness of the path of the calibration file "
          "in the parameter 'general.optional_opencv_calibration_file': '"
            << mOpencvCalibFile << "'.");
        RCLCPP_ERROR(
          get_logger(),
          "If the file exists, it may contain invalid information.");
      }
      return false;
    }

    if (mSvoMode) {
      RCLCPP_WARN(
        get_logger(), "Error opening SVO: %s",
        sl::toString(mConnStatus).c_str());
      return false;
    } else if (mSimMode) {
      RCLCPP_WARN(
        get_logger(), "Error connecting to the simulation server: %s",
        sl::toString(mConnStatus).c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "Error opening camera: %s",
        sl::toString(mConnStatus).c_str());
      if (mConnStatus == sl::ERROR_CODE::CAMERA_DETECTION_ISSUE &&
        sl_tools::isZEDM(mCamUserModel))
      {
        RCLCPP_INFO(
          get_logger(),
          "Try to flip the USB3 Type-C connector and verify the USB3 "
          "connection");
      } else {
        RCLCPP_INFO(get_logger(), "Please verify the camera connection");
      }
    }

    if (!rclcpp::ok() || mThreadStop) {
      RCLCPP_INFO(get_logger(), "ZED activation interrupted by user.");
      return false;
    }

    if (connectTimer.toc() > mMaxReconnectTemp * mCamTimeoutSec) {
      RCLCPP_ERROR(get_logger(), "Camera detection timeout");
      return false;
    }

    rclcpp::sleep_for(std::chrono::seconds(mCamTimeoutSec));
  }
  // ----> Try to connect to a camera, to a stream, or to load an SVO


  // ----> If SVO and GNSS enabled check that it's a valid SV0 Gen.2
  if (mSvoMode && mGnssFusionEnabled) {
    // TODO(Walter) Check SVO version when it's available

    mGnssReplay = std::make_unique<sl_tools::GNSSReplay>(mZed);
    if (!mGnssReplay->initialize()) {
      RCLCPP_ERROR(get_logger(), "The SVO file does not contain valid GNSS information.");
      return false;
    } else {
      RCLCPP_INFO(
        get_logger(),
        "GNSS information will be retrieved from the SVO file.");
    }
  }
  // <---- If SVO and GNSS enabled check that it's a valid SV0 Gen.2

  // ----> If SVO and positional tracking Gen2 check that it's a valid SV0 Gen2
  if (mSvoMode && mPosTrackingEnabled &&
    mPosTrkMode == sl::POSITIONAL_TRACKING_MODE::GEN_2)
  {
    // TODO(Walter) Check SVO version when it's available
  }
  // <---- If SVO and positional tracking Gen2 check that it's a valid SV0 Gen2

  // ----> Camera information
  sl::CameraInformation camInfo = mZed->getCameraInformation();

  float realFps = camInfo.camera_configuration.fps;
  if (realFps != static_cast<float>(mCamGrabFrameRate)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "!!! `general.grab_frame_rate` value is not valid: '"
        << mCamGrabFrameRate
        << "'. Automatically replaced with '" << realFps
        << "'. Please fix the parameter !!!");
    mCamGrabFrameRate = realFps;
  }

  // CUdevice devid;
  cuCtxGetDevice(&mGpuId);
  RCLCPP_INFO_STREAM(get_logger(), "ZED SDK running on GPU #" << mGpuId);

  // Camera model
  mCamRealModel = camInfo.camera_model;

  if (mCamRealModel == sl::MODEL::ZED) {
    if (mCamUserModel != sl::MODEL::ZED) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zed'");
    }
  } else if (mCamRealModel == sl::MODEL::ZED_M) {
    if (mCamUserModel != sl::MODEL::ZED_M) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zedm'");
    }
  } else if (mCamRealModel == sl::MODEL::ZED2) {
    if (mCamUserModel != sl::MODEL::ZED2) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zed2'");
    }
  } else if (mCamRealModel == sl::MODEL::ZED2i) {
    if (mCamUserModel != sl::MODEL::ZED2i) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zed2i'");
    }
  } else if (mCamRealModel == sl::MODEL::ZED_X) {
    if (mCamUserModel != sl::MODEL::ZED_X) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zedx'");
    }
  } else if (mCamRealModel == sl::MODEL::ZED_XM) {
    if (mCamUserModel != sl::MODEL::ZED_XM) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zedxm'");
    }
  } else if (mCamRealModel == sl::MODEL::VIRTUAL_ZED_X) {
    if (mCamUserModel != sl::MODEL::VIRTUAL_ZED_X) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zedxm'");
    }
  }

  RCLCPP_INFO_STREAM(
    get_logger(), " * Camera Model  -> "
      << sl::toString(mCamRealModel).c_str());
  mCamSerialNumber = camInfo.serial_number;
  RCLCPP_INFO_STREAM(get_logger(), " * Serial Number -> " << mCamSerialNumber);

  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Focal Lenght -> "
      << camInfo.camera_configuration.calibration_parameters
      .left_cam.focal_length_metric
      << " mm");

  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Input\t -> "
      << sl::toString(mZed->getCameraInformation().input_type).c_str());
  if (mSvoMode) {
    RCLCPP_INFO(
      get_logger(), " * SVO resolution\t-> %ldx%ld",
      mZed->getCameraInformation().camera_configuration.resolution.width,
      mZed->getCameraInformation().camera_configuration.resolution.height);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * SVO framerate\t-> "
        << (mZed->getCameraInformation().camera_configuration.fps));
  }

  // Firmwares
  if (!mSvoMode) {
    mCamFwVersion = camInfo.camera_configuration.firmware_version;

    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Camera FW Version  -> " << mCamFwVersion);
    if (!sl_tools::isZED(mCamRealModel)) {
      mSensFwVersion = camInfo.sensors_configuration.firmware_version;
      RCLCPP_INFO_STREAM(
        get_logger(),
        " * Sensors FW Version -> " << mSensFwVersion);
    }
  }

  // Camera/IMU transform
  if (!sl_tools::isZED(mCamRealModel)) {
    mSlCamImuTransf = camInfo.sensors_configuration.camera_imu_transform;

    DEBUG_SENS("Camera-IMU Transform:\n%s", mSlCamImuTransf.getInfos().c_str());
  }

  mCamWidth = camInfo.camera_configuration.resolution.width;
  mCamHeight = camInfo.camera_configuration.resolution.height;

  RCLCPP_INFO_STREAM(
    get_logger(), " * Camera grab frame size -> "
      << mCamWidth << "x" << mCamHeight);

  int pub_w, pub_h;
  pub_w = static_cast<int>(std::round(mCamWidth / mCustomDownscaleFactor));
  pub_h = static_cast<int>(std::round(mCamHeight / mCustomDownscaleFactor));

  if (pub_w > mCamWidth || pub_h > mCamHeight) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The publishing resolution ("
        << pub_w << "x" << pub_h
        << ") cannot be higher than the grabbing resolution ("
        << mCamWidth << "x" << mCamHeight
        << "). Using grab resolution for output messages.");
    pub_w = mCamWidth;
    pub_h = mCamHeight;
  }

  mMatResol = sl::Resolution(pub_w, pub_h);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Publishing frame size  -> "
      << mMatResol.width << "x"
      << mMatResol.height);
  // <---- Camera information

  // ----> Set Region of Interest
  if (!mDepthDisabled) {
    if (mAutoRoiEnabled) {
      RCLCPP_INFO(get_logger(), "*** Enabling Automatic ROI ***");

      sl::RegionOfInterestParameters roi_param;
      roi_param.depth_far_threshold_meters = mRoiDepthFarThresh;
      roi_param.image_height_ratio_cutoff = mRoiImgHeightRationCutOff;
      roi_param.auto_apply_module = mRoiModules;

      sl::ERROR_CODE err = mZed->startRegionOfInterestAutoDetection(roi_param);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(),
          " * Error while starting automatic ROI generation: "
            << sl::toString(err).c_str());
      } else {
        RCLCPP_INFO(
          get_logger(),
          " * Automatic Region of Interest generation started.");
      }
    } else if (!mRoyPolyParam.empty()) {
      RCLCPP_INFO(get_logger(), "*** Setting Manual ROI ***");
      sl::Resolution resol(mCamWidth, mCamHeight);
      std::vector<sl::float2> sl_poly;

      DEBUG_ROI("Parse ROI Polygon parameter");
      std::string poly_str = parseRoiPoly(mRoyPolyParam, sl_poly);
      DEBUG_STREAM_ROI("Parsed ROI Polygon: " << poly_str);
      DEBUG_STREAM_ROI(" * Polygon size: " << sl_poly.size());

      DEBUG_ROI("Create ROI Mask mat");
      sl::Mat roi_mask(resol, sl::MAT_TYPE::U8_C1, sl::MEM::CPU);

      // Create ROI mask
      DEBUG_ROI("Generate ROI Mask");
      if (!sl_tools::generateROI(sl_poly, roi_mask)) {
        RCLCPP_WARN(
          get_logger(),
          " * Error generating the manual region of interest image mask.");
      } else {
        DEBUG_ROI("Enable ROI");
        sl::ERROR_CODE err = mZed->setRegionOfInterest(roi_mask, mRoiModules);
        DEBUG_ROI("ROI Enabled");
        if (err != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            " * Error while setting ZED SDK manual region of interest: "
              << sl::toString(err).c_str());
        } else {
          RCLCPP_INFO(
            get_logger(),
            " * Manual Region of Interest correctly set.");
          mManualRoiEnabled = true;
        }
      }
    }
  }
  // <---- Set Region of Interest

  // ----> Check default camera settings
  if (mDebugCamCtrl) {
    int value;
    sl::ERROR_CODE err;
    sl::VIDEO_SETTINGS setting;

    if (!sl_tools::isZEDX(mCamRealModel)) {
      setting = sl::VIDEO_SETTINGS::BRIGHTNESS;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "Default value for " << sl::toString(setting).c_str()
                             << ": " << value);

      setting = sl::VIDEO_SETTINGS::CONTRAST;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "Default value for " << sl::toString(setting).c_str()
                             << ": " << value);

      setting = sl::VIDEO_SETTINGS::HUE;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "Default value for " << sl::toString(setting).c_str()
                             << ": " << value);
    }

    setting = sl::VIDEO_SETTINGS::SATURATION;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::SHARPNESS;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::GAMMA;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::AEC_AGC;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::EXPOSURE;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::GAIN;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    if (sl_tools::isZEDX(mCamRealModel)) {
      setting = sl::VIDEO_SETTINGS::EXPOSURE_TIME;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "[ZEDX] Default value for "
          << sl::toString(setting).c_str() << ": " << value);

      // TODO(Walter) Enable when fixed in the SDK
      // setting = sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE;
      // err = mZed->getCameraSettings(setting, value_min, value_max);
      // if(err!=sl::ERROR_CODE::SUCCESS) {
      //   RCLCPP_ERROR_STREAM( get_logger(), "Error Getting default param for
      //   "
      //   << sl::toString(setting).c_str() << ": " <<
      //   sl::toString(err).c_str()); exit(EXIT_FAILURE);
      // }
      // DEBUG_STREAM_CTRL("[ZEDX] Default value for " <<
      // sl::toString(setting).c_str() << ": [" << value_min << "," <<
      // value_max
      // << "]");

      if (!mStreamMode) {
        setting = sl::VIDEO_SETTINGS::EXPOSURE_COMPENSATION;
        err = mZed->getCameraSettings(setting, value);
        if (err != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_ERROR_STREAM(
            get_logger(), "Error Getting default param for "
              << sl::toString(setting).c_str()
              << ": "
              << sl::toString(err).c_str());
          exit(EXIT_FAILURE);
        }
        DEBUG_STREAM_CTRL(
          "[ZEDX] Default value for "
            << sl::toString(setting).c_str() << ": " << value);
      }

      setting = sl::VIDEO_SETTINGS::ANALOG_GAIN;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "[ZEDX] Default value for "
          << sl::toString(setting).c_str() << ": " << value);

      // TODO(Walter) Enable when fixed in the SDK
      // setting = sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE;
      // err = mZed->getCameraSettings(setting, value_min, value_max);
      // if(err!=sl::ERROR_CODE::SUCCESS) {
      //   RCLCPP_ERROR_STREAM( get_logger(), "Error Getting default param for
      //   "
      //   << sl::toString(setting).c_str() << ": " <<
      //   sl::toString(err).c_str()); exit(EXIT_FAILURE);
      // }
      // DEBUG_STREAM_CTRL("[ZEDX] Default value for " <<
      // sl::toString(setting).c_str() << ": [" << value_min << "," <<
      // value_max
      // << "]");

      setting = sl::VIDEO_SETTINGS::DIGITAL_GAIN;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "[ZEDX] Default value for "
          << sl::toString(setting).c_str() << ": " << value);

      // TODO(Walter) Enable when fixed in the SDK
      // setting = sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE;
      // err = mZed->getCameraSettings(setting, value_min, value_max);
      // if(err!=sl::ERROR_CODE::SUCCESS) {
      //   RCLCPP_ERROR_STREAM( get_logger(), "Error Getting default param for
      //   "
      //   << sl::toString(setting).c_str() << ": " <<
      //   sl::toString(err).c_str()); exit(EXIT_FAILURE);
      // }
      // DEBUG_STREAM_CTRL("[ZEDX] Default value for " <<
      // sl::toString(setting).c_str() << ": [" << value_min << "," <<
      // value_max
      // << "]");

      if (!mStreamMode) {
        setting = sl::VIDEO_SETTINGS::DENOISING;
        err = mZed->getCameraSettings(setting, value);
        if (err != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_ERROR_STREAM(
            get_logger(), "Error Getting default param for "
              << sl::toString(setting).c_str()
              << ": "
              << sl::toString(err).c_str());
          exit(EXIT_FAILURE);
        }
        DEBUG_STREAM_CTRL(
          "[ZEDX] Default value for "
            << sl::toString(setting).c_str() << ": " << value);
      }
    }
  }
  // <----> Check default camera settings

  // ----> Camera Info messages
  mRgbCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mRgbCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mLeftCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mLeftCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mRightCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mRightCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mDepthCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();

  setTFCoordFrameNames();  // Requires mZedRealCamModel available only after
                           // camera opening

  fillCamInfo(
    mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId,
    mRightCamOptFrameId);
  fillCamInfo(
    mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId,
    mRightCamOptFrameId, true);
  mRgbCamInfoMsg = mLeftCamInfoMsg;
  mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
  mDepthCamInfoMsg = mLeftCamInfoMsg;
  // <---- Camera Info messages

  initPublishers();  // Requires mZedRealCamModel available only after camera
                     // opening
  initSubscribers();

  // Disable AEC_AGC and Auto Whitebalance to trigger it if user set it to
  // automatic
  if (!mSvoMode && !mSimMode) {
    mZed->setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
    mZed->setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0);
    // Force parameters with a dummy grab
    mZed->grab();
  }

  // Initialialized timestamp to avoid wrong initial data
  // ----> Timestamp
  if (mSvoMode) {
    mFrameTimestamp =
      sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::CURRENT));
  } else if (mSimMode) {
    if (mUseSimTime) {
      mFrameTimestamp = get_clock()->now();
    } else {
      mFrameTimestamp =
        sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
    }
  } else {
    mFrameTimestamp =
      sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
  }
  // <---- Timestamp

  // RCLCPP_INFO_STREAM(
  //   get_logger(),
  //   "Timestamp - CURRENT: "
  //     << mZed->getTimestamp(sl::TIME_REFERENCE::CURRENT).getNanoseconds());
  // RCLCPP_INFO_STREAM(
  //   get_logger(),
  //   "Timestamp - IMAGE: "
  //     << mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds());

  // ----> Initialize Diagnostic statistics
  mElabPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  mGrabPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  mVideoDepthPeriodMean_sec =
    std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  mVideoDepthElabMean_sec =
    std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  mPcPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  mPcProcMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  mObjDetPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  mObjDetElabMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  mBodyTrkPeriodMean_sec =
    std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  mBodyTrkElabMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  mImuPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
  mBaroPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
  mMagPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
  mPubFusedCloudPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mPcPubRate);
  mPubOdomTF_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
  mPubPoseTF_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
  mPubImuTF_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
  mGnssFix_sec = std::make_unique<sl_tools::WinAvg>(10);
  // <---- Initialize Diagnostic statistics

  if (mGnssFusionEnabled) {
    DEBUG_GNSS("Initialize Fusion module");

    // ----> Retrieve GNSS to ZED transform
    RCLCPP_INFO(get_logger(), "*** Initialize GNSS Offset ***");
    if (!mGnss2BaseTransfValid) {
      getGnss2BaseTransform();
    }

    mGnssAntennaPose[0] = mGnss2BaseTransf.getOrigin().x();
    mGnssAntennaPose[1] = mGnss2BaseTransf.getOrigin().y();
    mGnssAntennaPose[2] = mGnss2BaseTransf.getOrigin().z();
    // <---- Retrieve GNSS to ZED transform

    // ----> Initialize Fusion module

    // Fusion parameters
    mFusionInitParams.coordinate_system = ROS_COORDINATE_SYSTEM;
    mFusionInitParams.coordinate_units = ROS_MEAS_UNITS;
    mFusionInitParams.verbose = mVerbose != 0;
    mFusionInitParams.output_performance_metrics = true;
    mFusionInitParams.timeout_period_number =
      20;    // TODO(Walter) Evaluate this: mCamGrabFrameRate *
             // mCamTimeoutSec;

    // Fusion initialization
    sl::FUSION_ERROR_CODE fus_err = mFusion.init(mFusionInitParams);
    if (fus_err != sl::FUSION_ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error initializing the Fusion module: "
          << sl::toString(fus_err).c_str()
          << ".");
      exit(EXIT_FAILURE);
    }
    DEBUG_GNSS(" Fusion params OK");

    mFusionConfig = std::make_shared<sl::FusionConfiguration>();

    if (mSimMode) {
      // TODO(Walter) Modify when support for streaming input is added in the
      // SDK mFusionConfig->input_type.setFromStream(mSimAddr, mSimPort);
      mFusionConfig->input_type.setFromSerialNumber(mCamSerialNumber);
      mFusionConfig->communication_parameters.setForSharedMemory();
    } else if (mSvoMode) {
      mFusionConfig->input_type.setFromSVOFile(mSvoFilepath.c_str());
      mFusionConfig->communication_parameters.setForSharedMemory();
    } else {
      mFusionConfig->input_type.setFromSerialNumber(mCamSerialNumber);
      mFusionConfig->communication_parameters.setForSharedMemory();
    }
    mFusionConfig->serial_number = mCamSerialNumber;
    mFusionConfig->pose = sl::Transform::identity();

    DEBUG_GNSS(" Fusion communication params OK");

    // Camera identifier
    mCamUuid.sn = mCamSerialNumber;

    // Enable camera publishing to Fusion
    mZed->startPublishing(mFusionConfig->communication_parameters);
    DEBUG_GNSS(" Camera publishing OK");

    // Fusion subscribe to camera data
    fus_err = mFusion.subscribe(
      mCamUuid, mFusionConfig->communication_parameters, mFusionConfig->pose);
    if (fus_err != sl::FUSION_ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error initializing the Fusion module: "
          << sl::toString(fus_err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_GNSS(" Fusion subscribing OK");
    DEBUG_GNSS("Fusion module ready");
    // <---- Initialize Fusion module
  }

  // Init and start threads
  initThreads();

  return true;
}  // namespace stereolabs

void ZedCamera::initThreads()
{
  // ----> Start CMOS Temperatures thread
  if (!mSimMode && !sl_tools::isZED(mCamRealModel) &&
    !sl_tools::isZEDM(mCamRealModel))
  {
    startTempPubTimer();
  }
  // <---- Start CMOS Temperatures thread

  // ----> Start Sensors thread if not sync
  if (!mSimMode && !mSvoMode && !mSensCameraSync &&
    !sl_tools::isZED(mCamRealModel))
  {
    mSensThread = std::thread(&ZedCamera::threadFunc_pubSensorsData, this);
  }
  // <---- Start Sensors thread if not sync

  // ----> Start Pointcloud thread
  if (!mDepthDisabled) {
    mPcDataReady = false;
    // DEBUG_STREAM_PC( "on_activate -> mPcDataReady FALSE")
    mPcThread = std::thread(&ZedCamera::threadFunc_pointcloudElab, this);
  }
  // <---- Start Pointcloud thread

  // Start grab thread
  mGrabThread = std::thread(&ZedCamera::threadFunc_zedGrab, this);
}

void ZedCamera::startTempPubTimer()
{
  if (mTempPubTimer != nullptr) {
    mTempPubTimer->cancel();
  }

  std::chrono::milliseconds pubPeriod_msec(static_cast<int>(1000.0));
  mTempPubTimer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(pubPeriod_msec),
    std::bind(&ZedCamera::callback_pubTemp, this));
}

void ZedCamera::startFusedPcTimer(double fusedPcRate)
{
  if (mFusedPcTimer != nullptr) {
    mFusedPcTimer->cancel();
  }

  std::chrono::milliseconds pubPeriod_msec(
    static_cast<int>(1000.0 / (fusedPcRate)));
  mFusedPcTimer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(pubPeriod_msec),
    std::bind(&ZedCamera::callback_pubFusedPc, this));
}

void ZedCamera::startPathPubTimer(double pathTimerRate)
{
  if (mPathTimer != nullptr) {
    mPathTimer->cancel();
  }

  DEBUG_PT("Starting path pub. timer");

  if (pathTimerRate > 0) {
    std::chrono::milliseconds pubPeriod_msec(
      static_cast<int>(1000.0 / (pathTimerRate)));
    mPathTimer = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(pubPeriod_msec),
      std::bind(&ZedCamera::callback_pubPaths, this));

    if (mOdomPath.size() == 0 && mPosePath.size() == 0) {
      if (mPathMaxCount != -1) {
        DEBUG_STREAM_PT("Path vectors reserved " << mPathMaxCount << " poses.");
        mOdomPath.reserve(mPathMaxCount);
        mPosePath.reserve(mPathMaxCount);

        DEBUG_STREAM_PT(
          "Path vector sizes: " << mOdomPath.size() << " "
                                << mPosePath.size());
      }
    }
  } else {
    mOdomPath.clear();
    mPosePath.clear();
    mPathTimer->cancel();
    RCLCPP_INFO_STREAM(
      get_logger(), "Path topics not published -> Pub. rate: "
        << pathTimerRate << " Hz");
  }
}

bool ZedCamera::startPosTracking()
{
  if (mDepthDisabled) {
    RCLCPP_WARN(
      get_logger(),
      "Cannot start Positional Tracking if "
      "`depth.depth_mode` is set to `0` [NONE]");
    return false;
  }

  RCLCPP_INFO(get_logger(), "*** Starting Positional Tracking ***");

  RCLCPP_INFO(get_logger(), " * Waiting for valid static transformations...");

  bool transformOk = false;
  double elapsed = 0.0;
  mPosTrackingReady = false;
  mGnssInitGood = false;

  // auto start = std::chrono::high_resolution_clock::now();

  sl_tools::StopWatch stopWatch(get_clock());

  do {
    transformOk =
      setPose(
      mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2],
      mInitialBasePose[3], mInitialBasePose[4], mInitialBasePose[5]);

    // elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    //   std::chrono::high_resolution_clock::now() - start)
    //   .count();
    elapsed = stopWatch.toc();

    rclcpp::sleep_for(1ms);

    if (elapsed > 10000) {
      RCLCPP_WARN(
        get_logger(),
        " !!! Failed to get static transforms. Is the "
        "'ROBOT STATE PUBLISHER' node correctly "
        "working? ");
      break;
    }
  } while (transformOk == false);

  if (transformOk) {
    DEBUG_STREAM_PT(
      "Time required to get valid static transforms: "
        << elapsed / 1000. << " sec");
  }

  RCLCPP_INFO(
    get_logger(),
    "Initial ZED left camera pose (ZED pos. tracking): ");
  RCLCPP_INFO(
    get_logger(), " * T: [%g,%g,%g]", mInitialPoseSl.getTranslation().x,
    mInitialPoseSl.getTranslation().y, mInitialPoseSl.getTranslation().z);
  RCLCPP_INFO(
    get_logger(), " * Q: [%g,%g,%g,%g]", mInitialPoseSl.getOrientation().ox,
    mInitialPoseSl.getOrientation().oy, mInitialPoseSl.getOrientation().oz,
    mInitialPoseSl.getOrientation().ow);

  if (mAreaMemoryDbPath != "" && !sl_tools::file_exist(mAreaMemoryDbPath)) {
    mAreaMemoryDbPath = "";
    RCLCPP_WARN_STREAM(
      get_logger(),
      "'area_memory_db_path' path doesn't exist or is unreachable: "
        << mAreaMemoryDbPath);
  }

  // Tracking parameters
  sl::PositionalTrackingParameters ptParams;

  mPoseSmoothing = false;  // Always false. Pose Smoothing is to be enabled only
                           // for VR/AR applications

  ptParams.enable_pose_smoothing = mPoseSmoothing;
  ptParams.enable_area_memory = mAreaMemory;
  ptParams.area_file_path = mAreaMemoryDbPath.c_str();
  ptParams.enable_imu_fusion = mImuFusion;
  ptParams.initial_world_transform = mInitialPoseSl;
  ptParams.set_floor_as_origin = mFloorAlignment;
  ptParams.depth_min_range = mPosTrackDepthMinRange;
  ptParams.set_as_static = mSetAsStatic;
  ptParams.set_gravity_as_origin = mSetGravityAsOrigin;
  ptParams.mode = mPosTrkMode;

  sl::ERROR_CODE err = mZed->enablePositionalTracking(ptParams);

  if (err != sl::ERROR_CODE::SUCCESS) {
    mPosTrackingStarted = false;
    RCLCPP_WARN(
      get_logger(), "Pos. Tracking not started: %s",
      sl::toString(err).c_str());
    return false;
  }

  DEBUG_PT("Positional Tracking started");

  // ----> Enable Fusion Positional Tracking if required
  if (mGnssFusionEnabled && err == sl::ERROR_CODE::SUCCESS) {
    mMap2UtmTransfValid = false;

    sl::PositionalTrackingFusionParameters fusion_params;
    fusion_params.enable_GNSS_fusion = mGnssFusionEnabled;

    sl::GNSSCalibrationParameters gnss_par;
    gnss_par.target_yaw_uncertainty = mGnssTargetYawUncertainty;
    gnss_par.enable_translation_uncertainty_target =
      mGnssEnableTranslationUncertaintyTarget;
    gnss_par.target_translation_uncertainty = mGnssTargetTranslationUncertainty;
    gnss_par.enable_reinitialization = mGnssEnableReinitialization;
    gnss_par.gnss_vio_reinit_threshold = mGnssVioReinitThreshold;
    gnss_par.enable_rolling_calibration = mGnssEnableRollingCalibration;
    gnss_par.gnss_antenna_position = mGnssAntennaPose;

    DEBUG_STREAM_GNSS(
      "GNSS antenna pose in ZED SDK coordinate: "
        << mGnssAntennaPose[0] << "," << mGnssAntennaPose[1]
        << "," << mGnssAntennaPose[2]);

    fusion_params.gnss_calibration_parameters = gnss_par;

    sl::FUSION_ERROR_CODE fus_err =
      mFusion.enablePositionalTracking(fusion_params);

    if (fus_err != sl::FUSION_ERROR_CODE::SUCCESS) {
      mPosTrackingStarted = false;
      RCLCPP_WARN(
        get_logger(), "Fusion Pos. Tracking not started: %s",
        sl::toString(fus_err).c_str());
      mZed->disablePositionalTracking();
      return false;
    }
    DEBUG_GNSS("Fusion Positional Tracking started");
  }
  // <---- Enable Fusion Positional Tracking if required

  mPosTrackingStarted = true;

  startPathPubTimer(mPathPubRate);

  return mPosTrackingStarted;
}

bool ZedCamera::start3dMapping()
{
  DEBUG_MAP("start3dMapping");
  if (mDepthDisabled) {
    RCLCPP_WARN(
      get_logger(),
      "Cannot start 3D Mapping if `depth.depth_mode` is set to `0` [NONE]");
    return false;
  }

  if (mSpatialMappingRunning) {
    RCLCPP_WARN(
      get_logger(),
      "Cannot start 3D Mapping. The module is already running!");
    return false;
  }

  bool required = mMappingEnabled;

  if (!required) {
    return false;
  }

  RCLCPP_INFO_STREAM(get_logger(), "*** Starting Spatial Mapping ***");

  sl::SpatialMappingParameters params;
  params.map_type =
    sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
  params.use_chunk_only = true;

  sl::SpatialMappingParameters spMapPar;

  float lRes = spMapPar.allowed_resolution.first;
  float hRes = spMapPar.allowed_resolution.second;

  if (mMappingRes < lRes) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "'mapping.resolution' value ("
        << mMappingRes
        << " m) is lower than the allowed resolution "
        "values. Fixed automatically");
    mMappingRes = lRes;
  }
  if (mMappingRes > hRes) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "'mapping.resolution' value ("
        << mMappingRes
        << " m) is higher than the allowed resolution "
        "values. Fixed automatically");
    mMappingRes = hRes;
  }

  params.resolution_meter = mMappingRes;

  float lRng = spMapPar.allowed_range.first;
  float hRng = spMapPar.allowed_range.second;

  if (mMappingRangeMax < 0) {
    mMappingRangeMax =
      sl::SpatialMappingParameters::getRecommendedRange(mMappingRes, *mZed.get());
    RCLCPP_INFO_STREAM(
      get_logger(), "Mapping: max range set to "
        << mMappingRangeMax
        << " m for a resolution of "
        << mMappingRes << " m");
  } else if (mMappingRangeMax < lRng) {
    RCLCPP_WARN_STREAM(
      get_logger(), "'mapping.max_mapping_range_m' value ("
        << mMappingRangeMax
        << " m) is lower than the allowed "
        "resolution values. Fixed "
        "automatically");
    mMappingRangeMax = lRng;
  } else if (mMappingRangeMax > hRng) {
    RCLCPP_WARN_STREAM(
      get_logger(), "'mapping.max_mapping_range_m' value ("
        << mMappingRangeMax
        << " m) is higher than the allowed "
        "resolution values. Fixed "
        "automatically");
    mMappingRangeMax = hRng;
  }

  params.range_meter = mMappingRangeMax;

  sl::ERROR_CODE err = mZed->enableSpatialMapping(params);

  if (err == sl::ERROR_CODE::SUCCESS) {
    if (mPubFusedCloud == nullptr) {
#ifndef FOUND_FOXY
      mPubFusedCloud = point_cloud_transport::create_publisher(
        this->shared_from_this(), mPointcloudFusedTopic,
        mQos.get_rmw_qos_profile(), mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic "
          << mPubFusedCloud.getTopic()
          << " @ " << mFusedPcPubRate
          << " Hz");
#else
      mPubFusedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
        mPointcloudFusedTopic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic "
          << mPubFusedCloud->get_topic_name()
          << " @ " << mFusedPcPubRate
          << " Hz");
#endif
    }

    mSpatialMappingRunning = true;

    startFusedPcTimer(mFusedPcPubRate);

    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Resolution: " << params.resolution_meter << " m");
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Max Mapping Range: " << params.range_meter << " m");
    RCLCPP_INFO_STREAM(
      get_logger(), " * Map point cloud publishing rate: "
        << mFusedPcPubRate << " Hz");

    return true;
  } else {
    mSpatialMappingRunning = false;
    if (mFusedPcTimer) {
      mFusedPcTimer->cancel();
    }

    RCLCPP_WARN(
      get_logger(), "Mapping not activated: %s",
      sl::toString(err).c_str());

    return false;
  }
}

void ZedCamera::stop3dMapping()
{
  if (mFusedPcTimer) {
    mFusedPcTimer->cancel();
  }
  mSpatialMappingRunning = false;
  mMappingEnabled = false;
  mZed->disableSpatialMapping();

  RCLCPP_INFO(get_logger(), "*** Spatial Mapping stopped ***");
}

bool ZedCamera::startObjDetect()
{
  DEBUG_OD("startObjDetect");

  if (!sl_tools::isObjDetAvailable(mCamRealModel)) {
    RCLCPP_ERROR(
      get_logger(),
      "Object detection not started. The camera model does not "
      "support it with the current version "
      "of the "
      "SDK");
    return false;
  }

  if (mDepthDisabled) {
    RCLCPP_WARN(
      get_logger(),
      "Cannot start Object Detection if "
      "`depth.depth_mode` is set to `0` [NONE]");
    return false;
  }

  if (!mObjDetEnabled) {
    return false;
  }

  if (!mCamera2BaseTransfValid || !mSensor2CameraTransfValid ||
    !mSensor2BaseTransfValid)
  {
    DEBUG_OD("Tracking transforms not yet ready, OD starting postponed");
    return false;
  }

  RCLCPP_INFO(get_logger(), "*** Starting Object Detection ***");

  sl::ObjectDetectionParameters od_p;
  od_p.enable_segmentation = false;
  od_p.enable_tracking = mObjDetTracking;
  od_p.detection_model = mObjDetModel;
  od_p.filtering_mode = mObjFilterMode;
  od_p.prediction_timeout_s = mObjDetPredTimeout;
  od_p.allow_reduced_precision_inference = mObjDetReducedPrecision;
  od_p.max_range = mObjDetMaxRange;

  mObjDetInstID = ++mAiInstanceID;
  od_p.instance_module_id = mObjDetInstID;

  mObjDetFilter.clear();
  if (mObjDetPeopleEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::PERSON);
  }
  if (mObjDetVehiclesEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::VEHICLE);
  }
  if (mObjDetBagsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::BAG);
  }
  if (mObjDetAnimalsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::ANIMAL);
  }
  if (mObjDetElectronicsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::ELECTRONICS);
  }
  if (mObjDetFruitsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::FRUIT_VEGETABLE);
  }
  if (mObjDetSportEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::SPORT);
  }

  sl::ERROR_CODE objDetError = mZed->enableObjectDetection(od_p);

  if (objDetError != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Object detection error: " << sl::toString(objDetError));

    mObjDetRunning = false;
    return false;
  }

  if (!mPubObjDet) {
    mPubObjDet = create_publisher<zed_interfaces::msg::ObjectsStamped>(
      mObjectDetTopic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Advertised on topic " << mPubObjDet->get_topic_name());
  }

  mObjDetRunning = true;
  return true;
}

void ZedCamera::stopObjDetect()
{
  if (mObjDetRunning) {
    RCLCPP_INFO(get_logger(), "*** Stopping Object Detection ***");
    mObjDetRunning = false;
    mObjDetEnabled = false;
    mZed->disableObjectDetection();

    // ----> Send an empty message to indicate that no more objects are tracked
    // (e.g clean RVIZ2)
    objDetMsgPtr objMsg =
      std::make_unique<zed_interfaces::msg::ObjectsStamped>();

    objMsg->header.stamp = mFrameTimestamp;
    objMsg->header.frame_id = mLeftCamFrameId;

    objMsg->objects.clear();

    DEBUG_STREAM_OD(
      "Publishing EMPTY OBJ message "
        << mPubObjDet->get_topic_name());
    try {
      mPubObjDet->publish(std::move(objMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what() );
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
    // <---- Send an empty message to indicate that no more objects are tracked
    // (e.g clean RVIZ2)
  }
}

bool ZedCamera::startBodyTracking()
{
  DEBUG_BT("startBodyTracking");

  if (!sl_tools::isObjDetAvailable(mCamRealModel)) {
    RCLCPP_ERROR(
      get_logger(),
      "Body Tracking not started. The camera model does not support "
      "it with the current version "
      "of the SDK");
    return false;
  }

  DEBUG_BT("Body Tracking available");

  if (mDepthDisabled) {
    RCLCPP_WARN(
      get_logger(),
      "Cannot start Body Tracking if "
      "`depth.depth_mode` is set to `0` [NONE]");
    return false;
  }

  if (!mBodyTrkEnabled) {
    DEBUG_BT("Body Tracking not enabled -> NOT STARTING");
    return false;
  }

  if (!mCamera2BaseTransfValid || !mSensor2CameraTransfValid ||
    !mSensor2BaseTransfValid)
  {
    DEBUG_OD(
      "Tracking transforms not yet ready, Body Tracking starting postponed");
    return false;
  }

  RCLCPP_INFO(get_logger(), "*** Starting Body Tracking ***");

  sl::BodyTrackingParameters bt_p;
  bt_p.allow_reduced_precision_inference = mBodyTrkReducedPrecision;
  bt_p.body_format = mBodyTrkFmt;
  bt_p.body_selection = mBodyTrkKpSelection;
  bt_p.detection_model = mBodyTrkModel;
  bt_p.enable_body_fitting = mBodyTrkFitting;
  bt_p.enable_segmentation = false;
  bt_p.enable_tracking = mBodyTrkEnableTracking;
  bt_p.max_range = mBodyTrkMaxRange;
  bt_p.prediction_timeout_s = mBodyTrkPredTimeout;

  mBodyTrkInstID = ++mAiInstanceID;
  bt_p.instance_module_id = mBodyTrkInstID;

  sl::ERROR_CODE btError = mZed->enableBodyTracking(bt_p);

  if (btError != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Body Tracking error: " << sl::toString(btError));

    mBodyTrkRunning = false;
    return false;
  }

  DEBUG_BT("Body Tracking enabled");

  if (!mPubBodyTrk) {
    mPubBodyTrk = create_publisher<zed_interfaces::msg::ObjectsStamped>(
      mBodyTrkTopic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic " << mPubBodyTrk->get_topic_name());
  }

  DEBUG_BT("Body Tracking publisher created");

  mBodyTrkRunning = true;
  return true;
}

void ZedCamera::stopBodyTracking()
{
  if (mBodyTrkRunning) {
    RCLCPP_INFO(get_logger(), "*** Stopping Body Tracking ***");
    mBodyTrkRunning = false;
    mBodyTrkEnabled = false;
    mZed->disableBodyTracking();

    // ----> Send an empty message to indicate that no more objects are tracked
    // (e.g clean RVIZ2)
    objDetMsgPtr objMsg =
      std::make_unique<zed_interfaces::msg::ObjectsStamped>();

    objMsg->header.stamp = mFrameTimestamp;
    objMsg->header.frame_id = mLeftCamFrameId;

    objMsg->objects.clear();

    DEBUG_STREAM_OD(
      "Publishing EMPTY OBJ message "
        << mPubBodyTrk->get_topic_name());
    try {
      mPubBodyTrk->publish(std::move(objMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
    // <---- Send an empty message to indicate that no more objects are tracked
    // (e.g clean RVIZ2)
  }
}

bool ZedCamera::startSvoRecording(std::string & errMsg)
{
  sl::RecordingParameters params;

  params.bitrate = mSvoRecBitrate;
  params.compression_mode = mSvoRecCompr;
  params.target_framerate = mSvoRecFramerate;
  params.transcode_streaming_input = mSvoRecTranscode;
  params.video_filename = mSvoRecFilename.c_str();

  sl::ERROR_CODE err = mZed->enableRecording(params);
  errMsg = sl::toString(err);

  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Error starting SVO recording: " << errMsg);
    return false;
  }

  mRecording = true;

  return true;
}

void ZedCamera::stopSvoRecording()
{
  if (mRecording) {
    mRecording = false;
    mZed->disableRecording();
  }
}

void ZedCamera::initTransforms()
{
  // According to REP 105 -> http://www.ros.org/reps/rep-0105.html

  // camera_link <- odom <- map
  //     ^                   |
  //     |                   |
  //     ---------------------

  // ----> Dynamic transforms
  mOdom2BaseTransf.setIdentity();  // broadcasted if `publish_tf` is true
  mMap2OdomTransf.setIdentity();   // broadcasted if `publish_map_tf` is true
  mMap2BaseTransf.setIdentity();   // used internally, but not broadcasted
  mMap2UtmTransf.setIdentity();    // broadcasted if GNSS Fusion is enabled
  // <---- Dynamic transforms
}

bool ZedCamera::getCamera2BaseTransform()
{
  DEBUG_STREAM_PT(
    "Getting static TF from '" << mCameraFrameId.c_str()
                               << "' to '" << mBaseFrameId.c_str()
                               << "'");

  mCamera2BaseTransfValid = false;

  // ----> Static transforms
  // Sensor to Base link
  try {
    // Save the transformation
    geometry_msgs::msg::TransformStamped c2b = mTfBuffer->lookupTransform(
      mCameraFrameId, mBaseFrameId, TIMEZERO_SYS, rclcpp::Duration(1, 0));

    // Get the TF2 transformation
    // tf2::fromMsg(c2b.transform, mCamera2BaseTransf);
    geometry_msgs::msg::Transform in = c2b.transform;
    mCamera2BaseTransf.setOrigin(
      tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
    // w at the end in the constructor
    mCamera2BaseTransf.setRotation(
      tf2::Quaternion(
        in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));

    double roll, pitch, yaw;
    tf2::Matrix3x3(mCamera2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(
      get_logger(),
      " Static transform Camera Center to Base [%s -> %s]",
      mCameraFrameId.c_str(), mBaseFrameId.c_str());
    RCLCPP_INFO(
      get_logger(), "  * Translation: {%.3f,%.3f,%.3f}",
      mCamera2BaseTransf.getOrigin().x(),
      mCamera2BaseTransf.getOrigin().y(),
      mCamera2BaseTransf.getOrigin().z());
    RCLCPP_INFO(
      get_logger(), "  * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG,
      pitch * RAD2DEG, yaw * RAD2DEG);
  } catch (tf2::TransformException & ex) {
    if (!mCamera2BaseFirstErr) {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock, 1000.0,
        "Transform error: %s", ex.what());
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock, 1000.0,
        "The tf from '%s' to '%s' is not available.",
        mCameraFrameId.c_str(), mBaseFrameId.c_str());
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock, 1000.0,
        "Note: one of the possible cause of the problem is the absense of an "
        "instance "
        "of the `robot_state_publisher` node publishing the correct static "
        "TF transformations "
        "or a modified URDF not correctly reproducing the ZED "
        "TF chain '%s' -> '%s' -> '%s'",
        mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
      mCamera2BaseFirstErr = false;
    }

    mCamera2BaseTransf.setIdentity();
    return false;
  }
  // <---- Static transforms

  mCamera2BaseTransfValid = true;
  return true;
}

bool ZedCamera::getSens2CameraTransform()
{
  DEBUG_STREAM_PT(
    "Getting static TF from '"
      << mDepthFrameId.c_str() << "' to '" << mCameraFrameId.c_str()
      << "'");

  mSensor2CameraTransfValid = false;

  // ----> Static transforms
  // Sensor to Camera Center
  try {
    // Save the transformation
    geometry_msgs::msg::TransformStamped s2c = mTfBuffer->lookupTransform(
      mDepthFrameId, mCameraFrameId, TIMEZERO_SYS, rclcpp::Duration(1, 0));

    // Get the TF2 transformation
    // tf2::fromMsg(s2c.transform, mSensor2CameraTransf);
    geometry_msgs::msg::Transform in = s2c.transform;
    mSensor2CameraTransf.setOrigin(
      tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
    // w at the end in the constructor
    mSensor2CameraTransf.setRotation(
      tf2::Quaternion(
        in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));

    double roll, pitch, yaw;
    tf2::Matrix3x3(mSensor2CameraTransf.getRotation()).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(
      get_logger(),
      " Static transform ref. CMOS Sensor to Camera Center [%s -> %s]",
      mDepthFrameId.c_str(), mCameraFrameId.c_str());
    RCLCPP_INFO(
      get_logger(), "  * Translation: {%.3f,%.3f,%.3f}",
      mSensor2CameraTransf.getOrigin().x(),
      mSensor2CameraTransf.getOrigin().y(),
      mSensor2CameraTransf.getOrigin().z());
    RCLCPP_INFO(
      get_logger(), "  * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG,
      pitch * RAD2DEG, yaw * RAD2DEG);
  } catch (tf2::TransformException & ex) {
    if (!mSensor2CameraTransfFirstErr) {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock, 1000.0,
        "Transform error: %s", ex.what());
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock, 1000.0,
        "The tf from '%s' to '%s' is not available.",
        mDepthFrameId.c_str(), mCameraFrameId.c_str());
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock, 1000.0,
        "Note: one of the possible cause of the problem is the absense of an "
        "instance "
        "of the `robot_state_publisher` node publishing the correct static "
        "TF transformations "
        "or a modified URDF not correctly reproducing the ZED "
        "TF chain '%s' -> '%s' -> '%s'",
        mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
      mSensor2CameraTransfFirstErr = false;
    }

    mSensor2CameraTransf.setIdentity();
    return false;
  }
  // <---- Static transforms

  mSensor2CameraTransfValid = true;
  return true;
}

bool ZedCamera::getSens2BaseTransform()
{
  DEBUG_STREAM_PT(
    "Getting static TF from '" << mDepthFrameId.c_str()
                               << "' to '" << mBaseFrameId.c_str()
                               << "'");

  mSensor2BaseTransfValid = false;

  // ----> Static transforms
  // Sensor to Base link
  try {
    // Save the transformation
    geometry_msgs::msg::TransformStamped s2b = mTfBuffer->lookupTransform(
      mDepthFrameId, mBaseFrameId, TIMEZERO_SYS, rclcpp::Duration(1, 0));

    // Get the TF2 transformation
    // tf2::fromMsg(s2b.transform, mSensor2BaseTransf);
    geometry_msgs::msg::Transform in = s2b.transform;
    mSensor2BaseTransf.setOrigin(
      tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
    // w at the end in the constructor
    mSensor2BaseTransf.setRotation(
      tf2::Quaternion(
        in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));

    double roll, pitch, yaw;
    tf2::Matrix3x3(mSensor2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(
      get_logger(),
      " Static transform ref. CMOS Sensor to Base [%s -> %s]",
      mDepthFrameId.c_str(), mBaseFrameId.c_str());
    RCLCPP_INFO(
      get_logger(), "  * Translation: {%.3f,%.3f,%.3f}",
      mSensor2BaseTransf.getOrigin().x(),
      mSensor2BaseTransf.getOrigin().y(),
      mSensor2BaseTransf.getOrigin().z());
    RCLCPP_INFO(
      get_logger(), "  * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG,
      pitch * RAD2DEG, yaw * RAD2DEG);
  } catch (tf2::TransformException & ex) {
    if (!mSensor2BaseTransfFirstErr) {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock, 1000.0,
        "Transform error: %s", ex.what());
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock, 1000.0,
        "The tf from '%s' to '%s' is not available.",
        mDepthFrameId.c_str(), mBaseFrameId.c_str());
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock, 1000.0,
        "Note: one of the possible cause of the problem is the absense of an "
        "instance "
        "of the `robot_state_publisher` node publishing the correct static "
        "TF transformations "
        "or a modified URDF not correctly reproducing the ZED "
        "TF chain '%s' -> '%s' -> '%s'",
        mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
      mSensor2BaseTransfFirstErr = false;
    }

    mSensor2BaseTransf.setIdentity();
    return false;
  }

  // <---- Static transforms

  mSensor2BaseTransfValid = true;
  return true;
}

bool ZedCamera::getGnss2BaseTransform()
{
  DEBUG_GNSS(
    "Getting static TF from '%s' to '%s'", mGnssFrameId.c_str(),
    mBaseFrameId.c_str());

  mGnss2BaseTransfValid = false;

  // ----> Static transforms
  // Sensor to Base link
  try {
    // Save the transformation
    geometry_msgs::msg::TransformStamped g2b = mTfBuffer->lookupTransform(
      mGnssFrameId, mBaseFrameId, TIMEZERO_SYS, rclcpp::Duration(1, 0));

    // Get the TF2 transformation
    geometry_msgs::msg::Transform in = g2b.transform;
    mGnss2BaseTransf.setOrigin(
      tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
    // w at the end in the constructor
    mGnss2BaseTransf.setRotation(
      tf2::Quaternion(
        in.rotation.x, in.rotation.y,
        in.rotation.z, in.rotation.w));

    double roll, pitch, yaw;
    tf2::Matrix3x3(mGnss2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(
      get_logger(),
      " Static transform GNSS Antenna to Camera Base [%s -> %s]",
      mGnssFrameId.c_str(), mBaseFrameId.c_str());
    RCLCPP_INFO(
      get_logger(), "  * Translation: {%.3f,%.3f,%.3f}",
      mGnss2BaseTransf.getOrigin().x(),
      mGnss2BaseTransf.getOrigin().y(),
      mGnss2BaseTransf.getOrigin().z());
    RCLCPP_INFO(
      get_logger(), "  * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG,
      pitch * RAD2DEG, yaw * RAD2DEG);
  } catch (tf2::TransformException & ex) {
    if (!mGnss2BaseTransfFirstErr) {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      DEBUG_STREAM_THROTTLE_GNSS(1000.0, "Transform error: " << ex.what());
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock, 1000.0,
        "The tf from '%s' to '%s' is not available.",
        mGnssFrameId.c_str(), mBaseFrameId.c_str());
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock, 1000.0,
        "Note: one of the possible cause of the problem is the absense of an "
        "instance "
        "of the `robot_state_publisher` node publishing the correct static "
        "TF transformations "
        "or a modified URDF not correctly reproducing the "
        "TF chain '%s' -> '%s'",
        mBaseFrameId.c_str(), mGnssFrameId.c_str());
      mGnss2BaseTransfFirstErr = false;
    }

    mGnss2BaseTransf.setIdentity();
    return false;
  }
  // <---- Static transforms

  mGnss2BaseTransfValid = true;
  return true;
}

bool ZedCamera::setPose(
  float xt, float yt, float zt, float rr, float pr,
  float yr)
{
  initTransforms();

  if (!mSensor2BaseTransfValid) {
    getSens2BaseTransform();
  }

  if (!mSensor2CameraTransfValid) {
    getSens2CameraTransform();
  }

  if (!mCamera2BaseTransfValid) {
    getCamera2BaseTransform();
  }

  // Apply Base to sensor transform
  tf2::Transform initPose;
  tf2::Vector3 origin(xt, yt, zt);
  initPose.setOrigin(origin);
  tf2::Quaternion quat;
  quat.setRPY(rr, pr, yr);
  initPose.setRotation(quat);

  initPose = initPose * mSensor2BaseTransf.inverse();

  // SL pose
  sl::float3 t_vec;
  t_vec[0] = initPose.getOrigin().x();
  t_vec[1] = initPose.getOrigin().y();
  t_vec[2] = initPose.getOrigin().z();

  sl::float4 q_vec;
  q_vec[0] = initPose.getRotation().x();
  q_vec[1] = initPose.getRotation().y();
  q_vec[2] = initPose.getRotation().z();
  q_vec[3] = initPose.getRotation().w();

  sl::Translation trasl(t_vec);
  sl::Orientation orient(q_vec);
  mInitialPoseSl.setTranslation(trasl);
  mInitialPoseSl.setOrientation(orient);

  return mSensor2BaseTransfValid & mSensor2CameraTransfValid &
         mCamera2BaseTransfValid;
}

void ZedCamera::publishImuFrameAndTopic()
{
  sl::Orientation sl_rot = mSlCamImuTransf.getOrientation();
  sl::Translation sl_tr = mSlCamImuTransf.getTranslation();

  transfMsgPtr cameraImuTransfMgs =
    std::make_unique<geometry_msgs::msg::TransformStamped>();

  cameraImuTransfMgs->header.stamp = get_clock()->now();

  cameraImuTransfMgs->header.frame_id = mLeftCamFrameId;
  cameraImuTransfMgs->child_frame_id = mImuFrameId;

  cameraImuTransfMgs->transform.rotation.x = sl_rot.ox;
  cameraImuTransfMgs->transform.rotation.y = sl_rot.oy;
  cameraImuTransfMgs->transform.rotation.z = sl_rot.oz;
  cameraImuTransfMgs->transform.rotation.w = sl_rot.ow;

  cameraImuTransfMgs->transform.translation.x = sl_tr.x;
  cameraImuTransfMgs->transform.translation.y = sl_tr.y;
  cameraImuTransfMgs->transform.translation.z = sl_tr.z;

  try {
    mPubCamImuTransf->publish(std::move(cameraImuTransfMgs));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }

  // Publish IMU TF as static TF
  if (!mPublishImuTF) {
    return;
  }

  // ----> Publish TF
  // RCLCPP_INFO(get_logger(), "Broadcasting Camera-IMU TF ");

  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp =
    get_clock()->now() + rclcpp::Duration(0, mTfOffset * 1e9);

  transformStamped.header.frame_id = mLeftCamFrameId;
  transformStamped.child_frame_id = mImuFrameId;

  transformStamped.transform.rotation.x = sl_rot.ox;
  transformStamped.transform.rotation.y = sl_rot.oy;
  transformStamped.transform.rotation.z = sl_rot.oz;
  transformStamped.transform.rotation.w = sl_rot.ow;

  transformStamped.transform.translation.x = sl_tr.x;
  transformStamped.transform.translation.y = sl_tr.y;
  transformStamped.transform.translation.z = sl_tr.z;

  mTfBroadcaster->sendTransform(transformStamped);
  // <---- Publish TF

  // IMU TF publishing diagnostic
  double elapsed_sec = mImuTfFreqTimer.toc();
  mPubImuTF_sec->addValue(elapsed_sec);
  mImuTfFreqTimer.tic();
}

void ZedCamera::threadFunc_zedGrab()
{
  DEBUG_STREAM_COMM("Grab thread started");

  // ----> Advanced thread settings
  DEBUG_STREAM_ADV("Grab thread settings");
  if (mDebugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * Default GRAB thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par.sched_priority);
    }
  }

  if (mThreadSchedPolicy == "SCHED_OTHER") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_BATCH") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_BATCH, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_FIFO") {
    sched_param par;
    par.sched_priority = mThreadPrioGrab;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_RR") {
    sched_param par;
    par.sched_priority = mThreadPrioGrab;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(), " ! Failed to set thread params! - Policy not supported");
  }

  if (mDebugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * New GRAB thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par.sched_priority);
    }
  }
  // <---- Advanced thread settings

  mFrameCount = 0;

  // ----> Grab Runtime parameters
  mRunParams.enable_depth = false;
  mRunParams.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
  mRunParams.remove_saturated_areas = mRemoveSatAreas;
  // <---- Grab Runtime parameters

  // Infinite grab thread
  while (1) {
    try {
      if (mUseSimTime && !mClockAvailable) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(
          get_logger(), steady_clock, 5000.0,
          "Waiting for a valid simulation time on the '/clock' topic...");
        continue;
      }

      sl_tools::StopWatch grabElabTimer(get_clock());

      // ----> Interruption check
      if (!rclcpp::ok()) {
        DEBUG_STREAM_COMM("Ctrl+C received: stopping grab thread");
        break;
      }

      if (mThreadStop) {
        DEBUG_STREAM_COMM("Grab thread stopped");
        break;
      }
      // <---- Interruption check

      // ----> Apply depth settings
      applyDepthSettings();
      // <---- Apply depth settings

      // ----> Apply video dynamic parameters
      if (!mSimMode && !mSvoMode) {
        applyVideoSettings();
      }
      // <---- Apply video dynamic parameters

      // ----> Check for Positional Tracking requirement
      if (isPosTrackingRequired() && !mPosTrackingStarted) {
        static int pt_err_count = 0;
        if (!startPosTracking()) {
          if (++pt_err_count >= 3) {
            RCLCPP_FATAL(
              get_logger(),
              "It's not possible to enable the required Positional "
              "Tracking module.");
            exit(EXIT_FAILURE);
          }
        } else {
          pt_err_count = 0;
        }
      }

      if (mGnssFusionEnabled && !mGnssFixValid) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(
          get_logger(), steady_clock, 5000.0,
          " * Waiting for the first valid GNSS fix...");
      }
      // ----> Check for Positional Tracking requirement

      if (!mDepthDisabled) {
        // ----> Check for Spatial Mapping requirement

        mMappingMutex.lock();
        bool required = mMappingEnabled;

        if (required && !mSpatialMappingRunning) {
          start3dMapping();
        }
        mMappingMutex.unlock();

        // <---- Check for Spatial Mapping requirement

        // ----> Check for Object Detection requirement
        mObjDetMutex.lock();
        if (mObjDetEnabled && !mObjDetRunning) {
          startObjDetect();
          if (!sl_tools::isObjDetAvailable(mCamRealModel)) {
            mObjDetEnabled = false;
          }
        }
        mObjDetMutex.unlock();

        // ----> Check for Object Detection requirement

        // ----> Check for Body Tracking requirement
        mBodyTrkMutex.lock();
        if (mBodyTrkEnabled && !mBodyTrkRunning) {
          startBodyTracking();
          if (!sl_tools::isObjDetAvailable(mCamRealModel)) {
            mBodyTrkEnabled = false;
          }
        }
        mBodyTrkMutex.unlock();
        // ----> Check for Object Detection requirement
      }

      // ----> Grab freq calculation
      double elapsed_sec = mGrabFreqTimer.toc();
      mGrabPeriodMean_sec->addValue(elapsed_sec);
      mGrabFreqTimer.tic();

      // RCLCPP_INFO_STREAM(get_logger(), "Grab period: "
      // << mGrabPeriodMean_sec->getAvg() / 1e6
      // << " Freq: " << 1e6 / mGrabPeriodMean_usec->getAvg());
      // <---- Grab freq calculation

      if (!mSvoPause) {
        // Start processing timer for diagnostic
        grabElabTimer.tic();

        // ZED grab
        mGrabStatus = mZed->grab(mRunParams);

        // ----> Grab errors?
        // Note: disconnection are automatically handled by the ZED SDK
        if (mGrabStatus != sl::ERROR_CODE::SUCCESS) {
          if (mSvoMode && mGrabStatus == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
            // ----> Check SVO status
            if (mSvoLoop) {
              mZed->setSVOPosition(0);
              RCLCPP_WARN(
                get_logger(),
                "SVO reached the end and it has been restarted.");
              rclcpp::sleep_for(
                std::chrono::microseconds(
                  static_cast<int>(mGrabPeriodMean_sec->getAvg() * 1e6)));
              continue;
            } else {
              RCLCPP_WARN(
                get_logger(),
                "SVO reached the end. The node has been stopped.");
              break;
            }
            // <---- Check SVO status
          } else if (mGrabStatus == sl::ERROR_CODE::CAMERA_REBOOTING) {
            RCLCPP_ERROR_STREAM(
              get_logger(),
              "Connection issue detected: "
                << sl::toString(mGrabStatus).c_str());
            rclcpp::sleep_for(1000ms);
            continue;
          } else if (mGrabStatus == sl::ERROR_CODE::CAMERA_NOT_INITIALIZED ||
            mGrabStatus == sl::ERROR_CODE::FAILURE)
          {
            RCLCPP_ERROR_STREAM(
              get_logger(),
              "Camera issue detected: "
                << sl::toString(mGrabStatus).c_str() << ". Trying to recover the connection...");
            rclcpp::sleep_for(1000ms);
            continue;
          } else {
            RCLCPP_ERROR_STREAM(
              get_logger(),
              "Critical camera error: " << sl::toString(mGrabStatus).c_str()
                                        << ". NODE KILLED.");
            mZed.reset();
            exit(EXIT_FAILURE);
          }
        }
        // <---- Grab errors?

        mFrameCount++;

        if (mGnssFusionEnabled) {
          // Process Fusion data
          mFusionStatus = mFusion.process();
          // ----> Fusion errors?
          if (mFusionStatus != sl::FUSION_ERROR_CODE::SUCCESS &&
            mFusionStatus != sl::FUSION_ERROR_CODE::NO_NEW_DATA_AVAILABLE)
          {
            RCLCPP_ERROR_STREAM(
              get_logger(),
              "Fusion error: " << sl::toString(mFusionStatus).c_str());
          }
          // <---- Fusion errors?
        }

        // ----> Timestamp
        if (mSvoMode) {
          mFrameTimestamp = sl_tools::slTime2Ros(
            mZed->getTimestamp(sl::TIME_REFERENCE::CURRENT));
        } else if (mSimMode) {
          if (mUseSimTime) {
            mFrameTimestamp = get_clock()->now();
          } else {
            mFrameTimestamp = sl_tools::slTime2Ros(
              mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
          }
        } else {
          mFrameTimestamp =
            sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
        }
        // <---- Timestamp

        if (mStreamingServerRequired && !mStreamingServerRunning) {
          DEBUG_STR("Streaming server required, but not running");
          startStreamingServer();
        }

        if (!mSimMode) {
          if (mGnssFusionEnabled && mGnssFixNew) {
            mGnssFixNew = false;

            rclcpp::Time real_frame_ts = sl_tools::slTime2Ros(
              mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
            DEBUG_STREAM_GNSS(
              "GNSS synced frame ts: "
                << real_frame_ts.nanoseconds() << " nsec");
            float dT_sec = (static_cast<float>(real_frame_ts.nanoseconds()) -
              static_cast<float>(mGnssTimestamp.nanoseconds())) /
              1e9;
            DEBUG_STREAM_GNSS(
              "DeltaT: "
                << dT_sec << " sec [" << std::fixed << std::setprecision(9)
                << static_cast<float>(real_frame_ts.nanoseconds()) / 1e9 << "-"
                << static_cast<float>(mGnssTimestamp.nanoseconds()) / 1e9 << "]");

            if (dT_sec < 0.0) {
              RCLCPP_WARN_STREAM(
                get_logger(),
                "GNSS sensor and ZED Timestamps are not good. dT = " << dT_sec
                                                                     << " sec");
            }
          }
        }

        // ----> Check recording status
        mRecMutex.lock();
        if (mRecording) {
          mRecStatus = mZed->getRecordingStatus();

          if (!mRecStatus.status) {
            rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_ERROR_THROTTLE(
              get_logger(), steady_clock, 1000.0,
              "Error saving frame to SVO");
          }
        }
        mRecMutex.unlock();
        // <---- Check recording status
      }

      // ----> Retrieve Image/Depth data if someone has subscribed to
      // Retrieve data if there are subscriber to topics
      if (areVideoDepthSubscribed()) {
        DEBUG_STREAM_VD("Retrieving video/depth data");
        retrieveVideoDepth();

        rclcpp::Time pub_ts;
        publishVideoDepth(pub_ts);

        if (!sl_tools::isZED(mCamRealModel) && mVdPublishing &&
          pub_ts != TIMEZERO_ROS)
        {
          if (mSensCameraSync || mSvoMode || mSimMode) {
            publishSensorsData(pub_ts);
          }
        }

        mVdPublishing = true;
      } else {
        mVdPublishing = false;
      }
      // <---- Retrieve Image/Depth data if someone has subscribed to

      if (!mDepthDisabled) {
        // ----> Retrieve the point cloud if someone has subscribed to

        size_t cloudSubnumber = 0;
        try {
  #ifndef FOUND_FOXY
          cloudSubnumber = mPubCloud.getNumSubscribers();
  #else
          cloudSubnumber = count_subscribers(mPubCloud->get_topic_name());
  #endif
        } catch (...) {
          rcutils_reset_error();
          DEBUG_STREAM_PC(
            "threadFunc_zedGrab: Exception while counting point cloud "
            "subscribers");
          continue;
        }

        if (cloudSubnumber > 0) {
          // Run the point cloud conversion asynchronously to avoid slowing down
          // all the program
          // Retrieve raw pointCloud data if latest Pointcloud is ready
          std::unique_lock<std::mutex> pc_lock(mPcMutex, std::defer_lock);

          if (pc_lock.try_lock()) {
            DEBUG_STREAM_PC("Retrieving point cloud");
            mZed->retrieveMeasure(
              mMatCloud, sl::MEASURE::XYZBGRA, sl::MEM::CPU,
              mMatResol);

            // Signal Pointcloud thread that a new pointcloud is ready
            mPcDataReadyCondVar.notify_one();
            mPcDataReady = true;
            mPcPublishing = true;
          }
        } else {
          mPcPublishing = false;
        }
        // <---- Retrieve the point cloud if someone has subscribed to

        // ----> Localization processing
        if (mPosTrackingStarted) {
          if (!mSvoPause) {
            DEBUG_PT("================================================================");
            DEBUG_PT("***** processOdometry *****");
            processOdometry();
            DEBUG_PT("***** processPose *****");
            processPose();
            if (mGnssFusionEnabled) {
              if (mSvoMode) {
                DEBUG_PT("***** processSvoGnssData *****");
                processSvoGnssData();
              }
              DEBUG_PT("***** processGeoPose *****");
              processGeoPose();
            }
          }

          // Publish `odom` and `map` TFs at the grab frequency
          // RCLCPP_INFO(get_logger(), "Publishing TF -> threadFunc_zedGrab");
          DEBUG_PT("***** publishTFs *****");
          publishTFs(mFrameTimestamp);
        }
        // <---- Localization processing

        mObjDetMutex.lock();
        if (mObjDetRunning) {
          processDetectedObjects(mFrameTimestamp);
        }
        mObjDetMutex.unlock();

        mBodyTrkMutex.lock();
        if (mBodyTrkRunning) {
          processBodies(mFrameTimestamp);
        }
        mBodyTrkMutex.unlock();

        // ----> Region of interest
        processRtRoi(mFrameTimestamp);
        // <---- Region of interest
      }

      // Diagnostic statistics update
      double mean_elab_sec = mElabPeriodMean_sec->addValue(grabElabTimer.toc());
    } catch (...) {
      rcutils_reset_error();
      DEBUG_STREAM_COMM("threadFunc_zedGrab: Generic exception.");
      continue;
    }
  }

  DEBUG_STREAM_COMM("Grab thread finished");
}

rclcpp::Time ZedCamera::publishSensorsData(rclcpp::Time t)
{
  if (mGrabStatus != sl::ERROR_CODE::SUCCESS) {
    DEBUG_SENS("Camera not ready");
    rclcpp::sleep_for(1s);
    return TIMEZERO_ROS;
  }

  // ----> Subscribers count
  DEBUG_STREAM_SENS("Sensors callback: counting subscribers");

  size_t imu_SubNumber = 0;
  size_t imu_RawSubNumber = 0;
  size_t imu_TempSubNumber = 0;
  size_t imu_MagSubNumber = 0;
  size_t pressSubNumber = 0;

  try {
    imu_SubNumber = count_subscribers(mPubImu->get_topic_name());
    imu_RawSubNumber = count_subscribers(mPubImuRaw->get_topic_name());
    imu_MagSubNumber = 0;
    pressSubNumber = 0;

    if (sl_tools::isZED2OrZED2i(mCamRealModel)) {
      imu_MagSubNumber = count_subscribers(mPubImuMag->get_topic_name());
      pressSubNumber = count_subscribers(mPubPressure->get_topic_name());
    }
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_SENS("pubSensorsData: Exception while counting subscribers");
    return TIMEZERO_ROS;
  }
  // <---- Subscribers count

  // ----> Grab data and setup timestamps
  DEBUG_STREAM_ONCE_SENS("Sensors callback: Grab data and setup timestamps");
  rclcpp::Time ts_imu;
  rclcpp::Time ts_baro;
  rclcpp::Time ts_mag;

  rclcpp::Time now = get_clock()->now();

  sl::SensorsData sens_data;

  if (mSvoMode || mSensCameraSync || mSimMode) {
    sl::ERROR_CODE err =
      mZed->getSensorsData(sens_data, sl::TIME_REFERENCE::IMAGE);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "sl::getSensorsData error: "
          << sl::toString(err).c_str());
      return TIMEZERO_ROS;
    }
  } else {
    sl::ERROR_CODE err =
      mZed->getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "sl::getSensorsData error: "
          << sl::toString(err).c_str());
      return TIMEZERO_ROS;
    }
  }

  if (mSensCameraSync) {
    ts_imu = t;
    ts_baro = t;
    ts_mag = t;
  } else if (mSimMode) {
    if (mUseSimTime) {
      ts_imu = get_clock()->now();
    } else {
      ts_imu = sl_tools::slTime2Ros(sens_data.imu.timestamp);
    }
    ts_baro = ts_imu;
    ts_mag = ts_imu;
  } else {
    ts_imu = sl_tools::slTime2Ros(sens_data.imu.timestamp);
    ts_baro = sl_tools::slTime2Ros(sens_data.barometer.timestamp);
    ts_mag = sl_tools::slTime2Ros(sens_data.magnetometer.timestamp);
  }
  // <---- Grab data and setup timestamps

  // ----> Check for duplicated data
  bool new_imu_data = ts_imu != mLastTs_imu;
  double dT = ts_imu.seconds() - mLastTs_imu.seconds();

  bool new_baro_data = ts_baro != mLastTs_baro;
  mLastTs_baro = ts_baro;
  bool new_mag_data = ts_mag != mLastTs_mag;
  mLastTs_mag = ts_mag;

  if (!new_imu_data && !new_baro_data && !new_mag_data) {
    DEBUG_STREAM_SENS("No new sensors data");
    return TIMEZERO_ROS;
  }

  if (mSimMode) {
    new_baro_data = false;
    new_mag_data = false;
  }
  // <---- Check for duplicated data

  mLastTs_imu = ts_imu;

  DEBUG_STREAM_SENS(
    "SENSOR LAST PERIOD: " << dT << " sec @" << 1. / dT
                           << " Hz");

  // ----> Sensors freq for diagnostic
  if (new_imu_data) {
    double mean = mImuPeriodMean_sec->addValue(mImuFreqTimer.toc());
    mImuFreqTimer.tic();

    DEBUG_STREAM_SENS("IMU MEAN freq: " << 1. / mean);
  }

  if (new_baro_data) {
    double mean = mBaroPeriodMean_sec->addValue(mBaroFreqTimer.toc());
    mBaroFreqTimer.tic();
    DEBUG_STREAM_SENS("Barometer freq: " << 1. / mean);
  }

  if (new_mag_data) {
    double mean = mMagPeriodMean_sec->addValue(mMagFreqTimer.toc());
    mMagFreqTimer.tic();

    DEBUG_STREAM_SENS("Magnetometer freq: " << 1. / mean);
  }
  // <---- Sensors freq for diagnostic

  // ----> Sensors data publishing
  if (new_imu_data) {
    publishImuFrameAndTopic();

    if (imu_SubNumber > 0) {
      mImuPublishing = true;

      imuMsgPtr imuMsg = std::make_unique<sensor_msgs::msg::Imu>();

      imuMsg->header.stamp = ts_imu;
      imuMsg->header.frame_id = mImuFrameId;

      imuMsg->orientation.x = sens_data.imu.pose.getOrientation()[0];
      imuMsg->orientation.y = sens_data.imu.pose.getOrientation()[1];
      imuMsg->orientation.z = sens_data.imu.pose.getOrientation()[2];
      imuMsg->orientation.w = sens_data.imu.pose.getOrientation()[3];

      imuMsg->angular_velocity.x = sens_data.imu.angular_velocity[0] * DEG2RAD;
      imuMsg->angular_velocity.y = sens_data.imu.angular_velocity[1] * DEG2RAD;
      imuMsg->angular_velocity.z = sens_data.imu.angular_velocity[2] * DEG2RAD;

      imuMsg->linear_acceleration.x = sens_data.imu.linear_acceleration[0];
      imuMsg->linear_acceleration.y = sens_data.imu.linear_acceleration[1];
      imuMsg->linear_acceleration.z = sens_data.imu.linear_acceleration[2];

      // ----> Covariances copy
      // Note: memcpy not allowed because ROS2 uses double and ZED SDK uses
      // float
      for (int i = 0; i < 3; ++i) {
        int r = 0;

        if (i == 0) {
          r = 0;
        } else if (i == 1) {
          r = 1;
        } else {
          r = 2;
        }

        imuMsg->orientation_covariance[i * 3 + 0] =
          sens_data.imu.pose_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
        imuMsg->orientation_covariance[i * 3 + 1] =
          sens_data.imu.pose_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
        imuMsg->orientation_covariance[i * 3 + 2] =
          sens_data.imu.pose_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;

        imuMsg->linear_acceleration_covariance[i * 3 + 0] =
          sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
        imuMsg->linear_acceleration_covariance[i * 3 + 1] =
          sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
        imuMsg->linear_acceleration_covariance[i * 3 + 2] =
          sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];

        imuMsg->angular_velocity_covariance[i * 3 + 0] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD *
          DEG2RAD;
        imuMsg->angular_velocity_covariance[i * 3 + 1] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD *
          DEG2RAD;
        imuMsg->angular_velocity_covariance[i * 3 + 2] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD *
          DEG2RAD;
      }
      // <---- Covariances copy

      DEBUG_STREAM_SENS("Publishing IMU message");
      try {
        mPubImu->publish(std::move(imuMsg));
      } catch (std::system_error & e) {
        DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
      } catch (...) {
        DEBUG_STREAM_COMM("Message publishing generic ecception: ");
      }
    } else {
      mImuPublishing = false;
    }

    if (imu_RawSubNumber > 0) {
      mImuPublishing = true;

      imuMsgPtr imuRawMsg = std::make_unique<sensor_msgs::msg::Imu>();

      imuRawMsg->header.stamp = ts_imu;
      imuRawMsg->header.frame_id = mImuFrameId;

      imuRawMsg->angular_velocity.x =
        sens_data.imu.angular_velocity_uncalibrated[0] * DEG2RAD;
      imuRawMsg->angular_velocity.y =
        sens_data.imu.angular_velocity_uncalibrated[1] * DEG2RAD;
      imuRawMsg->angular_velocity.z =
        sens_data.imu.angular_velocity_uncalibrated[2] * DEG2RAD;

      imuRawMsg->linear_acceleration.x = sens_data.imu.linear_acceleration_uncalibrated[0];
      imuRawMsg->linear_acceleration.y = sens_data.imu.linear_acceleration_uncalibrated[1];
      imuRawMsg->linear_acceleration.z = sens_data.imu.linear_acceleration_uncalibrated[2];

      // ----> Covariances copy
      // Note: memcpy not allowed because ROS2 uses double and ZED SDK uses
      // float
      for (int i = 0; i < 3; ++i) {
        int r = 0;

        if (i == 0) {
          r = 0;
        } else if (i == 1) {
          r = 1;
        } else {
          r = 2;
        }

        imuRawMsg->linear_acceleration_covariance[i * 3 + 0] =
          sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
        imuRawMsg->linear_acceleration_covariance[i * 3 + 1] =
          sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
        imuRawMsg->linear_acceleration_covariance[i * 3 + 2] =
          sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];

        imuRawMsg->angular_velocity_covariance[i * 3 + 0] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD *
          DEG2RAD;
        imuRawMsg->angular_velocity_covariance[i * 3 + 1] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD *
          DEG2RAD;
        imuRawMsg->angular_velocity_covariance[i * 3 + 2] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD *
          DEG2RAD;
      }
      // <---- Covariances copy

      DEBUG_STREAM_SENS("Publishing IMU RAW message");
      try {
        mPubImuRaw->publish(std::move(imuRawMsg));
      } catch (std::system_error & e) {
        DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
      } catch (...) {
        DEBUG_STREAM_COMM("Message publishing generic ecception: ");
      }
    }
  }

  if (sens_data.barometer.is_available && new_baro_data) {
    if (pressSubNumber > 0) {
      mBaroPublishing = true;

      pressMsgPtr pressMsg =
        std::make_unique<sensor_msgs::msg::FluidPressure>();

      pressMsg->header.stamp = ts_baro;
      pressMsg->header.frame_id = mBaroFrameId;
      pressMsg->fluid_pressure =
        sens_data.barometer.pressure;    // Pascals -> see
      // https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/FluidPressure.msg
      pressMsg->variance = 1.0585e-2;

      DEBUG_STREAM_SENS("Publishing PRESS message");
      try {
        mPubPressure->publish(std::move(pressMsg));
      } catch (std::system_error & e) {
        DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
      } catch (...) {
        DEBUG_STREAM_COMM("Message publishing generic ecception: ");
      }
    } else {
      mBaroPublishing = false;
    }
  }

  if (sens_data.magnetometer.is_available && new_mag_data) {
    if (imu_MagSubNumber > 0) {
      mMagPublishing = true;

      magMsgPtr magMsg = std::make_unique<sensor_msgs::msg::MagneticField>();

      magMsg->header.stamp = ts_mag;
      magMsg->header.frame_id = mMagFrameId;
      magMsg->magnetic_field.x =
        sens_data.magnetometer.magnetic_field_calibrated.x * 1e-6;    // Tesla
      magMsg->magnetic_field.y =
        sens_data.magnetometer.magnetic_field_calibrated.y * 1e-6;    // Tesla
      magMsg->magnetic_field.z =
        sens_data.magnetometer.magnetic_field_calibrated.z * 1e-6;    // Tesla
      magMsg->magnetic_field_covariance[0] = 0.039e-6;
      magMsg->magnetic_field_covariance[1] = 0.0f;
      magMsg->magnetic_field_covariance[2] = 0.0f;
      magMsg->magnetic_field_covariance[3] = 0.0f;
      magMsg->magnetic_field_covariance[4] = 0.037e-6;
      magMsg->magnetic_field_covariance[5] = 0.0f;
      magMsg->magnetic_field_covariance[6] = 0.0f;
      magMsg->magnetic_field_covariance[7] = 0.0f;
      magMsg->magnetic_field_covariance[8] = 0.047e-6;

      DEBUG_STREAM_SENS("Publishing MAG message");
      try {
        mPubImuMag->publish(std::move(magMsg));
      } catch (std::system_error & e) {
        DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
      } catch (...) {
        DEBUG_STREAM_COMM("Message publishing generic ecception: ");
      }
    } else {
      mMagPublishing = false;
    }
  }
  // <---- Sensors data publishing

  return ts_imu;
}

void ZedCamera::publishTFs(rclcpp::Time t)
{
  // DEBUG_STREAM_PT("publishTFs");

  // RCLCPP_INFO_STREAM(get_logger(), "publishTFs - t type:" <<
  // t.get_clock_type());

  if (!mPosTrackingReady) {
    return;
  }

  if (t == TIMEZERO_ROS) {
    DEBUG_STREAM_PT("Time zero: not publishing TFs");
    return;
  }

  // Publish pose tf only if enabled
  if (mDepthMode != sl::DEPTH_MODE::NONE && mPublishTF) {
    publishOdomTF(t);  // publish the base Frame in odometry frame

    if (mPublishMapTF) {
      publishPoseTF(t);  // publish the odometry Frame in map frame
    }
  }
}

void ZedCamera::publishOdomTF(rclcpp::Time t)
{
  // DEBUG_STREAM_PT("publishOdomTF");

  // ----> Avoid duplicated TF publishing
  if (t == mLastTs_odom) {
    return;
  }
  mLastTs_odom = t;
  // <---- Avoid duplicated TF publishing

  if (!mSensor2BaseTransfValid) {
    getSens2BaseTransform();
  }

  if (!mSensor2CameraTransfValid) {
    getSens2CameraTransform();
  }

  if (!mCamera2BaseTransfValid) {
    getCamera2BaseTransform();
  }

  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = t + rclcpp::Duration(0, mTfOffset * 1e9);

  // RCLCPP_INFO_STREAM(get_logger(), "Odom TS: " <<
  // transformStamped.header.stamp);

  transformStamped.header.frame_id = mOdomFrameId;
  transformStamped.child_frame_id = mBaseFrameId;
  // conversion from Tranform to message
  tf2::Vector3 translation = mOdom2BaseTransf.getOrigin();
  tf2::Quaternion quat = mOdom2BaseTransf.getRotation();
  transformStamped.transform.translation.x = translation.x();
  transformStamped.transform.translation.y = translation.y();
  transformStamped.transform.translation.z = translation.z();
  transformStamped.transform.rotation.x = quat.x();
  transformStamped.transform.rotation.y = quat.y();
  transformStamped.transform.rotation.z = quat.z();
  transformStamped.transform.rotation.w = quat.w();

  // Publish transformation
  mTfBroadcaster->sendTransform(transformStamped);

  // Odom TF publishing diagnostic
  double elapsed_sec = mOdomFreqTimer.toc();
  mPubOdomTF_sec->addValue(elapsed_sec);
  mOdomFreqTimer.tic();
}

void ZedCamera::publishPoseTF(rclcpp::Time t)
{
  // DEBUG_STREAM_PT("publishPoseTF");

  // ----> Avoid duplicated TF publishing
  if (t == mLastTs_pose) {
    return;
  }
  mLastTs_pose = t;
  // <---- Avoid duplicated TF publishing

  if (!mSensor2BaseTransfValid) {
    getSens2BaseTransform();
  }

  if (!mSensor2CameraTransfValid) {
    getSens2CameraTransform();
  }

  if (!mCamera2BaseTransfValid) {
    getCamera2BaseTransform();
  }

  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = t + rclcpp::Duration(0, mTfOffset * 1e9);
  transformStamped.header.frame_id = mMapFrameId;
  transformStamped.child_frame_id = mOdomFrameId;
  // conversion from Tranform to message
  tf2::Vector3 translation = mMap2OdomTransf.getOrigin();
  tf2::Quaternion quat = mMap2OdomTransf.getRotation();
  transformStamped.transform.translation.x = translation.x();
  transformStamped.transform.translation.y = translation.y();
  transformStamped.transform.translation.z = translation.z();
  transformStamped.transform.rotation.x = quat.x();
  transformStamped.transform.rotation.y = quat.y();
  transformStamped.transform.rotation.z = quat.z();
  transformStamped.transform.rotation.w = quat.w();

  // Publish transformation
  mTfBroadcaster->sendTransform(transformStamped);

  // Pose TF publishing diagnostic
  double elapsed_sec = mPoseFreqTimer.toc();
  mPubPoseTF_sec->addValue(elapsed_sec);
  mPoseFreqTimer.tic();
}

void ZedCamera::threadFunc_pointcloudElab()
{
  DEBUG_STREAM_PC("Point Cloud thread started");

  // ----> Advanced thread settings
  DEBUG_STREAM_ADV("Point Cloud thread settings");
  if (mDebugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * Default Point Cloud thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par.sched_priority);
    }
  }

  if (mThreadSchedPolicy == "SCHED_OTHER") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_BATCH") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_BATCH, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_FIFO") {
    sched_param par;
    par.sched_priority = mThreadPrioPointCloud;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_RR") {
    sched_param par;
    par.sched_priority = mThreadPrioPointCloud;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(), " ! Failed to set thread params! - Policy not supported");
  }

  if (mDebugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * New Point Cloud thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par.sched_priority);
    }
  }
  // <---- Advanced thread settings

  mPcDataReady = false;

  std::unique_lock<std::mutex> lock(mPcMutex);

  while (1) {
    if (!rclcpp::ok()) {
      DEBUG_STREAM_PC("Ctrl+C received: stopping point cloud thread");
      break;
    }

    // DEBUG_STREAM_PC( "pointcloudThreadFunc -> mPcDataReady value:
    // %s", mPcDataReady ? "TRUE" : "FALSE");

    while (!mPcDataReady) {  // loop to avoid spurious wakeups
      if (mPcDataReadyCondVar.wait_for(lock, std::chrono::milliseconds(500)) ==
        std::cv_status::timeout)
      {
        // Check thread stopping
        if (!rclcpp::ok()) {
          DEBUG_STREAM_PC("Ctrl+C received: stopping point cloud thread");
          mThreadStop = true;
          break;
        }
        if (mThreadStop) {
          DEBUG_STREAM_PC(
            "threadFunc_pointcloudElab (2): Point Cloud thread stopped");
          break;
        } else {
          // DEBUG_STREAM_PC( "pointcloudThreadFunc -> WAIT FOR CLOUD
          // DATA");
          continue;
        }
      }
    }

    if (mThreadStop) {
      DEBUG_STREAM_PC(
        "threadFunc_pointcloudElab (1): Point Cloud thread stopped");
      break;
    }

    publishPointCloud();

    // ----> Check publishing frequency
    double pc_period_usec = 1e6 / mPcPubRate;

    double elapsed_usec = mPcPubFreqTimer.toc() * 1e6;

    DEBUG_STREAM_PC("threadFunc_pointcloudElab: elapsed_usec " << elapsed_usec);

    if (elapsed_usec < pc_period_usec) {
      int wait_usec = static_cast<int>(pc_period_usec - elapsed_usec);
      rclcpp::sleep_for(std::chrono::microseconds(wait_usec));
      DEBUG_STREAM_PC("threadFunc_pointcloudElab: wait_usec " << wait_usec);
    }

    mPcPubFreqTimer.tic();
    // <---- Check publishing frequency

    mPcDataReady = false;
    // DEBUG_STREAM_PC( "pointcloudThreadFunc -> mPcDataReady FALSE")
  }

  DEBUG_STREAM_PC("Pointcloud thread finished");
}

void ZedCamera::threadFunc_pubSensorsData()
{
  DEBUG_STREAM_SENS("Sensors thread started");

  // ----> Advanced thread settings
  DEBUG_STREAM_ADV("Sensors thread settings");
  if (mDebugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * Default Sensors thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par.sched_priority);
    }
  }

  if (mThreadSchedPolicy == "SCHED_OTHER") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_BATCH") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_BATCH, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_FIFO") {
    sched_param par;
    par.sched_priority = mThreadPrioSens;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_RR") {
    sched_param par;
    par.sched_priority = mThreadPrioSens;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(), " ! Failed to set thread params! - Policy not supported");
  }

  if (mDebugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * New Sensors thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par.sched_priority);
    }
  }
  // <---- Advanced thread settings

  while (1) {
    try {
      if (!rclcpp::ok()) {
        DEBUG_STREAM_SENS("Ctrl+C received: stopping sensors thread");
        mThreadStop = true;
        break;
      }
      if (mThreadStop) {
        DEBUG_STREAM_SENS(
          "threadFunc_pubSensorsData (2): Sensors thread stopped");
        break;
      }

      // std::lock_guard<std::mutex> lock(mCloseZedMutex);
      if (!mZed->isOpened()) {
        DEBUG_STREAM_SENS("threadFunc_pubSensorsData: the camera is not open");
        continue;
      }

      // RCLCPP_INFO_STREAM(get_logger(),
      // "threadFunc_pubSensorsData: Publishing Camera-IMU transform ");
      // publishImuFrameAndTopic();
      rclcpp::Time sens_ts = publishSensorsData();

      // RCLCPP_INFO_STREAM(get_logger(), "threadFunc_pubSensorsData - sens_ts
      // type:"
      // << sens_ts.get_clock_type());

      // Publish TF at the same frequency of IMU data, so they are always
      // synchronized
      /*if (sens_ts != TIMEZERO_ROS)
      {
        RCLCPP_INFO(get_logger(), "Publishing TF -> threadFunc_pubSensorsData");
        publishTFs(sens_ts);
      }*/

      // ----> Check publishing frequency
      double sens_period_usec = 1e6 / mSensPubRate;

      double elapsed_usec = mSensPubFreqTimer.toc() * 1e6;

      if (elapsed_usec < sens_period_usec) {
        rclcpp::sleep_for(
          std::chrono::microseconds(
            static_cast<int>(sens_period_usec - elapsed_usec)));
      }

      mSensPubFreqTimer.tic();
      // <---- Check publishing frequency
    } catch (...) {
      rcutils_reset_error();
      DEBUG_STREAM_COMM("threadFunc_pubSensorsData: Generic exception.");
      continue;
    }
  }

  DEBUG_STREAM_SENS("Sensors thread finished");
}

bool ZedCamera::areVideoDepthSubscribed()
{
  mRgbSubnumber = 0;
  mRgbRawSubnumber = 0;
  mRgbGraySubnumber = 0;
  mRgbGrayRawSubnumber = 0;
  mLeftSubnumber = 0;
  mLeftRawSubnumber = 0;
  mLeftGraySubnumber = 0;
  mLeftGrayRawSubnumber = 0;
  mRightSubnumber = 0;
  mRightRawSubnumber = 0;
  mRightGraySubnumber = 0;
  mRightGrayRawSubnumber = 0;
  mStereoSubnumber = 0;
  mStereoRawSubnumber = 0;
  mDepthSubnumber = 0;
  mConfMapSubnumber = 0;
  mDisparitySubnumber = 0;
  mDepthInfoSubnumber = 0;

  try {
    mRgbSubnumber = mPubRgb.getNumSubscribers();
    mRgbRawSubnumber = mPubRawRgb.getNumSubscribers();
    mRgbGraySubnumber = mPubRgbGray.getNumSubscribers();
    mRgbGrayRawSubnumber = mPubRawRgbGray.getNumSubscribers();
    mLeftSubnumber = mPubLeft.getNumSubscribers();
    mLeftRawSubnumber = mPubRawLeft.getNumSubscribers();
    mLeftGraySubnumber = mPubLeftGray.getNumSubscribers();
    mLeftGrayRawSubnumber = mPubRawLeftGray.getNumSubscribers();
    mRightSubnumber = mPubRight.getNumSubscribers();
    mRightRawSubnumber = mPubRawRight.getNumSubscribers();
    mRightGraySubnumber = mPubRightGray.getNumSubscribers();
    mRightGrayRawSubnumber = mPubRawRightGray.getNumSubscribers();
    mStereoSubnumber = mPubStereo.getNumSubscribers();
    mStereoRawSubnumber = mPubRawStereo.getNumSubscribers();

    if (!mDepthDisabled) {
      mDepthSubnumber = mPubDepth.getNumSubscribers();
      mDepthInfoSubnumber = count_subscribers(mPubDepthInfo->get_topic_name());
      mConfMapSubnumber = count_subscribers(mPubConfMap->get_topic_name());
      mDisparitySubnumber = count_subscribers(mPubDisparity->get_topic_name());
    }
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_VD("publishImages: Exception while counting subscribers");
    return 0;
  }

  return (mRgbSubnumber + mRgbRawSubnumber + mRgbGraySubnumber +
         mRgbGrayRawSubnumber + mLeftSubnumber + mLeftRawSubnumber +
         mLeftGraySubnumber + mLeftGrayRawSubnumber + mRightSubnumber +
         mRightRawSubnumber + mRightGraySubnumber + mRightGrayRawSubnumber +
         mStereoSubnumber + mStereoRawSubnumber + mDepthSubnumber +
         mConfMapSubnumber + mDisparitySubnumber + mDepthInfoSubnumber) > 0;
}

void ZedCamera::retrieveVideoDepth()
{
  mRgbSubscribed = false;
  bool retrieved = false;

  // ----> Retrieve all required data
  DEBUG_STREAM_VD("Retrieving Video Data");
  if (mRgbSubnumber + mLeftSubnumber + mStereoSubnumber > 0) {
    retrieved |=
      sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(mMatLeft, sl::VIEW::LEFT, sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatLeft.timestamp;
    mRgbSubscribed = true;
  }
  if (mRgbRawSubnumber + mLeftRawSubnumber + mStereoRawSubnumber > 0) {
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatLeftRaw, sl::VIEW::LEFT_UNRECTIFIED,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatLeftRaw.timestamp;
  }
  if (mRightSubnumber + mStereoSubnumber > 0) {
    retrieved |=
      sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(mMatRight, sl::VIEW::RIGHT, sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatRight.timestamp;
  }
  if (mRightRawSubnumber + mStereoRawSubnumber > 0) {
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatRightRaw, sl::VIEW::RIGHT_UNRECTIFIED,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatRightRaw.timestamp;
  }
  if (mRgbGraySubnumber + mLeftGraySubnumber > 0) {
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatLeftGray, sl::VIEW::LEFT_GRAY,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatLeftGray.timestamp;
  }
  if (mRgbGrayRawSubnumber + mLeftGrayRawSubnumber > 0) {
    retrieved |=
      sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatLeftRawGray, sl::VIEW::LEFT_UNRECTIFIED_GRAY,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatLeftRawGray.timestamp;
  }
  if (mRightGraySubnumber > 0) {
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatRightGray, sl::VIEW::RIGHT_GRAY,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatRightGray.timestamp;
  }
  if (mRightGrayRawSubnumber > 0) {
    retrieved |=
      sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatRightRawGray, sl::VIEW::RIGHT_UNRECTIFIED_GRAY,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatRightRawGray.timestamp;
  }
  if (retrieved) {
    DEBUG_STREAM_VD("Video Data retrieved");
  }

  retrieved = false;
  DEBUG_STREAM_VD("Retrieving Depth Data");
  if (mDepthSubnumber > 0 || mDepthInfoSubnumber > 0) {
    DEBUG_STREAM_VD("Retrieving Depth");
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveMeasure(
      mMatDepth, sl::MEASURE::DEPTH,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatDepth.timestamp;
  }
  if (mDisparitySubnumber > 0) {
    DEBUG_STREAM_VD("Retrieving Disparity");
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveMeasure(
      mMatDisp, sl::MEASURE::DISPARITY,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatDisp.timestamp;
  }
  if (mConfMapSubnumber > 0) {
    DEBUG_STREAM_VD("Retrieving Confidence");
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveMeasure(
      mMatConf, sl::MEASURE::CONFIDENCE,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatConf.timestamp;
  }
  if (mDepthInfoSubnumber > 0) {
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->getCurrentMinMaxDepth(mMinDepth, mMaxDepth);
    mSdkGrabTS = mMatConf.timestamp;
  }
  if (retrieved) {
    DEBUG_STREAM_VD("Depth Data retrieved");
  }
  // <---- Retrieve all required data
}

void ZedCamera::publishVideoDepth(rclcpp::Time & out_pub_ts)
{
  DEBUG_VD("*** Publish Video and Depth topics *** ");
  sl_tools::StopWatch vdElabTimer(get_clock());

  // ----> Check RGB/Depth sync
  sl::Timestamp ts_rgb = 0;
  sl::Timestamp ts_depth = 0;

  if (mRgbSubscribed && (mDepthSubnumber > 0 || mDepthInfoSubnumber > 0)) {
    ts_rgb = mMatLeft.timestamp;
    ts_depth = mMatDepth.timestamp;

    if (mRgbSubscribed &&
      (ts_rgb.data_ns != 0 && (ts_depth.data_ns != ts_rgb.data_ns)))
    {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "!!!!! DEPTH/RGB ASYNC !!!!! - Delta: "
          << 1e-9 * static_cast<double>(ts_depth - ts_rgb)
          << " sec");
      RCLCPP_WARN(
        get_logger(),
        "NOTE: this should never happen, please contact the node "
        "maintainer in case you get this warning.");
    }
  }
  // <---- Check RGB/Depth sync

  // Start processing timer for diagnostic
  vdElabTimer.tic();

  // ----> Check if a grab has been done before publishing the same images
  if (mSdkGrabTS.data_ns == mLastTs_grab.data_ns) {
    out_pub_ts = TIMEZERO_ROS;
    // Data not updated by a grab calling in the grab thread
    DEBUG_STREAM_VD("publishVideoDepth: ignoring not update data");
    return;
  }

  if (mLastTs_grab.data_ns != 0) {
    double period_sec =
      static_cast<double>(mSdkGrabTS.data_ns - mLastTs_grab.data_ns) / 1e9;
    DEBUG_STREAM_VD(
      "VIDEO/DEPTH PUB LAST PERIOD: "
        << period_sec << " sec @" << 1. / period_sec << " Hz");

    mVideoDepthPeriodMean_sec->addValue(period_sec);
    DEBUG_STREAM_VD(
      "VIDEO/DEPTH PUB MEAN PERIOD: "
        << mVideoDepthPeriodMean_sec->getAvg() << " sec @"
        << 1. / mVideoDepthPeriodMean_sec->getAvg() << " Hz");
  }
  mLastTs_grab = mSdkGrabTS;
  // <---- Check if a grab has been done before publishing the same images

  rclcpp::Time timeStamp;
  if (mSvoMode) {
    timeStamp = mFrameTimestamp;
  } else if (mSimMode) {
    if (mUseSimTime) {
      timeStamp = get_clock()->now();
    } else {
      timeStamp =
        sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
    }
  } else {
    timeStamp = sl_tools::slTime2Ros(mSdkGrabTS, get_clock()->get_clock_type());
  }

  out_pub_ts = timeStamp;

  // ----> Publish the left=rgb image if someone has subscribed to
  if (mLeftSubnumber > 0) {
    DEBUG_STREAM_VD("mLeftSubnumber: " << mLeftSubnumber);
    publishImageWithInfo(
      mMatLeft, mPubLeft, mLeftCamInfoMsg,
      mLeftCamOptFrameId, out_pub_ts);
  }

  if (mRgbSubnumber > 0) {
    DEBUG_STREAM_VD("mRgbSubnumber: " << mRgbSubnumber);
    publishImageWithInfo(
      mMatLeft, mPubRgb, mRgbCamInfoMsg, mDepthOptFrameId,
      out_pub_ts);
  }
  // <---- Publish the left=rgb image if someone has subscribed to

  // ----> Publish the left_raw=rgb_raw image if someone has subscribed to
  if (mLeftRawSubnumber > 0) {
    DEBUG_STREAM_VD("mLeftRawSubnumber: " << mLeftRawSubnumber);
    publishImageWithInfo(
      mMatLeftRaw, mPubRawLeft, mLeftCamInfoRawMsg,
      mLeftCamOptFrameId, out_pub_ts);
  }
  if (mRgbRawSubnumber > 0) {
    DEBUG_STREAM_VD("mRgbRawSubnumber: " << mRgbRawSubnumber);
    publishImageWithInfo(
      mMatLeftRaw, mPubRawRgb, mRgbCamInfoRawMsg,
      mDepthOptFrameId, out_pub_ts);
  }
  // <---- Publish the left_raw=rgb_raw image if someone has subscribed to

  // ----> Publish the left_gray=rgb_gray image if someone has subscribed to
  if (mLeftGraySubnumber > 0) {
    DEBUG_STREAM_VD("mLeftGraySubnumber: " << mLeftGraySubnumber);
    publishImageWithInfo(
      mMatLeftGray, mPubLeftGray, mLeftCamInfoMsg,
      mLeftCamOptFrameId, out_pub_ts);
  }
  if (mRgbGraySubnumber > 0) {
    DEBUG_STREAM_VD("mRgbGraySubnumber: " << mRgbGraySubnumber);
    publishImageWithInfo(
      mMatLeftGray, mPubRgbGray, mRgbCamInfoMsg,
      mDepthOptFrameId, out_pub_ts);
  }
  // <---- Publish the left_raw=rgb_raw image if someone has subscribed to

  // ----> Publish the left_raw_gray=rgb_raw_gray image if someone has
  // subscribed to
  if (mLeftGrayRawSubnumber > 0) {
    DEBUG_STREAM_VD("mLeftGrayRawSubnumber: " << mLeftGrayRawSubnumber);
    publishImageWithInfo(
      mMatLeftRawGray, mPubRawLeftGray, mLeftCamInfoRawMsg,
      mLeftCamOptFrameId, out_pub_ts);
  }
  if (mRgbGrayRawSubnumber > 0) {
    DEBUG_STREAM_VD("mRgbGrayRawSubnumber: " << mRgbGrayRawSubnumber);
    publishImageWithInfo(
      mMatLeftRawGray, mPubRawRgbGray, mRgbCamInfoRawMsg,
      mDepthOptFrameId, out_pub_ts);
  }
  // ----> Publish the left_raw_gray=rgb_raw_gray image if someone has
  // subscribed to

  // ----> Publish the right image if someone has subscribed to
  if (mRightSubnumber > 0) {
    DEBUG_STREAM_VD("mRightSubnumber: " << mRightSubnumber);
    publishImageWithInfo(
      mMatRight, mPubRight, mRightCamInfoMsg,
      mRightCamOptFrameId, out_pub_ts);
  }
  // <---- Publish the right image if someone has subscribed to

  // ----> Publish the right raw image if someone has subscribed to
  if (mRightRawSubnumber > 0) {
    DEBUG_STREAM_VD("mRightRawSubnumber: " << mRightRawSubnumber);
    publishImageWithInfo(
      mMatRightRaw, mPubRawRight, mRightCamInfoRawMsg,
      mRightCamOptFrameId, out_pub_ts);
  }
  // <---- Publish the right raw image if someone has subscribed to

  // ----> Publish the right gray image if someone has subscribed to
  if (mRightGraySubnumber > 0) {
    DEBUG_STREAM_VD("mRightGraySubnumber: " << mRightGraySubnumber);
    publishImageWithInfo(
      mMatRightGray, mPubRightGray, mRightCamInfoMsg,
      mRightCamOptFrameId, out_pub_ts);
  }
  // <---- Publish the right gray image if someone has subscribed to

  // ----> Publish the right raw gray image if someone has subscribed to
  if (mRightGrayRawSubnumber > 0) {
    DEBUG_STREAM_VD("mRightGrayRawSubnumber: " << mRightGrayRawSubnumber);
    publishImageWithInfo(
      mMatRightRawGray, mPubRawRightGray,
      mRightCamInfoRawMsg, mRightCamOptFrameId, out_pub_ts);
  }
  // <---- Publish the right raw gray image if someone has subscribed to

  // ----> Publish the side-by-side image if someone has subscribed to
  if (mStereoSubnumber > 0) {
    DEBUG_STREAM_VD("mStereoSubnumber: " << mStereoSubnumber);
    auto combined = sl_tools::imagesToROSmsg(
      mMatLeft, mMatRight,
      mCameraFrameId, out_pub_ts);
    DEBUG_STREAM_VD("Publishing SIDE-BY-SIDE message");
    try {
      mPubStereo.publish(std::move(combined));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
  // <---- Publish the side-by-side image if someone has subscribed to

  // ----> Publish the side-by-side image if someone has subscribed to
  if (mStereoRawSubnumber > 0) {
    DEBUG_STREAM_VD("mStereoRawSubnumber: " << mStereoRawSubnumber);
    auto combined = sl_tools::imagesToROSmsg(
      mMatLeftRaw, mMatRightRaw,
      mCameraFrameId, out_pub_ts);
    DEBUG_STREAM_VD("Publishing SIDE-BY-SIDE RAW message");
    try {
      mPubRawStereo.publish(std::move(combined));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
  // <---- Publish the side-by-side image if someone has subscribed to

  // ---->  Publish the depth image if someone has subscribed to
  if (mDepthSubnumber > 0) {
    publishDepthMapWithInfo(mMatDepth, out_pub_ts);
  }
  // <----  Publish the depth image if someone has subscribed to

  // ---->  Publish the confidence image and map if someone has subscribed to
  if (mConfMapSubnumber > 0) {
    DEBUG_STREAM_VD("Publishing CONF MAP message");
    try {
      mPubConfMap->publish(
        *sl_tools::imageToROSmsg(mMatConf, mDepthOptFrameId, out_pub_ts));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
  // <----  Publish the confidence image and map if someone has subscribed to

  // ----> Publish the disparity image if someone has subscribed to
  if (mDisparitySubnumber > 0) {
    publishDisparity(mMatDisp, out_pub_ts);
  }
  // <---- Publish the disparity image if someone has subscribed to

  // ----> Publish the depth info if someone has subscribed to
  if (mDepthInfoSubnumber > 0) {
    depthInfoMsgPtr depthInfoMsg =
      std::make_unique<zed_interfaces::msg::DepthInfoStamped>();
    depthInfoMsg->header.stamp = timeStamp;
    depthInfoMsg->header.frame_id = mDepthOptFrameId;
    depthInfoMsg->min_depth = mMinDepth;
    depthInfoMsg->max_depth = mMaxDepth;

    DEBUG_STREAM_VD("Publishing DEPTH INFO message");
    try {
      mPubDepthInfo->publish(std::move(depthInfoMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
  // <---- Publish the depth info if someone has subscribed to

  // Diagnostic statistic
  mVideoDepthElabMean_sec->addValue(vdElabTimer.toc());

  // ----> Check publishing frequency
  double vd_period_usec = 1e6 / mPubFrameRate;

  double elapsed_usec = mVdPubFreqTimer.toc() * 1e6;

  if (elapsed_usec < vd_period_usec) {
    rclcpp::sleep_for(
      std::chrono::microseconds(
        static_cast<int>(vd_period_usec - elapsed_usec)));
  }

  mVdPubFreqTimer.tic();
  // <---- Check publishing frequency

  DEBUG_VD("*** Video and Depth topics published *** ");
}

void ZedCamera::publishImageWithInfo(
  sl::Mat & img,
  image_transport::CameraPublisher & pubImg,
  camInfoMsgPtr & camInfoMsg,
  std::string imgFrameId, rclcpp::Time t)
{
  auto image = sl_tools::imageToROSmsg(img, imgFrameId, t);
  camInfoMsg->header.stamp = t;
  DEBUG_STREAM_VD("Publishing IMAGE message: " << t.nanoseconds() << " nsec");
  try {
    pubImg.publish(std::move(image), camInfoMsg);
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
}

void ZedCamera::processOdometry()
{
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  if (!mSensor2BaseTransfValid) {
    getSens2BaseTransform();
  }

  if (!mSensor2CameraTransfValid) {
    getSens2CameraTransform();
  }

  if (!mCamera2BaseTransfValid) {
    getCamera2BaseTransform();
  }

  sl::Pose deltaOdom;

  mPosTrackingStatus = mZed->getPositionalTrackingStatus();

  if (mResetOdomFromSrv || (mResetOdomWhenLoopClosure &&
    mPosTrackingStatus.spatial_memory_status ==
    sl::SPATIAL_MEMORY_STATUS::LOOP_CLOSED))
  {
    if (mPosTrackingStatus.spatial_memory_status ==
      sl::SPATIAL_MEMORY_STATUS::LOOP_CLOSED)
    {
      RCLCPP_INFO_STREAM(
        get_logger(),
        "*** Odometry reset for LOOP CLOSURE event ***");
    }

    // Propagate Odom transform in time
    mOdom2BaseTransf.setIdentity();
    mOdomPath.clear();

    mResetOdomFromSrv = false;
  } else {
    if (!mGnssFusionEnabled) {
      mZed->getPosition(deltaOdom, sl::REFERENCE_FRAME::CAMERA);
    } else {
      mFusion.getPosition(
        deltaOdom, sl::REFERENCE_FRAME::CAMERA /*,mCamUuid*/,
        sl::CameraIdentifier(), sl::POSITION_TYPE::FUSION);
    }

    DEBUG_STREAM_PT(
      "MAP -> Odometry Status: "
        << sl::toString(mPosTrackingStatus.odometry_status).c_str());

    DEBUG_PT(
      "delta ODOM %s- [%s]:\n%s", mDebugGnss ? "(`sl::Fusion`) " : "",
      sl::toString(mPosTrackingStatus.odometry_status).c_str(),
      deltaOdom.pose_data.getInfos().c_str());

    if (mDebugGnss) {
      sl::Pose camera_delta_odom;
      auto status =
        mZed->getPosition(camera_delta_odom, sl::REFERENCE_FRAME::CAMERA);

      DEBUG_PT(
        "delta ODOM (`sl::Camera`) [%s]:\n%s",
        sl::toString(status).c_str(),
        camera_delta_odom.pose_data.getInfos().c_str());
    }

    if (mPosTrackingStatus.odometry_status == sl::ODOMETRY_STATUS::OK) {
      sl::Translation translation = deltaOdom.getTranslation();
      sl::Orientation quat = deltaOdom.getOrientation();

      // Transform ZED delta odom pose in TF2 Transformation
      tf2::Transform deltaOdomTf;
      deltaOdomTf.setOrigin(
        tf2::Vector3(translation(0), translation(1), translation(2)));
      // w at the end in the constructor
      deltaOdomTf.setRotation(
        tf2::Quaternion(quat(0), quat(1), quat(2), quat(3)));

      // delta odom from sensor to base frame
      tf2::Transform deltaOdomTf_base =
        mSensor2BaseTransf.inverse() * deltaOdomTf * mSensor2BaseTransf;

      // Propagate Odom transform in time
      mOdom2BaseTransf = mOdom2BaseTransf * deltaOdomTf_base;

      if (mTwoDMode) {
        tf2::Vector3 tr_2d = mOdom2BaseTransf.getOrigin();
        tr_2d.setZ(mFixedZValue);
        mOdom2BaseTransf.setOrigin(tr_2d);

        double roll, pitch, yaw;
        tf2::Matrix3x3(mOdom2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

        tf2::Quaternion quat_2d;
        quat_2d.setRPY(0.0, 0.0, yaw);

        mOdom2BaseTransf.setRotation(quat_2d);
      }
      mPosTrackingReady = true;
    } else if (mFloorAlignment) {
      DEBUG_STREAM_THROTTLE_PT(
        5000.0,
        "Odometry will be published as soon as the floor as "
        "been detected for the first time");
    }
  }

  if (mDebugPosTracking) {
    double roll, pitch, yaw;
    tf2::Matrix3x3(mOdom2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

    DEBUG_PT(
      "+++ Odometry [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
      mOdomFrameId.c_str(), mBaseFrameId.c_str(),
      mOdom2BaseTransf.getOrigin().x(), mOdom2BaseTransf.getOrigin().y(),
      mOdom2BaseTransf.getOrigin().z(), roll * RAD2DEG, pitch * RAD2DEG,
      yaw * RAD2DEG);
  }

  // Publish odometry message
  publishOdom(mOdom2BaseTransf, deltaOdom, mFrameTimestamp);
}

void ZedCamera::publishOdom(
  tf2::Transform & odom2baseTransf, sl::Pose & slPose,
  rclcpp::Time t)
{
  size_t odomSub = 0;

  try {
    odomSub = count_subscribers(mOdomTopic);  // mPubOdom subscribers
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_PT("publishPose: Exception while counting subscribers");
    return;
  }

  if (odomSub) {
    odomMsgPtr odomMsg = std::make_unique<nav_msgs::msg::Odometry>();

    odomMsg->header.stamp = t;
    odomMsg->header.frame_id = mOdomFrameId;  // frame
    odomMsg->child_frame_id = mBaseFrameId;   // camera_frame

    // Add all value in odometry message
    odomMsg->pose.pose.position.x = odom2baseTransf.getOrigin().x();
    odomMsg->pose.pose.position.y = odom2baseTransf.getOrigin().y();
    odomMsg->pose.pose.position.z = odom2baseTransf.getOrigin().z();
    odomMsg->pose.pose.orientation.x = odom2baseTransf.getRotation().x();
    odomMsg->pose.pose.orientation.y = odom2baseTransf.getRotation().y();
    odomMsg->pose.pose.orientation.z = odom2baseTransf.getRotation().z();
    odomMsg->pose.pose.orientation.w = odom2baseTransf.getRotation().w();

    // Odometry pose covariance
    for (size_t i = 0; i < odomMsg->pose.covariance.size(); i++) {
      odomMsg->pose.covariance[i] =
        static_cast<double>(slPose.pose_covariance[i]);

      if (mTwoDMode) {
        if (i == 14 || i == 21 || i == 28) {
          odomMsg->pose.covariance[i] = 1e-9;  // Very low covariance if 2D mode
        } else if ((i >= 2 && i <= 4) || (i >= 8 && i <= 10) ||
          (i >= 12 && i <= 13) || (i >= 15 && i <= 16) ||
          (i >= 18 && i <= 20) || (i == 22) || (i >= 24 && i <= 27))
        {
          odomMsg->pose.covariance[i] = 0.0;
        }
      }
    }

    // Publish odometry message
    DEBUG_STREAM_PT("Publishing ODOM message");
    try {
      mPubOdom->publish(std::move(odomMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
}

void ZedCamera::processPose()
{
  if (!mSensor2BaseTransfValid) {
    getSens2BaseTransform();
  }

  if (!mSensor2CameraTransfValid) {
    getSens2CameraTransform();
  }

  if (!mCamera2BaseTransfValid) {
    getCamera2BaseTransform();
  }

  if (!mGnssFusionEnabled && mFusionStatus != sl::FUSION_ERROR_CODE::SUCCESS) {
    mZed->getPosition(mLastZedPose, sl::REFERENCE_FRAME::WORLD);
  } else {
    mFusion.getPosition(
      mLastZedPose, sl::REFERENCE_FRAME::WORLD /*,mCamUuid*/,
      sl::CameraIdentifier(), sl::POSITION_TYPE::FUSION);
  }

  publishPoseStatus();
  publishGnssPoseStatus();

  sl::Translation translation = mLastZedPose.getTranslation();
  sl::Orientation quat = mLastZedPose.getOrientation();

  if (quat.sum() == 0) {
    return;
  }

  DEBUG_STREAM_PT(
    "MAP -> Tracking Status: "
      << sl::toString(mPosTrackingStatus.spatial_memory_status).c_str());

  DEBUG_PT(
    "Sensor POSE %s- [%s -> %s]:\n%s}",
    mDebugGnss ? "(`sl::Fusion`) " : "", mLeftCamFrameId.c_str(),
    mMapFrameId.c_str(), mLastZedPose.pose_data.getInfos().c_str());

  if (mDebugGnss) {
    sl::Pose camera_pose;
    mZed->getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD);

    DEBUG_PT(
      "Sensor POSE (`sl::Camera`) [%s -> %s]:\n%s",
      mLeftCamFrameId.c_str(), mMapFrameId.c_str(),
      camera_pose.pose_data.getInfos().c_str());
  }

  if (mPosTrackingStatus.odometry_status == sl::ODOMETRY_STATUS::OK) {
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(quat.ox, quat.oy, quat.oz, quat.ow))
    .getRPY(roll, pitch, yaw);

    tf2::Transform map_to_sens_transf;
    map_to_sens_transf.setOrigin(
      tf2::Vector3(translation(0), translation(1), translation(2)));
    map_to_sens_transf.setRotation(
      tf2::Quaternion(quat(0), quat(1), quat(2), quat(3)));

    mMap2BaseTransf =
      map_to_sens_transf * mSensor2BaseTransf;    // Base position in map frame

    if (mTwoDMode) {
      tf2::Vector3 tr_2d = mMap2BaseTransf.getOrigin();
      tr_2d.setZ(mFixedZValue);
      mMap2BaseTransf.setOrigin(tr_2d);

      tf2::Matrix3x3(mMap2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

      tf2::Quaternion quat_2d;
      quat_2d.setRPY(0.0, 0.0, yaw);

      mMap2BaseTransf.setRotation(quat_2d);
    }

    // double roll, pitch, yaw;
    tf2::Matrix3x3(mMap2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

    DEBUG_PT(
      "*** Base POSE [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
      mMapFrameId.c_str(), mBaseFrameId.c_str(),
      mMap2BaseTransf.getOrigin().x(), mMap2BaseTransf.getOrigin().y(),
      mMap2BaseTransf.getOrigin().z(), roll * RAD2DEG, pitch * RAD2DEG,
      yaw * RAD2DEG);

    // Transformation from map to odometry frame
    mMap2OdomTransf = mMap2BaseTransf * mOdom2BaseTransf.inverse();

    tf2::Matrix3x3(mMap2OdomTransf.getRotation()).getRPY(roll, pitch, yaw);

    DEBUG_PT(
      "+++ Diff [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
      mMapFrameId.c_str(), mOdomFrameId.c_str(),
      mMap2OdomTransf.getOrigin().x(), mMap2OdomTransf.getOrigin().y(),
      mMap2OdomTransf.getOrigin().z(), roll * RAD2DEG, pitch * RAD2DEG,
      yaw * RAD2DEG);


    // Publish Pose message
    publishPose();
    mPosTrackingReady = true;
  }
}

void ZedCamera::publishPoseStatus()
{
  size_t statusSub = 0;

  try {
    statusSub =
      count_subscribers(mPoseStatusTopic);    // mPubPoseStatus subscribers
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_PT("publishPose: Exception while counting subscribers");
    return;
  }

  if (statusSub > 0) {
    poseStatusMsgPtr msg =
      std::make_unique<zed_interfaces::msg::PosTrackStatus>();
    msg->odometry_status = static_cast<uint8_t>(mPosTrackingStatus.odometry_status);
    msg->spatial_memory_status = static_cast<uint8_t>(mPosTrackingStatus.spatial_memory_status);

    try {
      mPubPoseStatus->publish(std::move(msg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
}

void ZedCamera::publishGnssPoseStatus()
{
  size_t statusSub = 0;

  try {
    statusSub = count_subscribers(
      mGnssPoseStatusTopic);    // mPubGnssPoseStatus subscribers
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_PT("publishPose: Exception while counting subscribers");
    return;
  }

  if (statusSub > 0) {
    gnssFusionStatusMsgPtr msg = std::make_unique<zed_interfaces::msg::GnssFusionStatus>();

    msg->gnss_fusion_status = static_cast<uint8_t>(mFusedPosTrackingStatus.gnss_fusion_status);

    try {
      mPubGnssPoseStatus->publish(std::move(msg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
}

void ZedCamera::publishGeoPoseStatus()
{
  size_t statusSub = 0;

  try {
    statusSub = count_subscribers(
      mGeoPoseStatusTopic);    // mPubGnssPoseStatus subscribers
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_PT("publishPose: Exception while counting subscribers");
    return;
  }

  if (statusSub > 0) {
    gnssFusionStatusMsgPtr msg =
      std::make_unique<zed_interfaces::msg::GnssFusionStatus>();

    msg->gnss_fusion_status =
      static_cast<uint8_t>(mFusedPosTrackingStatus.gnss_fusion_status);

    try {
      mPubGeoPoseStatus->publish(std::move(msg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
}

void ZedCamera::publishPose()
{
  size_t poseSub = 0;
  size_t poseCovSub = 0;

  try {
    poseSub = count_subscribers(mPoseTopic);        // mPubPose subscribers
    poseCovSub = count_subscribers(mPoseCovTopic);   // mPubPoseCov subscribers
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_PT("publishPose: Exception while counting subscribers");
    return;
  }

  tf2::Transform base_pose;
  base_pose.setIdentity();

  base_pose = mMap2BaseTransf;

  std_msgs::msg::Header header;
  header.stamp = mFrameTimestamp;
  header.frame_id = mMapFrameId;   // frame

  geometry_msgs::msg::Pose pose;

  // Add all value in Pose message
  pose.position.x = mMap2BaseTransf.getOrigin().x();
  pose.position.y = mMap2BaseTransf.getOrigin().y();
  pose.position.z = mMap2BaseTransf.getOrigin().z();
  pose.orientation.x = mMap2BaseTransf.getRotation().x();
  pose.orientation.y = mMap2BaseTransf.getRotation().y();
  pose.orientation.z = mMap2BaseTransf.getRotation().z();
  pose.orientation.w = mMap2BaseTransf.getRotation().w();

  if (poseSub > 0) {
    poseMsgPtr poseNoCov = std::make_unique<geometry_msgs::msg::PoseStamped>();

    poseNoCov->header = header;
    poseNoCov->pose = pose;

    // Publish pose stamped message
    DEBUG_STREAM_PT("Publishing POSE NO COV message");
    try {
      mPubPose->publish(std::move(poseNoCov));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }

  if (mPublishPoseCov) {
    if (poseCovSub > 0) {
      poseCovMsgPtr poseCov =
        std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();

      poseCov->header = header;
      poseCov->pose.pose = pose;

      // Odometry pose covariance if available

      for (size_t i = 0; i < poseCov->pose.covariance.size(); i++) {
        poseCov->pose.covariance[i] =
          static_cast<double>(mLastZedPose.pose_covariance[i]);

        if (mTwoDMode) {
          if ((i >= 2 && i <= 4) || (i >= 8 && i <= 10) ||
            (i >= 12 && i <= 29) || (i >= 32 && i <= 34))
          {
            poseCov->pose.covariance[i] =
              1e-9;    // Very low covariance if 2D mode
          }
        }
      }

      // Publish pose with covariance stamped message
      DEBUG_STREAM_PT("Publishing POSE COV message");
      try {
        mPubPoseCov->publish(std::move(poseCov));
      } catch (std::system_error & e) {
        DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
      } catch (...) {
        DEBUG_STREAM_COMM("Message publishing generic ecception: ");
      }
    }
  }
}

void ZedCamera::processGeoPose()
{
  if (!mGnssMsgReceived) {
    return;
  }

  if (!mGnssFusionEnabled) {
    return;
  }

  if (!mGnss2BaseTransfValid) {
    getGnss2BaseTransform();
  }

  // Update GeoPose
  mFusion.getGeoPose(mLastGeoPose);

  // ----> Update GeoPose status
  mFusedPosTrackingStatus = mFusion.getFusedPositionalTrackingStatus();
  publishGeoPoseStatus();

  if (mFusedPosTrackingStatus.gnss_fusion_status !=
    sl::GNSS_FUSION_STATUS::OK)
  {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "GNSS fusion status: " << sl::toString(
        mFusedPosTrackingStatus.gnss_fusion_status));
    mGnssInitGood = false;
  }
  // <---- Update GeoPose status

  // ----> Setup Lat/Long
  double altitude = mLastGeoPose.latlng_coordinates.getAltitude();
  if (mGnssZeroAltitude) {
    altitude = 0.0;
  }
  mLastLatLongPose.setCoordinates(
    mLastGeoPose.latlng_coordinates.getLatitude(),
    mLastGeoPose.latlng_coordinates.getLongitude(), altitude, true);

  mLastHeading = mLastGeoPose.heading;
  // <---- Setup Lat/Long

  // Get ECEF
  sl::GeoConverter::latlng2ecef(mLastLatLongPose, mLastEcefPose);

  // Get UTM
  sl::GeoConverter::latlng2utm(mLastLatLongPose, mLastUtmPose);

  DEBUG_GNSS("Good GNSS localization:");
  DEBUG_GNSS(
    " * ECEF: %.6f m, %.6f m, %.6f m", mLastEcefPose.x,
    mLastEcefPose.y, mLastEcefPose.z);
  DEBUG_GNSS(
    " * Lat. Long.: %.6f°, %.6f°,%.3f m", mLastLatLongPose.getLatitude(false),
    mLastLatLongPose.getLongitude(false), mLastLatLongPose.getAltitude());
  DEBUG_GNSS(" * Heading: %.3f°", mLastHeading * RAD2DEG);
  DEBUG_GNSS(
    " * UTM: %.5f m, %.5f m, %.5f°, %s", mLastUtmPose.easting,
    mLastUtmPose.northing, mLastUtmPose.gamma,
    mLastUtmPose.UTMZone.c_str());

  // If calibration is not good me must update the `utm -> map` transform
  if (!mGnssInitGood) {
    mInitEcefPose = mLastEcefPose;
    mInitUtmPose = mLastUtmPose;
    mInitLatLongPose = mLastLatLongPose;
    mInitHeading = mLastHeading;

    if (mFusedPosTrackingStatus.gnss_fusion_status == sl::GNSS_FUSION_STATUS::OK) {
      mGnssInitGood = true;
    } else {
      mGnssInitGood = false;
    }

    RCLCPP_INFO(get_logger(), "GNSS reference localization initialized");

    // ----> Create (static) transform UTM to MAP
    // Add only Heading to pose
    tf2::Quaternion pose_quat_yaw;
    pose_quat_yaw.setRPY(0.0, 0.0, mLastHeading);
    mLastHeadingQuat = pose_quat_yaw;

    // Set UTM transform
    // Get position from ZED SDK UTM pose
    mMap2UtmTransf.setOrigin(
      tf2::Vector3(
        mLastUtmPose.easting, mLastUtmPose.northing,
        mLastGeoPose.latlng_coordinates.getAltitude()));
    mMap2UtmTransf.setRotation(pose_quat_yaw);
    // ----> Create (static) transform UTM to MAP

    mMap2UtmTransfValid = true;

    if (!mUtmAsParent) {
      tf2::Transform map2utmInverse = mMap2UtmTransf.inverse();

      double roll, pitch, yaw;
      tf2::Matrix3x3(map2utmInverse.getRotation()).getRPY(roll, pitch, yaw);

      RCLCPP_INFO(
        get_logger(), " Static transform UTM to MAP [%s -> %s]",
        mUtmFrameId.c_str(), mMapFrameId.c_str());
      RCLCPP_INFO(
        get_logger(), "  * Translation: {%.3f,%.3f,%.3f}",
        map2utmInverse.getOrigin().x(),
        map2utmInverse.getOrigin().y(),
        map2utmInverse.getOrigin().z());
      RCLCPP_INFO(
        get_logger(), "  * Rotation: {%.3f,%.3f,%.3f}",
        roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
    } else {
      double roll, pitch, yaw;
      tf2::Matrix3x3(mMap2UtmTransf.getRotation()).getRPY(roll, pitch, yaw);

      RCLCPP_INFO(
        get_logger(), " Static transform MAP to UTM [%s -> %s]",
        mMapFrameId.c_str(), mUtmFrameId.c_str());
      RCLCPP_INFO(
        get_logger(), "  * Translation: {%.3f,%.3f,%.3f}",
        mMap2UtmTransf.getOrigin().x(),
        mMap2UtmTransf.getOrigin().y(),
        mMap2UtmTransf.getOrigin().z());
      RCLCPP_INFO(
        get_logger(), "  * Rotation: {%.3f,%.3f,%.3f}",
        roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
    }
  }

  if (mMap2UtmTransfValid && mPublishUtmTf) {
    // Publish the transform
    // Note: we cannot use a static TF publisher because IPC is enabled
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp =
      mFrameTimestamp + rclcpp::Duration(0, mTfOffset * 1e9);
    transformStamped.header.frame_id = mUtmAsParent ? mUtmFrameId : mMapFrameId;
    transformStamped.child_frame_id = mUtmAsParent ? mMapFrameId : mUtmFrameId;

    // conversion from Transform to message
    transformStamped.transform = mUtmAsParent ?
      (tf2::toMsg(mMap2UtmTransf)) :
      (tf2::toMsg(mMap2UtmTransf.inverse()));

    mTfBroadcaster->sendTransform(transformStamped);
  }

  publishGnssPose();
}

void ZedCamera::publishGnssPose()
{
  DEBUG_GNSS("*** publishGnssPose ***");

  size_t gnssSub = 0;
  size_t geoPoseSub = 0;
  size_t fusedFixSub = 0;
  size_t originFixSub = 0;

  try {
    gnssSub = count_subscribers(mGnssPoseTopic);
    geoPoseSub = count_subscribers(mGeoPoseTopic);
    fusedFixSub = count_subscribers(mFusedFixTopic);
    originFixSub = count_subscribers(mOriginFixTopic);
  } catch (...) {
    rcutils_reset_error();
    DEBUG_GNSS("publishGnssPose: Exception while counting subscribers");
    return;
  }

  if (gnssSub > 0) {
    odomMsgPtr msg = std::make_unique<nav_msgs::msg::Odometry>();

    msg->header.stamp = mFrameTimestamp;
    msg->header.frame_id = mMapFrameId;
    msg->child_frame_id = mBaseFrameId;

    // Add all value in odometry message
    msg->pose.pose.position.x = mMap2BaseTransf.getOrigin().x();
    msg->pose.pose.position.y = mMap2BaseTransf.getOrigin().y();
    msg->pose.pose.position.z = mMap2BaseTransf.getOrigin().z();
    msg->pose.pose.orientation.x = mMap2BaseTransf.getRotation().x();
    msg->pose.pose.orientation.y = mMap2BaseTransf.getRotation().y();
    msg->pose.pose.orientation.z = mMap2BaseTransf.getRotation().z();
    msg->pose.pose.orientation.w = mMap2BaseTransf.getRotation().w();

    // Odometry pose covariance
    for (size_t i = 0; i < msg->pose.covariance.size(); i++) {
      msg->pose.covariance[i] =
        static_cast<double>(mLastZedPose.pose_covariance[i]);

      if (mTwoDMode) {
        if (i == 14 || i == 21 || i == 28) {
          msg->pose.covariance[i] = 1e-9;   // Very low covariance if 2D mode
        } else if ((i >= 2 && i <= 4) || (i >= 8 && i <= 10) ||
          (i >= 12 && i <= 13) || (i >= 15 && i <= 16) ||
          (i >= 18 && i <= 20) || (i == 22) || (i >= 24 && i <= 27))
        {
          msg->pose.covariance[i] = 0.0;
        }
      }
    }

    // Publish gnss message
    // DEBUG_GNSS("Publishing GNSS pose message");
    try {
      mPubGnssPose->publish(std::move(msg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }

  if (geoPoseSub > 0) {
    geoPoseMsgPtr msg =
      std::make_unique<geographic_msgs::msg::GeoPoseStamped>();

    msg->header.stamp = mFrameTimestamp;
    msg->header.frame_id = mMapFrameId;

    // Latest Lat Long data
    msg->pose.position.latitude = mLastLatLongPose.getLatitude(false);
    msg->pose.position.longitude = mLastLatLongPose.getLongitude(false);
    msg->pose.position.altitude = mLastLatLongPose.getAltitude();

    // Latest Heading Quaternion
    msg->pose.orientation.x = mLastHeadingQuat.getX();
    msg->pose.orientation.y = mLastHeadingQuat.getY();
    msg->pose.orientation.z = mLastHeadingQuat.getZ();
    msg->pose.orientation.w = mLastHeadingQuat.getW();

    // Publish gnss message
    // DEBUG_GNSS("Publishing GeoPose message");
    try {
      mPubGeoPose->publish(std::move(msg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }

  if (fusedFixSub > 0) {
    navsatMsgPtr msg = std::make_unique<sensor_msgs::msg::NavSatFix>();

    msg->header.stamp = mFrameTimestamp;
    msg->header.frame_id = mMapFrameId;

    msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
    msg->status.service = mGnssService;
    msg->latitude = mLastLatLongPose.getLatitude(false);
    msg->longitude = mLastLatLongPose.getLongitude(false);
    msg->altitude = mLastLatLongPose.getAltitude();

    // ----> Covariance
    msg->position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    for (size_t i = 0; i < msg->position_covariance.size(); i++) {
      msg->position_covariance[i] =
        static_cast<double>(mLastZedPose.pose_covariance[i]);

      if (mTwoDMode) {
        if (i == 14 || i == 21 || i == 28) {
          msg->position_covariance[i] = 1e-9;   // Very low covariance if 2D mode
        } else if ((i >= 2 && i <= 4) || (i >= 8 && i <= 10) ||
          (i >= 12 && i <= 13) || (i >= 15 && i <= 16) ||
          (i >= 18 && i <= 20) || (i == 22) || (i >= 24 && i <= 27))
        {
          msg->position_covariance[i] = 0.0;
        }
      }
    }
    // <---- Covariance

    // Publish Fused Fix message
    // DEBUG_GNSS("Publishing Fused Fix message");ù
    try {
      mPubFusedFix->publish(std::move(msg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }

  if (originFixSub > 0) {
    navsatMsgPtr msg = std::make_unique<sensor_msgs::msg::NavSatFix>();

    msg->header.stamp = mFrameTimestamp;
    msg->header.frame_id = mGnssOriginFrameId;

    msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
    msg->status.service = mGnssService;
    msg->latitude = mInitLatLongPose.getLatitude(false);
    msg->longitude = mInitLatLongPose.getLongitude(false);
    msg->altitude = mInitLatLongPose.getAltitude();

    // ----> Covariance
    msg->position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    for (size_t i = 0; i < msg->position_covariance.size(); i++) {
      msg->position_covariance[i] = -1.0;
    }
    // <---- Covariance

    // Publish Fused Fix message
    // DEBUG_GNSS("Publishing Fused Fix message");
    try {
      mPubOriginFix->publish(std::move(msg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
}

void ZedCamera::processDetectedObjects(rclcpp::Time t)
{
  size_t objdet_sub_count = 0;

  try {
    objdet_sub_count = count_subscribers(mPubObjDet->get_topic_name());
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_OD(
      "processDetectedObjects: Exception while counting subscribers");
    return;
  }

  if (objdet_sub_count < 1) {
    mObjDetSubscribed = false;
    return;
  }

  sl_tools::StopWatch odElabTimer(get_clock());

  mObjDetSubscribed = true;

  sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;

  // ----> Process realtime dynamic parameters
  objectTracker_parameters_rt.detection_confidence_threshold =
    mObjDetConfidence;
  mObjDetFilter.clear();
  if (mObjDetPeopleEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::PERSON);
  }
  if (mObjDetVehiclesEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::VEHICLE);
  }
  if (mObjDetBagsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::BAG);
  }
  if (mObjDetAnimalsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::ANIMAL);
  }
  if (mObjDetElectronicsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::ELECTRONICS);
  }
  if (mObjDetFruitsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::FRUIT_VEGETABLE);
  }
  if (mObjDetSportEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::SPORT);
  }
  objectTracker_parameters_rt.object_class_filter = mObjDetFilter;
  // <---- Process realtime dynamic parameters

  sl::Objects objects;

  sl::ERROR_CODE objDetRes = mZed->retrieveObjects(
    objects, objectTracker_parameters_rt, mObjDetInstID);

  if (objDetRes != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Object Detection error: " << sl::toString(objDetRes));
    return;
  }

  if (!objects.is_new) {    // Async object detection. Update data only if new
    // detection is available
    return;
  }

  // DEBUG_STREAM_OD( "Detected " << objects.object_list.size()
  // << " objects");

  size_t objCount = objects.object_list.size();

  objDetMsgPtr objMsg =
    std::make_unique<zed_interfaces::msg::ObjectsStamped>();

  objMsg->header.stamp = t;
  objMsg->header.frame_id = mLeftCamFrameId;

  objMsg->objects.resize(objCount);

  size_t idx = 0;
  for (auto data : objects.object_list) {
    objMsg->objects[idx].label = sl::toString(data.label).c_str();
    objMsg->objects[idx].sublabel = sl::toString(data.sublabel).c_str();
    objMsg->objects[idx].label_id = data.id;
    objMsg->objects[idx].confidence = data.confidence;

    memcpy(
      &(objMsg->objects[idx].position[0]), &(data.position[0]),
      3 * sizeof(float));
    memcpy(
      &(objMsg->objects[idx].position_covariance[0]),
      &(data.position_covariance[0]), 6 * sizeof(float));
    memcpy(
      &(objMsg->objects[idx].velocity[0]), &(data.velocity[0]),
      3 * sizeof(float));

    objMsg->objects[idx].tracking_available = mObjDetTracking;
    objMsg->objects[idx].tracking_state =
      static_cast<int8_t>(data.tracking_state);
    objMsg->objects[idx].action_state =
      static_cast<int8_t>(data.action_state);

    if (data.bounding_box_2d.size() == 4) {
      memcpy(
        &(objMsg->objects[idx].bounding_box_2d.corners[0]),
        &(data.bounding_box_2d[0]), 8 * sizeof(unsigned int));
    }
    if (data.bounding_box.size() == 8) {
      memcpy(
        &(objMsg->objects[idx].bounding_box_3d.corners[0]),
        &(data.bounding_box[0]), 24 * sizeof(float));
    }

    memcpy(
      &(objMsg->objects[idx].dimensions_3d[0]), &(data.dimensions[0]),
      3 * sizeof(float));

    if (data.head_bounding_box_2d.size() == 4) {
      memcpy(
        &(objMsg->objects[idx].head_bounding_box_2d.corners[0]),
        &(data.head_bounding_box_2d[0]), 8 * sizeof(unsigned int));
    }
    if (data.head_bounding_box.size() == 8) {
      memcpy(
        &(objMsg->objects[idx].head_bounding_box_3d.corners[0]),
        &(data.head_bounding_box[0]), 24 * sizeof(float));
    }

    memcpy(
      &(objMsg->objects[idx].head_position[0]), &(data.head_position[0]),
      3 * sizeof(float));

    objMsg->objects[idx].skeleton_available = false;
    // at the end of the loop
    idx++;
  }

  // DEBUG_STREAM_OD("Publishing OBJ DET message");
  try {
    mPubObjDet->publish(std::move(objMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }

  // ----> Diagnostic information update
  mObjDetElabMean_sec->addValue(odElabTimer.toc());
  mObjDetPeriodMean_sec->addValue(mOdFreqTimer.toc());
  mOdFreqTimer.tic();
  // <---- Diagnostic information update
}

void ZedCamera::processBodies(rclcpp::Time t)
{
  // DEBUG_BT("processBodies");

  size_t bt_sub_count = 0;

  try {
    bt_sub_count = count_subscribers(mPubBodyTrk->get_topic_name());
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_OD("processBodies: Exception while counting subscribers");
    return;
  }

  if (bt_sub_count < 1) {
    mBodyTrkSubscribed = false;
    return;
  }

  sl_tools::StopWatch btElabTimer(get_clock());

  mBodyTrkSubscribed = true;

  // ----> Process realtime dynamic parameters
  sl::BodyTrackingRuntimeParameters bt_params_rt;
  bt_params_rt.detection_confidence_threshold = mBodyTrkConfThresh;
  bt_params_rt.minimum_keypoints_threshold = mBodyTrkMinKp;
  // <---- Process realtime dynamic parameters

  sl::Bodies bodies;
  sl::ERROR_CODE btRes =
    mZed->retrieveBodies(bodies, bt_params_rt, mBodyTrkInstID);

  if (btRes != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Body Tracking error: " << sl::toString(btRes));
    return;
  }

  if (!bodies.is_new) {    // Async body tracking. Update data only if new
    // detection is available
    DEBUG_BT("No new bodies detected");
    return;
  }

  size_t bodyCount = bodies.body_list.size();

  DEBUG_STREAM_BT("Detected " << bodyCount << " bodies");

  objDetMsgPtr bodyMsg =
    std::make_unique<zed_interfaces::msg::ObjectsStamped>();

  bodyMsg->header.stamp = t;
  bodyMsg->header.frame_id = mLeftCamFrameId;

  bodyMsg->objects.resize(bodyCount);

  size_t idx = 0;
  for (auto body : bodies.body_list) {
    std::string label = "Body_";
    label += std::to_string(body.id);
    DEBUG_STREAM_BT("Processing body: " << label);
    bodyMsg->objects[idx].label = sl::String(label.c_str());
    bodyMsg->objects[idx].sublabel = "";
    bodyMsg->objects[idx].label_id = body.id;
    bodyMsg->objects[idx].confidence = body.confidence;
    DEBUG_BT(" - Info OK");

    memcpy(
      &(bodyMsg->objects[idx].position[0]), &(body.position[0]),
      3 * sizeof(float));
    memcpy(
      &(bodyMsg->objects[idx].position_covariance[0]),
      &(body.position_covariance[0]), 6 * sizeof(float));
    memcpy(
      &(bodyMsg->objects[idx].velocity[0]), &(body.velocity[0]),
      3 * sizeof(float));
    DEBUG_BT(" - Pos/Cov/Speed OK");

    bodyMsg->objects[idx].tracking_available = mBodyTrkEnableTracking;
    bodyMsg->objects[idx].tracking_state =
      static_cast<int8_t>(body.tracking_state);
    bodyMsg->objects[idx].action_state =
      static_cast<int8_t>(body.action_state);
    DEBUG_BT(" - Status OK");

    if (body.bounding_box_2d.size() == 4) {
      memcpy(
        &(bodyMsg->objects[idx].bounding_box_2d.corners[0]),
        &(body.bounding_box_2d[0]), 8 * sizeof(unsigned int));
    }
    DEBUG_BT(" - BBox 2D OK");
    if (body.bounding_box.size() == 8) {
      memcpy(
        &(bodyMsg->objects[idx].bounding_box_3d.corners[0]),
        &(body.bounding_box[0]), 24 * sizeof(float));
    }
    DEBUG_BT(" - BBox 3D OK");

    memcpy(
      &(bodyMsg->objects[idx].dimensions_3d[0]), &(body.dimensions[0]),
      3 * sizeof(float));
    DEBUG_BT(" - Dims OK");

    bodyMsg->objects[idx].body_format = static_cast<uint8_t>(mBodyTrkFmt);

    if (body.head_bounding_box_2d.size() == 4) {
      memcpy(
        &(bodyMsg->objects[idx].head_bounding_box_2d.corners[0]),
        &(body.head_bounding_box_2d[0]), 8 * sizeof(unsigned int));
    }
    if (body.head_bounding_box.size() == 8) {
      memcpy(
        &(bodyMsg->objects[idx].head_bounding_box_3d.corners[0]),
        &(body.head_bounding_box[0]), 24 * sizeof(float));
    }

    memcpy(
      &(bodyMsg->objects[idx].head_position[0]),
      &(body.head_position[0]), 3 * sizeof(float));

    bodyMsg->objects[idx].skeleton_available = true;

    uint8_t kp_size = body.keypoint_2d.size();
    DEBUG_STREAM_BT(" * Skeleton KP: " << static_cast<int>(kp_size));
    if (kp_size <= 70) {
      memcpy(
        &(bodyMsg->objects[idx].skeleton_2d.keypoints[0]),
        &(body.keypoint_2d[0]), 2 * kp_size * sizeof(float));

      memcpy(
        &(bodyMsg->objects[idx].skeleton_3d.keypoints[0]),
        &(body.keypoint[0]), 3 * kp_size * sizeof(float));
    }

    // ----------------------------------
    // at the end of the loop

    // TODO(Walter) Add support for
    // body.global_root_orientation;
    // body.local_orientation_per_joint;
    // body.local_orientation_per_joint;

    idx++;
  }

  DEBUG_STREAM_OD("Publishing BODY TRK message");
  try {
    mPubBodyTrk->publish(std::move(bodyMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }

  // ----> Diagnostic information update
  mBodyTrkElabMean_sec->addValue(btElabTimer.toc());
  mBodyTrkPeriodMean_sec->addValue(mBtFreqTimer.toc());
  mBtFreqTimer.tic();
  // <---- Diagnostic information update
}

bool ZedCamera::isDepthRequired()
{
  // DEBUG_STREAM_VD( "isDepthRequired called");

  if (mDepthDisabled) {
    return false;
  }

  size_t tot_sub = 0;

  try {
    size_t depthSub = 0;
    size_t confMapSub = 0;
    size_t dispSub = 0;
    size_t pcSub = 0;
    size_t depthInfoSub = 0;
    depthSub = mPubDepth.getNumSubscribers();
    confMapSub = count_subscribers(mPubConfMap->get_topic_name());
    dispSub = count_subscribers(mPubDisparity->get_topic_name());
#ifndef FOUND_FOXY
    pcSub = mPubCloud.getNumSubscribers();
#else
    pcSub = count_subscribers(mPubCloud->get_topic_name());
#endif
    depthInfoSub = count_subscribers(mPubDepthInfo->get_topic_name());

    tot_sub = depthSub + confMapSub + dispSub + pcSub + depthInfoSub;
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_VD("isDepthRequired: Exception while counting subscribers");
    return false;
  }

  return tot_sub > 0 || isPosTrackingRequired();
}

void ZedCamera::applyDepthSettings()
{
  if (isDepthRequired()) {
    std::lock_guard<std::mutex> lock(mDynParMutex);
    mRunParams.confidence_threshold =
      mDepthConf;      // Update depth confidence if changed
    mRunParams.texture_confidence_threshold =
      mDepthTextConf;      // Update depth texture confidence if changed
    mRunParams.remove_saturated_areas = mRemoveSatAreas;

    DEBUG_STREAM_COMM("Depth extraction enabled");
    mRunParams.enable_depth = true;
  } else {
    DEBUG_STREAM_COMM("Depth extraction disabled");
    mRunParams.enable_depth = false;
  }
}

void ZedCamera::applyVideoSettings()
{
  sl::ERROR_CODE err;
  sl::VIDEO_SETTINGS setting;

  if (!mSvoMode && mFrameCount % 10 == 0) {
    std::lock_guard<std::mutex> lock(mDynParMutex);

    if (mTriggerAutoExpGain) {
      setting = sl::VIDEO_SETTINGS::AEC_AGC;
      err = mZed->setCameraSettings(setting, (mCamAutoExpGain ? 1 : 0));
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting AEC_AGC: "
            << sl::toString(err).c_str());
      } else {
        mTriggerAutoExpGain = false;
        DEBUG_STREAM_CTRL(
          "New setting for " << sl::toString(setting).c_str()
                             << ": "
                             << (mCamAutoExpGain ? 1 : 0));
      }
    }

    if (!mCamAutoExpGain) {
      int value;
      err = mZed->getCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamExposure) {
        mZed->setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, mCamExposure);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }

      err = mZed->getCameraSettings(sl::VIDEO_SETTINGS::GAIN, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamGain) {
        err = mZed->setCameraSettings(sl::VIDEO_SETTINGS::GAIN, mCamGain);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }
    }

    if (mTriggerAutoWB) {
      setting = sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO;
      err = mZed->setCameraSettings(setting, (mCamAutoWB ? 1 : 0));
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      } else {
        mTriggerAutoWB = false;
        DEBUG_STREAM_CTRL(
          "New setting for " << sl::toString(setting).c_str()
                             << ": " << (mCamAutoWB ? 1 : 0));
      }
    }

    if (!mCamAutoWB) {
      int value;
      setting = sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE;
      err = mZed->getCameraSettings(setting, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamWBTemp) {
        err = mZed->setCameraSettings(setting, mCamWBTemp);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }
    }

    // ----> BRIGHTNESS, CONTRAST, HUE controls not available for ZED X and
    // ZED X Mini
    if (!sl_tools::isZEDX(mCamRealModel)) {
      int value;
      setting = sl::VIDEO_SETTINGS::BRIGHTNESS;
      err = mZed->getCameraSettings(setting, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamBrightness) {
        mZed->setCameraSettings(setting, mCamBrightness);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }

      setting = sl::VIDEO_SETTINGS::CONTRAST;
      err = mZed->getCameraSettings(setting, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamContrast) {
        err = mZed->setCameraSettings(setting, mCamContrast);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }

      setting = sl::VIDEO_SETTINGS::HUE;
      err = mZed->getCameraSettings(setting, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamHue) {
        mZed->setCameraSettings(setting, mCamHue);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }
    }
    // <---- BRIGHTNESS, CONTRAST, HUE controls not available for ZED X and
    // ZED X Mini

    setting = sl::VIDEO_SETTINGS::SATURATION;
    int value;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamSaturation) {
      mZed->setCameraSettings(setting, mCamSaturation);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }

    setting = sl::VIDEO_SETTINGS::SHARPNESS;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamSharpness) {
      mZed->setCameraSettings(setting, mCamSharpness);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }

    setting = sl::VIDEO_SETTINGS::GAMMA;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamGamma) {
      err = mZed->setCameraSettings(setting, mCamGamma);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }

    if (sl_tools::isZEDX(mCamRealModel)) {
      if (!mCamAutoExpGain) {
        setting = sl::VIDEO_SETTINGS::EXPOSURE_TIME;
        err = mZed->getCameraSettings(setting, value);
        if (err == sl::ERROR_CODE::SUCCESS && value != mGmslExpTime) {
          err = mZed->setCameraSettings(setting, mGmslExpTime);
          DEBUG_STREAM_CTRL(
            "New setting for "
              << sl::toString(setting).c_str() << ": "
              << mGmslExpTime << " [Old " << value << "]");
        }

        if (err != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            "Error setting " << sl::toString(setting).c_str()
                             << ": "
                             << sl::toString(err).c_str());
        }
      }

      // TODO(Walter) Enable when fixed in the SDK
      // err = mZed->getCameraSettings(
      //   sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE, value_min,
      //   value_max);
      // if (err == sl::ERROR_CODE::SUCCESS &&
      //   (value_min != mGmslAutoExpTimeRangeMin || value_max !=
      //   mGmslAutoExpTimeRangeMax))
      // {
      //   err = mZed->setCameraSettings(
      //     sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE,
      //     mGmslAutoExpTimeRangeMin, mGmslAutoExpTimeRangeMax);
      // } else if (err != sl::ERROR_CODE::SUCCESS) {
      //   RCLCPP_WARN_STREAM(
      //     get_logger(),
      //     "Error setting AUTO_EXPOSURE_TIME_RANGE: " <<
      //     sl::toString(err).c_str() );
      // }

      if (!mStreamMode) {
        setting = sl::VIDEO_SETTINGS::EXPOSURE_COMPENSATION;
        err = mZed->getCameraSettings(setting, value);
        if (err == sl::ERROR_CODE::SUCCESS && value != mGmslExposureComp) {
          err = mZed->setCameraSettings(setting, mGmslExposureComp);
          DEBUG_STREAM_CTRL(
            "New setting for " << sl::toString(setting).c_str()
                               << ": " << mGmslExposureComp
                               << " [Old " << value << "]");
        }

        if (err != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting "
              << sl::toString(setting).c_str()
              << ": "
              << sl::toString(err).c_str());
        }
      }

      setting = sl::VIDEO_SETTINGS::ANALOG_GAIN;
      if (!mCamAutoExpGain) {
        err = mZed->getCameraSettings(setting, value);
        if (err == sl::ERROR_CODE::SUCCESS && value != mGmslAnalogGain) {
          err = mZed->setCameraSettings(setting, mGmslAnalogGain);
          DEBUG_STREAM_CTRL(
            "New setting for "
              << sl::toString(setting).c_str() << ": "
              << mGmslAnalogGain << " [Old " << value << "]");
        }

        if (err != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            "Error setting " << sl::toString(setting).c_str()
                             << ": "
                             << sl::toString(err).c_str());
        }

        // TODO(Walter) Enable when fixed in the SDK
        // err =
        //   mZed->getCameraSettings(sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE,
        //   value_min, value_max);
        // if (err == sl::ERROR_CODE::SUCCESS &&
        //   (value_min != mGmslAnalogGainRangeMin || value_max !=
        //   mGmslAnalogGainRangeMax))
        // {
        //   err = mZed->setCameraSettings(
        //     sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE,
        //     mGmslAnalogGainRangeMin, mGmslAnalogGainRangeMax);
        // } else if (err != sl::ERROR_CODE::SUCCESS) {
        //   RCLCPP_WARN_STREAM(
        //     get_logger(),
        //     "Error setting AUTO_ANALOG_GAIN_RANGE: " <<
        //     sl::toString(err).c_str() );
        // }

        setting = sl::VIDEO_SETTINGS::DIGITAL_GAIN;
        err = mZed->getCameraSettings(setting, value);
        if (err == sl::ERROR_CODE::SUCCESS && value != mGmslDigitalGain) {
          err = mZed->setCameraSettings(setting, mGmslDigitalGain);
          DEBUG_STREAM_CTRL(
            "New setting for "
              << sl::toString(setting).c_str() << ": "
              << mGmslDigitalGain << " [Old " << value << "]");
        }

        if (err != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            "Error setting " << sl::toString(setting).c_str()
                             << ": "
                             << sl::toString(err).c_str());
        }
      }

      // TODO(Walter) Enable when fixed in the SDK
      // err =
      //   mZed->getCameraSettings(sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE,
      //   value_min, value_max);
      // if (err == sl::ERROR_CODE::SUCCESS &&
      //   (value_min != mGmslAutoDigitalGainRangeMin || value_max !=
      //   mGmslAutoDigitalGainRangeMax))
      // {
      //   err = mZed->setCameraSettings(
      //     sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE,
      //     mGmslAutoDigitalGainRangeMin, mGmslAnalogGainRangeMax);
      // } else if (err != sl::ERROR_CODE::SUCCESS) {
      //   RCLCPP_WARN_STREAM(
      //     get_logger(),
      //     "Error setting AUTO_DIGITAL_GAIN_RANGE: " <<
      //     sl::toString(err).c_str() );
      // }

      if (!mStreamMode) {
        setting = sl::VIDEO_SETTINGS::DENOISING;
        err = mZed->getCameraSettings(setting, value);
        if (err == sl::ERROR_CODE::SUCCESS && value != mGmslDenoising) {
          err = mZed->setCameraSettings(setting, mGmslDenoising);
          DEBUG_STREAM_CTRL(
            "New setting for " << sl::toString(setting).c_str()
                               << ": " << mGmslDenoising
                               << " [Old " << value << "]");
        }

        if (err != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting "
              << sl::toString(setting).c_str()
              << ": "
              << sl::toString(err).c_str());
        }
      }
    }
  }
}

bool ZedCamera::isPosTrackingRequired()
{
  if (mDepthDisabled) {
    DEBUG_ONCE_PT("POS. TRACKING not required: Depth disabled.");
    return false;
  }

  if (mPosTrackingEnabled) {
    DEBUG_ONCE_PT("POS. TRACKING required: enabled by param.");
    return true;
  }

  if (mPublishTF) {
    DEBUG_ONCE_PT("POS. TRACKING required: enabled by TF param.");
    return true;
  }

  if (mDepthStabilization > 0) {
    DEBUG_ONCE_PT(
      "POS. TRACKING required: enabled by depth stabilization param.");
    return true;
  }

  if (mMappingEnabled || mObjDetEnabled) {
    DEBUG_ONCE_PT(
      "POS. TRACKING required: enabled by mapping or object detection.");
    return true;
  }

  size_t topics_sub = 0;
  try {
    topics_sub = count_subscribers(mPubPose->get_topic_name()) +
      count_subscribers(mPubPoseCov->get_topic_name()) +
      count_subscribers(mPubPosePath->get_topic_name()) +
      count_subscribers(mPubOdom->get_topic_name()) +
      count_subscribers(mPubOdomPath->get_topic_name());
  } catch (...) {
    rcutils_reset_error();
    RCLCPP_WARN(
      get_logger(),
      "isPosTrackingRequired: Exception while counting subscribers");
    return false;
  }

  if (topics_sub > 0) {
    DEBUG_ONCE_PT("POS. TRACKING required: topic subscribed.");
    return true;
  }

  DEBUG_ONCE_PT("POS. TRACKING not required.");
  return false;
}

void ZedCamera::publishDepthMapWithInfo(sl::Mat & depth, rclcpp::Time t)
{
  mDepthCamInfoMsg->header.stamp = t;

  if (!mOpenniDepthMode) {
    auto depth_img = sl_tools::imageToROSmsg(depth, mDepthOptFrameId, t);
    DEBUG_STREAM_VD(
      "Publishing DEPTH message: " << t.nanoseconds()
                                   << " nsec");
    try {
      mPubDepth.publish(std::move(depth_img), mDepthCamInfoMsg);
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
    return;
  }

  // OPENNI CONVERSION (meter -> millimeters - float32 -> uint16)
  std::unique_ptr<sensor_msgs::msg::Image> openniDepthMsg =
    std::make_unique<sensor_msgs::msg::Image>();

  openniDepthMsg->header.stamp = t;
  openniDepthMsg->header.frame_id = mDepthOptFrameId;
  openniDepthMsg->height = depth.getHeight();
  openniDepthMsg->width = depth.getWidth();

  int num = 1;    // for endianness detection
  openniDepthMsg->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

  openniDepthMsg->step = openniDepthMsg->width * sizeof(uint16_t);
  openniDepthMsg->encoding = sensor_msgs::image_encodings::MONO16;

  size_t size = openniDepthMsg->step * openniDepthMsg->height;
  openniDepthMsg->data.resize(size);

  uint16_t * data = reinterpret_cast<uint16_t *>(&openniDepthMsg->data[0]);

  int dataSize = openniDepthMsg->width * openniDepthMsg->height;
  sl::float1 * depthDataPtr = depth.getPtr<sl::float1>();

  for (int i = 0; i < dataSize; i++) {
    *(data++) = static_cast<uint16_t>(
      std::round(*(depthDataPtr++) * 1000));      // in mm, rounded
  }

  DEBUG_STREAM_VD("Publishing OPENNI DEPTH message");
  try {
    mPubDepth.publish(std::move(openniDepthMsg), mDepthCamInfoMsg);
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
}

void ZedCamera::publishDisparity(sl::Mat disparity, rclcpp::Time t)
{
  sl::CameraInformation zedParam = mZed->getCameraInformation(mMatResol);

  std::unique_ptr<sensor_msgs::msg::Image> disparity_image =
    sl_tools::imageToROSmsg(disparity, mDepthOptFrameId, t);

  dispMsgPtr disparityMsg =
    std::make_unique<stereo_msgs::msg::DisparityImage>();
  disparityMsg->image = *disparity_image.get();
  disparityMsg->header = disparityMsg->image.header;
  disparityMsg->f =
    zedParam.camera_configuration.calibration_parameters.left_cam.fx;
  disparityMsg->t = zedParam.camera_configuration.calibration_parameters
    .getCameraBaseline();
  disparityMsg->min_disparity =
    disparityMsg->f * disparityMsg->t /
    mZed->getInitParameters().depth_minimum_distance;
  disparityMsg->max_disparity =
    disparityMsg->f * disparityMsg->t /
    mZed->getInitParameters().depth_maximum_distance;

  DEBUG_STREAM_VD("Publishing DISPARITY message");
  try {
    mPubDisparity->publish(std::move(disparityMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
}

void ZedCamera::publishPointCloud()
{
  sl_tools::StopWatch pcElabTimer(get_clock());

  pointcloudMsgPtr pcMsg = std::make_unique<sensor_msgs::msg::PointCloud2>();

  // Initialize Point Cloud message
  // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h

  int width = mMatResol.width;
  int height = mMatResol.height;

  int ptsCount = width * height;

  if (mSvoMode) {
    // pcMsg->header.stamp =
    // sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::CURRENT));
    pcMsg->header.stamp = mFrameTimestamp;
  } else if (mSimMode) {
    if (mUseSimTime) {
      pcMsg->header.stamp = mFrameTimestamp;
    } else {
      pcMsg->header.stamp = sl_tools::slTime2Ros(mMatCloud.timestamp);
    }
  } else {
    pcMsg->header.stamp = sl_tools::slTime2Ros(mMatCloud.timestamp);
  }

  // ---> Check that `pcMsg->header.stamp` is not the same of the latest
  // published pointcloud Avoid to publish the same old data
  if (mLastTs_pc == pcMsg->header.stamp) {
    // Data not updated by a grab calling in the grab thread
    DEBUG_STREAM_PC("publishPointCloud: ignoring not update data");
    return;
  }
  mLastTs_pc = pcMsg->header.stamp;
  // <--- Check that `pcMsg->header.stamp` is not the same of the latest
  // published pointcloud

  if (pcMsg->width != width || pcMsg->height != height) {
    pcMsg->header.frame_id =
      mPointCloudFrameId;      // Set the header values of the ROS message

    pcMsg->is_bigendian = false;
    pcMsg->is_dense = false;

    pcMsg->width = width;
    pcMsg->height = height;

    sensor_msgs::PointCloud2Modifier modifier(*(pcMsg.get()));
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "rgb", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  }

  sl::Vector4<float> * cpu_cloud = mMatCloud.getPtr<sl::float4>();

  // Data copy
  float * ptCloudPtr = reinterpret_cast<float *>(&pcMsg->data[0]);
  memcpy(
    ptCloudPtr, reinterpret_cast<float *>(cpu_cloud),
    ptsCount * 4 * sizeof(float));

  // Pointcloud publishing
  DEBUG_STREAM_PC("Publishing POINT CLOUD message");
#ifndef FOUND_FOXY
  try {
    mPubCloud.publish(std::move(pcMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
#else
  try {
    mPubCloud->publish(std::move(pcMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
#endif

  // Publish freq calculation
  double mean = mPcPeriodMean_sec->addValue(mPcFreqTimer.toc());
  mPcFreqTimer.tic();

  // Point cloud elaboration time
  mPcProcMean_sec->addValue(pcElabTimer.toc());
  DEBUG_STREAM_PC("Point cloud freq: " << 1. / mean);
}

void ZedCamera::callback_pubTemp()
{
  DEBUG_STREAM_ONCE_SENS("Temperatures callback called");

  if (mGrabStatus != sl::ERROR_CODE::SUCCESS) {
    DEBUG_SENS("Camera not ready");
    rclcpp::sleep_for(1s);
    return;
  }


  if (sl_tools::isZED(mCamRealModel) || sl_tools::isZEDM(mCamRealModel)) {
    DEBUG_SENS(
      "callback_pubTemp: the callback should never be called for the ZED "
      "or "
      "ZEDM camera models!");
    return;
  }

  // ----> Always update temperature values for diagnostic
  sl::SensorsData sens_data;
  sl::ERROR_CODE err =
    mZed->getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT);
  if (err != sl::ERROR_CODE::SUCCESS) {
    DEBUG_STREAM_SENS(
      "[callback_pubTemp] sl::getSensorsData error: "
        << sl::toString(err).c_str());
    return;
  }

  if (sl_tools::isZED2OrZED2i(mCamRealModel)) {
    sens_data.temperature.get(
      sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT,
      mTempLeft);
    sens_data.temperature.get(
      sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_RIGHT,
      mTempRight);
  } else {
    mTempLeft = NOT_VALID_TEMP;
    mTempRight = NOT_VALID_TEMP;
  }

  sens_data.temperature.get(
    sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU, mTempImu);
  // <---- Always update temperature values for diagnostic

  // ----> Subscribers count
  size_t tempLeftSubNumber = 0;
  size_t tempRightSubNumber = 0;
  size_t tempImuSubNumber = 0;

  try {
    tempLeftSubNumber = 0;
    tempRightSubNumber = 0;
    tempImuSubNumber = 0;

    if (sl_tools::isZED2OrZED2i(mCamRealModel)) {
      tempLeftSubNumber = count_subscribers(mPubTempL->get_topic_name());
      tempRightSubNumber = count_subscribers(mPubTempR->get_topic_name());
    }
    tempImuSubNumber = count_subscribers(mPubImuTemp->get_topic_name());
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_SENS(
      "callback_pubTemp: Exception while counting subscribers");
    return;
  }
  // <---- Subscribers count

  rclcpp::Time now = get_clock()->now();

  if (tempLeftSubNumber > 0) {
    tempMsgPtr leftTempMsg =
      std::make_unique<sensor_msgs::msg::Temperature>();

    leftTempMsg->header.stamp = now;

    leftTempMsg->header.frame_id = mTempLeftFrameId;
    leftTempMsg->temperature = static_cast<double>(mTempLeft);
    leftTempMsg->variance = 0.0;

    try {
      mPubTempL->publish(std::move(leftTempMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }

  if (tempRightSubNumber > 0) {
    tempMsgPtr rightTempMsg =
      std::make_unique<sensor_msgs::msg::Temperature>();

    rightTempMsg->header.stamp = now;

    rightTempMsg->header.frame_id = mTempRightFrameId;
    rightTempMsg->temperature = static_cast<double>(mTempRight);
    rightTempMsg->variance = 0.0;

    DEBUG_STREAM_SENS("Publishing RIGHT TEMP message");
    try {
      mPubTempR->publish(std::move(rightTempMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }

  if (tempImuSubNumber > 0) {
    tempMsgPtr imuTempMsg = std::make_unique<sensor_msgs::msg::Temperature>();

    imuTempMsg->header.stamp = now;

    imuTempMsg->header.frame_id = mImuFrameId;
    imuTempMsg->temperature = static_cast<double>(mTempImu);
    imuTempMsg->variance = 0.0;

    DEBUG_SENS("Publishing IMU TEMP message");
    try {
      mPubImuTemp->publish(std::move(imuTempMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
}

void ZedCamera::callback_pubFusedPc()
{
  DEBUG_STREAM_ONCE_MAP("Mapping callback called");

  pointcloudMsgPtr pointcloudFusedMsg =
    std::make_unique<sensor_msgs::msg::PointCloud2>();

  uint32_t fusedCloudSubnumber = 0;
  try {
#ifndef FOUND_FOXY
    fusedCloudSubnumber = mPubFusedCloud.getNumSubscribers();
#else
    fusedCloudSubnumber = count_subscribers(mPubFusedCloud->get_topic_name());
#endif
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_MAP("pubFusedPc: Exception while counting subscribers");
    return;
  }

  if (fusedCloudSubnumber == 0) {
    return;
  }

  if (!mZed->isOpened()) {
    return;
  }

  mZed->requestSpatialMapAsync();

  while (mZed->getSpatialMapRequestStatusAsync() == sl::ERROR_CODE::FAILURE) {
    // Mesh is still generating
    rclcpp::sleep_for(1ms);
  }

  sl::ERROR_CODE res = mZed->retrieveSpatialMapAsync(mFusedPC);

  if (res != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Fused point cloud not extracted: "
        << sl::toString(res).c_str());
    return;
  }

  size_t ptsCount = mFusedPC.getNumberOfPoints();
  bool resized = false;

  if (pointcloudFusedMsg->width != ptsCount ||
    pointcloudFusedMsg->height != 1)
  {
    // Initialize Point Cloud message
    // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h
    pointcloudFusedMsg->header.frame_id =
      mMapFrameId;      // Set the header values of the ROS message
    pointcloudFusedMsg->is_bigendian = false;
    pointcloudFusedMsg->is_dense = false;
    pointcloudFusedMsg->width = ptsCount;
    pointcloudFusedMsg->height = 1;

    sensor_msgs::PointCloud2Modifier modifier(*pointcloudFusedMsg);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "rgb", 1,
      sensor_msgs::msg::PointField::FLOAT32);

    resized = true;
  }

  float * ptCloudPtr = reinterpret_cast<float *>(&pointcloudFusedMsg->data[0]);
  int updated = 0;

  for (size_t c = 0; c < mFusedPC.chunks.size(); c++) {
    if (mFusedPC.chunks[c].has_been_updated || resized) {
      updated++;
      size_t chunkSize = mFusedPC.chunks[c].vertices.size();

      if (chunkSize > 0) {
        float * cloud_pts =
          reinterpret_cast<float *>(mFusedPC.chunks[c].vertices.data());
        memcpy(ptCloudPtr, cloud_pts, 4 * chunkSize * sizeof(float));
        ptCloudPtr += 4 * chunkSize;
        if (mSvoMode) {
          pointcloudFusedMsg->header.stamp = mFrameTimestamp;
        } else if (mSimMode) {
          if (mUseSimTime) {
            pointcloudFusedMsg->header.stamp = mFrameTimestamp;
          } else {
            pointcloudFusedMsg->header.stamp =
              sl_tools::slTime2Ros(mFusedPC.chunks[c].timestamp);
          }
        } else {
          pointcloudFusedMsg->header.stamp =
            sl_tools::slTime2Ros(mFusedPC.chunks[c].timestamp);
        }
      }
    }
  }

  rclcpp::Time ros_ts = get_clock()->now();

  if (mPrevTs_pc != TIMEZERO_ROS) {
    mPubFusedCloudPeriodMean_sec->addValue((ros_ts - mPrevTs_pc).seconds());
  }
  mPrevTs_pc = ros_ts;

  // Pointcloud publishing
  DEBUG_STREAM_MAP("Publishing FUSED POINT CLOUD message");
#ifndef FOUND_FOXY
  try {
    mPubFusedCloud.publish(std::move(pointcloudFusedMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
#else
  try {
    mPubFusedCloud->publish(std::move(pointcloudFusedMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
#endif
}

void ZedCamera::callback_pubPaths()
{
  uint32_t mapPathSub = 0;
  uint32_t odomPathSub = 0;
  uint32_t utmPathSub = 0;

  try {
    mapPathSub = count_subscribers(mPosePathTopic);
    odomPathSub = count_subscribers(mOdomPathTopic);
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_PT("pubPaths: Exception while counting subscribers");
    return;
  }

  geometry_msgs::msg::PoseStamped odomPose;
  geometry_msgs::msg::PoseStamped mapPose;
  geometry_msgs::msg::PoseStamped utmPose;

  odomPose.header.stamp =
    mFrameTimestamp + rclcpp::Duration(0, mTfOffset * 1e9);
  odomPose.header.frame_id = mMapFrameId;    // map_frame
  odomPose.pose.position.x = mOdom2BaseTransf.getOrigin().x();
  odomPose.pose.position.y = mOdom2BaseTransf.getOrigin().y();
  odomPose.pose.position.z = mOdom2BaseTransf.getOrigin().z();
  odomPose.pose.orientation.x = mOdom2BaseTransf.getRotation().x();
  odomPose.pose.orientation.y = mOdom2BaseTransf.getRotation().y();
  odomPose.pose.orientation.z = mOdom2BaseTransf.getRotation().z();
  odomPose.pose.orientation.w = mOdom2BaseTransf.getRotation().w();

  mapPose.header.stamp =
    mFrameTimestamp + rclcpp::Duration(0, mTfOffset * 1e9);
  mapPose.header.frame_id = mMapFrameId;    // map_frame
  mapPose.pose.position.x = mMap2BaseTransf.getOrigin().x();
  mapPose.pose.position.y = mMap2BaseTransf.getOrigin().y();
  mapPose.pose.position.z = mMap2BaseTransf.getOrigin().z();
  mapPose.pose.orientation.x = mMap2BaseTransf.getRotation().x();
  mapPose.pose.orientation.y = mMap2BaseTransf.getRotation().y();
  mapPose.pose.orientation.z = mMap2BaseTransf.getRotation().z();
  mapPose.pose.orientation.w = mMap2BaseTransf.getRotation().w();

  if (mGnssFusionEnabled) {
    utmPose.header.stamp =
      mFrameTimestamp + rclcpp::Duration(0, mTfOffset * 1e9);
    utmPose.header.frame_id = mMapFrameId;    // mUtmFrameId;  // map_frame
    utmPose.pose.position.x = mMap2UtmTransf.getOrigin().x();
    utmPose.pose.position.y = mMap2UtmTransf.getOrigin().y();
    utmPose.pose.position.z = mMap2UtmTransf.getOrigin().z();
    utmPose.pose.orientation.x = mMap2UtmTransf.getRotation().x();
    utmPose.pose.orientation.y = mMap2UtmTransf.getRotation().y();
    utmPose.pose.orientation.z = mMap2UtmTransf.getRotation().z();
    utmPose.pose.orientation.w = mMap2UtmTransf.getRotation().w();
  }

  // Circular vector
  if (mPathMaxCount != -1) {
    if (mOdomPath.size() == mPathMaxCount) {
      DEBUG_STREAM_PT("Path vectors full: rotating ");
      std::rotate(mOdomPath.begin(), mOdomPath.begin() + 1, mOdomPath.end());
      std::rotate(mPosePath.begin(), mPosePath.begin() + 1, mPosePath.end());

      mPosePath[mPathMaxCount - 1] = mapPose;
      mOdomPath[mPathMaxCount - 1] = odomPose;
    } else {
      // DEBUG_STREAM_PT( "Path vectors adding last available poses");
      mPosePath.push_back(mapPose);
      mOdomPath.push_back(odomPose);
    }
  } else {
    // DEBUG_STREAM_PT( "No limit path vectors, adding last available
    // poses");
    mPosePath.push_back(mapPose);
    mOdomPath.push_back(odomPose);
  }

  if (mapPathSub > 0) {
    pathMsgPtr mapPathMsg = std::make_unique<nav_msgs::msg::Path>();
    mapPathMsg->header.frame_id = mMapFrameId;
    mapPathMsg->header.stamp = mFrameTimestamp;
    mapPathMsg->poses = mPosePath;

    DEBUG_STREAM_PT("Publishing MAP PATH message");
    try {
      mPubPosePath->publish(std::move(mapPathMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }

  if (odomPathSub > 0) {
    pathMsgPtr odomPathMsg = std::make_unique<nav_msgs::msg::Path>();
    odomPathMsg->header.frame_id = mOdomFrameId;
    odomPathMsg->header.stamp = mFrameTimestamp;
    odomPathMsg->poses = mOdomPath;

    DEBUG_STREAM_PT("Publishing ODOM PATH message");
    try {
      mPubOdomPath->publish(std::move(odomPathMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
}

void ZedCamera::callback_resetOdometry(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
  std::shared_ptr<std_srvs::srv::Trigger_Response> res)
{
  (void)request_header;
  (void)req;

  RCLCPP_INFO(get_logger(), "** Reset Odometry service called **");
  mResetOdomFromSrv = true;
  res->message = "Odometry reset OK";
  res->success = true;
}

void ZedCamera::callback_resetPosTracking(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
  std::shared_ptr<std_srvs::srv::Trigger_Response> res)
{
  (void)request_header;
  (void)req;

  RCLCPP_INFO(get_logger(), "** Reset Pos. Tracking service called **");

  if (!mPosTrackingStarted) {
    RCLCPP_WARN(get_logger(), "Pos. Tracking was not started");
    res->message = "Positional tracking not active";
    res->success = false;
    return;
  }

  mResetOdomFromSrv = true;
  mOdomPath.clear();
  mPosePath.clear();

  // Restart tracking
  startPosTracking();

  res->message = "Positional tracking reset OK";
  res->success = true;
}

void ZedCamera::callback_setPose(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<zed_interfaces::srv::SetPose_Request> req,
  std::shared_ptr<zed_interfaces::srv::SetPose_Response> res)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "** Set Pose service called **");

  if (mGnssFusionEnabled) {
    RCLCPP_WARN(
      get_logger(),
      "Service call not valid: GNSS fusion is enabled.");
    res->message = "GNSS fusion is enabled";
    res->success = false;
    return;
  }

  RCLCPP_INFO_STREAM(
    get_logger(),
    "New pose: [" << req->pos[0] << "," << req->pos[1] << ","
                  << req->pos[2] << ", " << req->orient[0]
                  << "," << req->orient[1] << ","
                  << req->orient[2] << "]");

  if (!mPosTrackingStarted) {
    RCLCPP_WARN(get_logger(), "Pos. Tracking was not active");
    res->message = "Positional tracking not active";
    res->success = false;
    return;
  }

  mInitialBasePose[0] = req->pos[0];
  mInitialBasePose[1] = req->pos[1];
  mInitialBasePose[2] = req->pos[2];

  mInitialBasePose[3] = req->orient[0];
  mInitialBasePose[4] = req->orient[1];
  mInitialBasePose[5] = req->orient[2];

  mResetOdomFromSrv = true;
  mOdomPath.clear();
  mPosePath.clear();

  // Restart tracking
  startPosTracking();

  res->message = "Positional Tracking new pose OK";
  res->success = true;
}

void ZedCamera::callback_enableObjDet(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
  std::shared_ptr<std_srvs::srv::SetBool_Response> res)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "** Enable Object Detection service called **");

  std::lock_guard<std::mutex> lock(mObjDetMutex);

  if (sl_tools::isZED(mCamRealModel)) {
    RCLCPP_WARN(get_logger(), "Object Detection not available for ZED");
    res->message = "Object Detection not available for ZED";
    res->success = false;
    return;
  }

  if (req->data) {
    RCLCPP_INFO(get_logger(), "Starting Object Detection");
    // Start
    if (mObjDetEnabled && mObjDetRunning) {
      RCLCPP_WARN(get_logger(), "Object Detection is already running");
      res->message = "Object Detection is already running";
      res->success = false;
      return;
    }

    mObjDetEnabled = true;

    if (startObjDetect()) {
      res->message = "Object Detection started";
      res->success = true;
      return;
    } else {
      res->message =
        "Error occurred starting Object Detection. Read the log for more info";
      res->success = false;
      return;
    }
  } else {
    RCLCPP_INFO(get_logger(), "Stopping Object Detection");
    // Stop
    if (!mObjDetEnabled || !mObjDetRunning) {
      RCLCPP_WARN(get_logger(), "Object Detection was not running");
      res->message = "Object Detection was not running";
      res->success = false;
      return;
    }

    stopObjDetect();

    res->message = "Object Detection stopped";
    res->success = true;
    return;
  }
}

void ZedCamera::callback_enableBodyTrk(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
  std::shared_ptr<std_srvs::srv::SetBool_Response> res)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "** Enable Body Tracking service called **");

  std::lock_guard<std::mutex> lock(mBodyTrkMutex);

  if (sl_tools::isZED(mCamRealModel)) {
    RCLCPP_WARN(get_logger(), "Body Tracking not available for ZED");
    res->message = "Body Tracking  not available for ZED";
    res->success = false;
    return;
  }

  if (req->data) {
    RCLCPP_INFO(get_logger(), "Starting Body Tracking");
    // Start
    if (mBodyTrkEnabled && mBodyTrkRunning) {
      RCLCPP_WARN(get_logger(), "Body Tracking is already running");
      res->message = "Body Tracking  is already running";
      res->success = false;
      return;
    }

    mBodyTrkEnabled = true;

    if (startBodyTracking()) {
      res->message = "Body Tracking started";
      res->success = true;
      return;
    } else {
      res->message =
        "Error occurred starting Body Tracking. Read the log for more info";
      res->success = false;
      return;
    }
  } else {
    RCLCPP_INFO(get_logger(), "Stopping Body Tracking");
    // Stop
    if (!mBodyTrkEnabled || !mBodyTrkRunning) {
      RCLCPP_WARN(get_logger(), "Body Tracking was not running");
      res->message = "Body Tracking was not running";
      res->success = false;
      return;
    }

    stopBodyTracking();

    res->message = "Body Tracking stopped";
    res->success = true;
    return;
  }
}

void ZedCamera::callback_enableMapping(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
  std::shared_ptr<std_srvs::srv::SetBool_Response> res)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "** Enable Spatial Mapping service called **");

  std::lock_guard<std::mutex> lock(mMappingMutex);

  if (req->data) {
    RCLCPP_INFO(get_logger(), "Starting Spatial Mapping");
    // Start
    if (mMappingEnabled && mSpatialMappingRunning) {
      RCLCPP_WARN(get_logger(), "Spatial Mapping is already running");
      res->message = "Spatial Mapping is already running";
      res->success = false;
      return;
    }

    mMappingEnabled = true;

    if (start3dMapping()) {
      res->message = "Spatial Mapping started";
      res->success = true;
      return;
    } else {
      res->message =
        "Error occurred starting Spatial Mapping. Read the log for more info";
      res->success = false;
      return;
    }
  } else {
    RCLCPP_INFO(get_logger(), "Stopping Spatial Mapping");
    // Stop
    if (!mMappingEnabled || !mSpatialMappingRunning) {
      RCLCPP_WARN(get_logger(), "Spatial Mapping was not running");
      res->message = "Spatial Mapping was not running";
      res->success = false;
      return;
    }

    stop3dMapping();

    res->message = "Spatial Mapping stopped";
    res->success = true;
    return;
  }
}

void ZedCamera::callback_enableStreaming(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
  std::shared_ptr<std_srvs::srv::SetBool_Response> res)
{
  (void)request_header;

  if (req->data) {

    if (mStreamingServerRunning) {
      RCLCPP_WARN(get_logger(), "A Streaming Server is already running");
      res->message = "A Streaming Server is already running";
      res->success = false;
      return;
    }
    // Start
    if (startStreamingServer()) {
      res->message = "Streaming Server started";
      res->success = true;
      return;
    } else {
      res->message =
        "Error occurred starting the Streaming Server. Read the log for more info";
      res->success = false;
      return;
    }
  } else {
    // Stop
    if (!mStreamingServerRunning) {
      RCLCPP_WARN(get_logger(), "There is no Streaming Server active to be stopped");
      res->message = "There is no Streaming Server active to be stopped";
      res->success = false;
      return;
    }

    RCLCPP_INFO(get_logger(), "Stopping the Streaming Server");
    stopStreamingServer();

    res->message = "Streaming Server stopped";
    res->success = true;
    return;
  }
}

void ZedCamera::callback_startSvoRec(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<zed_interfaces::srv::StartSvoRec_Request> req,
  std::shared_ptr<zed_interfaces::srv::StartSvoRec_Response> res)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "** Start SVO Recording service called **");

  if (mSvoMode) {
    RCLCPP_WARN(
      get_logger(),
      "Cannot start SVO recording while playing SVO as input");
    res->message = "Cannot start SVO recording while playing SVO as input";
    res->success = false;
    return;
  }

  std::lock_guard<std::mutex> lock(mRecMutex);

  if (mRecording) {
    RCLCPP_WARN(get_logger(), "SVO Recording is already enabled");
    res->message = "SVO Recording is already enabled";
    res->success = false;
    return;
  }

  mSvoRecBitrate = req->bitrate;
  if ((mSvoRecBitrate != 0) &&
    ((mSvoRecBitrate < 1000) || (mSvoRecBitrate > 60000)))
  {
    RCLCPP_WARN(
      get_logger(),
      "'bitrate' value not valid. Please use a value "
      "in range [1000,60000], or 0 for default");
    res->message =
      "'bitrate' value not valid. Please use a value in range "
      "[1000,60000], or 0 for default";
    res->success = false;
    return;
  }
  mSvoRecCompr = static_cast<sl::SVO_COMPRESSION_MODE>(req->compression_mode);
  if (mSvoRecCompr >= sl::SVO_COMPRESSION_MODE::LAST) {
    RCLCPP_WARN(
      get_logger(),
      "'compression_mode' mode not valid. Please use a value in "
      "range [0,2]");
    res->message =
      "'compression_mode' mode not valid. Please use a value in range "
      "[0,2]";
    res->success = false;
    return;
  }
  mSvoRecFramerate = req->target_framerate;
  mSvoRecTranscode = req->input_transcode;
  mSvoRecFilename = req->svo_filename;

  if (mSvoRecFilename.empty()) {
    mSvoRecFilename = "zed.svo";
  }

  std::string err;

  if (!startSvoRecording(err)) {
    res->message = "Error starting SVO recording: " + err;
    res->success = false;
    return;
  }

  RCLCPP_INFO(get_logger(), "SVO Recording started: ");
  RCLCPP_INFO_STREAM(get_logger(), " * Bitrate: " << mSvoRecBitrate);
  RCLCPP_INFO_STREAM(get_logger(), " * Compression: " << mSvoRecCompr);
  RCLCPP_INFO_STREAM(get_logger(), " * Framerate: " << mSvoRecFramerate);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Input Transcode: " << (mSvoRecTranscode ? "TRUE" : "FALSE"));
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Filename: " << (mSvoRecFilename.empty() ? "zed.svo" :
    mSvoRecFilename));

  res->message = "SVO Recording started";
  res->success = true;
}

void ZedCamera::callback_stopSvoRec(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
  std::shared_ptr<std_srvs::srv::Trigger_Response> res)
{
  (void)request_header;
  (void)req;

  RCLCPP_INFO(get_logger(), "** Stop SVO Recording service called **");

  std::lock_guard<std::mutex> lock(mRecMutex);

  if (!mRecording) {
    RCLCPP_WARN(get_logger(), "SVO Recording is NOT enabled");
    res->message = "SVO Recording is NOT enabled";
    res->success = false;
    return;
  }

  stopSvoRecording();

  RCLCPP_WARN(get_logger(), "SVO Recording stopped");
  res->message = "SVO Recording stopped";
  res->success = true;
}

void ZedCamera::callback_pauseSvoInput(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
  std::shared_ptr<std_srvs::srv::Trigger_Response> res)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "** Pause SVO Input service called **");

  std::lock_guard<std::mutex> lock(mRecMutex);

  if (!mSvoMode) {
    RCLCPP_WARN(get_logger(), "The node is not using an SVO as input");
    res->message = "The node is not using an SVO as inpu";
    res->success = false;
    return;
  }

  if (mSvoRealtime) {
    RCLCPP_WARN(
      get_logger(),
      "SVO input can be paused only if SVO is not in RealTime mode");
    res->message =
      "SVO input can be paused only if SVO is not in RealTime mode";
    res->success = false;
    mSvoPause = false;
    return;
  }

  if (!mSvoPause) {
    RCLCPP_WARN(get_logger(), "SVO is paused");
    res->message = "SVO is paused";
    mSvoPause = true;
  } else {
    RCLCPP_WARN(get_logger(), "SVO is playing");
    res->message = "SVO is playing";
    mSvoPause = false;
  }
  res->success = true;
}

void ZedCamera::callback_updateDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  DEBUG_COMM("*** Update Diagnostic ***");

  if (mConnStatus != sl::ERROR_CODE::SUCCESS) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      sl::toString(mConnStatus).c_str());
    return;
  }

  if (mGrabStatus == sl::ERROR_CODE::SUCCESS) {
    double freq = 1. / mGrabPeriodMean_sec->getAvg();
    double freq_perc = 100. * freq / mPubFrameRate;
    stat.addf("Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);

    double frame_proc_sec = mElabPeriodMean_sec->getAvg();
    // double frame_grab_period = 1. / mCamGrabFrameRate;
    double frame_grab_period = 1. / mPubFrameRate;
    stat.addf(
      "Capture", "Tot. Processing Time: %.6f sec (Max. %.3f sec)",
      frame_proc_sec, frame_grab_period);

    if (frame_proc_sec > frame_grab_period) {
      mSysOverloadCount++;
    }

    if (mSysOverloadCount >= 10) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "System overloaded. Consider reducing "
        "'general.pub_frame_rate' or 'general.grab_resolution'");
    } else {
      mSysOverloadCount = 0;
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Camera grabbing");
    }

    if (mSimMode) {
      stat.add("Input mode", "SIMULATION");
    } else if (mSvoMode) {
      stat.add("Input mode", "SVO");
    }
    if (mStreamMode) {
      stat.add("Input mode", "LOCAL STREAM");
    } else {
      stat.add("Input mode", "Live Camera");
    }

    if (mVdPublishing) {
      freq = 1. / mVideoDepthPeriodMean_sec->getAvg();
      freq_perc = 100. * freq / mPubFrameRate;
      frame_grab_period = 1. / mPubFrameRate;
      stat.addf(
        "Video/Depth", "Mean Frequency: %.1f Hz (%.1f%%)", freq,
        freq_perc);
      stat.addf(
        "Video/Depth", "Processing Time: %.6f sec (Max. %.3f sec)",
        mVideoDepthElabMean_sec->getAvg(), frame_grab_period);
    } else {
      stat.add("Video/Depth", "Topic not subscribed");
    }

    if (mSvoMode) {
      int frame = mZed->getSVOPosition();
      int totFrames = mZed->getSVONumberOfFrames();
      double svo_perc = 100. * (static_cast<double>(frame) / totFrames);

      stat.addf(
        "Playing SVO", "Frame: %d/%d (%.1f%%)", frame, totFrames,
        svo_perc);
    }

    if (isDepthRequired()) {
      stat.add("Depth status", "ACTIVE");
      stat.add("Depth mode", sl::toString(mDepthMode).c_str());

      if (mPcPublishing) {
        freq = 1. / mPcPeriodMean_sec->getAvg();
        freq_perc = 100. * freq / mPcPubRate;
        stat.addf(
          "Point Cloud", "Mean Frequency: %.1f Hz (%.1f%%)", freq,
          freq_perc);
        stat.addf(
          "Point Cloud", "Processing Time: %.3f sec (Max. %.3f sec)",
          mPcProcMean_sec->getAvg(), 1. / mPcPubRate);
      } else {
        stat.add("Point Cloud", "Topic not subscribed");
      }

      if (mFloorAlignment) {
        if (mPosTrackingStatus.spatial_memory_status == sl::SPATIAL_MEMORY_STATUS::SEARCHING) {
          stat.add("Floor Detection", "NOT INITIALIZED");
        } else {
          stat.add("Floor Detection", "INITIALIZED");
        }
      }

      if (mAutoRoiEnabled) {
        stat.add("Automatic ROI", sl::toString(mAutoRoiStatus).c_str());
      } else if (mManualRoiEnabled) {
        stat.add("Manual ROI", "ENABLED");
      }

      if (mGnssFusionEnabled) {
        stat.addf("Fusion status", sl::toString(mFusionStatus).c_str());
        if (mPosTrackingStarted) {
          stat.addf(
            "GNSS Tracking status", "%s",
            sl::toString(mFusedPosTrackingStatus.gnss_fusion_status).c_str());
        }
        if (mGnssMsgReceived) {
          freq = 1. / mGnssFix_sec->getAvg();
          stat.addf("GNSS input", "Mean Frequency: %.1f Hz", freq);
          stat.addf("GNSS Services", "%s", mGnssServiceStr.c_str());
          if (mGnssFixValid) {
            stat.add("GNSS Status", "FIX OK");
          } else {
            stat.add("GNSS Status", "NO FIX");
          }
        } else {
          stat.add("GNSS Fusion", "Waiting for GNSS messages");
        }
      } else {
        stat.add("GNSS Fusion", "DISABLED");
      }

      if (mPosTrackingStarted) {
        stat.addf(
          "Odometry tracking status", "%s",
          sl::toString(mPosTrackingStatus.odometry_status)
          .c_str());
        stat.addf(
          "Spatial Memory status", "%s",
          sl::toString(mPosTrackingStatus.spatial_memory_status).c_str());

        if (mPublishTF) {
          freq = 1. / mPubOdomTF_sec->getAvg();
          stat.addf("TF Odometry", "Mean Frequency: %.1f Hz", freq);

          if (mPublishMapTF) {
            freq = 1. / mPubPoseTF_sec->getAvg();
            stat.addf("TF Pose", "Mean Frequency: %.1f Hz", freq);
          } else {
            stat.add("TF Pose", "DISABLED");
          }
        } else {
          stat.add("TF Odometry", "DISABLED");
          stat.add("TF Pose", "DISABLED");
        }
      } else {
        stat.add("Pos. Tracking status", "INACTIVE");
      }

      if (mObjDetRunning) {
        if (mObjDetSubscribed) {
          freq = 1. / mObjDetPeriodMean_sec->getAvg();
          freq_perc = 100. * freq / mPubFrameRate;
          frame_grab_period = 1. / mPubFrameRate;
          stat.addf(
            "Object detection", "Mean Frequency: %.3f Hz  (%.1f%%)",
            freq, freq_perc);
          stat.addf(
            "Object detection",
            "Processing Time: %.3f sec (Max. %.3f sec)",
            mObjDetElabMean_sec->getAvg(), frame_grab_period);
        } else {
          stat.add("Object Detection", "Active, topic not subscribed");
        }
      } else {
        stat.add("Object Detection", "INACTIVE");
      }

      if (mBodyTrkRunning) {
        if (mBodyTrkSubscribed) {
          freq = 1. / mBodyTrkPeriodMean_sec->getAvg();
          freq_perc = 100. * freq / mPubFrameRate;
          frame_grab_period = 1. / mPubFrameRate;
          stat.addf(
            "Body Tracking", "Mean Frequency: %.3f Hz  (%.1f%%)",
            freq, freq_perc);
          stat.addf(
            "Body Tracking",
            "Processing Time: %.3f sec (Max. %.3f sec)",
            mBodyTrkElabMean_sec->getAvg(), frame_grab_period);
        } else {
          stat.add("Body Tracking", "Active, topic not subscribed");
        }
      } else {
        stat.add("Body Tracking", "INACTIVE");
      }
    } else {
      stat.add("Depth status", "INACTIVE");
    }

    if (mPublishImuTF) {
      freq = 1. / mPubImuTF_sec->getAvg();
      stat.addf("TF IMU", "Mean Frequency: %.1f Hz", freq);
    } else {
      stat.add("TF IMU", "DISABLED");
    }
  } else if (mGrabStatus == sl::ERROR_CODE::LAST) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Camera initializing");
  } else {
    stat.summaryf(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Camera error: %s", sl::toString(mGrabStatus).c_str());
  }

  if (mImuPublishing) {
    double freq = 1. / mImuPeriodMean_sec->getAvg();
    stat.addf("IMU", "Mean Frequency: %.1f Hz", freq);
  } else {
    stat.add("IMU Sensor", "Topics not subscribed");
  }

  if (!mSvoMode && !mSimMode && sl_tools::isZED2OrZED2i(mCamRealModel)) {
    if (mMagPublishing) {
      double freq = 1. / mMagPeriodMean_sec->getAvg();
      stat.addf("Magnetometer", "Mean Frequency: %.1f Hz", freq);
    } else {
      stat.add("Magnetometer Sensor", "Topics not subscribed");
    }
  } else {
    stat.add("Magnetometer Sensor", "N/A");
  }

  if (!mSvoMode && !mSimMode && sl_tools::isZED2OrZED2i(mCamRealModel)) {
    if (mBaroPublishing) {
      double freq = 1. / mBaroPeriodMean_sec->getAvg();
      stat.addf("Barometer", "Mean Frequency: %.1f Hz", freq);
    } else {
      stat.add("Barometer Sensor", "Topics not subscribed");
    }
  } else {
    stat.add("Barometer Sensor", "N/A");
  }

  if (!mSvoMode && !mSimMode && sl_tools::isZED2OrZED2i(mCamRealModel)) {
    stat.addf("Left CMOS Temp.", "%.1f °C", mTempLeft);
    stat.addf("Right CMOS Temp.", "%.1f °C", mTempRight);

    if (mTempLeft > 70.f || mTempRight > 70.f) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "High Camera temperature");
    }
  } else {
    stat.add("Left CMOS Temp.", "N/A");
    stat.add("Right CMOS Temp.", "N/A");
  }

  if (!mSvoMode && !mSimMode && sl_tools::isZEDX(mCamRealModel)) {
    stat.addf("Camera Temp.", "%.1f °C", mTempImu);

    if (mTempImu > 70.f) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "High Camera temperature");
    }
  }

  if (mRecording) {
    if (!mRecStatus.status) {
      // if (mGrabActive)
      {
        stat.add("SVO Recording", "ERROR");
        stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "Error adding frames to SVO file while recording. "
          "Check "
          "free disk space");
      }
    } else {
      stat.add("SVO Recording", "ACTIVE");
      stat.addf(
        "SVO compression time", "%g msec",
        mRecStatus.average_compression_time);
      stat.addf(
        "SVO compression ratio", "%.1f%%",
        mRecStatus.average_compression_ratio);
    }
  } else {
    stat.add("SVO Recording", "NOT ACTIVE");
  }

  if (mStreamingServerRunning) {
    stat.add("Streaming Server", "ACTIVE");

    sl::StreamingParameters params;
    params = mZed->getStreamingParameters();

    stat.addf("Streaming port", "%d", static_cast<int>(params.port));
    stat.addf(
      "Streaming codec", "%s",
      (params.codec == sl::STREAMING_CODEC::H264 ? "H264" : "H265"));
    stat.addf("Streaming bitrate", "%d mbps", static_cast<int>(params.bitrate));
    stat.addf("Streaming chunk size", "%d B", static_cast<int>(params.chunk_size));
    stat.addf("Streaming GOP size", "%d", static_cast<int>(params.gop_size));
  } else {
    stat.add("Streaming Server", "NOT ACTIVE");
  }
}

void ZedCamera::callback_gnssPubTimerTimeout()
{
  if (!mGnssMsgReceived) {
    return;
  }

  mGnssMsgReceived = false;

  RCLCPP_WARN(
    get_logger(),
    "* GNSS subscriber timeout. No GNSS data available.");

  mGnssPubCheckTimer.reset();
}

void ZedCamera::processSvoGnssData()
{
  if (!mGnssReplay) {
    mGnssFixValid = false;
    return;
  }
  uint64_t current_ts = mLastZedPose.timestamp.getNanoseconds();
  static uint64_t old_gnss_ts = 0;

  if (current_ts == old_gnss_ts) {
    return;
  }
  old_gnss_ts = current_ts;

  // DEBUG_STREAM_GNSS("Retrieving GNSS data from SVO. TS: " << current_ts
  //                                                         << " nsec");

  sl::GNSSData gnssData;
  auto err = mGnssReplay->grab(gnssData, current_ts);
  if (err != sl::FUSION_ERROR_CODE::SUCCESS) {
    //DEBUG_STREAM_GNSS("Error grabbing GNSS data from SVO: " << sl::toString(err));
    return;
  }

  mGnssMsgReceived = true;

  // ----> GNSS Fix stats
  double elapsed_sec = mGnssFixFreqTimer.toc();
  mGnssFix_sec->addValue(elapsed_sec);
  mGnssFixFreqTimer.tic();
  // <---- GNSS Fix stats

  double latitude;
  double longitude;
  double altitude;

  gnssData.getCoordinates(latitude, longitude, altitude, false);
  mGnssTimestamp = rclcpp::Time(gnssData.ts.getNanoseconds(), RCL_ROS_TIME);

  DEBUG_STREAM_GNSS(
    "Latest GNSS data from SVO - ["
      << mGnssTimestamp.nanoseconds() << " nsec] " << latitude
      << "°," << longitude << "° / " << altitude << " m");

  std::stringstream ss;
  for (size_t i = 0; i < gnssData.position_covariance.size(); i++) {
    ss << gnssData.position_covariance[i];
    if (i != gnssData.position_covariance.size() - 1) {ss << ", ";}
  }
  DEBUG_STREAM_GNSS("Covariance- [" << ss.str() << "]");
  DEBUG_STREAM_GNSS("GNSS Status: " << sl::toString(gnssData.gnss_status));

  if (gnssData.gnss_status == sl::GNSS_STATUS::UNKNOWN) {
    gnssData.gnss_status = sl::GNSS_STATUS::SINGLE;
    DEBUG_STREAM_GNSS(" * Forced to: " << sl::toString(gnssData.gnss_status));
  }

  DEBUG_STREAM_GNSS("GNSS Mode: " << sl::toString(gnssData.gnss_mode));

  mGnssFixValid = true;   // Used to keep track of signal loss

  if (mZed->isOpened() && mZed->isPositionalTrackingEnabled()) {
    auto ingest_error = mFusion.ingestGNSSData(gnssData);
    if (ingest_error == sl::FUSION_ERROR_CODE::SUCCESS) {
      DEBUG_STREAM_GNSS(
        "Datum ingested - ["
          << mGnssTimestamp.nanoseconds() << " nsec] " << latitude
          << "°," << longitude << "° / " << altitude << " m");
    } else {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Ingest error occurred when ingesting GNSSData: "
          << sl::toString(ingest_error));
    }
    mGnssFixNew = true;
  }
}

void ZedCamera::callback_gnssFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  sl::GNSSData gnssData;

  // ----> GNSS Fix stats
  double elapsed_sec = mGnssFixFreqTimer.toc();
  mGnssFix_sec->addValue(elapsed_sec);
  mGnssFixFreqTimer.tic();
  // <---- GNSS Fix stats

  if (!mGnssPubCheckTimer) {
    std::chrono::milliseconds gnssTimeout_msec(static_cast<int>(2000.0));
    mGnssPubCheckTimer = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        gnssTimeout_msec),
      std::bind(&ZedCamera::callback_gnssPubTimerTimeout, this));
  } else {
    mGnssPubCheckTimer->reset();
  }

  mGnssMsgReceived = true;

  RCLCPP_INFO_ONCE(get_logger(), "*** GNSS subscriber ***");
  RCLCPP_INFO_ONCE(
    get_logger(),
    " * First message received. GNSS Sender active.");

  mGnssServiceStr = "";
  mGnssService = msg->status.service;
  if (mGnssService & sensor_msgs::msg::NavSatStatus::SERVICE_GPS) {
    mGnssServiceStr += "GPS ";
  }
  if (mGnssService & sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS) {
    mGnssServiceStr += "GLONASS ";
  }
  if (mGnssService & sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS) {
    mGnssServiceStr += "COMPASS ";
  }
  if (mGnssService & sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO) {
    mGnssServiceStr += "GALILEO";
  }

  RCLCPP_INFO_STREAM_ONCE(
    get_logger(),
    " * Service: " << mGnssServiceStr.c_str());

  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    DEBUG_GNSS("callback_gnssFix: fix not valid");
    if (mGnssFixValid) {
      RCLCPP_INFO(get_logger(), "GNSS: signal lost.");
    }
    mGnssFixValid = false;

    return;
  }

  switch (msg->status.status) {
    case ::sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX:
      gnssData.gnss_status = sl::GNSS_STATUS::RTK_FIX;
      break;
    case ::sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX:
      gnssData.gnss_status = sl::GNSS_STATUS::RTK_FLOAT;
      break;
    case ::sensor_msgs::msg::NavSatStatus::STATUS_FIX:
    default:
      gnssData.gnss_status = sl::GNSS_STATUS::SINGLE;
      break;
  }

  // ----> Check timestamp
  // Note: this is the ROS timestamp, not the GNSS timestamp that is available
  // in a
  //       "sensor_msgs/TimeReference message", e.g. `/time_reference`
  uint64_t ts_gnss_part_sec =
    static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000;
  uint64_t ts_gnss_part_nsec =
    static_cast<uint64_t>(msg->header.stamp.nanosec);
  uint64_t ts_gnss_nsec = ts_gnss_part_sec + ts_gnss_part_nsec;

  DEBUG_STREAM_GNSS(
    "GNSS Ts: " << ts_gnss_part_sec / 1000000000 << " sec + "
                << ts_gnss_part_nsec << " nsec = "
                << ts_gnss_nsec << " nsec fused");

  if (ts_gnss_nsec <= mLastTs_gnss_nsec) {
    DEBUG_GNSS(
      "callback_gnssFix: data not valid (timestamp did not increment)");
    return;
  }

  DEBUG_STREAM_GNSS(
    "GNSS dT: " << ts_gnss_nsec - mLastTs_gnss_nsec
                << " nsec");
  mLastTs_gnss_nsec = ts_gnss_nsec;
  // <---- Check timestamp

  mGnssTimestamp = rclcpp::Time(ts_gnss_nsec, RCL_ROS_TIME);
  DEBUG_STREAM_GNSS("Stored Ts: " << mGnssTimestamp.nanoseconds());

  double altit = msg->altitude;
  if (mGnssZeroAltitude) {
    altit = 0.0;
  }
  double latit = msg->latitude;
  double longit = msg->longitude;

  // std::lock_guard<std::mutex> lock(mGnssDataMutex);
  gnssData.ts.setNanoseconds(ts_gnss_nsec);
  gnssData.setCoordinates(latit, longit, altit, false);

  if (msg->position_covariance_type !=
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
  {
    gnssData.latitude_std = msg->position_covariance[0];
    gnssData.longitude_std = msg->position_covariance[1 * 3 + 1];
    gnssData.altitude_std = msg->position_covariance[2 * 3 + 2];
    if (mGnssZeroAltitude) {
      gnssData.altitude_std = 1e-9;
    }
    std::array<double, 9> position_covariance;
    position_covariance[0] = gnssData.latitude_std * mGnssHcovMul;    // X
    position_covariance[1 * 3 + 1] =
      gnssData.longitude_std * mGnssHcovMul;      // Y
    position_covariance[2 * 3 + 2] =
      gnssData.altitude_std * mGnssVcovMul;      // Z
    gnssData.position_covariance = position_covariance;
  }

  if (!mGnssFixValid) {
    DEBUG_GNSS("GNSS: valid fix received.");
    DEBUG_STREAM_GNSS(
      " * First valid datum - Lat: "
        << std::fixed << std::setprecision(9) << latit
        << "° - Long: " << longit << "° - Alt: " << altit
        << " m");
  }

  mGnssFixValid = true;    // Used to keep track of signal loss

  if (mZed->isOpened() && mZed->isPositionalTrackingEnabled()) {
    auto ingest_error = mFusion.ingestGNSSData(gnssData);
    if (ingest_error == sl::FUSION_ERROR_CODE::SUCCESS) {
      DEBUG_STREAM_GNSS(
        "Datum ingested - ["
          << mGnssTimestamp.nanoseconds() << " nsec] " << latit
          << "°," << longit << "° / " << altit << " m");
    } else {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Ingest error occurred when ingesting GNSSData: "
          << sl::toString(ingest_error));
    }
    mGnssFixNew = true;
  }
}

void ZedCamera::callback_clickedPoint(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // ----> Check for result subscribers
  size_t markerSubNumber = 0;
  size_t planeSubNumber = 0;
  try {
    markerSubNumber = count_subscribers(mPubMarker->get_topic_name());
    planeSubNumber = count_subscribers(mPubPlane->get_topic_name());
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_MAP(
      "callback_clickedPoint: Exception while counting point plane "
      "subscribers");
    return;
  }

  if ((markerSubNumber + planeSubNumber) == 0) {
    return;
  }
  // <---- Check for result subscribers

  rclcpp::Time ts = get_clock()->now();

  float X = msg->point.x;
  float Y = msg->point.y;
  float Z = msg->point.z;

  RCLCPP_INFO_STREAM(
    get_logger(), "Clicked 3D point [X FW, Y LF, Z UP]: ["
      << X << "," << Y << "," << Z << "]");

  // ----> Transform the point from `map` frame to `left_camera_optical_frame`
  double camX, camY, camZ;
  try {
    // Save the transformation
    geometry_msgs::msg::TransformStamped m2o =
      mTfBuffer->lookupTransform(
      mLeftCamOptFrameId, msg->header.frame_id,
      TIMEZERO_SYS, rclcpp::Duration(1, 0));

    RCLCPP_INFO(
      get_logger(),
      "'%s' -> '%s': {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f,%.3f}",
      msg->header.frame_id.c_str(), mLeftCamOptFrameId.c_str(),
      m2o.transform.translation.x, m2o.transform.translation.y,
      m2o.transform.translation.z, m2o.transform.rotation.x,
      m2o.transform.rotation.y, m2o.transform.rotation.z,
      m2o.transform.rotation.w);

    // Get the TF2 transformation
    geometry_msgs::msg::PointStamped ptCam;

    tf2::doTransform(*(msg.get()), ptCam, m2o);

    camX = ptCam.point.x;
    camY = ptCam.point.y;
    camZ = ptCam.point.z;

    RCLCPP_INFO(
      get_logger(),
      "Point in camera coordinates [Z FW, X RG, Y DW]: {%.3f,%.3f,%.3f}",
      camX, camY, camZ);
  } catch (tf2::TransformException & ex) {
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock, 1000.0,
      "Transform error: %s", ex.what());
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock, 1000.0,
      "The tf from '%s' to '%s' is not available.",
      msg->header.frame_id.c_str(),
      mLeftCamOptFrameId.c_str());

    return;
  }
  // <---- Transform the point from `map` frame to `left_camera_optical_frame`

  // ----> Project the point into 2D image coordinates
  sl::CalibrationParameters zedParam;
  zedParam = mZed->getCameraInformation(mMatResol)
    .camera_configuration.calibration_parameters;                 // ok

  float f_x = zedParam.left_cam.fx;
  float f_y = zedParam.left_cam.fy;
  float c_x = zedParam.left_cam.cx;
  float c_y = zedParam.left_cam.cy;

  float out_scale_factor = static_cast<float>(mMatResol.width) / mCamWidth;

  float u = ((camX / camZ) * f_x + c_x) / out_scale_factor;
  float v = ((camY / camZ) * f_y + c_y) / out_scale_factor;
  DEBUG_STREAM_MAP(
    "Clicked point image coordinates: ["
      << u << "," << v
      << "] - out_scale_factor: " << out_scale_factor);
  // <---- Project the point into 2D image coordinates

  // ----> Extract plane from clicked point
  sl::Plane plane;
  sl::PlaneDetectionParameters params;
  params.max_distance_threshold = mPdMaxDistanceThreshold;
  params.normal_similarity_threshold = mPdNormalSimilarityThreshold;
  sl::ERROR_CODE err = mZed->findPlaneAtHit(sl::uint2(u, v), plane, params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN(
      get_logger(),
      "Error extracting plane at point [%.3f,%.3f,%.3f]: %s", X, Y,
      Z, sl::toString(err).c_str());
    return;
  }

  sl::float3 center = plane.getCenter();
  sl::float2 dims = plane.getExtents();

  if (dims[0] == 0 || dims[1] == 0) {
    RCLCPP_INFO(
      get_logger(), "Plane not found at point [%.3f,%.3f,%.3f]", X,
      Y, Z);
    return;
  }

  DEBUG_MAP(
    "Found plane at point [%.3f,%.3f,%.3f] -> Center: [%.3f,%.3f,%.3f], "
    "Dims: %.3fx%.3f",
    X, Y, Z, center.x, center.y, center.z, dims[0], dims[1]);
  // <---- Extract plane from clicked point

  if (markerSubNumber > 0) {
    // ----> Publish a blue sphere in the clicked point
    markerMsgPtr pt_marker =
      std::make_unique<visualization_msgs::msg::Marker>();
    // Set the frame ID and timestamp.  See the TF tutorials for information
    // on these.
    static int hit_pt_id =
      0;      // This ID must be unique in the same process. Thus it is good to
              // keep it as a static variable
    pt_marker->header.stamp = ts;
    // Set the marker action.  Options are ADD and DELETE
    pt_marker->action = visualization_msgs::msg::Marker::ADD;
    pt_marker->lifetime = rclcpp::Duration(0, 0);

    // Set the namespace and id for this marker.  This serves to create a
    // unique ID Any marker sent with the same namespace and id will overwrite
    // the old one
    pt_marker->ns = "plane_hit_points";
    pt_marker->id = hit_pt_id++;
    pt_marker->header.frame_id = mMapFrameId;

    // Set the marker type.
    pt_marker->type = visualization_msgs::msg::Marker::SPHERE;

    // Set the pose of the marker.
    // This is a full 6DOF pose relative to the frame/time specified in the
    // header
    pt_marker->pose.position.x = X;
    pt_marker->pose.position.y = Y;
    pt_marker->pose.position.z = Z;
    pt_marker->pose.orientation.x = 0.0;
    pt_marker->pose.orientation.y = 0.0;
    pt_marker->pose.orientation.z = 0.0;
    pt_marker->pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    pt_marker->scale.x = 0.025;
    pt_marker->scale.y = 0.025;
    pt_marker->scale.z = 0.025;

    // Set the color -- be sure to set alpha to something non-zero!
    pt_marker->color.r = 0.2f;
    pt_marker->color.g = 0.1f;
    pt_marker->color.b = 0.75f;
    pt_marker->color.a = 0.8;

    // Publish the marker
    DEBUG_STREAM_MAP("Publishing PT MARKER message");
    try {
      mPubMarker->publish(std::move(pt_marker));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
    // ----> Publish a blue sphere in the clicked point

    // ----> Publish the plane as green mesh
    markerMsgPtr plane_marker =
      std::make_unique<visualization_msgs::msg::Marker>();
    // Set the frame ID and timestamp.  See the TF tutorials for information
    // on these.
    static int plane_mesh_id =
      0;      // This ID must be unique in the same process. Thus it is good to
              // keep it as a static variable
    plane_marker->header.stamp = ts;
    // Set the marker action.  Options are ADD and DELETE
    plane_marker->action = visualization_msgs::msg::Marker::ADD;
    plane_marker->lifetime = rclcpp::Duration(0, 0);

    // Set the namespace and id for this marker.  This serves to create a
    // unique ID Any marker sent with the same namespace and id will overwrite
    // the old one
    plane_marker->ns = "plane_meshes";
    plane_marker->id = plane_mesh_id++;
    plane_marker->header.frame_id = mLeftCamFrameId;

    // Set the marker type.
    plane_marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

    // Set the pose of the marker.  This isplane_marker
    plane_marker->pose.orientation.w = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    plane_marker->color.r = 0.10f;
    plane_marker->color.g = 0.75f;
    plane_marker->color.b = 0.20f;
    plane_marker->color.a = 0.75;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    plane_marker->scale.x = 1.0;
    plane_marker->scale.y = 1.0;
    plane_marker->scale.z = 1.0;

    sl::Mesh mesh = plane.extractMesh();
    size_t triangCount = mesh.getNumberOfTriangles();
    size_t ptCount = triangCount * 3;
    plane_marker->points.resize(ptCount);
    plane_marker->colors.resize(ptCount);

    size_t ptIdx = 0;
    for (size_t t = 0; t < triangCount; t++) {
      for (int p = 0; p < 3; p++) {
        uint vIdx = mesh.triangles[t][p];
        plane_marker->points[ptIdx].x = mesh.vertices[vIdx][0];
        plane_marker->points[ptIdx].y = mesh.vertices[vIdx][1];
        plane_marker->points[ptIdx].z = mesh.vertices[vIdx][2];

        // Set the color -- be sure to set alpha to something non-zero!
        plane_marker->colors[ptIdx].r = 0.10f;
        plane_marker->colors[ptIdx].g = 0.75f;
        plane_marker->colors[ptIdx].b = 0.20f;
        plane_marker->colors[ptIdx].a = 0.75;

        ptIdx++;
      }
    }

    // Publish the marker
    DEBUG_STREAM_MAP("Publishing PLANE MARKER message");
    try {
      mPubMarker->publish(std::move(plane_marker));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
    // <---- Publish the plane as green mesh
  }

  if (planeSubNumber > 0) {
    // ----> Publish the plane as custom message

    planeMsgPtr planeMsg =
      std::make_unique<zed_interfaces::msg::PlaneStamped>();
    planeMsg->header.stamp = ts;
    planeMsg->header.frame_id = mLeftCamFrameId;

    // Plane equation
    sl::float4 sl_coeff = plane.getPlaneEquation();
    planeMsg->coefficients.coef[0] = static_cast<double>(sl_coeff[0]);
    planeMsg->coefficients.coef[1] = static_cast<double>(sl_coeff[1]);
    planeMsg->coefficients.coef[2] = static_cast<double>(sl_coeff[2]);
    planeMsg->coefficients.coef[3] = static_cast<double>(sl_coeff[3]);

    // Plane Normal
    sl::float3 sl_normal = plane.getNormal();
    planeMsg->normal.x = sl_normal[0];
    planeMsg->normal.y = sl_normal[1];
    planeMsg->normal.z = sl_normal[2];

    // Plane Center
    sl::float3 sl_center = plane.getCenter();
    planeMsg->center.x = sl_center[0];
    planeMsg->center.y = sl_center[1];
    planeMsg->center.z = sl_center[2];

    // Plane extents
    sl::float3 sl_extents = plane.getExtents();
    planeMsg->extents[0] = sl_extents[0];
    planeMsg->extents[1] = sl_extents[1];

    // Plane pose
    sl::Pose sl_pose = plane.getPose();
    sl::Orientation sl_rot = sl_pose.getOrientation();
    sl::Translation sl_tr = sl_pose.getTranslation();

    planeMsg->pose.rotation.x = sl_rot.ox;
    planeMsg->pose.rotation.y = sl_rot.oy;
    planeMsg->pose.rotation.z = sl_rot.oz;
    planeMsg->pose.rotation.w = sl_rot.ow;

    planeMsg->pose.translation.x = sl_tr.x;
    planeMsg->pose.translation.y = sl_tr.y;
    planeMsg->pose.translation.z = sl_tr.z;

    // Plane Bounds
    std::vector<sl::float3> sl_bounds = plane.getBounds();
    planeMsg->bounds.points.resize(sl_bounds.size());
    memcpy(
      planeMsg->bounds.points.data(), sl_bounds.data(),
      3 * sl_bounds.size() * sizeof(float));

    // Plane mesh
    sl::Mesh sl_mesh = plane.extractMesh();
    size_t triangCount = sl_mesh.triangles.size();
    size_t ptsCount = sl_mesh.vertices.size();
    planeMsg->mesh.triangles.resize(triangCount);
    planeMsg->mesh.vertices.resize(ptsCount);

    // memcpy not allowed because data types are different
    for (size_t i = 0; i < triangCount; i++) {
      planeMsg->mesh.triangles[i].vertex_indices[0] = sl_mesh.triangles[i][0];
      planeMsg->mesh.triangles[i].vertex_indices[1] = sl_mesh.triangles[i][1];
      planeMsg->mesh.triangles[i].vertex_indices[2] = sl_mesh.triangles[i][2];
    }

    // memcpy not allowed because data types are different
    for (size_t i = 0; i < ptsCount; i++) {
      planeMsg->mesh.vertices[i].x = sl_mesh.vertices[i][0];
      planeMsg->mesh.vertices[i].y = sl_mesh.vertices[i][1];
      planeMsg->mesh.vertices[i].z = sl_mesh.vertices[i][2];
    }

    DEBUG_STREAM_MAP("Publishing PLANE message");
    try {
      mPubPlane->publish(std::move(planeMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
    // <---- Publish the plane as custom message
  }
}

void ZedCamera::callback_setRoi(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<zed_interfaces::srv::SetROI_Request> req,
  std::shared_ptr<zed_interfaces::srv::SetROI_Response> res)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "** Set ROI service called **");

  if (mDepthDisabled) {
    std::string err_msg =
      "Error while setting ZED SDK region of interest: depth processing is "
      "disabled!";

    RCLCPP_WARN_STREAM(get_logger(), " * " << err_msg);

    res->message = err_msg;
    res->success = false;
    return;
  }

  RCLCPP_INFO_STREAM(get_logger(), " * ROI string: " << req->roi.c_str());

  if (req->roi == "") {
    std::string err_msg =
      "Error while setting ZED SDK region of interest: a vector of "
      "normalized points describing a "
      "polygon is required. e.g. "
      "'[[0.5,0.25],[0.75,0.5],[0.5,0.75],[0.25,0.5]]'";

    RCLCPP_WARN_STREAM(get_logger(), " * " << err_msg);

    res->message = err_msg;
    res->success = false;
    return;
  }

  std::string error;
  std::vector<std::vector<float>> parsed_poly =
    sl_tools::parseStringVector(req->roi, error);

  if (error != "") {
    std::string err_msg = "Error while setting ZED SDK region of interest: ";
    err_msg += error;

    RCLCPP_WARN_STREAM(get_logger(), " * " << err_msg);

    res->message = err_msg;
    res->success = false;
    return;
  }

  // ----> Set Region of Interest
  // Create mask
  RCLCPP_INFO(get_logger(), " * Setting ROI");
  std::vector<sl::float2> sl_poly;
  parseRoiPoly(parsed_poly, sl_poly);

  sl::Resolution resol(mCamWidth, mCamHeight);
  sl::Mat roi_mask(resol, sl::MAT_TYPE::U8_C1, sl::MEM::CPU);
  if (!sl_tools::generateROI(sl_poly, roi_mask)) {
    std::string err_msg =
      "Error generating the region of interest image mask. ";
    err_msg += error;

    RCLCPP_WARN_STREAM(get_logger(), "  * " << err_msg);

    res->message = err_msg;
    res->success = false;
    return;
  } else {
    sl::ERROR_CODE err = mZed->setRegionOfInterest(roi_mask);
    if (err != sl::ERROR_CODE::SUCCESS) {
      std::string err_msg =
        "Error while setting ZED SDK region of interest: ";
      err_msg += sl::toString(err).c_str();

      RCLCPP_WARN_STREAM(get_logger(), "  * " << err_msg);

      mManualRoiEnabled = false;

      if (mPubRoiMask.getTopic().empty()) {
        mPubRoiMask = image_transport::create_camera_publisher(
          this, mRoiMaskTopic, mQos.get_rmw_qos_profile());
        RCLCPP_INFO_STREAM(
          get_logger(),
          "Advertised on topic: " << mPubRoiMask.getTopic());
      }

      res->message = err_msg;
      res->success = false;
      return;
    } else {
      RCLCPP_INFO(get_logger(), "  * Region of Interest correctly set.");

      mManualRoiEnabled = true;

      res->message = "Region of Interest correctly set.";
      res->success = true;
      return;
    }
  }
  // <---- Set Region of Interest
}

void ZedCamera::callback_resetRoi(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
  std::shared_ptr<std_srvs::srv::Trigger_Response> res)
{
  RCLCPP_INFO(get_logger(), "** Reset ROI service called **");

  if (mDepthDisabled) {
    std::string err_msg =
      "Error while resetting ZED SDK region of interest: depth processing "
      "is "
      "disabled!";

    RCLCPP_WARN_STREAM(get_logger(), " * " << err_msg);

    res->message = err_msg;
    res->success = false;
    return;
  }

  sl::Mat empty_roi;
  sl::ERROR_CODE err = mZed->setRegionOfInterest(empty_roi);

  if (err != sl::ERROR_CODE::SUCCESS) {
    std::string err_msg =
      " * Error while resetting ZED SDK region of interest: ";
    err_msg += sl::toString(err);

    RCLCPP_WARN_STREAM(
      get_logger(),
      " * Error while resetting ZED SDK region of interest: " << err_msg);

    mManualRoiEnabled = false;

    res->message = err_msg;
    res->success = false;
  } else {
    RCLCPP_INFO(get_logger(), " * Region of Interest correctly reset.");

    mManualRoiEnabled = false;

    res->message = "Region of Interest correctly reset.";
    res->success = true;
    return;
  }
}

void ZedCamera::callback_toLL(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<robot_localization::srv::ToLL_Request> req,
  std::shared_ptr<robot_localization::srv::ToLL_Response> res)
{
  RCLCPP_INFO(get_logger(), "** Map to Lat/Long service called **");

  if (!mGnssFusionEnabled) {
    RCLCPP_WARN(get_logger(), " * GNSS fusion is not enabled");
    return;
  }

  if (mFusedPosTrackingStatus.gnss_fusion_status != sl::GNSS_FUSION_STATUS::OK) {
    RCLCPP_WARN(get_logger(), " * GNSS fusion is not yet ready");
    return;
  }

  sl::Translation map_pt;
  map_pt.x = req->map_point.x;
  map_pt.y = req->map_point.y;
  map_pt.z = req->map_point.z;

  sl::GeoPose geo_pose;
  sl::Pose map_pose;
  map_pose.pose_data.setIdentity();
  map_pose.pose_data.setTranslation(map_pt);
  mFusion.Camera2Geo(map_pose, geo_pose);

  res->ll_point.altitude = geo_pose.latlng_coordinates.getAltitude();
  res->ll_point.latitude = geo_pose.latlng_coordinates.getLatitude(false);
  res->ll_point.longitude = geo_pose.latlng_coordinates.getLongitude(false);

  RCLCPP_INFO(
    get_logger(),
    "* Converted the MAP point (%.2fm,%.2fm,%.2fm)to GeoPoint "
    "%.6f°,%.6f° / %.2f m",
    req->map_point.x, req->map_point.y, req->map_point.z,
    geo_pose.latlng_coordinates.getLatitude(false),
    geo_pose.latlng_coordinates.getLongitude(false),
    geo_pose.latlng_coordinates.getAltitude());
}

void ZedCamera::callback_fromLL(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<robot_localization::srv::FromLL_Request> req,
  std::shared_ptr<robot_localization::srv::FromLL_Response> res)
{
  RCLCPP_INFO(get_logger(), "** Lat/Long to Map service called **");

  if (!mGnssFusionEnabled) {
    RCLCPP_WARN(get_logger(), " * GNSS fusion is not enabled");
    return;
  }

  if (mFusedPosTrackingStatus.gnss_fusion_status !=
    sl::GNSS_FUSION_STATUS::OK)
  {
    RCLCPP_WARN(get_logger(), " * GNSS fusion is not ready");
    return;
  }

  sl::LatLng ll_pt;
  ll_pt.setCoordinates(
    req->ll_point.latitude, req->ll_point.longitude,
    req->ll_point.altitude, false);

  sl::Pose sl_pt_cam;
  mFusion.Geo2Camera(ll_pt, sl_pt_cam);

  // the point is already in the MAP Frame as it is converted from a GeoPoint
  res->map_point.x = sl_pt_cam.getTranslation().x;
  res->map_point.y = sl_pt_cam.getTranslation().y;
  res->map_point.z = sl_pt_cam.getTranslation().z;

  RCLCPP_INFO(
    get_logger(),
    "* Converted the GeoPoint %.6f°,%.6f° / %.2f m to MAP point "
    "(%.2fm,%.2fm,%.2fm)",
    ll_pt.getLatitude(false), ll_pt.getLongitude(false),
    ll_pt.getAltitude(), res->map_point.x, res->map_point.y,
    res->map_point.z);
}

void ZedCamera::callback_clock(
  const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  DEBUG_SIM("*** CLOCK CALLBACK ***");
  rclcpp::Time msg_time(msg->clock, RCL_ROS_TIME);

  try {
    if (msg_time != mLastClock) {
      mClockAvailable = true;
      DEBUG_SIM("Received an updated '/clock' message.");
    } else {
      mClockAvailable = false;
      DEBUG_SIM("Received a NOT updated '/clock' message.");
    }
    mLastClock = msg_time;
  } catch (...) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Error comparing clock messages: "
        << static_cast<int>(msg_time.get_clock_type())
        << " vs "
        << static_cast<int>(mLastClock.get_clock_type()));

    mClockAvailable = false;
  }
}

void ZedCamera::processRtRoi(rclcpp::Time ts)
{
  if (!mAutoRoiEnabled && !mManualRoiEnabled) {
    mAutoRoiStatus = sl::REGION_OF_INTEREST_AUTO_DETECTION_STATE::NOT_ENABLED;
    return;
  }

  if (mAutoRoiEnabled) {
    mAutoRoiStatus = mZed->getRegionOfInterestAutoDetectionStatus();
    DEBUG_STREAM_ROI("Automatic ROI Status:" << sl::toString(mAutoRoiStatus));
    if (mAutoRoiStatus ==
      sl::REGION_OF_INTEREST_AUTO_DETECTION_STATE::RUNNING)
    {
      if (mAutoRoiStatus ==
        sl::REGION_OF_INTEREST_AUTO_DETECTION_STATE::READY)
      {
        RCLCPP_INFO(
          get_logger(),
          "Region of interest auto detection is done!");
      }
    }
  }

  if (mAutoRoiStatus == sl::REGION_OF_INTEREST_AUTO_DETECTION_STATE::READY ||
    mManualRoiEnabled)
  {
    uint8_t subCount = 0;

    try {
      subCount = mPubRoiMask.getNumSubscribers();
    } catch (...) {
      rcutils_reset_error();
      DEBUG_STREAM_VD("processRtRoi: Exception while counting subscribers");
      return;
    }

    if (subCount > 0) {
      DEBUG_ROI("Retrieve ROI Mask");
      sl::Mat roi_mask;
      mZed->getRegionOfInterest(roi_mask);

      DEBUG_ROI("Publish ROI Mask");
      publishImageWithInfo(
        roi_mask, mPubRoiMask, mLeftCamInfoMsg,
        mLeftCamOptFrameId, ts);
    }
  }
}

bool ZedCamera::startStreamingServer()
{
  DEBUG_STR("Starting streaming server");

  if (mZed->isStreamingEnabled()) {
    mZed->disableStreaming();
    RCLCPP_WARN(get_logger(), "A streaming server was already running and has been stopped");
  }

  sl::StreamingParameters params;
  params.adaptative_bitrate = mStreamingServerAdaptiveBitrate;
  params.bitrate = mStreamingServerBitrate;
  params.chunk_size = mStreamingServerChunckSize;
  params.codec = mStreamingServerCodec;
  params.gop_size = mStreamingServerGopSize;
  params.port = mStreamingServerPort;
  params.target_framerate = mStreamingServerTargetFramerate;

  sl::ERROR_CODE res;
  res = mZed->enableStreaming(params);
  if (res != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Error starting the Streaming server: " << sl::toString(
        res) << " - " << sl::toVerbose(res));
    mStreamingServerRunning = false;
  } else {
    mStreamingServerRunning = true;
    RCLCPP_INFO(get_logger(), "Streaming server started");
  }
  return mStreamingServerRunning;
}

void ZedCamera::stopStreamingServer()
{
  if (mZed->isStreamingEnabled()) {
    mZed->disableStreaming();
    RCLCPP_INFO(get_logger(), "Streaming server stopped");
  } else {
    RCLCPP_WARN(get_logger(), "A streaming server was not enabled.");
  }

  mStreamingServerRunning = false;
  mStreamingServerRequired = false;
}
}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCamera)
