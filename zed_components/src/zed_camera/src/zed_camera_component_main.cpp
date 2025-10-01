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

#include "zed_camera_component.hpp"

#include <sys/resource.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <limits>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include <sstream>
#include <filesystem>

#include "sl_logging.hpp"

#ifdef FOUND_HUMBLE
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif defined FOUND_IRON
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif defined FOUND_JAZZY
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif defined FOUND_ROLLING
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif defined FOUND_FOXY
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#error Unsupported ROS 2 distro
#endif

#include <sl/Camera.hpp>

#include "sl_tools.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

// Used for simulation data input
#define ZED_SDK_PORT 30000

// Used to enable ZED SDK RealTime SVO pause
//#define USE_SVO_REALTIME_PAUSE

namespace stereolabs
{

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
  mStreamingServerRunning(false),
  mUptimer(get_clock()),
  mSetSvoFrameCheckTimer(get_clock())
{
  RCLCPP_INFO(get_logger(), "================================");
  RCLCPP_INFO(get_logger(), "      ZED Camera Component ");
  RCLCPP_INFO(get_logger(), "================================");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(
    get_logger(), " * IPC: %s",
    options.use_intra_process_comms() ? "enabled" : "disabled");
  RCLCPP_INFO(get_logger(), "================================");

  if (((ZED_SDK_MAJOR_VERSION * 10 + ZED_SDK_MINOR_VERSION) <
    (SDK_MAJOR_MIN_SUPP * 10 + SDK_MINOR_MIN_SUPP)) ||
    ((ZED_SDK_MAJOR_VERSION * 10 + ZED_SDK_MINOR_VERSION) >
    (SDK_MAJOR_MAX_SUPP * 10 + SDK_MINOR_MAX_SUPP)))
  {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "This version of the ZED ROS2 wrapper is designed to work with ZED SDK "
      "v" << static_cast<int>(SDK_MAJOR_MIN_SUPP)
          << "." << static_cast<int>(SDK_MINOR_MIN_SUPP) << " or newer up to v" <<
        static_cast<int>(SDK_MAJOR_MAX_SUPP) << "." << static_cast<int>(SDK_MINOR_MAX_SUPP) << ".");
    RCLCPP_INFO_STREAM(
      get_logger(), "* Detected SDK v"
        << ZED_SDK_MAJOR_VERSION << "."
        << ZED_SDK_MINOR_VERSION << "."
        << ZED_SDK_PATCH_VERSION << "-"
        << ZED_SDK_BUILD_ID);
    RCLCPP_INFO(get_logger(), "Node stopped. Press Ctrl+C to exit.");
    exit(EXIT_FAILURE);
  }

#if (ZED_SDK_MAJOR_VERSION * 10 + ZED_SDK_MINOR_VERSION) >= 50
  sl::setEnvironmentVariable("ZED_SDK_DISABLE_PROGRESS_BAR_LOG", "1");
#endif

  // ----> Publishers/Subscribers options
  #ifndef FOUND_FOXY
  mPubOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  mSubOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  #endif
  // <---- Publishers/Subscribers options

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
    mCameraName, this,
    &ZedCamera::callback_updateDiagnostic);
  std::string hw_id = std::string("Stereolabs ");
  hw_id += sl::toString(mCamUserModel).c_str();
  hw_id += " - '" + mCameraName + "'";
  mDiagUpdater.setHardwareID(hw_id);
  //mDiagUpdater.force_update();
  // <---- Diagnostic initialization

  // Services initialization
  initServices();

  // ----> Start camera
  if (!startCamera()) {
    exit(EXIT_FAILURE);
  }
  // <---- Start camera

  // Callback when the node is destroyed
  // This is used to stop the camera when the node is destroyed
  // and to stop the timers

  // Close camera callback before shutdown
  using rclcpp::contexts::get_global_default_context;
  get_global_default_context()->add_pre_shutdown_callback(
    [this]() {
      DEBUG_COMM("ZED Component is shutting down");
      close();
      DEBUG_COMM("ZED Component is shutting down - done");
    });

  // Dynamic parameters callback
  mParamChangeCallbackHandle = add_on_set_parameters_callback(
    std::bind(&ZedCamera::callback_dynamicParamChange, this, _1));
}

void ZedCamera::close()
{
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

  DEBUG_STREAM_VD("Waiting for video/depth thread...");
  try {
    if (mVdThread.joinable()) {
      mVdThread.join();
    }
  } catch (std::system_error & e) {
    DEBUG_STREAM_VD("Video/Depth thread joining exception: " << e.what());
  }
  DEBUG_STREAM_VD("... video/depth thread stopped");

  DEBUG_STREAM_PC("Waiting for Point Cloud thread...");
  try {
    if (mPcThread.joinable()) {
      mPcThread.join();
    }
  } catch (std::system_error & e) {
    DEBUG_STREAM_PC("Pointcloud thread joining exception: " << e.what());
  }
  DEBUG_STREAM_PC("... Point Cloud thread stopped");

  closeCamera();
}

ZedCamera::~ZedCamera()
{
  close();
}

void ZedCamera::initServices()
{
  RCLCPP_INFO(get_logger(), "=== SERVICES ===");

  std::string srv_name;

  std::string srv_prefix = "~/";

  if (!mDepthDisabled) {
    // Reset Odometry
    srv_name = srv_prefix + mSrvResetOdomName;
    mResetOdomSrv = create_service<std_srvs::srv::Trigger>(
      srv_name,
      std::bind(&ZedCamera::callback_resetOdometry, this, _1, _2, _3));
    RCLCPP_INFO_STREAM(
      get_logger(), " * Advertised on service: '" << mResetOdomSrv->get_service_name() << "'");
    // Reset Pose
    srv_name = srv_prefix + mSrvResetPoseName;
    mResetPosTrkSrv = create_service<std_srvs::srv::Trigger>(
      srv_name,
      std::bind(&ZedCamera::callback_resetPosTracking, this, _1, _2, _3));
    RCLCPP_INFO_STREAM(
      get_logger(), " * Advertised on service: '" << mResetPosTrkSrv->get_service_name() << "'");
    // Set Pose
    srv_name = srv_prefix + mSrvSetPoseName;
    mSetPoseSrv = create_service<zed_msgs::srv::SetPose>(
      srv_name, std::bind(&ZedCamera::callback_setPose, this, _1, _2, _3));
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Advertised on service: '" << mSetPoseSrv->get_service_name() << "'");
    // Save Area Memory
    srv_name = srv_prefix + mSrvSaveAreaMemoryName;
    /*mSaveAreaMemorySrv = create_service<zed_msgs::srv::SaveAreaMemory>(
      srv_name, std::bind(&ZedCamera::callback_saveAreaMemory, this, _1, _2, _3));*/// TODO(Walter): Uncomment when available in `zed_msgs` package from APT
    mSaveAreaMemorySrv = create_service<zed_msgs::srv::SetROI>(
      srv_name, std::bind(&ZedCamera::callback_saveAreaMemory, this, _1, _2, _3));
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Advertised on service: '" << mSaveAreaMemorySrv->get_service_name() << "'");

    // Enable Object Detection
    srv_name = srv_prefix + mSrvEnableObjDetName;
    mEnableObjDetSrv = create_service<std_srvs::srv::SetBool>(
      srv_name,
      std::bind(&ZedCamera::callback_enableObjDet, this, _1, _2, _3));
    RCLCPP_INFO_STREAM(
      get_logger(), " * Advertised on service: '" << mEnableObjDetSrv->get_service_name() << "'");

    // Enable BodyTracking
    srv_name = srv_prefix + mSrvEnableBodyTrkName;
    mEnableBodyTrkSrv = create_service<std_srvs::srv::SetBool>(
      srv_name,
      std::bind(&ZedCamera::callback_enableBodyTrk, this, _1, _2, _3));
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Advertised on service: '" << mEnableBodyTrkSrv->get_service_name() << "'");

    // Enable Mapping
    srv_name = srv_prefix + mSrvEnableMappingName;
    mEnableMappingSrv = create_service<std_srvs::srv::SetBool>(
      srv_name,
      std::bind(&ZedCamera::callback_enableMapping, this, _1, _2, _3));
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Advertised on service: '" << mEnableMappingSrv->get_service_name() << "'");


  }

  // Enable Streaming
  srv_name = srv_prefix + mSrvEnableStreamingName;
  mEnableStreamingSrv = create_service<std_srvs::srv::SetBool>(
    srv_name, std::bind(&ZedCamera::callback_enableStreaming, this, _1, _2, _3));
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Advertised on service: '" << mEnableStreamingSrv->get_service_name() << "'");

  // Start SVO Recording
  srv_name = srv_prefix + mSrvStartSvoRecName;
  mStartSvoRecSrv = create_service<zed_msgs::srv::StartSvoRec>(
    srv_name, std::bind(&ZedCamera::callback_startSvoRec, this, _1, _2, _3));
  RCLCPP_INFO_STREAM(
    get_logger(), " * Advertised on service: '" << mStartSvoRecSrv->get_service_name() << "'");
  // Stop SVO Recording
  srv_name = srv_prefix + mSrvStopSvoRecName;
  mStopSvoRecSrv = create_service<std_srvs::srv::Trigger>(
    srv_name, std::bind(&ZedCamera::callback_stopSvoRec, this, _1, _2, _3));
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Advertised on service: '" << mStopSvoRecSrv->get_service_name() << "'");

  // Pause SVO (only if the realtime playing mode is disabled)
  if (mSvoMode) {
#ifndef USE_SVO_REALTIME_PAUSE
    if (!mSvoRealtime) {
#endif
    srv_name = srv_prefix + mSrvToggleSvoPauseName;
    mPauseSvoSrv = create_service<std_srvs::srv::Trigger>(
      srv_name,
      std::bind(&ZedCamera::callback_pauseSvoInput, this, _1, _2, _3));
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Advertised on service: '" << mPauseSvoSrv->get_service_name() << "'");
#ifndef USE_SVO_REALTIME_PAUSE
  }
#endif

    //Set Service for SVO frame
    srv_name = srv_prefix + mSrvSetSvoFrameName;
    mSetSvoFrameSrv = create_service<zed_msgs::srv::SetSvoFrame>(
      srv_name,
      std::bind(&ZedCamera::callback_setSvoFrame, this, _1, _2, _3));
    RCLCPP_INFO_STREAM(
      get_logger(), " * Advertised on service: '" << mSetSvoFrameSrv->get_service_name() << "'");
  }

  // Set ROI
  srv_name = srv_prefix + mSrvSetRoiName;
  mSetRoiSrv = create_service<zed_msgs::srv::SetROI>(
    srv_name, std::bind(&ZedCamera::callback_setRoi, this, _1, _2, _3));
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Advertised on service: '" << mSetRoiSrv->get_service_name() << "'");
  // Reset ROI
  srv_name = srv_prefix + mSrvResetRoiName;
  mResetRoiSrv = create_service<std_srvs::srv::Trigger>(
    srv_name, std::bind(&ZedCamera::callback_resetRoi, this, _1, _2, _3));
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Advertised on service: '" << mResetRoiSrv->get_service_name() << "'");

  if (mGnssFusionEnabled) {
    // To Latitude/Longitude
    srv_name = srv_prefix + mSrvToLlName;
    mToLlSrv = create_service<robot_localization::srv::ToLL>(
      srv_name, std::bind(&ZedCamera::callback_toLL, this, _1, _2, _3));
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Advertised on service: '" << mToLlSrv->get_service_name() << "'");
    // From Latitude/Longitude
    srv_name = srv_prefix + mSrvFromLlName;
    mFromLlSrv = create_service<robot_localization::srv::FromLL>(
      srv_name, std::bind(&ZedCamera::callback_fromLL, this, _1, _2, _3));
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Advertised on service: '" << mFromLlSrv->get_service_name() << "'");
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


void ZedCamera::initParameters()
{
  // DEBUG parameters
  getDebugParams();

  // SIMULATION parameters
  getSimParams();

  // GENERAL parameters
  getGeneralParams();

  // VIDEO parameters
  if (!mSvoMode && !mSimMode) {
    getVideoParams();
  }

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

  RCLCPP_INFO(get_logger(), "=== DEBUG parameters ===");

  sl_tools::getParam(
    shared_from_this(), "debug.sdk_verbose", mVerbose,
    mVerbose, " * SDK Verbose: ", false, 0, 9999);
  sl_tools::getParam(
    shared_from_this(), "debug.use_pub_timestamps",
    mUsePubTimestamps, mUsePubTimestamps,
    " * Use Pub Timestamps: ");
  sl_tools::getParam(
    shared_from_this(), "debug.sdk_verbose_log_file",
    mVerboseLogFile, mVerboseLogFile, " * SDK Verbose File: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_common", _debugCommon,
    _debugCommon, " * Debug Common: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_sim", _debugSim,
    _debugSim, " * Debug Simulation: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_video_depth",
    _debugVideoDepth, _debugVideoDepth,
    " * Debug Video/Depth: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_camera_controls",
    _debugCamCtrl, _debugCamCtrl,
    " * Debug Control settings: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_point_cloud",
    _debugPointCloud, _debugPointCloud,
    " * Debug Point Cloud: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_gnss", _debugGnss,
    _debugGnss, " * Debug GNSS: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_positional_tracking",
    _debugPosTracking, _debugPosTracking,
    " * Debug Positional Tracking: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_sensors", _debugSensors,
    _debugSensors, " * Debug sensors: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_mapping", _debugMapping,
    _debugMapping, " * Debug Mapping: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_object_detection",
    _debugObjectDet, _debugObjectDet,
    " * Debug Object Detection: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_body_tracking",
    _debugBodyTrk, _debugBodyTrk, " * Debug Body Tracking: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_streaming",
    _debugStreaming, _debugStreaming, " * Debug Streaming: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_roi", _debugRoi,
    _debugRoi, " * Debug ROI: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_nitros", _debugNitros,
    _debugNitros, " * Debug Nitros: ");
  sl_tools::getParam(
    shared_from_this(), "debug.debug_advanced", _debugAdvanced,
    _debugAdvanced, " * Debug Advanced: ");

  mDebugMode = _debugCommon || _debugSim || _debugVideoDepth || _debugCamCtrl ||
    _debugPointCloud || _debugPosTracking || _debugGnss ||
    _debugSensors || _debugMapping || _debugObjectDet ||
    _debugBodyTrk || _debugAdvanced || _debugRoi || _debugStreaming || _debugNitros;

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
    "[ROS 2] Using RMW_IMPLEMENTATION "
      << rmw_get_implementation_identifier());

#ifdef FOUND_ISAAC_ROS_NITROS
  sl_tools::getParam(
    shared_from_this(), "debug.disable_nitros",
    _nitrosDisabled, _nitrosDisabled);

  if (_nitrosDisabled) {
    RCLCPP_WARN(
      get_logger(),
      "NITROS is available, but is disabled by 'debug.disable_nitros'");
  }
#else
  _nitrosDisabled = true;  // Force disable NITROS if not available
#endif
}

void ZedCamera::getSimParams()
{
  // SIMULATION active?
  sl_tools::getParam(
    shared_from_this(), "simulation.sim_enabled", mSimMode,
    mSimMode);

  if (!get_parameter("use_sim_time", mUseSimTime)) {
    RCLCPP_WARN(
      get_logger(),
      "The parameter 'use_sim_time' is not available or is not "
      "valid, using the default value.");
  }

  if (mSimMode) {
    RCLCPP_INFO(get_logger(), " === SIMULATION MODE ACTIVE ===");
    sl_tools::getParam(
      shared_from_this(), "simulation.sim_address", mSimAddr,
      mSimAddr, " * Sim. server address: ");
    sl_tools::getParam(
      shared_from_this(), "simulation.sim_port", mSimPort,
      mSimPort, " * Sim. server port: ");

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

  RCLCPP_INFO(get_logger(), "=== SVO INPUT parameters ===");

  sl_tools::getParam(
    shared_from_this(), "svo.svo_path", std::string(),
    mSvoFilepath);
  if (mSvoFilepath.compare("live") == 0) {
    mSvoFilepath = "";
  }

  if (mSvoFilepath == "") {
    mSvoMode = false;
  } else {
    RCLCPP_INFO_STREAM(get_logger(), " * SVO: '" << mSvoFilepath.c_str() << "'");
    mSvoMode = true;
    sl_tools::getParam(
      shared_from_this(), "svo.use_svo_timestamps",
      mUseSvoTimestamp, mUseSvoTimestamp,
      " * Use SVO timestamp: ");
    if (mUseSvoTimestamp) {
      sl_tools::getParam(
        shared_from_this(), "svo.publish_svo_clock",
        mPublishSvoClock, mPublishSvoClock,
        " * Publish SVO timestamp: ");
    }

    sl_tools::getParam(shared_from_this(), "svo.svo_loop", mSvoLoop, mSvoLoop);
    if (mUseSvoTimestamp) {
      if (mSvoLoop) {
        RCLCPP_WARN(
          get_logger(),
          "SVO Loop is not supported when using SVO timestamps. Loop playback disabled.");
        mSvoLoop = false;
      }
      RCLCPP_INFO_STREAM(
        get_logger(),
        " * SVO Loop: " << (mSvoLoop ? "TRUE" : "FALSE"));
    }
    sl_tools::getParam(
      shared_from_this(), "svo.svo_realtime", mSvoRealtime,
      mSvoRealtime, " * SVO Realtime: ");

    sl_tools::getParam(
      shared_from_this(), "svo.play_from_frame",
      mSvoFrameStart, mSvoFrameStart,
      " * SVO start frame: ", false, 0);

    if (!mSvoRealtime) {
      sl_tools::getParam(
        shared_from_this(), "svo.replay_rate", mSvoRate,
        mSvoRate, " * SVO replay rate: ", true, 0.1, 5.0);
    }
  }

  mStreamMode = false;
  if (!mSvoMode) {
    RCLCPP_INFO(get_logger(), "=== LOCAL STREAMING parameters ===");
    sl_tools::getParam(
      shared_from_this(), "stream.stream_address",
      std::string(), mStreamAddr);
    if (mStreamAddr != "") {
      mStreamMode = true;
      sl_tools::getParam(
        shared_from_this(), "stream.stream_port", mStreamPort,
        mStreamPort);
      RCLCPP_INFO_STREAM(
        get_logger(), " * Local stream input: " << mStreamAddr << ":" << mStreamPort);
    }
  }

  RCLCPP_INFO(get_logger(), "=== GENERAL parameters ===");

  std::string camera_model = "zed";
  sl_tools::getParam(
    shared_from_this(), "general.camera_model", camera_model,
    camera_model);
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

  sl_tools::getParam(
    shared_from_this(), "general.camera_name", mCameraName,
    mCameraName, " * Camera name: ");

  if (!mSvoMode) {
    sl_tools::getParam(
      shared_from_this(), "general.serial_number",
      mCamSerialNumber, mCamSerialNumber, " * Camera SN: ", false, 0);
    sl_tools::getParam(
      shared_from_this(), "general.camera_id", mCamId, mCamId,
      " * Camera ID: ", false, -1, 256);
    sl_tools::getParam(
      shared_from_this(), "general.camera_timeout_sec",
      mCamTimeoutSec, mCamTimeoutSec,
      " * Camera timeout [sec]: ", false, 0, 9999);
    sl_tools::getParam(
      shared_from_this(), "general.camera_max_reconnect",
      mMaxReconnectTemp, mMaxReconnectTemp,
      " * Camera reconnection temptatives: ", false, 0, 9999);
    if (mSimMode) {
      RCLCPP_INFO(
        get_logger(),
        "* [Simulation mode] Camera framerate forced to 60 Hz");
      mCamGrabFrameRate = 60;
    } else {
      sl_tools::getParam(
        shared_from_this(), "general.grab_frame_rate",
        mCamGrabFrameRate, mCamGrabFrameRate,
        " * Camera framerate: ", false, 0, 120);
    }
  }
  sl_tools::getParam(
    shared_from_this(), "general.gpu_id", mGpuId, mGpuId,
    " * GPU ID: ", false, -1, 999);
  sl_tools::getParam(
    shared_from_this(), "general.async_image_retrieval",
    mAsyncImageRetrieval, mAsyncImageRetrieval,
    " * Asynchronous image retrieval: ");

  sl_tools::getParam(
    shared_from_this(), "general.enable_image_validity_check",
    mImageValidityCheck, mImageValidityCheck,
    " * Image Validity Check: ", false, 0, 10);

  // TODO(walter) ADD SVO SAVE COMPRESSION PARAMETERS

  if (mSimMode) {
    RCLCPP_INFO(
      get_logger(),
      "* [Simulation mode] Camera resolution forced to 'HD1080'");
    mCamResol = sl::RESOLUTION::HD1080;
  } else {
    std::string resol = "AUTO";
    sl_tools::getParam(
      shared_from_this(), "general.grab_resolution", resol,
      resol);
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

  std::string out_resol = "NATIVE";
  sl_tools::getParam(
    shared_from_this(), "general.pub_resolution", out_resol,
    out_resol);
  if (out_resol == toString(PubRes::NATIVE)) {
    mPubResolution = PubRes::NATIVE;
  } else if (out_resol == toString(PubRes::CUSTOM)) {
    mPubResolution = PubRes::CUSTOM;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Not valid 'general.pub_resolution' value: '%s'. Using default "
      "setting instead.",
      out_resol.c_str());
    out_resol = "NATIVE -> Fix the value in YAML!";
    mPubResolution = PubRes::NATIVE;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Publishing resolution: " << out_resol.c_str());

  if (mPubResolution == PubRes::CUSTOM) {
    sl_tools::getParam(
      shared_from_this(), "general.pub_downscale_factor",
      mCustomDownscaleFactor, mCustomDownscaleFactor,
      " * Publishing downscale factor: ", false, 1.0, 5.0);
  } else {
    mCustomDownscaleFactor = 1.0;
  }

  sl_tools::getParam(
    shared_from_this(), "general.optional_opencv_calibration_file",
    mOpencvCalibFile, mOpencvCalibFile, " * OpenCV custom calibration: ");

  sl_tools::getParam(
    shared_from_this(), "general.self_calib", mCameraSelfCalib,
    mCameraSelfCalib, " * Camera self calibration: ");

  sl_tools::getParam(
    shared_from_this(), "general.camera_flip", mCameraFlip,
    mCameraFlip, " * Camera flip: ");

  // Dynamic parameters

  if (mSimMode) {
    RCLCPP_INFO(
      get_logger(),
      "* [Simulation mode] Publish framerate forced to 60 Hz");
    mVdPubRate = 60;
  }

  if (mSvoMode && !mSvoRealtime) {
    RCLCPP_INFO(
      get_logger(),
      " * [SVO mode - not realtime] Publish framerate forced to SVO Playback rate");
    mVdPubRate = 0;
  } else {
    sl_tools::getParam(
      shared_from_this(), "general.pub_frame_rate", mVdPubRate,
      mVdPubRate, "", true, 0.1, static_cast<double>(mCamGrabFrameRate));
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Publish framerate [Hz]:  " << mVdPubRate);
  }
}

void ZedCamera::getRoiParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "=== Region of Interest parameters ===");

  sl_tools::getParam(
    shared_from_this(), "region_of_interest.automatic_roi",
    mAutoRoiEnabled, mAutoRoiEnabled,
    " * Automatic ROI generation: ");

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

    sl_tools::getParam(
      shared_from_this(), "region_of_interest.depth_far_threshold_meters",
      mRoiDepthFarThresh, mRoiDepthFarThresh, " * Depth far threshold [m]: ");
    sl_tools::getParam(
      shared_from_this(),
      "region_of_interest.image_height_ratio_cutoff",
      mRoiImgHeightRationCutOff, mRoiImgHeightRationCutOff,
      " * Image Height Ratio Cut Off: ");
  } else {
    std::string parsed_str =
      this->getParam("region_of_interest.manual_polygon", mRoyPolyParam);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Manual ROI polygon: " << parsed_str.c_str());
  }

  if (mRoyPolyParam.size() > 0 || mAutoRoiEnabled) {
    mRoiModules.clear();
    bool apply = true;

    sl_tools::getParam(
      shared_from_this(), "region_of_interest.apply_to_depth",
      apply, apply, " * Apply to depth: ");
    if (apply) {
      mRoiModules.insert(sl::MODULE::DEPTH);
    }

    apply = true;
    sl_tools::getParam(
      shared_from_this(),
      "region_of_interest.apply_to_positional_tracking", apply,
      apply, " * Apply to positional tracking: ");
    if (apply) {
      mRoiModules.insert(sl::MODULE::POSITIONAL_TRACKING);
    }

    apply = true;
    sl_tools::getParam(
      shared_from_this(),
      "region_of_interest.apply_to_object_detection", apply,
      apply, " * Apply to object detection: ");
    if (apply) {
      mRoiModules.insert(sl::MODULE::OBJECT_DETECTION);
    }

    apply = true;
    sl_tools::getParam(
      shared_from_this(),
      "region_of_interest.apply_to_body_tracking", apply,
      apply, " * Apply to body tracking: ");
    if (apply) {
      mRoiModules.insert(sl::MODULE::BODY_TRACKING);
    }

    apply = true;
    sl_tools::getParam(
      shared_from_this(),
      "region_of_interest.apply_to_spatial_mapping", apply,
      apply, " * Apply to spatial mapping: ");
    if (apply) {
      mRoiModules.insert(sl::MODULE::SPATIAL_MAPPING);
    }
  }
}

void ZedCamera::getSensorsParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "=== SENSORS STACK parameters ===");
  if (sl_tools::isZED(mCamUserModel)) {
    RCLCPP_WARN(
      get_logger(),
      "!!! SENSORS parameters are not used with ZED !!!");
    return;
  }

  sl_tools::getParam(
    shared_from_this(), "sensors.publish_imu_tf",
    mPublishImuTF, mPublishImuTF,
    " * Broadcast IMU TF [not for ZED]: ");

  sl_tools::getParam(
    shared_from_this(), "sensors.sensors_image_sync",
    mSensCameraSync, mSensCameraSync,
    " * Sensors Camera Sync: ");

  sl_tools::getParam(
    shared_from_this(), "sensors.sensors_pub_rate",
    mSensPubRate, mSensPubRate, " * Sensors publishing rate [Hz]: ", true, 1.0, 400.0);
}

void ZedCamera::getMappingParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "=== Spatial Mapping parameters ===");

  sl_tools::getParam(
    shared_from_this(), "mapping.mapping_enabled",
    mMappingEnabled, mMappingEnabled,
    " * Spatial Mapping Enabled: ");

  sl_tools::getParam(
    shared_from_this(), "mapping.resolution", mMappingRes,
    mMappingRes, " * Spatial Mapping resolution [m]: ");
  sl_tools::getParam(
    shared_from_this(), "mapping.max_mapping_range",
    mMappingRangeMax, mMappingRangeMax);
  if (mMappingRangeMax == -1.0f) {
    RCLCPP_INFO(get_logger(), " * 3D Max Mapping range: AUTO");
  } else {
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * 3D Max Mapping range [m]: " << mMappingRangeMax);
  }
  sl_tools::getParam(
    shared_from_this(), "mapping.fused_pointcloud_freq",
    mFusedPcPubRate, mFusedPcPubRate,
    " * Map publishing rate [Hz]: ", true, 0.1, 30.0);

  sl_tools::getParam(
    shared_from_this(), "mapping.clicked_point_topic",
    mClickedPtTopic, mClickedPtTopic,
    " * Clicked point topic: ");

  sl_tools::getParam(
    shared_from_this(), "mapping.pd_max_distance_threshold",
    mPdMaxDistanceThreshold, mPdMaxDistanceThreshold,
    " * Plane Det. Max Dist. Thresh.: ", false, 0.0, 100.0);
  sl_tools::getParam(
    shared_from_this(),
    "mapping.pd_normal_similarity_threshold",
    mPdNormalSimilarityThreshold, mPdNormalSimilarityThreshold,
    " * Plane Det. Normals Sim. Thresh.: ", false, -180.0, 180.0);
}

void ZedCamera::getPosTrackingParams()
{
  rclcpp::Parameter paramVal;
  std::string paramName;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "=== POSITIONAL TRACKING parameters ===");

  sl_tools::getParam(
    shared_from_this(), "pos_tracking.pos_tracking_enabled",
    mPosTrackingEnabled, mPosTrackingEnabled,
    " * Positional tracking enabled: ");

  std::string pos_trk_mode_str = "GEN_1";
  sl_tools::getParam(
    shared_from_this(), "pos_tracking.pos_tracking_mode",
    pos_trk_mode_str, pos_trk_mode_str);
  bool matched = false;
  for (int idx = static_cast<int>(sl::POSITIONAL_TRACKING_MODE::GEN_1);
    idx < static_cast<int>(sl::POSITIONAL_TRACKING_MODE::LAST); idx++)
  {
    sl::POSITIONAL_TRACKING_MODE test_mode =
      static_cast<sl::POSITIONAL_TRACKING_MODE>(idx);
    std::string test_mode_str = sl::toString(test_mode).c_str();
    std::replace(
      test_mode_str.begin(), test_mode_str.end(), ' ', '_'); // Replace spaces with underscores to match the YAML setting
    DEBUG_PT(" Comparing '%s' to '%s'", test_mode_str.c_str(), pos_trk_mode_str.c_str());
    if (pos_trk_mode_str == test_mode_str) {
      mPosTrkMode = test_mode;
      matched = true;
      break;
    }
  }
  if (!matched) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The value of the parameter 'pos_tracking.pos_tracking_mode' is not valid: '"
        << pos_trk_mode_str << "'. Using the default value.");
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Positional tracking mode: "
      << sl::toString(mPosTrkMode).c_str());

  mBaseFrameId = mCameraName;
  mBaseFrameId += "_camera_link";

  sl_tools::getParam(
    shared_from_this(), "pos_tracking.map_frame", mMapFrameId,
    mMapFrameId, " * Map frame id: ");
  sl_tools::getParam(
    shared_from_this(), "pos_tracking.odometry_frame",
    mOdomFrameId, mOdomFrameId, " * Odometry frame id: ");

  sl_tools::getParam(
    shared_from_this(), "pos_tracking.publish_tf", mPublishTF,
    mPublishTF, " * Broadcast Odometry TF: ");
  if (mPublishTF) {
    sl_tools::getParam(
      shared_from_this(), "pos_tracking.publish_map_tf",
      mPublishMapTF, mPublishMapTF, " * Broadcast Pose TF: ");
  } else {
    mPublishMapTF = false;
  }

  sl_tools::getParam(
    shared_from_this(), "pos_tracking.depth_min_range",
    mPosTrackDepthMinRange, mPosTrackDepthMinRange,
    " * Depth minimum range: ", false, 0.0f, 40.0f);
  sl_tools::getParam(
    shared_from_this(), "pos_tracking.transform_time_offset",
    mTfOffset, mTfOffset, " * TF timestamp offset: ", true,
    -10.0, 10.0);
  sl_tools::getParam(
    shared_from_this(), "pos_tracking.path_pub_rate",
    mPathPubRate, mPathPubRate,
    " * Path publishing rate: ", true, 0.1, 120.0);
  sl_tools::getParam(
    shared_from_this(), "pos_tracking.path_max_count",
    mPathMaxCount, mPathMaxCount, " * Path history lenght: ", false, -1, 10000);

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
    sl_tools::getParam(
      shared_from_this(), "pos_tracking.area_memory",
      mAreaMemory, mAreaMemory, " * Area Memory: ");
    sl_tools::getParam(
      shared_from_this(), "pos_tracking.area_file_path",
      mAreaMemoryFilePath, mAreaMemoryFilePath,
      " * Area Memory File: ");
    sl_tools::getParam(
      shared_from_this(), "pos_tracking.save_area_memory_on_closing",
      mSaveAreaMemoryOnClosing, mSaveAreaMemoryOnClosing,
      " * Save Area Memory on closing: ");

    if (mAreaMemoryFilePath.empty()) {
      if (mSaveAreaMemoryOnClosing) {
        RCLCPP_WARN(
          get_logger(),
          "  * The parameter 'pos_tracking.area_file_path' is empty, "
          "no Area Memory File will be saved on closing.");
        mSaveAreaMemoryOnClosing = false;
      }
    } else {
      mAreaMemoryFilePath = sl_tools::getFullFilePath(mAreaMemoryFilePath);
      mAreaFileExists = std::filesystem::exists(mAreaMemoryFilePath);

      if (mAreaFileExists) {
        RCLCPP_INFO_STREAM(
          get_logger(),
          "  * Using the existing Area Memory file '" << mAreaMemoryFilePath << "'");
        if (mSaveAreaMemoryOnClosing) {
          RCLCPP_INFO(
            get_logger(),
            "  * The Area Memory file will be updated on node closing or by manually calling the `save_area_memory` service with empty parameter.");
        }
      } else {
        RCLCPP_INFO_STREAM(
          get_logger(),
          "  * The Area Memory file '" << mAreaMemoryFilePath << "' does not exist.");
        if (mSaveAreaMemoryOnClosing) {
          RCLCPP_INFO(
            get_logger(),
            "  * The Area Memory file will be created on node closing or by manually calling the `save_area_memory` service with empty parameter.");
        }
      }
    }

  }
  sl_tools::getParam(
    shared_from_this(), "pos_tracking.set_as_static",
    mSetAsStatic, mSetAsStatic, " * Camera is static: ");

  sl_tools::getParam(
    shared_from_this(), "pos_tracking.set_gravity_as_origin",
    mSetGravityAsOrigin, mSetGravityAsOrigin,
    " * Gravity as origin: ");
  sl_tools::getParam(
    shared_from_this(), "pos_tracking.imu_fusion", mImuFusion,
    mImuFusion, " * IMU Fusion: ");

  sl_tools::getParam(
    shared_from_this(), "pos_tracking.floor_alignment",
    mFloorAlignment, mFloorAlignment, " * Floor Alignment: ");

  sl_tools::getParam(
    shared_from_this(),
    "pos_tracking.reset_odom_with_loop_closure",
    mResetOdomWhenLoopClosure, mResetOdomWhenLoopClosure,
    " * Reset Odometry with Loop Closure: ");

  sl_tools::getParam(
    shared_from_this(), "pos_tracking.two_d_mode", mTwoDMode,
    mTwoDMode, " * 2D mode: ");

  if (mTwoDMode) {
    sl_tools::getParam(
      shared_from_this(), "pos_tracking.fixed_z_value",
      mFixedZValue, mFixedZValue, " * Fixed Z value: ");
  }
  sl_tools::getParam(
    shared_from_this(),
    "pos_tracking.reset_pose_with_svo_loop",
    mResetPoseWithSvoLoop, mResetPoseWithSvoLoop,
    " * Reset pose with SVO loop: ");
}

void ZedCamera::getGnssFusionParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "=== GNSS FUSION parameters ===");
  if (sl_tools::isZED(mCamUserModel)) {
    RCLCPP_WARN(
      get_logger(),
      "!!! GNSS FUSION module cannot be enabled with ZED!!!");
    return;
  }

  sl_tools::getParam(
    shared_from_this(), "gnss_fusion.gnss_fusion_enabled",
    mGnssFusionEnabled, mGnssFusionEnabled,
    " * GNSS fusion enabled: ");

  if (mGnssFusionEnabled) {
    mGnssFrameId = mCameraName + "_gnss_link";

    sl_tools::getParam(
      shared_from_this(), "gnss_fusion.gnss_fix_topic",
      mGnssTopic, mGnssTopic, " * GNSS topic name: ");
    sl_tools::getParam(
      shared_from_this(),
      "gnss_fusion.enable_reinitialization",
      mGnssEnableReinitialization, mGnssEnableReinitialization,
      " * GNSS Reinitialization: ");
    sl_tools::getParam(
      shared_from_this(), "gnss_fusion.enable_rolling_calibration",
      mGnssEnableRollingCalibration, mGnssEnableRollingCalibration,
      " * GNSS Rolling Calibration: ");
    sl_tools::getParam(
      shared_from_this(),
      "gnss_fusion.enable_translation_uncertainty_target",
      mGnssEnableTranslationUncertaintyTarget,
      mGnssEnableTranslationUncertaintyTarget,
      " * GNSS Transl. Uncert. Target: ");
    sl_tools::getParam(
      shared_from_this(),
      "gnss_fusion.gnss_vio_reinit_threshold",
      mGnssVioReinitThreshold, mGnssVioReinitThreshold,
      " * GNSS VIO Reinit. Thresh.: ");
    sl_tools::getParam(
      shared_from_this(), "gnss_fusion.target_translation_uncertainty",
      mGnssTargetTranslationUncertainty, mGnssTargetTranslationUncertainty,
      " * GNSS Target Transl. Uncert.: ");
    sl_tools::getParam(
      shared_from_this(), "gnss_fusion.target_yaw_uncertainty",
      mGnssTargetYawUncertainty, mGnssTargetYawUncertainty,
      " * GNSS Target Yaw Uncert.: ");
    sl_tools::getParam(
      shared_from_this(), "gnss_fusion.gnss_zero_altitude",
      mGnssZeroAltitude, mGnssZeroAltitude,
      " * GNSS Zero Altitude: ");

    sl_tools::getParam(
      shared_from_this(), "gnss_fusion.h_covariance_mul",
      mGnssHcovMul, mGnssHcovMul,
      " * Horiz. Covariance mult.: ");
    sl_tools::getParam(
      shared_from_this(), "gnss_fusion.v_covariance_mul",
      mGnssVcovMul, mGnssVcovMul,
      " * Vert. Covariance mult.: ");

    sl_tools::getParam(
      shared_from_this(), "gnss_fusion.publish_utm_tf",
      mPublishUtmTf, mPublishUtmTf, " * Publish UTM TF: ");

    sl_tools::getParam(
      shared_from_this(),
      "gnss_fusion.broadcast_utm_transform_as_parent_frame",
      mUtmAsParent, mUtmAsParent,
      " * Publish UTM TF as parent of 'map': ");
  }
}

void ZedCamera::getStreamingServerParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "=== STREAMING SERVER parameters ===");

  bool stream_server = false;
  sl_tools::getParam(
    shared_from_this(), "stream_server.stream_enabled",
    stream_server, stream_server,
    " * Streaming Server enabled: ");
  mStreamingServerRequired = stream_server;

  std::string codec = "H264";
  sl_tools::getParam(shared_from_this(), "stream_server.codec", codec, codec);
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

  sl_tools::getParam(
    shared_from_this(), "stream_server.port",
    mStreamingServerPort, mStreamingServerPort,
    " * Stream port: ", false, 0, 65535);

  sl_tools::getParam(
    shared_from_this(), "stream_server.bitrate",
    mStreamingServerBitrate, mStreamingServerBitrate, " * Stream bitrate: ", false, 1000, 60000);

  sl_tools::getParam(
    shared_from_this(), "stream_server.gop_size",
    mStreamingServerGopSize, mStreamingServerGopSize, " * Stream GOP size: ", false, -1, 256);

  sl_tools::getParam(
    shared_from_this(), "stream_server.chunk_size",
    mStreamingServerChunckSize, mStreamingServerChunckSize, " * Stream Chunk size: ", false, 1024,
    65000);

  sl_tools::getParam(
    shared_from_this(), "stream_server.adaptative_bitrate",
    mStreamingServerAdaptiveBitrate,
    mStreamingServerAdaptiveBitrate, " * Adaptive bitrate: ");

  sl_tools::getParam(
    shared_from_this(), "stream_server.target_framerate",
    mStreamingServerTargetFramerate,
    mStreamingServerTargetFramerate, " * Target frame rate:", false, 0, 120);
}

void ZedCamera::getAdvancedParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "=== ADVANCED parameters ===");

  sl_tools::getParam(
    shared_from_this(), "advanced.thread_sched_policy",
    mThreadSchedPolicy, mThreadSchedPolicy,
    " * Thread sched. policy: ");

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
      sl_tools::getParam(
        shared_from_this(), "advanced.thread_grab_priority",
        mThreadPrioGrab, mThreadPrioGrab,
        " * Grab thread priority: ");
      sl_tools::getParam(
        shared_from_this(), "advanced.thread_sensor_priority",
        mThreadPrioSens, mThreadPrioSens,
        " * Sensors thread priority: ");
      sl_tools::getParam(
        shared_from_this(),
        "advanced.thread_pointcloud_priority",
        mThreadPrioPointCloud, mThreadPrioPointCloud,
        " * Point Cloud thread priority: ");
    }
  }
}

rcl_interfaces::msg::SetParametersResult ZedCamera::callback_dynamicParamChange(
  std::vector<rclcpp::Parameter> parameters)
{
  DEBUG_STREAM_COMM("Parameter change callback");

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  DEBUG_STREAM_COMM("Modifying " << parameters.size() << " parameters");

  int count = 0;

  for (const rclcpp::Parameter & param : parameters) {
    count++;

    DEBUG_STREAM_COMM("Param #" << count << ": " << param.get_name());


    if (param.get_name() == "pos_tracking.transform_time_offset") {
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
    } else if (param.get_name() == "sensors.sensors_pub_rate") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }
      double value = param.as_double();
      if (value != mSensPubRate) {
        mSensPubRate = value;
        mPubImuTF_sec->setNewSize(static_cast<int>(mSensPubRate));
        mImuPeriodMean_sec->setNewSize(static_cast<int>(mSensPubRate));
        RCLCPP_INFO_STREAM(
          get_logger(), "Parameter '" << param.get_name()
                                      << "' correctly set to "
                                      << value);
      }
    } else if (param.get_name() == "svo.replay_rate") {
      rclcpp::ParameterType correctType =
        rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (param.get_type() != correctType) {
        result.successful = false;
        result.reason =
          param.get_name() + " must be a " + rclcpp::to_string(correctType);
        RCLCPP_WARN_STREAM(get_logger(), result.reason);
        break;
      }
      double value = param.as_double();

      mSvoRate = value;
      RCLCPP_INFO_STREAM(
        get_logger(), "Parameter '" << param.get_name()
                                    << "' correctly set to "
                                    << value);
    }

    // ----> Video/Depth dynamic parameters
    if (!handleVideoDepthDynamicParams(param, result)) {
      break;
    }
    // <---- Video/Depth dynamic parameters

    // ----> Object Detection dynamic parameters
    if (mUsingCustomOd) {
      if (!handleCustomOdDynamicParams(param, result)) {
        break;
      }
    } else {
      if (!handleOdDynamicParams(param, result)) {
        break;
      }
    }
    // <---- Object Detection dynamic parameters

    // ----> Body Tracking dynamica parameters
    if (!handleBodyTrkDynamicParams(param, result)) {
      break;
    }
    // <---- Body Tracking dynamica parameters
  }

  if (result.successful) {
    DEBUG_STREAM_COMM(
      "Correctly set " << count << "/" << parameters.size()
                       << " parameters");
  } else {
    DEBUG_STREAM_COMM(
      "Correctly set " << count - 1 << "/"
                       << parameters.size() << " parameters");
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

  // Print TF frames
  RCLCPP_INFO_STREAM(get_logger(), "=== TF FRAMES ===");
  RCLCPP_INFO_STREAM(get_logger(), " * Map\t\t\t-> " << mMapFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Odometry\t\t-> " << mOdomFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Base\t\t\t-> " << mBaseFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Camera\t\t-> " << mCameraFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Left\t\t\t-> " << mLeftCamFrameId);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Left Optical\t\t-> " << mLeftCamOptFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Right\t\t\t-> " << mRightCamFrameId);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Right Optical\t\t-> " << mRightCamOptFrameId);
  if (!mDepthDisabled) {
    RCLCPP_INFO_STREAM(get_logger(), " * Depth\t\t\t-> " << mDepthFrameId);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Depth Optical\t\t-> " << mDepthOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Point Cloud\t\t-> " << mPointCloudFrameId);
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

void ZedCamera::initPublishers()
{
  RCLCPP_INFO(get_logger(), "=== PUBLISHED TOPICS ===");

  // ----> Topics names definition
  mPointcloudFusedTopic = mTopicRoot + "mapping/fused_cloud";

  std::string object_det_topic_root = "obj_det";
  mObjectDetTopic = mTopicRoot + object_det_topic_root + "/objects";

  std::string body_trk_topic_root = "body_trk";
  mBodyTrkTopic = mTopicRoot + body_trk_topic_root + "/skeletons";

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

  // Status topic name
  std::string status_root = mTopicRoot + "status/";
  std::string svo_status_topic = status_root + "svo";
  std::string health_status_topic = status_root + "health";
  std::string heartbeat_topic = status_root + "heartbeat";
  // <---- Topics names definition

  // ----> SVO Status publisher
  if (mSvoMode) {
    mPubSvoStatus = create_publisher<zed_msgs::msg::SvoStatus>(
      svo_status_topic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubSvoStatus->get_topic_name());
    if (mUseSvoTimestamp && mPublishSvoClock) {
      auto clock_qos = rclcpp::ClockQoS();
      clock_qos.reliability(rclcpp::ReliabilityPolicy::Reliable); // REQUIRED
      mPubClock =
        create_publisher<rosgraph_msgs::msg::Clock>("/clock", clock_qos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Advertised on topic: " << mPubClock->get_topic_name());
    }
  }
  // <---- SVO Status publisher

  // ----> Health Status publisher
  mPubHealthStatus = create_publisher<zed_msgs::msg::HealthStatusStamped>(
    health_status_topic,
    mQos, mPubOpt);
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubHealthStatus->get_topic_name());
  // <---- Health Status publisher

  // ----> Heartbeat Status publisher
  mPubHeartbeatStatus = create_publisher<zed_msgs::msg::Heartbeat>(
    heartbeat_topic,
    mQos, mPubOpt);
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPubHeartbeatStatus->get_topic_name());
  // <---- Heartbeat Status publisher

  initVideoDepthPublishers();

  if (!mDepthDisabled) {
    // ----> Pos Tracking
    mPubPose = create_publisher<geometry_msgs::msg::PoseStamped>(
      mPoseTopic,
      mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubPose->get_topic_name());
    mPubPoseStatus = create_publisher<zed_msgs::msg::PosTrackStatus>(
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
      mPubGnssPoseStatus = create_publisher<zed_msgs::msg::GnssFusionStatus>(
        mGnssPoseStatusTopic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Advertised on topic: " << mPubGnssPoseStatus->get_topic_name());
      mPubGeoPose = create_publisher<geographic_msgs::msg::GeoPoseStamped>(
        mGeoPoseTopic, mQos, mPubOpt);
      RCLCPP_INFO_STREAM(
        get_logger(), "Advertised on topic (GNSS): "
          << mPubGeoPose->get_topic_name());
      mPubGeoPoseStatus = create_publisher<zed_msgs::msg::GnssFusionStatus>(
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
        shared_from_this(), mPointcloudFusedTopic, mQos.get_rmw_qos_profile(),
        mPubOpt);
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

    std::string marker_topic = mTopicRoot + "plane_marker";
    std::string plane_topic = mTopicRoot + "plane";
    // Rviz markers publisher
    mPubMarker = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic: " << mPubMarker->get_topic_name());
    // Detected planes publisher
    mPubPlane = create_publisher<zed_msgs::msg::PlaneStamped>(
      plane_topic, mQos,
      mPubOpt);
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
  RCLCPP_INFO(get_logger(), "===Subscribers ===");
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
  RCLCPP_INFO(get_logger(), "=== STARTING CAMERA ===");

  // // CUDA context check
  // CUcontext * primary_cuda_context;
  // cuCtxGetCurrent(primary_cuda_context);
  // int ctx_gpu_id;
  // cudaGetDevice(&ctx_gpu_id);

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

  // if (primary_cuda_context) {
  //   mInitParams.sdk_cuda_ctx = *primary_cuda_context;
  // } else {
  //   RCLCPP_INFO(
  //     get_logger(),
  //     "No ready CUDA context found, using default ZED SDK context.");
  // }

  if (mSimMode) {  // Simulation?
    RCLCPP_INFO_STREAM(
      get_logger(), "=== CONNECTING TO THE SIMULATION SERVER ["
        << mSimAddr.c_str() << ":" << mSimPort
        << "] ===");

    mInitParams.input.setFromStream(mSimAddr.c_str(), mSimPort);
  } else if (!mSvoFilepath.empty()) {
    RCLCPP_INFO(get_logger(), "=== SVO OPENING ===");

    mInitParams.input.setFromSVOFile(mSvoFilepath.c_str());
    mInitParams.svo_real_time_mode = mSvoRealtime;
  } else if (!mStreamAddr.empty()) {
    RCLCPP_INFO(get_logger(), "=== LOCAL STREAMING OPENING ===");

    mInitParams.input.setFromStream(mStreamAddr.c_str(), static_cast<unsigned short>(mStreamPort));
  } else {
    RCLCPP_INFO(get_logger(), "=== CAMERA OPENING ===");

    mInitParams.camera_fps = mCamGrabFrameRate;
    //mInitParams.grab_compute_capping_fps = static_cast<float>(mVdPubRate); // Using Wrapper multi-threading instead
    mInitParams.grab_compute_capping_fps = 0.0f;
    mInitParams.camera_resolution = static_cast<sl::RESOLUTION>(mCamResol);
    mInitParams.async_image_retrieval = mAsyncImageRetrieval;
    mInitParams.enable_image_validity_check = mImageValidityCheck;

    if (mCamSerialNumber > 0) {
      mInitParams.input.setFromSerialNumber(mCamSerialNumber);
    } else if (mCamId >= 0) {
      mInitParams.input.setFromCameraID(mCamId);
    }
  }

  mInitParams.coordinate_system = ROS_COORDINATE_SYSTEM;
  mInitParams.coordinate_units = ROS_MEAS_UNITS;
  mInitParams.depth_mode = mDepthMode;
  mInitParams.sdk_verbose = mVerbose;
  mInitParams.sdk_verbose_log_file = mVerboseLogFile.c_str();
  mInitParams.sdk_gpu_id = mGpuId;
  mInitParams.depth_stabilization = mDepthStabilization;
  mInitParams.camera_image_flip = (mCameraFlip ? sl::FLIP_MODE::ON : sl::FLIP_MODE::OFF);
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

  // Set the maximum working resolution between video and point cloud to boost the pipeline processing
  if (mMatResol.width > mPcResol.width) {
    mInitParams.maximum_working_resolution = mMatResol;
  } else {
    mInitParams.maximum_working_resolution = mPcResol;
  }
  // <---- ZED configuration

  // ----> Try to connect to a camera, to a stream, or to load an SVO
  sl_tools::StopWatch connectTimer(get_clock());

  mThreadStop = false;
  mGrabStatus = sl::ERROR_CODE::LAST;

  while (1) {
    rclcpp::sleep_for(500ms);

    mConnStatus = mZed->open(mInitParams);

    if (mConnStatus == sl::ERROR_CODE::SUCCESS) {
      DEBUG_STREAM_COMM("Opening successfull");
      mUptimer.tic(); // Sets the beginning of the camera connection time
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

    mDiagUpdater.force_update();

    rclcpp::sleep_for(std::chrono::seconds(mCamTimeoutSec));
  }
  // ----> Try to connect to a camera, to a stream, or to load an SVO

  // ----> Set SVO first frame if required
  if (mSvoMode && mSvoFrameStart != 0) {
    int svo_frames = mZed->getSVONumberOfFrames();

    if (mSvoFrameStart > svo_frames) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "The SVO contains " << svo_frames << " frames. The requested starting frame ("
                            << mSvoFrameStart << ") is invalid.");
      return false;
    }

    mZed->setSVOPosition(mSvoFrameStart);
    RCLCPP_WARN_STREAM(
      get_logger(),
      "SVO playing from frame #" << mSvoFrameStart);
  }


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
    if (!mSvoMode) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "!!! `general.grab_frame_rate` value is not valid: '"
          << mCamGrabFrameRate
          << "'. Automatically replaced with '" << realFps
          << "'. Please fix the parameter !!!");
    }
    mCamGrabFrameRate = realFps;
  }
  if (mSvoMode && !mSvoRealtime) {
    mVdPubRate = static_cast<double>(mCamGrabFrameRate) * mSvoRate;
  }

  // CUdevice devid;
  cuCtxGetDevice(&mGpuId);
  RCLCPP_INFO_STREAM(get_logger(), " * ZED SDK running on GPU #" << mGpuId);

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

  // ----> Update HW ID
  std::string hw_id = std::string("Stereolabs ");
  hw_id += sl::toString(mCamRealModel).c_str();
  hw_id += " - '" + mCameraName + "'" + " - S/N: " + std::to_string(mCamSerialNumber);
  mDiagUpdater.setHardwareID(hw_id);
  mDiagUpdater.force_update();
  // <---- Update HW ID

  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Focal Lenght\t-> "
      << camInfo.camera_configuration.calibration_parameters
      .left_cam.focal_length_metric
      << " mm");

  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Input\t\t-> "
      << sl::toString(mZed->getCameraInformation().input_type).c_str());
  if (mSvoMode) {
  #if (ZED_SDK_MAJOR_VERSION * 10 + ZED_SDK_MINOR_VERSION) >= 50
    RCLCPP_INFO(
      get_logger(), " * SVO resolution -> %dx%d",
      mZed->getCameraInformation().camera_configuration.resolution.width,
      mZed->getCameraInformation().camera_configuration.resolution.height);
  #else
    RCLCPP_INFO(
      get_logger(), " * SVO resolution -> %ldx%ld",
      mZed->getCameraInformation().camera_configuration.resolution.width,
      mZed->getCameraInformation().camera_configuration.resolution.height);
  #endif
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * SVO framerate\t -> "
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
    get_logger(), " * Camera grab size -> "
      << mCamWidth << "x" << mCamHeight);

  int pub_w = static_cast<int>(std::round(mCamWidth / mCustomDownscaleFactor));
  int pub_h = static_cast<int>(std::round(mCamHeight / mCustomDownscaleFactor));
  mMatResol = sl::Resolution(pub_w, pub_h);

  RCLCPP_INFO_STREAM(
    get_logger(), " * Color/Depth publishing size -> "
      << mMatResol.width << "x" << mMatResol.height);
  // <---- Camera information

  // ----> Point Cloud resolution
  int pc_w = 0, pc_h = 0;
  switch (mPcResolution) {
    case PcRes::PUB: // Same as image and depth map
      pc_w = pub_w;
      pc_h = pub_h;
      break;
    case PcRes::FULL:
      pc_w = NEURAL_W;
      pc_h = NEURAL_H;
      break;
    case PcRes::COMPACT:
      pc_w = NEURAL_W / 2;
      pc_h = NEURAL_H / 2;
      break;
    case PcRes::REDUCED:
      pc_w = NEURAL_W / 4;
      pc_h = NEURAL_H / 4;
      break;
  }
  mPcResol = sl::Resolution(pc_w, pc_h);

  RCLCPP_INFO_STREAM(
    get_logger(), " * Point Cloud publishing size -> "
      << mPcResol.width << "x" << mPcResol.height);
  // <---- Point Cloud resolution1


  // ----> Set Region of Interest
  if (!mDepthDisabled) {
    if (mAutoRoiEnabled) {
      RCLCPP_INFO(get_logger(), "=== Enabling Automatic ROI ===");

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
      RCLCPP_INFO(get_logger(), "=== Setting Manual ROI ===");
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
  if (_debugCamCtrl && !mSvoMode) {
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
      int value_min, value_max;

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

      setting = sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE;
      err = mZed->getCameraSettings(setting, value_min, value_max);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str() << ": " <<
            sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "[ZEDX] Default value for " <<
          sl::toString(setting).c_str() << ": [" << value_min << "," <<
          value_max
                                    << "]");

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

      setting = sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE;
      err = mZed->getCameraSettings(setting, value_min, value_max);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str() << ": " <<
            sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "[ZEDX] Default value for " <<
          sl::toString(setting).c_str() << ": [" << value_min << "," <<
          value_max
                                    << "]");

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

      setting = sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE;
      err = mZed->getCameraSettings(setting, value_min, value_max);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str() << ": " <<
            sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "[ZEDX] Default value for " <<
          sl::toString(setting).c_str() << ": [" << value_min << "," <<
          value_max
                                    << "]");

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
  mLeftCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mLeftCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mRightCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mRightCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();

  setTFCoordFrameNames();  // Requires mZedRealCamModel available only after
                           // camera opening

  fillCamInfo(
    mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId,
    mRightCamOptFrameId);
  fillCamInfo(
    mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId,
    mRightCamOptFrameId, true);
  // <---- Camera Info messages

  initPublishers();  // Requires mZedRealCamModel available only after camera
                     // opening
  initSubscribers();

  // Disable AEC_AGC and Auto Whitebalance to trigger it if user set it to
  // automatic
  if (!mSvoMode && !mSimMode) {
    mZed->setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
    mZed->setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0);

    // Lock on Positional Tracking mutex to avoid race conditions
    std::lock_guard<std::mutex> lock(mPtMutex);

    // Force parameters with a dummy grab
    mZed->grab();
  }

  // Initialialized timestamp to avoid wrong initial data
  // ----> Timestamp
  if (mSvoMode) {
    if (mUseSvoTimestamp) {
      mFrameTimestamp = sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));

      DEBUG_COMM("=========================================================*");
      DEBUG_STREAM_COMM("SVO Timestamp\t\t" << mFrameTimestamp.nanoseconds() << " nsec");
      DEBUG_STREAM_COMM(
        "Current Timestamp\t" <<
          sl_tools::slTime2Ros(
          mZed->getTimestamp(
            sl::TIME_REFERENCE::CURRENT)).nanoseconds() << " nsec");
      DEBUG_COMM("=========================================================*");
    } else {
      mFrameTimestamp =
        sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::CURRENT));
    }
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
  mImuPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(20);
  mBaroPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(20);
  mMagPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(20);
  mPubFusedCloudPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mPcPubRate);
  mPubOdomTF_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
  mPubPoseTF_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
  mPubImuTF_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
  mGnssFix_sec = std::make_unique<sl_tools::WinAvg>(10);
  // <---- Initialize Diagnostic statistics

  if (mGnssFusionEnabled) {
    DEBUG_GNSS("Initialize Fusion module");

    // ----> Retrieve GNSS to ZED transform
    RCLCPP_INFO(get_logger(), "=== Initialize GNSS Offset ===");
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
    mFusionInitParams.timeout_period_number = 20;

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

void ZedCamera::closeCamera()
{
  std::lock_guard<std::mutex> lock(mCloseCameraMutex);
  if (mZed == nullptr) {
    return;
  }

  RCLCPP_INFO(get_logger(), "=== CLOSING CAMERA ===");

  if (mPosTrackingStarted && !mAreaMemoryFilePath.empty() &&
    mSaveAreaMemoryOnClosing)
  {
    saveAreaMemoryFile(mAreaMemoryFilePath);
  }

  mZed->close();
  mZed.reset();
  DEBUG_COMM("Camera closed");
}

void ZedCamera::initThreads()
{
  // Start Heartbeat timer
  startHeartbeatTimer();

  // ----> Start CMOS Temperatures thread
  if (!mSimMode && !sl_tools::isZED(mCamRealModel) &&
    !sl_tools::isZEDM(mCamRealModel))
  {
    startTempPubTimer();
  }
  // <---- Start CMOS Temperatures thread

  // ----> Start Sensors thread if not sync
  if (!mSensCameraSync && !sl_tools::isZED(mCamRealModel)) {
    mSensThread = std::thread(&ZedCamera::threadFunc_pubSensorsData, this);
  }
  // <---- Start Sensors thread if not sync

  // ----> Start Video/Depth thread
  mVdDataReady = false;
  mVdThread = std::thread(&ZedCamera::threadFunc_videoDepthElab, this);
  // <---- Start Video/Depth thread

  // ----> Start Pointcloud thread
  if (!mDepthDisabled) {
    mPcDataReady = false;
    mPcThread = std::thread(&ZedCamera::threadFunc_pointcloudElab, this);
  }
  // <---- Start Pointcloud thread

  // Start grab thread
  mGrabThread = std::thread(&ZedCamera::threadFunc_zedGrab, this);
}

void ZedCamera::startHeartbeatTimer()
{
  if (mHeartbeatTimer != nullptr) {
    mHeartbeatTimer->cancel();
  }

  std::chrono::milliseconds pubPeriod_msec(1000);
  mHeartbeatTimer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(pubPeriod_msec),
    std::bind(&ZedCamera::callback_pubHeartbeat, this));
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
  // Lock on Positional Tracking mutex to avoid race conditions
  std::lock_guard<std::mutex> lock(mPtMutex);

  if (mDepthDisabled) {
    RCLCPP_WARN(
      get_logger(),
      "Cannot start Positional Tracking if "
      "`depth.depth_mode` is set to `0` [NONE]");
    return false;
  }

  if (mZed && mZed->isPositionalTrackingEnabled()) {
    if (!mAreaMemoryFilePath.empty() && mSaveAreaMemoryOnClosing) {
      mZed->disablePositionalTracking(mAreaMemoryFilePath.c_str());
      RCLCPP_INFO(
        get_logger(), "Area memory updated before restarting the Positional Tracking module.");
    } else {
      mZed->disablePositionalTracking();
    }
  }

  RCLCPP_INFO(get_logger(), "=== Starting Positional Tracking ===");

  RCLCPP_INFO(get_logger(), " * Waiting for valid static transformations...");

  bool transformOk = false;
  double elapsed = 0.0;
  mPosTrackingReady = false;
  mGnssInitGood = false;

  // auto start = std::chrono::high_resolution_clock::now();

  sl_tools::StopWatch stopWatch(get_clock());

  do {
    transformOk =// true;
      setPose(
      mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2],
      mInitialBasePose[3], mInitialBasePose[4], mInitialBasePose[5]);

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

  // Tracking parameters
  sl::PositionalTrackingParameters ptParams;

  mPoseSmoothing = false;  // Always false. Pose Smoothing is to be enabled only
                           // for VR/AR applications

  ptParams.enable_pose_smoothing = mPoseSmoothing;
  ptParams.enable_area_memory = mAreaMemory;
  ptParams.area_file_path = (mAreaFileExists ? mAreaMemoryFilePath.c_str() : "");
  ptParams.enable_imu_fusion = mImuFusion;
  ptParams.initial_world_transform = mInitialPoseSl;
  ptParams.set_floor_as_origin = mFloorAlignment;
  ptParams.depth_min_range = mPosTrackDepthMinRange;
  ptParams.set_as_static = mSetAsStatic;
  ptParams.set_gravity_as_origin = mSetGravityAsOrigin;
  ptParams.mode = mPosTrkMode;

  if (_debugPosTracking) {
    DEBUG_PT(" * Positional Tracking parameters:");
    sl::String json;
    ptParams.encode(json);
    DEBUG_PT(json.c_str());
  }

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

bool ZedCamera::saveAreaMemoryFile(const std::string & filePath)
{
  if (!mZed) {
    RCLCPP_WARN(get_logger(), "ZED camera is not initialized");
    return false;
  }

  if (!mAreaMemory) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to save area memory: 'Area Memory was not enabled'");
    return false;
  }

  RCLCPP_INFO_STREAM(get_logger(), "Saving area memory to: '" << filePath << "' ...");
  sl::ERROR_CODE err = mZed->saveAreaMap(filePath.c_str());

  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Failed to save area memory: '"
        << sl::toString(err) << "'");
    return false;
  }

  auto export_state = sl::AREA_EXPORTING_STATE::RUNNING;
  while (export_state == sl::AREA_EXPORTING_STATE::RUNNING) {
    export_state = mZed->getAreaExportState();
    sl::sleep_ms(5);
  }

  if (export_state != sl::AREA_EXPORTING_STATE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Failed to save area memory: '"
        << sl::toString(export_state) << "'");
    return false;
  }

  RCLCPP_INFO(get_logger(), "... Area memory saved successfully");
  return true;
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

  RCLCPP_INFO_STREAM(get_logger(), "=== Starting Spatial Mapping ===");

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
        shared_from_this(), mPointcloudFusedTopic, mQos.get_rmw_qos_profile(),
        mPubOpt);
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

  RCLCPP_INFO(get_logger(), "=== Spatial Mapping stopped ===");
}

bool ZedCamera::startSvoRecording(std::string & errMsg)
{
  sl::RecordingParameters params;

  params.bitrate = mSvoRecBitrate;
  params.compression_mode = mSvoRecCompression;
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

  auto cameraImuTransfMgs = std::make_unique<geometry_msgs::msg::TransformStamped>();

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
    DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic exception: ");
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
  if (_debugAdvanced) {
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

  if (_debugAdvanced) {
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
    auto t0 = get_clock()->now().nanoseconds();
    try {
      // ----> Interruption check
      if (!rclcpp::ok()) {
        mThreadStop = true;
        DEBUG_STREAM_COMM("Ctrl+C received: stopping grab thread");
        break;
      }

      if (mThreadStop) {
        DEBUG_STREAM_COMM("Grab thread stopped");
        break;
      }
      // <---- Interruption check

      if (mSvoMode && mSvoPause) {
        if (!mGrabOnce) {
          rclcpp::sleep_for(100ms);
  #ifdef USE_SVO_REALTIME_PAUSE
          // Lock on Positional Tracking mutex to avoid race conditions
          std::lock_guard<std::mutex> lock(mPtMutex);

          // Dummy grab
          mZed->grab();
  #endif
          continue;
        } else {
          mGrabOnce = false; // Reset the flag and grab once
        }
      }

      if (mUseSimTime && !mClockAvailable) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(
          get_logger(), steady_clock, 5000.0,
          "Waiting for a valid simulation time on the '/clock' topic...");
        continue;
      }

      sl_tools::StopWatch grabElabTimer(get_clock());

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

      // ----> Publish SVO Status information
      if (mSvoMode) {
        publishSvoStatus(mFrameTimestamp.nanoseconds());
      }
      // <---- Publish SVO Status information

      // Lock on Positional Tracking mutex to avoid race conditions
      std::lock_guard<std::mutex> lock(mPtMutex);

      // Start processing timer for diagnostic
      grabElabTimer.tic();

      // ZED grab
      //DEBUG_STREAM_COMM("Grab thread: grabbing frame #" << mFrameCount);

      mGrabStatus = mZed->grab(mRunParams);

      //DEBUG_COMM("Grabbed");

      // ----> Grab errors?
      // Note: disconnection are automatically handled by the ZED SDK
      if (mGrabStatus != sl::ERROR_CODE::SUCCESS) {
        if (mSvoMode && mGrabStatus == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
          // ----> Check SVO status
          if (mSvoLoop) {
            mSvoLoopCount++;
            mZed->setSVOPosition(mSvoFrameStart);
            RCLCPP_WARN_STREAM(
              get_logger(),
              "SVO reached the end and it has been restarted from frame #" << mSvoFrameStart);
            rclcpp::sleep_for(
              std::chrono::microseconds(
                static_cast<int>(mGrabPeriodMean_sec->getAvg() * 1e6)));
            if (mResetPoseWithSvoLoop) {
              RCLCPP_WARN(
                get_logger(),
                " * Camera pose reset to initial conditions.");

              mResetOdomFromSrv = true;
              mOdomPath.clear();
              mPosePath.clear();

              // Restart tracking
              startPosTracking();
            }
            continue;
          } else {
            // ----> Stop all the other threads and Timers
            mThreadStop = true;
            if (mPathTimer) {mPathTimer->cancel();}
            if (mFusedPcTimer) {mFusedPcTimer->cancel();}
            if (mTempPubTimer) {mTempPubTimer->cancel();}
            if (mGnssPubCheckTimer) {mGnssPubCheckTimer->cancel();}
            // <---- Stop all the other threads and Timers

            RCLCPP_WARN(get_logger(), "SVO reached the end.");

            // Force SVO status update
            if (!publishSvoStatus(mFrameTimestamp.nanoseconds())) {
              RCLCPP_WARN(get_logger(), "Node stopped. Press Ctrl+C to exit.");
              break;
            } else {
              RCLCPP_WARN_STREAM(
                get_logger(),
                "Waiting for SVO status subscribers to unsubscribe. Active subscribers: " <<
                  mPubSvoStatus->get_subscription_count());
              mDiagUpdater.force_update();
              rclcpp::sleep_for(1s);
              continue;
            }
          }
          // <---- Check SVO status
        } else if (mGrabStatus == sl::ERROR_CODE::CAMERA_REBOOTING) {
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "Connection issue detected: "
              << sl::toString(mGrabStatus).c_str());
          rclcpp::sleep_for(1s);
          continue;
        } else if (mGrabStatus == sl::ERROR_CODE::CAMERA_NOT_INITIALIZED ||
          mGrabStatus == sl::ERROR_CODE::FAILURE)
        {
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "Camera issue detected: "
              << sl::toString(mGrabStatus).c_str() << ". Trying to recover the connection...");
          rclcpp::sleep_for(1s);
          continue;
        } else if (mGrabStatus == sl::ERROR_CODE::CORRUPTED_FRAME) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            "Corrupted frame detected: "
              << sl::toString(mGrabStatus).c_str());
          static const int frame_grab_period =
            static_cast<int>(std::round(1000. / mCamGrabFrameRate));
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
      if (mSvoMode) {
        mSvoFrameId = mZed->getSVOPosition();
        mSvoFrameCount = mZed->getSVONumberOfFrames();

        // ----> Publish Clock if required
        if (mUseSvoTimestamp && mPublishSvoClock) {
          publishClock(mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
        }
        // <---- Publish Clock if required
      }

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
        if (mUseSvoTimestamp) {
          mFrameTimestamp = sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
        } else {
          mFrameTimestamp =
            sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::CURRENT));
        }
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
      //DEBUG_STREAM_COMM("Grab timestamp: " << mFrameTimestamp.nanoseconds() << " nsec");
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

      publishHealthStatus();

      // ----> Check recording status
      mRecMutex.lock();
      if (mRecording) {
        mRecStatus = mZed->getRecordingStatus();
        static int svo_rec_err_count = 0;
        if (mRecStatus.is_recording && !mRecStatus.status) {
          if (++svo_rec_err_count > 3) {
            rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_WARN_THROTTLE(
              get_logger(), steady_clock, 1000.0,
              "Error saving frame to SVO");
          }
        } else {
          svo_rec_err_count = 0;
        }
      }
      mRecMutex.unlock();
      // <---- Check recording status

      // ----> Retrieve Image/Depth data if someone has subscribed to
      processVideoDepth();
      // <---- Retrieve Image/Depth data if someone has subscribed to

      if (!mDepthDisabled) {
        // ----> Retrieve the point cloud if someone has subscribed to
        processPointCloud();
        // <---- Retrieve the point cloud if someone has subscribed to

        // ----> Localization processing
        if (mPosTrackingStarted) {
          if (!mSvoPause) {
            DEBUG_PT("================================================================");
            DEBUG_PT("=== processOdometry ===");
            processOdometry();
            DEBUG_PT("=== processPose ===");
            processPose();
            if (mGnssFusionEnabled) {
              if (mSvoMode) {
                DEBUG_PT("=== processSvoGnssData ===");
                processSvoGnssData();
              }
              DEBUG_PT("=== processGeoPose ===");
              processGeoPose();
            }
          }

          // Publish `odom` and `map` TFs at the grab frequency
          // RCLCPP_INFO(get_logger(), "Publishing TF -> threadFunc_zedGrab");
          DEBUG_PT("=== publishTFs ===");
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

    if (mSvoMode && !mSvoRealtime) {
      double effective_grab_period = mElabPeriodMean_sec->getAvg();
      mSvoExpectedPeriod = 1.0 / (mSvoRate * static_cast<double>(mCamGrabFrameRate));
      double sleep = std::max(0.001, mSvoExpectedPeriod - effective_grab_period);
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep * 1000)));


      DEBUG_STREAM_COMM(
        "SVO sleep time: " << sleep << " sec - Expecter grab period:"
                           << mSvoExpectedPeriod << " sec - Elab time:"
                           << effective_grab_period << " sec");
    }
  }

  // Stop the heartbeat
  mHeartbeatTimer->cancel();

  DEBUG_STREAM_COMM("Grab thread finished");
}

bool ZedCamera::publishSensorsData(rclcpp::Time force_ts)
{
  if (mGrabStatus != sl::ERROR_CODE::SUCCESS && mGrabStatus != sl::ERROR_CODE::CORRUPTED_FRAME) {
    DEBUG_SENS("Camera not ready");
    rclcpp::sleep_for(1s);
    return false;
  }

  // ----> Subscribers count
  DEBUG_STREAM_SENS("Sensors callback: counting subscribers");

  size_t imu_SubCount = 0;
  size_t imu_RawSubCount = 0;
  size_t imu_TempSubCount = 0;
  size_t imu_MagSubCount = 0;
  size_t pressSubCount = 0;

  try {
    imu_SubCount = count_subscribers(mPubImu->get_topic_name());
    imu_RawSubCount = count_subscribers(mPubImuRaw->get_topic_name());
    imu_MagSubCount = 0;
    pressSubCount = 0;

    if (sl_tools::isZED2OrZED2i(mCamRealModel)) {
      imu_MagSubCount = count_subscribers(mPubImuMag->get_topic_name());
      pressSubCount = count_subscribers(mPubPressure->get_topic_name());
    }
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_SENS("pubSensorsData: Exception while counting subscribers");
    return false;
  }
  // <---- Subscribers count

  // ----> Grab data and setup timestamps
  DEBUG_STREAM_ONCE_SENS("Sensors callback: Grab data and setup timestamps");
  rclcpp::Time ts_imu;
  rclcpp::Time ts_baro;
  rclcpp::Time ts_mag;

  rclcpp::Time now = get_clock()->now();

  sl::SensorsData sens_data;
  sl::ERROR_CODE err;

  if (mSensCameraSync) {
    err = mZed->getSensorsData(sens_data, sl::TIME_REFERENCE::IMAGE);
  } else {
    err = mZed->getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT);
  }

  if (err != sl::ERROR_CODE::SUCCESS) {
    if (mSvoMode && err != sl::ERROR_CODE::SENSORS_NOT_AVAILABLE) {
      RCLCPP_WARN_STREAM(
        get_logger(), "sl::getSensorsData error: "
          << sl::toString(err).c_str());
    }
    return false;
  }

  if (mSensCameraSync) {
    ts_imu = force_ts;
    ts_baro = force_ts;
    ts_mag = force_ts;
  } else if (mSvoMode && !mUseSvoTimestamp) {
    ts_imu = now;
    ts_baro = now;
    ts_mag = now;
  } else if (mSimMode) {
    if (mUseSimTime) {
      ts_imu = now;
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

  // ----> Respect data frequency for SVO2
  if (mSvoMode) {
    const double imu_period = 1.0 / mSensPubRate;

    if (dT < imu_period) {
      DEBUG_SENS("SENSOR: IMU data not ready yet");
      return false;
    }
  }
  DEBUG_STREAM_SENS(
    "IMU TS: " << ts_imu.seconds() << " - Grab TS: " << mFrameTimestamp.seconds() << " - Diff: " <<
      mFrameTimestamp.seconds() - ts_imu.seconds());
  // <---- Respect data frequency for SVO2

  if (!new_imu_data && !new_baro_data && !new_mag_data) {
    DEBUG_STREAM_SENS("No new sensors data");
    return false;
  }

  if (mSimMode) {
    new_baro_data = false;
    new_mag_data = false;
  }
  // <---- Check for duplicated data

  mLastTs_imu = ts_imu;

  DEBUG_STREAM_SENS("SENSOR LAST PERIOD: " << dT << " sec @" << 1. / dT << " Hz");

  // ----> Sensors freq for diagnostic
  if (new_imu_data) {
    double mean = mImuPeriodMean_sec->addValue(mImuFreqTimer.toc());
    mImuFreqTimer.tic();

    DEBUG_STREAM_SENS("Thread New data MEAN freq: " << 1. / mean);
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

    if (imu_SubCount > 0) {
      mImuPublishing = true;

      auto imuMsg = std::make_unique<sensor_msgs::msg::Imu>();

      imuMsg->header.stamp = mUsePubTimestamps ? get_clock()->now() : ts_imu;
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
      // Note: memcpy not allowed because ROS 2 uses double and ZED SDK uses
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
        DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
      } catch (...) {
        DEBUG_STREAM_COMM("Message publishing generic exception: ");
      }
    } else {
      mImuPublishing = false;
    }

    if (imu_RawSubCount > 0) {
      mImuPublishing = true;

      auto imuRawMsg = std::make_unique<sensor_msgs::msg::Imu>();

      imuRawMsg->header.stamp = mUsePubTimestamps ? get_clock()->now() : ts_imu;
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
      // Note: memcpy not allowed because ROS 2 uses double and ZED SDK uses
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
        DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
      } catch (...) {
        DEBUG_STREAM_COMM("Message publishing generic exception: ");
      }
    }
  }

  if (sens_data.barometer.is_available && new_baro_data) {
    if (pressSubCount > 0) {
      mBaroPublishing = true;

      auto pressMsg = std::make_unique<sensor_msgs::msg::FluidPressure>();

      pressMsg->header.stamp = mUsePubTimestamps ? get_clock()->now() : ts_baro;
      pressMsg->header.frame_id = mBaroFrameId;
      pressMsg->fluid_pressure =
        sens_data.barometer.pressure;    // Pascals -> see
      // https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/FluidPressure.msg
      pressMsg->variance = 1.0585e-2;

      DEBUG_STREAM_SENS("Publishing PRESS message");
      try {
        mPubPressure->publish(std::move(pressMsg));
      } catch (std::system_error & e) {
        DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
      } catch (...) {
        DEBUG_STREAM_COMM("Message publishing generic exception: ");
      }
    } else {
      mBaroPublishing = false;
    }
  }

  if (sens_data.magnetometer.is_available && new_mag_data) {
    if (imu_MagSubCount > 0) {
      mMagPublishing = true;

      auto magMsg = std::make_unique<sensor_msgs::msg::MagneticField>();

      magMsg->header.stamp = mUsePubTimestamps ? get_clock()->now() : ts_mag;
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
        DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
      } catch (...) {
        DEBUG_STREAM_COMM("Message publishing generic exception: ");
      }
    } else {
      mMagPublishing = false;
    }
  }
  // <---- Sensors data publishing

  return true;
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

  transformStamped.header.stamp =
    mUsePubTimestamps ? get_clock()->now() :
    (t + rclcpp::Duration(0, mTfOffset * 1e9));

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

  transformStamped.header.stamp =
    mUsePubTimestamps ? get_clock()->now() :
    (t + rclcpp::Duration(0, mTfOffset * 1e9));
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

void ZedCamera::threadFunc_pubSensorsData()
{
  DEBUG_STREAM_SENS("Sensors thread started");

  // ----> Advanced thread settings
  DEBUG_STREAM_ADV("Sensors thread settings");
  if (_debugAdvanced) {
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

  if (_debugAdvanced) {
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

      if (mSvoMode && mSvoPause) {
        if (!mGrabImuOnce) {
          rclcpp::sleep_for(100ms);
          continue;
        } else {
          mGrabImuOnce = false; // Reset the flag and grab the IMU data
        }
      }

      // std::lock_guard<std::mutex> lock(mCloseZedMutex);
      if (!mZed->isOpened()) {
        DEBUG_STREAM_SENS("threadFunc_pubSensorsData: the camera is not open");
        continue;
      }

      if (!publishSensorsData()) {
        auto sleep_usec =
          static_cast<int>(mSensRateComp * (1000000. / mSensPubRate));
        sleep_usec = std::max(100, sleep_usec);
        DEBUG_STREAM_SENS(
          "[threadFunc_pubSensorsData] Thread sleep: "
            << sleep_usec << " µsec");
        rclcpp::sleep_for(
          std::chrono::microseconds(sleep_usec)); // Avoid busy-waiting
        continue;
      }

      // ----> Check publishing frequency
      double sens_period_usec = 1e6 / mSensPubRate;
      double avg_freq = 1. / mImuPeriodMean_sec->getAvg();

      double err = std::fabs(mSensPubRate - avg_freq);

      const double COMP_P_GAIN = 0.0005;

      if (avg_freq < mSensPubRate) {
        mSensRateComp -= COMP_P_GAIN * err;
      } else if (avg_freq > mSensPubRate) {
        mSensRateComp += COMP_P_GAIN * err;
      }

      mSensRateComp = std::max(0.001, mSensRateComp);
      mSensRateComp = std::min(3.0, mSensRateComp);
      DEBUG_STREAM_SENS(
        "[threadFunc_pubSensorsData] mSensRateComp: " << mSensRateComp);
      // <---- Check publishing frequency
    } catch (...) {
      rcutils_reset_error();
      DEBUG_STREAM_COMM("threadFunc_pubSensorsData: Generic exception.");
      continue;
    }
  }

  DEBUG_STREAM_SENS("Sensors thread finished");
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
        "=== Odometry reset for LOOP CLOSURE event ===");
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
      "delta ODOM %s- [%s]:\n%s", _debugGnss ? "(`sl::Fusion`) " : "",
      sl::toString(mPosTrackingStatus.odometry_status).c_str(),
      deltaOdom.pose_data.getInfos().c_str());

    if (_debugGnss) {
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

  if (_debugPosTracking) {
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
    auto odomMsg = std::make_unique<nav_msgs::msg::Odometry>();

    odomMsg->header.stamp = mUsePubTimestamps ? get_clock()->now() : t;
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
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
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
    _debugGnss ? "(`sl::Fusion`) " : "", mLeftCamFrameId.c_str(),
    mMapFrameId.c_str(), mLastZedPose.pose_data.getInfos().c_str());

  if (_debugGnss) {
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
      "=== Base POSE [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
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
    auto msg = std::make_unique<zed_msgs::msg::PosTrackStatus>();
    msg->odometry_status = static_cast<uint8_t>(mPosTrackingStatus.odometry_status);
    msg->spatial_memory_status = static_cast<uint8_t>(mPosTrackingStatus.spatial_memory_status);

    try {
      mPubPoseStatus->publish(std::move(msg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
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
    auto msg = std::make_unique<zed_msgs::msg::GnssFusionStatus>();

    msg->gnss_fusion_status = static_cast<uint8_t>(mFusedPosTrackingStatus.gnss_fusion_status);

    try {
      mPubGnssPoseStatus->publish(std::move(msg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
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
    auto msg = std::make_unique<zed_msgs::msg::GnssFusionStatus>();

    msg->gnss_fusion_status =
      static_cast<uint8_t>(mFusedPosTrackingStatus.gnss_fusion_status);

    try {
      mPubGeoPoseStatus->publish(std::move(msg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
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
  header.stamp = mUsePubTimestamps ? get_clock()->now() : mFrameTimestamp;
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
    auto poseNoCov = std::make_unique<geometry_msgs::msg::PoseStamped>();

    poseNoCov->header = header;
    poseNoCov->pose = pose;

    // Publish pose stamped message
    DEBUG_STREAM_PT("Publishing POSE NO COV message");
    try {
      mPubPose->publish(std::move(poseNoCov));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
  }

  if (mPublishPoseCov) {
    if (poseCovSub > 0) {
      auto poseCov = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();

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
        DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
      } catch (...) {
        DEBUG_STREAM_COMM("Message publishing generic exception: ");
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
      mUsePubTimestamps ?
      get_clock()->now() :
      (mFrameTimestamp + rclcpp::Duration(0, mTfOffset * 1e9));
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
  DEBUG_GNSS("=== publishGnssPose ===");

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
    auto msg = std::make_unique<nav_msgs::msg::Odometry>();

    msg->header.stamp =
      mUsePubTimestamps ? get_clock()->now() : mFrameTimestamp;
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
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
  }

  if (geoPoseSub > 0) {
    auto msg = std::make_unique<geographic_msgs::msg::GeoPoseStamped>();

    msg->header.stamp =
      mUsePubTimestamps ? get_clock()->now() : mFrameTimestamp;
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
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
  }

  if (fusedFixSub > 0) {
    auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();

    msg->header.stamp =
      mUsePubTimestamps ? get_clock()->now() :
      mFrameTimestamp;
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
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
  }

  if (originFixSub > 0) {
    auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();

    msg->header.stamp = mUsePubTimestamps ?
      get_clock()->now() :
      mFrameTimestamp;
    msg->header.frame_id =
      mGnssOriginFrameId;

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
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
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

    if (!mPosTrackingEnabled) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "POSITIONAL TRACKING disabled in the parameters, but forced to "
        "ENABLE because required by `pos_tracking.publish_tf: true`");
    }
    return true;
  }

  if (mDepthStabilization > 0) {
    DEBUG_ONCE_PT(
      "POS. TRACKING required: enabled by depth stabilization param.");

    if (!mPosTrackingEnabled) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "POSITIONAL TRACKING disabled in the parameters, but forced to "
        "ENABLE because required by `depth.depth_stabilization > 0`");
    }

    return true;
  }

  if (mMappingEnabled) {
    DEBUG_ONCE_PT("POS. TRACKING required: enabled by mapping");

    if (!mPosTrackingEnabled) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "POSITIONAL TRACKING disabled in the parameters, but forced to "
        "ENABLE because required by `mapping.mapping_enabled: true`");
    }

    return true;
  }

  if (mObjDetEnabled && mObjDetTracking) {
    DEBUG_ONCE_PT("POS. TRACKING required: enabled by object detection.");

    if (!mPosTrackingEnabled) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "POSITIONAL TRACKING disabled in the parameters, but forced to "
        "ENABLE because required by `object_detection.enable_tracking: true`");
    }

    return true;
  }

  if (mBodyTrkEnabled && mBodyTrkEnableTracking) {
    DEBUG_ONCE_PT("POS. TRACKING required: enabled by body tracking.");

    if (!mPosTrackingEnabled) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "POSITIONAL TRACKING disabled in the parameters, but forced to "
        "ENABLE because required by `body_tracking.enable_tracking: true`");
    }

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

  if (mZed->isPositionalTrackingEnabled()) {

    DEBUG_ONCE_PT("POS. TRACKING required: enabled by ZED SDK.");

    RCLCPP_WARN_ONCE(
      get_logger(),
      "POSITIONAL TRACKING disabled in the parameters, enabled by the ZED SDK because required by one of the modules.");

    return true;
  }

  DEBUG_ONCE_PT("POS. TRACKING not required.");
  return false;
}

void ZedCamera::callback_pubTemp()
{
  DEBUG_STREAM_ONCE_SENS("Temperatures callback called");

  if (mGrabStatus != sl::ERROR_CODE::SUCCESS && mGrabStatus != sl::ERROR_CODE::CORRUPTED_FRAME) {
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
  size_t tempLeftSubCount = 0;
  size_t tempRightSubCount = 0;
  size_t tempImuSubCount = 0;

  try {
    tempLeftSubCount = 0;
    tempRightSubCount = 0;
    tempImuSubCount = 0;

    if (sl_tools::isZED2OrZED2i(mCamRealModel)) {
      tempLeftSubCount = count_subscribers(mPubTempL->get_topic_name());
      tempRightSubCount = count_subscribers(mPubTempR->get_topic_name());
    }
    tempImuSubCount = count_subscribers(mPubImuTemp->get_topic_name());
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_SENS(
      "callback_pubTemp: Exception while counting subscribers");
    return;
  }
  // <---- Subscribers count

  rclcpp::Time now = get_clock()->now();

  if (tempLeftSubCount > 0) {
    auto leftTempMsg = std::make_unique<sensor_msgs::msg::Temperature>();

    leftTempMsg->header.stamp = now;

    leftTempMsg->header.frame_id = mTempLeftFrameId;
    leftTempMsg->temperature = static_cast<double>(mTempLeft);
    leftTempMsg->variance = 0.0;

    try {
      mPubTempL->publish(std::move(leftTempMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
  }

  if (tempRightSubCount > 0) {
    auto rightTempMsg = std::make_unique<sensor_msgs::msg::Temperature>();

    rightTempMsg->header.stamp = now;

    rightTempMsg->header.frame_id = mTempRightFrameId;
    rightTempMsg->temperature = static_cast<double>(mTempRight);
    rightTempMsg->variance = 0.0;

    DEBUG_STREAM_SENS("Publishing RIGHT TEMP message");
    try {
      mPubTempR->publish(std::move(rightTempMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
  }

  if (tempImuSubCount > 0) {
    auto imuTempMsg = std::make_unique<sensor_msgs::msg::Temperature>();

    imuTempMsg->header.stamp = now;

    imuTempMsg->header.frame_id = mImuFrameId;
    imuTempMsg->temperature = static_cast<double>(mTempImu);
    imuTempMsg->variance = 0.0;

    DEBUG_SENS("Publishing IMU TEMP message");
    try {
      mPubImuTemp->publish(std::move(imuTempMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
  }
}

void ZedCamera::callback_pubFusedPc()
{
  DEBUG_STREAM_ONCE_MAP("Mapping callback called");

  auto pointcloudFusedMsg = std::make_unique<sensor_msgs::msg::PointCloud2>();

  uint32_t fusedCloudSubCount = 0;
  try {
#ifndef FOUND_FOXY
    fusedCloudSubCount = mPubFusedCloud.getNumSubscribers();
#else
    fusedCloudSubCount = count_subscribers(mPubFusedCloud->get_topic_name());
#endif
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_MAP("pubFusedPc: Exception while counting subscribers");
    return;
  }

  if (fusedCloudSubCount == 0) {
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
          pointcloudFusedMsg->header.stamp =
            mUsePubTimestamps ? get_clock()->now() : mFrameTimestamp;
        } else if (mSimMode) {
          if (mUseSimTime) {
            pointcloudFusedMsg->header.stamp =
              mUsePubTimestamps ? get_clock()->now() : mFrameTimestamp;
          } else {
            pointcloudFusedMsg->header.stamp = mUsePubTimestamps ? get_clock()->now() :
              sl_tools::slTime2Ros(mFusedPC.chunks[c].timestamp);
          }
        } else {
          pointcloudFusedMsg->header.stamp = mUsePubTimestamps ? get_clock()->now() :
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
    DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic exception: ");
  }
#else
  try {
    mPubFusedCloud->publish(std::move(pointcloudFusedMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic exception: ");
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
    auto mapPathMsg = std::make_unique<nav_msgs::msg::Path>();
    mapPathMsg->header.frame_id = mMapFrameId;
    mapPathMsg->header.stamp =
      mUsePubTimestamps ? get_clock()->now() :
      mFrameTimestamp;
    mapPathMsg->poses = mPosePath;

    DEBUG_STREAM_PT("Publishing MAP PATH message");
    try {
      mPubPosePath->publish(std::move(mapPathMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
  }

  if (odomPathSub > 0) {
    auto odomPathMsg = std::make_unique<nav_msgs::msg::Path>();
    odomPathMsg->header.frame_id = mOdomFrameId;
    odomPathMsg->header.stamp =
      mUsePubTimestamps ? get_clock()->now() :
      mFrameTimestamp;
    odomPathMsg->poses = mOdomPath;

    DEBUG_STREAM_PT("Publishing ODOM PATH message");
    try {
      mPubOdomPath->publish(std::move(odomPathMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
  }
}

/*void callback_saveAreaMemory(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_msgs::srv::SaveAreaMemory_Request> req,
    std::shared_ptr<zed_msgs::srv::SaveAreaMemory_Response> res);*/// TODO(Walter): Uncomment when available in `zed_msgs` package from APT
void ZedCamera::callback_saveAreaMemory(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<zed_msgs::srv::SetROI_Request> req,
  std::shared_ptr<zed_msgs::srv::SetROI_Response> res)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "** Save Area Memory_Response service called **");

  if (!mPosTrackingStarted) {
    RCLCPP_WARN(get_logger(), " * Pos. Tracking was not started");
    res->message = "Positional tracking not started";
    res->success = false;
    return;
  }

  std::string filename = req->roi;
  // std::string filename = req->area_file_path; // TODO(Walter): Uncomment when available in `zed_msgs` package from APT

  if (filename.empty()) {
    if (mAreaMemoryFilePath.empty()) {
      RCLCPP_WARN(
        get_logger(),
        " * Empty filename and empty 'pos_tracking.area_file_path' parameter.");
      res->message = "Empty filename and empty 'pos_tracking.area_file_path' parameter.";
      res->success = false;
      return;
    }
    filename = mAreaMemoryFilePath;
  }

  filename = sl_tools::getFullFilePath(filename);

  if (!saveAreaMemoryFile(filename) ) {
    res->message = "Error saving Area Memory File. Read node log for more information.";
    res->success = false;
    return;
  }

  res->message = "Area Memory File saved";
  res->success = true;
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
  const std::shared_ptr<zed_msgs::srv::SetPose_Request> req,
  std::shared_ptr<zed_msgs::srv::SetPose_Response> res)
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
  const std::shared_ptr<zed_msgs::srv::StartSvoRec_Request> req,
  std::shared_ptr<zed_msgs::srv::StartSvoRec_Response> res)
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

  if (req->compression_mode < 0 || req->compression_mode > 5) {
    RCLCPP_WARN(
      get_logger(),
      "'compression_mode' mode not valid. Please use a value in "
      "range [0,5]");
    res->message =
      "'compression_mode' mode not valid. Please use a value in range "
      "[0,5]";
    res->success = false;
    return;
  }
  switch (req->compression_mode) {
    case 1:
      mSvoRecCompression = sl::SVO_COMPRESSION_MODE::H264;
      break;
    case 3:
      mSvoRecCompression = sl::SVO_COMPRESSION_MODE::H264_LOSSLESS;
      break;
    case 4:
      mSvoRecCompression = sl::SVO_COMPRESSION_MODE::H265_LOSSLESS;
      break;
    case 5:
      mSvoRecCompression = sl::SVO_COMPRESSION_MODE::LOSSLESS;
      break;
    default:
      mSvoRecCompression = sl::SVO_COMPRESSION_MODE::H265;
      break;
  }
  mSvoRecFramerate = req->target_framerate;
  mSvoRecTranscode = req->input_transcode;
  mSvoRecFilename = req->svo_filename;

  if (mSvoRecFilename.empty()) {
    mSvoRecFilename = "zed.svo2";
  }

  std::string err;

  if (!startSvoRecording(err)) {
    res->message = "Error starting SVO recording: " + err;
    res->success = false;
    return;
  }

  RCLCPP_INFO(get_logger(), "SVO Recording started: ");
  RCLCPP_INFO_STREAM(get_logger(), " * Bitrate: " << mSvoRecBitrate);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Compression mode: " << sl::toString(
      mSvoRecCompression).c_str());
  RCLCPP_INFO_STREAM(get_logger(), " * Framerate: " << mSvoRecFramerate);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Input Transcode: " << (mSvoRecTranscode ? "TRUE" : "FALSE"));
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Filename: " << (mSvoRecFilename.empty() ? "zed.svo2" :
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

  RCLCPP_INFO(get_logger(), "SVO Recording stopped");
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
    res->message = "The node is not using an SVO as input";
    res->success = false;
    return;
  }

#ifndef USE_SVO_REALTIME_PAUSE
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
#endif

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

#ifdef USE_SVO_REALTIME_PAUSE
  mZed->pauseSVOReading(mSvoPause);
#endif

}

void ZedCamera::callback_setSvoFrame(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<zed_msgs::srv::SetSvoFrame_Request> req,
  std::shared_ptr<zed_msgs::srv::SetSvoFrame_Response> res)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "** Set SVO Frame service called **");

  // ----> Check service call frequency
  if (mSetSvoFrameCheckTimer.toc() < 0.5) {
    RCLCPP_WARN(get_logger(), "SVO frame set too fast");
    res->message = "SVO frame set too fast";
    res->success = false;
    return;
  }
  mSetSvoFrameCheckTimer.tic();
  // <---- Check service call frequency

  std::lock_guard<std::mutex> lock(mRecMutex);

  if (!mSvoMode) {
    RCLCPP_WARN(get_logger(), "The node is not using an SVO as input");
    res->message = "The node is not using an SVO as input";
    res->success = false;
    return;
  }

  int frame = req->frame_id;
  int svo_frames = mZed->getSVONumberOfFrames();
  if (frame >= svo_frames) {
    std::stringstream ss;
    ss << "Frame number is out of range. SVO has " << svo_frames << " frames";
    RCLCPP_WARN(get_logger(), ss.str().c_str());
    res->message = ss.str();
    res->success = false;
    return;
  }

  mZed->setSVOPosition(frame);
  RCLCPP_INFO_STREAM(get_logger(), "SVO frame set to " << frame);
  res->message = "SVO frame set to " + std::to_string(frame);

  // ----> Set camera pose to identity
  RCLCPP_WARN(get_logger(), " * Camera pose reset to identity.");
  mInitialBasePose[0] = 0.0;
  mInitialBasePose[1] = 0.0;
  mInitialBasePose[2] = 0.0;

  mInitialBasePose[3] = 0.0;
  mInitialBasePose[4] = 0.0;
  mInitialBasePose[5] = 0.0;

  mResetOdomFromSrv = true;
  mOdomPath.clear();
  mPosePath.clear();

  // Restart tracking
  startPosTracking();
  // <---- Set camera pose to identity

  //if svo is paused, ensure one grab can update topics
  if (mSvoPause) {
    mGrabOnce = true;
    mGrabImuOnce = true;
  }

  res->success = true;
}

void ZedCamera::callback_updateDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  DEBUG_COMM("=== Update Diagnostic ===");

  std::lock_guard<std::mutex> lock(mCloseCameraMutex);

  if (mZed == nullptr) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Camera not opened");
    return;
  }

  if (mConnStatus != sl::ERROR_CODE::SUCCESS) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      sl::toString(mConnStatus).c_str());
    return;
  }

  stat.addf("Uptime", "%s", sl_tools::seconds2str(mUptimer.toc()).c_str());

  if (mGrabStatus == sl::ERROR_CODE::SUCCESS || mGrabStatus == sl::ERROR_CODE::CORRUPTED_FRAME) {
    stat.addf("Camera Grab rate", "%d Hz", mCamGrabFrameRate);

    double freq = 1. / mGrabPeriodMean_sec->getAvg();
    double freq_perc = 100. * freq / mCamGrabFrameRate;
    stat.addf("Data Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);

    double frame_proc_sec = mElabPeriodMean_sec->getAvg();
    double frame_grab_period = 1. / mCamGrabFrameRate;
    stat.addf(
      "Data Capture", "Tot. Processing Time: %.6f sec (Max. %.3f sec)",
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

    // ----> Frame drop count
    auto dropped = mZed->getFrameDroppedCount();
    uint64_t total = dropped + mFrameCount;
    auto perc_drop = 100. * static_cast<double>(dropped) / total;
    stat.addf(
      "Frame Drop rate", "%u/%lu (%g%%)",
      dropped, total, perc_drop);
    // <---- Frame drop count

    if (mSimMode) {
      stat.add("Input mode", "SIMULATION");
    } else if (mSvoMode) {
      stat.add("Input mode", "SVO");
    } else if (mStreamMode) {
      stat.add("Input mode", "LOCAL STREAM");
    } else {
      stat.add("Input mode", "Live Camera");
    }

    if (mVdPublishing) {
      if (mSvoMode && !mSvoRealtime) {
        freq = 1. / mGrabPeriodMean_sec->getAvg();
        freq_perc = 100. * freq / mVdPubRate;
        stat.addf(
          "Video/Depth", "Mean Frequency: %.1f Hz (%.1f%%)", freq,
          freq_perc);
      } else {
        freq = 1. / mVideoDepthPeriodMean_sec->getAvg();
        freq_perc = 100. * freq / mVdPubRate;
        frame_grab_period = 1. / mVdPubRate;
        stat.addf(
          "Video/Depth", "Mean Frequency: %.1f Hz (%.1f%%)", freq,
          freq_perc);
      }
      stat.addf(
        "Video/Depth", "Processing Time: %.6f sec (Max. %.3f sec)",
        mVideoDepthElabMean_sec->getAvg(), frame_grab_period);
    } else {
      stat.add("Video/Depth", "Topic not subscribed");
    }

    if (mSvoMode) {
      double svo_perc = 100. * (static_cast<double>(mSvoFrameId) / mSvoFrameCount);

      stat.addf(
        "Playing SVO", "%sFrame: %d/%d (%.1f%%)",
        (mSvoPause ? "PAUSED - " : ""), mSvoFrameId, mSvoFrameCount, svo_perc);
      stat.addf("SVO Loop", "%s", (mSvoLoop ? "ON" : "OFF"));
      if (mSvoLoop) {
        stat.addf("SVO Loop Count", "%d", mSvoLoopCount);
      }
      stat.addf("Real Time mode", "%s", (mSvoRealtime ? "ON" : "OFF"));
      if (!mSvoRealtime) {
        stat.addf("SVO Playback rate", "%.1fx -> %.1f Hz", mSvoRate, mSvoRate * mCamGrabFrameRate);
      }
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
          freq_perc = 100. * freq / mVdPubRate;
          frame_grab_period = 1. / mVdPubRate;
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
          freq_perc = 100. * freq / mVdPubRate;
          frame_grab_period = 1. / mVdPubRate;
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

    if (mGrabStatus == sl::ERROR_CODE::CORRUPTED_FRAME) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Performance Degraded - Corrupted frame received");
    }
  } else if (mGrabStatus == sl::ERROR_CODE::LAST) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Camera initializing");
  } else {
    stat.summaryf(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "%s", sl::toString(mGrabStatus).c_str());
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

  RCLCPP_INFO_ONCE(get_logger(), "=== GNSS subscriber ===");
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
  size_t markerSubCount = 0;
  size_t planeSubCount = 0;
  try {
    markerSubCount = count_subscribers(mPubMarker->get_topic_name());
    planeSubCount = count_subscribers(mPubPlane->get_topic_name());
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_MAP(
      "callback_clickedPoint: Exception while counting point plane "
      "subscribers");
    return;
  }

  if ((markerSubCount + planeSubCount) == 0) {
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

  if (markerSubCount > 0) {
    // ----> Publish a blue sphere in the clicked point
    auto pt_marker = std::make_unique<visualization_msgs::msg::Marker>();
    // Set the frame ID and timestamp.  See the TF tutorials for information
    // on these.
    static int hit_pt_id =
      0;      // This ID must be unique in the same process. Thus it is good to
              // keep it as a static variable
    pt_marker->header.stamp = mUsePubTimestamps ? get_clock()->now() : ts;
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
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
    // ----> Publish a blue sphere in the clicked point

    // ----> Publish the plane as green mesh
    auto plane_marker = std::make_unique<visualization_msgs::msg::Marker>();
    // Set the frame ID and timestamp.  See the TF tutorials for information
    // on these.
    static int plane_mesh_id =
      0;      // This ID must be unique in the same process. Thus it is good to
              // keep it as a static variable
    plane_marker->header.stamp = mUsePubTimestamps ? get_clock()->now() : ts;
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
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
    // <---- Publish the plane as green mesh
  }

  if (planeSubCount > 0) {
    // ----> Publish the plane as custom message

    auto planeMsg = std::make_unique<zed_msgs::msg::PlaneStamped>();
    planeMsg->header.stamp = mUsePubTimestamps ? get_clock()->now() : ts;
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
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
    // <---- Publish the plane as custom message
  }
}

void ZedCamera::callback_setRoi(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<zed_msgs::srv::SetROI_Request> req,
  std::shared_ptr<zed_msgs::srv::SetROI_Response> res)
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


      if (_nitrosDisabled) {
        if (mPubRoiMask.getTopic().empty()) {
          mPubRoiMask = image_transport::create_publisher(
            this, mRoiMaskTopic, mQos.get_rmw_qos_profile());
          RCLCPP_INFO_STREAM(
            get_logger(), "Advertised on topic: "
              << mPubRoiMask.getTopic());
        }
      } else {
#ifdef FOUND_ISAAC_ROS_NITROS
        if (!mNitrosPubRoiMask) {
          mNitrosPubRoiMask = std::make_shared<
            nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
              nvidia::isaac_ros::nitros::NitrosImage>>(
            this, mRoiMaskTopic,
            nvidia::isaac_ros::nitros::nitros_image_bgra8_t::
            supported_type_name,
            nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(), mQos);
          RCLCPP_INFO_STREAM(
            get_logger(),
            "Advertised on topic: " << mRoiMaskTopic);
          RCLCPP_INFO_STREAM(
            get_logger(), "Advertised on topic: "
              << mRoiMaskTopic + "/nitros");
        }
#endif
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
  DEBUG_SIM("=== CLOCK CALLBACK ===");
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
      if (_nitrosDisabled) {
        subCount = mPubRoiMask.getNumSubscribers();
      } else {
#ifdef FOUND_ISAAC_ROS_NITROS
        subCount = count_subscribers(mRoiMaskTopic);
#endif
      }
    } catch (...) {
      rcutils_reset_error();
      DEBUG_STREAM_COMM("processRtRoi: Exception while counting subscribers");
      return;
    }

    if (subCount > 0) {
      DEBUG_ROI("Retrieve ROI Mask");
      sl::Mat roi_mask;
      mZed->getRegionOfInterest(roi_mask);

      DEBUG_ROI("Publish ROI Mask");

      if (_nitrosDisabled) {
        publishImageWithInfo(
          roi_mask, mPubRoiMask, mPubRoiMaskCamInfo, mPubRoiMaskCamInfoTrans, mLeftCamInfoMsg,
          mLeftCamOptFrameId, ts);
      } else {
#ifdef FOUND_ISAAC_ROS_NITROS
        publishImageWithInfo(
          roi_mask, mNitrosPubRoiMask, mPubRoiMaskCamInfo, mPubRoiMaskCamInfoTrans, mLeftCamInfoMsg,
          mLeftCamOptFrameId, ts);
#endif
      }
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

void ZedCamera::publishHealthStatus()
{
  if (mImageValidityCheck <= 0) {
    return;
  }

  size_t sub_count = 0;
  try {
    sub_count = mPubHealthStatus->get_subscription_count();
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_COMM("publishHealthStatus: Exception while counting subscribers");
    return;
  }

  if (sub_count == 0) {
    return;
  }

  sl::HealthStatus status = mZed->getHealthStatus();
  auto msg = std::make_unique<zed_msgs::msg::HealthStatusStamped>();
  msg->header.stamp = mUsePubTimestamps ?
    get_clock()->now() :
    mFrameTimestamp;
  msg->header.frame_id = mBaseFrameId;
  msg->serial_number = mCamSerialNumber;
  msg->camera_name = mCameraName;
  msg->low_image_quality = status.low_image_quality;
  msg->low_lighting = status.low_lighting;
  msg->low_depth_reliability = status.low_depth_reliability;
  msg->low_motion_sensors_reliability =
    status.low_motion_sensors_reliability;

  mPubHealthStatus->publish(std::move(msg));

}

bool ZedCamera::publishSvoStatus(uint64_t frame_ts)
{
  if (!mSvoMode) {
    return false;
  }

  size_t subCount = 0;
  try {
    subCount = mPubSvoStatus->get_subscription_count();
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_COMM("publishSvoStatus: Exception while counting subscribers");
    return false;
  }

  if (subCount > 0) {
    auto msg = std::make_unique<zed_msgs::msg::SvoStatus>();

    // ----> Fill the status message
    msg->file_name = mSvoFilepath;
    msg->frame_id = mZed->getSVOPosition();
    msg->total_frames = mZed->getSVONumberOfFrames();
    msg->frame_ts = frame_ts;

    if (mSvoPause) {
      msg->status = zed_msgs::msg::SvoStatus::STATUS_PAUSED;
    } else if (mGrabStatus == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
      msg->frame_id = msg->total_frames - 1;
      msg->status = zed_msgs::msg::SvoStatus::STATUS_END;
    } else {
      msg->status = zed_msgs::msg::SvoStatus::STATUS_PLAYING;
    }

    msg->loop_active = mSvoLoop;
    msg->loop_count = mSvoLoopCount;
    msg->real_time_mode = mSvoRealtime;
    // <---- Fill the status message

    // Publish the message
    mPubSvoStatus->publish(std::move(msg));
    return true;
  }
  return false;
}

void ZedCamera::callback_pubHeartbeat()
{
  if (mThreadStop) {
    return;
  }

  // ----> Count the subscribers
  size_t sub_count = 0;
  try {
    sub_count = mPubHeartbeatStatus->get_subscription_count();
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_COMM("publishHeartbeat: Exception while counting subscribers");
    return;
  }

  if (sub_count == 0) {
    return;
  }
  // <---- Count the subscribers

  // ----> Fill the message
  auto msg = std::make_unique<zed_msgs::msg::Heartbeat>();
  msg->beat_count = ++mHeartbeatCount;
  msg->camera_sn = mCamSerialNumber;
  msg->full_name = this->get_fully_qualified_name();
  msg->node_name = this->get_name();
  msg->node_ns = this->get_namespace();
  msg->simul_mode = mSimMode;
  msg->svo_mode = mSvoMode;
  // <---- Fill the message

  // Publish the hearbeat
  mPubHeartbeatStatus->publish(std::move(msg));
}

void ZedCamera::publishClock(const sl::Timestamp & ts)
{
  DEBUG_COMM("Publishing clock");

  size_t subCount = 0;
  try {
    subCount = mPubClock->get_subscription_count();
  } catch (...) {
    rcutils_reset_error();
    DEBUG_COMM("publishClock: Exception while counting subscribers");
    return;
  }

  if (subCount == 0) {
    return;
  }

  auto msg = std::make_unique<rosgraph_msgs::msg::Clock>();
  msg->clock = sl_tools::slTime2Ros(ts);

  mPubClock->publish(std::move(msg));
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCamera)
