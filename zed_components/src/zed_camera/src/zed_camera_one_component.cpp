// Copyright 2024 Stereolabs
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

#include "zed_camera_one_component.hpp"

#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

#include "sl_logging.hpp"

using namespace std::placeholders;
namespace stereolabs
{
ZedCameraOne::ZedCameraOne(const rclcpp::NodeOptions & options)
: Node("zed_node_one", options),
  _threadStop(false),
  _qos(QOS_QUEUE_SIZE),
  _diagUpdater(this)
{
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), "    ZED Camera One Component ");
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "********************************");

  const size_t SDK_MAJOR_REQ = 4;
  const size_t SDK_MINOR_REQ = 2;

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
  _pubOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  _subOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
#endif
  // <---- Publishers/Sbscribers options

  // ----> Start a "one shot timer" to initialize the node and make
  // `shared_from_this` available
  std::chrono::milliseconds init_msec(static_cast<int>(50.0));
  _initTimer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(init_msec),
    std::bind(&ZedCameraOne::init, this));
  // <---- Start a "one shot timer" to initialize the node and make
  // `shared_from_this` available
}

ZedCameraOne::~ZedCameraOne()
{
  DEBUG_STREAM_COMM("Destroying node");

  // ----> Stop subscribers
  // <---- Stop subscribers

  DEBUG_STREAM_SENS("Stopping temperatures timer");
  if (_tempPubTimer) {
    _tempPubTimer->cancel();
  }

  // ----> Verify that all the threads are not active
  DEBUG_STREAM_COMM("Stopping running threads");
  if (!_threadStop) {
    _threadStop = true;
  }

  DEBUG_STREAM_COMM("Waiting for grab thread...");
  try {
    if (_grabThread.joinable()) {
      _grabThread.join();
    }
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Grab thread joining exception: " << e.what());
  }
  DEBUG_STREAM_COMM("... grab thread stopped");

  DEBUG_STREAM_SENS("Waiting for sensors thread...");
  try {
    if (_sensThread.joinable()) {
      _sensThread.join();
    }
  } catch (std::system_error & e) {
    DEBUG_STREAM_SENS("Sensors thread joining exception: " << e.what());
  }
  DEBUG_STREAM_SENS("... sensors thread stopped");

  DEBUG_STREAM_VD("Waiting for RGB thread...");
  try {
    if (_videoThread.joinable()) {
      _videoThread.join();
    }
  } catch (std::system_error & e) {
    DEBUG_STREAM_VD("RGB thread joining exception: " << e.what());
  }
  DEBUG_STREAM_VD("... RGB thread stopped");

  // <---- Verify that all the threads are not active
}

void ZedCameraOne::initParameters()
{
  // DEBUG parameters
  getDebugParams();
}

void ZedCameraOne::getDebugParams()
{
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "*** DEBUG parameters ***");

  getParam("debug.sdk_verbose", _sdkVerbose, _sdkVerbose, " * SDK Verbose: ");

  getParam("debug.debug_common", _debugCommon, _debugCommon);
  RCLCPP_INFO(
    get_logger(), " * Debug Common: %s",
    _debugCommon ? "TRUE" : "FALSE");

  getParam("debug.debug_video_depth", _debugVideoDepth, _debugVideoDepth);
  RCLCPP_INFO(
    get_logger(), " * Debug Video/Depth: %s",
    _debugVideoDepth ? "TRUE" : "FALSE");

  getParam("debug.debug_camera_controls", _debugCamCtrl, _debugCamCtrl);
  RCLCPP_INFO(
    get_logger(), " * Debug Control settings: %s",
    _debugCamCtrl ? "TRUE" : "FALSE");

  getParam("debug.debug_sensors", _debugSensors, _debugSensors);
  RCLCPP_INFO(
    get_logger(), " * Debug sensors: %s",
    _debugSensors ? "TRUE" : "FALSE");

  getParam("debug.debug_streaming", _debugStreaming, _debugStreaming);
  RCLCPP_INFO(
    get_logger(), " * Debug Streaming: %s",
    _debugStreaming ? "TRUE" : "FALSE");

  _debugMode = _debugCommon || _debugVideoDepth || _debugCamCtrl ||
    _debugSensors || _debugStreaming;

  if (_debugMode) {
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

void ZedCameraOne::init()
{
  // Stop the timer for "one shot" initialization
  _initTimer->cancel();

  // ----> Diagnostic initialization
  _diagUpdater.add(
    "ZED Diagnostic", this,
    &ZedCameraOne::callback_updateDiagnostic);
  std::string hw_id = std::string("Stereolabs camera: ") + _cameraName;
  _diagUpdater.setHardwareID(hw_id);
  // <---- Diagnostic initialization

  // Parameters initialization
  initParameters();

  // Services initialization
  initServices();

  // ----> Start camera
  if (!startCamera()) {
    exit(EXIT_FAILURE);
  }
  // <---- Start camera

  // Dynamic parameters callback
  _paramChangeCallbackHandle = add_on_set_parameters_callback(
    std::bind(&ZedCameraOne::callback_paramChange, this, _1));
}

void ZedCameraOne::initServices()
{
  RCLCPP_INFO(get_logger(), "*** SERVICES ***");

  std::string srv_name;
  std::string srv_prefix = "~/";
}

bool ZedCameraOne::startCamera()
{
  RCLCPP_INFO(get_logger(), "***** STARTING CAMERA *****");

  // Create a ZED object
  _zed = std::make_shared<sl::CameraOne>();

  // ----> SDK version
  RCLCPP_INFO(
    get_logger(), "ZED SDK Version: %d.%d.%d - Build %s",
    ZED_SDK_MAJOR_VERSION, ZED_SDK_MINOR_VERSION,
    ZED_SDK_PATCH_VERSION, ZED_SDK_BUILD_ID);
  // <---- SDK version

  // ----> ZED configuration
  if (!_svoFilepath.empty()) {
    RCLCPP_INFO(get_logger(), "*** SVO OPENING ***");

    _initParams.input.setFromSVOFile(_svoFilepath.c_str());
    _initParams.svo_real_time_mode = _svoRealtime;

    _svoMode = true;
  } else if (!_streamAddr.empty()) {
    RCLCPP_INFO(get_logger(), "*** LOCAL STREAMING OPENING ***");

    _initParams.input.setFromStream(
      _streamAddr.c_str(),
      static_cast<unsigned short>(_streamPort));

    _streamMode = true;
  } else {
    RCLCPP_INFO(get_logger(), "*** CAMERA OPENING ***");

    _initParams.camera_fps = _camGrabFrameRate;
    _initParams.camera_resolution = static_cast<sl::RESOLUTION>(_camResol);

    if (_camSerialNumber > 0) {
      _initParams.input.setFromSerialNumber(_camSerialNumber);
    }
  }

  _initParams.async_grab_camera_recovery =
    true;    // Camera recovery is handled asynchronously to provide
             // information about this status
  _initParams.camera_image_flip =
    (_cameraFlip ? sl::FLIP_MODE::ON : sl::FLIP_MODE::OFF);
  _initParams.coordinate_system = ROS_COORDINATE_SYSTEM;
  _initParams.coordinate_units = ROS_MEAS_UNITS;
  _initParams.enable_hdr = _enableHDR;
  _initParams.open_timeout_sec = _openTimeout_sec;
  if (!_opencvCalibFile.empty()) {
    _initParams.optional_opencv_calibration_file = _opencvCalibFile.c_str();
  }
  _initParams.sdk_verbose = _sdkVerbose;
  // <---- ZED configuration

  // ----> Try to connect to a camera, to a stream, or to load an SVO
  _grabStatus = sl::ERROR_CODE::LAST;

  _connStatus = _zed->open(_initParams);

  if (_connStatus != sl::ERROR_CODE::SUCCESS) {
    if (_connStatus == sl::ERROR_CODE::INVALID_CALIBRATION_FILE) {
      if (_opencvCalibFile.empty()) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Calibration file error: "
            << sl::toVerbose(_connStatus));
      } else {
        RCLCPP_ERROR_STREAM(
          get_logger(),
          "If you are using a custom OpenCV calibration file, please check "
          "the correctness of the path of the calibration file "
          "in the parameter 'general.optional_opencv_calibration_file': '"
            << _opencvCalibFile << "'.");
        RCLCPP_ERROR(
          get_logger(),
          "If the file exists, it may contain invalid information.");
      }
      return false;
    } else if (_svoMode) {
      RCLCPP_WARN(
        get_logger(), "Error opening SVO: %s",
        sl::toString(_connStatus).c_str());
      return false;
    } else if (_streamMode) {
      RCLCPP_WARN(
        get_logger(), "Error opening Local Stream: %s",
        sl::toString(_connStatus).c_str());
      return false;
    } else {
      RCLCPP_WARN(
        get_logger(), "Error opening camera: %s",
        sl::toString(_connStatus).c_str());
      RCLCPP_INFO(get_logger(), "Please verify the camera connection");
    }
  } else {
    DEBUG_STREAM_COMM("Opening successfull");
  }
  // ----> Try to connect to a camera, to a stream, or to load an SVO

  return true;
}

void ZedCameraOne::callback_updateDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  DEBUG_COMM("*** Update Diagnostic ***");

  if (_connStatus == sl::ERROR_CODE::LAST) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Initializing...");
    return;
  } else if (_connStatus != sl::ERROR_CODE::SUCCESS) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      sl::toString(_connStatus).c_str());
    return;
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

    if (_debugMode) {
      DEBUG_STREAM_COMM("Param #" << count << ": " << param.get_name());
    }
  }
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCameraOne)
