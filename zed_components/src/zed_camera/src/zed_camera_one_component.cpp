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
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "sl_logging.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
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
    _zed.reset();
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
      return false;
    }
  } else {
    DEBUG_STREAM_COMM("Opening successfull");
  }
  // ----> Try to connect to a camera, to a stream, or to load an SVO

  // ----> Camera information
  sl::CameraOneInformation camInfo = _zed->getCameraInformation();

  float realFps = camInfo.camera_configuration.fps;
  if (realFps != static_cast<float>(_camGrabFrameRate)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "!!! `general.grab_frame_rate` value is not valid: '"
        << _camGrabFrameRate
        << "'. Automatically replaced with '" << realFps
        << "'. Please fix the parameter !!!");
    _camGrabFrameRate = realFps;
  }

  // CUdevice devid;
  cuCtxGetDevice(&_gpuId);
  RCLCPP_INFO_STREAM(get_logger(), "ZED SDK running on GPU #" << _gpuId);

  // Camera model
  _camRealModel = camInfo.camera_model;

  if (_camRealModel != _camUserModel) {
    RCLCPP_WARN( get_logger(), "Camera model does not match user parameter.");

    if(_camRealModel==sl::MODEL::ZED_XONE_GS)
    {
      RCLCPP_WARN( get_logger(), "Please set the parameter 'general.camera_model' to 'zedxonegs'");
    } else if(_camRealModel==sl::MODEL::ZED_XONE_UHD)
    {
      RCLCPP_WARN( get_logger(), "Please set the parameter 'general.camera_model' to 'zedxone4k'");
    }
  }

  RCLCPP_INFO_STREAM(
    get_logger(), " * Camera Model  -> "
      << sl::toString(_camRealModel).c_str());
  _camSerialNumber = camInfo.serial_number;
  RCLCPP_INFO_STREAM(get_logger(), " * Serial Number -> " << _camSerialNumber);

  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Focal Lenght -> "
      << camInfo.camera_configuration.calibration_parameters.focal_length_metric
      << " mm");

  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Input\t -> "
      << sl::toString(camInfo.input_type).c_str());
  if (_svoMode) {
    RCLCPP_INFO(
      get_logger(), " * SVO resolution\t-> %ldx%ld",
      camInfo.camera_configuration.resolution.width,
      camInfo.camera_configuration.resolution.height);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * SVO framerate\t-> "
        << (camInfo.camera_configuration.fps));
  }

  // Firmwares
  if (!_svoMode) {
    _camFwVersion = camInfo.camera_configuration.firmware_version;

    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Camera FW Version  -> " << _camFwVersion);
      _sensFwVersion = camInfo.sensors_configuration.firmware_version;
      RCLCPP_INFO_STREAM(
        get_logger(),
        " * Sensors FW Version -> " << _sensFwVersion);
  }

  // Camera/IMU transform  
  _slCamImuTransf = camInfo.sensors_configuration.camera_imu_transform;
  DEBUG_SENS("Camera-IMU Transform:\n%s", _slCamImuTransf.getInfos().c_str());
  

  _camWidth = camInfo.camera_configuration.resolution.width;
  _camHeight = camInfo.camera_configuration.resolution.height;

  RCLCPP_INFO_STREAM(
    get_logger(), " * Camera grab frame size -> "
      << _camWidth << "x" << _camHeight);

  int pub_w, pub_h;
  pub_w = static_cast<int>(std::round(_camWidth / _customDownscaleFactor));
  pub_h = static_cast<int>(std::round(_camHeight /_customDownscaleFactor));

  if (pub_w > _camWidth || pub_h > _camHeight) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The publishing resolution ("
        << pub_w << "x" << pub_h
        << ") cannot be higher than the grabbing resolution ("
        << _camWidth << "x" << _camHeight
        << "). Using grab resolution for output messages.");
    pub_w = _camWidth;
    pub_h = _camHeight;
  }

  _matResol = sl::Resolution(pub_w, pub_h);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Publishing frame size  -> "
      << _matResol.width << "x"
      << _matResol.height);
  // <---- Camera information

  // ----> Camera Info messages
  _camInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  _camInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();

  initTFCoordFrameNames();

  // Rectified image
  fillCamInfo( _camInfoMsg, _camImgFrameId, false);
  // Raw image
  fillCamInfo( _camInfoRawMsg, _camImgFrameId, true);
  // <---- Camera Info messages

  // Initialize publishers
  initPublishers();

  // Initialialized timestamp to avoid wrong initial data
  // ----> Timestamp
  if (_svoMode) {
    _frameTimestamp =
      sl_tools::slTime2Ros(_zed->getTimestamp(sl::TIME_REFERENCE::CURRENT));
  } else {
    _frameTimestamp =
      sl_tools::slTime2Ros(_zed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
  }
  // <---- Timestamp

  // TODO Initialize diagnostic statistics

  // Init and start threads
  initThreadsAndTimers();

  return true;
}

void ZedCameraOne::initThreadsAndTimers() 
{
  // ----> Start CMOS Temperatures thread
  startTempPubTimer();
  // <---- Start CMOS Temperatures thread
}

void ZedCameraOne::startTempPubTimer()
{
  if (_tempPubTimer != nullptr) {
    _tempPubTimer->cancel();
  }

  std::chrono::milliseconds pubPeriod_msec(static_cast<int>(1000.0));
  _tempPubTimer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(pubPeriod_msec),
    std::bind(&ZedCameraOne::callback_pubTemp, this));
}

void ZedCameraOne::callback_pubTemp()
{
  DEBUG_STREAM_ONCE_SENS("Temperatures callback called");

  if (_grabStatus != sl::ERROR_CODE::SUCCESS) {
    DEBUG_SENS("Camera not ready");
    rclcpp::sleep_for(1s);
    return;
  }

  // ----> Always update temperature values for diagnostic
  sl::SensorsData sens_data;
  sl::ERROR_CODE err = _zed->getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT);
  if (err != sl::ERROR_CODE::SUCCESS) {
    DEBUG_STREAM_SENS(
      "[callback_pubTemp] sl::getSensorsData error: "
        << sl::toString(err).c_str());
    return;
  }

  sens_data.temperature.get(
    sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU, _tempImu);
  // <---- Always update temperature values for diagnostic

  // ----> Subscribers count
  size_t tempSubNumber;

  try {
    tempSubNumber = count_subscribers(_pubTemp->get_topic_name());
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_SENS(
      "callback_pubTemp: Exception while counting subscribers");
    return;
  }
  // <---- Subscribers count

  // ----> Publish temperature
  if (tempSubNumber > 0) {
    tempMsgPtr imuTempMsg = std::make_unique<sensor_msgs::msg::Temperature>();

    imuTempMsg->header.stamp = get_clock()->now();

    imuTempMsg->header.frame_id = _imuFrameId;
    imuTempMsg->temperature = static_cast<double>(_tempImu);
    imuTempMsg->variance = 0.0;

    DEBUG_SENS("Publishing IMU TEMP message");
    try {
      _pubTemp->publish(std::move(imuTempMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
  // <---- Publish temperature
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

rcl_interfaces::msg::SetParametersResult ZedCameraOne::callback_paramChange(
  std::vector<rclcpp::Parameter> parameters)
{
  DEBUG_STREAM_COMM("Parameter change callback");

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Modifying " << parameters.size() << " parameters");

  int count = 0;

  for (const rclcpp::Parameter & param : parameters) {
    count++;

    DEBUG_STREAM_COMM("Param #" << count << ": " << param.get_name());
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

void ZedCameraOne::initTFCoordFrameNames()
{
  // ----> Coordinate frames
  _cameraLinkFrameId = _cameraName + "_camera_link";
  _cameraCenterFrameId = _cameraName + "_camera_center";
  _camImgFrameId = _cameraName + "_camera_optical_frame";
  _camOptFrameId = _cameraName + "_camera_optical_frame";
  _imuFrameId = _cameraName + "_imu_link";

  // Print TF frames
  RCLCPP_INFO_STREAM(get_logger(), "*** TF FRAMES ***");
  RCLCPP_INFO_STREAM(get_logger(), " * Camera link\t-> " << _cameraLinkFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Camera center\t-> " << _cameraCenterFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Image\t\t-> " << _camImgFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Optical\t-> " << _camOptFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * IMU\t\t-> " << _imuFrameId);
  // <---- Coordinate frames
}

void ZedCameraOne::fillCamInfo(
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> & camInfoMsg,
    const std::string & frameId,
    bool rawParam)
{
  sl::CameraParameters zedParam;

  if (rawParam) {
    zedParam = _zed->getCameraInformation(_matResol).camera_configuration.calibration_parameters_raw;
  } else {
    zedParam = _zed->getCameraInformation(_matResol).camera_configuration.calibration_parameters;
  }

  // https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html

  // ----> Distortion models
  // ZED SDK params order: [ k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]
  // Radial (k1, k2, k3, k4, k5, k6), Tangential (p1,p2) and Prism (s1, s2, s3,
  // s4) distortion. Prism not currently used.

  // ROS2 order (OpenCV) -> k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
  switch (_camRealModel) {
    case sl::MODEL::ZED_XONE_GS: // RATIONAL_POLYNOMIAL

      camInfoMsg->distortion_model =
        sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

      camInfoMsg->d.resize(8);
      camInfoMsg->d[0] = zedParam.disto[0];    // k1
      camInfoMsg->d[1] = zedParam.disto[1];    // k2
      camInfoMsg->d[2] = zedParam.disto[2];    // p1
      camInfoMsg->d[3] = zedParam.disto[3];    // p2
      camInfoMsg->d[4] = zedParam.disto[4];    // k3
      camInfoMsg->d[5] = zedParam.disto[5];    // k4
      camInfoMsg->d[6] = zedParam.disto[6];    // k5
      camInfoMsg->d[7] = zedParam.disto[7];    // k6
      break;

    case sl::MODEL::ZED_XONE_UHD: // RATIONAL_POLYNOMIAL

      camInfoMsg->distortion_model =
        sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

      camInfoMsg->d.resize(8);
      camInfoMsg->d[0] = zedParam.disto[0];    // k1
      camInfoMsg->d[1] = zedParam.disto[1];    // k2
      camInfoMsg->d[2] = zedParam.disto[2];    // p1
      camInfoMsg->d[3] = zedParam.disto[3];    // p2
      camInfoMsg->d[4] = zedParam.disto[4];    // k3
      camInfoMsg->d[5] = zedParam.disto[5];    // k4
      camInfoMsg->d[6] = zedParam.disto[6];    // k5
      camInfoMsg->d[7] = zedParam.disto[7];    // k6
      break;
  }

  // Intrinsic
  camInfoMsg->k.fill(0.0);
  camInfoMsg->k[0] = static_cast<double>(zedParam.fx);
  camInfoMsg->k[2] = static_cast<double>(zedParam.cx);
  camInfoMsg->k[4] = static_cast<double>(zedParam.fy);
  camInfoMsg->k[5] = static_cast<double>(zedParam.cy);
  camInfoMsg->k[8] = 1.0;

  // Rectification
  camInfoMsg->r.fill(0.0);
  for (size_t i = 0; i < 3; i++) {
    // identity
    camInfoMsg->r[i + i * 3] = 1.0;
  }

  // Projection/camera matrix
  camInfoMsg->p.fill(0.0);
  camInfoMsg->p[0] = static_cast<double>(zedParam.fx);
  camInfoMsg->p[2] = static_cast<double>(zedParam.cx);
  camInfoMsg->p[5] = static_cast<double>(zedParam.fy);
  camInfoMsg->p[6] = static_cast<double>(zedParam.cy);
  camInfoMsg->p[10] = 1.0;
  
  // Image size
  camInfoMsg->width = static_cast<uint32_t>(_matResol.width);
  camInfoMsg->height = static_cast<uint32_t>(_matResol.height);
  camInfoMsg->header.frame_id = frameId;
}

void ZedCameraOne::initPublishers() 
{
  RCLCPP_INFO(get_logger(), "*** PUBLISHED TOPICS ***");

  // ----> Images
  RCLCPP_INFO(get_logger(), " +++ IMAGE TOPICS +++");
  std::string rect_prefix = "rect/";
  std::string raw_prefix = "raw/";
  std::string color_prefix = "rgb/";
  std::string gray_prefix = "gray/";
  std::string image_topic = "image";


  _imgTopic = _topicRoot + color_prefix + rect_prefix + image_topic;
  _pubColorImg = image_transport::create_camera_publisher(
    this, _imgTopic, _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger()," * Advertised on topic: " << _pubColorImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger()," * Advertised on topic: " << _pubColorImg.getInfoTopic());

  _imgRawTopic = _topicRoot + color_prefix + raw_prefix + image_topic;
  _pubColorRawImg = image_transport::create_camera_publisher(
    this, _imgRawTopic, _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger()," * Advertised on topic: " << _pubColorRawImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger()," * Advertised on topic: " << _pubColorRawImg.getInfoTopic());

  _imgGrayTopic = _topicRoot + gray_prefix + rect_prefix + image_topic;
  _pubGrayImg = image_transport::create_camera_publisher(
    this, _imgGrayTopic, _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger()," * Advertised on topic: " << _pubGrayImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger()," * Advertised on topic: " << _pubGrayImg.getInfoTopic());

  _imgRawGrayTopic = _topicRoot + gray_prefix + raw_prefix + image_topic;
  _pubGrayRawImg = image_transport::create_camera_publisher(
    this, _imgRawGrayTopic, _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger()," * Advertised on topic: " << _pubGrayRawImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger()," * Advertised on topic: " << _pubGrayRawImg.getInfoTopic());
  // <---- Images

  // ----> Temperature
  RCLCPP_INFO(get_logger(), " +++ SENSOR TOPICS +++");
  _tempTopic = _topicRoot + "temperature";
  _pubTemp = create_publisher<sensor_msgs::msg::Temperature>(_tempTopic, _qos, _pubOpt);
  RCLCPP_INFO_STREAM(get_logger()," * Advertised on topic: " << _pubTemp->get_topic_name());
  // <---- Temperature

  // TODO initialize IMU publishers
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCameraOne)
