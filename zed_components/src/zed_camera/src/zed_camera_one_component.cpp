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
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "sl_logging.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
namespace stereolabs
{
ZedCameraOne::ZedCameraOne(const rclcpp::NodeOptions & options)
: Node("zed_node_one", options)
  , _threadStop(false)
  , _qos(QOS_QUEUE_SIZE)
  , _diagUpdater(this)
  , _grabFreqTimer(get_clock())
  , _imuFreqTimer(get_clock())
  , _imuTfFreqTimer(get_clock())
  , _sensPubFreqTimer(get_clock())
  , _frameTimestamp(_frameTimestamp)
  , _lastTs_imu(_frameTimestamp)
{
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), "    ZED Camera One Component    ");
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

// ----> Publishers/Subscribers options
#ifndef FOUND_FOXY
  _pubOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  _subOpt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
#endif
  // <---- Publishers/Subscribers options

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

  // <---- Verify that all the threads are not active
}

void ZedCameraOne::initParameters()
{
  // DEBUG parameters
  getDebugParams();

  // GENERAL parameters
  getGeneralParams();

  // SENSORS parameters
  getSensorsParams();

  // STREAMING SERVER parameters
  getStreamingServerParams();

  // ADVANCED parameters
  getAdvancedParams();
}

void ZedCameraOne::getGeneralParams()
{
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "*** GENERAL parameters ***");

  getParam("svo.svo_path", std::string(), _svoFilepath);
  if (_svoFilepath.compare("live") == 0) {
    _svoFilepath = "";
  }

  if (_svoFilepath == "") {
    _svoMode = false;
  } else {
    RCLCPP_INFO_STREAM(get_logger(), " * SVO: '" << _svoFilepath.c_str() << "'");
    _svoMode = true;
    getParam("svo.svo_loop", _svoLoop, _svoLoop);
    RCLCPP_INFO(get_logger(), " * SVO Loop: %s", _svoLoop ? "TRUE" : "FALSE");
    getParam("svo.svo_realtime", _svoRealtime, _svoRealtime);
    RCLCPP_INFO(
      get_logger(), " * SVO Realtime: %s",
      _svoRealtime ? "TRUE" : "FALSE");
  }

  _streamMode = false;
  if (!_svoMode) {
    getParam("stream.stream_address", std::string(), _streamAddr);
    if (_streamAddr != "") {
      _streamMode = true;
      getParam("stream.stream_port", _streamPort, _streamPort);
      RCLCPP_INFO_STREAM(
        get_logger(), " * Local stream input: " << _streamAddr << ":" << _streamPort);
    }
  }

  std::string camera_model = "zed";
  getParam("general.camera_model", camera_model, camera_model);
  if (camera_model == "zedxonegs") {
    _camUserModel = sl::MODEL::ZED_XONE_GS;
    if (_svoMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Playing an SVO for "
          << sl::toString(_camUserModel)
          << " camera model.");
    } else if (_streamMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Playing a network stream from a "
          << sl::toString(_camUserModel)
          << " camera model.");
    } else if (!IS_JETSON) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Camera model " << sl::toString(_camUserModel).c_str()
                        << " is available only with NVIDIA Jetson devices.");
      exit(EXIT_FAILURE);
    }
  } else if (camera_model == "zedxone4k") {
    _camUserModel = sl::MODEL::ZED_XONE_UHD;
    if (_svoMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Playing an SVO for "
          << sl::toString(_camUserModel)
          << " camera model.");
    } else if (_streamMode) {
      RCLCPP_INFO_STREAM(
        get_logger(), " + Playing a network stream from a "
          << sl::toString(_camUserModel)
          << " camera model.");
    } else if (!IS_JETSON) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Camera model " << sl::toString(_camUserModel).c_str()
                        << " is available only with NVIDIA Jetson devices.");
      exit(EXIT_FAILURE);
    }
  } else {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Camera model not valid in parameter values: " << camera_model);
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Camera model: " << camera_model << " - " << _camUserModel);

  getParam("general.camera_name", _cameraName, _cameraName, " * Camera name: ");
  getParam(
    "general.serial_number", _camSerialNumber, _camSerialNumber,
    " * Camera SN: ");
  getParam(
    "general.camera_timeout_sec", _openTimeout_sec, _openTimeout_sec,
    " * Camera timeout [sec]: ");
  getParam(
    "general.grab_frame_rate", _camGrabFrameRate, _camGrabFrameRate,
    " * Camera framerate: ");
  getParam("general.gpu_id", _gpuId, _gpuId, " * GPU ID: ");

  // TODO(walter) ADD SVO SAVE COMPRESSION PARAMETERS


  std::string resol = "AUTO";
  getParam("general.grab_resolution", resol, resol);
  if (resol == "AUTO") {
    _camResol = sl::RESOLUTION::AUTO;
  } else if (resol == "HD1200") {
    _camResol = sl::RESOLUTION::HD1200;
  } else if (resol == "HD1080") {
    _camResol = sl::RESOLUTION::HD1080;
  } else if (resol == "SVGA") {
    _camResol = sl::RESOLUTION::SVGA;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Not valid 'general.grab_resolution' value: '%s'. Using "
      "'AUTO' setting.",
      resol.c_str());
    _camResol = sl::RESOLUTION::AUTO;
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Camera resolution: "
      << sl::toString(_camResol).c_str());


  std::string out_resol = "NATIVE";
  getParam("general.pub_resolution", out_resol, out_resol);
  if (out_resol == "NATIVE") {
    _pubResolution = PubRes::NATIVE;
  } else if (out_resol == "CUSTOM") {
    _pubResolution = PubRes::CUSTOM;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Not valid 'general.pub_resolution' value: '%s'. Using default "
      "setting instead.",
      out_resol.c_str());
    out_resol = "NATIVE";
    _pubResolution = PubRes::NATIVE;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Publishing resolution: " << out_resol.c_str());

  if (_pubResolution == PubRes::CUSTOM) {
    getParam(
      "general.pub_downscale_factor", _customDownscaleFactor,
      _customDownscaleFactor, " * Publishing downscale factor: ");
  } else {
    _customDownscaleFactor = 1.0;
  }

  getParam(
    "general.optional_opencv_calibration_file", _opencvCalibFile,
    _opencvCalibFile, " * OpenCV custom calibration: ");

  getParam("general.self_calib", _cameraSelfCalib, _cameraSelfCalib);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Camera self calibration: " << (_cameraSelfCalib ? "TRUE" : "FALSE"));
  getParam("general.camera_flip", _cameraFlip, _cameraFlip);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Camera flip: " << (_cameraFlip ? "TRUE" : "FALSE"));
}

void ZedCameraOne::getSensorsParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** SENSORS parameters ***");

  getParam("sensors.publish_imu_tf", _publishImuTF, _publishImuTF);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Broadcast IMU TF: "
      << (_publishImuTF ? "TRUE" : "FALSE"));

  getParam("sensors.sensors_pub_rate", _sensPubRate, _sensPubRate);
  if (_sensPubRate < _camGrabFrameRate) {
    _sensPubRate = _camGrabFrameRate;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Sensors publishing rate: " << _sensPubRate << " Hz");
}

void ZedCameraOne::getStreamingServerParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** Streaming Server parameters ***");

  bool stream_server = false;
  getParam("stream_server.stream_enabled", stream_server, stream_server);
  _streamingServerRequired = stream_server;
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Streaming Server enabled: "
      << (_streamingServerRequired ? "TRUE" : "FALSE"));

  std::string codec = "H264";
  getParam("stream_server.codec", codec, codec);
  if (codec == "H264") {
    _streamingServerCodec = sl::STREAMING_CODEC::H264;
    RCLCPP_INFO(get_logger(), " * Stream codec: H264");
  } else if (codec == "H265") {
    _streamingServerCodec = sl::STREAMING_CODEC::H265;
    RCLCPP_INFO(get_logger(), " * Stream codec: H265");
  } else {
    _streamingServerCodec = sl::STREAMING_CODEC::H264;
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.codec': " << codec <<
        ". Using the default value.");
    RCLCPP_INFO(get_logger(), " * Stream codec: H264");
  }

  getParam("stream_server.port", _streamingServerPort, _streamingServerPort, " * Stream port:");

  getParam("stream_server.bitrate", _streamingServerBitrate, _streamingServerBitrate);
  if (_streamingServerBitrate < 1000) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.bitrate': " << codec <<
        ". The minimum allowed value is 1000");
    _streamingServerBitrate = 1000;
  }
  if (_streamingServerBitrate > 60000) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.bitrate': " << codec <<
        ". The maximum allowed value is 60000");
    _streamingServerBitrate = 60000;
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Stream bitrate: " << _streamingServerBitrate);

  getParam("stream_server.gop_size", _streamingServerGopSize, _streamingServerGopSize);
  if (_streamingServerGopSize < -1) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.gop_size': " << codec <<
        ". The minimum allowed value is -1");
    _streamingServerGopSize = -1;
  }
  if (_streamingServerGopSize > 256) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.gop_size': " << codec <<
        ". The maximum allowed value is 256");
    _streamingServerGopSize = 256;
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Stream GOP size: " << _streamingServerGopSize);

  getParam("stream_server.chunk_size", _streamingServerChunckSize, _streamingServerChunckSize);
  if (_streamingServerChunckSize < 1024) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.chunk_size': " << codec <<
        ". The minimum allowed value is 1024");
    _streamingServerChunckSize = 1024;
  }
  if (_streamingServerChunckSize > 65000) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid value for the parameter 'stream_server.chunk_size': " << codec <<
        ". The maximum allowed value is 65000");
    _streamingServerChunckSize = 65000;
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Stream Chunk size: " << _streamingServerChunckSize);

  getParam(
    "stream_server.adaptative_bitrate", _streamingServerAdaptiveBitrate,
    _streamingServerAdaptiveBitrate);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Adaptive bitrate: " << (_streamingServerAdaptiveBitrate ? "TRUE" : "FALSE"));

  getParam(
    "stream_server.target_framerate", _streamingServerTargetFramerate,
    _streamingServerTargetFramerate, " * Target frame rate:");
}

void ZedCameraOne::getAdvancedParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** Advanced parameters ***");

  getParam(
    "advanced.thread_sched_policy", _threadSchedPolicy,
    _threadSchedPolicy, " * Thread sched. policy: ");

  if (_threadSchedPolicy == "SCHED_FIFO" || _threadSchedPolicy == "SCHED_RR") {
    if (!sl_tools::checkRoot()) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "'sudo' permissions required to set "
          << _threadSchedPolicy
          << " thread scheduling policy. Using Linux "
          "default [SCHED_OTHER]");
      _threadSchedPolicy = "SCHED_OTHER";
    } else {
      getParam(
        "advanced.thread_grab_priority", _threadPrioGrab,
        _threadPrioGrab, " * Grab thread priority: ");
      getParam(
        "advanced.thread_sensor_priority", _threadPrioSens,
        _threadPrioSens, " * Sensors thread priority: ");
    }
  }
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

  getParam("debug.debug_advanced", _debugAdvanced, _debugAdvanced);
  RCLCPP_INFO(
    get_logger(), " * Debug Advanced: %s",
    _debugAdvanced ? "TRUE" : "FALSE");

  // Set debug mode
  _debugMode = _debugCommon || _debugVideoDepth || _debugCamCtrl ||
    _debugSensors || _debugStreaming || _debugAdvanced;

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
    "ZED X One Diagnostic", this,
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

  // ----> TF2 Transform
  _tfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  // Start TF Listener thread
  _tfListener = std::make_unique<tf2_ros::TransformListener>(*_tfBuffer);
  _tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  // <---- TF2 Transform

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
    RCLCPP_WARN(get_logger(), "Camera model does not match user parameter.");

    if (_camRealModel == sl::MODEL::ZED_XONE_GS) {
      RCLCPP_WARN(get_logger(), "Please set the parameter 'general.camera_model' to 'zedxonegs'");
    } else if (_camRealModel == sl::MODEL::ZED_XONE_UHD) {
      RCLCPP_WARN(get_logger(), "Please set the parameter 'general.camera_model' to 'zedxone4k'");
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
  pub_h = static_cast<int>(std::round(_camHeight / _customDownscaleFactor));

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
  fillCamInfo(_camInfoMsg, _camImgFrameId, false);
  // Raw image
  fillCamInfo(_camInfoRawMsg, _camImgFrameId, true);
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

  // ----> Initialize Diagnostic statistics
  _elabPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(_camGrabFrameRate);
  _grabPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(_camGrabFrameRate);
  _videoPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(_camGrabFrameRate);
  _videoElabMean_sec = std::make_unique<sl_tools::WinAvg>(_camGrabFrameRate);
  _imuPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(_sensPubRate);
  _pubImuTF_sec = std::make_unique<sl_tools::WinAvg>(_sensPubRate);
  // <---- Initialize Diagnostic statistics

  // Init and start threads
  initThreadsAndTimers();

  return true;
}

void ZedCameraOne::initThreadsAndTimers()
{
  // ----> Start CMOS Temperatures thread
  startTempPubTimer();
  // <---- Start CMOS Temperatures thread

  // Start sensor thread
  _sensThread = std::thread(&ZedCameraOne::threadFunc_pubSensorsData, this);

  // Start grab thread
  _grabThread = std::thread(&ZedCameraOne::threadFunc_zedGrab, this);
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
  DEBUG_STREAM_SENS("Camera temperature: " << _tempImu << "°C");
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

  if (_connStatus != sl::ERROR_CODE::SUCCESS) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      sl::toString(_connStatus).c_str());
    return;
  }

  if (_grabStatus == sl::ERROR_CODE::SUCCESS) {
    double freq = 1. / _grabPeriodMean_sec->getAvg();
    double freq_perc = 100. * freq / _camGrabFrameRate;
    stat.addf("Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);

    double frame_proc_sec = _elabPeriodMean_sec->getAvg();
    double frame_grab_period = 1. / _camGrabFrameRate;
    stat.addf(
      "Capture", "Tot. Processing Time: %.6f sec (Max. %.3f sec)",
      frame_proc_sec, frame_grab_period);

    if (frame_proc_sec > frame_grab_period) {
      _sysOverloadCount++;
    }

    if (_sysOverloadCount >= 10) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "System overloaded. Consider reducing "
        "'general.pub_frame_rate' or 'general.grab_resolution'");
    } else {
      _sysOverloadCount = 0;
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Camera grabbing");
    }

    if (_streamMode) {
      stat.add("Input mode", "LOCAL STREAM");
    } else {
      stat.add("Input mode", "Live Camera");
    }

    if (_videoPublishing) {
      freq = 1. / _videoPeriodMean_sec->getAvg();
      freq_perc = 100. * freq / _camGrabFrameRate;
      frame_grab_period = 1. / _camGrabFrameRate;
      stat.addf(
        "Video", "Mean Frequency: %.1f Hz (%.1f%%)", freq,
        freq_perc);
      stat.addf(
        "Video", "Processing Time: %.6f sec (Max. %.3f sec)",
        _videoPeriodMean_sec->getAvg(), frame_grab_period);
    } else {
      stat.add("Video", "Topic not subscribed");
    }

    if (_svoMode) {
      int frame = _zed->getSVOPosition();
      int totFrames = _zed->getSVONumberOfFrames();
      double svo_perc = 100. * (static_cast<double>(frame) / totFrames);

      stat.addf(
        "Playing SVO", "Frame: %d/%d (%.1f%%)", frame, totFrames,
        svo_perc);
    }
    if (_publishImuTF) {
      freq = 1. / _pubImuTF_sec->getAvg();
      stat.addf("TF IMU", "Mean Frequency: %.1f Hz", freq);
    } else {
      stat.add("TF IMU", "DISABLED");
    }
  } else if (_grabStatus == sl::ERROR_CODE::LAST) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Camera initializing");
  } else {
    stat.summaryf(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Camera error: %s", sl::toString(_grabStatus).c_str());
  }

  if (_imuPublishing) {
    double freq = 1. / _imuPeriodMean_sec->getAvg();
    stat.addf("IMU", "Mean Frequency: %.1f Hz", freq);
  } else {
    stat.add("IMU Sensor", "Topics not subscribed");
  }


  stat.addf("Camera Temp.", "%.1f °C", _tempImu);

  if (_tempImu > 70.f) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "High Camera temperature");
  }


  if (_recording) {
    if (!_recStatus.status) {
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
        _recStatus.average_compression_time);
      stat.addf(
        "SVO compression ratio", "%.1f%%",
        _recStatus.average_compression_ratio);
    }
  } else {
    stat.add("SVO Recording", "NOT ACTIVE");
  }

  if (_streamingServerRunning) {
    stat.add("Streaming Server", "ACTIVE");

    sl::StreamingParameters params;
    params = _zed->getStreamingParameters();

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
  _camImgFrameId = _cameraName + "_camera_frame";
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
    zedParam =
      _zed->getCameraInformation(_matResol).camera_configuration.calibration_parameters_raw;
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
  RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << _pubColorImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << _pubColorImg.getInfoTopic());

  _imgRawTopic = _topicRoot + color_prefix + raw_prefix + image_topic;
  _pubColorRawImg = image_transport::create_camera_publisher(
    this, _imgRawTopic, _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << _pubColorRawImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << _pubColorRawImg.getInfoTopic());

  _imgGrayTopic = _topicRoot + gray_prefix + rect_prefix + image_topic;
  _pubGrayImg = image_transport::create_camera_publisher(
    this, _imgGrayTopic, _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << _pubGrayImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << _pubGrayImg.getInfoTopic());

  _imgRawGrayTopic = _topicRoot + gray_prefix + raw_prefix + image_topic;
  _pubGrayRawImg = image_transport::create_camera_publisher(
    this, _imgRawGrayTopic, _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << _pubGrayRawImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << _pubGrayRawImg.getInfoTopic());
  // <---- Images

  // ----> Sensors
  RCLCPP_INFO(get_logger(), " +++ SENSOR TOPICS +++");
  _tempTopic = _topicRoot + "temperature";
  _pubTemp = create_publisher<sensor_msgs::msg::Temperature>(_tempTopic, _qos, _pubOpt);
  RCLCPP_INFO_STREAM(get_logger(), " * Advertised on topic: " << _pubTemp->get_topic_name());

  std::string imuTopicRoot = "imu";
  std::string imu_topic_name = "data";
  std::string imu_topic_raw_name = "data_raw";
  std::string imu_topic = _topicRoot + imuTopicRoot + "/" + imu_topic_name;
  std::string imu_topic_raw = _topicRoot + imuTopicRoot + "/" + imu_topic_raw_name;
  _pubImu = create_publisher<sensor_msgs::msg::Imu>(imu_topic, _qos, _pubOpt);
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << _pubImu->get_topic_name());
  _pubImuRaw =
    create_publisher<sensor_msgs::msg::Imu>(imu_topic_raw, _qos, _pubOpt);
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << _pubImuRaw->get_topic_name());
  // <---- Sensors

  // ----> Camera/imu transform message
  std::string cam_imu_tr_topic = _topicRoot + "left_cam_imu_transform";
  _pubCamImuTransf = create_publisher<geometry_msgs::msg::TransformStamped>(
    cam_imu_tr_topic, _qos, _pubOpt);

  RCLCPP_INFO_STREAM(
    get_logger(), "Advertised on topic: "
      << _pubCamImuTransf->get_topic_name());

  sl::Orientation sl_rot = _slCamImuTransf.getOrientation();
  sl::Translation sl_tr = _slCamImuTransf.getTranslation();
  RCLCPP_INFO(
    get_logger(), "Camera-IMU Translation: \n %g %g %g", sl_tr.x,
    sl_tr.y, sl_tr.z);
  RCLCPP_INFO(
    get_logger(), "Camera-IMU Rotation:\n%s",
    sl_rot.getRotationMatrix().getInfos().c_str());
  // <---- Camera/imu transform message
}

void ZedCameraOne::threadFunc_zedGrab()
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

  if (_threadSchedPolicy == "SCHED_OTHER") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (_threadSchedPolicy == "SCHED_BATCH") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_BATCH, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (_threadSchedPolicy == "SCHED_FIFO") {
    sched_param par;
    par.sched_priority = _threadPrioGrab;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (_threadSchedPolicy == "SCHED_RR") {
    sched_param par;
    par.sched_priority = _threadPrioGrab;
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

  // ----> Init status variables
  _frameCount = 0;
  _threadStop = false;
  // <---- Init status variables

  // ----> Infinite grab thread
  while (1) {
    try {
      sl_tools::StopWatch grabElabTimer(get_clock());

      // ----> Interruption check
      if (!rclcpp::ok()) {
        DEBUG_STREAM_COMM("Ctrl+C received: stopping grab thread");
        _threadStop = true;
        break;
      }

      if (_threadStop) {
        DEBUG_STREAM_COMM("Grab thread stopped");
        break;
      }
      // <---- Interruption check

      // ----> Apply video dynamic parameters
      if (_svoMode) {
        applyVideoSettings();
      }
      // <---- Apply video dynamic parameters

      // ----> Grab freq calculation
      double elapsed_sec = _grabFreqTimer.toc();
      _grabPeriodMean_sec->addValue(elapsed_sec);
      _grabFreqTimer.tic();
      // <---- Grab freq calculation

      if (!_svoPause) {
        // Start processing timer for diagnostic
        grabElabTimer.tic();

        // ZED grab
        _grabStatus = _zed->grab();

        // ----> Grab errors?
        // Note: disconnection are automatically handled by the ZED SDK
        if (_grabStatus != sl::ERROR_CODE::SUCCESS) {
          if (_svoMode && _grabStatus == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
            // ----> Check SVO status
            if (_svoLoop) {
              _zed->setSVOPosition(0);
              RCLCPP_WARN(
                get_logger(),
                "SVO reached the end and it has been restarted.");
              rclcpp::sleep_for(
                std::chrono::microseconds(
                  static_cast<int>(_grabPeriodMean_sec->getAvg() * 1e6)));
              continue;
            } else {
              RCLCPP_WARN(
                get_logger(),
                "SVO reached the end. The node has been stopped.");
              break;
            }
            // <---- Check SVO status
          } else if (_grabStatus == sl::ERROR_CODE::CAMERA_REBOOTING) {
            RCLCPP_ERROR_STREAM(
              get_logger(),
              "Connection issue detected: "
                << sl::toString(_grabStatus).c_str());
            rclcpp::sleep_for(1000ms);
            continue;
          } else if (_grabStatus == sl::ERROR_CODE::CAMERA_NOT_INITIALIZED ||
            _grabStatus == sl::ERROR_CODE::FAILURE)
          {
            RCLCPP_ERROR_STREAM(
              get_logger(),
              "Camera issue detected: "
                << sl::toString(_grabStatus).c_str() << ". " << sl::toVerbose(
                _grabStatus).c_str() << ". Trying to recover the connection...");
            rclcpp::sleep_for(1000ms);
            continue;
          } else {
            RCLCPP_ERROR_STREAM(
              get_logger(),
              "Critical camera error: " << sl::toString(
                _grabStatus).c_str() << ". " << sl::toVerbose(
                _grabStatus).c_str() << ". NODE KILLED.");
            _zed.reset();
            exit(EXIT_FAILURE);
          }
        }
        // <---- Grab errors?

        // Update frame count
        _frameCount++;

        // ----> Timestamp
        if (_svoMode) {
          _frameTimestamp = sl_tools::slTime2Ros(
            _zed->getTimestamp(sl::TIME_REFERENCE::CURRENT));
        } else {
          _frameTimestamp =
            sl_tools::slTime2Ros(_zed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
        }
        DEBUG_STREAM_COMM("Grab timestamp: " << _frameTimestamp.nanoseconds() << " nsec");
        // <---- Timestamp

        if (_streamingServerRequired && !_streamingServerRunning) {
          DEBUG_STR("Streaming server required, but not running");
          startStreamingServer();
        }
      }

      // ----> Check recording status
      _recMutex.lock();
      if (_recording) {
        _recStatus = _zed->getRecordingStatus();

        if (!_recStatus.status) {
          rclcpp::Clock steady_clock(RCL_STEADY_TIME);
          RCLCPP_ERROR_THROTTLE(
            get_logger(), steady_clock, 1000.0,
            "Error saving frame to SVO");
        }
      }
      _recMutex.unlock();
      // <---- Check recording status

      // ----> Retrieve Image/Depth data if someone has subscribed to
      // Retrieve data if there are subscriber to topics
      _imageSubscribed = areVideoDepthSubscribed();
      if (_imageSubscribed) {
        DEBUG_STREAM_VD("Retrieving video data");
        retrieveVideo();

        rclcpp::Time pub_ts;
        publishVideo(pub_ts);

        _videoPublishing = true;
      } else {
        _videoPublishing = false;
      }
      // <---- Retrieve Image/Depth data if someone has subscribed to

      // Diagnostic statistics update
      double mean_elab_sec = _elabPeriodMean_sec->addValue(grabElabTimer.toc());
    } catch (...) {
      rcutils_reset_error();
      DEBUG_STREAM_COMM("threadFunc_zedGrab: Generic exception.");
      continue;
    }
  }
  // <---- Infinite grab thread
  DEBUG_STREAM_COMM("Grab thread finished");
}

void ZedCameraOne::threadFunc_pubSensorsData()
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

  if (_threadSchedPolicy == "SCHED_OTHER") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (_threadSchedPolicy == "SCHED_BATCH") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_BATCH, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (_threadSchedPolicy == "SCHED_FIFO") {
    sched_param par;
    par.sched_priority = _threadPrioSens;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (_threadSchedPolicy == "SCHED_RR") {
    sched_param par;
    par.sched_priority = _threadPrioSens;
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

  // ----> Infinite sensor loop
  DEBUG_STREAM_SENS("Sensors thread loop starting...");

  _lastTs_imu = TIMEZERO_ROS;

  while (1) {
    try {
      if (!rclcpp::ok()) {
        DEBUG_STREAM_SENS("Ctrl+C received: stopping sensors thread");
        _threadStop = true;
        break;
      }
      if (_threadStop) {
        DEBUG_STREAM_SENS(
          "[threadFunc_pubSensorsData] (2): Sensors thread stopped");
        break;
      }

      // std::lock_guard<std::mutex> lock(mCloseZedMutex);
      if (!_zed->isOpened()) {
        DEBUG_STREAM_SENS("[threadFunc_pubSensorsData] the camera is not open");
        rclcpp::sleep_for(std::chrono::milliseconds(100));// Avoid busy-waiting
        continue;
      }

      rclcpp::Time sens_ts = publishSensorsData();

    } catch (...) {
      rcutils_reset_error();
      DEBUG_STREAM_COMM("[threadFunc_pubSensorsData] Generic exception.");
      continue;
    }

    // ----> Check publishing frequency
    double sens_period_usec = 1e6 / _sensPubRate;
    DEBUG_STREAM_SENS(
      "[threadFunc_pubSensorsData] sens_period_usec: " << sens_period_usec << " usec");

    double elapsed_usec = _sensPubFreqTimer.toc() * 1e6;

    DEBUG_STREAM_SENS(
      "[threadFunc_pubSensorsData] elapsed_usec: " << elapsed_usec << " - Freq: " << 1e6 / elapsed_usec <<
        " Hz");

    if (elapsed_usec < sens_period_usec) {
      int sleep_usec = static_cast<int>(sens_period_usec - elapsed_usec);
      DEBUG_STREAM_SENS("[threadFunc_pubSensorsData] Sleep period: " << sleep_usec << " usec");
      rclcpp::sleep_for(
        std::chrono::microseconds(sleep_usec));
    }

    _sensPubFreqTimer.tic();
    // <---- Check publishing frequency
  }
  // <---- Infinite sensor loop

  DEBUG_STREAM_SENS("Sensors thread finished");

}

void ZedCameraOne::applyVideoSettings()
{

}

bool ZedCameraOne::startStreamingServer()
{
  DEBUG_STR("Starting streaming server");

  if (_zed->isStreamingEnabled()) {
    _zed->disableStreaming();
    RCLCPP_WARN(get_logger(), "A streaming server was already running and has been stopped");
  }

  sl::StreamingParameters params;
  params.adaptative_bitrate = _streamingServerAdaptiveBitrate;
  params.bitrate = _streamingServerBitrate;
  params.chunk_size = _streamingServerChunckSize;
  params.codec = _streamingServerCodec;
  params.gop_size = _streamingServerGopSize;
  params.port = _streamingServerPort;
  params.target_framerate = _streamingServerTargetFramerate;

  sl::ERROR_CODE res;
  res = _zed->enableStreaming(params);
  if (res != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Error starting the Streaming server: " << sl::toString(
        res) << " - " << sl::toVerbose(res));
    _streamingServerRunning = false;
  } else {
    _streamingServerRunning = true;
    RCLCPP_INFO(get_logger(), "Streaming server started");
  }
  return _streamingServerRunning;
}

void ZedCameraOne::stopStreamingServer()
{
  if (_zed->isStreamingEnabled()) {
    _zed->disableStreaming();
    RCLCPP_INFO(get_logger(), "Streaming server stopped");
  } else {
    RCLCPP_WARN(get_logger(), "A streaming server was not enabled.");
  }

  _streamingServerRunning = false;
  _streamingServerRequired = false;
}

rclcpp::Time ZedCameraOne::publishSensorsData(rclcpp::Time t /*= TIMEZERO_ROS*/)
{
  if (_grabStatus != sl::ERROR_CODE::SUCCESS) {
    DEBUG_SENS("Camera not ready");
    return TIMEZERO_ROS;
  }

  // ----> Subscribers count
  size_t imu_SubNumber = 0;
  size_t imu_RawSubNumber = 0;

  try {
    imu_SubNumber = count_subscribers(_pubImu->get_topic_name());
    imu_RawSubNumber = count_subscribers(_pubImuRaw->get_topic_name());

    if (imu_SubNumber + imu_RawSubNumber == 0) {
      _imuPublishing = false;
    }

    DEBUG_STREAM_SENS("[publishSensorsData] subscribers: " << imu_SubNumber + imu_RawSubNumber);
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_SENS("[publishSensorsData] Exception while counting subscribers");
    return TIMEZERO_ROS;
  }
  // <---- Subscribers count

  // ----> Grab data and setup timestamps
  DEBUG_STREAM_SENS("[publishSensorsData] Grab data and setup timestamps");
  rclcpp::Time ts_imu;

  rclcpp::Time now = get_clock()->now();

  sl::SensorsData sens_data;

  if (_svoMode) {
    sl::ERROR_CODE err =
      _zed->getSensorsData(sens_data, sl::TIME_REFERENCE::IMAGE);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "[publishSensorsData] sl::getSensorsData error: "
          << sl::toString(err).c_str());
      return TIMEZERO_ROS;
    }
  } else {
    sl::ERROR_CODE err =
      _zed->getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "[publishSensorsData] sl::getSensorsData error: "
          << sl::toString(err).c_str());
      return TIMEZERO_ROS;
    }
  }

  DEBUG_STREAM_SENS("[publishSensorsData] Sensors data grabbed");

  ts_imu = sl_tools::slTime2Ros(sens_data.imu.timestamp);
  DEBUG_STREAM_SENS("[publishSensorsData] ts_imu: " << ts_imu.nanoseconds() << " nsec");
  // <---- Grab data and setup timestamps

  // ----> Check for not updated data
  double dT = ts_imu.seconds() - _lastTs_imu.seconds();
  _lastTs_imu = ts_imu;
  DEBUG_STREAM_SENS("[publishSensorsData] dT: " << dT << " sec");
  bool new_imu_data = (dT > 0.0);

  if (!new_imu_data) {
    DEBUG_STREAM_SENS("[publishSensorsData] No new sensors data");
    return TIMEZERO_ROS;
  }
  // <---- Check for not updated data

  DEBUG_STREAM_SENS(
    "SENSOR LAST PERIOD: " << dT << " sec @" << 1. / dT
                           << " Hz");

  // ----> Sensors freq for diagnostic
  double mean = _imuPeriodMean_sec->addValue(_imuFreqTimer.toc());
  _imuFreqTimer.tic();

  DEBUG_STREAM_SENS("IMU MEAN freq: " << 1. / mean);
  // <---- Sensors freq for diagnostic

  // ----> Sensors data publishing

  publishImuFrameAndTopic();

  if (imu_SubNumber > 0) {
    DEBUG_STREAM_SENS("[publishSensorsData] IMU subscribers: " << static_cast<int>(imu_SubNumber));
    _imuPublishing = true;

    imuMsgPtr imuMsg = std::make_unique<sensor_msgs::msg::Imu>();

    imuMsg->header.stamp = ts_imu;
    imuMsg->header.frame_id = _imuFrameId;

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

    DEBUG_STREAM_SENS("[publishSensorsData] Publishing IMU message");
    try {
      _pubImu->publish(std::move(imuMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }

  if (imu_RawSubNumber > 0) {
    DEBUG_STREAM_SENS(
      "[publishSensorsData] IMU subscribers: " <<
        static_cast<int>(imu_RawSubNumber));
    _imuPublishing = true;

    imuMsgPtr imuRawMsg = std::make_unique<sensor_msgs::msg::Imu>();

    imuRawMsg->header.stamp = ts_imu;
    imuRawMsg->header.frame_id = _imuFrameId;

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

    DEBUG_STREAM_SENS("[publishSensorsData] Publishing IMU RAW message");
    try {
      _pubImuRaw->publish(std::move(imuRawMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
  // <---- Sensors data publishing

  return ts_imu;
}

void ZedCameraOne::publishImuFrameAndTopic()
{
  sl::Orientation sl_rot = _slCamImuTransf.getOrientation();
  sl::Translation sl_tr = _slCamImuTransf.getTranslation();

  auto cameraImuTransfMsg = std::make_unique<geometry_msgs::msg::TransformStamped>();

  cameraImuTransfMsg->header.stamp = get_clock()->now();
  cameraImuTransfMsg->header.frame_id = _camImgFrameId;
  cameraImuTransfMsg->child_frame_id = _imuFrameId;

  cameraImuTransfMsg->transform.rotation.x = sl_rot.ox;
  cameraImuTransfMsg->transform.rotation.y = sl_rot.oy;
  cameraImuTransfMsg->transform.rotation.z = sl_rot.oz;
  cameraImuTransfMsg->transform.rotation.w = sl_rot.ow;

  cameraImuTransfMsg->transform.translation.x = sl_tr.x;
  cameraImuTransfMsg->transform.translation.y = sl_tr.y;
  cameraImuTransfMsg->transform.translation.z = sl_tr.z;

  try {
    _pubCamImuTransf->publish(std::move(cameraImuTransfMsg));
  } catch (const std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic exception.");
  }

  if (!_publishImuTF) {
    return;
  }

  auto transformStamped = std::make_unique<geometry_msgs::msg::TransformStamped>();

  transformStamped->header.stamp = get_clock()->now();
  transformStamped->header.frame_id = _camImgFrameId;
  transformStamped->child_frame_id = _imuFrameId;

  transformStamped->transform.rotation.x = sl_rot.ox;
  transformStamped->transform.rotation.y = sl_rot.oy;
  transformStamped->transform.rotation.z = sl_rot.oz;
  transformStamped->transform.rotation.w = sl_rot.ow;

  transformStamped->transform.translation.x = sl_tr.x;
  transformStamped->transform.translation.y = sl_tr.y;
  transformStamped->transform.translation.z = sl_tr.z;

  _tfBroadcaster->sendTransform(*transformStamped);

  double elapsed_sec = _imuTfFreqTimer.toc();
  _pubImuTF_sec->addValue(elapsed_sec);
  _imuTfFreqTimer.tic();
}

bool ZedCameraOne::areVideoDepthSubscribed()
{
  _colorSubNumber = 0;
  _colorRawSubNumber = 0;
  _graySubNumber = 0;
  _grayRawSubNumber = 0;

  try {
    _colorSubNumber = _pubColorImg.getNumSubscribers();
    _colorRawSubNumber = _pubColorRawImg.getNumSubscribers();
    _graySubNumber = _pubGrayImg.getNumSubscribers();
    _grayRawSubNumber = _pubGrayRawImg.getNumSubscribers();
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_VD("publishImages: Exception while counting subscribers");
    return false;
  }

  return (_colorSubNumber + _colorRawSubNumber + _graySubNumber +
         _grayRawSubNumber) > 0;
}

void ZedCameraOne::retrieveVideo()
{
  bool retrieved = false;

  // ----> Retrieve all required data
  DEBUG_VD("Retrieving Video Data");
  if (_colorSubNumber > 0) {
    retrieved |=
      (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(_matColor, sl::VIEW::LEFT, sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matColor.timestamp;
    DEBUG_VD("Color image retrieved");
  }
  if (_colorRawSubNumber > 0) {
    retrieved |= (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(
        _matColorRaw, sl::VIEW::LEFT_UNRECTIFIED,
        sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matColorRaw.timestamp;
    DEBUG_VD("Color raw image retrieved");
  }
  if (_graySubNumber > 0) {
    retrieved |=
      (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(_matGray, sl::VIEW::LEFT_GRAY sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matGray.timestamp;
    DEBUG_VD("Gray image retrieved");
  }
  if (_grayRawSubNumber > 0) {
    retrieved |=
      (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(_matGrayRaw, sl::VIEW::RIGHT_UNRECTIFIED, sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matGrayRaw.timestamp;
    DEBUG_VD("Gray raw image retrieved");
  }
  if (retrieved) {
    DEBUG_STREAM_VD("Video Data retrieved");
  }
  // <---- Retrieve all required data
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCameraOne)
