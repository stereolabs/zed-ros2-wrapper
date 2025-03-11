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
: Node("zed_node_one", options),
  _threadStop(false),
  _qos(QOS_QUEUE_SIZE),
  _diagUpdater(this),
  _grabFreqTimer(get_clock()),
  _imuFreqTimer(get_clock()),
  _imuTfFreqTimer(get_clock()),
  _imgPubFreqTimer(get_clock()),
  _frameTimestamp(_frameTimestamp),
  _lastTs_imu(_frameTimestamp),
  _colorSubCount(0),
  _colorRawSubCount(0),
#if ENABLE_GRAY_IMAGE
  _graySubCount(0),
  _grayRawSubCount(0),
#endif
  _imuSubCount(0),
  _imuRawSubCount(0),
  _streamingServerRequired(false),
  _streamingServerRunning(false),
  _triggerUpdateDynParams(true),
  _uptimer(get_clock())
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

  // ----> Start a "one shot timer" to initialize the node
  // This is required to make `shared_from_this` available
  std::chrono::milliseconds init_msec(static_cast<int>(50.0));
  _initTimer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(init_msec),
    std::bind(&ZedCameraOne::init, this));
  // <---- Start a "one shot timer" to initialize the node
}

ZedCameraOne::~ZedCameraOne()
{
  DEBUG_STREAM_COMM("Destroying node");

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

  DEBUG_STREAM_COMM("Waiting for sensors thread...");
  try {
    if (_sensThread.joinable()) {
      _sensThread.join();
    }
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Sensors thread joining exception: " << e.what());
  }
  DEBUG_STREAM_COMM("... sensors thread stopped");
  // <---- Verify that all the threads are not active

  // ----> Close the ZED camera
  DEBUG_STREAM_COMM("Closing ZED camera...");
  _zed->close();
  DEBUG_STREAM_COMM("... ZED camera closed");
  // <---- Close the ZED camera
}

void ZedCameraOne::initParameters()
{
  // DEBUG parameters
  getDebugParams();

  // GENERAL parameters
  getGeneralParams();

  // Image Parameters
  getVideoParams();

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

#if !ENABLE_SVO
  if (_svoFilepath != "") {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "SVO input is not enabled in this version. This feature will be available later");
    exit(EXIT_FAILURE);
  }
#endif

#if ENABLE_SVO
  if (_svoFilepath == "") {
    _svoMode = false;
  } else {
    RCLCPP_INFO_STREAM(get_logger(), " * SVO: '" << _svoFilepath.c_str() << "'");
    _svoMode = true;
    getParam("svo.svo_loop", _svoLoop, _svoLoop, " * SVO Loop: ");
    getParam("svo.svo_realtime", _svoRealtime, _svoRealtime, " * SVO Real Time: ");
    getParam(
      "svo.use_svo_timestamps", _useSvoTimestamp, _useSvoTimestamp,
      " * Use SVO timestamp: ");
  }
#endif

  _streamMode = false;
  if (!_svoMode) {
    getParam("stream.stream_address", std::string(), _streamAddr);
    if (_streamAddr != "") {
#if ENABLE_STREAM_INPUT
      _streamMode = true;
      getParam("stream.stream_port", _streamPort, _streamPort);
      RCLCPP_INFO_STREAM(
        get_logger(), " * Local stream input: " << _streamAddr << ":" << _streamPort);
#else
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Local stream input is not enabled in this version. This feature will be available later");
      exit(EXIT_FAILURE);
#endif
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
    "general.camera_id", _camId, _camId,
    " * Camera ID: ");
  getParam(
    "general.grab_frame_rate", _camGrabFrameRate, _camGrabFrameRate,
    " * Camera framerate: ", false, 15, 120);
  getParam("general.gpu_id", _gpuId, _gpuId, " * GPU ID: ", false, -1, 256);

  std::string resol = "AUTO";
  getParam("general.grab_resolution", resol, resol);
  if (resol == "AUTO") {
    _camResol = sl::RESOLUTION::AUTO;
  } else if (resol == "HD4K" && _camUserModel == sl::MODEL::ZED_XONE_UHD) {
    _camResol = sl::RESOLUTION::HD4K;
  } else if (resol == "QHDPLUS" && _camUserModel == sl::MODEL::ZED_XONE_UHD) {
    _camResol = sl::RESOLUTION::QHDPLUS;
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
      _customDownscaleFactor, " * Publishing downscale factor: ", false, 1.0, 5.0);
  } else {
    _customDownscaleFactor = 1.0;
  }

  getParam(
    "general.optional_opencv_calibration_file", _opencvCalibFile,
    _opencvCalibFile, " * OpenCV custom calibration: ");
}

void ZedCameraOne::getSensorsParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** SENSORS parameters ***");

  getParam("sensors.publish_imu_tf", _publishImuTF, _publishImuTF, " * Publish IMU TF: ");
  getParam(
    "sensors.sensors_pub_rate", _sensPubRate, _sensPubRate,
    " * Sensors publishing rate [Hz]: ", true, 1.0, 400.0);
}

void ZedCameraOne::getStreamingServerParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** Streaming Server parameters ***");

  bool stream_server = false;
  getParam("stream_server.stream_enabled", stream_server, stream_server, " * Stream enabled: ");
  _streamingServerRequired = stream_server;

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

  getParam(
    "stream_server.port", _streamingServerPort, _streamingServerPort, " * Stream port:",
    false, 1024, 65535);
  getParam(
    "stream_server.bitrate", _streamingServerBitrate, _streamingServerBitrate,
    " * Stream bitrate:", false, 1000, 60000);
  getParam(
    "stream_server.gop_size", _streamingServerGopSize, _streamingServerGopSize,
    " * Stream GOP size:", false, -1, 256);
  getParam(
    "stream_server.chunk_size", _streamingServerChunckSize, _streamingServerChunckSize,
    " * Stream Chunk size:", false, 1024, 65000);
  getParam(
    "stream_server.adaptative_bitrate", _streamingServerAdaptiveBitrate,
    _streamingServerAdaptiveBitrate, " * Adaptative bitrate:");
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

  getParam("debug.sdk_verbose", _sdkVerbose, _sdkVerbose, " * SDK Verbose: ", false, 0, 1000);

  getParam("debug.debug_common", _debugCommon, _debugCommon, " * Debug Common: ");
  getParam("debug.debug_video_depth", _debugVideoDepth, _debugVideoDepth, " * Debug Image/Depth: ");
  getParam(
    "debug.debug_camera_controls", _debugCamCtrl, _debugCamCtrl,
    " * Debug Camera Controls: ");
  getParam("debug.debug_sensors", _debugSensors, _debugSensors, " * Debug Sensors: ");
  getParam("debug.debug_streaming", _debugStreaming, _debugStreaming, " * Debug Streaming: ");
  getParam("debug.debug_advanced", _debugAdvanced, _debugAdvanced, " * Debug Advanced: ");

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

void ZedCameraOne::getVideoParams()
{
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "*** CAMERA CONTROL parameters ***");

  getParam("video.enable_hdr", _enableHDR, _enableHDR, " * Enable HDR: ");

  getParam("video.saturation", _camSaturation, _camSaturation, " * [DYN] Saturation: ", true, 0, 8);
  _camDynParMapChanged["video.saturation"] = true;
  getParam(
    "video.sharpness", _camSharpness, _camSharpness,
    " * [DYN] Sharpness: ", true, 0, 8);
  _camDynParMapChanged["video.sharpness"] = true;
  getParam("video.gamma", _camGamma, _camGamma, " * [DYN] Gamma: ", true, 1, 9);
  _camDynParMapChanged["video.gamma"] = true;
  getParam(
    "video.auto_whitebalance", _camAutoWB, _camAutoWB, " * [DYN] Auto White Balance: ",
    true);
  _camDynParMapChanged["video.auto_whitebalance"] = true;
  getParam(
    "video.whitebalance_temperature", _camWBTemp, _camWBTemp,
    " * [DYN] White Balance Temp (x100): ", true, 28, 65);
  _camDynParMapChanged["video.whitebalance_temperature"] = true;

  getParam(
    "video.auto_exposure", _camAutoExposure, _camAutoExposure,
    " * [DYN] Auto Exposure: ", true);
  _camDynParMapChanged["video.auto_exposure"] = true;
  getParam(
    "video.exposure_time", _camExpTime, _camExpTime,
    " * [DYN] Exposure (us): ", true, 28, 30000);
  _camDynParMapChanged["video.exposure_time"] = true;
  getParam(
    "video.auto_exposure_time_range_min", _camAutoExpTimeRangeMin,
    _camAutoExpTimeRangeMin, " * [DYN] Auto Exp Time Min (us): ", true,
    28, 30000);
  _camDynParMapChanged["video.auto_exposure_time_range_min"] = true;
  getParam(
    "video.auto_exposure_time_range_max", _camAutoExpTimeRangeMax,
    _camAutoExpTimeRangeMax, " * [DYN] Auto Exp Time Max (us): ", true,
    28, 30000);
  _camDynParMapChanged["video.auto_exposure_time_range_max"] = true;
  getParam(
    "video.exposure_compensation", _camExposureComp, _camExposureComp,
    " * [DYN] Exposure Compensation: ", true, 0, 100);
  _camDynParMapChanged["video.exposure_compensation"] = true;
  getParam(
    "video.auto_analog_gain", _camAutoAnalogGain, _camAutoAnalogGain,
    " * [DYN] Auto Analog Gain: ", true);
  _camDynParMapChanged["video.auto_analog_gain"] = true;
  getParam(
    "video.analog_gain", _camAnalogGain, _camAnalogGain,
    " * [DYN] Analog Gain: ", true, 1000, 16000);
  _camDynParMapChanged["video.analog_gain"] = true;
  getParam(
    "video.auto_analog_gain_range_min", _camAutoAnalogGainRangeMin,
    _camAutoAnalogGainRangeMin, " * [DYN] Analog Gain Min: ", true, 1000,
    16000);
  _camDynParMapChanged["video.auto_analog_gain_range_min"] = true;
  getParam(
    "video.auto_analog_gain_range_max", _camAutoAnalogGainRangeMax,
    _camAutoAnalogGainRangeMax, " * [DYN] Analog Gain Max: ", true, 1000,
    16000);
  _camDynParMapChanged["video.auto_analog_gain_range_max"] = true;
  getParam(
    "video.auto_digital_gain", _camAutoDigitalGain, _camAutoDigitalGain,
    " * [DYN] Auto Digital Gain: ", true);
  _camDynParMapChanged["video.auto_digital_gain"] = true;
  getParam(
    "video.digital_gain", _camDigitalGain, _camDigitalGain,
    " * [DYN] Digital Gain: ", true, 1, 256);
  _camDynParMapChanged["video.digital_gain"] = true;
  getParam(
    "video.auto_digital_gain_range_min", _camAutoDigitalGainRangeMin,
    _camAutoDigitalGainRangeMin, " * [DYN] Digital Gain Min: ", true, 1,
    256);
  _camDynParMapChanged["video.auto_digital_gain_range_min"] = true;
  getParam(
    "video.auto_digital_gain_range_max", _camAutoDigitalGainRangeMax,
    _camAutoDigitalGainRangeMax, " * [DYN] Digital Gain Max: ", true, 1,
    256);
  _camDynParMapChanged["video.auto_digital_gain_range_max"] = true;
  getParam(
    "video.denoising", _camDenoising, _camDenoising,
    " * [DYN] Denoising: ", true, 0, 100);
  _camDynParMapChanged["video.denoising"] = true;

  _triggerUpdateDynParams = true;
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
    std::bind(&ZedCameraOne::callback_setParameters, this, _1));
}

void ZedCameraOne::initServices()
{
  RCLCPP_INFO(get_logger(), "*** SERVICES ***");

  std::string srv_name;
  std::string srv_prefix = "~/";

  // Enable Streaming
  srv_name = srv_prefix + _srvEnableStreamingName;
  _srvEnableStreaming = create_service<std_srvs::srv::SetBool>(
    srv_name,
    std::bind(&ZedCameraOne::callback_enableStreaming, this, _1, _2, _3));
  RCLCPP_INFO(get_logger(), " * '%s'", _srvEnableStreaming->get_service_name());

#if ENABLE_SVO
  // Start SVO Recording
  srv_name = srv_prefix + _srvStartSvoRecName;
  _srvStartSvoRec = create_service<zed_msgs::srv::StartSvoRec>(
    srv_name,
    std::bind(&ZedCameraOne::callback_startSvoRec, this, _1, _2, _3));
  RCLCPP_INFO(get_logger(), " * '%s'", _srvStartSvoRec->get_service_name());
  // Stop SVO Recording
  srv_name = srv_prefix + _srvStopSvoRecName;
  _srvStopSvoRec = create_service<std_srvs::srv::Trigger>(
    srv_name,
    std::bind(&ZedCameraOne::callback_stopSvoRec, this, _1, _2, _3));
  RCLCPP_INFO(get_logger(), " * '%s'", _srvStopSvoRec->get_service_name());

  // Pause SVO Playback
  if (_svoMode && !_svoRealtime) {
    srv_name = srv_prefix + _srvToggleSvoPauseName;
    _srvPauseSvo = create_service<std_srvs::srv::Trigger>(
      srv_name,
      std::bind(&ZedCameraOne::callback_pauseSvoInput, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", _srvPauseSvo->get_service_name());
  }
#endif
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
#if ENABLE_SVO
  if (!_svoFilepath.empty()) {
    RCLCPP_INFO(get_logger(), "*** SVO OPENING ***");

    _initParams.input.setFromSVOFile(_svoFilepath.c_str());
    _initParams.svo_real_time_mode = _svoRealtime;

    _svoMode = true;
  } else
#endif
  if (!_streamAddr.empty()) {
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
      _initParams.input.setFromSerialNumber(_camSerialNumber, sl::BUS_TYPE::GMSL);
    } else if (_camId >= 0) {
      _initParams.input.setFromCameraID(_camId, sl::BUS_TYPE::GMSL, sl::CAMERA_TYPE::MONO);
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

  // Initialize Up timer
  _uptimer.tic();

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
#if ENABLE_SVO
  if (_svoMode) {
    RCLCPP_INFO(
      get_logger(), " * SVO resolution\t-> %dx%d",
      camInfo.camera_configuration.resolution.width,
      camInfo.camera_configuration.resolution.height);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * SVO framerate\t-> "
        << (camInfo.camera_configuration.fps));
  }
#endif

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
    if (_useSvoTimestamp) {
      _frameTimestamp = sl_tools::slTime2Ros(_zed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
    } else {
      _frameTimestamp =
        sl_tools::slTime2Ros(_zed->getTimestamp(sl::TIME_REFERENCE::CURRENT));
    }
  } else {
    _frameTimestamp =
      sl_tools::slTime2Ros(_zed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
  }
  // <---- Timestamp

  // ----> Initialize Diagnostic statistics
  _elabPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(_camGrabFrameRate);
  _grabPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(_camGrabFrameRate);
  _imagePeriodMean_sec = std::make_unique<sl_tools::WinAvg>(_camGrabFrameRate);
  _imageElabMean_sec = std::make_unique<sl_tools::WinAvg>(_camGrabFrameRate);
  _imuPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(20);
  _pubImuTF_sec = std::make_unique<sl_tools::WinAvg>(_sensPubRate);
  _pubImu_sec = std::make_unique<sl_tools::WinAvg>(_sensPubRate);
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
  size_t tempSubCount;

  try {
    tempSubCount = count_subscribers(_pubTemp->get_topic_name());
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_SENS(
      "callback_pubTemp: Exception while counting subscribers");
    return;
  }
  // <---- Subscribers count

  // ----> Publish temperature
  if (tempSubCount > 0) {
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

  stat.addf("Uptime", "%s", sl_tools::seconds2str(_uptimer.toc()).c_str());

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

    // ----> Frame drop count
    auto dropped = _zed->getFrameDroppedCount();
    uint64_t total = dropped + _frameCount;
    auto perc_drop = 100. * static_cast<double>(dropped) / total;
    stat.addf(
      "Frame Drop rate", "%u/%lu (%g%%)",
      dropped, total, perc_drop);
    // <---- Frame drop count

#if ENABLE_SVO
    if (_svoMode) {
      stat.add("Input mode", "SVO");
      stat.addf("SVO file", "%s", _svoFilepath.c_str());
    } else
#endif
    if (_streamMode) {
      stat.add("Input mode", "LOCAL STREAM");
    } else {
      stat.add("Input mode", "Live Camera");
    }

    if (_videoPublishing) {
      freq = 1. / _imagePeriodMean_sec->getAvg();
      freq_perc = 100. * freq / _camGrabFrameRate;
      frame_grab_period = 1. / _camGrabFrameRate;
      stat.addf("Image", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
      stat.addf(
        "Image", "Processing Time: %.6f sec (Max. %.3f sec)",
        _imagePeriodMean_sec->getAvg(), frame_grab_period);
    } else {
      stat.add("Image", "Topic not subscribed");
    }

#if ENABLE_SVO
    if (_svoMode) {
      int frame = _zed->getSVOPosition();
      int totFrames = _zed->getSVONumberOfFrames();
      double svo_perc = 100. * (static_cast<double>(frame) / totFrames);

      stat.addf(
        "Playing SVO", "Frame: %d/%d (%.1f%%)", frame, totFrames,
        svo_perc);
    }
#endif

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
      "%s", sl::toString(_grabStatus).c_str());
  }

  if (_imuPublishing) {
    double freq = 1. / _pubImu_sec->getAvg();
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

#if ENABLE_SVO
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
      stat.addf("SVO Frame Encoded", "%d", _recStatus.number_frames_encoded);
      stat.addf("SVO Frame Ingested", "%d", _recStatus.number_frames_ingested);
    }
  } else {
    stat.add("SVO Recording", "NOT ACTIVE");
  }
#endif

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

rcl_interfaces::msg::SetParametersResult ZedCameraOne::callback_setParameters(
  std::vector<rclcpp::Parameter> parameters)
{
  DEBUG_STREAM_COMM("*** Parameter change callback ***");

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  int count_ok = 0;
  int count = 0;

  DEBUG_STREAM_COMM("Modifying " << parameters.size() << " parameters");

  // ----> Lambda function: Update dynamic int parameters
  auto updateIntParam = [this](const std::string & paramName, const int value, int & paramVal) {
      if (value != paramVal) {
        paramVal = value;
        _camDynParMapChanged[paramName] = true;
        DEBUG_STREAM_COMM(" * " << paramName << " changed to " << value);
      } else {
        _camDynParMapChanged[paramName] = false;
        DEBUG_STREAM_COMM(" * " << paramName << " not changed: " << value);
      }
      return _camDynParMapChanged[paramName];
    };
  // <---- Lambda function: Update dynamic int parameters

  // ----> Lambda function: Update dynamic bool parameters
  auto updateBoolParam = [this](const std::string & paramName, const bool value, bool & paramVal) {
      if (value != paramVal) {
        paramVal = value;
        _camDynParMapChanged[paramName] = true;
        DEBUG_STREAM_COMM(" * " << paramName << " changed to " << value);
      } else {
        _camDynParMapChanged[paramName] = false;
        DEBUG_STREAM_COMM(" * " << paramName << " not changed: " << value);
      }
      return _camDynParMapChanged[paramName];
    };
  // <---- Lambda function: Update dynamic bool parameters

  for (const auto & param : parameters) {
    DEBUG_STREAM_COMM("Param #" << count << ": " << param.get_name());

    count++;

    std::string param_name = param.get_name();

    if (param_name == "sensors.sensors_pub_rate") {
      double value = param.as_double();
      if (value != _sensPubRate) {
        _sensPubRate = value;
        _pubImuTF_sec->setNewSize(static_cast<int>(_sensPubRate));
        _pubImu_sec->setNewSize(static_cast<int>(_sensPubRate));
        DEBUG_STREAM_COMM(" * " << param_name << " changed to " << value);
      } else {
        DEBUG_STREAM_COMM(" * " << param_name << " not changed: " << value);
      }
    } else {
      auto found = _camDynParMapChanged.find(param_name);
      if (found != _camDynParMapChanged.end()) {
        if (param_name == "video.saturation") { // Saturation
          if (updateIntParam(param_name, param.as_int(), _camSaturation)) {
            count_ok++;
          }
        } else if (param_name == "video.sharpness") { // Sharpness
          if (updateIntParam(param_name, param.as_int(), _camSharpness)) {
            count_ok++;
          }
        } else if (param_name == "video.gamma") { // Gamma
          if (updateIntParam(param_name, param.as_int(), _camGamma)) {
            count_ok++;
          }
        } else if (param_name ==
          "video.auto_whitebalance")           // Auto White Balance
        {
          if (updateBoolParam(param_name, param.as_bool(), _camAutoWB)) {
            count_ok++;
          }
        } else if (param_name ==
          "video.whitebalance_temperature")           // White Balance Temp
        {
          if (updateIntParam(param_name, param.as_int(), _camWBTemp)) {
            count_ok++;

            // Force disable auto white balance
            updateBoolParam("video.auto_whitebalance", false, _camAutoWB);
          }
        } else if (param_name == "video.auto_exposure") { // Auto Exposure
          if (updateBoolParam(param_name, param.as_bool(), _camAutoExposure)) {
            count_ok++;
          }
        } else if (param_name ==
          "video.exposure_compensation")           // Exposure Compensation
        {
          if (updateIntParam(param_name, param.as_int(), _camExposureComp)) {
            count_ok++;
          }
        } else if (param_name == "video.exposure_time") { // Exposure Time
          if (updateIntParam(param_name, param.as_int(), _camExpTime)) {
            count_ok++;

            // Force disable auto exposure
            updateBoolParam("video.auto_exposure", false, _camAutoExposure);

            // Force exposure range min and max values
            updateIntParam(
              "video.auto_exposure_time_range_min", _camExpTime,
              _camAutoExpTimeRangeMin);
            updateIntParam(
              "video.auto_exposure_time_range_max", _camExpTime,
              _camAutoExpTimeRangeMax);
          }
        } else if (param_name == "video.auto_exposure_time_range_min") { // Auto Exp Time Min
          if (updateIntParam(param_name, param.as_int(), _camAutoExpTimeRangeMin)) {
            count_ok++;

            // Force exposure
            if (_camAutoExpTimeRangeMin != _camAutoExpTimeRangeMax) {
              updateBoolParam("video.auto_exposure", true, _camAutoExposure);
            } else {
              updateBoolParam("video.auto_exposure", false, _camAutoExposure);
              updateIntParam(
                "video.exposure_time", _camAutoExpTimeRangeMin,
                _camExpTime);
            }
          }
        } else if (param_name == "video.auto_exposure_time_range_max") { // Auto Exp Time Max
          if (updateIntParam(param_name, param.as_int(), _camAutoExpTimeRangeMax)) {
            count_ok++;

            // Force exposure
            if (_camAutoExpTimeRangeMin != _camAutoExpTimeRangeMax) {
              updateBoolParam("video.auto_exposure", true, _camAutoExposure);
            } else {
              updateBoolParam("video.auto_exposure", false, _camAutoExposure);
              updateIntParam(
                "video.exposure_time", _camAutoExpTimeRangeMax,
                _camExpTime);
            }
          }
        } else if (param_name == "video.auto_analog_gain") { // Auto Analog Gain
          if (updateBoolParam(param_name, param.as_bool(), _camAutoAnalogGain)) {
            count_ok++;
          }
        } else if (param_name == "video.analog_gain") { // Analog Gain
          if (updateIntParam(param_name, param.as_int(), _camAnalogGain)) {
            count_ok++;

            // Force disable auto analog gain
            updateBoolParam("video.auto_analog_gain", false, _camAutoAnalogGain);

            // Force analog gain range min and max values
            updateIntParam(
              "video.auto_analog_gain_range_min", _camAnalogGain,
              _camAutoAnalogGainRangeMin);
            updateIntParam(
              "video.auto_analog_gain_range_max", _camAnalogGain,
              _camAutoAnalogGainRangeMax);
          }
        } else if (param_name == "video.auto_analog_gain_range_min") { // Auto Analog Gain Min
          if (updateIntParam(param_name, param.as_int(), _camAutoAnalogGainRangeMin)) {
            count_ok++;

            // Force analog gain
            if (_camAutoAnalogGainRangeMin != _camAutoAnalogGainRangeMax) {
              updateBoolParam("video.auto_analog_gain", true, _camAutoAnalogGain);
            } else {
              updateBoolParam("video.auto_analog_gain", false, _camAutoAnalogGain);
              updateIntParam(
                "video.analog_gain", _camAutoAnalogGainRangeMin,
                _camAnalogGain);
            }
          }
        } else if (param_name == "video.auto_analog_gain_range_max") { // Auto Analog Gain Max
          if (updateIntParam(param_name, param.as_int(), _camAutoAnalogGainRangeMax)) {
            count_ok++;

            // Force analog gain
            if (_camAutoAnalogGainRangeMin != _camAutoAnalogGainRangeMax) {
              updateBoolParam("video.auto_analog_gain", true, _camAutoAnalogGain);
            } else {
              updateBoolParam("video.auto_analog_gain", false, _camAutoAnalogGain);
              updateIntParam(
                "video.analog_gain", _camAutoAnalogGainRangeMax,
                _camAnalogGain);
            }
          }
        } else if (param_name == "video.auto_digital_gain") { // Auto Digital Gain
          if (updateBoolParam(param_name, param.as_bool(), _camAutoDigitalGain)) {
            count_ok++;
          }
        } else if (param_name == "video.digital_gain") { // Digital Gain
          if (updateIntParam(param_name, param.as_int(), _camDigitalGain)) {
            count_ok++;

            // Force disable auto digital gain
            updateBoolParam("video.auto_digital_gain", false, _camAutoDigitalGain);

            // Force digital gain range min and max values
            updateIntParam(
              "video.auto_digital_gain_range_min", _camDigitalGain,
              _camAutoDigitalGainRangeMin);
            updateIntParam(
              "video.auto_digital_gain_range_max", _camDigitalGain,
              _camAutoDigitalGainRangeMax);
          }
        } else if (param_name == "video.auto_digital_gain_range_min") { // Auto Digital Gain Min
          if (updateIntParam(param_name, param.as_int(), _camAutoDigitalGainRangeMin)) {
            count_ok++;

            // Force digital gain
            if (_camAutoDigitalGainRangeMin != _camAutoDigitalGainRangeMax) {
              updateBoolParam("video.auto_digital_gain", true, _camAutoDigitalGain);
            } else {
              updateBoolParam("video.auto_digital_gain", false, _camAutoDigitalGain);
              updateIntParam(
                "video.digital_gain", _camAutoDigitalGainRangeMin,
                _camDigitalGain);
            }
          }
        } else if (param_name == "video.auto_digital_gain_range_max") { // Auto Digital Gain Max
          if (updateIntParam(param_name, param.as_int(), _camAutoDigitalGainRangeMax)) {
            count_ok++;

            // Force digital gain
            if (_camAutoDigitalGainRangeMin != _camAutoDigitalGainRangeMax) {
              updateBoolParam("video.auto_digital_gain", true, _camAutoDigitalGain);
            } else {
              updateBoolParam("video.auto_digital_gain", false, _camAutoDigitalGain);
              updateIntParam(
                "video.digital_gain", _camAutoDigitalGainRangeMax,
                _camDigitalGain);
            }
          }
        } else {
          result.successful = false;
          result.reason = "Parameter " + param_name + " not mapped";
          DEBUG_COMM(result.reason.c_str());
          break;
        }
        result.successful = true;
      } else {
        result.successful = false;
        result.reason = "Parameter " + param_name + " not recognized";
        DEBUG_COMM(result.reason.c_str());
        break;
      }
    }
  }

  DEBUG_STREAM_COMM("Updated parameters: " << count_ok << "/" << parameters.size());

  _triggerUpdateDynParams = (count_ok > 0);

  if (count_ok > 0) {
    if (_debugCommon) {
      DEBUG_COMM("Settings to be applied: ");
      for (auto & param: _camDynParMapChanged) {
        if (param.second) {DEBUG_STREAM_COMM(" * " << param.first);}
      }
    }
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
#if ENABLE_GRAY_IMAGE
  std::string gray_prefix = "gray/";
#endif
  std::string image_topic = "image";


  _imgTopic = _topicRoot + color_prefix + rect_prefix + image_topic;
  _pubColorImg = image_transport::create_camera_publisher(
    this, _imgTopic, _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubColorImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubColorImg.getInfoTopic());

  _imgRawTopic = _topicRoot + color_prefix + raw_prefix + image_topic;
  _pubColorRawImg = image_transport::create_camera_publisher(
    this, _imgRawTopic, _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubColorRawImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubColorRawImg.getInfoTopic());

#if ENABLE_GRAY_IMAGE
  _imgGrayTopic = _topicRoot + gray_prefix + rect_prefix + image_topic;
  _pubGrayImg = image_transport::create_camera_publisher(
    this, _imgGrayTopic, _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubGrayImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubGrayImg.getInfoTopic());

  _imgRawGrayTopic = _topicRoot + gray_prefix + raw_prefix + image_topic;
  _pubGrayRawImg = image_transport::create_camera_publisher(
    this, _imgRawGrayTopic, _qos.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubGrayRawImg.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubGrayRawImg.getInfoTopic());
#endif
  // <---- Images

  // ----> Sensors
  RCLCPP_INFO(get_logger(), " +++ SENSOR TOPICS +++");
  _tempTopic = _topicRoot + "temperature";
  _pubTemp = create_publisher<sensor_msgs::msg::Temperature>(_tempTopic, _qos, _pubOpt);
  RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubTemp->get_topic_name());

  std::string imuTopicRoot = "imu";
  std::string imu_topic_name = "data";
  std::string imu_topic_raw_name = "data_raw";
  std::string imu_topic = _topicRoot + imuTopicRoot + "/" + imu_topic_name;
  std::string imu_topic_raw = _topicRoot + imuTopicRoot + "/" + imu_topic_raw_name;
  _pubImu = create_publisher<sensor_msgs::msg::Imu>(imu_topic, _qos, _pubOpt);
  RCLCPP_INFO_STREAM(
    get_logger(),
    "  * Advertised on topic: " << _pubImu->get_topic_name());
  _pubImuRaw =
    create_publisher<sensor_msgs::msg::Imu>(imu_topic_raw, _qos, _pubOpt);
  RCLCPP_INFO_STREAM(
    get_logger(),
    "  * Advertised on topic: " << _pubImuRaw->get_topic_name());
  // <---- Sensors

  // ----> Camera/imu transform message
  std::string cam_imu_tr_topic = _topicRoot + "left_cam_imu_transform";
  _pubCamImuTransf = create_publisher<geometry_msgs::msg::TransformStamped>(
    cam_imu_tr_topic, _qos, _pubOpt);

  RCLCPP_INFO_STREAM(
    get_logger(), "  * Advertised on topic: "
      << _pubCamImuTransf->get_topic_name());

  sl::Orientation sl_rot = _slCamImuTransf.getOrientation();
  sl::Translation sl_tr = _slCamImuTransf.getTranslation();
  RCLCPP_INFO(
    get_logger(), "  * Camera-IMU Translation: \n %g %g %g", sl_tr.x,
    sl_tr.y, sl_tr.z);
  RCLCPP_INFO(
    get_logger(), "  * Camera-IMU Rotation:\n%s",
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
      RCLCPP_INFO_STREAM_ONCE(get_logger(), "*** " << _cameraName << " started ***");

      // Start grab timer for diagnostic
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
      if (!_svoMode && _triggerUpdateDynParams) {
        applyDynamicSettings();
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
#if ENABLE_SVO
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
          } else
#endif
          if (_grabStatus == sl::ERROR_CODE::CAMERA_REBOOTING) {
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
          if (_useSvoTimestamp) {
            _frameTimestamp = sl_tools::slTime2Ros(_zed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
          } else {
            _frameTimestamp =
              sl_tools::slTime2Ros(_zed->getTimestamp(sl::TIME_REFERENCE::CURRENT));
          }
        } else {
          _frameTimestamp =
            sl_tools::slTime2Ros(_zed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
        }
        //DEBUG_STREAM_COMM("Grab timestamp: " << _frameTimestamp.nanoseconds() << " nsec");
        // <---- Timestamp

        if (_streamingServerRequired && !_streamingServerRunning) {
          DEBUG_STR("Streaming server required, but not running");
          startStreamingServer();
        }
      }

#if ENABLE_SVO
      // ----> Check recording status
      _recMutex.lock();
      if (_recording) {
        _recStatus = _zed->getRecordingStatus();

        if (!_recStatus.status) {
          rclcpp::Clock steady_clock(RCL_STEADY_TIME);
          RCLCPP_WARN_THROTTLE(
            get_logger(), steady_clock, 1000.0,
            "Error saving frame to SVO");
        }
      }
      _recMutex.unlock();
      // <---- Check recording status
#endif

      // ----> Retrieve Image/Depth data if someone has subscribed to
      // Retrieve data if there are subscriber to topics
      _imageSubscribed = areImageTopicsSubscribed();
      if (_imageSubscribed) {
        DEBUG_STREAM_VD("Retrieving video data");
        retrieveImages();
        publishImages();

        _videoPublishing = true;
      } else {
        _videoPublishing = false;
      }
      // <---- Retrieve Image/Depth data if someone has subscribed to

      // Diagnostic statistics update
      double mean_elab_sec = _elabPeriodMean_sec->addValue(grabElabTimer.toc());
    } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "ParameterNotDeclaredException: " << ex.what());
      continue;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Exception: " << e.what());
      continue;
    } catch (...) {
      rcutils_reset_error();
      DEBUG_STREAM_COMM("threadFunc_zedGrab: Generic exception.");
      continue;
    }
  }
  // <---- Infinite grab thread

  _diagUpdater.broadcast(diagnostic_msgs::msg::DiagnosticStatus::STALE, "Grab thread stopped");
  _diagUpdater.force_update();

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
        rclcpp::sleep_for(
          std::chrono::milliseconds(200));    // Avoid busy-waiting
        continue;
      }

      _imuPublishing = areSensorsTopicsSubscribed();

      if (!_imuPublishing && !_publishImuTF) {
        rclcpp::sleep_for(
          std::chrono::milliseconds(200));    // Avoid busy-waiting
        continue;
      }

      if (!publishSensorsData()) {
        auto sleep_msec =
          static_cast<int>(_sensRateComp * (1000. / _sensPubRate));
        sleep_msec = std::max(1, sleep_msec);
        DEBUG_STREAM_SENS(
          "[threadFunc_pubSensorsData] Thread sleep: "
            << sleep_msec << " msec");
        rclcpp::sleep_for(
          std::chrono::milliseconds(sleep_msec));    // Avoid busy-waiting
        continue;
      }

    } catch (...) {
      rcutils_reset_error();
      DEBUG_STREAM_COMM("[threadFunc_pubSensorsData] Generic exception.");
      continue;
    }

    // ----> Check publishing frequency
    double sens_period_usec = 1e6 / _sensPubRate;
    double avg_freq = 1. / _imuPeriodMean_sec->getAvg();

    double err = std::fabs(_sensPubRate - avg_freq);

    const double COMP_P_GAIN = 0.0005;

    if (avg_freq < _sensPubRate) {
      _sensRateComp -= COMP_P_GAIN * err;
    } else if (avg_freq > _sensPubRate) {
      _sensRateComp += COMP_P_GAIN * err;
    }

    _sensRateComp = std::max(0.05, _sensRateComp);
    _sensRateComp = std::min(2.0, _sensRateComp);
    DEBUG_STREAM_SENS(
      "[threadFunc_pubSensorsData] _sensRateComp: " << _sensRateComp);
    // <---- Check publishing frequency
  }
  // <---- Infinite sensor loop

  DEBUG_STREAM_SENS("Sensors thread finished");

}

void ZedCameraOne::applyDynamicSettings()
{
  DEBUG_STREAM_COMM("*** Applying dynamic settings ***");

  if (_debugCommon) {
    DEBUG_COMM("Settings to apply: ");
    for (auto & param: _camDynParMapChanged) {
      if (param.second) {DEBUG_STREAM_COMM(" * " << param.first);}
    }
  }

  // ----> Set video settings lambda function
  auto setVideoSetting = [this](sl::VIDEO_SETTINGS setting, int value,
      const std::string & settingName) {
      if (this->_camDynParMapChanged[settingName]) {
        sl::ERROR_CODE ret_code = _zed->setCameraSettings(setting, value);
        if (ret_code != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting "
              << settingName << ": "
              << sl::toString(ret_code));
        } else {
          this->_camDynParMapChanged[settingName] = false;
          DEBUG_STREAM_COMM("Set " << settingName << " to " << value);
        }
      }
    };
  // <---- Set video settings lambda function

  // ----> Set video settings range lambda function
  auto setVideoSettingRange = [this](sl::VIDEO_SETTINGS setting, int value_min,
      int value_max, const std::string & settingName_min, const std::string & settingName_max) {
      if (this->_camDynParMapChanged[settingName_min] ||
        this->_camDynParMapChanged[settingName_max])
      {
        sl::ERROR_CODE ret_code = _zed->setCameraSettings(setting, value_min, value_max);
        if (ret_code != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting the range of ["
              << settingName_min << "," << settingName_max << ": "
              << sl::toString(ret_code));
          return false;
        } else {
          this->_camDynParMapChanged[settingName_min] = false;
          this->_camDynParMapChanged[settingName_max] = false;
          DEBUG_STREAM_COMM("Set " << settingName_min << " to " << value_min);
          DEBUG_STREAM_COMM("Set " << settingName_max << " to " << value_max);
          return true;
        }
        return false;
      }
      return true;
    };
  // <---- Set video settings range lambda function

  // Saturation
  setVideoSetting(
    sl::VIDEO_SETTINGS::SATURATION, _camSaturation, "video.saturation");
  // Sharpness
  setVideoSetting(
    sl::VIDEO_SETTINGS::SHARPNESS, _camSharpness, "video.sharpness");
  // Gamma
  setVideoSetting(
    sl::VIDEO_SETTINGS::GAMMA, _camGamma, "video.gamma");

  // ----> White Balance
  setVideoSetting(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, _camAutoWB, "video.auto_whitebalance");
  if (!_camAutoWB) {
    setVideoSetting(
      sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, _camWBTemp * 100,
      "video.whitebalance_temperature");

    // Force the WB Auto to disable
    set_parameter(rclcpp::Parameter("video.auto_whitebalance", false));
  } else {
    // Report the WB temperature as to not be updated
    _camDynParMapChanged["video.whitebalance_temperature"] = false;
  }
  // <---- White Balance

  // ----> Exposure
  if (_camAutoExposure) {
    if (_camDynParMapChanged["video.auto_exposure"]) {
      // Force new
      set_parameters(
        {
          rclcpp::Parameter("video.auto_exposure_time_range_min", 28),
          rclcpp::Parameter("video.auto_exposure_time_range_max", 30000)
        });
      DEBUG_STREAM_COMM("Forced exposure range to [" << 28 << "," << 30000 << "]");
    }
    DEBUG_STREAM_COMM("Set video.auto_exposure to " << (_camAutoExposure ? "true" : "false"));
  } else {
    setVideoSetting(
      sl::VIDEO_SETTINGS::EXPOSURE_TIME, _camExpTime,
      "video.exposure_time");
    // Force new
    set_parameters(
      {
        rclcpp::Parameter("video.auto_exposure", false),
        rclcpp::Parameter("video.auto_exposure_time_range_min", _camExpTime),
        rclcpp::Parameter("video.auto_exposure_time_range_max", _camExpTime)
      });
    DEBUG_STREAM_COMM(
      "Forced video.auto_exposure to false and exposure range to [" << _camExpTime << "," <<
        _camExpTime <<
        "]");
  }
  _camDynParMapChanged["video.auto_exposure"] = false;
  _camDynParMapChanged["video.exposure_time"] = false;

  if (setVideoSettingRange(
      sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE,
      _camAutoExpTimeRangeMin, _camAutoExpTimeRangeMax,
      "video.auto_exposure_time_range_min",
      "video.auto_exposure_time_range_max"))
  {
    if (_camAutoExpTimeRangeMin == _camAutoExpTimeRangeMax) {
      // Force new
      set_parameters(
        {
          rclcpp::Parameter("video.auto_exposure", false),
          rclcpp::Parameter("video.exposure_time", _camAutoExpTimeRangeMin)
        });
      DEBUG_STREAM_COMM(
        "Forced video.auto_exposure to false and video.exposure_time to " <<
          _camAutoExpTimeRangeMin);
    } else {
      set_parameter(rclcpp::Parameter("video.auto_exposure", true));
      DEBUG_STREAM_COMM("Forced video.auto_exposure to true");
    }
  }
  // <---- Exposure

  // ----> Analog Gain
  if (_camAutoAnalogGain) {
    if (_camDynParMapChanged["video.auto_analog_gain"]) {
      // Force new
      set_parameters(
        {
          rclcpp::Parameter("video.auto_analog_gain_range_min", 1000),
          rclcpp::Parameter("video.auto_analog_gain_range_max", 16000)
        });
      DEBUG_STREAM_COMM("Forced analog gain range to [" << 1000 << "," << 16000 << "]");
    }
    DEBUG_STREAM_COMM("Set video.auto_analog_gain to " << (_camAutoAnalogGain ? "true" : "false"));
  } else {
    setVideoSetting(
      sl::VIDEO_SETTINGS::ANALOG_GAIN, _camAnalogGain, "video.analog_gain");
    // Force new
    set_parameters(
      {
        rclcpp::Parameter("video.auto_analog_gain", false),
        rclcpp::Parameter("video.auto_analog_gain_range_min", _camAnalogGain),
        rclcpp::Parameter("video.auto_analog_gain_range_max", _camAnalogGain)
      });
    DEBUG_STREAM_COMM(
      "Forced video.auto_analog_gain to false and analog gain range to [" << _camAnalogGain <<
        "," << _camAnalogGain <<
        "]");
  }
  _camDynParMapChanged["video.auto_analog_gain"] = false;
  _camDynParMapChanged["video.analog_gain"] = false;

  if (setVideoSettingRange(
      sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE,
      _camAutoAnalogGainRangeMin, _camAutoAnalogGainRangeMax,
      "video.auto_analog_gain_range_min",
      "video.auto_analog_gain_range_max"))
  {
    if (_camAutoAnalogGainRangeMin == _camAutoAnalogGainRangeMax) {
      // Force new
      set_parameters(
        {
          rclcpp::Parameter("video.auto_analog_gain", false),
          rclcpp::Parameter("video.analog_gain", _camAutoAnalogGainRangeMin)
        });
      DEBUG_STREAM_COMM(
        "Forced video.auto_analog_gain to false and video.analog_gain to " <<
          _camAutoAnalogGainRangeMin);
    } else {
      set_parameter(rclcpp::Parameter("video.auto_analog_gain", true));
      DEBUG_STREAM_COMM("Forced video.auto_analog_gain to true");
    }
  }
  // <---- Analog Gain

  // ----> Digital Gain
  if (_camAutoDigitalGain) {
    if (_camDynParMapChanged["video.auto_digital_gain"]) {
      // Force new
      set_parameters(
        {
          rclcpp::Parameter("video.auto_digital_gain_range_min", 1),
          rclcpp::Parameter("video.auto_digital_gain_range_max", 256)
        });
      DEBUG_STREAM_COMM("Forced digital gain range to [" << 1 << "," << 256 << "]");
    }
    DEBUG_STREAM_COMM(
      "Set video.auto_digital_gain to " <<
        (_camAutoDigitalGain ? "true" : "false"));
  } else {
    setVideoSetting(
      sl::VIDEO_SETTINGS::DIGITAL_GAIN, _camDigitalGain, "video.digital_gain");
    // Force new
    set_parameters(
      {
        rclcpp::Parameter("video.auto_digital_gain", false),
        rclcpp::Parameter("video.auto_digital_gain_range_min", _camDigitalGain),
        rclcpp::Parameter("video.auto_digital_gain_range_max", _camDigitalGain)
      });
    DEBUG_STREAM_COMM(
      "Forced video.auto_digital_gain to false and digital gain range to [" << _camDigitalGain <<
        "," << _camDigitalGain <<
        "]");
  }
  _camDynParMapChanged["video.auto_digital_gain"] = false;
  _camDynParMapChanged["video.digital_gain"] = false;

  if (setVideoSettingRange(
      sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE,
      _camAutoDigitalGainRangeMin, _camAutoDigitalGainRangeMax,
      "video.auto_digital_gain_range_min",
      "video.auto_digital_gain_range_max"))
  {
    if (_camAutoDigitalGainRangeMin == _camAutoDigitalGainRangeMax) {
      // Force new
      set_parameters(
        {
          rclcpp::Parameter("video.auto_digital_gain", false),
          rclcpp::Parameter("video.digital_gain", _camAutoDigitalGainRangeMin)
        });
      DEBUG_STREAM_COMM(
        "Forced video.auto_digital_gain to false and video.digital_gain to " <<
          _camAutoDigitalGainRangeMin);
    } else {
      set_parameter(rclcpp::Parameter("video.auto_digital_gain", true));
      DEBUG_STREAM_COMM("Forced video.auto_digital_gain to true");
    }
  }
  // <---- Digital Gain

  // Exposure compensation
  setVideoSetting(
    sl::VIDEO_SETTINGS::EXPOSURE_COMPENSATION, _camExposureComp,
    "video.exposure_compensation");

  setVideoSetting(sl::VIDEO_SETTINGS::DENOISING, _camDenoising, "video.denoising");

  DEBUG_COMM("Settings yet to apply: ");
  int count = 0;
  for (auto & param: _camDynParMapChanged) {
    if (param.second) {
      count++;
      DEBUG_STREAM_COMM(" * " << param.first);
    }
  }
  if (count == 0) {
    DEBUG_COMM(" * NONE");
  }

  _triggerUpdateDynParams = (count > 0);

  DEBUG_STREAM_COMM("*** Dynamic settings applied ***");
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

bool ZedCameraOne::publishSensorsData()
{
  if (_grabStatus != sl::ERROR_CODE::SUCCESS) {
    DEBUG_SENS("Camera not ready");
    return false;
  }

  // ----> Grab data and setup timestamps
  DEBUG_STREAM_SENS("[publishSensorsData] Grab data and setup timestamps");
  rclcpp::Time ts_imu;

  rclcpp::Time now = get_clock()->now();

  sl::SensorsData sens_data;

  sl::ERROR_CODE err =
    _zed->getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT);
  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(), "[publishSensorsData] sl::getSensorsData error: "
        << sl::toString(err).c_str());
    return false;
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
    return false;
  }
  // <---- Check for not updated data

  DEBUG_STREAM_SENS(
    "SENSOR LAST PERIOD: " << dT << " sec @" << 1. / dT
                           << " Hz");

  // ----> Sensors freq for diagnostic
  auto elapsed = _imuFreqTimer.toc();
  _imuFreqTimer.tic();
  double mean = _imuPeriodMean_sec->addValue(elapsed);
  _pubImu_sec->addValue(mean);
  DEBUG_STREAM_SENS("IMU MEAN freq: " << 1. / mean);
  // <---- Sensors freq for diagnostic

  // ----> Sensors data publishing
  publishImuFrameAndTopic();

  if (_imuSubCount > 0) {
    DEBUG_STREAM_SENS(
      "[publishSensorsData] IMU subscribers: "
        << static_cast<int>(_imuSubCount));

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

  if (_imuRawSubCount > 0) {
    DEBUG_STREAM_SENS(
      "[publishSensorsData] IMU subscribers: "
        << static_cast<int>(_imuRawSubCount));

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

  return true;
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

bool ZedCameraOne::areImageTopicsSubscribed()
{
  _colorSubCount = 0;
  _colorRawSubCount = 0;
#if ENABLE_GRAY_IMAGE
  _graySubCount = 0;
  _grayRawSubCount = 0;
#endif

  try {
    _colorSubCount = _pubColorImg.getNumSubscribers();
    _colorRawSubCount = _pubColorRawImg.getNumSubscribers();
#if ENABLE_GRAY_IMAGE
    _graySubCount = _pubGrayImg.getNumSubscribers();
    _grayRawSubCount = _pubGrayRawImg.getNumSubscribers();
#endif
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_VD("publishImages: Exception while counting subscribers");
    return false;
  }

  return (_colorSubCount + _colorRawSubCount
#if ENABLE_GRAY_IMAGE
         + _graySubCount + _grayRawSubCount
#endif
  ) > 0;
}

bool ZedCameraOne::areSensorsTopicsSubscribed()
{
  try {
    _imuSubCount = _pubImu->get_subscription_count();
    _imuRawSubCount = _pubImuRaw->get_subscription_count();
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_SENS(
      "areSensorsTopicsSubscribed: Exception while counting subscribers");
    return false;
  }

  DEBUG_STREAM_SENS(
    "[areSensorsTopicsSubscribed] IMU subscribers: " << _imuSubCount);
  DEBUG_STREAM_SENS(
    "[areSensorsTopicsSubscribed] IMU RAW subscribers: " << _imuRawSubCount);

  return (_imuSubCount + _imuRawSubCount) > 0;
}

void ZedCameraOne::retrieveImages()
{
  bool retrieved = false;

  // ----> Retrieve all required data
  DEBUG_VD("Retrieving Image Data");
  if (_colorSubCount > 0) {
    retrieved |=
      (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(_matColor, sl::VIEW::LEFT, sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matColor.timestamp;
    DEBUG_STREAM_VD(
      "Color image " << _matResol.width << "x" << _matResol.height << " retrieved - timestamp: " <<
        _sdkGrabTS.getNanoseconds() <<
        " nsec");
  }
  if (_colorRawSubCount > 0) {
    retrieved |= (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(
        _matColorRaw, sl::VIEW::LEFT_UNRECTIFIED,
        sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matColorRaw.timestamp;
    DEBUG_STREAM_VD(
      "Color raw image " << _matResol.width << "x" << _matResol.height <<
        " retrieved - timestamp: " << _sdkGrabTS.getNanoseconds() <<
        " nsec");
  }
#if ENABLE_GRAY_IMAGE
  if (_graySubCount > 0) {
    retrieved |= (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(
        _matGray, sl::VIEW::LEFT_GRAY,
        sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matGray.timestamp;
    DEBUG_STREAM_VD(
      "Gray image " << _matResol.width << "x" << _matResol.height << " retrieved - timestamp: " <<
        _sdkGrabTS.getNanoseconds() <<
        " nsec");
  }
  if (_grayRawSubCount > 0) {
    retrieved |=
      (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(
        _matGrayRaw, sl::VIEW::LEFT_UNRECTIFIED_GRAY,
        sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matGrayRaw.timestamp;
    DEBUG_STREAM_VD(
      "Gray raw image " << _matResol.width << "x" << _matResol.height <<
        " retrieved - timestamp: " << _sdkGrabTS.getNanoseconds() <<
        " nsec");
  }
#endif
  if (retrieved) {
    DEBUG_VD("Image Data retrieved");
  }
  // <---- Retrieve all required data
}

void ZedCameraOne::publishImages()
{
  DEBUG_VD("*** Publish Image topics *** ");
  sl_tools::StopWatch vdElabTimer(get_clock());

  // Start processing timer for diagnostic
  vdElabTimer.tic();

  // ----> Check if a grab has been done before publishing the same images
  if (_sdkGrabTS.getNanoseconds() == _lastTs_grab.getNanoseconds()) {
    // Data not updated by a grab calling in the grab thread
    DEBUG_VD("publishImages: ignoring not update data");
    DEBUG_STREAM_VD(
      "Latest Ts: " << _lastTs_grab.getNanoseconds()
                    << " - New Ts: "
                    << _sdkGrabTS.getNanoseconds());
    return;
  }

  if (_lastTs_grab.data_ns != 0) {
    double period_sec =
      static_cast<double>(_sdkGrabTS.data_ns - _lastTs_grab.data_ns) / 1e9;
    DEBUG_STREAM_VD(
      "IMAGE PUB LAST PERIOD: " << period_sec << " sec @"
                                << 1. / period_sec << " Hz");

    _imagePeriodMean_sec->addValue(period_sec);
    DEBUG_STREAM_VD(
      "IMAGE PUB MEAN PERIOD: "
        << _imagePeriodMean_sec->getAvg() << " sec @"
        << 1. / _imagePeriodMean_sec->getAvg() << " Hz");
  }
  _lastTs_grab = _sdkGrabTS;
  // <---- Check if a grab has been done before publishing the same images

  // ----> Timestamp
  rclcpp::Time timeStamp;
  if (_svoMode) {
    timeStamp = _frameTimestamp;
  } else {
    timeStamp = sl_tools::slTime2Ros(_sdkGrabTS, get_clock()->get_clock_type());
  }
  // <---- Timestamp

  // ----> Publish the COLOR image if someone has subscribed to
  if (_colorSubCount > 0) {
    DEBUG_STREAM_VD("_colorSubCount: " << _colorSubCount);
    publishImageWithInfo(
      _matColor, _pubColorImg, _camInfoMsg, _camImgFrameId,
      timeStamp);
  }
  // <---- Publish the COLOR image if someone has subscribed to

  // ----> Publish the COLOR RAW image if someone has subscribed to
  if (_colorRawSubCount > 0) {
    DEBUG_STREAM_VD("_colorRawSubCount: " << _colorSubCount);
    publishImageWithInfo(
      _matColorRaw, _pubColorRawImg, _camInfoRawMsg,
      _camImgFrameId, timeStamp);
  }
  // <---- Publish the COLOR RAW image if someone has subscribed to

#if ENABLE_GRAY_IMAGE
  // ----> Publish the GRAY image if someone has subscribed to
  if (_graySubCount > 0) {
    DEBUG_STREAM_VD("_graySubCount: " << _graySubCount);
    publishImageWithInfo(
      _matGray, _pubGrayImg, _camInfoMsg, _camImgFrameId,
      timeStamp);
  }
  // <---- Publish the GRAY image if someone has subscribed to

  // ----> Publish the GRAY RAW image if someone has subscribed to
  if (_grayRawSubCount > 0) {
    DEBUG_STREAM_VD("_grayRawSubCount: " << _grayRawSubCount);
    publishImageWithInfo(
      _matGrayRaw, _pubGrayRawImg, _camInfoRawMsg,
      _camImgFrameId, timeStamp);
  }
  // <---- Publish the GRAY RAW image if someone has subscribed to
#endif

  // Diagnostic statistic
  _imageElabMean_sec->addValue(vdElabTimer.toc());

  // ----> Check publishing frequency
  double vd_period_usec = 1e6 / _camGrabFrameRate;

  double elapsed_usec = _imgPubFreqTimer.toc() * 1e6;

  if (elapsed_usec < vd_period_usec) {
    rclcpp::sleep_for(
      std::chrono::microseconds(
        static_cast<int>(vd_period_usec - elapsed_usec)));
  }

  _imgPubFreqTimer.tic();
  // <---- Check publishing frequency

  DEBUG_VD("*** Video and Depth topics published *** ");
}

void ZedCameraOne::publishImageWithInfo(
  const sl::Mat & img, const image_transport::CameraPublisher & pubImg,
  const camInfoMsgPtr & camInfoMsg, const std::string & imgFrameId,
  const rclcpp::Time & t)
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

void ZedCameraOne::callback_enableStreaming(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
  std::shared_ptr<std_srvs::srv::SetBool_Response> res)
{
  (void)request_header;

  if (req->data) {
    if (_streamingServerRunning) {
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
        "Error occurred starting the Streaming Server. Read the log for more "
        "info";
      res->success = false;
      return;
    }
  } else {
    // Stop
    if (!_streamingServerRunning) {
      RCLCPP_WARN(
        get_logger(),
        "There is no Streaming Server active to be stopped");
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

#if ENABLE_SVO
void ZedCameraOne::callback_startSvoRec(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<zed_msgs::srv::StartSvoRec_Request> req,
  std::shared_ptr<zed_msgs::srv::StartSvoRec_Response> res)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "** Start SVO Recording service called **");

  if (_svoMode) {
    RCLCPP_WARN(
      get_logger(),
      "Cannot start SVO recording while playing SVO as input");
    res->message = "Cannot start SVO recording while playing SVO as input";
    res->success = false;
    return;
  }

  std::lock_guard<std::mutex> lock(_recMutex);

  if (_recording) {
    RCLCPP_WARN(get_logger(), "SVO Recording is already enabled");
    res->message = "SVO Recording is already enabled";
    res->success = false;
    return;
  }

  _svoRecBitrate = req->bitrate;
  if ((_svoRecBitrate != 0) &&
    ((_svoRecBitrate < 1000) || (_svoRecBitrate > 60000)))
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
  _svoRecCompr = static_cast<sl::SVO_COMPRESSION_MODE>(req->compression_mode);
  if (_svoRecCompr >= sl::SVO_COMPRESSION_MODE::LAST) {
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
  _svoRecFramerate = req->target_framerate;
  _svoRecTranscode = req->input_transcode;
  _svoRecFilename = req->svo_filename;

  if (_svoRecFilename.empty()) {
    _svoRecFilename = "zed.svo";
  }

  std::string err;

  if (!startSvoRecording(err)) {
    res->message = "Error starting SVO recording: " + err;
    res->success = false;
    return;
  }

  RCLCPP_INFO(get_logger(), "SVO Recording started: ");
  RCLCPP_INFO_STREAM(get_logger(), " * Bitrate: " << _svoRecBitrate);
  RCLCPP_INFO_STREAM(get_logger(), " * Compression: " << _svoRecCompr);
  RCLCPP_INFO_STREAM(get_logger(), " * Framerate: " << _svoRecFramerate);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Input Transcode: " << (_svoRecTranscode ? "TRUE" : "FALSE"));
  RCLCPP_INFO_STREAM(
    get_logger(), " * Filename: " << (_svoRecFilename.empty() ?
    "zed.svo" :
    _svoRecFilename));

  res->message = "SVO Recording started";
  res->success = true;
}

void ZedCameraOne::callback_stopSvoRec(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
  std::shared_ptr<std_srvs::srv::Trigger_Response> res)
{
  (void)request_header;
  (void)req;

  RCLCPP_INFO(get_logger(), "** Stop SVO Recording service called **");

  std::lock_guard<std::mutex> lock(_recMutex);

  if (!_recording) {
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

void ZedCameraOne::callback_pauseSvoInput(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
  std::shared_ptr<std_srvs::srv::Trigger_Response> res)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "** Pause SVO Input service called **");

  std::lock_guard<std::mutex> lock(_recMutex);

  if (!_svoMode) {
    RCLCPP_WARN(get_logger(), "The node is not using an SVO as input");
    res->message = "The node is not using an SVO as inpu";
    res->success = false;
    return;
  }

  if (_svoRealtime) {
    RCLCPP_WARN(
      get_logger(),
      "SVO input can be paused only if SVO is not in RealTime mode");
    res->message =
      "SVO input can be paused only if SVO is not in RealTime mode";
    res->success = false;
    _svoPause = false;
    return;
  }

  if (!_svoPause) {
    RCLCPP_WARN(get_logger(), "SVO is paused");
    res->message = "SVO is paused";
    _svoPause = true;
  } else {
    RCLCPP_WARN(get_logger(), "SVO is playing");
    res->message = "SVO is playing";
    _svoPause = false;
  }
  res->success = true;
}

bool ZedCameraOne::startSvoRecording(std::string & errMsg)
{
  sl::RecordingParameters params;

  params.bitrate = _svoRecBitrate;
  params.compression_mode = _svoRecCompr;
  params.target_framerate = _svoRecFramerate;
  params.transcode_streaming_input = _svoRecTranscode;
  params.video_filename = _svoRecFilename.c_str();

  sl::ERROR_CODE err = _zed->enableRecording(params);
  errMsg = sl::toString(err);

  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Error starting SVO recording: " << errMsg);
    return false;
  }

  _recording = true;

  return true;
}

void ZedCameraOne::stopSvoRecording()
{
  if (_recording) {
    _recording = false;
    _zed->disableRecording();
  }
}
#endif

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCameraOne)
