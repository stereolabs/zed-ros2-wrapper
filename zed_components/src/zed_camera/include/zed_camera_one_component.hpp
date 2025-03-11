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

#ifndef ZED_CAMERA_ONE_COMPONENT_HPP_
#define ZED_CAMERA_ONE_COMPONENT_HPP_

#define ENABLE_GRAY_IMAGE 1
#define ENABLE_STREAM_INPUT 1
#define ENABLE_SVO 0

#include <atomic>
#include <sl/CameraOne.hpp>

#include "sl_tools.hpp"
#include "sl_types.hpp"
#include "visibility_control.hpp"


namespace stereolabs
{

class ZedCameraOne : public rclcpp::Node
{
public:
  ZED_COMPONENTS_PUBLIC
  explicit ZedCameraOne(const rclcpp::NodeOptions & options);

  virtual ~ZedCameraOne();

protected:
  // ----> Initialization functions
  void init();
  void initParameters();
  void initServices();
  void initThreadsAndTimers();
  void initTFCoordFrameNames();
  void initPublishers();

  void getSensorsParams();
  void getDebugParams();
  void getVideoParams();
  void getGeneralParams();
  void getStreamingServerParams();
  void getAdvancedParams();

  bool startCamera();
  void startTempPubTimer();
  bool startStreamingServer();
  void stopStreamingServer();
#if ENABLE_SVO
  bool startSvoRecording(std::string & errMsg);
  void stopSvoRecording();
#endif
  // <---- Initialization functions

  // ----> Utility functions
  template<typename T>
  void getParam(
    std::string paramName, T defValue, T & outVal,
    std::string log_info = std::string(), bool dynamic = false,
    T min = std::numeric_limits<T>::min(), T max = std::numeric_limits<T>::max());

  void fillCamInfo(
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> & camInfoMsg,
    const std::string & frameId,
    bool rawParam = false);

  void applyDynamicSettings();
  bool areImageTopicsSubscribed();
  bool areSensorsTopicsSubscribed();
  void retrieveImages();
  void publishImages();
  void publishImageWithInfo(
    const sl::Mat & img,
    const image_transport::CameraPublisher & pubImg,
    const camInfoMsgPtr & camInfoMsg, const std::string & imgFrameId,
    const rclcpp::Time & t);
  bool publishSensorsData();
  void publishImuFrameAndTopic();
  // <---- Utility functions

  // ----> Callbacks functions
  rcl_interfaces::msg::SetParametersResult callback_setParameters(
    std::vector<rclcpp::Parameter> parameters);
  void callback_updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void callback_pubTemp();

  void callback_enableStreaming(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
    std::shared_ptr<std_srvs::srv::SetBool_Response> res);
#if ENABLE_SVO
  void callback_startSvoRec(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_msgs::srv::StartSvoRec_Request> req,
    std::shared_ptr<zed_msgs::srv::StartSvoRec_Response> res);
  void callback_stopSvoRec(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
  void callback_pauseSvoInput(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
#endif
  // <---- Callbacks functions

  // ----> Thread functions
  void threadFunc_zedGrab();
  void threadFunc_pubSensorsData();
  // <---- Threads functions

private:
  // ----> ZED SDK
  std::shared_ptr<sl::CameraOne> _zed;
  sl::InitParametersOne _initParams;
  // <---- ZED SDK

  // ----> Threads and Timers
  std::thread _grabThread;        // Main grab thread
  std::thread _sensThread;        // Sensors data publish thread

  std::atomic<bool> _threadStop;
  rclcpp::TimerBase::SharedPtr _initTimer;
  rclcpp::TimerBase::SharedPtr _tempPubTimer;    // Timer to retrieve and publish camera temperature
  // <---- Threads and Timers

  // ----> Thread Sync
  std::mutex _recMutex;
  // <---- Thread Sync

  // ----> Debug variables
  bool _debugCommon = false;
  bool _debugVideoDepth = false;
  bool _debugSensors = false;
  bool _debugCamCtrl = false;
  bool _debugStreaming = false;
  bool _debugAdvanced = false;
  // <---- Debug variables

  // ----> QoS
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
  rclcpp::QoS _qos;
  rclcpp::PublisherOptions _pubOpt;
  rclcpp::SubscriptionOptions _subOpt;
  // <---- QoS

  // ----> Topics
  std::string _topicRoot = "~/";
  std::string _imgTopic;
  std::string _imgRawTopic;
#if ENABLE_GRAY_IMAGE
  std::string _imgGrayTopic;
  std::string _imgRawGrayTopic;
#endif

  std::string _tempTopic;
  // <---- Topics

  // ----> Publishers
  image_transport::CameraPublisher _pubColorImg;
  image_transport::CameraPublisher _pubColorRawImg;
#if ENABLE_GRAY_IMAGE
  image_transport::CameraPublisher _pubGrayImg;
  image_transport::CameraPublisher _pubGrayRawImg;
#endif

  imuPub _pubImu;
  imuPub _pubImuRaw;
  tempPub _pubTemp;

  transfPub _pubCamImuTransf;
  // <---- Publishers

  // ----> Publisher variables
  sl::Timestamp _lastTs_grab = 0;  // Used to calculate stable publish frequency
  sl::Timestamp _sdkGrabTS = 0;
  std::atomic<size_t> _colorSubCount;
  std::atomic<size_t> _colorRawSubCount;
#if ENABLE_GRAY_IMAGE
  std::atomic<size_t> _graySubCount = 0;
  std::atomic<size_t> _grayRawSubCount = 0;
#endif

  std::atomic<size_t> _imuSubCount;
  std::atomic<size_t> _imuRawSubCount;
  double _sensRateComp = 1.0;

  sl::Mat _matColor, _matColorRaw;
#if ENABLE_GRAY_IMAGE
  sl::Mat _matGray, _matGrayRaw;
#endif
  // <---- Publisher variables

  // ----> Parameters
  std::string _cameraName = "zed_one";  // Name of the camera
  int _camGrabFrameRate = 30; // Grab frame rate
  sl::RESOLUTION _camResol = sl::RESOLUTION::HD1080; // Default resolution: RESOLUTION_HD1080
  PubRes _pubResolution = PubRes::NATIVE; // Use native grab resolution by default
  double _customDownscaleFactor = 1.0;  // Used to rescale data with user factor
  bool _cameraFlip = false; // Camera flipped?
  bool _enableHDR = false; // Enable HDR if supported?
  std::string _opencvCalibFile; // Custom OpenCV calibration file
  int _sdkVerbose = 0; // SDK verbose level
  int _gpuId = -1; // GPU ID

  int _camSerialNumber = 0; // Camera serial number
  int _camId = -1; // Camera ID

  sl::MODEL _camUserModel = sl::MODEL::ZED_XONE_GS;  // Default camera model

  std::string _svoFilepath = ""; // SVO input
#if ENABLE_SVO
  bool _svoRealtime = true; // SVO playback with real time
  bool _svoLoop = false; // SVO loop playback
#endif

  std::string _streamAddr = ""; // Address for local streaming input
  int _streamPort = 10000;

  std::string _threadSchedPolicy;
  int _threadPrioGrab;
  int _threadPrioSens;

  std::atomic<bool> _streamingServerRequired;
  sl::STREAMING_CODEC _streamingServerCodec = sl::STREAMING_CODEC::H264;
  int _streamingServerPort = 30000;
  int _streamingServerBitrate = 12500;
  int _streamingServerGopSize = -1;
  bool _streamingServerAdaptiveBitrate = false;
  int _streamingServerChunckSize = 16084;
  int _streamingServerTargetFramerate = 0;

  bool _publishImuTF = false;
  double _sensPubRate = 200.;
  // <---- Parameters

  // ----> Dynamic params
  OnSetParametersCallbackHandle::SharedPtr _paramChangeCallbackHandle;

  int _camSaturation = 4;
  int _camSharpness = 4;
  int _camGamma = 8;
  bool _camAutoWB = true;
  int _camWBTemp = 42;

  bool _camAutoExposure = true;
  int _camExpTime = 16666;
  int _camAutoExpTimeRangeMin = 28;
  int _camAutoExpTimeRangeMax = 30000;
  int _camExposureComp = 50;
  bool _camAutoAnalogGain = true;
  int _camAnalogGain = 8000;
  int _camAutoAnalogGainRangeMin = 1000;
  int _camAutoAnalogGainRangeMax = 16000;
  bool _camAutoDigitalGain = true;
  int _camDigitalGain = 128;
  int _camAutoDigitalGainRangeMin = 1;
  int _camAutoDigitalGainRangeMax = 256;
  int _camDenoising = 50;
  std::unordered_map<std::string, bool> _camDynParMapChanged;
  // <---- Dynamic params

  // ----> Running status
  bool _debugMode = false;  // Debug mode active?
  bool _svoMode = false;        // Input from SVO?
  bool _svoPause = false;       // SVO pause status
  bool _useSvoTimestamp = false; // Use SVO timestamp
  bool _streamMode = false;     // Expecting local streaming data?

  std::atomic<bool> _triggerUpdateDynParams;  // Trigger auto exposure/gain

  bool _recording = false;
  sl::RecordingStatus _recStatus = sl::RecordingStatus();
  // <---- Running status

  // ----> Timestamps
  rclcpp::Time _frameTimestamp;
  rclcpp::Time _lastTs_imu;
  // <---- Timestamps

  // ----> TF handling
  std::unique_ptr<tf2_ros::Buffer> _tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> _tfListener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tfBroadcaster;

  // Camera IMU transform
  sl::Transform _slCamImuTransf;
  // <---- TF handling

  // ----> Camera info
  sl::MODEL _camRealModel;                   // Camera model requested to SDK
  unsigned int _camFwVersion;                // Camera FW version
  unsigned int _sensFwVersion;               // Sensors FW version
  // <---- Camera info

  // ----> Stereolabs Mat Info
  int _camWidth;   // Camera frame width
  int _camHeight;  // Camera frame height
  sl::Resolution _matResol;
  // <---- Stereolabs Mat Info

  // ----> Camera infos
  camInfoMsgPtr _camInfoMsg;
  camInfoMsgPtr _camInfoRawMsg;
  // <---- Camera infos

  // ----> Frame IDs
  std::string _cameraLinkFrameId;
  std::string _cameraCenterFrameId;
  std::string _camImgFrameId;
  std::string _camOptFrameId;
  std::string _imuFrameId;
  // <---- Frame IDs

  // ----> Diagnostic variables
  diagnostic_updater::Updater _diagUpdater;  // Diagnostic Updater

  sl_tools::StopWatch _uptimer;

  sl::ERROR_CODE _connStatus = sl::ERROR_CODE::LAST; // Connection status
  sl::ERROR_CODE _grabStatus = sl::ERROR_CODE::LAST; // Grab status
  float _tempImu = NOT_VALID_TEMP;
  uint64_t _frameCount = 0;
  std::unique_ptr<sl_tools::WinAvg> _elabPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> _grabPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> _imagePeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> _imageElabMean_sec;
  std::unique_ptr<sl_tools::WinAvg> _imuPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> _pubImuTF_sec;
  std::unique_ptr<sl_tools::WinAvg> _pubImu_sec;
  bool _imuPublishing = false;
  bool _videoPublishing = false;
  bool _imageSubscribed = false;

  sl_tools::StopWatch _grabFreqTimer;
  sl_tools::StopWatch _imuFreqTimer;
  sl_tools::StopWatch _imuTfFreqTimer;
  sl_tools::StopWatch _imgPubFreqTimer;
  int _sysOverloadCount = 0;

  std::atomic<bool> _streamingServerRunning;
  // <---- Diagnostic variables

  // ----> SVO Recording parameters
#if ENABLE_SVO
  unsigned int _svoRecBitrate = 0;
  sl::SVO_COMPRESSION_MODE _svoRecCompr = sl::SVO_COMPRESSION_MODE::H264;
  unsigned int _svoRecFramerate = 0;
  bool _svoRecTranscode = false;
  std::string _svoRecFilename;
#endif
  // <---- SVO Recording parameters

  // ----> Services
  enableStreamingPtr _srvEnableStreaming;
#if ENABLE_SVO
  startSvoRecSrvPtr _srvStartSvoRec;
  stopSvoRecSrvPtr _srvStopSvoRec;
  pauseSvoSrvPtr _srvPauseSvo;
#endif
  // <---- Services

  // ----> Services names
  const std::string _srvEnableStreamingName = "enable_streaming";
#if ENABLE_SVO
  const std::string _srvStartSvoRecName = "start_svo_rec";
  const std::string _srvStopSvoRecName = "stop_svo_rec";
  const std::string _srvToggleSvoPauseName = "toggle_svo_pause";
#endif
  // <---- Services names
};

// ----> Template Function definitions
template<typename T>
void ZedCameraOne::getParam(
  std::string paramName, T defValue, T & outVal,
  std::string log_info, bool dynamic, T minVal, T maxVal)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = !dynamic;

  std::stringstream ss;
  if constexpr (std::is_same<T, bool>::value) {
    ss << "Default value: " << (defValue ? "TRUE" : "FALSE");
  } else {
    ss << "Default value: " << defValue;
  }
  descriptor.description = ss.str();

  if constexpr (std::is_same<T, double>::value) {
    descriptor.additional_constraints = "Range: [" + std::to_string(minVal) + ", " + std::to_string(
      maxVal) + "]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = minVal;
    range.to_value = maxVal;
    descriptor.floating_point_range.push_back(range);
  } else if constexpr (std::is_same<T, int>::value) {
    descriptor.additional_constraints = "Range: [" + std::to_string(minVal) + ", " + std::to_string(
      maxVal) + "]";
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = minVal;
    range.to_value = maxVal;
    descriptor.integer_range.push_back(range);
  }


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
    if constexpr (std::is_same<T, bool>::value) {
      RCLCPP_INFO_STREAM(get_logger(), log_info << (outVal ? "TRUE" : "FALSE"));
    } else {
      RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
    }
  }
}

}  // namespace stereolabs

#endif  // ZED_CAMERA_ONE_COMPONENT_HPP_
