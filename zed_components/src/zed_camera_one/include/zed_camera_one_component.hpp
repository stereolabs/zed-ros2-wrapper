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

#ifndef ZED_CAMERA_ONE_COMPONENT_HPP_
#define ZED_CAMERA_ONE_COMPONENT_HPP_

#define ENABLE_STREAM_INPUT 1
#define ENABLE_SVO 0

#include <atomic>
#include <sl/CameraOne.hpp>

#include "sl_version.hpp"
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
  void initNode();
  void deInitNode();

  void initParameters();
  void initServices();
  void initTFCoordFrameNames();
  void initPublishers();
  void initVideoPublishers();
  void initSensorPublishers();
  void initializeTimestamp();
  void initializeDiagnosticStatistics();
  void initThreadsAndTimers();

  void getSensorsParams();
  void getDebugParams();
  void getVideoParams();
  void getGeneralParams();
  void getTopicEnableParams();
  void getSvoParams();
  void getStreamParams();
  void getCameraModelParams();
  void getCameraInfoParams();
  void getResolutionParams();
  void getOpencvCalibrationParam();

  void getStreamingServerParams();
  void getAdvancedParams();

  bool startCamera();
  void closeCamera();
  void createZedObject();
  void logSdkVersion();
  void setupTf2();
  void configureZedInput();
  void setZedInitParams();
  bool openZedCamera();
  void processCameraInformation();
  void setupCameraInfoMessages();

  void startTempPubTimer();
  bool startStreamingServer();
  void stopStreamingServer();
#if ENABLE_SVO
  bool startSvoRecording(std::string & errMsg);
  void stopSvoRecording();
#endif
  // <---- Initialization functions

  // ----> Utility functions
  void fillCamInfo(
    sensor_msgs::msg::CameraInfo::SharedPtr camInfoMsg,
    const std::string & frameId, bool rawParam = false);

  void applyDynamicSettings();
  void applySaturationSharpnessGamma();
  void applyWhiteBalance();
  void applyExposure();
  void applyAnalogGain();
  void applyDigitalGain();
  void applyExposureCompensationAndDenoising();

  bool areImageTopicsSubscribed();
  bool areSensorsTopicsSubscribed();
  void retrieveImages(bool gpu);
  void publishImages();
  void publishColorImage(const rclcpp::Time & timeStamp);
  void publishColorRawImage(const rclcpp::Time & timeStamp);
  void publishGrayImage(const rclcpp::Time & timeStamp);
  void publishGrayRawImage(const rclcpp::Time & timeStamp);
  void publishImageWithInfo(
    const sl::Mat & img,
    const image_transport::Publisher & pubImg,
    const camInfoPub & infoPub,
    const camInfoPub & infoPubTrans,
    camInfoMsgPtr & camInfoMsg,
    const std::string & imgFrameId,
    const rclcpp::Time & t);
#ifdef FOUND_ISAAC_ROS_NITROS
  void publishImageWithInfo(
    const sl::Mat & img,
    const nitrosImgPub & nitrosPubImg,
    const camInfoPub & infoPub,
    const camInfoPub & infoPubTrans,
    camInfoMsgPtr & camInfoMsg,
    const std::string & imgFrameId,
    const rclcpp::Time & t);
#endif
  void publishCameraInfo(
    const camInfoPub & infoPub,
    camInfoMsgPtr & camInfoMsg, const rclcpp::Time & t);
  bool publishSensorsData();
  void publishImuFrameAndTopic();

  void updateImuFreqDiagnostics(double dT);
  void publishImuMsg(const rclcpp::Time & ts_imu, const sl::SensorsData & sens_data);
  void publishImuRawMsg(const rclcpp::Time & ts_imu, const sl::SensorsData & sens_data);

  void updateCaptureDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateInputModeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateImageDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateSvoDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateTfImuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateImuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateTemperatureDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateSvoRecordingDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateStreamingServerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void setupGrabThreadPolicy();
  void initializeGrabThreadStatus();
  bool checkGrabThreadInterruption();
  void handleDynamicSettings();
  void updateGrabFrequency();
  bool performCameraGrab();
  void updateFrameTimestamp();
  void handleStreamingServer();
  void handleSvoRecordingStatus();
  void handleImageRetrievalAndPublishing();

  void setupSensorThreadScheduling();
  bool handleSensorThreadInterruption();
  bool waitForCameraOpen();
  bool waitForSensorSubscribers();
  bool handleSensorPublishing();
  void adjustSensorPublishingFrequency();

  bool handleDynamicVideoParam(
    const rclcpp::Parameter & param, const std::string & param_name,
    int & count_ok, rcl_interfaces::msg::SetParametersResult & result);
  bool handleSaturationSharpnessGamma(
    const rclcpp::Parameter & param,
    const std::string & param_name, int & count_ok);
  bool handleWhiteBalance(
    const rclcpp::Parameter & param, const std::string & param_name,
    int & count_ok);
  bool handleExposure(
    const rclcpp::Parameter & param, const std::string & param_name,
    int & count_ok);
  bool handleAnalogGain(
    const rclcpp::Parameter & param, const std::string & param_name,
    int & count_ok);
  bool handleDigitalGain(
    const rclcpp::Parameter & param, const std::string & param_name,
    int & count_ok);
  // <---- Utility functions

  // ----> Callbacks functions
  rcl_interfaces::msg::SetParametersResult callback_dynamicParamChange(
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
  bool _debugNitros = false;
  // If available, force disable NITROS usage for debugging and testing
  // purposes; otherwise, this is always true.
  bool _nitrosDisabled = false;
  // <---- Debug variables

  // ----> QoS
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
  rclcpp::QoS _qos;
  rclcpp::PublisherOptions _pubOpt;
  rclcpp::SubscriptionOptions _subOpt;
  // <---- QoS

  // ----> Topics
  std::string _topicRoot = "~/";
  std::string _imgColorTopic;
  std::string _imgColorRawTopic;
  std::string _imgGrayTopic;
  std::string _imgRawGrayTopic;

  std::string _sensImuTopic;
  std::string _sensImuRawTopic;
  std::string _sensTempTopic;
  // <---- Topics

  // ----> Publishers
  // Image publishers
  image_transport::Publisher _pubColorImg;
  image_transport::Publisher _pubColorRawImg;
  image_transport::Publisher _pubGrayImg;
  image_transport::Publisher _pubGrayRawImg;

#ifdef FOUND_ISAAC_ROS_NITROS
  // Nitros image publishers with camera info
  nitrosImgPub _nitrosPubColorImg;
  nitrosImgPub _nitrosPubColorRawImg;
  nitrosImgPub _nitrosPubGrayImg;
  nitrosImgPub _nitrosPubGrayRawImg;
#endif


  // Camera Info publishers
  camInfoPub _pubColorImgInfo;
  camInfoPub _pubColorRawImgInfo;
  camInfoPub _pubGrayImgInfo;
  camInfoPub _pubGrayRawImgInfo;
  camInfoPub _pubColorImgInfoTrans; // `camera_info` topic for the transported image (compressed, theora, nitros, etc)
  camInfoPub _pubColorRawImgInfoTrans; // `camera_info` topic for the transported image (compressed, theora, nitros, etc)
  camInfoPub _pubGrayImgInfoTrans; // `camera_info` topic for the transported image (compressed, theora, nitros, etc)
  camInfoPub _pubGrayRawImgInfoTrans; // `camera_info` topic for the transported image (compressed, theora, nitros, etc)

  // Sensor publishers
  imuPub _pubImu;
  imuPub _pubImuRaw;
  tempPub _pubTemp;

  // Camera-IMU Transform publisher
  transfPub _pubCamImuTransf;
  // <---- Publishers

  // ----> Publisher variables
  sl::Timestamp _lastTs_grab = 0;  // Used to calculate stable publish frequency
  sl::Timestamp _sdkGrabTS = 0;
  std::atomic<size_t> _colorSubCount;
  std::atomic<size_t> _colorRawSubCount;
  std::atomic<size_t> _graySubCount = 0;
  std::atomic<size_t> _grayRawSubCount = 0;

  std::atomic<size_t> _imuSubCount;
  std::atomic<size_t> _imuRawSubCount;
  double _sensRateComp = 1.0;

  sl::Mat _matColor, _matColorRaw;
  sl::Mat _matGray, _matGrayRaw;
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
  std::string _sdkVerboseLogFile = ""; // SDK Verbose Log file
  int _gpuId = -1; // GPU ID
  bool _useSvoTimestamp = false; // Use SVO timestamp
  bool _usePubTimestamps = false; // Use publishing timestamp instead of grab timestamp

  int _camSerialNumber = 0; // Camera serial number
  int _camId = -1; // Camera ID

  sl::MODEL _camUserModel = sl::MODEL::ZED_XONE_GS;  // Default camera model

  //Topic enabler parameters
  bool _publishImgRgb = true;
  bool _publishImgRaw = false;
  bool _publishImgGray = false;
  bool _publishSensImu = true;
  bool _publishSensImuRaw = false;
  bool _publishSensImuTransf = false;
  bool _publishSensImuTF = false;
  bool _publishSensTemp = false;

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

}  // namespace stereolabs

#endif  // ZED_CAMERA_ONE_COMPONENT_HPP_
