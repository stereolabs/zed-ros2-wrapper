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

#ifndef ZED_CAMERA_COMPONENT_HPP_
#define ZED_CAMERA_COMPONENT_HPP_

#include <atomic>
#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>
#include <unordered_set>

#include "sl_version.hpp"
#include "sl_tools.hpp"
#include "sl_types.hpp"
#include "visibility_control.hpp"

namespace stereolabs
{

class ZedCamera : public rclcpp::Node
{
public:
  ZED_COMPONENTS_PUBLIC
  explicit ZedCamera(const rclcpp::NodeOptions & options);

  virtual ~ZedCamera();

protected:
  // ----> Initialization functions
  void initNode();
  void deInitNode();

  void initParameters();
  void initServices();
  void initThreads();

  void getDebugParams();
  void getTopicEnableParams();
  void getSimParams();
  void getGeneralParams();
  void getVideoParams();
  void getRoiParams();
  void getDepthParams();
  void getPosTrackingParams();
  void getGnssFusionParams();
  void getSensorsParams();
  void getMappingParams();
  void getOdParams();
  void getCustomOdParams();
  void getBodyTrkParams();
  void getStreamingServerParams();
  void getAdvancedParams();

  void setTFCoordFrameNames();
  void initPublishers();
  void initVideoDepthPublishers();

  void initSubscribers();

  void fillCamInfo(
    const std::shared_ptr<sl::Camera> & zed,
    const sensor_msgs::msg::CameraInfo::SharedPtr & leftCamInfoMsg,
    const sensor_msgs::msg::CameraInfo::SharedPtr & rightCamInfoMsg,
    const std::string & leftFrameId, const std::string & rightFrameId,
    bool rawParam = false);

  bool startCamera();
  bool startPosTracking();
  bool saveAreaMemoryFile(const std::string & filePath);
  bool start3dMapping();
  void stop3dMapping();
  bool startObjDetect();
  void stopObjDetect();
  bool startBodyTracking();
  void stopBodyTracking();
  bool startSvoRecording(std::string & errMsg);
  void stopSvoRecording();
  bool startStreamingServer();
  void stopStreamingServer();
  void closeCamera();
  // <---- Initialization functions

  // ----> Dynamic Parameters Handlers
  // Video/Depth
  bool handleVideoDepthDynamicParams(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result);
  bool handleGmsl2Params(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result);
  bool handleUsb3Params(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result);
  bool handleCommonVideoParams(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result);
  bool handleDepthParams(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result);
  // Object Detection
  bool handleOdDynamicParams(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result);
  bool handleCustomOdDynamicParams(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result);
  // Body Tracking
  bool handleBodyTrkDynamicParams(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result);
  // <---- Dynamic Parameters Handlers

  // ----> Callbacks
  void callback_pubFusedPc();
  void callback_pubPaths();
  void callback_pubTemp();
  void callback_pubHeartbeat();
  void callback_gnssPubTimerTimeout();
  rcl_interfaces::msg::SetParametersResult callback_dynamicParamChange(
    std::vector<rclcpp::Parameter> parameters);
  void callback_updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper & stat);

  void callback_resetOdometry(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
  void callback_resetPosTracking(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
  void callback_setPose(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_msgs::srv::SetPose_Request> req,
    std::shared_ptr<zed_msgs::srv::SetPose_Response> res);
  /*void callback_saveAreaMemory(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_msgs::srv::SaveAreaMemory_Request> req,
    std::shared_ptr<zed_msgs::srv::SaveAreaMemory_Response> res);*/// TODO(Walter): Uncomment when available in `zed_msgs` package from APT
  void callback_saveAreaMemory(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_msgs::srv::SetROI_Request> req,
    std::shared_ptr<zed_msgs::srv::SetROI_Response> res);
  void callback_enableObjDet(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
    std::shared_ptr<std_srvs::srv::SetBool_Response> res);
  void callback_enableBodyTrk(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
    std::shared_ptr<std_srvs::srv::SetBool_Response> res);
  void callback_enableMapping(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
    std::shared_ptr<std_srvs::srv::SetBool_Response> res);
  void callback_enableStreaming(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
    std::shared_ptr<std_srvs::srv::SetBool_Response> res);
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
  void callback_setSvoFrame(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_msgs::srv::SetSvoFrame_Request> req,
    std::shared_ptr<zed_msgs::srv::SetSvoFrame_Response> res);
  void callback_clickedPoint(
    const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void callback_gnssFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void callback_clock(const rosgraph_msgs::msg::Clock::SharedPtr msg);
  void callback_setRoi(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_msgs::srv::SetROI_Request> req,
    std::shared_ptr<zed_msgs::srv::SetROI_Response> res);
  void callback_resetRoi(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
  void callback_toLL(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<robot_localization::srv::ToLL_Request> req,
    std::shared_ptr<robot_localization::srv::ToLL_Response> res);
  void callback_fromLL(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<robot_localization::srv::FromLL_Request> req,
    std::shared_ptr<robot_localization::srv::FromLL_Response> res);
  // <---- Callbacks

  // ----> Thread functions
  // Main thread
  void threadFunc_zedGrab();

  // Video/Depth thread
  void threadFunc_videoDepthElab();
  void setupVideoDepthThread();
  bool waitForVideoDepthData(std::unique_lock<std::mutex> & lock);
  void handleVideoDepthPublishing();
  // Point Cloud thread
  void threadFunc_pointcloudElab();
  void setupPointCloudThread();
  bool waitForPointCloudData(std::unique_lock<std::mutex> & lock);
  void handlePointCloudPublishing();
  // Sensors thread
  void threadFunc_pubSensorsData();
  // <---- Thread functions

  // ----> Publishing functions

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

  void publishDepthMapWithInfo(sl::Mat & depth, rclcpp::Time t);
  void publishDisparity(sl::Mat disparity, rclcpp::Time t);

  void processVideoDepth();
  bool areVideoDepthSubscribed();
  void retrieveVideoDepth(bool gpu);
  bool retrieveLeftImage(bool gpu);
  bool retrieveLeftRawImage(bool gpu);
  bool retrieveRightImage(bool gpu);
  bool retrieveRightRawImage(bool gpu);
  bool retrieveLeftGrayImage(bool gpu);
  bool retrieveLeftRawGrayImage(bool gpu);
  bool retrieveRightGrayImage(bool gpu);
  bool retrieveRightRawGrayImage(bool gpu);
  bool retrieveDepthMap(bool gpu);
  bool retrieveConfidence(bool gpu);
  bool retrieveDisparity();
  bool retrieveDepthInfo();

  void publishVideoDepth(rclcpp::Time & out_pub_ts);
  void publishLeftAndRgbImages(const rclcpp::Time & t);
  void publishLeftRawAndRgbRawImages(const rclcpp::Time & t);
  void publishLeftGrayAndRgbGrayImages(const rclcpp::Time & t);
  void publishLeftRawGrayAndRgbRawGrayImages(const rclcpp::Time & t);
  void publishRightImages(const rclcpp::Time & t);
  void publishRightRawImages(const rclcpp::Time & t);
  void publishRightGrayImages(const rclcpp::Time & t);
  void publishRightRawGrayImages(const rclcpp::Time & t);
  void publishStereoImages(const rclcpp::Time & t);
  void publishStereoRawImages(const rclcpp::Time & t);
  void publishDepthImage(const rclcpp::Time & t);
  void publishConfidenceMap(const rclcpp::Time & t);
  void publishDisparityImage(const rclcpp::Time & t);
  void publishDepthInfo(const rclcpp::Time & t);

  void checkRgbDepthSync();
  bool checkGrabAndUpdateTimestamp(rclcpp::Time & out_pub_ts);

  void processPointCloud();
  bool isPointCloudSubscribed();
  void publishPointCloud();
  void publishImuFrameAndTopic();

  void publishOdom(
    tf2::Transform & odom2baseTransf, sl::Pose & slPose,
    rclcpp::Time t);
  void publishPose();
  void publishPoseLandmarks();
  void publishGnssPose();
  void publishPoseStatus();
  void publishGnssPoseStatus();
  void publishGeoPoseStatus();
  void publishTFs(rclcpp::Time t);
  void publishOdomTF(rclcpp::Time t);
  void publishPoseTF(rclcpp::Time t);
  bool publishSensorsData(rclcpp::Time force_ts = TIMEZERO_ROS);
  void publishHealthStatus();
  bool publishSvoStatus(uint64_t frame_ts);

  void publishClock(const sl::Timestamp & ts);
  // <---- Publishing functions

  // ----> Utility functions
  bool isDepthRequired();
  bool isPosTrackingRequired();

  void applyVideoSettings();
  void applyAutoExposureGainSettings();
  void applyExposureGainSettings();
  void applyWhiteBalanceSettings();
  void applyBrightnessContrastHueSettings();
  void applySaturationSharpnessGammaSettings();
  void applyZEDXSettings();
  void applyZEDXExposureSettings();
  void applyZEDXAutoExposureTimeRange();
  void applyZEDXExposureCompensation();
  void applyZEDXAnalogDigitalGain();
  void applyZEDXAutoAnalogGainRange();
  void applyZEDXAutoDigitalGainRange();
  void applyZEDXDenoising();

  void applyDepthSettings();

  void processOdometry();
  void processPose();
  void processGeoPose();
  void processSvoGnssData();

  void processDetectedObjects(rclcpp::Time t);
  void processBodies(rclcpp::Time t);

  void processRtRoi(rclcpp::Time t);

  bool setPose(float xt, float yt, float zt, float rr, float pr, float yr);
  void initTransforms();
  bool getSens2BaseTransform();
  bool getSens2CameraTransform();
  bool getCamera2BaseTransform();
  bool getGnss2BaseTransform();

  void startFusedPcTimer(double fusedPcRate);
  void startPathPubTimer(double pathTimerRate);
  void startTempPubTimer();
  void startHeartbeatTimer();

  // Region of Interest
  std::string getParam(
    std::string paramName,
    std::vector<std::vector<float>> & outVal);
  std::string parseRoiPoly(
    const std::vector<std::vector<float>> & in_poly,
    std::vector<sl::float2> & out_poly);
  // <---- Utility functions

private:
  // ZED SDK
  std::shared_ptr<sl::Camera> mZed;
  sl::InitParameters mInitParams;
  sl::RuntimeParameters mRunParams;


  // ----> Fusion module
  std::shared_ptr<sl::FusionConfiguration> mFusionConfig;
  sl::Fusion mFusion;
  sl::InitFusionParameters mFusionInitParams;
  sl::CameraIdentifier mCamUuid;
  // <---- Fusion module

  uint64_t mFrameCount = 0;
  uint32_t mSvoLoopCount = 0;

  // ----> Topics
  std::string mTopicRoot = "~/";

  // Image Topics
  std::string mLeftTopic;
  std::string mLeftRawTopic;
  std::string mRightTopic;
  std::string mRightRawTopic;
  std::string mRgbTopic;
  std::string mRgbRawTopic;
  std::string mStereoTopic;
  std::string mStereoRawTopic;
  std::string mLeftGrayTopic;
  std::string mLeftRawGrayTopic;
  std::string mRightGrayTopic;
  std::string mRightRawGrayTopic;
  std::string mRgbGrayTopic;
  std::string mRgbRawGrayTopic;

  // Depth Topics
  std::string mDisparityTopic;
  std::string mDepthTopic;
  std::string mDepthInfoTopic;
  std::string mConfMapTopic;
  std::string mPointcloudTopic;

  // Localization Topics
  std::string mOdomTopic;
  std::string mPoseTopic;
  std::string mPoseStatusTopic;
  std::string mPoseCovTopic;
  std::string mGnssPoseTopic;
  std::string mGnssPoseStatusTopic;
  std::string mGeoPoseTopic;
  std::string mGeoPoseStatusTopic;
  std::string mFusedFixTopic;
  std::string mOriginFixTopic;
  std::string mPointcloudFusedTopic;
  std::string mPointcloud3DLandmarksTopic;
  std::string mObjectDetTopic;
  std::string mBodyTrkTopic;
  std::string mOdomPathTopic;
  std::string mPosePathTopic;
  std::string mClickedPtTopic;  // Clicked point
  std::string mRoiMaskTopic;
  // <---- Topics

  // ----> Parameter variables
  // Debug
  bool _debugCommon = false;
  bool _debugSim = false;
  bool _debugVideoDepth = false;
  bool _debugCamCtrl = false;
  bool _debugPointCloud = false;
  bool _debugPosTracking = false;
  bool _debugGnss = false;
  bool _debugSensors = false;
  bool _debugMapping = false;
  bool _debugObjectDet = false;
  bool _debugBodyTrk = false;
  bool _debugAdvanced = false;
  bool _debugRoi = false;
  bool _debugStreaming = false;
  bool _debugNitros = false;
  // If available, force disable NITROS usage for debugging and testing
  // purposes; otherwise, this is always true.
  bool _nitrosDisabled = false;

  // Topic Enablers
  bool mPublishSensImu = true;
  bool mPublishSensImuRaw = false;
  bool mPublishSensMag = false;
  bool mPublishSensBaro = false;
  bool mPublishSensTemp = false;
  bool mPublishSensImuTransf = false;
  bool mPublishImgLeftRight = false;
  bool mPublishImgRaw = false;
  bool mPublishImgGray = false;
  bool mPublishImgRgb = true;
  bool mPublishImgStereo = false;
  bool mPublishImgRoiMask = false;
  bool mPublishOdomPose = true;
  bool mPublishPoseCov = false;
  bool mPublishPath = false;
  bool mPublishDetPlane = false;
  bool mPublishDepthMap = true;
  bool mPublishDepthInfo = false;
  bool mPublishPointcloud = true;
  bool mPublishConfidence = false;
  bool mPublishDisparity = false;
  bool mPublishStatus = true;
  bool mPublishSvoClock = false;

  // General
  int mCamSerialNumber = 0;
  int mCamId = -1;
  bool mSimMode = false;     // Expecting simulation data?
  bool mUseSimTime = false;  // Use sim time?
  std::string mSimAddr =
    "127.0.0.1";    // The local address of the machine running the simulator
  int mSimPort = 30000;  // The port to be used to connect to the simulator

  bool mStreamMode = false;     // Expecting local streaming data?
  std::string mStreamAddr = "";  // The local address of the streaming server
  int mStreamPort = 30000;  // The port to be used to connect to a local streaming server

  sl::MODEL mCamUserModel = sl::MODEL::ZED;  // Default camera model
  sl::MODEL mCamRealModel;                   // Camera model requested to SDK
  unsigned int mCamFwVersion;                // Camera FW version
  unsigned int mSensFwVersion;               // Sensors FW version
  std::string mCameraName = "zed";           // Default camera name
  int mCamGrabFrameRate = 15;
  bool mAsyncImageRetrieval = false;
  int mImageValidityCheck = 1;
  std::string mSvoFilepath = "";
  bool mSvoLoop = false;
  bool mSvoRealtime = false;
  int mSvoFrameStart = 0;
  double mSvoRate = 1.0;
  double mSvoExpectedPeriod = 0.0;
  bool mUseSvoTimestamp = false;
  bool mUsePubTimestamps = false;
  bool mGrabOnce = false;
  bool mGrabImuOnce = false;
  int mVerbose = 1;
  std::string mVerboseLogFile = "";
  int mGpuId = -1;
  std::string mOpencvCalibFile;
  sl::RESOLUTION mCamResol = sl::RESOLUTION::HD1080;    // Default resolution: RESOLUTION_HD1080
  PubRes mPubResolution = PubRes::NATIVE;                     // Use native grab resolution by default
  double mCustomDownscaleFactor = 1.0;  // Used to rescale data with user factor
  bool mOpenniDepthMode =
    false;    // 16 bit UC data in mm else 32F in m,
              // for more info -> http://www.ros.org/reps/rep-0118.html
  double mCamMinDepth = 0.1;
  double mCamMaxDepth = 10.0;
  sl::DEPTH_MODE mDepthMode = sl::DEPTH_MODE::NEURAL;
  PcRes mPcResolution = PcRes::COMPACT;
  bool mDepthDisabled = false;  // Indicates if depth calculation is not required (DEPTH_MODE::NONE)
  int mDepthStabilization = 1;

  int mCamTimeoutSec = 5;
  int mMaxReconnectTemp = 5;
  bool mCameraSelfCalib = true;
  bool mCameraFlip = false;


  bool mSensCameraSync = false;
  double mSensPubRate = 200.;

  std::vector<std::vector<float>> mRoyPolyParam;  // Manual ROI polygon
  bool mAutoRoiEnabled = false;
  bool mManualRoiEnabled = false;
  float mRoiDepthFarThresh = 2.5f;
  float mRoiImgHeightRationCutOff = 0.5f;
  std::unordered_set<sl::MODULE> mRoiModules;

  bool mPosTrackingEnabled = false;
  bool mPublishTF = false;
  bool mPublishMapTF = false;
  bool mPublishImuTF = false;
  bool mPoseSmoothing = false;
  bool mAreaMemory = true;
  std::string mAreaMemoryFilePath = "";
  sl::POSITIONAL_TRACKING_MODE mPosTrkMode =
    sl::POSITIONAL_TRACKING_MODE::GEN_1;
  bool mSaveAreaMemoryOnClosing = true;
  bool mImuFusion = true;
  bool mFloorAlignment = false;
  bool mTwoDMode = false;
  float mFixedZValue = 0.0;
  std::vector<double> mInitialBasePose = std::vector<double>(6, 0.0);
  bool mResetOdomWhenLoopClosure = true;
  bool mResetPoseWithSvoLoop = true;
  bool mPublish3DLandmarks = false;
  uint8_t mPublishLandmarkSkipFrame = 15;
  double mPathPubRate = 2.0;
  double mTfOffset = 0.0;
  float mPosTrackDepthMinRange = 0.0f;
  bool mSetAsStatic = false;
  bool mSetGravityAsOrigin = false;
  int mPathMaxCount = -1;

  bool mGnssFusionEnabled = false;
  std::string mGnssTopic = "/gps/fix";
  bool mGnssEnableReinitialization = true;
  bool mGnssEnableRollingCalibration = true;
  bool mGnssEnableTranslationUncertaintyTarget = false;
  double mGnssVioReinitThreshold = 5.0;
  double mGnssTargetTranslationUncertainty = 0.1;
  double mGnssTargetYawUncertainty = 0.1;
  double mGnssHcovMul = 1.0;
  double mGnssVcovMul = 1.0;
  bool mGnssZeroAltitude = false;
  bool mPublishUtmTf = true;
  bool mUtmAsParent = true;

  bool mMappingEnabled = false;
  float mMappingRes = 0.05f;
  float mMappingRangeMax = 10.0f;

  bool mObjDetEnabled = false;
  bool mObjDetTracking = true;
  double mObjDetPredTimeout = 0.5;
  bool mObjDetReducedPrecision = false;
  double mObjDetMaxRange = 15.0;
  std::vector<sl::OBJECT_CLASS> mObjDetFilter;
  std::map<sl::OBJECT_CLASS, float> mObjDetClassConfMap;
  bool mObjDetPeopleEnable = true;
  double mObjDetPeopleConf = 50.0;
  bool mObjDetVehiclesEnable = true;
  double mObjDetVehiclesConf = 50.0;
  bool mObjDetBagsEnable = true;
  double mObjDetBagsConf = 50.0;
  bool mObjDetAnimalsEnable = true;
  double mObjDetAnimalsConf = 50.0;
  bool mObjDetElectronicsEnable = true;
  double mObjDetElectronicsConf = 50.0;
  bool mObjDetFruitsEnable = true;
  double mObjDetFruitsConf = 50.0;
  bool mObjDetSportEnable = true;
  double mObjDetSportConf = 50.0;
  sl::OBJECT_DETECTION_MODEL mObjDetModel =
    sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_FAST;
  sl::OBJECT_FILTERING_MODE mObjFilterMode = sl::OBJECT_FILTERING_MODE::NMS3D;
  std::string mYoloOnnxPath = "";
  int mYoloOnnxSize = 512;
  int mCustomClassCount = 1;
  std::unordered_map<int, sl::CustomObjectDetectionProperties> mCustomOdProperties;
  std::unordered_map<int, std::string> mCustomLabels;
  std::unordered_map<std::string, int> mCustomClassIdMap;


  bool mBodyTrkEnabled = false;
  sl::BODY_TRACKING_MODEL mBodyTrkModel =
    sl::BODY_TRACKING_MODEL::HUMAN_BODY_FAST;
  sl::BODY_FORMAT mBodyTrkFmt = sl::BODY_FORMAT::BODY_38;
  bool mBodyTrkReducedPrecision = false;
  double mBodyTrkMaxRange = 15.0f;
  sl::BODY_KEYPOINTS_SELECTION mBodyTrkKpSelection =
    sl::BODY_KEYPOINTS_SELECTION::FULL;
  bool mBodyTrkFitting = true;
  bool mBodyTrkEnableTracking = true;
  double mBodyTrkPredTimeout = 0.5;
  double mBodyTrkConfThresh = 50.0;
  int mBodyTrkMinKp = 10;

  double mPdMaxDistanceThreshold = 0.15;
  double mPdNormalSimilarityThreshold = 15.0;

  std::string mThreadSchedPolicy;
  int mThreadPrioGrab;
  int mThreadPrioSens;
  int mThreadPrioPointCloud;

  std::atomic<bool> mStreamingServerRequired;
  sl::STREAMING_CODEC mStreamingServerCodec = sl::STREAMING_CODEC::H264;
  int mStreamingServerPort = 30000;
  int mStreamingServerBitrate = 12500;
  int mStreamingServerGopSize = -1;
  bool mStreamingServerAdaptiveBitrate = false;
  int mStreamingServerChunckSize = 16084;
  int mStreamingServerTargetFramerate = 0;
  // <---- Parameter variables

  // ----> Dynamic params
  OnSetParametersCallbackHandle::SharedPtr mParamChangeCallbackHandle;

  double mVdPubRate = 15.0;
  int mCamBrightness = 4;
  int mCamContrast = 4;
  int mCamHue = 0;
  int mCamSaturation = 4;
  int mCamSharpness = 4;
  int mCamGamma = 8;
  bool mCamAutoExpGain = true;
  int mCamGain = 80;
  int mCamExposure = 80;
  bool mCamAutoWB = true;
  int mCamWBTemp = 42;
  int mDepthConf = 50;
  int mDepthTextConf = 100;
  double mPcPubRate = 15.0;
  double mFusedPcPubRate = 1.0;
  bool mRemoveSatAreas = true;

  int mGmslExpTime = 16666;
  int mGmslAutoExpTimeRangeMin = 28;
  int mGmslAutoExpTimeRangeMax = 30000;
  int mGmslExposureComp = 50;
  int mGmslAnalogGain = 8000;
  int mGmslAnalogGainRangeMin = 1000;
  int mGmslAnalogGainRangeMax = 16000;
  int mGmslDigitalGain = 128;
  int mGmslAutoDigitalGainRangeMin = 1;
  int mGmslAutoDigitalGainRangeMax = 256;
  int mGmslDenoising = 50;
  // <---- Dynamic params

  // ----> QoS
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
  rclcpp::QoS mQos;
  rclcpp::PublisherOptions mPubOpt;
  rclcpp::SubscriptionOptions mSubOpt;
  // <---- QoS

  // ----> Frame IDs
  std::string mDepthFrameId;
  std::string mDepthOptFrameId;

  std::string mPointCloudFrameId;

  std::string mUtmFrameId = "utm";
  std::string mMapFrameId = "map";
  std::string mOdomFrameId = "odom";
  std::string mBaseFrameId = "";
  std::string mGnssFrameId = "";
  std::string mGnssOriginFrameId = "gnss_ref_pose";

  std::string mCameraFrameId;

  std::string mRightCamFrameId;
  std::string mRightCamOptFrameId;
  std::string mLeftCamFrameId;
  std::string mLeftCamOptFrameId;

  std::string mImuFrameId;
  std::string mBaroFrameId;
  std::string mMagFrameId;
  std::string mTempLeftFrameId;
  std::string mTempRightFrameId;
  // <---- Frame IDs

  // ----> Stereolabs Mat Info
  int mCamWidth;   // Camera frame width
  int mCamHeight;  // Camera frame height
  sl::Resolution mMatResol;
  sl::Resolution mPcResol;
  // <---- Stereolabs Mat Info

  // Camera IMU transform
  sl::Transform mSlCamImuTransf;

  // ----> initialization Transform listener
  std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> mTfListener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
  // <---- initialization Transform listener

  // ----> TF Transforms
  tf2::Transform
    mMap2OdomTransf;    // Coordinates of the odometry frame in map frame
  tf2::Transform mOdom2BaseTransf;  // Coordinates of the base in odometry frame
  tf2::Transform mMap2BaseTransf;   // Coordinates of the base in map frame
  tf2::Transform
    mSensor2BaseTransf;    // Coordinates of the base frame in sensor frame
  tf2::Transform
    mSensor2CameraTransf;    // Coordinates of the camera frame in sensor frame
  tf2::Transform
    mCamera2BaseTransf;    // Coordinates of the base frame in camera frame
  tf2::Transform mMap2UtmTransf;  // Coordinates of the UTM frame in map frame
  tf2::Transform
    mGnss2BaseTransf;    // Coordinates of the base in GNSS sensor frame
  // <---- TF Transforms

  // ----> TF Transforms Flags
  bool mSensor2BaseTransfValid = false;
  bool mSensor2BaseTransfFirstErr = true;
  bool mSensor2CameraTransfValid = false;
  bool mSensor2CameraTransfFirstErr = true;
  bool mCamera2BaseTransfValid = false;
  bool mCamera2BaseFirstErr = true;
  bool mGnss2BaseTransfValid = false;
  bool mGnss2BaseTransfFirstErr = true;
  bool mMap2UtmTransfValid = false;

  std::atomic_uint16_t mAiInstanceID;
  uint16_t mObjDetInstID;
  uint16_t mBodyTrkInstID;
  // <---- TF Transforms Flags

  // ----> Messages (ONLY THOSE NOT CHANGING WHILE NODE RUNS)
  // Camera infos
  camInfoMsgPtr mLeftCamInfoMsg;
  camInfoMsgPtr mRightCamInfoMsg;
  camInfoMsgPtr mLeftCamInfoRawMsg;
  camInfoMsgPtr mRightCamInfoRawMsg;
  // <---- Messages

  // ----> Publishers
  clockPub mPubClock;

  // Image publishers with camera info
  image_transport::Publisher mPubRgb;
  image_transport::Publisher mPubRawRgb;
  image_transport::Publisher mPubLeft;
  image_transport::Publisher mPubRawLeft;
  image_transport::Publisher mPubRight;
  image_transport::Publisher mPubRawRight;
  image_transport::Publisher mPubRgbGray;
  image_transport::Publisher mPubRawRgbGray;
  image_transport::Publisher mPubLeftGray;
  image_transport::Publisher mPubRawLeftGray;
  image_transport::Publisher mPubRightGray;
  image_transport::Publisher mPubRawRightGray;
  image_transport::Publisher mPubRoiMask;
  image_transport::Publisher mPubDepth;
  image_transport::Publisher mPubConfMap;
#ifdef FOUND_ISAAC_ROS_NITROS
  // Nitros image publishers with camera info
  nitrosImgPub mNitrosPubRgb;
  nitrosImgPub mNitrosPubRawRgb;
  nitrosImgPub mNitrosPubLeft;
  nitrosImgPub mNitrosPubRawLeft;
  nitrosImgPub mNitrosPubRight;
  nitrosImgPub mNitrosPubRawRight;
  nitrosImgPub mNitrosPubRgbGray;
  nitrosImgPub mNitrosPubRawRgbGray;
  nitrosImgPub mNitrosPubLeftGray;
  nitrosImgPub mNitrosPubRawLeftGray;
  nitrosImgPub mNitrosPubRightGray;
  nitrosImgPub mNitrosPubRawRightGray;
  nitrosImgPub mNitrosPubRoiMask;
  nitrosImgPub mNitrosPubDepth;
  nitrosImgPub mNitrosPubConfMap;
#endif

  // Image publishers without camera info (no NITROS)
  image_transport::Publisher mPubStereo;
  image_transport::Publisher mPubRawStereo;

  // Camera Info publishers
  camInfoPub mPubRgbCamInfo;
  camInfoPub mPubRawRgbCamInfo;
  camInfoPub mPubLeftCamInfo;
  camInfoPub mPubRawLeftCamInfo;
  camInfoPub mPubRightCamInfo;
  camInfoPub mPubRawRightCamInfo;
  camInfoPub mPubRgbGrayCamInfo;
  camInfoPub mPubRawRgbGrayCamInfo;
  camInfoPub mPubLeftGrayCamInfo;
  camInfoPub mPubRawLeftGrayCamInfo;
  camInfoPub mPubRightGrayCamInfo;
  camInfoPub mPubRawRightGrayCamInfo;
  camInfoPub mPubRoiMaskCamInfo;
  camInfoPub mPubDepthCamInfo;
  camInfoPub mPubConfMapCamInfo;
  camInfoPub mPubRgbCamInfoTrans;
  camInfoPub mPubRawRgbCamInfoTrans;
  camInfoPub mPubLeftCamInfoTrans;
  camInfoPub mPubRawLeftCamInfoTrans;
  camInfoPub mPubRightCamInfoTrans;
  camInfoPub mPubRawRightCamInfoTrans;
  camInfoPub mPubRgbGrayCamInfoTrans;
  camInfoPub mPubRawRgbGrayCamInfoTrans;
  camInfoPub mPubLeftGrayCamInfoTrans;
  camInfoPub mPubRawLeftGrayCamInfoTrans;
  camInfoPub mPubRightGrayCamInfoTrans;
  camInfoPub mPubRawRightGrayCamInfoTrans;
  camInfoPub mPubRoiMaskCamInfoTrans;
  camInfoPub mPubDepthCamInfoTrans;
  camInfoPub mPubConfMapCamInfoTrans;

#ifdef FOUND_POINT_CLOUD_TRANSPORT
  point_cloud_transport::Publisher mPubCloud;
  point_cloud_transport::Publisher mPubFusedCloud;
  point_cloud_transport::Publisher mPub3DLandmarks;
#else
  pointcloudPub mPubCloud;
  pointcloudPub mPubFusedCloud;
  pointcloudPub mPub3DLandmarks;
#endif

  svoStatusPub mPubSvoStatus;
  healthStatusPub mPubHealthStatus;
  heartbeatStatusPub mPubHeartbeatStatus;
  disparityPub mPubDisparity;
  posePub mPubPose;
  poseStatusPub mPubPoseStatus;
  poseCovPub mPubPoseCov;
  odomPub mPubOdom;
  odomPub mPubGnssPose;
  int mFrameSkipCountLandmarks = 0;
  gnssFusionStatusPub mPubGnssPoseStatus;
  pathPub mPubOdomPath;
  pathPub mPubPosePath;
  imuPub mPubImu;
  imuPub mPubImuRaw;
  tempPub mPubImuTemp;
  magPub mPubImuMag;
  pressPub mPubPressure;
  tempPub mPubTempL;
  tempPub mPubTempR;
  transfPub mPubCamImuTransf;
  objPub mPubObjDet;
  objPub mPubBodyTrk;
  depthInfoPub mPubDepthInfo;
  planePub mPubPlane;
  markerPub mPubMarker;

  geoPosePub mPubGeoPose;
  gnssFusionStatusPub mPubGeoPoseStatus;
  gnssFixPub mPubFusedFix;
  gnssFixPub mPubOriginFix;
  // <---- Publishers

  // <---- Publisher variables
  sl::Timestamp mSdkGrabTS = 0;
  size_t mRgbSubCount = 0;
  size_t mRgbRawSubCount = 0;
  size_t mRgbGraySubCount = 0;
  size_t mRgbGrayRawSubCount = 0;
  size_t mLeftSubCount = 0;
  size_t mLeftRawSubCount = 0;
  size_t mLeftGraySubCount = 0;
  size_t mLeftGrayRawSubCount = 0;
  size_t mRightSubCount = 0;
  size_t mRightRawSubCount = 0;
  size_t mRightGraySubCount = 0;
  size_t mRightGrayRawSubCount = 0;
  size_t mStereoSubCount = 0;
  size_t mStereoRawSubCount = 0;
  size_t mDepthSubCount = 0;
  size_t mConfMapSubCount = 0;
  size_t mDisparitySubCount = 0;
  size_t mDepthInfoSubCount = 0;

  sl::Mat mMatLeft, mMatLeftRaw;
  sl::Mat mMatRight, mMatRightRaw;
  sl::Mat mMatLeftGray, mMatLeftRawGray;
  sl::Mat mMatRightGray, mMatRightRawGray;
  sl::Mat mMatDepth, mMatDisp, mMatConf;

  float mMinDepth = 0.0f;
  float mMaxDepth = 0.0f;
  // <---- Publisher variables

  // ----> Point cloud variables
  sl::Mat mMatCloud;
  sl::FusedPointCloud mFusedPC;
  // <---- Point cloud variables

  // ----> Subscribers
  clickedPtSub mClickedPtSub;
  gnssFixSub mGnssFixSub;
  clockSub mClockSub;
  // <---- Subscribers

  // ----> Threads and Timers
  sl::ERROR_CODE mGrabStatus;
  sl::ERROR_CODE mConnStatus;
  sl::FUSION_ERROR_CODE mFusionStatus = sl::FUSION_ERROR_CODE::MODULE_NOT_ENABLED;
  std::thread mGrabThread;        // Main grab thread
  std::thread mVdThread;          // Video and Depth data processing thread
  std::thread mPcThread;          // Point Cloud publish thread
  std::thread mSensThread;        // Sensors data publish thread
  std::atomic<bool> mThreadStop;
  rclcpp::TimerBase::SharedPtr mInitTimer;
  rclcpp::TimerBase::SharedPtr mPathTimer;
  rclcpp::TimerBase::SharedPtr mFusedPcTimer;
  rclcpp::TimerBase::SharedPtr
    mTempPubTimer;    // Timer to retrieve and publish CMOS temperatures
  rclcpp::TimerBase::SharedPtr mGnssPubCheckTimer;
  rclcpp::TimerBase::SharedPtr mHeartbeatTimer;
  double mSensRateComp = 1.0;
  // <---- Threads and Timers

  // ----> Thread Sync
  std::mutex mRecMutex;
  std::mutex mDynParMutex;
  std::mutex mMappingMutex;
  std::mutex mObjDetMutex;
  std::mutex mBodyTrkMutex;
  std::mutex mPcMutex;
  std::mutex mCloseCameraMutex;
  std::mutex mPtMutex;
  std::condition_variable mPcDataReadyCondVar;
  std::atomic_bool mPcDataReady;
  std::mutex mVdMutex;
  std::condition_variable mVdDataReadyCondVar;
  std::atomic_bool mVdDataReady;
  // <---- Thread Sync

  // ----> Status Flags
  bool mDebugMode = false;  // Debug mode active?
  bool mSvoMode = false;
  bool mSvoPause = false;
  int mSvoFrameId = 0;
  int mSvoFrameCount = 0;
  bool mPosTrackingStarted = false;
  bool mVdPublishing = false;  // Indicates if video and depth data are
                               // subscribed and then published
  bool mPcPublishing =
    false;    // Indicates if point cloud data are subscribed and then published
  bool mTriggerAutoExpGain = true;  // Triggered on start
  bool mTriggerAutoWB = true;       // Triggered on start
  bool mRecording = false;
  sl::RecordingStatus mRecStatus = sl::RecordingStatus();
  bool mPosTrackingReady = false;

  sl::FusedPositionalTrackingStatus mFusedPosTrackingStatus;
  sl::PositionalTrackingStatus mPosTrackingStatus;

  sl::REGION_OF_INTEREST_AUTO_DETECTION_STATE mAutoRoiStatus =
    sl::REGION_OF_INTEREST_AUTO_DETECTION_STATE::NOT_ENABLED;

  bool mAreaFileExists = false;
  bool mResetOdomFromSrv = false;
  bool mSpatialMappingRunning = false;
  bool mObjDetRunning = false;
  bool mBodyTrkRunning = false;
  bool mRgbSubscribed = false;
  bool mGnssMsgReceived = false;  // Indicates if a NavSatFix topic has been
                                  // received, also with invalid position fix
  bool mGnssFixValid = false;     // Used to keep track of signal loss
  bool mGnssFixNew = false;       // Used to keep track of signal loss
  std::string mGnssServiceStr = "";
  uint16_t mGnssService;
  std::atomic<bool> mClockAvailable;  // Indicates if the "/clock" topic is
  // published when `use_sim_time` is true

  std::atomic<bool> mStreamingServerRunning;

  bool mUsingCustomOd = false;
  uint64_t mHeartbeatCount = 0;
  // <---- Status Flags

  // ----> Positional Tracking
  sl::Pose mLastZedPose;
  sl::Transform mInitialPoseSl;
  std::vector<geometry_msgs::msg::PoseStamped> mOdomPath;
  std::vector<geometry_msgs::msg::PoseStamped> mPosePath;
  sl::GeoPose mLastGeoPose;
  sl::ECEF mLastEcefPose;
  sl::UTM mLastUtmPose;
  sl::LatLng mLastLatLongPose;
  double mLastHeading;
  tf2::Quaternion mLastHeadingQuat;
  sl::ECEF mInitEcefPose;
  sl::UTM mInitUtmPose;
  sl::LatLng mInitLatLongPose;
  double mInitHeading;
  bool mGnssInitGood = false;
  sl::float3 mGnssAntennaPose;
  // <---- Positional Tracking

  // ----> Diagnostic
  sl_tools::StopWatch mUptimer;
  float mTempImu = NOT_VALID_TEMP;
  float mTempLeft = NOT_VALID_TEMP;
  float mTempRight = NOT_VALID_TEMP;
  std::unique_ptr<sl_tools::WinAvg> mElabPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mGrabPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mVideoDepthPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mVideoDepthElabMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mPcPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mPcProcMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mImuPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mBaroPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mMagPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mObjDetPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mObjDetElabMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mBodyTrkPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mBodyTrkElabMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mPubFusedCloudPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mPubOdomTF_sec;
  std::unique_ptr<sl_tools::WinAvg> mPubPoseTF_sec;
  std::unique_ptr<sl_tools::WinAvg> mPubImuTF_sec;
  std::unique_ptr<sl_tools::WinAvg> mGnssFix_sec;
  bool mImuPublishing = false;
  bool mMagPublishing = false;
  bool mBaroPublishing = false;
  bool mObjDetSubscribed = false;
  bool mBodyTrkSubscribed = false;

  diagnostic_updater::Updater mDiagUpdater;  // Diagnostic Updater

  sl_tools::StopWatch mImuTfFreqTimer;
  sl_tools::StopWatch mGrabFreqTimer;
  sl_tools::StopWatch mImuFreqTimer;
  sl_tools::StopWatch mBaroFreqTimer;
  sl_tools::StopWatch mMagFreqTimer;
  sl_tools::StopWatch mOdomFreqTimer;
  sl_tools::StopWatch mPoseFreqTimer;
  sl_tools::StopWatch mPcPubFreqTimer;
  sl_tools::StopWatch mVdPubFreqTimer;
  sl_tools::StopWatch mSensPubFreqTimer;
  sl_tools::StopWatch mOdFreqTimer;
  sl_tools::StopWatch mBtFreqTimer;
  sl_tools::StopWatch mPcFreqTimer;
  sl_tools::StopWatch mGnssFixFreqTimer;

  int mSysOverloadCount = 0;
  // <---- Diagnostic

  // ----> Timestamps
  sl::Timestamp mLastTs_grab = 0;  // Used to calculate stable publish frequency
  rclcpp::Time mFrameTimestamp;
  rclcpp::Time mGnssTimestamp;
  rclcpp::Time mLastTs_imu;
  rclcpp::Time mLastTs_baro;
  rclcpp::Time mLastTs_mag;
  rclcpp::Time mLastTs_odom;
  rclcpp::Time mLastTs_pose;
  rclcpp::Time mLastTs_pc;
  rclcpp::Time mPrevTs_pc;
  uint64_t mLastTs_gnss_nsec = 0;
  rclcpp::Time mLastClock;
  // <---- Timestamps

  // ----> SVO Recording parameters
  unsigned int mSvoRecBitrate = 0;
  sl::SVO_COMPRESSION_MODE mSvoRecCompression = sl::SVO_COMPRESSION_MODE::H265;
  unsigned int mSvoRecFramerate = 0;
  bool mSvoRecTranscode = false;
  std::string mSvoRecFilename;
  // <---- SVO Recording parameters

  // ----> Services
  resetOdomSrvPtr mResetOdomSrv;
  resetPosTrkSrvPtr mResetPosTrkSrv;
  setPoseSrvPtr mSetPoseSrv;
  saveAreaMemorySrvPtr mSaveAreaMemorySrv;
  enableObjDetPtr mEnableObjDetSrv;
  enableBodyTrkPtr mEnableBodyTrkSrv;
  enableMappingPtr mEnableMappingSrv;
  startSvoRecSrvPtr mStartSvoRecSrv;
  stopSvoRecSrvPtr mStopSvoRecSrv;
  pauseSvoSrvPtr mPauseSvoSrv;
  setSvoFramePtr mSetSvoFrameSrv;
  setRoiSrvPtr mSetRoiSrv;
  resetRoiSrvPtr mResetRoiSrv;
  toLLSrvPtr mToLlSrv;
  fromLLSrvPtr mFromLlSrv;
  enableStreamingPtr mEnableStreamingSrv;

  sl_tools::StopWatch mSetSvoFrameCheckTimer;
  // <---- Services

  // ----> Services names
  const std::string mSrvResetOdomName = "reset_odometry";
  const std::string mSrvResetPoseName = "reset_pos_tracking";
  const std::string mSrvSetPoseName = "set_pose";
  const std::string mSrvSaveAreaMemoryName = "save_area_memory";
  const std::string mSrvEnableObjDetName = "enable_obj_det";
  const std::string mSrvEnableBodyTrkName = "enable_body_trk";
  const std::string mSrvEnableMappingName = "enable_mapping";
  const std::string mSrvEnableStreamingName = "enable_streaming";
  const std::string mSrvStartSvoRecName = "start_svo_rec";
  const std::string mSrvStopSvoRecName = "stop_svo_rec";
  const std::string mSrvToggleSvoPauseName = "toggle_svo_pause";
  const std::string mSrvSetSvoFrameName = "set_svo_frame";
  const std::string mSrvSetRoiName = "set_roi";
  const std::string mSrvResetRoiName = "reset_roi";
  const std::string mSrvToLlName = "toLL";  // Convert from `map` to `Lat Long`
  const std::string mSrvFromLlName = "fromLL";  // Convert from `Lat Long` to `map`
  // <---- Services names

  // ----> SVO v2
  std::unique_ptr<sl_tools::GNSSReplay> mGnssReplay;
  // <---- SVO v2
};

}  // namespace stereolabs

#endif  // ZED_CAMERA_COMPONENT_HPP_
