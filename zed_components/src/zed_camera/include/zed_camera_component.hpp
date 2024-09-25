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

#ifndef ZED_CAMERA_COMPONENT_HPP_
#define ZED_CAMERA_COMPONENT_HPP_

#include <atomic>
#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>
#include <unordered_set>

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
  void init();
  void initParameters();
  void initServices();
  void initThreads();

  void getDebugParams();
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
  void getBodyTrkParams();
  void getOutStreamingParams();
  void getStreamingServerParams();
  void getAdvancedParams();

  void setTFCoordFrameNames();
  void initPublishers();
  void initSubscribers();
  void fillCamInfo(
    const std::shared_ptr<sl::Camera> zed,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> & leftCamInfoMsg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> & rightCamInfoMsg,
    const std::string & leftFrameId, const std::string & rightFrameId,
    bool rawParam = false);

  bool startCamera();
  bool startPosTracking();
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
  // <---- Initialization functions

  // ----> Callbacks
  void callback_pubFusedPc();
  void callback_pubPaths();
  void callback_pubTemp();
  void callback_gnssPubTimerTimeout();
  rcl_interfaces::msg::SetParametersResult callback_paramChange(
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
    const std::shared_ptr<zed_interfaces::srv::SetPose_Request> req,
    std::shared_ptr<zed_interfaces::srv::SetPose_Response> res);
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
    const std::shared_ptr<zed_interfaces::srv::StartSvoRec_Request> req,
    std::shared_ptr<zed_interfaces::srv::StartSvoRec_Response> res);
  void callback_stopSvoRec(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
  void callback_pauseSvoInput(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
  void callback_clickedPoint(
    const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void callback_gnssFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void callback_clock(const rosgraph_msgs::msg::Clock::SharedPtr msg);
  void callback_setRoi(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_interfaces::srv::SetROI_Request> req,
    std::shared_ptr<zed_interfaces::srv::SetROI_Response> res);
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
  void threadFunc_zedGrab();
  void threadFunc_pointcloudElab();
  void threadFunc_pubSensorsData();
  // <---- Thread functions

  // ----> Publishing functions
  void publishImageWithInfo(
    sl::Mat & img,
    image_transport::CameraPublisher & pubImg,
    camInfoMsgPtr & camInfoMsg, std::string imgFrameId,
    rclcpp::Time t);
  void publishDepthMapWithInfo(sl::Mat & depth, rclcpp::Time t);
  void publishDisparity(sl::Mat disparity, rclcpp::Time t);

  bool areVideoDepthSubscribed();
  void retrieveVideoDepth();
  void publishVideoDepth(rclcpp::Time & out_pub_ts);
  void publishPointCloud();
  void publishImuFrameAndTopic();

  void publishOdom(
    tf2::Transform & odom2baseTransf, sl::Pose & slPose,
    rclcpp::Time t);
  void publishPose();
  void publishGnssPose();
  void publishPoseStatus();
  void publishGnssPoseStatus();
  void publishGeoPoseStatus();
  void publishTFs(rclcpp::Time t);
  void publishOdomTF(rclcpp::Time t);
  void publishPoseTF(rclcpp::Time t);
  rclcpp::Time publishSensorsData(rclcpp::Time t = TIMEZERO_ROS);
  // <---- Publishing functions

  // ----> Utility functions
  bool isDepthRequired();
  bool isPosTrackingRequired();

  void applyVideoSettings();
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

  template<typename T>
  void getParam(
    std::string paramName, T defValue, T & outVal,
    std::string log_info = std::string(), bool dynamic = false);

  // Region of Interest
  std::string getParam(
    std::string paramName,
    std::vector<std::vector<float>> & outVal);
  std::string parseRoiPoly(
    const std::vector<std::vector<float>> & in_poly,
    std::vector<sl::float2> & out_poly);
  void resetRoi();
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

  // ----> Topics
  std::string mTopicRoot = "~/";
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
  std::string mObjectDetTopic;
  std::string mBodyTrkTopic;
  std::string mOdomPathTopic;
  std::string mPosePathTopic;
  std::string mClickedPtTopic;  // Clicked point
  std::string mRoiMaskTopic;
  // <---- Topics

  // ----> Parameter variables
  bool mDebugCommon = false;
  bool mDebugSim = false;
  bool mDebugVideoDepth = false;
  bool mDebugCamCtrl = false;
  bool mDebugPointCloud = false;
  bool mDebugPosTracking = false;
  bool mDebugGnss = false;
  bool mDebugSensors = false;
  bool mDebugMapping = false;
  bool mDebugObjectDet = false;
  bool mDebugBodyTrk = false;
  bool mDebugAdvanced = false;
  bool mDebugRoi = false;
  bool mDebugStreaming = false;

  int mCamSerialNumber = 0;
  bool mSimMode = false;     // Expecting simulation data?
  bool mUseSimTime = false;  // Use sim time?
  std::string mSimAddr =
    "127.0.0.1";    // The local address of the machine running the simulator
  int mSimPort = 30000;  // The port to be used to connect to the simulator

  bool mStreamMode = false;     // Expecting simulation data?
  std::string mStreamAddr = "";  // The local address of the streaming server
  int mStreamPort = 30000;  // The port to be used to connect to a local streaming server

  sl::MODEL mCamUserModel = sl::MODEL::ZED;  // Default camera model
  sl::MODEL mCamRealModel;                   // Camera model requested to SDK
  unsigned int mCamFwVersion;                // Camera FW version
  unsigned int mSensFwVersion;               // Sensors FW version
  std::string mCameraName = "zed";           // Default camera name
  int mCamGrabFrameRate = 15;
  std::string mSvoFilepath = "";
  bool mSvoLoop = false;
  bool mSvoRealtime = false;
  int mVerbose = 1;
  int mGpuId = -1;
  std::string mOpencvCalibFile;
  sl::RESOLUTION mCamResol =
    sl::RESOLUTION::HD1080;    // Default resolution: RESOLUTION_HD1080
  PubRes mPubResolution =
    PubRes::NATIVE;                     // Use native grab resolution by default
  double mCustomDownscaleFactor = 1.0;  // Used to rescale data with user factor
  sl::DEPTH_MODE mDepthMode =
    sl::DEPTH_MODE::ULTRA;      // Default depth mode: ULTRA
  bool mDepthDisabled = false;  // Indicates if depth calculation is not
                                // required (DEPTH_MODE::NONE)
  int mDepthStabilization = 1;
  int mCamTimeoutSec = 5;
  int mMaxReconnectTemp = 5;
  bool mCameraSelfCalib = true;
  bool mCameraFlip = false;
  bool mOpenniDepthMode =
    false;    // 16 bit UC data in mm else 32F in m,
              // for more info -> http://www.ros.org/reps/rep-0118.html
  double mCamMinDepth = 0.2;
  double mCamMaxDepth = 10.0;
  bool mSensCameraSync = false;
  double mSensPubRate = 400.;

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
  std::string mAreaMemoryDbPath = "";
  sl::POSITIONAL_TRACKING_MODE mPosTrkMode =
    sl::POSITIONAL_TRACKING_MODE::GEN_2;
  bool mImuFusion = true;
  bool mFloorAlignment = false;
  bool mTwoDMode = false;
  double mFixedZValue = 0.0;
  std::vector<double> mInitialBasePose = std::vector<double>(6, 0.0);
  bool mResetOdomWhenLoopClosure = true;
  double mPathPubRate = 2.0;
  double mTfOffset = 0.05;
  double mPosTrackDepthMinRange = 0.0;
  bool mSetAsStatic = false;
  bool mSetGravityAsOrigin = false;
  int mPathMaxCount = -1;
  bool mPublishPoseCov = true;

  bool mGnssFusionEnabled = false;
  std::string mGnssTopic = "/gps/fix";
  bool mGnssEnableReinitialization = true;
  bool mGnssEnableRollingCalibration = true;
  bool mGnssEnableTranslationUncertaintyTarget = false;
  double mGnssVioReinitThreshold = 5.0;
  double mGnssTargetTranslationUncertainty = 10e-2;
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
  float mObjDetConfidence = 40.0f;
  double mObjDetPredTimeout = 0.5;
  bool mObjDetReducedPrecision = false;
  float mObjDetMaxRange = 15.0f;
  std::vector<sl::OBJECT_CLASS> mObjDetFilter;
  bool mObjDetPeopleEnable = true;
  bool mObjDetVehiclesEnable = true;
  bool mObjDetBagsEnable = true;
  bool mObjDetAnimalsEnable = true;
  bool mObjDetElectronicsEnable = true;
  bool mObjDetFruitsEnable = true;
  bool mObjDetSportEnable = true;
  bool mObjDetBodyFitting = false;
  sl::OBJECT_DETECTION_MODEL mObjDetModel =
    sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_FAST;
  sl::OBJECT_FILTERING_MODE mObjFilterMode = sl::OBJECT_FILTERING_MODE::NMS3D;

  bool mBodyTrkEnabled = false;
  sl::BODY_TRACKING_MODEL mBodyTrkModel =
    sl::BODY_TRACKING_MODEL::HUMAN_BODY_FAST;
  sl::BODY_FORMAT mBodyTrkFmt = sl::BODY_FORMAT::BODY_38;
  bool mBodyTrkReducedPrecision = false;
  float mBodyTrkMaxRange = 15.0f;
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

  double mPubFrameRate = 15.0;
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
  std::string mRgbFrameId;
  std::string mRgbOptFrameId;

  std::string mDepthFrameId;
  std::string mDepthOptFrameId;

  std::string mDisparityFrameId;
  std::string mDisparityOptFrameId;

  std::string mConfidenceFrameId;
  std::string mConfidenceOptFrameId;

  std::string mCloudFrameId;
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
  camInfoMsgPtr mRgbCamInfoMsg;
  camInfoMsgPtr mLeftCamInfoMsg;
  camInfoMsgPtr mRightCamInfoMsg;
  camInfoMsgPtr mRgbCamInfoRawMsg;
  camInfoMsgPtr mLeftCamInfoRawMsg;
  camInfoMsgPtr mRightCamInfoRawMsg;
  camInfoMsgPtr mDepthCamInfoMsg;
  // <---- Messages

  // ----> Publishers
  image_transport::CameraPublisher mPubRgb;
  image_transport::CameraPublisher mPubRawRgb;
  image_transport::CameraPublisher mPubLeft;
  image_transport::CameraPublisher mPubRawLeft;
  image_transport::CameraPublisher mPubRight;
  image_transport::CameraPublisher mPubRawRight;
  image_transport::CameraPublisher mPubDepth;
  image_transport::Publisher mPubStereo;
  image_transport::Publisher mPubRawStereo;

  image_transport::CameraPublisher mPubRgbGray;
  image_transport::CameraPublisher mPubRawRgbGray;
  image_transport::CameraPublisher mPubLeftGray;
  image_transport::CameraPublisher mPubRawLeftGray;
  image_transport::CameraPublisher mPubRightGray;
  image_transport::CameraPublisher mPubRawRightGray;

  image_transport::CameraPublisher mPubRoiMask;

#ifndef FOUND_FOXY
  point_cloud_transport::Publisher mPubCloud;
  point_cloud_transport::Publisher mPubFusedCloud;
#else
  pointcloudPub mPubCloud;
  pointcloudPub mPubFusedCloud;
#endif

  imagePub mPubConfMap;
  disparityPub mPubDisparity;
  posePub mPubPose;
  poseStatusPub mPubPoseStatus;
  poseCovPub mPubPoseCov;
  odomPub mPubOdom;
  odomPub mPubGnssPose;
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
  size_t mRgbSubnumber = 0;
  size_t mRgbRawSubnumber = 0;
  size_t mRgbGraySubnumber = 0;
  size_t mRgbGrayRawSubnumber = 0;
  size_t mLeftSubnumber = 0;
  size_t mLeftRawSubnumber = 0;
  size_t mLeftGraySubnumber = 0;
  size_t mLeftGrayRawSubnumber = 0;
  size_t mRightSubnumber = 0;
  size_t mRightRawSubnumber = 0;
  size_t mRightGraySubnumber = 0;
  size_t mRightGrayRawSubnumber = 0;
  size_t mStereoSubnumber = 0;
  size_t mStereoRawSubnumber = 0;
  size_t mDepthSubnumber = 0;
  size_t mConfMapSubnumber = 0;
  size_t mDisparitySubnumber = 0;
  size_t mDepthInfoSubnumber = 0;

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
  std::thread mVideoDepthThread;  // RGB/Depth data publish thread
  std::thread mPcThread;          // Point Cloud publish thread
  std::thread mSensThread;        // Sensors data publish thread
  std::atomic<bool> mThreadStop;
  rclcpp::TimerBase::SharedPtr mInitTimer;
  rclcpp::TimerBase::SharedPtr mPathTimer;
  rclcpp::TimerBase::SharedPtr mFusedPcTimer;
  rclcpp::TimerBase::SharedPtr
    mTempPubTimer;    // Timer to retrieve and publish CMOS temperatures
  rclcpp::TimerBase::SharedPtr mGnssPubCheckTimer;
  // <---- Threads and Timers

  // ----> Thread Sync
  std::mutex mRecMutex;
  std::mutex mDynParMutex;
  std::mutex mMappingMutex;
  std::mutex mObjDetMutex;
  std::mutex mBodyTrkMutex;
  std::mutex mPcMutex;
  std::condition_variable mPcDataReadyCondVar;
  std::atomic_bool mPcDataReady;
  // <---- Thread Sync

  // ----> Status Flags
  bool mDebugMode = false;  // Debug mode active?
  bool mSvoMode = false;
  bool mSvoPause = false;
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
  sl::SVO_COMPRESSION_MODE mSvoRecCompr = sl::SVO_COMPRESSION_MODE::H264;
  unsigned int mSvoRecFramerate = 0;
  bool mSvoRecTranscode = false;
  std::string mSvoRecFilename;
  // <---- SVO Recording parameters

  // ----> Services
  resetOdomSrvPtr mResetOdomSrv;
  resetPosTrkSrvPtr mResetPosTrkSrv;
  setPoseSrvPtr mSetPoseSrv;
  enableObjDetPtr mEnableObjDetSrv;
  enableBodyTrkPtr mEnableBodyTrkSrv;
  enableMappingPtr mEnableMappingSrv;
  startSvoRecSrvPtr mStartSvoRecSrv;
  stopSvoRecSrvPtr mStopSvoRecSrv;
  pauseSvoSrvPtr mPauseSvoSrv;
  setRoiSrvPtr mSetRoiSrv;
  resetRoiSrvPtr mResetRoiSrv;
  toLLSrvPtr mToLlSrv;
  fromLLSrvPtr mFromLlSrv;
  enableStreamingPtr mEnableStreamingSrv;
  // <---- Services

  // ----> Services names
  const std::string mSrvResetOdomName = "reset_odometry";
  const std::string mSrvResetPoseName = "reset_pos_tracking";
  const std::string mSrvSetPoseName = "set_pose";
  const std::string mSrvEnableObjDetName = "enable_obj_det";
  const std::string mSrvEnableBodyTrkName = "enable_body_trk";
  const std::string mSrvEnableMappingName = "enable_mapping";
  const std::string mSrvEnableStreamingName = "enable_streaming";
  const std::string mSrvStartSvoRecName = "start_svo_rec";
  const std::string mSrvStopSvoRecName = "stop_svo_rec";
  const std::string mSrvToggleSvoPauseName = "toggle_svo_pause";
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
