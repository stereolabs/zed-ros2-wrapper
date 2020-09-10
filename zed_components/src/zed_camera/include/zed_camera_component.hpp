#ifndef ZED_CAMERA_COMPONENT_HPP
#define ZED_CAMERA_COMPONENT_HPP

#include "visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/publisher.h>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <sl/Camera.hpp>

namespace stereolabs {

// ----> Typedefs to simplify declarations

typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> imagePub;
typedef std::shared_ptr<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>> disparityPub;

typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloudPub;

typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imuPub;
typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::MagneticField>> magPub;
typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::FluidPressure>> pressPub;
typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Temperature>> tempPub;

typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> posePub;
typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> poseCovPub;
typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Transform>> transfPub;
typedef std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPub;
typedef std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> pathPub;

typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloudMsgPtr;
typedef std::shared_ptr<sensor_msgs::msg::Imu> imuMsgPtr;

typedef std::unique_ptr<geometry_msgs::msg::PoseStamped> poseMsgPtr;
typedef std::unique_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> poseCovMsgPtr;
typedef std::unique_ptr<geometry_msgs::msg::Transform> transfMsgPtr;
typedef std::unique_ptr<nav_msgs::msg::Odometry> odomMsgPtr;
typedef std::unique_ptr<nav_msgs::msg::Path> pathMsgPtr;

//typedef rclcpp::Service<stereolabs_zed_interfaces::srv::ResetOdometry>::SharedPtr resetOdomSrvPtr;
//typedef rclcpp::Service<stereolabs_zed_interfaces::srv::RestartTracking>::SharedPtr restartTrkSrvPtr;
//typedef rclcpp::Service<stereolabs_zed_interfaces::srv::SetPose>::SharedPtr setPoseSrvPtr;
//typedef rclcpp::Service<stereolabs_zed_interfaces::srv::StartSvoRecording>::SharedPtr startSvoRecSrvPtr;
//typedef rclcpp::Service<stereolabs_zed_interfaces::srv::StopSvoRecording>::SharedPtr stopSvoRecSrvPtr;
// <---- Typedefs to simplify declarations

class ZedCamera : public rclcpp::Node
{
public:
    ZED_COMPONENTS_PUBLIC
    explicit ZedCamera(const rclcpp::NodeOptions & options);

    virtual ~ZedCamera();

protected:
    // ----> Parameters handling
    void initParameters();

    template<typename T>
    void getParam(std::string paramName, T defValue, T& outVal, std::string log_info=std::string());

    void getGeneralParams();
    void getVideoParams();
    void getDepthParams();
    void getPosTrackingParams();
    void getSensorsParams();

    void setTFCoordFrameNames();

    // Dynamic parameters callback
    rcl_interfaces::msg::SetParametersResult paramChangeCallback(std::vector<rclcpp::Parameter> parameters);
    // ----> Parameters handling

    bool startCamera();

    void initPublishers();

    void fillCamInfo(sl::Camera& zed, std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
                     std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg,
                     std::string leftFrameId, std::string rightFrameId,
                     bool rawParam = false);


private:
    // ZED SDK
    sl::Camera mZed;

    // Params
    sl::InitParameters mZedParams;
    bool mDebugMode=false;
    int mZedId = 0;
    int mZedSerialNumber = 0;
    sl::MODEL mZedUserCamModel = sl::MODEL::ZED;   // Camera model set by ROS Param
    sl::MODEL mZedRealCamModel; // Camera model requested to SDK
    std::string mCameraName = "zed";
    int mZedFrameRate = 15;
    std::string mSvoFilepath = "";
    bool mSvoMode = false;
    bool mVerbose = true;
    int mGpuId = -1;
    sl::RESOLUTION mZedResol = sl::RESOLUTION::HD720; // Default resolution: RESOLUTION_HD720
    sl::DEPTH_MODE mDepthQuality = sl::DEPTH_MODE::PERFORMANCE; // Default quality: DEPTH_MODE_PERFORMANCE
    bool mDepthStabilization = true;
    int mCamTimeoutSec = 5;
    int mMaxReconnectTemp = 5;
    bool mZedReactivate = false;
    bool mCameraSelfCalib = true;
    bool mCameraFlip = false;
    sl::SENSING_MODE mDepthSensingMode = sl::SENSING_MODE::STANDARD; // Default Sensing mode: SENSING_MODE_STANDARD
    bool mOpenniDepthMode = false; // 16 bit UC data in mm else 32F in m,
    // for more info -> http://www.ros.org/reps/rep-0118.html

    double mZedMinDepth = 0.2;
    double mZedMaxDepth = 10.0;

    bool mSensTimestampSync = true;

    bool mUseOldExtrinsic = false;
    bool mPublishTF = true;
    bool mPublishMapTF = true;
    bool mPoseSmoothing = false;
    bool mAreaMemory = true;
    bool mImuFusion = true;
    bool mFloorAlignment = false;
    bool mTwoDMode = false;
    double mFixedZValue = 0.0;
    std::vector<double> mInitialBasePose = std::vector<double>(6, 0.0);
    bool mInitOdomWithPose = true;
    double mPathPubRate = 2.0;
    int mPathMaxCount = -1;
    bool mPublishPoseCov = true;
    std::string mAreaMemoryDbPath = "";
    bool mMappingEnabled = false;
    bool mObjDetEnabled = false;


    // QoS profiles
    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
    rclcpp::QoS mVideoQos;
    rclcpp::QoS mDepthQos;
    rclcpp::QoS mSensQos;
    rclcpp::QoS mPoseQos;

    // ZED dynamic params
    double mPubFrameRate = 15;
    double mZedImgDownsampleFactor = 1.0;
    int mZedBrightness = 4;
    int mZedContrast = 4;
    int mZedHue = 0;
    int mZedSaturation = 4;
    int mZedSharpness = 4;
    int mZedGamma = 8;
    bool mZedAutoExpGain;
    int mZedGain = 80;
    int mZedExposure = 80;
    bool mZedAutoWB = true;
    int mZedWBTemp = 42;
    int mDepthConf = 50;
    int mDepthTextConf = 100;
    double mDepthDownsampleFactor = 1.0;
    double mDepthPubRate = 15.0;
    double mPcPubRate = 15.0;

    // Frame IDs
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

    std::string mMapFrameId = "map";
    std::string mOdometryFrameId = "odom";
    std::string mBaseFrameId = "base_link";

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

    // Thread Sync
    std::mutex mCamDataMutex;
    bool mTriggerAutoExposure = false;

    // Mats
    int mCamWidth;  // Camera frame width
    int mCamHeight; // Camera frame height
    sl::Resolution mMatResolVideo;
    sl::Resolution mMatResolDepth;

    // initialization Transform listener
    std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> mTfListener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> mStaticTfBroadcaster;

    // Camera IMU transform
    sl::Transform mSlCamImuTransf;

    // ----> Topics (ONLY THOSE NOT CHANGING WHILE NODE RUNS)
    // Camera info
    camInfoMsgPtr mRgbCamInfoMsg;
    camInfoMsgPtr mLeftCamInfoMsg;
    camInfoMsgPtr mRightCamInfoMsg;
    camInfoMsgPtr mRgbCamInfoRawMsg;
    camInfoMsgPtr mLeftCamInfoRawMsg;
    camInfoMsgPtr mRightCamInfoRawMsg;
    camInfoMsgPtr mDepthCamInfoMsg;
    camInfoMsgPtr mConfidenceCamInfoMsg;

    transfMsgPtr mCameraImuTransfMgs;
    // <---- Topics

    // ----> Publishers
    image_transport::CameraPublisher mPubRgb; //
    image_transport::CameraPublisher mPubRawRgb; //
    image_transport::CameraPublisher mPubLeft; //
    image_transport::CameraPublisher mPubRawLeft; //
    image_transport::CameraPublisher mPubRight; //
    image_transport::CameraPublisher mPubRawRight; //
    image_transport::CameraPublisher mPubDepth; //
    image_transport::Publisher mPubStereo; //
    image_transport::Publisher mPubRawStereo; //

    image_transport::CameraPublisher mPubRgbGray; //
    image_transport::CameraPublisher mPubRawRgbGray; //
    image_transport::CameraPublisher mPubLeftGray; //
    image_transport::CameraPublisher mPubRawLeftGray; //
    image_transport::CameraPublisher mPubRightGray; //
    image_transport::CameraPublisher mPubRawRightGray; //

    imagePub mPubConfMap; //
    disparityPub mPubDisparity; //
    pointcloudPub mPubCloud; //
    pointcloudPub mPubFusedCloud; //
    posePub mPubPose; //
    poseCovPub mPubPoseCov; //
    odomPub mPubOdom; //
    pathPub mPubOdomPath; //
    pathPub mPubPosePath; //
    imuPub mPubImu; //
    imuPub mPubImuRaw; //
    tempPub mPubImuTemp; //
    magPub mPubImuMag; //
    pressPub mPubPressure; //
    tempPub mPubTempL; //
    tempPub mPubTempR; //
    transfPub mPubCamImuTransf;
    // <---- Publishers


};

} // namespace stereolabs

#endif // ZED_CAMERA_COMPONENT_HPP
