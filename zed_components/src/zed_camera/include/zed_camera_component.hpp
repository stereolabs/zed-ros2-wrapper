#ifndef ZED_CAMERA_COMPONENT_HPP
#define ZED_CAMERA_COMPONENT_HPP

#include "visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/camera_info.hpp>

#include <sl/Camera.hpp>

namespace stereolabs {

// ----> Typedefs to simplify declarations
typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;
// <---- Typedefs to simplify declarations

class ZedCamera : public rclcpp::Node
{
public:
    ZED_COMPONENTS_PUBLIC
    explicit ZedCamera(const rclcpp::NodeOptions & options);

    virtual ~ZedCamera();

protected:
    void initParameters();

    template<typename T>
    void getParam(std::string paramName, T defValue, T& outVal, std::string log_info=std::string());

    void getGeneralParams();
    void getVideoParams();
    void getDepthParams();
    void getPosTrackingParams();
    void getSensorsParams();

    void setTFCoordFrameNames();

    rcl_interfaces::msg::SetParametersResult paramChangeCallback(std::vector<rclcpp::Parameter> parameters);

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
    int mZedFrameRate = 30;
    std::string mSvoFilepath = "";
    bool mSvoMode = false;
    bool mVerbose = true;
    int mGpuId = -1;
    int mZedResol = 1; // Default resolution: RESOLUTION_HD720
    int mZedQuality = 1; // Default quality: DEPTH_MODE_PERFORMANCE
    bool mDepthStabilization = true;
    int mCamTimeoutSec = 5;
    int mMaxReconnectTemp = 5;
    bool mZedReactivate = false;
    bool mCameraFlip = false;
    int mZedSensingMode = 0; // Default Sensing mode: SENSING_MODE_STANDARD
    bool mOpenniDepthMode = false; // 16 bit UC data in mm else 32F in m,
    // for more info -> http://www.ros.org/reps/rep-0118.html

    double mZedMinDepth = 0.2;

    double mImuPubRate = 500.0;
    bool mImuTimestampSync = true;

    bool mPublishTF = true;
    bool mPublishMapTF = true;
    bool mPoseSmoothing = false;
    bool mSpatialMemory = true;
    bool mFloorAlignment = false;
    bool mTwoDMode = false;
    double mFixedZValue = 0.0;
    std::vector<double> mInitialBasePose;
    bool mInitOdomWithPose = true;
    double mPathPubRate = 2.0;
    int mPathMaxCount = -1;
    bool mPublishPoseCov = true;
    std::string mOdometryDb = "";


    // QoS profiles
    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
    rclcpp::QoS mVideoQos;
    rclcpp::QoS mDepthQos;
    rclcpp::QoS mImuQos;
    rclcpp::QoS mPoseQos;


    // ZED dynamic params
    double mZedMatResizeFactor = 1.0;   // Dynamic...
    int mZedConfidence = 80;            // Dynamic...
    double mZedMaxDepth = 10.0;         // Dynamic...
    bool mZedAutoExposure;              // Dynamic...
    int mZedGain = 80;                  // Dynamic...
    int mZedExposure = 80;              // Dynamic...

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
    int mMatWidth;  // Data width (mCamWidth*mZedMatResizeFactor)
    int mMatHeight; // Data height (mCamHeight*mZedMatResizeFactor)

    // Messages
    // Camera info
    camInfoMsgPtr mRgbCamInfoMsg;
    camInfoMsgPtr mLeftCamInfoMsg;
    camInfoMsgPtr mRightCamInfoMsg;
    camInfoMsgPtr mRgbCamInfoRawMsg;
    camInfoMsgPtr mLeftCamInfoRawMsg;
    camInfoMsgPtr mRightCamInfoRawMsg;
    camInfoMsgPtr mDepthCamInfoMsg;
    camInfoMsgPtr mConfidenceCamInfoMsg;
};

} // namespace stereolabs

#endif // ZED_CAMERA_COMPONENT_HPP
