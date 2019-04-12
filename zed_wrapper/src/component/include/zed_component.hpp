#ifndef ZED_COMPONENT_HPP
#define ZED_COMPONENT_HPP

// /////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// /////////////////////////////////////////////////////////////////////////

#include "visibility_control.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Services (defined in the package "stereolabs_zed_interfaces")
#include "stereolabs_zed_interfaces/srv/reset_odometry.hpp"
#include "stereolabs_zed_interfaces/srv/restart_tracking.hpp"
#include "stereolabs_zed_interfaces/srv/set_pose.hpp"
#include "stereolabs_zed_interfaces/srv/start_svo_recording.hpp"
#include "stereolabs_zed_interfaces/srv/stop_svo_recording.hpp"

#include "sl/Camera.hpp"

#include "sl_tools.h"

namespace stereolabs {

    // ----> Typedefs to simplify declarations
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> imagePub;
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>> camInfoPub;
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<stereo_msgs::msg::DisparityImage>> disparityPub;

    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pointcloudPub;

    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imuPub;

    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> posePub;
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>> poseCovPub;
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odomPub;
    typedef std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> pathPub;

    typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;
    typedef std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloudMsgPtr;
    typedef std::shared_ptr<sensor_msgs::msg::Imu> imuMsgPtr;

    typedef std::shared_ptr<geometry_msgs::msg::PoseStamped> poseMsgPtr;
    typedef std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> poseCovMsgPtr;
    typedef std::shared_ptr<nav_msgs::msg::Odometry> odomMsgPtr;
    typedef std::shared_ptr<nav_msgs::msg::Path> pathMsgPtr;

    typedef rclcpp::Service<stereolabs_zed_interfaces::srv::ResetOdometry>::SharedPtr resetOdomSrvPtr;
    typedef rclcpp::Service<stereolabs_zed_interfaces::srv::RestartTracking>::SharedPtr restartTrkSrvPtr;
    typedef rclcpp::Service<stereolabs_zed_interfaces::srv::SetPose>::SharedPtr setPoseSrvPtr;
    typedef rclcpp::Service<stereolabs_zed_interfaces::srv::StartSvoRecording>::SharedPtr startSvoRecSrvPtr;
    typedef rclcpp::Service<stereolabs_zed_interfaces::srv::StopSvoRecording>::SharedPtr stopSvoRecSrvPtr;

    // <---- Typedefs to simplify declarations

    /// ZedCameraComponent inheriting from rclcpp_lifecycle::LifecycleNode
    class ZedCameraComponent : public rclcpp_lifecycle::LifecycleNode {
      public:
        RCLCPP_SMART_PTR_DEFINITIONS(ZedCameraComponent)

        /// Create a new ZedCameraComponent/lifecycle node with the specified name.
        /**
         * \param[in] node_name Name of the node.
         * \param[in] namespace_ Namespace of the node.
         * \param[in] use_intra_process_comms True to use the optimized intra-process communication
         * pipeline to pass messages between nodes in the same process using shared memory.
         */
        ZED_PUBLIC
        explicit ZedCameraComponent(const std::string& node_name = "zed_node",
                                    const std::string& ros_namespace = "zed",
                                    bool intra_process_comms = false);

        /// Create a ZedCameraComponent/lifecycle node based on the node name and a rclcpp::Context.
        /**
         * \param[in] node_name Name of the node.
         * \param[in] namespace_ Namespace of the node.
         * \param[in] context The context for the node (usually represents the state of a process).
         * \param[in] arguments Command line arguments that should apply only to this node.
         * \param[in] initial_parameters a list of initial values for parameters on the node.
         * This can be used to provide remapping rules that only affect one instance.
         * \param[in] use_global_arguments False to prevent node using arguments passed to the process.
         * \param[in] use_intra_process_comms True to use the optimized intra-process communication
         * pipeline to pass messages between nodes in the same process using shared memory.
         * \param[in] start_parameter_services True to setup ROS interfaces for accessing parameters
         * in the node.
         */
        ZED_PUBLIC
        explicit ZedCameraComponent(
            const std::string& node_name,
            const std::string& ros_namespace,
            rclcpp::Context::SharedPtr context,
            const std::vector<std::string>& arguments,
            const std::vector<rclcpp::Parameter>& initial_parameters,
            bool use_global_arguments = true,
            bool use_intra_process_comms = false,
            bool start_parameter_services = true);

        virtual ~ZedCameraComponent();

        /// Transition callback for state error
        /**
        * on_error callback is being called when the lifecycle node
        * enters the "error" state.
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State&
                previous_state);

        /// Transition callback for state shutting down
        /**
        * on_shutdown callback is being called when the lifecycle node
        * enters the "shutting down" state.
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& previous_state);

        /// Transition callback for state configuring
        /**
        * on_configure callback is being called when the lifecycle node
        * enters the "configuring" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "inactive" state or stays
        * in "unconfigured".
        * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
        * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&);

        /// Transition callback for state activating
        /**
        * on_activate callback is being called when the lifecycle node
        * enters the "activating" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "active" state or stays
        * in "inactive".
        * TRANSITION_CALLBACK_SUCCESS transitions to "active"
        * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&);

        /// Transition callback for state deactivating
        /**
        * on_deactivate callback is being called when the lifecycle node
        * enters the "deactivating" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "inactive" state or stays
        * in "active".
        * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
        * TRANSITION_CALLBACK_FAILURE transitions to "active"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&);

        /// Transition callback for state cleaningup
        /**
        * on_cleanup callback is being called when the lifecycle node
        * enters the "cleaningup" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "unconfigured" state or stays
        * in "inactive".
        * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
        * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

        /** \brief Service callback to ResetOdometry service
         *  Odometry is reset to clear drift and odometry frame gets the latest pose
         *  from ZED tracking.
         */
        void on_reset_odometry(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<stereolabs_zed_interfaces::srv::ResetOdometry::Request>  req,
                               std::shared_ptr<stereolabs_zed_interfaces::srv::ResetOdometry::Response> res);

        /** \brief Service callback to RestartTracking service
         *  Tracking is restarted and pose set to the value of the parameter `initial_tracking_pose` or to
         *  the latest value set by the `SetPose` service
         */
        void on_restart_tracking(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<stereolabs_zed_interfaces::srv::RestartTracking::Request>  req,
                                 std::shared_ptr<stereolabs_zed_interfaces::srv::RestartTracking::Response> res);

        /** \brief Service callback to SetPose service
         *  Tracking is restarted and pose set to values passed with the service
         */
        void on_set_pose(const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<stereolabs_zed_interfaces::srv::SetPose::Request>  req,
                         std::shared_ptr<stereolabs_zed_interfaces::srv::SetPose::Response> res);

        /* \brief Service callback to StartSvoRecording service
                */
        void on_start_svo_recording(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<stereolabs_zed_interfaces::srv::StartSvoRecording::Request>  req,
                                    std::shared_ptr<stereolabs_zed_interfaces::srv::StartSvoRecording::Response> res);

        /* \brief Service callback to StopSvoRecording service
         */
        void on_stop_svo_recording(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<stereolabs_zed_interfaces::srv::StopSvoRecording::Request>  req,
                                   std::shared_ptr<stereolabs_zed_interfaces::srv::StopSvoRecording::Response> res);

      protected:
        void zedGrabThreadFunc();
        void pointcloudThreadFunc();
        void zedReconnectThreadFunc();

        void initPublishers();
        void initServices();

        void getGeneralParams();
        void getVideoParams();
        void getDepthParams();
        void getImuParams();
        void getPoseParams();
        void initParameters();

        void publishImages(rclcpp::Time timeStamp);
        void publishDepthData(rclcpp::Time timeStamp);
        void publishOdom(tf2::Transform odom2baseTransf, sl::Pose& slPose, rclcpp::Time t);

        void initTransforms();
        void startTracking();
        bool set_pose(float xt, float yt, float zt, float rr, float pr, float yr);

        void processOdometry();
        void processPose();
        void publishPose();
        void publishMapOdom();

        /** \brief Get the information of the ZED cameras and store them in an
         * information message
         * \param zed : the sl::zed::Camera* pointer to an instance
         * \param left_cam_info_msg : the information message to fill with the left
         * camera informations
         * \param right_cam_info_msg : the information message to fill with the right
         * camera informations
         * \param left_frame_id : the id of the reference frame of the left camera
         * \param right_frame_id : the id of the reference frame of the right camera
         */
        void fillCamInfo(sl::Camera& zed, std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
                         std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg,
                         std::string leftFrameId, std::string rightFrameId,
                         bool rawParam = false);

        /** \brief Publish the informations of a camera with a ros Publisher
         * \param cam_info_msg : the information message to publish
         * \param pub_cam_info : the publisher object to use
         * \param timeStamp : the ros::Time to stamp the message
         */
        void publishCamInfo(camInfoMsgPtr camInfoMsg, camInfoPub pubCamInfo, rclcpp::Time timeStamp);

        /** \brief Publish a cv::Mat image with a ros Publisher
         * \param img : the image to publish
         * \param pub_img : the publisher object to use (different image publishers
         * exist)
         * \param img_frame_id : the id of the reference frame of the image (different
         * image frames exist)
         * \param timeStamp : the ros::Time to stamp the image
         */
        void publishImage(sl::Mat img, imagePub pubImg, std::string imgFrameId, rclcpp::Time timeStamp);

        /** \brief Publish a cv::Mat depth image with a ros Publisher
         * \param depth : the depth image to publish
         * \param timeStamp : the ros::Time to stamp the depth image
         */
        void publishDepth(sl::Mat depth, rclcpp::Time timeStamp);

        /** \brief Publish a cv::Mat disparity image with a ros Publisher
         * \param disparity : the disparity image to publish
         * \param timestamp : the ros::Time to stamp the depth image
         */
        void publishDisparity(sl::Mat disparity, rclcpp::Time timestamp);

        /** \brief Publish a pointCloud with a ros Publisher
         */
        void publishPointCloud();

        /** \brief Callback to publish IMU raw data with a ROS publisher
         */
        void imuPubCallback();

        /** \brief Callback to publish Odometry and Pose paths with a ROS publisher
         */
        void pathPubCallback();

        /** \brief Callback to handle parameters changing
         * \param e : the ros::TimerEvent binded to the callback
         */
        rcl_interfaces::msg::SetParametersResult paramChangeCallback(std::vector<rclcpp::Parameter> parameters);

        /** \brief Utility to retrieve the static transform from Base to Depth Sensor
                 *        from static TF
                 */
        bool getSens2BaseTransform();

        /** \brief Utility to retrieve the static transform from Camera center to Depth Sensor
         *        from static TF
         */
        bool getSens2CameraTransform();

        /** \brief Utility to retrieve the static transform from Base Link to Camera center
         *        from static TF
         */
        bool getCamera2BaseTransform();

      private:
        // Status variables
        uint8_t mPrevTransition = lifecycle_msgs::msg::Transition::TRANSITION_CREATE;

        // Timestamps
        rclcpp::Time mPrevFrameTimestamp;
        rclcpp::Time mFrameTimestamp;
        rclcpp::Time mPointCloudTime;

        // Grab thread
        std::thread mGrabThread;
        bool mThreadStop = false;
        bool mRunGrabLoop = false;

        // Pointcloud thread
        std::thread mPcThread; // Point Cloud thread

        // Reconnect thread
        std::thread mReconnectThread;
        std::mutex mReconnectMutex;
        std::mutex mPosTrkMutex;

        // IMU Timer
        rclcpp::TimerBase::SharedPtr mImuTimer = nullptr;

        // Path Timer
        rclcpp::TimerBase::SharedPtr mPathTimer = nullptr;


        // ZED SDK
        sl::Camera mZed;

        // Params
        sl::InitParameters mZedParams;
        int mZedId = 0;
        int mZedSerialNumber = 0;
        int mZedUserCamModel = 1;   // Camera model set by ROS Param
        sl::MODEL mZedRealCamModel; // Camera model requested to SDK
        int mZedFrameRate = 30;
        std::string mSvoFilepath = "";
        bool mSvoMode = false;
        bool mVerbose = true;
        int mGpuId = -1;
        int mZedResol = 2; // Default resolution: RESOLUTION_HD720
        int mZedQuality = 1; // Default quality: DEPTH_MODE_PERFORMANCE
        int mDepthStabilization = 1;
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
        std::string mWorldFrameId = "map";
        std::string mMapFrameId = "map";
        std::string mOdomFrameId = "odom";
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
        rmw_qos_profile_t mVideoQos = rmw_qos_profile_default;
        rmw_qos_profile_t mDepthQos = rmw_qos_profile_default;
        rmw_qos_profile_t mImuQos   = rmw_qos_profile_default;
        rmw_qos_profile_t mPoseQos  = rmw_qos_profile_default;


        // ZED dynamic params
        double mZedMatResizeFactor = 1.0;   // Dynamic...
        int mZedConfidence = 80;            // Dynamic...
        double mZedMaxDepth = 10.0;         // Dynamic...
        bool mZedAutoExposure;              // Dynamic...
        int mZedGain = 80;                  // Dynamic...
        int mZedExposure = 80;              // Dynamic...

        // Publishers
        imagePub mPubRgb;
        imagePub mPubRawRgb;
        imagePub mPubLeft;
        imagePub mPubRawLeft;
        imagePub mPubRight;
        imagePub mPubRawRight;
        imagePub mPubDepth;
        imagePub mPubConfImg;
        imagePub mPubConfMap;

        camInfoPub mPubRgbCamInfo;
        camInfoPub mPubRgbCamInfoRaw;
        camInfoPub mPubLeftCamInfo;
        camInfoPub mPubLeftCamInfoRaw;
        camInfoPub mPubRightCamInfo;
        camInfoPub mPubRightCamInfoRaw;
        camInfoPub mPubDepthCamInfo;
        camInfoPub mPubConfidenceCamInfo;

        disparityPub mPubDisparity;

        pointcloudPub mPubPointcloud;

        imuPub mPubImu;
        imuPub mPubImuRaw;

        posePub mPubPose;
        poseCovPub mPubPoseCov;
        odomPub mPubOdom;
        odomPub mPubMapOdom;
        pathPub mPubPathPose;
        pathPub mPubPathOdom;

        // Topics
        std::string mLeftTopicRoot  = "left";
        std::string mRightTopicRoot = "right";
        std::string mRgbTopicRoot   = "rgb";
        std::string mDepthTopicRoot = "depth";
        std::string mConfTopicRoot = "confidence";
        std::string mImuTopicRoot = "imu";

        std::string mLeftTopic = "left";
        std::string mLeftRawTopic;
        std::string mLeftCamInfoTopic;
        std::string mLeftCamInfoRawTopic;
        std::string mRightTopic = "right";
        std::string mRightRawTopic;
        std::string mRightCamInfoTopic;
        std::string mRightCamInfoRawTopic;
        std::string mRgbTopic = "rgb";
        std::string mRgbRawTopic;
        std::string mRgbCamInfoTopic;
        std::string mRgbCamInfoRawTopic;

        std::string mDepthTopic = "depth";
        std::string mDepthCamInfoTopic;
        std::string mConfImgTopic = "confidence_image";
        std::string mConfCamInfoTopic;
        std::string mConfMapTopic = "confidence_map";
        std::string mDispTopic = "disparity/disparity_image";
        std::string mPointcloudTopic = "point_cloud/cloud_registered";
        std::string mImuTopic = "data";
        std::string mImuRawTopic = "data_raw";

        std::string mOdomTopic = "odom";
        std::string mMapOdomTopic = "map2odom";
        std::string mPoseTopic = "pose";
        std::string mPoseCovTopic = "pose_with_covariance";
        std::string mPosePathTopic = "path_pose";
        std::string mOdomPathTopic = "path_odom";


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
        // Pointcloud
        pointcloudMsgPtr mPointcloudMsg;
        // IMU
        imuMsgPtr mImuMsg;
        imuMsgPtr mImuMsgRaw;
        // Pos. Tracking
        poseMsgPtr mPoseMsg;
        poseCovMsgPtr mPoseCovMsg;
        odomMsgPtr mOdomMsg;
        pathMsgPtr mPathPoseMsg;
        pathMsgPtr mPathOdomMsg;

        // Frame IDs
        std::string mRightCamFrameId = "zed_right_camera_frame";
        std::string mRightCamOptFrameId = "zed_right_camera_optical_frame";
        std::string mLeftCamFrameId = "zed_left_camera_frame";
        std::string mLeftCamOptFrameId = "zed_left_camera_optical_frame";
        std::string mDepthFrameId;
        std::string mDepthOptFrameId;

        std::string mBaseFrameId = "base_link";
        std::string mCameraFrameId = "zed_camera_center";

        std::string mImuFrameId = "zed_imu_link";

        // SL Pointcloud
        sl::Mat mCloud;

        // Mats
        int mCamWidth;  // Camera frame width
        int mCamHeight; // Camera frame height
        int mMatWidth;  // Data width (mCamWidth*mZedMatResizeFactor)
        int mMatHeight; // Data height (mCamHeight*mZedMatResizeFactor)

        // Thread Sync
        std::mutex mImuMutex;
        std::mutex mCamDataMutex;
        std::mutex mPcMutex;
        std::mutex mRecMutex;
        std::condition_variable mPcDataReadyCondVar;
        bool mPcDataReady = false;
        bool mTriggerAutoExposure = false;

        // Diagnostic
        // TODO Publish Diagnostic when available
        std::unique_ptr<sl_tools::CSmartMean> mElabPeriodMean_sec;
        std::unique_ptr<sl_tools::CSmartMean> mGrabPeriodMean_usec;
        std::unique_ptr<sl_tools::CSmartMean> mPcPeriodMean_usec;
        std::unique_ptr<sl_tools::CSmartMean> mImuPeriodMean_usec;

        //Tracking variables
        sl::Pose mLastZedPose; // Sensor to Map transform
        sl::Transform mInitialPoseSl;
        std::vector<geometry_msgs::msg::PoseStamped> mOdomPath;
        std::vector<geometry_msgs::msg::PoseStamped> mMapPath;
        bool mTrackingActivated = false;
        bool mTrackingReady = false;
        sl::TRACKING_STATE mTrackingStatus;
        bool mResetOdom = false;

        // TF Transforms
        tf2::Transform mMap2OdomTransf;         // Coordinates of the odometry frame in map frame
        tf2::Transform mOdom2BaseTransf;        // Coordinates of the base in odometry frame
        tf2::Transform mMap2BaseTransf;         // Coordinates of the base in map frame
        tf2::Transform mMap2CameraTransf;       // Coordinates of the camera in base frame
        tf2::Transform mSensor2BaseTransf;      // Coordinates of the base frame in sensor frame
        tf2::Transform mSensor2CameraTransf;    // Coordinates of the camera frame in sensor frame
        tf2::Transform mCamera2BaseTransf;      // Coordinates of the base frame in camera frame

        bool mSensor2BaseTransfValid = false;
        bool mSensor2CameraTransfValid = false;
        bool mCamera2BaseTransfValid = false;

        // initialization Transform listener
        std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> mTfListener;

        // SVO recording
        bool mRecording = false;
        sl::RecordingState mRecState;

        // Services
        resetOdomSrvPtr mResetOdomSrv;
        restartTrkSrvPtr mRestartTrkSrv;
        setPoseSrvPtr mSetPoseSrv;
        startSvoRecSrvPtr mStartSvoRecSrv;
        stopSvoRecSrvPtr mStopSvoRecSrv;
    };
}

#endif
