#include "zed_camera_component.hpp"
#include "sl_tools.h"
#include <type_traits>

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::placeholders;

namespace stereolabs {

ZedCamera::ZedCamera(const rclcpp::NodeOptions &options)
    : Node("zed_node", options)
    , mVideoQos(10)
    , mDepthQos(10)
    , mSensQos(10)
    , mPoseQos(10) {

    std::string ros_namespace = get_namespace();
    std::string node_name = get_name();

    RCLCPP_INFO(get_logger(), "*******************************");
    RCLCPP_INFO(get_logger(), " ZED Camera Component created");
    RCLCPP_INFO(get_logger(), "  * namespace: %s", get_namespace());
    RCLCPP_INFO(get_logger(), "  * node name: %s", get_name());
    RCLCPP_INFO(get_logger(), "********************************");

    // ----> Parameters initialization
    getParam( "general.debug_mode_active", mDebugMode, mDebugMode );
    if(mDebugMode) {
        std::string logger = ros_namespace.empty() ? "" : ros_namespace + ".";
        logger += node_name;
        rcutils_ret_t res = rcutils_logging_set_logger_level(logger.c_str(), RCUTILS_LOG_SEVERITY_DEBUG);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting DEBUG logger");
        } else {
            RCLCPP_INFO(get_logger(), "*** Debug Mode enabled ***");
        }
    }

    RCLCPP_DEBUG(get_logger(), "[ROS2] Using RMW_IMPLEMENTATION = %s", rmw_get_implementation_identifier());

    initParameters();
    // <---- Parameters initialization

    // Start camera
    startCamera();
}

ZedCamera::~ZedCamera() {

}

template<typename T>
void ZedCamera::getParam(std::string paramName, T defValue, T& outVal, std::string log_info )
{
    declare_parameter(paramName, rclcpp::ParameterValue(defValue) );

    if(!get_parameter(paramName, outVal)) {
        RCLCPP_WARN_STREAM(get_logger(), "The parameter '" << paramName << "' is not available or is not valid, using the default value: " <<  defValue);
    }

    if( !log_info.empty() ) {
        RCLCPP_INFO_STREAM(get_logger(),  log_info << outVal );
    }
}

void ZedCamera::initParameters() {
    // GENERAL parameters
    getGeneralParams();

    // VIDEO parameters
    getVideoParams();

    // DEPTH parameters
    getDepthParams();

    // POS. TRACKING parameters
    getPosTrackingParams();

    // SENSORS parameters
    if(mZedUserCamModel != sl::MODEL::ZED) {
        //getSensorsParams();
    }

    // TODO MAPPING PARAMETERS

    // TODO OD PARAMETERS

    // Dynamic parameters callback
    set_on_parameters_set_callback(std::bind(&ZedCamera::paramChangeCallback, this, _1));
}

void ZedCamera::getGeneralParams() {
    rclcpp::Parameter paramVal;
    std::string paramName;

    RCLCPP_INFO(get_logger(), "*** GENERAL parameters ***");

    std::string camera_model = "zed";
    getParam( "general.camera_model", camera_model, camera_model );
    if (camera_model == "zed") {
        mZedUserCamModel = sl::MODEL::ZED;
    } else if (camera_model == "zedm") {
        mZedUserCamModel = sl::MODEL::ZED_M;
    } else if (camera_model == "zed2") {
        mZedUserCamModel = sl::MODEL::ZED2;
    } else {
        RCLCPP_ERROR_STREAM(get_logger(), "Camera model not valid in parameter values: " << camera_model);
    }
    RCLCPP_INFO(get_logger(), " * Camera model: %s (%s)", camera_model.c_str(),
                sl::toString(static_cast<sl::MODEL>(mZedUserCamModel)).c_str());

    getParam( "general.sdk_verbose", mVerbose, mVerbose,  " * SDK Verbose: ");
    getParam( "general.svo_file", std::string(), mSvoFilepath, " * SVO: ");
    getParam( "general.camera_name", mCameraName, mCameraName,  " * Camera name: ");
    getParam( "general.zed_id", mZedId, mZedId,  " * Camera ID: ");
    getParam( "general.serial_number", mZedSerialNumber, mZedSerialNumber,  " * Camera SN: ");
    getParam( "general.camera_timeout_sec", mCamTimeoutSec, mCamTimeoutSec,  " * Camera timeout [sec]: ");
    getParam( "general.camera_reactivate", mZedReactivate, mZedReactivate,  " * Camera reconnection if disconnected: ");
    getParam( "general.camera_max_reconnect", mMaxReconnectTemp, mMaxReconnectTemp,  " * Camera reconnection temptatives: ");
    getParam( "general.grab_frame_rate", mZedFrameRate, mZedFrameRate,  " * Camera framerate: ");
    getParam( "general.gpu_id", mGpuId, mGpuId,  " * GPU ID: ");
    getParam( "general.base_frame", mBaseFrameId, mBaseFrameId,  " * Base frame id: ");

    // TODO ADD SVO SAVE COMPRESSION PARAMETERS

    int resol = static_cast<int>(mZedResol);
    getParam( "general.resolution", resol, resol );
    mZedResol = static_cast<sl::RESOLUTION>(resol);
    RCLCPP_INFO(get_logger(), " * Camera resolution: %d (%s)", resol, sl::toString(mZedResol).c_str());

    getParam( "general.self_calib", mCameraSelfCalib, mCameraSelfCalib );
    RCLCPP_INFO(get_logger(), " * Camera self calibration: %s", mCameraSelfCalib?"TRUE":"FALSE");
    getParam( "general.camera_flip", mCameraFlip, mCameraFlip );
    RCLCPP_INFO(get_logger(), " * Camera flip: %s", mCameraFlip?"TRUE":"FALSE");

    // Dynamic parameters


    getParam( "general.pub_frame_rate", mPubFrameRate, mPubFrameRate );
    if( mPubFrameRate>mZedFrameRate )
    {
        RCLCPP_WARN(get_logger(), "'pub_frame_rate' cannot be bigger than 'grab_frame_rate'", paramName.c_str());
    }
    RCLCPP_INFO(get_logger(), " * [DYN] Publish framerate [Hz]: %g ", mPubFrameRate);
}

void ZedCamera::getVideoParams() {
    rclcpp::Parameter paramVal;
    std::string paramName;

    RCLCPP_INFO(get_logger(), "*** VIDEO parameters ***");

    rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    size_t qos_depth = 10;
    rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    getParam( "video.extrinsic_in_camera_frame", mUseOldExtrinsic, mUseOldExtrinsic,  " * Use old extrinsic parameters: ");


    getParam( "video.img_downsample_factor", mZedImgDownsampleFactor, mZedImgDownsampleFactor );
    if (mZedImgDownsampleFactor < 0.1) {
        mZedImgDownsampleFactor = 0.1;
        RCLCPP_WARN(get_logger(), "The minimum value allowed for '%s' is 0.1", paramName.c_str());
    } else if (mZedImgDownsampleFactor > 1.0) {
        mZedImgDownsampleFactor = 1.0;
        RCLCPP_WARN(get_logger(), "The maximum value allowed for '%s' is 1.0", paramName.c_str());
    }
    RCLCPP_INFO(get_logger(), " * [DYN] Image downsample factor: %g ", mZedImgDownsampleFactor);

    getParam( "video.brightness", mZedBrightness, mZedBrightness,  " * [DYN] Brightness: ");
    getParam( "video.contrast", mZedContrast, mZedContrast,  " * [DYN] Contrast: ");
    getParam( "video.hue", mZedHue, mZedHue,  " * [DYN] Hue: ");
    getParam( "video.saturation", mZedSaturation, mZedSaturation,  " * [DYN] Saturation: ");
    getParam( "video.sharpness", mZedSharpness, mZedSharpness,  " * [DYN] Sharpness: ");
    getParam( "video.gamma", mZedGamma, mZedGamma,  " * [DYN] Gamma: ");
    getParam( "video.auto_exposure_gain", mZedAutoExpGain, mZedAutoExpGain);
    RCLCPP_INFO(get_logger(), " * [DYN] Auto Exposure/Gain: %s", mZedAutoExpGain?"TRUE":"FALSE");
    if (mZedAutoExpGain) {
        mTriggerAutoExposure = true;
    }
    getParam( "video.exposure", mZedExposure, mZedExposure,  " * [DYN] Exposure: ");
    getParam( "video.gain", mZedGain, mZedGain,  " * [DYN] Gain: ");
    getParam( "video.auto_whitebalance", mZedAutoWB, mZedAutoWB);
    RCLCPP_INFO(get_logger(), " * [DYN] Auto White Balance: %s", mZedAutoWB?"TRUE":"FALSE");
    getParam( "video.whitebalance_temperature", mZedWBTemp, mZedWBTemp,  " * [DYN] White Balance Temperature: ");


    // ------------------------------------------

    paramName = "video.qos_history";
    declare_parameter(paramName, rclcpp::ParameterValue(0) );

    if (get_parameter(paramName, paramVal)) {
        qos_hist = paramVal.as_int() == 0 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        mVideoQos.history(qos_hist);
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Video QoS History: '%s'", sl_tools::qos2str(qos_hist).c_str());

    // ------------------------------------------

    paramName = "video.qos_depth";
    declare_parameter(paramName, rclcpp::ParameterValue(10) );

    if (get_parameter(paramName, paramVal)) {
        qos_depth = paramVal.as_int();
        mVideoQos.keep_last( qos_depth );
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Video QoS History depth: '%d'", qos_depth);

    // ------------------------------------------

    paramName = "video.qos_reliability";
    declare_parameter(paramName, rclcpp::ParameterValue(0) );

    if (get_parameter(paramName, paramVal)) {
        qos_reliability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT :
                                                   RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        mVideoQos.reliability(qos_reliability);
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Video QoS Reliability: '%s'", sl_tools::qos2str(qos_reliability).c_str());

    // ------------------------------------------

    paramName = "video.qos_durability";
    declare_parameter(paramName, rclcpp::ParameterValue(0) );

    if (get_parameter(paramName, paramVal)) {
        qos_durability= paramVal.as_int() == 0 ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL :
                                                 RMW_QOS_POLICY_DURABILITY_VOLATILE;
        mVideoQos.durability(qos_durability);
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Video QoS Durability: '%s'", sl_tools::qos2str(qos_durability).c_str());
}

void ZedCamera::getDepthParams() {
    rclcpp::Parameter paramVal;
    std::string paramName;

    rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    size_t qos_depth = 10;
    rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    RCLCPP_INFO(get_logger(), "*** DEPTH parameters ***");

    getParam( "depth.depth_downsample_factor", mDepthDownsampleFactor, mDepthDownsampleFactor );
    if (mDepthDownsampleFactor < 0.1) {
        mDepthDownsampleFactor = 0.1;
        RCLCPP_WARN(get_logger(), "The minimum value allowed for '%s' is 0.1", paramName.c_str());
    } else if (mDepthDownsampleFactor > 1.0) {
        mDepthDownsampleFactor = 1.0;
        RCLCPP_WARN(get_logger(), "The maximum value allowed for '%s' is 1.0", paramName.c_str());
    }
    RCLCPP_INFO(get_logger(), " * Depth downsample factor: %g ", mDepthDownsampleFactor);

    int depth_quality = static_cast<int>(mDepthQuality);
    getParam( "depth.quality", depth_quality, depth_quality );
    mDepthQuality = static_cast<sl::DEPTH_MODE>(depth_quality);
    RCLCPP_INFO(get_logger(), " * Depth quality: %d (%s)", depth_quality, sl::toString(mDepthQuality).c_str());

    getParam( "depth.min_depth", mZedMinDepth, mZedMinDepth, " * Min depth [m]: ");
    getParam( "depth.max_depth", mZedMaxDepth, mZedMaxDepth, " * Max depth [m]: ");

    int sens_mode = static_cast<int>(mDepthSensingMode);
    getParam( "depth.sensing_mode", sens_mode, sens_mode );
    mDepthSensingMode = static_cast<sl::SENSING_MODE>(sens_mode);
    RCLCPP_INFO(get_logger(), " * Depth Sensing Mode: %d (%s)", sens_mode, sl::toString(mDepthSensingMode).c_str());

    getParam( "depth.depth_stabilization", mDepthStabilization, mDepthStabilization );
    RCLCPP_INFO(get_logger(), " * Depth Stabilization: %s", mDepthStabilization?"TRUE":"FALSE");

    getParam( "depth.openni_depth_mode", mOpenniDepthMode, mOpenniDepthMode );
    RCLCPP_INFO(get_logger(), " * OpenNI mode (16bit point cloud): %s", mOpenniDepthMode?"TRUE":"FALSE");

    getParam( "depth.point_cloud_freq", mPcPubRate, mPcPubRate, " * [DYN] Point cloud rate [Hz]: " );

    getParam( "depth.depth_confidence", mDepthConf, mDepthConf, " * [DYN] Depth Confidence: " );
    getParam( "depth.depth_texture_conf", mDepthTextConf, mDepthTextConf, " * [DYN] Depth Texture Confidence: " );

    // ------------------------------------------

    paramName = "depth.qos_history";
    declare_parameter(paramName, rclcpp::ParameterValue(0) );

    if (get_parameter(paramName, paramVal)) {
        qos_hist = paramVal.as_int() == 0 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        mDepthQos.history(qos_hist);
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Depth QoS History: '%s'", sl_tools::qos2str(qos_hist).c_str());

    // ------------------------------------------

    paramName = "depth.qos_depth";
    declare_parameter(paramName, rclcpp::ParameterValue(10) );

    if (get_parameter(paramName, paramVal)) {
        qos_depth  = paramVal.as_int();
        mDepthQos.keep_last(qos_depth);
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Depth QoS History depth: '%d'", qos_depth);

    // ------------------------------------------

    paramName = "depth.qos_reliability";
    declare_parameter(paramName, rclcpp::ParameterValue(0) );

    if (get_parameter(paramName, paramVal)) {
        qos_reliability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT :
                                                   RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        mDepthQos.reliability(qos_reliability);
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Depth QoS Reliability: '%s'", sl_tools::qos2str(qos_reliability).c_str());

    // ------------------------------------------

    paramName = "depth.qos_durability";
    declare_parameter(paramName, rclcpp::ParameterValue(0) );

    if (get_parameter(paramName, paramVal)) {
        qos_durability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL :
                                                  RMW_QOS_POLICY_DURABILITY_VOLATILE;
        mDepthQos.durability(qos_durability);
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Depth QoS Durability: '%s'", sl_tools::qos2str(qos_durability).c_str());
}

void ZedCamera::getPosTrackingParams() {
    rclcpp::Parameter paramVal;
    std::string paramName;

    rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    size_t qos_depth = 10;
    rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    RCLCPP_INFO(get_logger(), "*** POSITIONAL TRACKING parameters ***");

    getParam( "pos_tracking.path_pub_rate", mPathPubRate, mPathPubRate, " * [DYN] Path publishing rate: " );
    getParam( "pos_tracking.path_max_count", mPathMaxCount, mPathMaxCount );
    if (mPathMaxCount < 2 && mPathMaxCount != -1) {
        mPathMaxCount = 2;
    }
    RCLCPP_INFO_STREAM(get_logger(), " * Path history lenght: " << mPathMaxCount);

    paramName = "pos_tracking.initial_base_pose";
    declare_parameter(paramName, rclcpp::ParameterValue(mInitialBasePose) );
    if(!get_parameter(paramName, mInitialBasePose)) {
        RCLCPP_WARN_STREAM(get_logger(), "The parameter '" << paramName << "' is not available or is not valid, using the default value");
        mInitialBasePose = std::vector<double>(6, 0.0);
    }
    if(mInitialBasePose.size()<6)
    {
        RCLCPP_WARN_STREAM(get_logger(), "The parameter '" << paramName << "' must be a vector of 6 values of double type");
        mInitialBasePose = std::vector<double>(6, 0.0);
    }
    RCLCPP_INFO(get_logger(), " * Initial pose: [%g,%g,%g,%g,%g,%g,]",
                mInitialBasePose[0],mInitialBasePose[1],mInitialBasePose[2],
            mInitialBasePose[3],mInitialBasePose[4],mInitialBasePose[5]);

    getParam( "pos_tracking.area_memory", mAreaMemory, mAreaMemory );
    RCLCPP_INFO_STREAM(get_logger(), " * Area Memory: " << (mAreaMemory?"TRUE":"FALSE") );
    getParam( "pos_tracking.area_memory_db_path", mAreaMemoryDbPath, mAreaMemoryDbPath, " * Area Memory DB: " );
    if(mZedUserCamModel != sl::MODEL::ZED) {
        getParam( "pos_tracking.imu_fusion", mImuFusion, mImuFusion );
        RCLCPP_INFO_STREAM(get_logger(), " * IMU Fusion: " << (mImuFusion?"TRUE":"FALSE") );
    }
    getParam( "pos_tracking.floor_alignment", mFloorAlignment, mFloorAlignment );
    RCLCPP_INFO_STREAM(get_logger(), " * Floor Alignment: " << (mFloorAlignment?"TRUE":"FALSE") );
    getParam( "pos_tracking.init_odom_with_first_valid_pose", mInitOdomWithPose, mInitOdomWithPose );
    RCLCPP_INFO_STREAM(get_logger(), " * Init Odometry with first valid pose data: " << (mInitOdomWithPose?"TRUE":"FALSE") );
    getParam( "pos_tracking.two_d_mode", mTwoDMode, mTwoDMode );
    RCLCPP_INFO_STREAM(get_logger(), " * 2D mode: " << (mTwoDMode?"TRUE":"FALSE") );
    if(mTwoDMode) {
        getParam( "pos_tracking.fixed_z_value", mFixedZValue, mFixedZValue, " * Fixed Z value: " );
    }

    // ------------------------------------------

    paramName = "tracking.qos_history";
    declare_parameter(paramName, rclcpp::ParameterValue(0) );

    if (get_parameter(paramName, paramVal)) {
        qos_hist = paramVal.as_int() == 0 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        mPoseQos.history(qos_hist);
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Pose/Odometry QoS History: '%s'", sl_tools::qos2str(qos_hist).c_str());

    // ------------------------------------------

    paramName = "tracking.qos_depth";
    declare_parameter(paramName, rclcpp::ParameterValue(10) );

    if (get_parameter(paramName, paramVal)) {
        qos_depth = paramVal.as_int();
        mPoseQos.keep_last(qos_depth);
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Pose/Odometry QoS History depth: '%d'", qos_depth);

    // ------------------------------------------

    paramName = "tracking.qos_reliability";
    declare_parameter(paramName, rclcpp::ParameterValue(0) );

    if (get_parameter(paramName, paramVal)) {
        qos_reliability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT :
                                                   RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        mPoseQos.reliability(qos_reliability);
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Pose/Odometry QoS Reliability: '%s'", sl_tools::qos2str(qos_reliability).c_str());

    // ------------------------------------------

    paramName = "tracking.qos_durability";
    declare_parameter(paramName, rclcpp::ParameterValue(0) );

    if (get_parameter(paramName, paramVal)) {
        qos_durability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL :
                                                  RMW_QOS_POLICY_DURABILITY_VOLATILE;
        mPoseQos.durability(qos_durability);
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Pose/Odometry QoS Durability: '%s'", sl_tools::qos2str(qos_durability).c_str());

}

rcl_interfaces::msg::SetParametersResult ZedCamera::paramChangeCallback(std::vector<rclcpp::Parameter> parameters) {

    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = false;

    //    for (size_t i = 0; i < parameters.size(); i++) {
    //        rclcpp::Parameter param = parameters[i];

    //        if (param.get_name() == "general.mat_resize_factor") {
    //            if (param.get_type() == rclcpp::PARAMETER_DOUBLE) {

    //                double new_val = param.as_double();

    //                if (new_val > 0.01 && new_val <= 1.0) {
    //                    mZedImgDownsampleFactor = new_val;
    //                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %g", param.get_name().c_str(), mZedImgDownsampleFactor);
    //                    result.successful = true;

    //                    // ----> Modify data sizes
    //                    mCamDataMutex.lock();
    //                    mMatWidth = static_cast<size_t>(mCamWidth * mZedImgDownsampleFactor);
    //                    mMatHeight = static_cast<size_t>(mCamHeight * mZedImgDownsampleFactor);
    //                    RCLCPP_INFO(get_logger(), "Data Mat size : %d x %d", sl::Resolution(mMatWidth, mMatHeight));

    //                    // Update Camera Info
    //                    fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId, mRightCamOptFrameId);
    //                    fillCamInfo(mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId, mRightCamOptFrameId, true);
    //                    mRgbCamInfoMsg = mDepthCamInfoMsg = mLeftCamInfoMsg;
    //                    mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
    //                    mCamDataMutex.unlock();
    //                    // <---- Modify data sizes
    //                } else {
    //                    RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT value in the range ]0.0,1.0]",
    //                                param.get_name().c_str());
    //                    result.successful = false;
    //                    return result;
    //                }
    //            } else {
    //                RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT positive value!", param.get_name().c_str());
    //                result.successful = false;
    //                return result;
    //            }
    //        } else if (param.get_name() == "video.auto_exposure_gain") {
    //            if (param.get_type() == rclcpp::PARAMETER_BOOL) {

    //                mZedAutoExpGain = param.as_bool();

    //                if (mZedAutoExpGain) {
    //                    mTriggerAutoExposure = true;
    //                }

    //                RCLCPP_INFO(get_logger(), "The param '%s' has changed to %s", param.get_name().c_str(),
    //                            mZedAutoExpGain ? "ENABLED" : "DISABLED");
    //                result.successful = true;
    //            } else {
    //                RCLCPP_WARN(get_logger(), "The param '%s' requires a BOOL value!", param.get_name().c_str());
    //                result.successful = false;
    //                return result;
    //            }
    //        } else if (param.get_name() == "video.exposure") {
    //            if (param.get_type() == rclcpp::PARAMETER_INTEGER) {

    //                int new_val = param.as_int();

    //                if (new_val > 0 && new_val <= 100) {
    //                    mZedExposure = new_val;
    //                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %d", param.get_name().c_str(), mZedExposure);
    //                    result.successful = true;
    //                } else {
    //                    RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value in the range ]0,100]", param.get_name().c_str());
    //                    result.successful = false;
    //                    return result;
    //                }
    //            } else {
    //                RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value!", param.get_name().c_str());
    //                result.successful = false;
    //                return result;
    //            }
    //        } else if (param.get_name() == "video.gain") {
    //            if (param.get_type() == rclcpp::PARAMETER_INTEGER) {

    //                int new_val = param.as_int();

    //                if (new_val > 0 && new_val <= 100) {
    //                    mZedGain = new_val;
    //                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %d", param.get_name().c_str(), mZedGain);
    //                    result.successful = true;
    //                } else {
    //                    RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value in the range ]0,100]", param.get_name().c_str());
    //                    result.successful = false;
    //                    return result;
    //                }
    //            } else {
    //                RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value!", param.get_name().c_str());
    //                result.successful = false;
    //                return result;
    //            }
    //        } else if (param.get_name() == "depth.confidence") {
    //            if (param.get_type() == rclcpp::PARAMETER_INTEGER) {
    //                int new_val = param.as_int();

    //                if (new_val > 0 && new_val <= 100) {
    //                    mDepthConf = new_val;
    //                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %d", param.get_name().c_str(), mDepthConf);
    //                    result.successful = true;
    //                } else {
    //                    RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value in the range ]0,100]", param.get_name().c_str());
    //                    result.successful = false;
    //                    return result;
    //                }
    //            } else {
    //                RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value!", param.get_name().c_str());
    //                result.successful = false;
    //                return result;
    //            }
    //        } else if (param.get_name() == "depth.max_depth") {
    //            if (param.get_type() == rclcpp::PARAMETER_DOUBLE) {

    //                double new_val = param.as_double();

    //                if (new_val > 0) {
    //                    mZedMaxDepth = new_val;
    //                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %g", param.get_name().c_str(), mZedMaxDepth);
    //                    result.successful = true;
    //                } else {
    //                    RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT positive value", param.get_name().c_str());
    //                    result.successful = false;
    //                    return result;
    //                }
    //            } else {
    //                RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT positive value!", param.get_name().c_str());
    //                result.successful = false;
    //                return result;
    //            }
    //        } else {
    //            RCLCPP_WARN(get_logger(), "The param '%s' cannot be dinamically changed!", param.get_name().c_str());
    //            result.successful = false;
    //            return result;
    //        }
    //    }

    return result;
}



void ZedCamera::setTFCoordFrameNames()
{
    // ----> Coordinate frames
    mCameraFrameId = mCameraName + "_camera_center";
    mLeftCamFrameId = mCameraName + "_left_camera_frame";
    mLeftCamOptFrameId = mCameraName + "_left_camera_optical_frame";
    mRightCamFrameId = mCameraName + "_right_camera_frame";
    mRightCamOptFrameId = mCameraName + "_right_camera_optical_frame";

    mImuFrameId = mCameraName + "_imu_link";
    mBaroFrameId = mCameraName + "_baro_link";
    mMagFrameId = mCameraName + "_mag_link";
    mTempLeftFrameId = mCameraName + "_temp_left_link";
    mTempRightFrameId = mCameraName + "_temp_right_link";

    mDepthFrameId = mLeftCamFrameId;
    mDepthOptFrameId = mLeftCamOptFrameId;

    // Note: Depth image frame id must match color image frame id
    mCloudFrameId = mDepthOptFrameId;
    mRgbFrameId = mDepthFrameId;
    mRgbOptFrameId = mCloudFrameId;
    mDisparityFrameId = mDepthFrameId;
    mDisparityOptFrameId = mDepthOptFrameId;
    mConfidenceFrameId = mDepthFrameId;
    mConfidenceOptFrameId = mDepthOptFrameId;

    // Print TF frames
    RCLCPP_INFO_STREAM(get_logger(), "*** TF FRAMES ***");
    RCLCPP_INFO_STREAM(get_logger(), " * Map\t\t\t-> " << mMapFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Odometry\t\t-> " << mOdometryFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Base\t\t\t-> " << mBaseFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Camera\t\t-> " << mCameraFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Left\t\t\t-> " << mLeftCamFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Left Optical\t\t-> " << mLeftCamOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * RGB\t\t\t-> " << mRgbFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * RGB Optical\t\t-> " << mRgbFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Right\t\t\t-> " << mRightCamFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Right Optical\t\t-> " << mRightCamOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Depth\t\t\t-> " << mDepthFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Depth Optical\t\t-> " << mDepthOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Point Cloud\t\t-> " << mCloudFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Disparity\t\t-> " << mDisparityFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Disparity Optical\t-> " << mDisparityOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Confidence\t\t-> " << mConfidenceFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Confidence Optical\t-> " << mConfidenceOptFrameId);
    if(mZedRealCamModel!=sl::MODEL::ZED)
    {
        RCLCPP_INFO_STREAM(get_logger(), " * IMU\t\t\t\t-> " << mImuFrameId);

        if(mZedUserCamModel==sl::MODEL::ZED2)
        {
            RCLCPP_INFO_STREAM(get_logger(), " * Barometer\t\t\t\t-> " << mBaroFrameId);
            RCLCPP_INFO_STREAM(get_logger(), " * Magnetometer\t\t\t\t-> " << mMagFrameId);
            RCLCPP_INFO_STREAM(get_logger(), " * Left Temperature\t\t\t\t-> " << mTempLeftFrameId);
            RCLCPP_INFO_STREAM(get_logger(), " * Right Temperature\t\t\t\t-> " << mTempRightFrameId);
        }
    }
    // <---- Coordinate frames
}

void ZedCamera::fillCamInfo(sl::Camera& zed, std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
                            std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg,
                            std::string leftFrameId, std::string rightFrameId,
                            bool rawParam /*= false*/) {
    sl::CalibrationParameters zedParam;

#if ZED_SDK_MAJOR_VERSION==3 && ZED_SDK_MINOR_VERSION<1
    if (rawParam) {
        zedParam = zed.getCameraInformation(mMatResolVideo).calibration_parameters_raw; // ok
    } else {
        zedParam = zed.getCameraInformation(mMatResolVideo).calibration_parameters; // ok
    }
#else
    if (rawParam) {
        zedParam = zed.getCameraInformation(mMatResolVideo).camera_configuration.calibration_parameters_raw;
    } else {
        zedParam = zed.getCameraInformation(mMatResolVideo).camera_configuration.calibration_parameters;
    }
#endif

    float baseline = zedParam.getCameraBaseline();
    leftCamInfoMsg->distortion_model =
            sensor_msgs::distortion_models::PLUMB_BOB;
    rightCamInfoMsg->distortion_model =
            sensor_msgs::distortion_models::PLUMB_BOB;
    leftCamInfoMsg->d.resize(5);
    rightCamInfoMsg->d.resize(5);
    leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];   // k1
    leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];   // k2
    leftCamInfoMsg->d[2] = zedParam.left_cam.disto[4];   // k3
    leftCamInfoMsg->d[3] = zedParam.left_cam.disto[2];   // p1
    leftCamInfoMsg->d[4] = zedParam.left_cam.disto[3];   // p2
    rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0]; // k1
    rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1]; // k2
    rightCamInfoMsg->d[2] = zedParam.right_cam.disto[4]; // k3
    rightCamInfoMsg->d[3] = zedParam.right_cam.disto[2]; // p1
    rightCamInfoMsg->d[4] = zedParam.right_cam.disto[3]; // p2
    leftCamInfoMsg->k.fill(0.0);
    rightCamInfoMsg->k.fill(0.0);
    leftCamInfoMsg->k[0] = static_cast<double>(zedParam.left_cam.fx);
    leftCamInfoMsg->k[2] = static_cast<double>(zedParam.left_cam.cx);
    leftCamInfoMsg->k[4] = static_cast<double>(zedParam.left_cam.fy);
    leftCamInfoMsg->k[5] = static_cast<double>(zedParam.left_cam.cy);
    leftCamInfoMsg->k[8] = 1.0;
    rightCamInfoMsg->k[0] = static_cast<double>(zedParam.right_cam.fx);
    rightCamInfoMsg->k[2] = static_cast<double>(zedParam.right_cam.cx);
    rightCamInfoMsg->k[4] = static_cast<double>(zedParam.right_cam.fy);
    rightCamInfoMsg->k[5] = static_cast<double>(zedParam.right_cam.cy);
    rightCamInfoMsg->k[8] = 1.0;
    leftCamInfoMsg->r.fill(0.0);
    rightCamInfoMsg->r.fill(0.0);

    for (size_t i = 0; i < 3; i++) {
        // identity
        rightCamInfoMsg->r[i + i * 3] = 1;
        leftCamInfoMsg->r[i + i * 3] = 1;
    }

#if ZED_SDK_MAJOR_VERSION==3 && ZED_SDK_MINOR_VERSION<1
    if (rawParam) {
        std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
        float* p = R_.data();

        for (int i = 0; i < 9; i++) {
            rightCamInfoMsg->r[i] = p[i];
        }
    }
#else
    if (rawParam) {
        if(mUseOldExtrinsic) { // Camera frame (Z forward, Y down, X right)

            std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
            float* p = R_.data();

            for (int i = 0; i < 9; i++) {
                rightCamInfoMsg->r[i] = p[i];
            }
        } else { // ROS frame (X forward, Z up, Y left)
            for (int i = 0; i < 9; i++) {
                rightCamInfoMsg->r[i] = zedParam.stereo_transform.getRotationMatrix().r[i];
            }
        }
    }
#endif

    leftCamInfoMsg->p.fill(0.0);
    rightCamInfoMsg->p.fill(0.0);
    leftCamInfoMsg->p[0] = static_cast<double>(zedParam.left_cam.fx);
    leftCamInfoMsg->p[2] = static_cast<double>(zedParam.left_cam.cx);
    leftCamInfoMsg->p[5] = static_cast<double>(zedParam.left_cam.fy);
    leftCamInfoMsg->p[6] = static_cast<double>(zedParam.left_cam.cy);
    leftCamInfoMsg->p[10] = 1.0;
    // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
    rightCamInfoMsg->p[3] = static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
    rightCamInfoMsg->p[0] = static_cast<double>(zedParam.right_cam.fx);
    rightCamInfoMsg->p[2] = static_cast<double>(zedParam.right_cam.cx);
    rightCamInfoMsg->p[5] = static_cast<double>(zedParam.right_cam.fy);
    rightCamInfoMsg->p[6] = static_cast<double>(zedParam.right_cam.cy);
    rightCamInfoMsg->p[10] = 1.0;
    leftCamInfoMsg->width = rightCamInfoMsg->width = static_cast<uint32_t>(mMatResolVideo.width);
    leftCamInfoMsg->height = rightCamInfoMsg->height = static_cast<uint32_t>(mMatResolVideo.height);
    leftCamInfoMsg->header.frame_id = leftFrameId;
    rightCamInfoMsg->header.frame_id = rightFrameId;
}

void ZedCamera::initPublishers() {
    RCLCPP_INFO(get_logger(), "*** PUBLISHED TOPICS ***");

    std::string topicPrefix = get_namespace();

    if (topicPrefix.length() > 1) {
        topicPrefix += "/";
    }

    topicPrefix += get_name();
    topicPrefix += "/";

    // ----> Create messages that do not change while running
    mRgbCamInfoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(); // TODO ARE THOSE MESSAGES USED???
    mRgbCamInfoRawMsg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    mLeftCamInfoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    mLeftCamInfoRawMsg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    mRightCamInfoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    mRightCamInfoRawMsg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    mDepthCamInfoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    mConfidenceCamInfoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    mCameraImuTransfMgs = std::make_unique<geometry_msgs::msg::Transform>();
    // <---- Create messages that do not change while running

    // ----> Topics names definition
    std::string root = "~/";
    std::string rgbTopicRoot = "rgb";
    std::string rightTopicRoot = "right";
    std::string leftTopicRoot = "left";
    std::string stereoTopicRoot = "stereo";
    std::string img_topic = "/image_rect_color";
    std::string img_raw_topic = "/image_raw_color";
    std::string img_gray_topic = "/image_rect_gray";
    std::string img_raw_gray_topic_ = "/image_raw_gray";
    std::string raw_suffix = "_raw";
    std::string left_topic = root + leftTopicRoot + img_topic;
    std::string left_raw_topic = root + leftTopicRoot + raw_suffix + img_raw_topic;
    std::string right_topic = root + rightTopicRoot + img_topic;
    std::string right_raw_topic = root + rightTopicRoot + raw_suffix + img_raw_topic;
    std::string rgb_topic = root + rgbTopicRoot + img_topic;
    std::string rgb_raw_topic = root + rgbTopicRoot + raw_suffix + img_raw_topic;
    std::string stereo_topic = root + stereoTopicRoot + img_topic;
    std::string stereo_raw_topic = root + stereoTopicRoot + raw_suffix + img_raw_topic;
    std::string left_gray_topic = root + leftTopicRoot + img_gray_topic;
    std::string left_raw_gray_topic = root + leftTopicRoot + raw_suffix + img_raw_gray_topic_;
    std::string right_gray_topic = root + rightTopicRoot + img_gray_topic;
    std::string right_raw_gray_topic = root + rightTopicRoot + raw_suffix + img_raw_gray_topic_;
    std::string rgb_gray_topic = root + rgbTopicRoot + img_gray_topic;
    std::string rgb_raw_gray_topic = root + rgbTopicRoot + raw_suffix + img_raw_gray_topic_;

    // Set the disparity topic name
    std::string disparity_topic = root + "disparity/disparity_image";

    // Set the depth topic names
    std::string depth_topic_root = "depth";

    if (mOpenniDepthMode) {
        RCLCPP_INFO_STREAM( get_logger(), "Openni depth mode activated -> Units: mm, Encoding: MONO16");
    }
    std::string depth_topic = root + depth_topic_root + "/depth_registered";

    std::string pointcloud_topic = root + "point_cloud/cloud_registered";
    std::string pointcloud_fused_topic = root + "mapping/fused_cloud";

    std::string object_det_topic_root = "obj_det";
    std::string object_det_topic = root + object_det_topic_root + "/objects";
    std::string object_det_rviz_topic = root + object_det_topic_root + "/object_markers";

    std::string confImgRoot = "confidence";
    std::string conf_map_topic_name = "confidence_map";
    std::string conf_map_topic = root + confImgRoot + "/" + conf_map_topic_name;

    // Set the positional tracking topic names
    std::string pose_topic = root + "pose";
    std::string pose_cov_topic;
    pose_cov_topic = root + pose_topic + "_with_covariance";

    std::string odometry_topic = root + "odom";
    std::string odom_path_topic = root + "path_odom";
    std::string map_path_topic = root + "path_map";

    // Set the Sensors topic names
    std::string temp_topic_root = "temperature";
    std::string imuTopicRoot = "imu";
    std::string imu_topic_name = "data";
    std::string imu_topic_raw_name = "data_raw";
    std::string imu_topic_mag_name = "mag";
    //std::string imu_topic_mag_raw_name = "mag_raw";
    std::string pressure_topic_name = "atm_press";

    std::string imu_topic = root + imuTopicRoot + "/" + imu_topic_name;
    std::string imu_topic_raw = root + imuTopicRoot + "/" + imu_topic_raw_name;
    std::string imu_temp_topic = root + temp_topic_root + "/" + imuTopicRoot;
    std::string imu_mag_topic = root + imuTopicRoot + "/" + imu_topic_mag_name;
    //std::string imu_mag_topic_raw = imuTopicRoot + "/" + imu_topic_mag_raw_name;
    std::string pressure_topic = root + /*imuTopicRoot + "/" +*/ pressure_topic_name;
    std::string temp_topic_left = root + temp_topic_root + "/left";
    std::string temp_topic_right = root + temp_topic_root + "/right";
    // <---- Topics names definition

    // ----> Camera publishers
    mPubRgb = image_transport::create_camera_publisher( this, rgb_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRgb.getTopic());
    mPubRgbGray = image_transport::create_camera_publisher( this, rgb_gray_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRgbGray.getTopic());
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRgb.getInfoTopic());
    mPubRawRgb = image_transport::create_camera_publisher( this, rgb_raw_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRawRgb.getTopic());
    mPubRawRgbGray = image_transport::create_camera_publisher( this, rgb_raw_gray_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRawRgbGray.getTopic());
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRawRgb.getInfoTopic());
    mPubLeft = image_transport::create_camera_publisher( this, left_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubLeft.getTopic());
    mPubLeftGray = image_transport::create_camera_publisher( this, left_gray_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubLeftGray.getTopic());
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubLeft.getInfoTopic());
    mPubRawLeft = image_transport::create_camera_publisher( this, left_raw_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRawLeft.getTopic());
    mPubRawLeftGray = image_transport::create_camera_publisher( this, left_raw_gray_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRawLeftGray.getTopic());
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRawLeft.getInfoTopic());
    mPubRight = image_transport::create_camera_publisher( this, right_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRight.getTopic());
    mPubRightGray = image_transport::create_camera_publisher( this, right_gray_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRightGray.getTopic());
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRight.getInfoTopic());
    mPubRawRight = image_transport::create_camera_publisher( this, right_raw_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRawRight.getTopic());
    mPubRawRightGray = image_transport::create_camera_publisher( this, right_raw_gray_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRawRightGray.getTopic());
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRawRight.getInfoTopic());

    mPubDepth = image_transport::create_camera_publisher( this, depth_topic, mDepthQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubDepth.getTopic());
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubDepth.getInfoTopic());

    mPubStereo = image_transport::create_publisher( this, stereo_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubStereo.getTopic());
    mPubRawStereo = image_transport::create_publisher( this, stereo_raw_topic, mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubRawStereo.getTopic());// <---- Camera publishers

    // ----> Depth publishers
    mPubConfMap = create_publisher<sensor_msgs::msg::Image>(conf_map_topic, mDepthQos);
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubConfMap->get_topic_name());
    mPubDisparity = create_publisher<stereo_msgs::msg::DisparityImage>( disparity_topic, mDepthQos );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubDisparity->get_topic_name());
    mPubCloud = create_publisher<sensor_msgs::msg::PointCloud2>( pointcloud_topic, mDepthQos );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubCloud->get_topic_name());
    // <---- Depth publishers

    // ----> Pos Tracking
    mPubPose = create_publisher<geometry_msgs::msg::PoseStamped>( pose_topic, mPoseQos );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubPose->get_topic_name());
    mPubPoseCov = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>( pose_cov_topic, mPoseQos );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubPoseCov->get_topic_name());
    mPubOdom = create_publisher<nav_msgs::msg::Odometry>( odometry_topic, mPoseQos );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubOdom->get_topic_name());
    mPubPosePath = create_publisher<nav_msgs::msg::Path>( map_path_topic, mPoseQos );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubPosePath->get_topic_name());
    mPubOdomPath = create_publisher<nav_msgs::msg::Path>( odom_path_topic, mPoseQos );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubOdomPath->get_topic_name());
    // <---- Pos Tracking

    // ----> Mapping
    if (mMappingEnabled) {
        mPubFusedCloud = create_publisher<sensor_msgs::msg::PointCloud2>( pointcloud_fused_topic, mDepthQos );
        RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubFusedCloud->get_topic_name());
    }
    // <---- Mapping

    // ----> Object Detection
    // TODO SEE ROS1
    // <---- Object Detection

    // ----> Sensors
    if(mZedRealCamModel != sl::MODEL::ZED) {
        mPubImu = create_publisher<sensor_msgs::msg::Imu>( imu_topic, mPoseQos );
        RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubImu->get_topic_name());
        mPubImuRaw = create_publisher<sensor_msgs::msg::Imu>( imu_topic_raw, mPoseQos );
        RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubImu->get_topic_name());
        mPubImuTemp = create_publisher<sensor_msgs::msg::Temperature>( imu_temp_topic, mPoseQos );
        RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubImuTemp->get_topic_name());

        if(mZedRealCamModel == sl::MODEL::ZED2) {
            mPubImuMag = create_publisher<sensor_msgs::msg::MagneticField>( imu_mag_topic, mPoseQos );
            RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubImuMag->get_topic_name());
            mPubPressure = create_publisher<sensor_msgs::msg::FluidPressure>( pressure_topic, mPoseQos );
            RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubPressure->get_topic_name());
            mPubTempL = create_publisher<sensor_msgs::msg::Temperature>( temp_topic_left, mPoseQos );
            RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubTempL->get_topic_name());
            mPubTempR = create_publisher<sensor_msgs::msg::Temperature>( temp_topic_right, mPoseQos );
            RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubTempR->get_topic_name());
        }

        // ----> Publish latched camera/imu transform message
        rclcpp::QoS transf_qos = mSensQos;
        mSensQos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); // Latched topic
        std::string cam_imu_tr_topic = root + "left_cam_imu_transform";
        mPubCamImuTransf = create_publisher<geometry_msgs::msg::Transform>(cam_imu_tr_topic, transf_qos);

        sl::Orientation sl_rot = mSlCamImuTransf.getOrientation();
        sl::Translation sl_tr = mSlCamImuTransf.getTranslation();

        mCameraImuTransfMgs = std::make_unique<geometry_msgs::msg::Transform>();

        mCameraImuTransfMgs->rotation.x = sl_rot.ox;
        mCameraImuTransfMgs->rotation.y = sl_rot.oy;
        mCameraImuTransfMgs->rotation.z = sl_rot.oz;
        mCameraImuTransfMgs->rotation.w = sl_rot.ow;

        mCameraImuTransfMgs->translation.x = sl_tr.x;
        mCameraImuTransfMgs->translation.y = sl_tr.y;
        mCameraImuTransfMgs->translation.z = sl_tr.z;

        RCLCPP_INFO( get_logger(), "Camera-IMU Rotation: \n %s", sl_rot.getRotationMatrix().getInfos().c_str() );
        RCLCPP_INFO( get_logger(), "Camera-IMU Translation: \n %g %g %g", sl_tr.x, sl_tr.y, sl_tr.z );

        mPubCamImuTransf->publish( std::move(mCameraImuTransfMgs) );

        RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubCamImuTransf->get_topic_name() << " [LATCHED]");

    }

    // <---- Sensors

}

bool ZedCamera::startCamera() {
    RCLCPP_INFO( get_logger(), "***** STARTING CAMERA *****");

    // ----> Check SDK version
    RCLCPP_INFO(get_logger(), "SDK Version: %d.%d.%d - Build %d",
                ZED_SDK_MAJOR_VERSION, ZED_SDK_MINOR_VERSION, ZED_SDK_PATCH_VERSION, ZED_SDK_BUILD_ID);
#if (ZED_SDK_MAJOR_VERSION<3)
    RCLCPP_ERROR(get_logger(), "ROS2 ZED node requires at least ZED SDK v3");

    return false;
#endif
    // <---- Check SDK version

    // ----> TF2 Transform
    mTfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);
    mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    mStaticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // <---- TF2 Transform

    // TODO Open Camera

    setTFCoordFrameNames(); // Requires mZedRealCamModel available only after camera opening
    initPublishers(); // Requires mZedRealCamModel available only after camera opening


}

} // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCamera)
