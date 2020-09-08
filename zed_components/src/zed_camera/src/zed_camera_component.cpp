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
    //getPosTrackingParams();

    // SENSORS parameters
    if(mZedUserCamModel != sl::MODEL::ZED) {
        //getSensorsParams();
    }

    // Dynamic parameters callback
    set_on_parameters_set_callback(std::bind(&ZedCamera::paramChangeCallback, this, _1));

    setTFCoordFrameNames();
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

    getParam( "video.img_downsample_factor", mZedImgDownsampleFactor, mZedImgDownsampleFactor );
    if (mZedImgDownsampleFactor < 0.1) {
        mZedImgDownsampleFactor = 0.1;
        RCLCPP_WARN(get_logger(), "The minimum value allowed for '%s' is 0.1", paramName.c_str());
    } else if (mZedImgDownsampleFactor > 1.0) {
        mZedImgDownsampleFactor = 1.0;
        RCLCPP_WARN(get_logger(), "The maximum value allowed for '%s' is 1.0", paramName.c_str());
    }
    RCLCPP_INFO(get_logger(), " * Image downsample factor: %g ", mZedImgDownsampleFactor);

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

rcl_interfaces::msg::SetParametersResult ZedCamera::paramChangeCallback(std::vector<rclcpp::Parameter> parameters) {

    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = false;

    for (size_t i = 0; i < parameters.size(); i++) {
        rclcpp::Parameter param = parameters[i];

        if (param.get_name() == "general.mat_resize_factor") {
            if (param.get_type() == rclcpp::PARAMETER_DOUBLE) {

                double new_val = param.as_double();

                if (new_val > 0.01 && new_val <= 1.0) {
                    mZedImgDownsampleFactor = new_val;
                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %g", param.get_name().c_str(), mZedImgDownsampleFactor);
                    result.successful = true;

                    // ----> Modify data sizes
                    mCamDataMutex.lock();
                    mMatWidth = static_cast<size_t>(mCamWidth * mZedImgDownsampleFactor);
                    mMatHeight = static_cast<size_t>(mCamHeight * mZedImgDownsampleFactor);
                    RCLCPP_INFO(get_logger(), "Data Mat size : %d x %d", sl::Resolution(mMatWidth, mMatHeight));

                    // Update Camera Info
                    fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId, mRightCamOptFrameId);
                    fillCamInfo(mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId, mRightCamOptFrameId, true);
                    mRgbCamInfoMsg = mDepthCamInfoMsg = mLeftCamInfoMsg;
                    mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
                    mCamDataMutex.unlock();
                    // <---- Modify data sizes
                } else {
                    RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT value in the range ]0.0,1.0]",
                                param.get_name().c_str());
                    result.successful = false;
                    return result;
                }
            } else {
                RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT positive value!", param.get_name().c_str());
                result.successful = false;
                return result;
            }
        } else if (param.get_name() == "video.auto_exposure_gain") {
            if (param.get_type() == rclcpp::PARAMETER_BOOL) {

                mZedAutoExpGain = param.as_bool();

                if (mZedAutoExpGain) {
                    mTriggerAutoExposure = true;
                }

                RCLCPP_INFO(get_logger(), "The param '%s' has changed to %s", param.get_name().c_str(),
                            mZedAutoExpGain ? "ENABLED" : "DISABLED");
                result.successful = true;
            } else {
                RCLCPP_WARN(get_logger(), "The param '%s' requires a BOOL value!", param.get_name().c_str());
                result.successful = false;
                return result;
            }
        } else if (param.get_name() == "video.exposure") {
            if (param.get_type() == rclcpp::PARAMETER_INTEGER) {

                int new_val = param.as_int();

                if (new_val > 0 && new_val <= 100) {
                    mZedExposure = new_val;
                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %d", param.get_name().c_str(), mZedExposure);
                    result.successful = true;
                } else {
                    RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value in the range ]0,100]", param.get_name().c_str());
                    result.successful = false;
                    return result;
                }
            } else {
                RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value!", param.get_name().c_str());
                result.successful = false;
                return result;
            }
        } else if (param.get_name() == "video.gain") {
            if (param.get_type() == rclcpp::PARAMETER_INTEGER) {

                int new_val = param.as_int();

                if (new_val > 0 && new_val <= 100) {
                    mZedGain = new_val;
                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %d", param.get_name().c_str(), mZedGain);
                    result.successful = true;
                } else {
                    RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value in the range ]0,100]", param.get_name().c_str());
                    result.successful = false;
                    return result;
                }
            } else {
                RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value!", param.get_name().c_str());
                result.successful = false;
                return result;
            }
        } else if (param.get_name() == "depth.confidence") {
            if (param.get_type() == rclcpp::PARAMETER_INTEGER) {
                int new_val = param.as_int();

                if (new_val > 0 && new_val <= 100) {
                    mDepthConf = new_val;
                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %d", param.get_name().c_str(), mDepthConf);
                    result.successful = true;
                } else {
                    RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value in the range ]0,100]", param.get_name().c_str());
                    result.successful = false;
                    return result;
                }
            } else {
                RCLCPP_WARN(get_logger(), "The param '%s' requires an INTEGER value!", param.get_name().c_str());
                result.successful = false;
                return result;
            }
        } else if (param.get_name() == "depth.max_depth") {
            if (param.get_type() == rclcpp::PARAMETER_DOUBLE) {

                double new_val = param.as_double();

                if (new_val > 0) {
                    mZedMaxDepth = new_val;
                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %g", param.get_name().c_str(), mZedMaxDepth);
                    result.successful = true;
                } else {
                    RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT positive value", param.get_name().c_str());
                    result.successful = false;
                    return result;
                }
            } else {
                RCLCPP_WARN(get_logger(), "The param '%s' requires a FLOATING POINT positive value!", param.get_name().c_str());
                result.successful = false;
                return result;
            }
        } else {
            RCLCPP_WARN(get_logger(), "The param '%s' cannot be dinamically changed!", param.get_name().c_str());
            result.successful = false;
            return result;
        }
    }

    return result;
}

void ZedCamera::getPosTrackingParams() {
    rclcpp::Parameter paramVal;
    std::string paramName;

    rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    size_t qos_depth = 10;
    rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    RCLCPP_INFO(get_logger(), "*** POSITIONAL TRACKING parameters ***");

    // ------------------------------------------

    paramName = "tracking.publish_tf";
    declare_parameter(paramName, rclcpp::ParameterValue(mPublishTF) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_BOOL) {
            mPublishTF = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a BOOL, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Publish TF: %s", mPublishTF ? "ENABLED" : "DISABLED");

    //    if (mPublishTF) {
    // ------------------------------------------

    paramName = "tracking.publish_map_tf";
    declare_parameter(paramName, rclcpp::ParameterValue(mPublishMapTF) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_BOOL) {
            mPublishMapTF = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a BOOL, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }
    //    } else {
    //        mPublishMapTF = false;
    //    }

    RCLCPP_INFO(get_logger(), " * Publish Map TF: %s", mPublishMapTF ? "ENABLED" : "DISABLED");

    // ------------------------------------------

    paramName = "tracking.pose_frame";
    declare_parameter(paramName, rclcpp::ParameterValue(mMapFrameId) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_STRING) {
            mMapFrameId = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a STRING, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Pose (map) frame: '%s'", mMapFrameId.c_str());

    // ------------------------------------------

    paramName = "tracking.odometry_frame";
    declare_parameter(paramName, rclcpp::ParameterValue(mOdometryFrameId) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_STRING) {
            mOdometryFrameId = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a STRING, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Odometry frame: '%s'", mOdometryFrameId.c_str());

    // ------------------------------------------

    // TODO Check how to handle the Odometry DB

    // ------------------------------------------

    paramName = "tracking.pose_smoothing";
    declare_parameter(paramName, rclcpp::ParameterValue(mPoseSmoothing) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_BOOL) {
            mPoseSmoothing = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a BOOL, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Pose Smothing: %s", mPoseSmoothing ? "ENABLED" : "DISABLED");

    // ------------------------------------------

    paramName = "tracking.spatial_memory";
    declare_parameter(paramName, rclcpp::ParameterValue(mSpatialMemory) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_BOOL) {
            mSpatialMemory = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a BOOL, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Spatial Memory: %s", mSpatialMemory ? "ENABLED" : "DISABLED");

    // ------------------------------------------

    paramName = "tracking.floor_alignment";
    declare_parameter(paramName, rclcpp::ParameterValue(mFloorAlignment) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_BOOL) {
            mFloorAlignment = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a BOOL, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Floor Alignment: %s", mFloorAlignment ? "ENABLED" : "DISABLED");

    // ------------------------------------------

    paramName = "tracking.two_d_mode";
    declare_parameter(paramName, rclcpp::ParameterValue(mTwoDMode) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_BOOL) {
            mTwoDMode = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a BOOL, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Force 2D mode: %s", mTwoDMode ? "ENABLED" : "DISABLED");

    // ------------------------------------------

    //if (mTwoDMode) {
    paramName = "tracking.fixed_z_value";
    declare_parameter(paramName, rclcpp::ParameterValue(mFixedZValue) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_DOUBLE) {
            mFixedZValue = paramVal.as_double();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a DOUBLE, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Fixed Z value: %g", mFixedZValue);
    //}

    // ------------------------------------------

    paramName = "tracking.initial_tracking_pose";
    declare_parameter(paramName, rclcpp::ParameterValue(mInitialBasePose) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_DOUBLE_ARRAY) {
            mInitialBasePose = paramVal.as_double_array();

            if (mInitialBasePose.size() != 6) {
                mInitialBasePose.resize(6, 0.0);

                RCLCPP_WARN(get_logger(), "The pose vector '%s' must contain SIX values, using the default value", paramName.c_str());
            }
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be an ARRAY OF DOUBLE, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());

        if (mInitialBasePose.size() != 6) {
            mInitialBasePose.resize(6, 0.0);
        }
    }

    RCLCPP_INFO(get_logger(), " * Initial pose: [%g,%g,%g, %g,%g,%g]",
                mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2],
            mInitialBasePose[3], mInitialBasePose[4], mInitialBasePose[5]);

    // ------------------------------------------

    paramName = "tracking.init_odom_with_first_valid_pose";
    declare_parameter(paramName, rclcpp::ParameterValue(mInitOdomWithPose) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_BOOL) {
            mInitOdomWithPose = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a BOOL, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Init odometry with first valid pose: %s", mInitOdomWithPose ? "ENABLED" : "DISABLED");

    // ------------------------------------------

    paramName = "tracking.path_pub_rate";
    declare_parameter(paramName, rclcpp::ParameterValue(mPathPubRate) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_DOUBLE) {
            mPathPubRate = paramVal.as_double();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a BOOL, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    if (mPathPubRate > 0.0) {
        RCLCPP_INFO(get_logger(), " * Path publishing frequency: %g Hz", mPathPubRate);
    } else {
        RCLCPP_INFO(get_logger(), " * Path publishing: DISABLED (%g)", mPathPubRate);
    }

    // ------------------------------------------

    paramName = "tracking.path_max_count";
    declare_parameter(paramName, rclcpp::ParameterValue(mPathMaxCount) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_INTEGER) {
            mPathMaxCount = paramVal.as_int();

            if (mPathMaxCount == 0) {
                RCLCPP_WARN(get_logger(),
                            "The parameter '%s' cannot be '0'. To disable path publishing use please set 'path_pub_rate' to '0,0'. Using the default value",
                            paramName.c_str());

                mPathMaxCount = -1;
            }
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a BOOL, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    if (mPathMaxCount > 0) {
        RCLCPP_INFO(get_logger(), " * Path queue size: %d", mPathMaxCount);
    } else {
        RCLCPP_INFO(get_logger(), " * Path queue size: INFINITE (%d)", mPathMaxCount);
    }

    // ------------------------------------------

    paramName = "tracking.publish_pose_covariance";
    declare_parameter(paramName, rclcpp::ParameterValue(mPublishPoseCov) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_BOOL) {
            mPublishPoseCov = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a BOOL, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Pose with covariance topic: %s", mPublishPoseCov ? "ENABLED" : "DISABLED");

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
    RCLCPP_INFO_STREAM(get_logger(), " * Odometry\t\t\t-> " << mOdometryFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Base\t\t\t-> " << mBaseFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Camera\t\t\t-> " << mCameraFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Left\t\t\t-> " << mLeftCamFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Left Optical\t\t-> " << mLeftCamOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * RGB\t\t\t-> " << mRgbFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * RGB Optical\t\t-> " << mRgbFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Right\t\t\t-> " << mRightCamFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Right Optical\t\t-> " << mRightCamOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Depth\t\t\t-> " << mDepthFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Depth Optical\t\t-> " << mDepthOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Point Cloud\t\t-> " << mCloudFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Disparity\t\t\t-> " << mDisparityFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Disparity Optical\t\t-> " << mDisparityOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Confidence\t\t-> " << mConfidenceFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Confidence Optical\t-> " << mConfidenceOptFrameId);
    if(mZedUserCamModel!=sl::MODEL::ZED)
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
    // TODO SEE ROS1 WRAPPER
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


}

} // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCamera)
