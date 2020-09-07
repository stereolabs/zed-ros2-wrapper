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
    , mImuQos(10)
    , mPoseQos(10) {
    RCLCPP_INFO(get_logger(), "*******************************");
    RCLCPP_INFO(get_logger(), " ZED Camera Component created");
    RCLCPP_INFO(get_logger(), "  * namespace: %s", get_namespace());
    RCLCPP_INFO(get_logger(), "  * node name: %s", get_name());
    RCLCPP_INFO(get_logger(), "********************************");

    // ----> Parameters initialization
    getParam( "general.debug_mode_active", mDebugMode, mDebugMode, " * DEBUG MODE: ");

    std::string ros_namespace = get_namespace();
    std::string node_name = get_name();

    if(mDebugMode) {
        std::string logger = ros_namespace.empty() ? "" : ros_namespace + ".";
        logger += node_name;
        rcutils_ret_t res = rcutils_logging_set_logger_level(logger.c_str(), RCUTILS_LOG_SEVERITY_DEBUG);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting DEBUG logger");
        } else {
            RCLCPP_INFO(get_logger(), "Debug Mode enabled");
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
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    if( !log_info.empty() ) {
        RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
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

    // IMU parameters
    getSensorsParams();

    // Dynamic parameters callback
    set_on_parameters_set_callback(std::bind(&ZedCamera::paramChangeCallback, this, _1));

    setTFCoordFrameNames();
}

void ZedCamera::getGeneralParams() {
    rclcpp::Parameter paramVal;
    std::string paramName;

    RCLCPP_INFO(get_logger(), "*** GENERAL parameters ***");

    getParam( "general.verbose", mVerbose, mVerbose,  " * SDK Verbose: ");
    getParam( "general.svo_file", std::string(), mSvoFilepath, " * SVO: ");
    getParam( "general.camera_name", mCameraName, mCameraName,  " * Camera name: ");

    // ----> Camera model must be handled differently
    paramName = "general.camera_model";
    declare_parameter(paramName, rclcpp::ParameterValue("zed") );

    if (get_parameter(paramName, paramVal)) {
        std::string camera_model = paramVal.as_string();

        if (camera_model == "zed") {
            mZedUserCamModel = sl::MODEL::ZED;
        } else if (camera_model == "zedm") {
            mZedUserCamModel = sl::MODEL::ZED_M;
        } else if (camera_model == "zed2") {
            mZedUserCamModel = sl::MODEL::ZED2;
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), "Camera model not valid in parameter values: " << camera_model);
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Camera model: %d (%s)",
                mZedUserCamModel, sl::toString(static_cast<sl::MODEL>(mZedUserCamModel)).c_str());
    // <---- Camera model must be handled differently

    getParam( "general.camera_timeout_sec", mCamTimeoutSec, mCamTimeoutSec,  " * Camera timeout [sec]: ");
    getParam( "general.camera_reactivate", mZedReactivate, mZedReactivate,  " * Camera reconnection if disconnected: ");
    getParam( "general.camera_max_reconnect", mMaxReconnectTemp, mMaxReconnectTemp,  " * Camera reconnection temptatives: ");
    getParam( "general.camera_flip", mCameraFlip, mCameraFlip,  " * Camera flip: ");
    getParam( "general.zed_id", mZedId, mZedId,  " * Camera ID: ");
    getParam( "general.serial_number", mZedSerialNumber, mZedSerialNumber,  " * Camera SN: ");
    getParam( "general.resolution", mZedResol, mZedResol,  " * Camera resolution: ");

    // ----> Mat resize factor must be handled differently
    paramName = "general.mat_resize_factor";
    declare_parameter(paramName, rclcpp::ParameterValue(mZedMatResizeFactor) );

    if (get_parameter(paramName, paramVal)) {
        if (paramVal.get_type() == rclcpp::PARAMETER_DOUBLE) {
            mZedMatResizeFactor = paramVal.as_double();

            if (mZedMatResizeFactor < 0.1) {
                mZedMatResizeFactor = 0.1;
                RCLCPP_WARN(get_logger(), "The minimum value allowed for '%s' is 0.1", paramName.c_str());
            } else if (mZedMatResizeFactor > 1.0) {
                mZedMatResizeFactor = 1.0;
                RCLCPP_WARN(get_logger(), "The maximum value allowed for '%s' is 1.0", paramName.c_str());
            }
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' must be a DOUBLE, using the default value", paramName.c_str());
        }
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * [DYN] Data resize factor: %g ", mZedMatResizeFactor);
    // <---- Mat resize factor must be handled differently

    getParam( "general.frame_rate", mZedFrameRate, mZedFrameRate,  " * Camera framerate: ");
    getParam( "general.gpu_id", mGpuId, mGpuId,  " * GPU ID: ");
    getParam( "general.base_frame", mBaseFrameId, mBaseFrameId,  " * Base frame id: ");
}

void ZedCamera::getVideoParams() {
    rclcpp::Parameter paramVal;
    std::string paramName;

    RCLCPP_INFO(get_logger(), "*** VIDEO parameters ***");

    rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    size_t qos_depth = 10;
    rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    getParam( "video.auto_exposure", mZedAutoExposure, mZedAutoExposure,  " * [DYN] Auto exposure: ");
    if (mZedAutoExposure) {
        mTriggerAutoExposure = true;
    }
    getParam( "video.exposure", mZedExposure, mZedExposure,  " * [DYN] Exposure: ");
    getParam( "video.gain", mZedGain, mZedGain,  " * [DYN] Gain: ");

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

    getParam( "depth.min_depth", mZedMinDepth, mZedMinDepth, " * Min depth [m]: ");
    getParam( "depth.min_depth", mZedMaxDepth, mZedMaxDepth, " * Max depth [m]: ");
    getParam( "depth.quality", mZedQuality, mZedQuality, " * Quality: ");
    getParam( "depth.sensing_mode", mZedSensingMode, mZedSensingMode, " * Sensing mode: ");

    paramName = "depth.sensing_mode";
    declare_parameter(paramName, rclcpp::ParameterValue(mZedSensingMode) );

    paramName = "depth.confidence";
    declare_parameter(paramName, rclcpp::ParameterValue(mZedConfidence) );

    if (get_parameter(paramName, paramVal)) {
        mZedConfidence = paramVal.as_int();
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Confidence: %d", mZedConfidence);

    // ------------------------------------------

    paramName = "depth.depth_stabilization";
    declare_parameter(paramName, rclcpp::ParameterValue(mDepthStabilization) );

    if (get_parameter(paramName, paramVal)) {
        mDepthStabilization = paramVal.as_bool();
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Depth stabilization: %s", mDepthStabilization ? "ENABLED" : "DISABLED");

    // ------------------------------------------

    paramName = "depth.openni_depth_mode";
    declare_parameter(paramName, rclcpp::ParameterValue(mOpenniDepthMode) );

    if (get_parameter(paramName, paramVal)) {
        mOpenniDepthMode = paramVal.as_bool();
    } else {
        RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * OpenNI mode: %s", mOpenniDepthMode == 0 ? "DISABLED" : "ENABLED");

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

rcl_interfaces::msg::SetParametersResult ZedCamera::paramChangeCallback(std::vector<rclcpp::Parameter>
                                                                        parameters) {

    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = false;

    for (size_t i = 0; i < parameters.size(); i++) {
        rclcpp::Parameter param = parameters[i];

        if (param.get_name() == "general.mat_resize_factor") {
            if (param.get_type() == rclcpp::PARAMETER_DOUBLE) {

                double new_val = param.as_double();

                if (new_val > 0.01 && new_val <= 1.0) {
                    mZedMatResizeFactor = new_val;
                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %g", param.get_name().c_str(), mZedMatResizeFactor);
                    result.successful = true;

                    // ----> Modify data sizes
                    mCamDataMutex.lock();
                    mMatWidth = static_cast<size_t>(mCamWidth * mZedMatResizeFactor);
                    mMatHeight = static_cast<size_t>(mCamHeight * mZedMatResizeFactor);
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
        } else if (param.get_name() == "video.auto_exposure") {
            if (param.get_type() == rclcpp::PARAMETER_BOOL) {

                mZedAutoExposure = param.as_bool();

                if (mZedAutoExposure) {
                    mTriggerAutoExposure = true;
                }

                RCLCPP_INFO(get_logger(), "The param '%s' has changed to %s", param.get_name().c_str(),
                            mZedAutoExposure ? "ENABLED" : "DISABLED");
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
                    mZedConfidence = new_val;
                    RCLCPP_INFO(get_logger(), "The param '%s' has changed to %d", param.get_name().c_str(), mZedConfidence);
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
    mImuFrameId = mCameraName + "_imu_link";
    mLeftCamFrameId = mCameraName + "_left_camera_frame";
    mLeftCamOptFrameId = mCameraName + "_left_camera_optical_frame";
    mRightCamFrameId = mCameraName + "_right_camera_frame";
    mRightCamOptFrameId = mCameraName + "_right_camera_optical_frame";

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
    RCLCPP_INFO_STREAM(get_logger(), " * map_frame\t\t\t-> " << mMapFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * odometry_frame\t\t-> " << mOdometryFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * base_frame\t\t\t-> " << mBaseFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * camera_frame\t\t\t-> " << mCameraFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * imu_link\t\t\t-> " << mImuFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * left_camera_frame\t\t-> " << mLeftCamFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * left_camera_optical_frame\t-> " << mLeftCamOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * right_camera_frame\t\t-> " << mRightCamFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * right_camera_optical_frame\t-> " << mRightCamOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * depth_frame\t\t\t-> " << mDepthFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * depth_optical_frame\t\t-> " << mDepthOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * disparity_frame\t\t-> " << mDisparityFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * disparity_optical_frame\t-> " << mDisparityOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * confidence_frame\t\t-> " << mConfidenceFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * confidence_optical_frame\t-> " << mConfidenceOptFrameId);
    // <---- Coordinate frames
}

void ZedCamera::fillCamInfo(sl::Camera& zed, std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
                            std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg,
                            std::string leftFrameId, std::string rightFrameId,
                            bool rawParam /*= false*/) {
    sl::CalibrationParameters zedParam;

    if (rawParam) {
        zedParam = zed.getCameraInformation(sl::Resolution(sl::Resolution(mMatWidth, mMatHeight)))
                .calibration_parameters_raw;
    } else {
        zedParam = zed.getCameraInformation(sl::Resolution(sl::Resolution(mMatWidth, mMatHeight)))
                .calibration_parameters;
    }

    float baseline = zedParam.T[0];
    leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
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

    if (rawParam) {
        std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
        float* p = R_.data();

        for (int i = 0; i < 9; i++) {
            rightCamInfoMsg->r[i] = p[i];
        }
    }

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
    leftCamInfoMsg->width = rightCamInfoMsg->width = static_cast<uint32_t>(mMatWidth);
    leftCamInfoMsg->height = rightCamInfoMsg->height = static_cast<uint32_t>(mMatHeight);
    leftCamInfoMsg->header.frame_id = leftFrameId;
    rightCamInfoMsg->header.frame_id = rightFrameId;
}

} // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCamera)
