/********************************************************************************
 * MIT License
 *
 * Copyright (c) 2020 Stereolabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ********************************************************************************/

#include "zed_camera_component.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <type_traits>

#include "sl_tools.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

#define TIMER_TIME_FACTOR 1.5

namespace stereolabs {
#ifndef DEG2RAD
#define DEG2RAD 0.017453293
#define RAD2DEG 57.295777937
#endif

ZedCamera::ZedCamera(const rclcpp::NodeOptions& options)
    : Node("zed_node", options)
    , mDiagUpdater(this)
    , mVideoQos(1)
    , mDepthQos(1)
    , mSensQos(1)
    , mPoseQos(1)
    , mMappingQos(1)
    , mObjDetQos(1)
    , mClickedPtQos(1)
{
    RCLCPP_INFO(get_logger(), "********************************");
    RCLCPP_INFO(get_logger(), "      ZED Camera Component ");
    RCLCPP_INFO(get_logger(), "********************************");
    RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
    RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
    RCLCPP_INFO(get_logger(), "********************************");

    if (ZED_SDK_MAJOR_VERSION < 3 || (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 3)) {
        RCLCPP_ERROR(
            get_logger(),
            "The ZED ROS2 wrapper is designed to work with ZED SDK v3.3 or newer.");
        RCLCPP_INFO_STREAM(get_logger(),
            "* Detected SDK v" << ZED_SDK_MAJOR_VERSION << "."
                               << ZED_SDK_MINOR_VERSION << "."
                               << ZED_SDK_PATCH_VERSION << "-"
                               << ZED_SDK_BUILD_ID);
        RCLCPP_INFO(get_logger(), "Node stopped");
        exit(EXIT_FAILURE);
    }

    // Parameters initialization
    initParameters();

    // ----> Diagnostic
    mDiagUpdater.add(
        "ZED Diagnostic", this, &ZedCamera::callback_updateDiagnostic);
    mDiagUpdater.setHardwareID("ZED camera");
    // <---- Diagnostic

    // Init services
    initServices();

    // Start camera
    if (!startCamera()) {
        exit(EXIT_FAILURE);
    }
}

ZedCamera::~ZedCamera()
{
    RCLCPP_DEBUG(get_logger(), "Destroying node");

    if (mObjDetRunning) {
        std::lock_guard<std::mutex> lock(mObjDetMutex);
        stopObjDetect();
    }

    if (mMappingRunning) {
        std::lock_guard<std::mutex> lock(mMappingMutex);
        stop3dMapping();
    }

    std::lock_guard<std::mutex> lock(mCloseZedMutex);

    if (mSensTimer) {
        mSensTimer->cancel();
    }
    if (mPathTimer) {
        mPathTimer->cancel();
    }
    if (mFusedPcTimer) {
        mFusedPcTimer->cancel();
    }
    if (mVideoDepthTimer) {
        mVideoDepthTimer->cancel();
    }

    // ----> Verify that all the threads are not active
    if (!mThreadStop) {
        mThreadStop = true;

        RCLCPP_DEBUG(get_logger(), "Stopping grab thread...");
        try {
            if (mGrabThread.joinable()) {
                mGrabThread.join();
            }
        } catch (std::system_error& e) {
            RCLCPP_WARN(get_logger(), "Grab thread joining exception: %s", e.what());
        }
        RCLCPP_DEBUG(get_logger(), "... grab thread stopped");

        RCLCPP_DEBUG(get_logger(), "Stopping Point Cloud thread...");
        try {
            if (mPcThread.joinable()) {
                mPcThread.join();
            }
        } catch (std::system_error& e) {
            RCLCPP_WARN(
                get_logger(), "Pointcloud thread joining exception: %s", e.what());
        }
        RCLCPP_DEBUG(get_logger(), "... Point Cloud thread stopped");
    }
    // <---- Verify that the grab thread is not active
}

void ZedCamera::initServices()
{
    RCLCPP_INFO(get_logger(), "*** SERVICES ***");

    std::string srv_name;

    /*std::string srv_name;
  std::string srv_prefix = get_namespace();

  if (srv_prefix.length() > 1) {
      srv_prefix += "/";
  }

  if ('/' != srv_prefix.at(0)) {
      srv_prefix = '/'  + srv_prefix;
  }

  srv_prefix += get_name();*/
    std::string srv_prefix = "~/";

    if(!mDepthDisabled)
    {
        // ResetOdometry
        srv_name = srv_prefix + mSrvResetOdomName;
        mResetOdomSrv = create_service<std_srvs::srv::Trigger>(
            srv_name, std::bind(&ZedCamera::callback_resetOdometry, this, _1, _2, _3));
        RCLCPP_INFO(get_logger(), " * '%s'", mResetOdomSrv->get_service_name());

        srv_name = srv_prefix + mSrvResetPoseName;
        mResetPosTrkSrv = create_service<std_srvs::srv::Trigger>(
            srv_name,
            std::bind(&ZedCamera::callback_resetPosTracking, this, _1, _2, _3));
        RCLCPP_INFO(get_logger(), " * '%s'", mResetPosTrkSrv->get_service_name());
        srv_name = srv_prefix + mSrvSetPoseName;
        mSetPoseSrv = create_service<zed_interfaces::srv::SetPose>(
            srv_name, std::bind(&ZedCamera::callback_setPose, this, _1, _2, _3));
        RCLCPP_INFO(get_logger(), " * '%s'", mSetPoseSrv->get_service_name());

        srv_name = srv_prefix + mSrvEnableObjDetName;
        mEnableObjDetSrv = create_service<std_srvs::srv::SetBool>(
            srv_name, std::bind(&ZedCamera::callback_enableObjDet, this, _1, _2, _3));
        RCLCPP_INFO(get_logger(), " * '%s'", mEnableObjDetSrv->get_service_name());

        srv_name = srv_prefix + mSrvEnableMappingName;
        mEnableMappingSrv = create_service<std_srvs::srv::SetBool>(
            srv_name, std::bind(&ZedCamera::callback_enableMapping, this, _1, _2, _3));
        RCLCPP_INFO(get_logger(), " * '%s'", mEnableMappingSrv->get_service_name());
    }

    srv_name = srv_prefix + mSrvStartSvoRecName;
    mStartSvoRecSrv = create_service<zed_interfaces::srv::StartSvoRec>(
        srv_name, std::bind(&ZedCamera::callback_startSvoRec, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mStartSvoRecSrv->get_service_name());
    srv_name = srv_prefix + mSrvStopSvoRecName;
    mStopSvoRecSrv = create_service<std_srvs::srv::Trigger>(
        srv_name, std::bind(&ZedCamera::callback_stopSvoRec, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mStopSvoRecSrv->get_service_name());
    srv_name = srv_prefix + mSrvToggleSvoPauseName;
    mPauseSvoSrv = create_service<std_srvs::srv::Trigger>(
        srv_name, std::bind(&ZedCamera::callback_pauseSvoInput, this, _1, _2, _3));
    RCLCPP_INFO(get_logger(), " * '%s'", mPauseSvoSrv->get_service_name());
}

template <typename T>
void ZedCamera::getParam(std::string paramName,
    T defValue,
    T& outVal,
    std::string log_info)
{
    declare_parameter(paramName, rclcpp::ParameterValue(defValue));

    if (!get_parameter(paramName, outVal)) {
        RCLCPP_WARN_STREAM(
            get_logger(),
            "The parameter '"
                << paramName
                << "' is not available or is not valid, using the default value: "
                << defValue);
    }

    if (!log_info.empty()) {
        RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
    }
}

void ZedCamera::initParameters()
{
    // DEBUG parameters
    getDebugParams();

    // GENERAL parameters
    getGeneralParams();

    // VIDEO parameters
    getVideoParams();

    // DEPTH parameters
    getDepthParams();
    
    // POS. TRACKING parameters
    if (!mDepthDisabled)
    {
        getPosTrackingParams();        
    }
    else
    {
        mPosTrackingEnabled = false;
    }

    // SENSORS parameters
    if (mCamUserModel != sl::MODEL::ZED)
        getSensorsParams();

    if (!mDepthDisabled)
    {
        getMappingParams();
    }
    else
    {
        mMappingEnabled = false;
    }

    // OD PARAMETERS
    if (!mDepthDisabled)
    {
        if (sl_tools::isObjDetAvailable(mCamUserModel))
            getOdParams();
    }
    else
    {
        mObjDetEnabled = false;
    }

    // Dynamic parameters callback
    //set_on_parameters_set_callback(std::bind(&ZedCamera::callback_paramChange, this, _1)); // deprecated
    mParamChangeCallbackHandle = add_on_set_parameters_callback(std::bind(&ZedCamera::callback_paramChange, this, _1));
}

void ZedCamera::getDebugParams()
{
    rclcpp::Parameter paramVal;
    std::string paramName;

    RCLCPP_INFO(get_logger(), "*** DEBUG parameters ***");

    getParam("debug.debug_mode", mDebugMode, mDebugMode);
    RCLCPP_INFO(get_logger(), " * Debug mode: %s", mDebugMode ? "TRUE" : "FALSE");
    if (mDebugMode) {
        rcutils_ret_t res = rcutils_logging_set_logger_level(
            get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting DEBUG level fot logger");
        } else {
            RCLCPP_INFO(get_logger(), " + Debug Mode enabled +");
        }
    } else {
        rcutils_ret_t res = rcutils_logging_set_logger_level(
            get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting INFO level for logger");
        }
    }

    getParam("debug.debug_sensors", mDebugSensors, mDebugSensors);
    RCLCPP_INFO(get_logger(), " * Debug sensors: %s", mDebugSensors ? "TRUE" : "FALSE");

    if(mDebugSensors && !mDebugMode)
    {
        RCLCPP_WARN(get_logger(), "To enable Sensors debug messages please set 'debug/debug_mode' to 'true'");
    }

    RCLCPP_DEBUG(get_logger(),
        "[ROS2] Using RMW_IMPLEMENTATION = %s",
        rmw_get_implementation_identifier());
}

void ZedCamera::getGeneralParams()
{
    rclcpp::Parameter paramVal;
    std::string paramName;

    RCLCPP_INFO(get_logger(), "*** GENERAL parameters ***");

    std::string camera_model = "zed";
    getParam("general.camera_model", camera_model, camera_model);
    if (camera_model == "zed") {
        mCamUserModel = sl::MODEL::ZED;
    } else if (camera_model == "zedm") {
        mCamUserModel = sl::MODEL::ZED_M;
    } else if (camera_model == "zed2") {
        mCamUserModel = sl::MODEL::ZED2;
    }
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 5
    else if (camera_model == "zed2i") {
        mCamUserModel = sl::MODEL::ZED2i;
    }
#endif
    else {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "Camera model not valid in parameter values: " << camera_model);
    }
    RCLCPP_INFO_STREAM(get_logger(),
        " * Camera model: " << camera_model << " - "
                            << mCamUserModel);

    getParam("general.sdk_verbose", mVerbose, mVerbose, " * SDK Verbose: ");
    getParam("general.svo_file", std::string(), mSvoFilepath, " * SVO: ");
    if (mSvoFilepath.compare("live") == 0) // Patch for Python launch files not allowing empty strings as default parameters
    {
        mSvoFilepath = "";
    }

    getParam("general.svo_loop", mSvoLoop, mSvoLoop);
    RCLCPP_INFO(get_logger(), " * SVO Loop: %s", mSvoLoop ? "TRUE" : "FALSE");
    getParam("general.svo_realtime", mSvoRealtime, mSvoRealtime);
    RCLCPP_INFO(
        get_logger(), " * SVO Realtime: %s", mSvoRealtime ? "TRUE" : "FALSE");
    getParam("general.camera_name", mCameraName, mCameraName, " * Camera name: ");
    getParam("general.zed_id", mCamId, mCamId, " * Camera ID: ");
    getParam("general.serial_number",
        mCamSerialNumber,
        mCamSerialNumber,
        " * Camera SN: ");
    getParam("general.camera_timeout_sec",
        mCamTimeoutSec,
        mCamTimeoutSec,
        " * Camera timeout [sec]: ");
    getParam("general.camera_max_reconnect",
        mMaxReconnectTemp,
        mMaxReconnectTemp,
        " * Camera reconnection temptatives: ");
    getParam("general.grab_frame_rate",
        mCamGrabFrameRate,
        mCamGrabFrameRate,
        " * Camera framerate: ");
    getParam("general.gpu_id", mGpuId, mGpuId, " * GPU ID: ");

    // TODO ADD SVO SAVE COMPRESSION PARAMETERS

    int resol = static_cast<int>(mCamResol);
    getParam("general.resolution", resol, resol);
    mCamResol = static_cast<sl::RESOLUTION>(resol);
    RCLCPP_INFO_STREAM(get_logger(),
        " * Camera resolution: " << resol << " - " << mCamResol);

    getParam("general.self_calib", mCameraSelfCalib, mCameraSelfCalib);
    RCLCPP_INFO(get_logger(),
        " * Camera self calibration: %s",
        mCameraSelfCalib ? "TRUE" : "FALSE");
    getParam("general.camera_flip", mCameraFlip, mCameraFlip);
    RCLCPP_INFO(
        get_logger(), " * Camera flip: %s", mCameraFlip ? "TRUE" : "FALSE");

    // Dynamic parameters

    getParam("general.pub_frame_rate", mPubFrameRate, mPubFrameRate);
    if (mPubFrameRate > mCamGrabFrameRate) {
        RCLCPP_WARN(get_logger(),
            "'pub_frame_rate' cannot be bigger than 'grab_frame_rate'",
            paramName.c_str());
    }
    RCLCPP_INFO(
        get_logger(), " * [DYN] Publish framerate [Hz]: %g ", mPubFrameRate);
}

void ZedCamera::getVideoParams()
{
    rclcpp::Parameter paramVal;
    std::string paramName;

    RCLCPP_INFO(get_logger(), "*** VIDEO parameters ***");

    rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    int qos_depth = 1;
    rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    getParam("video.extrinsic_in_camera_frame",
        mUseOldExtrinsic,
        mUseOldExtrinsic,
        " * Use old extrinsic parameters: ");

    getParam(
        "video.img_downsample_factor", mImgDownsampleFactor, mImgDownsampleFactor);
    if (mImgDownsampleFactor < 0.1) {
        mImgDownsampleFactor = 0.1;
        RCLCPP_WARN(get_logger(),
            "The minimum value allowed for '%s' is 0.1",
            paramName.c_str());
    } else if (mImgDownsampleFactor > 1.0) {
        mImgDownsampleFactor = 1.0;
        RCLCPP_WARN(get_logger(),
            "The maximum value allowed for '%s' is 1.0",
            paramName.c_str());
    }
    RCLCPP_INFO(get_logger(),
        " * [DYN] Image downsample factor: %g ",
        mImgDownsampleFactor);

    getParam("video.brightness",
        mCamBrightness,
        mCamBrightness,
        " * [DYN] Brightness: ");
    getParam("video.contrast", mCamContrast, mCamContrast, " * [DYN] Contrast: ");
    getParam("video.hue", mCamHue, mCamHue, " * [DYN] Hue: ");
    getParam("video.saturation",
        mCamSaturation,
        mCamSaturation,
        " * [DYN] Saturation: ");
    getParam(
        "video.sharpness", mCamSharpness, mCamSharpness, " * [DYN] Sharpness: ");
    getParam("video.gamma", mCamGamma, mCamGamma, " * [DYN] Gamma: ");
    getParam("video.auto_exposure_gain", mCamAutoExpGain, mCamAutoExpGain);
    RCLCPP_INFO(get_logger(),
        " * [DYN] Auto Exposure/Gain: %s",
        mCamAutoExpGain ? "TRUE" : "FALSE");
    if (mCamAutoExpGain) {
        mTriggerAutoExpGain = true;
    }
    getParam("video.exposure", mCamExposure, mCamExposure, " * [DYN] Exposure: ");
    getParam("video.gain", mCamGain, mCamGain, " * [DYN] Gain: ");
    getParam("video.auto_whitebalance", mCamAutoWB, mCamAutoWB);
    RCLCPP_INFO(get_logger(),
        " * [DYN] Auto White Balance: %s",
        mCamAutoWB ? "TRUE" : "FALSE");
    if (mCamAutoWB) {
        mTriggerAutoWB = true;
    }
    int wb = 42;
    getParam("video.whitebalance_temperature",
        wb,
        wb,
        " * [DYN] White Balance Temperature: ");
    mCamWBTemp = wb * 100;

    // ------------------------------------------

    paramName = "video.qos_history";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_hist));

    if (get_parameter(paramName, paramVal)) {
        qos_hist = paramVal.as_int() == 1 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST
                                          : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        mVideoQos.history(qos_hist);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Video QoS History: %s",
        sl_tools::qos2str(qos_hist).c_str());

    // ------------------------------------------

    paramName = "video.qos_depth";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_depth));

    if (get_parameter(paramName, paramVal)) {
        qos_depth = paramVal.as_int();
        mVideoQos.keep_last(qos_depth);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Video QoS History depth: %d", qos_depth);

    // ------------------------------------------

    paramName = "video.qos_reliability";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_reliability));

    if (get_parameter(paramName, paramVal)) {
        qos_reliability = paramVal.as_int() == 1
            ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
            : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        mVideoQos.reliability(qos_reliability);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Video QoS Reliability: %s",
        sl_tools::qos2str(qos_reliability).c_str());

    // ------------------------------------------

    paramName = "video.qos_durability";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_durability));

    if (get_parameter(paramName, paramVal)) {
        qos_durability = paramVal.as_int() == 0
            ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
            : RMW_QOS_POLICY_DURABILITY_VOLATILE;
        mVideoQos.durability(qos_durability);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Video QoS Durability: %s",
        sl_tools::qos2str(qos_durability).c_str());
}

void ZedCamera::getDepthParams()
{
    rclcpp::Parameter paramVal;
    std::string paramName;

    rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    int qos_depth = 1;
    rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    RCLCPP_INFO(get_logger(), "*** DEPTH parameters ***");

    int depth_quality = static_cast<int>(mDepthQuality);
    getParam("depth.quality", depth_quality, depth_quality);
    mDepthQuality = static_cast<sl::DEPTH_MODE>(depth_quality);

    if (mDepthQuality == sl::DEPTH_MODE::NONE) {
        mDepthDisabled = true;
        mDepthStabilization = false;
        RCLCPP_INFO_STREAM(get_logger(),
            " * Depth quality: " << depth_quality << " - "
                                 << mDepthQuality
                                 << " - DEPTH DISABLED");
                                 
    } else {
        mDepthDisabled = false;
        RCLCPP_INFO_STREAM(get_logger(),
            " * Depth quality: " << depth_quality << " - "
                                 << mDepthQuality);
    }

    if (!mDepthDisabled) {
        getParam("depth.depth_downsample_factor",
            mDepthDownsampleFactor,
            mDepthDownsampleFactor);
        if (mDepthDownsampleFactor < 0.1) {
            mDepthDownsampleFactor = 0.1;
            RCLCPP_WARN(get_logger(),
                "The minimum value allowed for '%s' is 0.1",
                paramName.c_str());
        } else if (mDepthDownsampleFactor > 1.0) {
            mDepthDownsampleFactor = 1.0;
            RCLCPP_WARN(get_logger(),
                "The maximum value allowed for '%s' is 1.0",
                paramName.c_str());
        }
        RCLCPP_INFO(
            get_logger(), " * Depth downsample factor: %g ", mDepthDownsampleFactor);

        getParam(
            "depth.min_depth", mCamMinDepth, mCamMinDepth, " * Min depth [m]: ");
        getParam(
            "depth.max_depth", mCamMaxDepth, mCamMaxDepth, " * Max depth [m]: ");

        int sens_mode = static_cast<int>(mDepthSensingMode);
        getParam("depth.sensing_mode", sens_mode, sens_mode);
        mDepthSensingMode = static_cast<sl::SENSING_MODE>(sens_mode);
        RCLCPP_INFO_STREAM(get_logger(),
            " * Depth Sensing Mode: " << sens_mode << " - "
                                      << mDepthSensingMode);

        getParam(
            "depth.depth_stabilization", mDepthStabilization, mDepthStabilization);
        RCLCPP_INFO(get_logger(),
            " * Depth Stabilization: %s",
            mDepthStabilization ? "TRUE" : "FALSE");

        getParam("depth.openni_depth_mode", mOpenniDepthMode, mOpenniDepthMode);
        RCLCPP_INFO(get_logger(),
            " * OpenNI mode (16bit point cloud): %s",
            mOpenniDepthMode ? "TRUE" : "FALSE");

        getParam("depth.point_cloud_freq",
            mPcPubRate,
            mPcPubRate,
            " * [DYN] Point cloud rate [Hz]: ");

        getParam("depth.depth_confidence",
            mDepthConf,
            mDepthConf,
            " * [DYN] Depth Confidence: ");
        getParam("depth.depth_texture_conf",
            mDepthTextConf,
            mDepthTextConf,
            " * [DYN] Depth Texture Confidence: ");
        getParam("depth.remove_saturated_areas", mRemoveSatAreas, mRemoveSatAreas);
        RCLCPP_INFO(get_logger(),
            " * [DYN] Remove saturated areas: %s",
            mRemoveSatAreas ? "TRUE" : "FALSE");
        // ------------------------------------------

        paramName = "depth.qos_history";
        declare_parameter(paramName, rclcpp::ParameterValue(qos_hist));

        if (get_parameter(paramName, paramVal)) {
            qos_hist = paramVal.as_int() == 1 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST
                                              : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
            mDepthQos.history(qos_hist);
        } else {
            RCLCPP_WARN(
                get_logger(),
                "The parameter '%s' is not available, using the default value",
                paramName.c_str());
        }

        RCLCPP_INFO(get_logger(),
            " * Depth QoS History: %s",
            sl_tools::qos2str(qos_hist).c_str());

        // ------------------------------------------

        paramName = "depth.qos_depth";
        declare_parameter(paramName, rclcpp::ParameterValue(qos_depth));

        if (get_parameter(paramName, paramVal)) {
            qos_depth = paramVal.as_int();
            mDepthQos.keep_last(qos_depth);
        } else {
            RCLCPP_WARN(
                get_logger(),
                "The parameter '%s' is not available, using the default value",
                paramName.c_str());
        }

        RCLCPP_INFO(get_logger(), " * Depth QoS History depth: %d", qos_depth);

        // ------------------------------------------

        paramName = "depth.qos_reliability";
        declare_parameter(paramName, rclcpp::ParameterValue(qos_reliability));

        if (get_parameter(paramName, paramVal)) {
            qos_reliability = paramVal.as_int() == 1
                ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
            mDepthQos.reliability(qos_reliability);
        } else {
            RCLCPP_WARN(
                get_logger(),
                "The parameter '%s' is not available, using the default value",
                paramName.c_str());
        }

        RCLCPP_INFO(get_logger(),
            " * Depth QoS Reliability: %s",
            sl_tools::qos2str(qos_reliability).c_str());

        // ------------------------------------------

        paramName = "depth.qos_durability";
        declare_parameter(paramName, rclcpp::ParameterValue(qos_durability));

        if (get_parameter(paramName, paramVal)) {
            qos_durability = paramVal.as_int() == 1
                ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
                : RMW_QOS_POLICY_DURABILITY_VOLATILE;
            mDepthQos.durability(qos_durability);
        } else {
            RCLCPP_WARN(
                get_logger(),
                "The parameter '%s' is not available, using the default value",
                paramName.c_str());
        }

        RCLCPP_INFO(get_logger(),
            " * Depth QoS Durability: %s",
            sl_tools::qos2str(qos_durability).c_str());
    }
}

void ZedCamera::getSensorsParams()
{
    rclcpp::Parameter paramVal;
    std::string paramName;

    rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    int qos_depth = 1;
    rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    RCLCPP_INFO(get_logger(), "*** SENSORS STACK parameters ***");
    if (mCamUserModel == sl::MODEL::ZED) {
        RCLCPP_WARN(get_logger(),
            "!!! SENSORS parameters are not used with ZED !!!");
    }

    getParam("sensors.sensors_image_sync", mSensCameraSync, mSensCameraSync);
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * Sensors Camera Sync: " << (mSensCameraSync ? "TRUE" : "FALSE"));

    getParam("sensors.sensors_pub_rate", mSensPubRate, mSensPubRate);
    if (mSensPubRate < mCamGrabFrameRate) {
        mSensPubRate = mCamGrabFrameRate;
    }
    RCLCPP_INFO_STREAM(get_logger(),
        " * Sensors publishing rate: " << mSensPubRate << " Hz");

    // ------------------------------------------

    paramName = "sensors.qos_history";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_hist));

    if (get_parameter(paramName, paramVal)) {
        qos_hist = paramVal.as_int() == 1 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST
                                          : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        mSensQos.history(qos_hist);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Sensors QoS History: %s",
        sl_tools::qos2str(qos_hist).c_str());

    // ------------------------------------------

    paramName = "sensors.qos_depth";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_depth));

    if (get_parameter(paramName, paramVal)) {
        qos_depth = paramVal.as_int();
        mSensQos.keep_last(qos_depth);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Sensors QoS History depth: %d", qos_depth);

    // ------------------------------------------

    paramName = "sensors.qos_reliability";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_reliability));

    if (get_parameter(paramName, paramVal)) {
        qos_reliability = paramVal.as_int() == 1
            ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
            : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        mSensQos.reliability(qos_reliability);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Sensors QoS Reliability: %s",
        sl_tools::qos2str(qos_reliability).c_str());

    // ------------------------------------------

    paramName = "sensors.qos_durability";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_durability));

    if (get_parameter(paramName, paramVal)) {
        qos_durability = paramVal.as_int() == 1
            ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
            : RMW_QOS_POLICY_DURABILITY_VOLATILE;
        mSensQos.durability(qos_durability);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Sensors QoS Durability: %s",
        sl_tools::qos2str(qos_durability).c_str());
}

void ZedCamera::getMappingParams()
{
    rclcpp::Parameter paramVal;
    std::string paramName;

    rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    int qos_depth = 1;
    rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    RCLCPP_INFO(get_logger(), "*** Spatial Mapping parameters ***");

    getParam("mapping.mapping_enabled", mMappingEnabled, mMappingEnabled);
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * Spatial Mapping Enabled: " << (mMappingEnabled ? "TRUE" : "FALSE"));

    getParam("mapping.resolution",
        mMappingRes,
        mMappingRes,
        " * Spatial Mapping resolution [m]: ");
    getParam("mapping.max_mapping_range", mMappingRangeMax, mMappingRangeMax);
    if (mMappingRangeMax == -1.0f) {
        RCLCPP_INFO(get_logger(), " * 3D Max Mapping range: AUTO");
    } else {
        RCLCPP_INFO_STREAM(get_logger(),
            " * 3D Max Mapping range [m]: " << mMappingRangeMax);
    }
    getParam("mapping.fused_pointcloud_freq",
        mFusedPcPubRate,
        mFusedPcPubRate,
        " * Map publishing rate [Hz]: ");

    getParam("mapping.clicked_point_topic",
        mClickedPtTopic,
        mClickedPtTopic,
        " * Clicked point topic: ");
    // ------------------------------------------

    paramName
        = "mapping.qos_history";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_hist));

    if (get_parameter(paramName, paramVal)) {
        qos_hist = paramVal.as_int() == 1 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST
                                          : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        mMappingQos.history(qos_hist);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Sensors QoS History: %s",
        sl_tools::qos2str(qos_hist).c_str());

    // ------------------------------------------

    paramName = "mapping.qos_depth";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_depth));

    if (get_parameter(paramName, paramVal)) {
        qos_depth = paramVal.as_int();
        mMappingQos.keep_last(qos_depth);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Sensors QoS History depth: %d", qos_depth);

    // ------------------------------------------

    paramName = "mapping.qos_reliability";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_reliability));

    if (get_parameter(paramName, paramVal)) {
        qos_reliability = paramVal.as_int() == 1
            ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
            : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        mMappingQos.reliability(qos_reliability);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Sensors QoS Reliability: %s",
        sl_tools::qos2str(qos_reliability).c_str());

    // ------------------------------------------

    paramName = "mapping.qos_durability";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_durability));

    if (get_parameter(paramName, paramVal)) {
        qos_durability = paramVal.as_int() == 1
            ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
            : RMW_QOS_POLICY_DURABILITY_VOLATILE;
        mMappingQos.durability(qos_durability);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Sensors QoS Durability: %s",
        sl_tools::qos2str(qos_durability).c_str());
}

void ZedCamera::getPosTrackingParams()
{
    rclcpp::Parameter paramVal;
    std::string paramName;

    rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    int qos_depth = 1;
    rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    RCLCPP_INFO(get_logger(), "*** POSITIONAL TRACKING parameters ***");

    getParam("pos_tracking.pos_tracking_enabled",
        mPosTrackingEnabled,
        mPosTrackingEnabled);
    RCLCPP_INFO_STREAM(get_logger(),
        " * Positional tracking enabled: "
            << (mPosTrackingEnabled ? "TRUE" : "FALSE"));

    getParam("pos_tracking.base_frame",
        mBaseFrameId,
        mBaseFrameId,
        " * Base frame id: ");
    getParam(
        "pos_tracking.map_frame", mMapFrameId, mMapFrameId, " * Map frame id: ");
    getParam("pos_tracking.odometry_frame",
        mOdomFrameId,
        mOdomFrameId,
        " * Odometry frame id: ");

    getParam("pos_tracking.publish_tf", mPublishTF, mPublishTF);
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * Broadcast Odometry TF: " << (mPublishTF ? "TRUE" : "FALSE"));
    if (mPublishTF) {
        getParam("pos_tracking.publish_map_tf", mPublishMapTF, mPublishMapTF);
        RCLCPP_INFO_STREAM(
            get_logger(),
            " * Broadcast Pose TF: " << (mPublishMapTF ? "TRUE" : "FALSE"));
        getParam("pos_tracking.publish_imu_tf", mPublishImuTF, mPublishImuTF);
        RCLCPP_INFO_STREAM(get_logger(),
            " * Broadcast Static IMU TF [not for ZED]: "
                << (mPublishImuTF ? "TRUE" : "FALSE"));
    }

    getParam("pos_tracking.path_pub_rate",
        mPathPubRate,
        mPathPubRate,
        " * [DYN] Path publishing rate: ");
    getParam("pos_tracking.path_max_count", mPathMaxCount, mPathMaxCount);
    if (mPathMaxCount < 2 && mPathMaxCount != -1) {
        mPathMaxCount = 2;
    }
    RCLCPP_INFO_STREAM(get_logger(), " * Path history lenght: " << mPathMaxCount);

    paramName = "pos_tracking.initial_base_pose";
    declare_parameter(paramName, rclcpp::ParameterValue(mInitialBasePose));
    if (!get_parameter(paramName, mInitialBasePose)) {
        RCLCPP_WARN_STREAM(
            get_logger(),
            "The parameter '"
                << paramName
                << "' is not available or is not valid, using the default value");
        mInitialBasePose = std::vector<double>(6, 0.0);
    }
    if (mInitialBasePose.size() < 6) {
        RCLCPP_WARN_STREAM(get_logger(),
            "The parameter '"
                << paramName
                << "' must be a vector of 6 values of double type");
        mInitialBasePose = std::vector<double>(6, 0.0);
    }
    RCLCPP_INFO(get_logger(),
        " * Initial pose: [%g,%g,%g,%g,%g,%g,]",
        mInitialBasePose[0],
        mInitialBasePose[1],
        mInitialBasePose[2],
        mInitialBasePose[3],
        mInitialBasePose[4],
        mInitialBasePose[5]);

    getParam("pos_tracking.area_memory", mAreaMemory, mAreaMemory);
    RCLCPP_INFO_STREAM(get_logger(),
        " * Area Memory: " << (mAreaMemory ? "TRUE" : "FALSE"));
    getParam("pos_tracking.area_memory_db_path",
        mAreaMemoryDbPath,
        mAreaMemoryDbPath,
        " * Area Memory DB: ");
    getParam("pos_tracking.imu_fusion", mImuFusion, mImuFusion);
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * IMU Fusion [not for ZED]: " << (mImuFusion ? "TRUE" : "FALSE"));
    getParam("pos_tracking.floor_alignment", mFloorAlignment, mFloorAlignment);
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * Floor Alignment: " << (mFloorAlignment ? "TRUE" : "FALSE"));
    getParam("pos_tracking.init_odom_with_first_valid_pose",
        mInitOdomWithPose,
        mInitOdomWithPose);
    RCLCPP_INFO_STREAM(get_logger(),
        " * Init Odometry with first valid pose data: "
            << (mInitOdomWithPose ? "TRUE" : "FALSE"));
    getParam("pos_tracking.two_d_mode", mTwoDMode, mTwoDMode);
    RCLCPP_INFO_STREAM(get_logger(),
        " * 2D mode: " << (mTwoDMode ? "TRUE" : "FALSE"));
    if (mTwoDMode) {
        getParam("pos_tracking.fixed_z_value",
            mFixedZValue,
            mFixedZValue,
            " * Fixed Z value: ");
    }

    // ------------------------------------------

    paramName = "pos_tracking.qos_history";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_hist));

    if (get_parameter(paramName, paramVal)) {
        qos_hist = paramVal.as_int() == 1 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST
                                          : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        mPoseQos.history(qos_hist);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Pose/Odometry QoS History: %s",
        sl_tools::qos2str(qos_hist).c_str());

    // ------------------------------------------

    paramName = "pos_tracking.qos_depth";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_depth));

    if (get_parameter(paramName, paramVal)) {
        qos_depth = paramVal.as_int();
        mPoseQos.keep_last(qos_depth);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(
        get_logger(), " * Pose/Odometry QoS History depth: %d", qos_depth);

    // ------------------------------------------

    paramName = "pos_tracking.qos_reliability";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_reliability));

    if (get_parameter(paramName, paramVal)) {
        qos_reliability = paramVal.as_int() == 1
            ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
            : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        mPoseQos.reliability(qos_reliability);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Pose/Odometry QoS Reliability: %s",
        sl_tools::qos2str(qos_reliability).c_str());

    // ------------------------------------------

    paramName = "pos_tracking.qos_durability";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_durability));

    if (get_parameter(paramName, paramVal)) {
        qos_durability = paramVal.as_int() == 1
            ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
            : RMW_QOS_POLICY_DURABILITY_VOLATILE;
        mPoseQos.durability(qos_durability);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Pose/Odometry QoS Durability: %s",
        sl_tools::qos2str(qos_durability).c_str());
}

void ZedCamera::getOdParams()
{
    rclcpp::Parameter paramVal;
    std::string paramName;

    rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    int qos_depth = 1;
    rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    RCLCPP_INFO(get_logger(), "*** OBJECT DETECTION parameters ***");
    if (mCamUserModel == sl::MODEL::ZED || mCamUserModel == sl::MODEL::ZED_M) {
        RCLCPP_WARN(get_logger(),
            "!!! OD parameters are not used with ZED and ZED Mini !!!");
    }

    getParam("object_detection.od_enabled", mObjDetEnabled, mObjDetEnabled);
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * Object Detection enabled: " << (mObjDetEnabled ? "TRUE" : "FALSE"));
    getParam("object_detection.confidence_threshold",
        mObjDetConfidence,
        mObjDetConfidence,
        " * OD min. confidence: ");
    getParam("object_detection.object_tracking_enabled", mObjDetTracking,
        mObjDetTracking);
    RCLCPP_INFO_STREAM(get_logger(), " * OD tracking: " << (mObjDetTracking ? "TRUE" : "FALSE"));
    int model = 0;
    getParam("object_detection.model", model, model);
    mObjDetModel = static_cast<sl::DETECTION_MODEL>(model);
    RCLCPP_INFO_STREAM(get_logger(),
        " * Object Detection model: " << model << " - "
                                      << sl::toString(mObjDetModel).c_str());
    int filtering_mode = static_cast<int>(mObjFilterMode);
    getParam("object_detection.filtering_mode", filtering_mode, filtering_mode);
    mObjFilterMode = static_cast<sl::OBJECT_FILTERING_MODE>(filtering_mode);
    RCLCPP_INFO_STREAM(get_logger(),
        " * Object Filtering mode: " << filtering_mode << " - "
                                     << sl::toString(mObjFilterMode).c_str());
    getParam(
        "object_detection.mc_people", mObjDetPeopleEnable, mObjDetPeopleEnable);
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * MultiClassBox people: " << (mObjDetPeopleEnable ? "TRUE" : "FALSE"));
    getParam("object_detection.mc_vehicle",
        mObjDetVehiclesEnable,
        mObjDetVehiclesEnable);
    RCLCPP_INFO_STREAM(get_logger(),
        " * MultiClassBox vehicles: "
            << (mObjDetVehiclesEnable ? "TRUE" : "FALSE"));
    getParam("object_detection.mc_bag", mObjDetBagsEnable, mObjDetBagsEnable);
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * MultiClassBox bags: " << (mObjDetBagsEnable ? "TRUE" : "FALSE"));
    getParam(
        "object_detection.mc_animal", mObjDetAnimalsEnable, mObjDetAnimalsEnable);
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * MultiClassBox animals: " << (mObjDetAnimalsEnable ? "TRUE" : "FALSE"));
    getParam("object_detection.mc_electronics",
        mObjDetElectronicsEnable,
        mObjDetElectronicsEnable);
    RCLCPP_INFO_STREAM(get_logger(),
        " * MultiClassBox electronics: "
            << (mObjDetElectronicsEnable ? "TRUE" : "FALSE"));
    getParam("object_detection.mc_fruit_vegetable",
        mObjDetFruitsEnable,
        mObjDetFruitsEnable);
    RCLCPP_INFO_STREAM(get_logger(),
        " * MultiClassBox fruits and vegetables: "
            << (mObjDetFruitsEnable ? "TRUE" : "FALSE"));
    getParam("object_detection.mc_sport",
        mObjDetSportEnable,
        mObjDetSportEnable);
    RCLCPP_INFO_STREAM(get_logger(),
        " * MultiClassBox sport-related objects: "
            << (mObjDetSportEnable ? "TRUE" : "FALSE"));
    int bodyFormat = 0;
    getParam("object_detection.body_format", bodyFormat, bodyFormat);
    mObjDetBodyFmt = static_cast<sl::BODY_FORMAT>(bodyFormat);
    RCLCPP_INFO_STREAM(get_logger(), " * Body format: " << bodyFormat << " - " << sl::toString(mObjDetBodyFmt).c_str());
    if (mObjDetBodyFmt == sl::BODY_FORMAT::POSE_34) {
        RCLCPP_INFO_STREAM(
            get_logger(), " * Skeleton fitting: TRUE (forced by `object_detection.body_format`)");
        mObjDetBodyFitting = true;
    } else {
        getParam("object_detection.body_fitting", mObjDetBodyFitting, mObjDetBodyFitting);
        RCLCPP_INFO_STREAM(
            get_logger(), " * Skeleton fitting: " << (mObjDetBodyFitting ? "TRUE" : "FALSE"));
    }
    // ------------------------------------------

    paramName = "object_detection.qos_history";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_hist));

    if (get_parameter(paramName, paramVal)) {
        qos_hist = paramVal.as_int() == 1 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST
                                          : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        mObjDetQos.history(qos_hist);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Obj. Det. QoS History: %s",
        sl_tools::qos2str(qos_hist).c_str());

    // ------------------------------------------

    paramName = "object_detection.qos_depth";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_depth));

    if (get_parameter(paramName, paramVal)) {
        qos_depth = paramVal.as_int();
        mObjDetQos.keep_last(qos_depth);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(), " * Obj. Det. QoS History depth: %d", qos_depth);

    // ------------------------------------------

    paramName = "object_detection.qos_reliability";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_reliability));

    if (get_parameter(paramName, paramVal)) {
        qos_reliability = paramVal.as_int() == 1
            ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
            : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        mObjDetQos.reliability(qos_reliability);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Obj. Det. QoS Reliability: %s",
        sl_tools::qos2str(qos_reliability).c_str());

    // ------------------------------------------

    paramName = "object_detection.qos_durability";
    declare_parameter(paramName, rclcpp::ParameterValue(qos_durability));

    if (get_parameter(paramName, paramVal)) {
        qos_durability = paramVal.as_int() == 1
            ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
            : RMW_QOS_POLICY_DURABILITY_VOLATILE;
        mObjDetQos.durability(qos_durability);
    } else {
        RCLCPP_WARN(get_logger(),
            "The parameter '%s' is not available, using the default value",
            paramName.c_str());
    }

    RCLCPP_INFO(get_logger(),
        " * Obj. Det. QoS Durability: %s",
        sl_tools::qos2str(qos_durability).c_str());
}

rcl_interfaces::msg::SetParametersResult
ZedCamera::callback_paramChange(std::vector<rclcpp::Parameter> parameters)
{
    //RCLCPP_INFO(get_logger(), "Parameter change callback");

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    for (const auto& param : parameters) {
        if (param.get_name() == "general.pub_frame_rate") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            double val = param.as_double();

            if ((val <= 0.0) || (val > mCamGrabFrameRate)) {
                result.successful = false;
                result.reason = param.get_name() + " must be positive and minor or equal to `grab_frame_rate`";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mPubFrameRate = val;
            startVideoDepthTimer(mPubFrameRate);

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "video.brightness") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            int val = param.as_int();

            if ((val < 0) || (val > 8)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [0,8]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mCamBrightness = val;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "video.contrast") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            int val = param.as_int();

            if ((val < 0) || (val > 8)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [0,8]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mCamContrast = val;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "video.hue") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            int val = param.as_int();

            if ((val < 0) || (val > 11)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [0,11]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mCamHue = val;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "video.sharpness") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            int val = param.as_int();

            if ((val < 0) || (val > 8)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [0,8]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mCamSharpness = val;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "video.gamma") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            int val = param.as_int();

            if ((val < 0) || (val > 8)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [0,8]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mCamGamma = val;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "video.auto_exposure_gain") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            bool val = param.as_bool();

            if (val && !mCamAutoExpGain) {
                mTriggerAutoExpGain = true;
            }

            mCamAutoExpGain = val;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "video.exposure") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            int val = param.as_int();

            if ((val < 0) || (val > 100)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [0,100]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mCamExposure = val;
            mCamAutoExpGain = false;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "video.gain") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            int val = param.as_int();

            if ((val < 0) || (val > 100)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [0,100]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mCamGain = val;
            mCamAutoExpGain = false;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "video.auto_whitebalance") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            bool val = param.as_bool();

            if (val && !mCamAutoWB) {
                mTriggerAutoWB = true;
            }

            mCamAutoWB = val;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "video.whitebalance_temperature") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            int val = param.as_int();

            if ((val < 28) || (val > 65)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [28,65]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mCamWBTemp = val * 100;
            mCamAutoWB = false;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "depth.point_cloud_freq") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            double val = param.as_double();

            if ((val <= 0.0) || (val > mCamGrabFrameRate)) {
                result.successful = false;
                result.reason = param.get_name() + " must be positive and minor of `grab_frame_rate`";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mPubFrameRate = val;
            startVideoDepthTimer(mPubFrameRate);

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "depth.depth_confidence") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            int val = param.as_int();

            if ((val < 0) || (val > 100)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [0,100]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mDepthConf = val;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "depth.depth_texture_conf") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            int val = param.as_int();

            if ((val < 0) || (val > 100)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [0,100]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mDepthTextConf = val;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "depth.remove_saturated_areas") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mRemoveSatAreas = param.as_bool();

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '"
                    << param.get_name() << "' correctly set to "
                    << (mRemoveSatAreas ? "TRUE" : "FALSE"));
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "mapping.fused_pointcloud_freq") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            double val = param.as_double();

            if ((val <= 0.0) || (val > mPcPubRate)) {
                result.successful = false;
                result.reason = param.get_name() + " must be positive and minor of `point_cloud_freq`";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mFusedPcPubRate = val;
            startFusedPcTimer(mFusedPcPubRate);

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "object_detection.confidence_threshold") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            double val = param.as_double();

            if ((val < 0.0) || (val > 100.0)) {
                result.successful = false;
                result.reason = param.get_name() + " must be positive double value in the range [0.0,100.0]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mObjDetConfidence = val;

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '" << param.get_name()
                              << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "object_detection.mc_people") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mObjDetPeopleEnable = param.as_bool();

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '"
                    << param.get_name() << "' correctly set to "
                    << (mObjDetPeopleEnable ? "TRUE" : "FALSE"));
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "object_detection.mc_vehicle") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mObjDetVehiclesEnable = param.as_bool();

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '"
                    << param.get_name() << "' correctly set to "
                    << (mObjDetVehiclesEnable ? "TRUE" : "FALSE"));
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "object_detection.mc_bag") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mObjDetBagsEnable = param.as_bool();

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '"
                    << param.get_name() << "' correctly set to "
                    << (mObjDetBagsEnable ? "TRUE" : "FALSE"));
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "object_detection.mc_animal") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mObjDetAnimalsEnable = param.as_bool();

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '"
                    << param.get_name() << "' correctly set to "
                    << (mObjDetAnimalsEnable ? "TRUE" : "FALSE"));
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "object_detection.mc_electronics") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mObjDetElectronicsEnable = param.as_bool();

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '"
                    << param.get_name() << "' correctly set to "
                    << (mObjDetElectronicsEnable ? "TRUE" : "FALSE"));
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "object_detection.mc_fruit_vegetable") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mObjDetFruitsEnable = param.as_bool();

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '"
                    << param.get_name() << "' correctly set to "
                    << (mObjDetFruitsEnable ? "TRUE" : "FALSE"));
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "object_detection.mc_sport") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            mObjDetSportEnable = param.as_bool();

            RCLCPP_INFO_STREAM(get_logger(),
                "Parameter '"
                    << param.get_name() << "' correctly set to "
                    << (mObjDetSportEnable ? "TRUE" : "FALSE"));
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else {
            result.reason = param.get_name() + " is not a dynamic parameter";
        }
    }

    RCLCPP_WARN_STREAM(get_logger(), result.reason);
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
    mBaroFrameId = mCameraFrameId; // mCameraName + "_baro_link";   // TODO fix
        // when XACRO is available
    mMagFrameId = mImuFrameId; // mCameraName + "_mag_link"; // TODO fix when
        // XACRO is available
    mTempLeftFrameId = mLeftCamFrameId; // mCameraName + "_temp_left_link"; //
        // TODO fix when XACRO is available
    mTempRightFrameId = mRightCamFrameId; // mCameraName + "_temp_right_link"; //
        // TODO fix when XACRO is available

    mDepthFrameId = mLeftCamFrameId;
    mDepthOptFrameId = mLeftCamOptFrameId;
    mPointCloudFrameId = mDepthFrameId;

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
    RCLCPP_INFO_STREAM(get_logger(), " * Odometry\t\t\t-> " << mOdomFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Base\t\t\t-> " << mBaseFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Camera\t\t\t-> " << mCameraFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Left\t\t\t-> " << mLeftCamFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Left Optical\t\t-> " << mLeftCamOptFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * RGB\t\t\t-> " << mRgbFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * RGB Optical\t\t-> " << mRgbFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Right\t\t\t-> " << mRightCamFrameId);
    RCLCPP_INFO_STREAM(get_logger(), " * Right Optical\t\t-> " << mRightCamOptFrameId);
    if(!mDepthDisabled)
    {
        RCLCPP_INFO_STREAM(get_logger(), " * Depth\t\t\t-> " << mDepthFrameId);
        RCLCPP_INFO_STREAM(get_logger(), " * Depth Optical\t\t-> " << mDepthOptFrameId);
        RCLCPP_INFO_STREAM(get_logger(), " * Point Cloud\t\t-> " << mCloudFrameId);
        RCLCPP_INFO_STREAM(get_logger(), " * Disparity\t\t-> " << mDisparityFrameId);
        RCLCPP_INFO_STREAM(get_logger(), " * Disparity Optical\t-> " << mDisparityOptFrameId);
        RCLCPP_INFO_STREAM(get_logger(), " * Confidence\t\t-> " << mConfidenceFrameId);
        RCLCPP_INFO_STREAM(get_logger(), " * Confidence Optical\t-> " << mConfidenceOptFrameId);
    }

    if (mCamRealModel != sl::MODEL::ZED) {
        RCLCPP_INFO_STREAM(get_logger(), " * IMU\t\t\t-> " << mImuFrameId);

        if (sl_tools::isZED2OrZED2i(mCamUserModel)) {
            RCLCPP_INFO_STREAM(get_logger(), " * Barometer\t\t-> " << mBaroFrameId);
            RCLCPP_INFO_STREAM(get_logger(), " * Magnetometer\t\t-> " << mMagFrameId);
            RCLCPP_INFO_STREAM(get_logger(),
                " * Left Temperature\t-> " << mTempLeftFrameId);
            RCLCPP_INFO_STREAM(get_logger(),
                " * Right Temperature\t-> " << mTempRightFrameId);
        }
    }
    // <---- Coordinate frames
}

void ZedCamera::fillCamInfo(
    sl::Camera& zed,
    std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
    std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg,
    std::string leftFrameId,
    std::string rightFrameId,
    bool rawParam /*= false*/)
{
    sl::CalibrationParameters zedParam;

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    if (rawParam) {
        zedParam = zed.getCameraInformation(mMatResolVideo).calibration_parameters_raw; // ok
    } else {
        zedParam = zed.getCameraInformation(mMatResolVideo).calibration_parameters; // ok
    }
#else
    if (rawParam) {
        zedParam = zed.getCameraInformation(mMatResolVideo)
                       .camera_configuration.calibration_parameters_raw;
    } else {
        zedParam = zed.getCameraInformation(mMatResolVideo)
                       .camera_configuration.calibration_parameters;
    }
#endif

    float baseline = zedParam.getCameraBaseline();
    leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    leftCamInfoMsg->d.resize(5);
    rightCamInfoMsg->d.resize(5);
    leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0]; // k1
    leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1]; // k2
    leftCamInfoMsg->d[2] = zedParam.left_cam.disto[4]; // k3
    leftCamInfoMsg->d[3] = zedParam.left_cam.disto[2]; // p1
    leftCamInfoMsg->d[4] = zedParam.left_cam.disto[3]; // p2
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

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    if (rawParam) {
        std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
        float* p = R_.data();

        for (int i = 0; i < 9; i++) {
            rightCamInfoMsg->r[i] = p[i];
        }
    }
#else
    if (rawParam) {
        if (mUseOldExtrinsic) { // Camera frame (Z forward, Y down, X right)

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

void ZedCamera::initPublishers()
{
    RCLCPP_INFO(get_logger(), "*** PUBLISHED TOPICS ***");

    std::string topicPrefix = get_namespace();

    if (topicPrefix.length() > 1) {
        topicPrefix += "/";
    }

    topicPrefix += get_name();
    topicPrefix += "/";
    // ----> Topics names definition
    std::string rgbTopicRoot = "rgb";
    std::string rightTopicRoot = "right";
    std::string leftTopicRoot = "left";
    std::string stereoTopicRoot = "stereo";
    std::string img_topic = "/image_rect_color";
    std::string img_raw_topic = "/image_raw_color";
    std::string img_gray_topic = "/image_rect_gray";
    std::string img_raw_gray_topic_ = "/image_raw_gray";
    std::string raw_suffix = "_raw";
    std::string left_topic = mTopicRoot + leftTopicRoot + img_topic;
    std::string left_raw_topic = mTopicRoot + leftTopicRoot + raw_suffix + img_raw_topic;
    std::string right_topic = mTopicRoot + rightTopicRoot + img_topic;
    std::string right_raw_topic = mTopicRoot + rightTopicRoot + raw_suffix + img_raw_topic;
    std::string rgb_topic = mTopicRoot + rgbTopicRoot + img_topic;
    std::string rgb_raw_topic = mTopicRoot + rgbTopicRoot + raw_suffix + img_raw_topic;
    std::string stereo_topic = mTopicRoot + stereoTopicRoot + img_topic;
    std::string stereo_raw_topic = mTopicRoot + stereoTopicRoot + raw_suffix + img_raw_topic;
    std::string left_gray_topic = mTopicRoot + leftTopicRoot + img_gray_topic;
    std::string left_raw_gray_topic = mTopicRoot + leftTopicRoot + raw_suffix + img_raw_gray_topic_;
    std::string right_gray_topic = mTopicRoot + rightTopicRoot + img_gray_topic;
    std::string right_raw_gray_topic = mTopicRoot + rightTopicRoot + raw_suffix + img_raw_gray_topic_;
    std::string rgb_gray_topic = mTopicRoot + rgbTopicRoot + img_gray_topic;
    std::string rgb_raw_gray_topic = mTopicRoot + rgbTopicRoot + raw_suffix + img_raw_gray_topic_;

    // Set the disparity topic name
    std::string disparity_topic = mTopicRoot + "disparity/disparity_image";

    // Set the depth topic names
    std::string depth_topic_root = "depth";

    if (mOpenniDepthMode) {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "Openni depth mode activated -> Units: mm, Encoding: MONO16");
    }
    std::string depth_topic = mTopicRoot + depth_topic_root + "/depth_registered";
    std::string depth_info_topic = mTopicRoot + depth_topic_root + "/depth_info";

    std::string pointcloud_topic = mTopicRoot + "point_cloud/cloud_registered";
    mPointcloudFusedTopic = mTopicRoot + "mapping/fused_cloud";

    std::string object_det_topic_root = "obj_det";
    mObjectDetTopic = mTopicRoot + object_det_topic_root + "/objects";

    std::string confImgRoot = "confidence";
    std::string conf_map_topic_name = "confidence_map";
    std::string conf_map_topic = mTopicRoot + confImgRoot + "/" + conf_map_topic_name;

    // Set the positional tracking topic names
    mPoseTopic = mTopicRoot + "pose";
    mPoseCovTopic = mPoseTopic + "_with_covariance";

    mOdomTopic = mTopicRoot + "odom";
    mOdomPathTopic = mTopicRoot + "path_odom";
    mMapPathTopic = mTopicRoot + "path_map";

    // Set the Sensors topic names
    std::string temp_topic_root = "temperature";
    std::string imuTopicRoot = "imu";
    std::string imu_topic_name = "data";
    std::string imu_topic_raw_name = "data_raw";
    std::string imu_topic_mag_name = "mag";
    // std::string imu_topic_mag_raw_name = "mag_raw";
    std::string pressure_topic_name = "atm_press";

    std::string imu_topic = mTopicRoot + imuTopicRoot + "/" + imu_topic_name;
    std::string imu_topic_raw = mTopicRoot + imuTopicRoot + "/" + imu_topic_raw_name;
    std::string imu_temp_topic = mTopicRoot + temp_topic_root + "/" + imuTopicRoot;
    std::string imu_mag_topic = mTopicRoot + imuTopicRoot + "/" + imu_topic_mag_name;
    // std::string imu_mag_topic_raw = imuTopicRoot + "/" +
    // imu_topic_mag_raw_name;
    std::string pressure_topic = mTopicRoot + /*imuTopicRoot + "/" +*/ pressure_topic_name;
    std::string temp_topic_left = mTopicRoot + temp_topic_root + "/left";
    std::string temp_topic_right = mTopicRoot + temp_topic_root + "/right";
    // <---- Topics names definition

    // ----> Camera publishers
    mPubRgb = image_transport::create_camera_publisher(this, rgb_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRgb.getTopic());
    mPubRgbGray = image_transport::create_camera_publisher(
        this, rgb_gray_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRgbGray.getTopic());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRgb.getInfoTopic());
    mPubRawRgb = image_transport::create_camera_publisher(
        this, rgb_raw_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRawRgb.getTopic());
    mPubRawRgbGray = image_transport::create_camera_publisher(
        this, rgb_raw_gray_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRawRgbGray.getTopic());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRawRgb.getInfoTopic());
    mPubLeft = image_transport::create_camera_publisher(
        this, left_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubLeft.getTopic());
    mPubLeftGray = image_transport::create_camera_publisher(
        this, left_gray_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubLeftGray.getTopic());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubLeft.getInfoTopic());
    mPubRawLeft = image_transport::create_camera_publisher(
        this, left_raw_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRawLeft.getTopic());
    mPubRawLeftGray = image_transport::create_camera_publisher(
        this, left_raw_gray_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRawLeftGray.getTopic());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRawLeft.getInfoTopic());
    mPubRight = image_transport::create_camera_publisher(
        this, right_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRight.getTopic());
    mPubRightGray = image_transport::create_camera_publisher(
        this, right_gray_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRightGray.getTopic());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRight.getInfoTopic());
    mPubRawRight = image_transport::create_camera_publisher(
        this, right_raw_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRawRight.getTopic());
    mPubRawRightGray = image_transport::create_camera_publisher(
        this, right_raw_gray_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRawRightGray.getTopic());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRawRight.getInfoTopic());

    if (!mDepthDisabled) {
        mPubDepth = image_transport::create_camera_publisher(
            this, depth_topic, mDepthQos.get_rmw_qos_profile());
        RCLCPP_INFO_STREAM(get_logger(),
            "Advertised on topic: " << mPubDepth.getTopic());
        RCLCPP_INFO_STREAM(get_logger(),
            "Advertised on topic: " << mPubDepth.getInfoTopic());
        mPubDepthInfo = create_publisher<zed_interfaces::msg::DepthInfoStamped>(
            depth_info_topic, mDepthQos);
        RCLCPP_INFO_STREAM(get_logger(),
            "Advertised on topic: " << mPubDepthInfo->get_topic_name());
    }

    mPubStereo = image_transport::create_publisher(
        this, stereo_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubStereo.getTopic());
    mPubRawStereo = image_transport::create_publisher(
        this, stereo_raw_topic, mVideoQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(),
        "Advertised on topic: " << mPubRawStereo.getTopic());
    // <---- Camera publishers

    // ----> Depth publishers
    if (!mDepthDisabled) {
        mPubConfMap = create_publisher<sensor_msgs::msg::Image>(conf_map_topic, mDepthQos);
        RCLCPP_INFO_STREAM(
            get_logger(), "Advertised on topic: " << mPubConfMap->get_topic_name());
        mPubDisparity = create_publisher<stereo_msgs::msg::DisparityImage>(
            disparity_topic, mDepthQos);
        RCLCPP_INFO_STREAM(
            get_logger(), "Advertised on topic: " << mPubDisparity->get_topic_name());
        mPubCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic, mDepthQos);
        RCLCPP_INFO_STREAM(get_logger(),
            "Advertised on topic: " << mPubCloud->get_topic_name());
    }
    // <---- Depth publishers

    // ----> Pos Tracking
    if (!mDepthDisabled) {
        mPubPose = create_publisher<geometry_msgs::msg::PoseStamped>(mPoseTopic, mPoseQos);
        RCLCPP_INFO_STREAM(get_logger(),
            "Advertised on topic: " << mPubPose->get_topic_name());
        mPubPoseCov = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            mPoseCovTopic, mPoseQos);
        RCLCPP_INFO_STREAM(
            get_logger(), "Advertised on topic: " << mPubPoseCov->get_topic_name());
        mPubOdom = create_publisher<nav_msgs::msg::Odometry>(mOdomTopic, mPoseQos);
        RCLCPP_INFO_STREAM(get_logger(),
            "Advertised on topic: " << mPubOdom->get_topic_name());
        mPubPosePath = create_publisher<nav_msgs::msg::Path>(mMapPathTopic, mPoseQos);
        RCLCPP_INFO_STREAM(
            get_logger(), "Advertised on topic: " << mPubPosePath->get_topic_name());
        mPubOdomPath = create_publisher<nav_msgs::msg::Path>(mOdomPathTopic, mPoseQos);
        RCLCPP_INFO_STREAM(
            get_logger(), "Advertised on topic: " << mPubOdomPath->get_topic_name());
    }
    // <---- Pos Tracking

    // ----> Mapping
    if (!mDepthDisabled && mMappingEnabled) {
        mPubFusedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            mPointcloudFusedTopic, mMappingQos);
        RCLCPP_INFO_STREAM(get_logger(),
            "Advertised on topic "
                << mPubFusedCloud->get_topic_name() << " @ "
                << mFusedPcPubRate << " Hz");
    
        std::string marker_topic = "plane_marker";
        std::string plane_topic = "plane";
        // Rviz markers publisher
        mPubMarker = create_publisher<visualization_msgs::msg::Marker>(marker_topic, mMappingQos);
        RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << mPubMarker->get_topic_name());
        // Detected planes publisher
        mPubPlane = create_publisher<zed_interfaces::msg::PlaneStamped>(plane_topic, mMappingQos);
        RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << mPubPlane->get_topic_name());
    }
    // <---- Mapping

    // ----> Sensors
    if (mCamRealModel != sl::MODEL::ZED) 
    {
        mPubImu = create_publisher<sensor_msgs::msg::Imu>(imu_topic, mSensQos);
        RCLCPP_INFO_STREAM(get_logger(),
            "Advertised on topic: " << mPubImu->get_topic_name());
        mPubImuRaw = create_publisher<sensor_msgs::msg::Imu>(imu_topic_raw, mSensQos);
        RCLCPP_INFO_STREAM(get_logger(),
            "Advertised on topic: " << mPubImuRaw->get_topic_name());
        mPubImuTemp = create_publisher<sensor_msgs::msg::Temperature>(imu_temp_topic, mSensQos);
        RCLCPP_INFO_STREAM(
            get_logger(), "Advertised on topic: " << mPubImuTemp->get_topic_name());

        if (sl_tools::isZED2OrZED2i(mCamRealModel))
        {
            mPubImuMag = create_publisher<sensor_msgs::msg::MagneticField>(
                imu_mag_topic, mSensQos);
            RCLCPP_INFO_STREAM(
                get_logger(), "Advertised on topic: " << mPubImuMag->get_topic_name());
            mPubPressure = create_publisher<sensor_msgs::msg::FluidPressure>(
                pressure_topic, mSensQos);
            RCLCPP_INFO_STREAM(
                get_logger(),
                "Advertised on topic: " << mPubPressure->get_topic_name());
            mPubTempL = create_publisher<sensor_msgs::msg::Temperature>(
                temp_topic_left, mSensQos);
            RCLCPP_INFO_STREAM(
                get_logger(), "Advertised on topic: " << mPubTempL->get_topic_name());
            mPubTempR = create_publisher<sensor_msgs::msg::Temperature>(
                temp_topic_right, mSensQos);
            RCLCPP_INFO_STREAM(
                get_logger(), "Advertised on topic: " << mPubTempR->get_topic_name());
        }

        // ----> Publish latched camera/imu transform message
        publishStaticImuFrameAndTopic();
        // <---- Publish latched camera/imu transform message
    }
    // <---- Sensors

    // ----> Subscribers
    // Subscribers
    /* From `$ ros2 topic info /clicked_point -v`
        QoS profile:
            Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
            Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
            Lifespan: 2147483651294967295 nanoseconds
            Deadline: 2147483651294967295 nanoseconds
            Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
            Liveliness lease duration: 2147483651294967295 nanoseconds
    */
    if(!mDepthDisabled)
    {
        mClickedPtQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        mClickedPtQos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        mClickedPtSub = create_subscription<geometry_msgs::msg::PointStamped>(
            mClickedPtTopic, mClickedPtQos,
            std::bind(&ZedCamera::callback_clickedPoint, this, _1));

        RCLCPP_INFO_STREAM(
            get_logger(), "Subscribed to topic " << mClickedPtTopic.c_str());
    }
    // <---- Subscribers
}

bool ZedCamera::startCamera()
{
    RCLCPP_INFO(get_logger(), "***** STARTING CAMERA *****");

    // ----> SDK version
    RCLCPP_INFO(get_logger(),
        "SDK Version: %d.%d.%d - Build %s",
        ZED_SDK_MAJOR_VERSION,
        ZED_SDK_MINOR_VERSION,
        ZED_SDK_PATCH_VERSION,
        ZED_SDK_BUILD_ID);
    // <---- SDK version

    // ----> TF2 Transform
    mTfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    mTfListener = std::make_shared<tf2_ros::TransformListener>(
        *mTfBuffer); // Start TF Listener thread
    mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    mStaticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // <---- TF2 Transform

    // ----> ZED configuration
    if (!mSvoFilepath.empty()) {
        RCLCPP_INFO(get_logger(), "*** SVO OPENING ***");

        mInitParams.input.setFromSVOFile(mSvoFilepath.c_str());
        mInitParams.svo_real_time_mode = mSvoRealtime;
        mSvoMode = true;
    } else {
        RCLCPP_INFO(get_logger(), "*** CAMERA OPENING ***");

        mInitParams.camera_fps = mCamGrabFrameRate;
        mInitParams.camera_resolution = static_cast<sl::RESOLUTION>(mCamResol);

        if (mCamSerialNumber == 0) {
            mInitParams.input.setFromCameraID(mCamId);
        } else {
            mInitParams.input.setFromSerialNumber(mCamSerialNumber);
        }
    }

    mInitParams.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    mInitParams.coordinate_units = sl::UNIT::METER;
    mInitParams.depth_mode = mDepthQuality;
    mInitParams.sdk_verbose = mVerbose;
    mInitParams.sdk_gpu_id = mGpuId;
    mInitParams.depth_stabilization = static_cast<int>(mDepthStabilization);
    mInitParams.camera_image_flip = mCameraFlip;
    mInitParams.depth_minimum_distance = mCamMinDepth;
    mInitParams.depth_maximum_distance = mCamMaxDepth;

    mInitParams.camera_disable_self_calib = !mCameraSelfCalib;
    mInitParams.enable_image_enhancement = true;
    mInitParams.enable_right_side_measure = false;
    // <---- ZED configuration

    // ----> Try to open ZED camera or to load SVO
    // INIT_TIMER;
    // START_TIMER;
    sl_tools::StopWatch connectTimer;

    mThreadStop = false;

    if (!mSvoMode) {
        if (mCamSerialNumber == 0) {
            mInitParams.input.setFromCameraID(mCamId);
        } else {
            bool waiting_for_camera = true;

            while (waiting_for_camera) {
                // Ctrl+C check
                if (!rclcpp::ok()) {
                    return false;
                }

                sl::DeviceProperties prop = sl_tools::getZEDFromSN(mCamSerialNumber);

                if (prop.id < -1 || prop.camera_state == sl::CAMERA_STATE::NOT_AVAILABLE) {
                    std::string msg = "Camera with SN " + std::to_string(mCamSerialNumber) + " not detected! Please verify the connection.";
                    RCLCPP_INFO(get_logger(), msg.c_str());
                } else {
                    waiting_for_camera = false;
                    mInitParams.input.setFromCameraID(prop.id);
                }

                if (connectTimer.toc() >= mMaxReconnectTemp * mCamTimeoutSec) {
                    RCLCPP_ERROR(get_logger(), "Camera detection timeout");

                    return false;
                }

                std::this_thread::sleep_for(std::chrono::seconds(mCamTimeoutSec));
            }
        }
    }

    while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        mConnStatus = mZed.open(mInitParams);

        if (mConnStatus == sl::ERROR_CODE::SUCCESS) {
            RCLCPP_DEBUG(get_logger(), "Opening successfull");
            break;
        }

        if (mSvoMode) {
            RCLCPP_WARN(get_logger(),
                "Error opening SVO: %s",
                sl::toString(mConnStatus).c_str());

            return false;
        }

        RCLCPP_WARN(get_logger(),
            "Error opening camera: %s",
            sl::toString(mConnStatus).c_str());

        if (mConnStatus == sl::ERROR_CODE::CAMERA_DETECTION_ISSUE && mCamUserModel == sl::MODEL::ZED_M) {
            RCLCPP_INFO(get_logger(), "Try to flip the USB3 Type-C connector");
        } else {
            RCLCPP_INFO(get_logger(), "Please verify the USB3 connection");
        }

        if (!rclcpp::ok() || mThreadStop) {
            RCLCPP_INFO(get_logger(), "ZED activation interrupted");

            return false;
        }

        if (connectTimer.toc() > mMaxReconnectTemp * mCamTimeoutSec) {
            RCLCPP_ERROR(get_logger(), "Camera detection timeout");

            return false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(mCamTimeoutSec));
    }
    // <---- Try to open ZED camera or to load SVO

    // ----> Camera information
    sl::CameraInformation camInfo = mZed.getCameraInformation();

    float realFps = camInfo.camera_configuration.fps;
    if (realFps != static_cast<float>(mCamGrabFrameRate)) {
        RCLCPP_WARN_STREAM(get_logger(),
            "!!! `general.grab_frame_rate` value is not valid: '"
                << mCamGrabFrameRate
                << "'. Automatically replaced with '" << realFps
                << "'. Please fix the parameter !!!");
        mCamGrabFrameRate = realFps;
    }

    // CUdevice devid;
    cuCtxGetDevice(&mGpuId);
    RCLCPP_INFO_STREAM(get_logger(), "ZED SDK running on GPU #" << mGpuId);

    // Camera model
    mCamRealModel = camInfo.camera_model;

    if (mCamRealModel == sl::MODEL::ZED) {
        if (mCamUserModel != sl::MODEL::ZED) {
            RCLCPP_WARN(get_logger(),
                "Camera model does not match user parameter. Please modify "
                "the value of the parameter 'general.camera_model' to 'zed'");
        }
    } else if (mCamRealModel == sl::MODEL::ZED_M) {
        if (mCamUserModel != sl::MODEL::ZED_M) {
            RCLCPP_WARN(
                get_logger(),
                "Camera model does not match user parameter. Please modify "
                "the value of the parameter 'general.camera_model' to 'zedm'");
        }
    } else if (mCamRealModel == sl::MODEL::ZED2) {
        if (mCamUserModel != sl::MODEL::ZED2) {
            RCLCPP_WARN(
                get_logger(),
                "Camera model does not match user parameter. Please modify "
                "the value of the parameter 'general.camera_model' to 'zed2'");
        }
    } else if (mCamRealModel == sl::MODEL::ZED2i) {
        if (mCamUserModel != sl::MODEL::ZED2i) {
            RCLCPP_WARN(
                get_logger(),
                "Camera model does not match user parameter. Please modify "
                "the value of the parameter 'general.camera_model' to 'zed2i'");
        }
    }
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 5
    else if (mCamRealModel == sl::MODEL::ZED2i) {
        if (mCamUserModel != sl::MODEL::ZED2i) {
            RCLCPP_WARN(
                get_logger(),
                "Camera model does not match user parameter. Please modify "
                "the value of the parameter 'general.camera_model' to 'zed2i'");
        }
    }
#endif

    RCLCPP_INFO_STREAM(get_logger(),
        " * Camera Model\t-> "
            << sl::toString(mCamRealModel).c_str());
    mCamSerialNumber = camInfo.serial_number;
    RCLCPP_INFO_STREAM(get_logger(), " * Serial Number\t-> " << mCamSerialNumber);

    RCLCPP_INFO_STREAM(
        get_logger(),
        " * Input type\t-> "
            << sl::toString(mZed.getCameraInformation().input_type).c_str());
    if (mSvoMode) {
        RCLCPP_INFO(
            get_logger(),
            " * SVO resolution\t-> %ldx%ld",
            mZed.getCameraInformation().camera_configuration.resolution.width,
            mZed.getCameraInformation().camera_configuration.resolution.height);
        RCLCPP_INFO_STREAM(
            get_logger(),
            " * SVO framerate\t-> "
                << (mZed.getCameraInformation().camera_configuration.fps));
    }

    // Firmwares
    if (!mSvoMode) {
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
        mCamFwVersion = camInfo.camera_firmware_version;
#else
        mCamFwVersion = camInfo.camera_configuration.firmware_version;
#endif

        RCLCPP_INFO_STREAM(get_logger(),
            " * Camera FW Version -> " << mCamFwVersion);
        if (mCamRealModel != sl::MODEL::ZED) {
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
            mSensFwVersion = camInfo.sensors_firmware_version;
#else
            mSensFwVersion = camInfo.sensors_configuration.firmware_version;
#endif
            RCLCPP_INFO_STREAM(get_logger(),
                " * Sensors FW Version -> " << mSensFwVersion);
        }
    }

    // Camera/IMU transform
    if (mCamRealModel != sl::MODEL::ZED) {
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
        mSlCamImuTransf = camInfo.camera_imu_transform;
#else
        mSlCamImuTransf = camInfo.sensors_configuration.camera_imu_transform;
#endif

        RCLCPP_DEBUG(get_logger(),
            "Camera-IMU Transform: \n %s",
            mSlCamImuTransf.getInfos().c_str());
    }

    mCamWidth = camInfo.camera_configuration.resolution.width;
    mCamHeight = camInfo.camera_configuration.resolution.height;

    RCLCPP_DEBUG_STREAM(get_logger(),
        "Camera Frame size : " << mCamWidth << "x" << mCamHeight);
    int v_w = static_cast<int>(mCamWidth * mImgDownsampleFactor);
    int v_h = static_cast<int>(mCamHeight * mImgDownsampleFactor);
    mMatResolVideo = sl::Resolution(v_w, v_h);
    RCLCPP_DEBUG_STREAM(get_logger(),
        "Image Mat size : " << mMatResolVideo.width << "x"
                            << mMatResolVideo.height);
    int d_w = static_cast<int>(mCamWidth * mDepthDownsampleFactor);
    int d_h = static_cast<int>(mCamHeight * mDepthDownsampleFactor);
    mMatResolDepth = sl::Resolution(d_w, d_h);
    RCLCPP_DEBUG_STREAM(get_logger(),
        "Depth Mat size : " << mMatResolDepth.width << "x"
                            << mMatResolDepth.height);
    // <---- Camera information

    // ----> Camera Info messages
    mRgbCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    mRgbCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    mLeftCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    mLeftCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    mRightCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    mRightCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    mDepthCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();

    fillCamInfo(mZed,
        mLeftCamInfoMsg,
        mRightCamInfoMsg,
        mLeftCamOptFrameId,
        mRightCamOptFrameId);
    fillCamInfo(mZed,
        mLeftCamInfoRawMsg,
        mRightCamInfoRawMsg,
        mLeftCamOptFrameId,
        mRightCamOptFrameId,
        true);
    mRgbCamInfoMsg = mLeftCamInfoMsg;
    mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
    mDepthCamInfoMsg = mLeftCamInfoMsg;
    // <---- Camera Info messages

    setTFCoordFrameNames(); // Requires mZedRealCamModel available only after
        // camera opening
    initPublishers(); // Requires mZedRealCamModel available only after camera
        // opening

    // Disable AEC_AGC and Auto Whitebalance to trigger it if user set it to
    // automatic
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0);
    // Force parameters with a dummy grab
    mZed.grab();

    // Initialialized timestamp to avoid wrong initial data
    // ----> Timestamp
    if (mSvoMode) {
        mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::CURRENT));
    } else {
        mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::IMAGE));
    }
    // <---- Timestamp

    // ----> Initialize Diagnostic statistics
    mElabPeriodMean_sec = std::make_unique<sl_tools::SmartMean>(mCamGrabFrameRate);
    mGrabPeriodMean_sec = std::make_unique<sl_tools::SmartMean>(mCamGrabFrameRate);
    mVideoDepthPeriodMean_sec = std::make_unique<sl_tools::SmartMean>(mCamGrabFrameRate);
    mVideoDepthElabMean_sec = std::make_unique<sl_tools::SmartMean>(mCamGrabFrameRate);
    mPcPeriodMean_sec = std::make_unique<sl_tools::SmartMean>(mCamGrabFrameRate);
    mPcProcMean_sec = std::make_unique<sl_tools::SmartMean>(mCamGrabFrameRate);
    mObjDetPeriodMean_sec = std::make_unique<sl_tools::SmartMean>(mCamGrabFrameRate);
    mObjDetElabMean_sec = std::make_unique<sl_tools::SmartMean>(mCamGrabFrameRate);
    mImuPeriodMean_sec = std::make_unique<sl_tools::SmartMean>(mSensPubRate);
    mBaroPeriodMean_sec = std::make_unique<sl_tools::SmartMean>(mSensPubRate);
    mMagPeriodMean_sec = std::make_unique<sl_tools::SmartMean>(mSensPubRate);
    mPubFusedCloudPeriodMean_sec = std::make_unique<sl_tools::SmartMean>(10);
    // <---- Initialize Diagnostic statistics

    // ----> Start Pointcloud thread
    if (!mDepthDisabled) {
        mPcDataReady = false;
        // RCLCPP_DEBUG(get_logger(), "on_activate -> mPcDataReady FALSE")
        mPcThread = std::thread(&ZedCamera::threadFunc_pointcloudElab, this);
    }
    // <---- Start Pointcloud thread

    // Start pool thread
    mGrabThread = std::thread(&ZedCamera::threadFunc_zedGrab, this);

    // Start data publishing timer
    startVideoDepthTimer(mPubFrameRate);

    if (!mSvoMode && mCamRealModel != sl::MODEL::ZED) {
        if (mSensPubRate == 400.) {
            mSensPubRate *= TIMER_TIME_FACTOR;
        }

        std::chrono::milliseconds sensorsPubPeriod_msec(
            static_cast<int>(1000.0 / (mSensPubRate)));
        mSensTimer = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
                                           sensorsPubPeriod_msec),
            std::bind(&ZedCamera::callback_pubSensorsData, this));
    }

    return true;
}

void ZedCamera::startVideoDepthTimer(double pubFrameRate)
{
    if (mVideoDepthTimer != nullptr) {
        mVideoDepthTimer->cancel();
    }

    std::chrono::milliseconds videoDepthPubPeriod_msec(
        static_cast<int>(1000.0 / (pubFrameRate)));
    mVideoDepthTimer = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
                                             videoDepthPubPeriod_msec),
        std::bind(&ZedCamera::callback_pubVideoDepth, this));
}

void ZedCamera::startFusedPcTimer(double fusedPcRate)
{
    if (mFusedPcTimer != nullptr) {
        mFusedPcTimer->cancel();
    }

    std::chrono::milliseconds pubPeriod_msec(
        static_cast<int>(1000.0 / (fusedPcRate)));
    mFusedPcTimer = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(pubPeriod_msec),
        std::bind(&ZedCamera::callback_pubFusedPc, this));
}

void ZedCamera::startPathPubTimer(double pathTimerRate)
{
    if (mPathTimer != nullptr) {
        mPathTimer->cancel();
    }

    if (pathTimerRate > 0) {
        std::chrono::milliseconds pubPeriod_msec(
            static_cast<int>(1000.0 / (pathTimerRate)));
        mPathTimer = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(pubPeriod_msec),
            std::bind(&ZedCamera::callback_pubPaths, this));

        if (mOdomPath.size() == 0 && mMapPath.size() == 0) {
            if (mPathMaxCount != -1) {
                RCLCPP_DEBUG_STREAM(
                    get_logger(), "Path vectors reserved " << mPathMaxCount << " poses.");
                mOdomPath.reserve(mPathMaxCount);
                mMapPath.reserve(mPathMaxCount);

                RCLCPP_DEBUG_STREAM(get_logger(),
                    "Path vector sizes: " << mOdomPath.size() << " "
                                          << mMapPath.size());
            }
        }
    } else {
        mOdomPath.clear();
        mMapPath.clear();
        mPathTimer->cancel();
        RCLCPP_INFO_STREAM(
            get_logger(),
            "Path topics not published -> Pub. rate: " << pathTimerRate << " Hz");
    }
}

bool ZedCamera::startPosTracking()
{
    if (mDepthDisabled) {
        RCLCPP_WARN(get_logger(),
            "Cannot start Positional Tracking if "
            "`depth.quality` is set to `0` [NONE]");
        return false;
    }

    RCLCPP_INFO_STREAM(get_logger(), "*** Starting Positional Tracking ***");

    RCLCPP_INFO(get_logger(), " * Waiting for valid static transformations...");

    bool transformOk = false;
    double elapsed = 0.0;
    mPosTrackingReady = false;

    auto start = std::chrono::high_resolution_clock::now();

    do {
        transformOk = setPose(mInitialBasePose[0],
            mInitialBasePose[1],
            mInitialBasePose[2],
            mInitialBasePose[3],
            mInitialBasePose[4],
            mInitialBasePose[5]);

        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start)
                      .count();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (elapsed > 10000) {
            RCLCPP_WARN(get_logger(),
                " !!! Failed to get static transforms. Is the "
                "'ROBOT STATE PUBLISHER' node correctly "
                "working? ");
            break;
        }

    } while (transformOk == false);

    if (transformOk) {
        RCLCPP_DEBUG(get_logger(),
            "Time required to get valid static transforms: %g sec",
            elapsed / 1000.);
    }

    RCLCPP_INFO(get_logger(),
        "Initial ZED left camera pose (ZED pos. tracking): ");
    RCLCPP_INFO(get_logger(),
        " * T: [%g,%g,%g]",
        mInitialPoseSl.getTranslation().x,
        mInitialPoseSl.getTranslation().y,
        mInitialPoseSl.getTranslation().z);
    RCLCPP_INFO(get_logger(),
        " * Q: [%g,%g,%g,%g]",
        mInitialPoseSl.getOrientation().ox,
        mInitialPoseSl.getOrientation().oy,
        mInitialPoseSl.getOrientation().oz,
        mInitialPoseSl.getOrientation().ow);

    if (mAreaMemoryDbPath != "" && !sl_tools::file_exist(mAreaMemoryDbPath)) {
        mAreaMemoryDbPath = "";
        RCLCPP_WARN_STREAM(
            get_logger(),
            "'area_memory_db_path' path doesn't exist or is unreachable: "
                << mAreaMemoryDbPath);
    }

    // Tracking parameters
    sl::PositionalTrackingParameters trackParams;

    trackParams.area_file_path = mAreaMemoryDbPath.c_str();

    mPoseSmoothing = false; // Always false. Pose Smoothing is to be enabled only
        // for VR/AR applications
    trackParams.enable_pose_smoothing = mPoseSmoothing;

    trackParams.enable_area_memory = mAreaMemory;
    trackParams.enable_imu_fusion = mImuFusion;
    trackParams.initial_world_transform = mInitialPoseSl;

    trackParams.set_floor_as_origin = mFloorAlignment;

    sl::ERROR_CODE err = mZed.enablePositionalTracking(trackParams);

    if (err == sl::ERROR_CODE::SUCCESS) {
        mPosTrackingStarted = true;
    } else {
        mPosTrackingStarted = false;

        RCLCPP_WARN(
            get_logger(), "Tracking not started: %s", sl::toString(err).c_str());
    }

    if (mPosTrackingStarted) {
        startPathPubTimer(mPathPubRate);
    }

    return mPosTrackingStarted;
}

bool ZedCamera::start3dMapping()
{
    if (mDepthDisabled) {
        RCLCPP_WARN(
            get_logger(),
            "Cannot start 3D Mapping if `depth.quality` is set to `0` [NONE]");
        return false;
    }
    if (!mMappingEnabled) {
        return false;
    }

    RCLCPP_INFO_STREAM(get_logger(), "*** Starting Spatial Mapping ***");

    sl::SpatialMappingParameters params;
    params.map_type = sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
    params.use_chunk_only = true;

    sl::SpatialMappingParameters spMapPar;

    float lRes = spMapPar.allowed_resolution.first;
    float hRes = spMapPar.allowed_resolution.second;

    if (mMappingRes < lRes) {
        RCLCPP_WARN_STREAM(get_logger(),
            "'mapping.resolution' value ("
                << mMappingRes
                << " m) is lower than the allowed resolution "
                   "values. Fixed automatically");
        mMappingRes = lRes;
    }
    if (mMappingRes > hRes) {
        RCLCPP_WARN_STREAM(get_logger(),
            "'mapping.resolution' value ("
                << mMappingRes
                << " m) is higher than the allowed resolution "
                   "values. Fixed automatically");
        mMappingRes = hRes;
    }

    params.resolution_meter = mMappingRes;

    float lRng = spMapPar.allowed_range.first;
    float hRng = spMapPar.allowed_range.second;

    if (mMappingRangeMax < 0) {
        mMappingRangeMax = sl::SpatialMappingParameters::getRecommendedRange(mMappingRes, mZed);
        RCLCPP_INFO_STREAM(get_logger(),
            "Mapping: max range set to " << mMappingRangeMax
                                         << " m for a resolution of "
                                         << mMappingRes << " m");
    } else if (mMappingRangeMax < lRng) {
        RCLCPP_WARN_STREAM(get_logger(),
            "'mapping.max_mapping_range_m' value ("
                << mMappingRangeMax
                << " m) is lower than the allowed "
                   "resolution values. Fixed "
                   "automatically");
        mMappingRangeMax = lRng;
    } else if (mMappingRangeMax > hRng) {
        RCLCPP_WARN_STREAM(get_logger(),
            "'mapping.max_mapping_range_m' value ("
                << mMappingRangeMax
                << " m) is higher than the allowed "
                   "resolution values. Fixed "
                   "automatically");
        mMappingRangeMax = hRng;
    }

    params.range_meter = mMappingRangeMax;

    sl::ERROR_CODE err = mZed.enableSpatialMapping(params);

    if (err == sl::ERROR_CODE::SUCCESS) {
        if (mPubFusedCloud == nullptr) {
            ;
            mPubFusedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
                mPointcloudFusedTopic, mMappingQos);
            RCLCPP_INFO_STREAM(get_logger(),
                "Advertised on topic "
                    << mPubFusedCloud->get_topic_name() << " @ "
                    << mFusedPcPubRate << " Hz");
        }

        mMappingRunning = true;

        startFusedPcTimer(mFusedPcPubRate);

        RCLCPP_INFO_STREAM(get_logger(),
            " * Resolution: " << params.resolution_meter << " m");
        RCLCPP_INFO_STREAM(get_logger(),
            " * Max Mapping Range: " << params.range_meter << " m");
        RCLCPP_INFO_STREAM(get_logger(),
            " * Map point cloud publishing rate: " << mFusedPcPubRate
                                                   << " Hz");

        return true;
    } else {
        mMappingRunning = false;
        if (mFusedPcTimer) {
            mFusedPcTimer->cancel();
        }

        RCLCPP_WARN(
            get_logger(), "Mapping not activated: %s", sl::toString(err).c_str());

        return false;
    }
}

void ZedCamera::stop3dMapping()
{
    if (mFusedPcTimer) {
        mFusedPcTimer->cancel();
    }
    mMappingRunning = false;
    mMappingEnabled = false;
    mZed.disableSpatialMapping();

    RCLCPP_INFO(get_logger(), "*** Spatial Mapping stopped ***");
}

bool ZedCamera::startObjDetect()
{
    if (!sl_tools::isObjDetAvailable(mCamRealModel)) {
        RCLCPP_ERROR(get_logger(),
            "Object detection not started. The camera model does not support it with the current version of the SDK");
        return false;
    }

    if (mDepthDisabled) {
        RCLCPP_WARN(get_logger(),
            "Cannot start Object Detection if "
            "`depth.quality` is set to `0` [NONE]");
        return false;
    }

    if (!mObjDetEnabled) {
        return false;
    }

    if (!mCamera2BaseTransfValid || !mSensor2CameraTransfValid || !mSensor2BaseTransfValid) {
        RCLCPP_DEBUG(get_logger(),
            "Tracking transforms not yet ready, OD starting postponed");
        return false;
    }

    RCLCPP_INFO(get_logger(), "*** Starting Object Detection ***");

    sl::ObjectDetectionParameters od_p;
    od_p.enable_mask_output = false;
    od_p.enable_tracking = mObjDetTracking;
    od_p.image_sync = true;
    od_p.detection_model = mObjDetModel;
    od_p.filtering_mode = mObjFilterMode;
    od_p.enable_body_fitting = mObjDetBodyFitting;
    od_p.body_format = mObjDetBodyFmt;

    mObjDetFilter.clear();
    if (mObjDetPeopleEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::PERSON);
    }
    if (mObjDetVehiclesEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::VEHICLE);
    }
    if (mObjDetBagsEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::BAG);
    }
    if (mObjDetAnimalsEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::ANIMAL);
    }
    if (mObjDetElectronicsEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::ELECTRONICS);
    }
    if (mObjDetFruitsEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::FRUIT_VEGETABLE);
    }
    if (mObjDetSportEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::SPORT);
    }

    sl::ERROR_CODE objDetError
        = mZed.enableObjectDetection(od_p);

    if (objDetError != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
            get_logger(), "Object detection error: " << sl::toString(objDetError));

        mObjDetRunning = false;
        return false;
    }

    if (!mPubObjDet) {
        mPubObjDet = create_publisher<zed_interfaces::msg::ObjectsStamped>(
            mObjectDetTopic, mObjDetQos);
        RCLCPP_INFO_STREAM(get_logger(),
            "Advertised on topic " << mPubObjDet->get_topic_name());
    }

    mObjDetRunning = true;
    return true;
}

void ZedCamera::stopObjDetect()
{
    if (mObjDetRunning) {
        RCLCPP_INFO(get_logger(), "*** Stopping Object Detection ***");
        mObjDetRunning = false;
        mObjDetEnabled = false;
        mZed.disableObjectDetection();

        // ----> Send an empty message to indicate that no more objects are tracked
        // (e.g clean Rviz2)
        objDetMsgPtr objMsg = std::make_unique<zed_interfaces::msg::ObjectsStamped>();

        objMsg->header.stamp = mFrameTimestamp;
        objMsg->header.frame_id = mLeftCamFrameId;

        objMsg->objects.clear();

        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing EMPTY OBJ message " << mPubObjDet->get_topic_name()  );
        mPubObjDet->publish(std::move(objMsg));
        // <---- Send an empty message to indicate that no more objects are tracked
        // (e.g clean Rviz2)
    }
}

bool ZedCamera::startSvoRecording(std::string& errMsg)
{
    sl::RecordingParameters params;

    params.bitrate = mSvoRecBitrate;
    params.compression_mode = mSvoRecCompr;
    params.target_framerate = mSvoRecFramerate;
    params.transcode_streaming_input = mSvoRecTranscode;
    params.video_filename = mSvoRecFilename.c_str();

    sl::ERROR_CODE err = mZed.enableRecording(params);
    errMsg = sl::toString(err);

    if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(get_logger(),
            "Error starting SVO recording: " << errMsg);
        return false;
    }

    mRecording = true;

    return true;
}

void ZedCamera::stopSvoRecording()
{
    if (mRecording) {
        mRecording = false;
        mZed.disableRecording();
    }
}

void ZedCamera::initTransforms()
{
    // According to REP 105 -> http://www.ros.org/reps/rep-0105.html

    // base_link <- odom <- map
    //     ^                 |
    //     |                 |
    //     -------------------

    // ----> Dynamic transforms
    mOdom2BaseTransf.setIdentity(); // broadcasted if `publish_tf` is true
    mMap2OdomTransf.setIdentity(); // broadcasted if `publish_map_tf` is true
    mMap2BaseTransf.setIdentity(); // used internally, but not broadcasted
    // <---- Dynamic transforms
}

bool ZedCamera::getCamera2BaseTransform()
{
    RCLCPP_DEBUG(get_logger(),
        "Getting static TF from '%s' to '%s'",
        mCameraFrameId.c_str(),
        mBaseFrameId.c_str());

    mCamera2BaseTransfValid = false;
    static bool first_error = true;

    // ----> Static transforms
    // Sensor to Base link
    try {
        // Save the transformation
        geometry_msgs::msg::TransformStamped c2b = mTfBuffer->lookupTransform(
            mCameraFrameId, mBaseFrameId, TIMEZERO_SYS, rclcpp::Duration(0.1));

        // Get the TF2 transformation
        // tf2::fromMsg(c2b.transform, mCamera2BaseTransf);
        geometry_msgs::msg::Transform in = c2b.transform;
        mCamera2BaseTransf.setOrigin(
            tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
        // w at the end in the constructor
        mCamera2BaseTransf.setRotation(tf2::Quaternion(
            in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));

        double roll, pitch, yaw;
        tf2::Matrix3x3(mCamera2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(get_logger(),
            "Static transform Camera Center to Base [%s -> %s]",
            mCameraFrameId.c_str(),
            mBaseFrameId.c_str());
        RCLCPP_INFO(get_logger(),
            " * Translation: {%.3f,%.3f,%.3f}",
            mCamera2BaseTransf.getOrigin().x(),
            mCamera2BaseTransf.getOrigin().y(),
            mCamera2BaseTransf.getOrigin().z());
        RCLCPP_INFO(get_logger(),
            " * Rotation: {%.3f,%.3f,%.3f}",
            roll * RAD2DEG,
            pitch * RAD2DEG,
            yaw * RAD2DEG);
    } catch (tf2::TransformException& ex) {
        if (!first_error) {
            rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_DEBUG_THROTTLE(
                get_logger(), steady_clock, 1.0, "Transform error: %s", ex.what());
            RCLCPP_WARN_THROTTLE(get_logger(),
                steady_clock,
                1.0,
                "The tf from '%s' to '%s' is not available.",
                mCameraFrameId.c_str(),
                mBaseFrameId.c_str());
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                steady_clock,
                1.0,
                "Note: one of the possible cause of the problem is the absense of an "
                "instance "
                "of the `robot_state_publisher` node publishing the correct static "
                "TF transformations "
                "or a modified URDF not correctly reproducing the ZED "
                "TF chain '%s' -> '%s' -> '%s'",
                mBaseFrameId.c_str(),
                mCameraFrameId.c_str(),
                mDepthFrameId.c_str());
            first_error = false;
        }

        mCamera2BaseTransf.setIdentity();
        return false;
    }

    // <---- Static transforms
    mCamera2BaseTransfValid = true;
    return true;
}

bool ZedCamera::getSens2CameraTransform()
{
    RCLCPP_DEBUG(get_logger(),
        "Getting static TF from '%s' to '%s'",
        mDepthFrameId.c_str(),
        mCameraFrameId.c_str());

    mSensor2CameraTransfValid = false;

    static bool first_error = true;

    // ----> Static transforms
    // Sensor to Camera Center
    try {
        // Save the transformation
        geometry_msgs::msg::TransformStamped s2c = mTfBuffer->lookupTransform(
            mDepthFrameId, mCameraFrameId, TIMEZERO_SYS, rclcpp::Duration(0.1));

        // Get the TF2 transformation
        // tf2::fromMsg(s2c.transform, mSensor2CameraTransf);
        geometry_msgs::msg::Transform in = s2c.transform;
        mSensor2CameraTransf.setOrigin(
            tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
        // w at the end in the constructor
        mSensor2CameraTransf.setRotation(tf2::Quaternion(
            in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));

        double roll, pitch, yaw;
        tf2::Matrix3x3(mSensor2CameraTransf.getRotation()).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(get_logger(),
            "Static transform Sensor to Camera Center [%s -> %s]",
            mDepthFrameId.c_str(),
            mCameraFrameId.c_str());
        RCLCPP_INFO(get_logger(),
            " * Translation: {%.3f,%.3f,%.3f}",
            mSensor2CameraTransf.getOrigin().x(),
            mSensor2CameraTransf.getOrigin().y(),
            mSensor2CameraTransf.getOrigin().z());
        RCLCPP_INFO(get_logger(),
            " * Rotation: {%.3f,%.3f,%.3f}",
            roll * RAD2DEG,
            pitch * RAD2DEG,
            yaw * RAD2DEG);
    } catch (tf2::TransformException& ex) {
        if (!first_error) {
            rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_DEBUG_THROTTLE(
                get_logger(), steady_clock, 1.0, "Transform error: %s", ex.what());
            RCLCPP_WARN_THROTTLE(get_logger(),
                steady_clock,
                1.0,
                "The tf from '%s' to '%s' is not available.",
                mDepthFrameId.c_str(),
                mCameraFrameId.c_str());
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                steady_clock,
                1.0,
                "Note: one of the possible cause of the problem is the absense of an "
                "instance "
                "of the `robot_state_publisher` node publishing the correct static "
                "TF transformations "
                "or a modified URDF not correctly reproducing the ZED "
                "TF chain '%s' -> '%s' -> '%s'",
                mBaseFrameId.c_str(),
                mCameraFrameId.c_str(),
                mDepthFrameId.c_str());
            first_error = false;
        }

        mSensor2CameraTransf.setIdentity();
        return false;
    }

    // <---- Static transforms

    mSensor2CameraTransfValid = true;
    return true;
}

bool ZedCamera::getSens2BaseTransform()
{
    RCLCPP_DEBUG(get_logger(),
        "Getting static TF from '%s' to '%s'",
        mDepthFrameId.c_str(),
        mBaseFrameId.c_str());

    mSensor2BaseTransfValid = false;
    static bool first_error = true;

    // ----> Static transforms
    // Sensor to Base link
    try {
        // Save the transformation
        geometry_msgs::msg::TransformStamped s2b = mTfBuffer->lookupTransform(
            mDepthFrameId, mBaseFrameId, TIMEZERO_SYS, rclcpp::Duration(0.1));

        // Get the TF2 transformation
        // tf2::fromMsg(s2b.transform, mSensor2BaseTransf);
        geometry_msgs::msg::Transform in = s2b.transform;
        mSensor2BaseTransf.setOrigin(
            tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
        // w at the end in the constructor
        mSensor2BaseTransf.setRotation(tf2::Quaternion(
            in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));

        double roll, pitch, yaw;
        tf2::Matrix3x3(mSensor2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(get_logger(),
            "Static transform Sensor to Base [%s -> %s]",
            mDepthFrameId.c_str(),
            mBaseFrameId.c_str());
        RCLCPP_INFO(get_logger(),
            " * Translation: {%.3f,%.3f,%.3f}",
            mSensor2BaseTransf.getOrigin().x(),
            mSensor2BaseTransf.getOrigin().y(),
            mSensor2BaseTransf.getOrigin().z());
        RCLCPP_INFO(get_logger(),
            " * Rotation: {%.3f,%.3f,%.3f}",
            roll * RAD2DEG,
            pitch * RAD2DEG,
            yaw * RAD2DEG);
    } catch (tf2::TransformException& ex) {
        if (!first_error) {
            rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_DEBUG_THROTTLE(
                get_logger(), steady_clock, 1.0, "Transform error: %s", ex.what());
            RCLCPP_WARN_THROTTLE(get_logger(),
                steady_clock,
                1.0,
                "The tf from '%s' to '%s' is not available.",
                mDepthFrameId.c_str(),
                mBaseFrameId.c_str());
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                steady_clock,
                1.0,
                "Note: one of the possible cause of the problem is the absense of an "
                "instance "
                "of the `robot_state_publisher` node publishing the correct static "
                "TF transformations "
                "or a modified URDF not correctly reproducing the ZED "
                "TF chain '%s' -> '%s' -> '%s'",
                mBaseFrameId.c_str(),
                mCameraFrameId.c_str(),
                mDepthFrameId.c_str());
            first_error = false;
        }

        mSensor2BaseTransf.setIdentity();
        return false;
    }

    // <---- Static transforms

    mSensor2BaseTransfValid = true;
    return true;
}

bool ZedCamera::setPose(float xt, float yt, float zt, float rr, float pr, float yr)
{
    initTransforms();

    if (!mSensor2BaseTransfValid) {
        getSens2BaseTransform();
    }

    if (!mSensor2CameraTransfValid) {
        getSens2CameraTransform();
    }

    if (!mCamera2BaseTransfValid) {
        getCamera2BaseTransform();
    }

    // Apply Base to sensor transform
    tf2::Transform initPose;
    tf2::Vector3 origin(xt, yt, zt);
    initPose.setOrigin(origin);
    tf2::Quaternion quat;
    quat.setRPY(rr, pr, yr);
    initPose.setRotation(quat);

    initPose = initPose * mSensor2BaseTransf.inverse(); // TODO Check this transformation. Rotation seems wrong

    // SL pose
    sl::float3 t_vec;
    t_vec[0] = initPose.getOrigin().x();
    t_vec[1] = initPose.getOrigin().y();
    t_vec[2] = initPose.getOrigin().z();

    sl::float4 q_vec;
    q_vec[0] = initPose.getRotation().x();
    q_vec[1] = initPose.getRotation().y();
    q_vec[2] = initPose.getRotation().z();
    q_vec[3] = initPose.getRotation().w();

    sl::Translation trasl(t_vec);
    sl::Orientation orient(q_vec);
    mInitialPoseSl.setTranslation(trasl);
    mInitialPoseSl.setOrientation(orient);

    return (mSensor2BaseTransfValid & mSensor2CameraTransfValid & mCamera2BaseTransfValid);
}

void ZedCamera::publishStaticImuFrameAndTopic()
{
    sl::Orientation sl_rot = mSlCamImuTransf.getOrientation();
    sl::Translation sl_tr = mSlCamImuTransf.getTranslation();

    mCameraImuTransfMgs = std::make_shared<geometry_msgs::msg::TransformStamped>();

    mCameraImuTransfMgs->header.stamp = get_clock()->now();

    mCameraImuTransfMgs->header.frame_id = mLeftCamFrameId;
    mCameraImuTransfMgs->child_frame_id = mImuFrameId;

    mCameraImuTransfMgs->transform.rotation.x = sl_rot.ox;
    mCameraImuTransfMgs->transform.rotation.y = sl_rot.oy;
    mCameraImuTransfMgs->transform.rotation.z = sl_rot.oz;
    mCameraImuTransfMgs->transform.rotation.w = sl_rot.ow;

    mCameraImuTransfMgs->transform.translation.x = sl_tr.x;
    mCameraImuTransfMgs->transform.translation.y = sl_tr.y;
    mCameraImuTransfMgs->transform.translation.z = sl_tr.z;

    if (!mStaticImuTopicPublished) {
        rclcpp::QoS transf_qos = mSensQos;
        transf_qos.durability(
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); // Latched topic
        transf_qos.keep_last(1);
        std::string cam_imu_tr_topic = mTopicRoot + "left_cam_imu_transform";
        mPubCamImuTransf = create_publisher<geometry_msgs::msg::TransformStamped>(
            cam_imu_tr_topic, transf_qos);

        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing Camera-IMU transform " << mPubCamImuTransf->get_topic_name());
        mPubCamImuTransf->publish(*(mCameraImuTransfMgs.get()));

        RCLCPP_INFO_STREAM(get_logger(),
            "Advertised on topic: "
                << mPubCamImuTransf->get_topic_name() << " [LATCHED]");
        RCLCPP_INFO(get_logger(),
            "Camera-IMU Translation: \n %g %g %g",
            sl_tr.x,
            sl_tr.y,
            sl_tr.z);
        RCLCPP_INFO(get_logger(),
            "Camera-IMU Rotation: \n %s",
            sl_rot.getRotationMatrix().getInfos().c_str());

        mStaticImuTopicPublished = true;
    }

    // Publish IMU TF as static TF
    if (!mPublishImuTF) {
        return;
    }

    if (mStaticImuFramePublished) {
        return;
    }


    // Publish transformation
    mStaticTfBroadcaster->sendTransform(*(mCameraImuTransfMgs.get()));

    RCLCPP_INFO_STREAM(get_logger(),
        "Published static TF: '" << mImuFrameId << "' -> '"
                                 << mLeftCamFrameId << "'");

    mStaticImuFramePublished = true;
}

void ZedCamera::threadFunc_zedGrab()
{
    RCLCPP_DEBUG(get_logger(), "Grab thread started");

    mFrameCount = 0;
    mRgbDepthDataRetrieved = true; // Force first grab

    // ----> Grab Runtime parameters
    mRunParams.sensing_mode = static_cast<sl::SENSING_MODE>(mDepthSensingMode);
    mRunParams.enable_depth = false;
    mRunParams.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
    mRunParams.remove_saturated_areas = mRemoveSatAreas;
    // <---- Grab Runtime parameters

    // Infinite grab thread
    while (1) {
        // ----> Interruption check
        if (!rclcpp::ok()) {
            RCLCPP_DEBUG(get_logger(), "Ctrl+C received: stopping grab thread");
            break;
        }

        if (mThreadStop) {
            RCLCPP_DEBUG(get_logger(), "Grab thread stopped");
            break;
        }
        // <---- Interruption check

        // Wait for operations on Positional Tracking
        std::lock_guard<std::mutex> lock(mPosTrkMutex);

        // ----> Apply depth settings
        applyDepthSettings();
        // <---- Apply depth settings

        // ----> Apply video dynamic parameters
        applyVideoSettings();
        // <---- Apply video dynamic parameters

        // ----> Check for Positional Tracking requirement
        if (isPosTrackingRequired() && !mPosTrackingStarted) {
            startPosTracking();
        }
        // ----> Check for Positional Tracking requirement

        // ----> Check for Spatial Mapping requirement
        if (!mDepthDisabled) {
            mMappingMutex.lock();
            if (mMappingEnabled && !mMappingRunning) {
                start3dMapping();
            }
            mMappingMutex.unlock();
        }
        // <---- Check for Spatial Mapping requirement

        // ----> Check for Object Detection requirement
        if (!mDepthDisabled) {
            mObjDetMutex.lock();
            if (mObjDetEnabled && !mObjDetRunning) {
                startObjDetect();
                if (!sl_tools::isObjDetAvailable(mCamRealModel)) {
                    mObjDetEnabled = false;
                }
            }
            mObjDetMutex.unlock();
        }
        // ----> Check for Object Detection requirement

        // ----> Wait for RGB/Depth synchronization before grabbing
        std::unique_lock<std::timed_mutex> datalock(mCamDataMutex);
        while (!mRgbDepthDataRetrieved) { // loop to avoid spurious wakeups
            if (mRgbDepthDataRetrievedCondVar.wait_for(
                    datalock, std::chrono::milliseconds(500))
                == std::cv_status::timeout) {
                // Check thread stopping
                if (mThreadStop) {
                    return;
                } else {
                    continue;
                }
            }
        }
        mRgbDepthDataRetrieved = false;
        // <---- Wait for RGB/Depth synchronization before grabbing

        // ----> Grab freq calculation
        static sl_tools::StopWatch freqTimer;

        double elapsed_sec = freqTimer.toc();
        mGrabPeriodMean_sec->addValue(elapsed_sec);
        freqTimer.tic();

        //        RCLCPP_INFO_STREAM( get_logger(), "Grab period: " <<
        //        mGrabPeriodMean_sec->getMean()/1e6 <<
        //                            " Freq: " <<
        //                            1e6/mGrabPeriodMean_usec->getMean() );
        // <---- Grab freq calculation

        if (mSvoMode && !mSvoRealtime) {
            static bool first = true;

            double grab_period_usec = (1. / mPubFrameRate) * 1e6;
            int64_t residual_period_usec = grab_period_usec - (elapsed_sec * 1e6);

            if (residual_period_usec > 0) {
                if (first) {
                    first = false;
                } else {
                    // RCLCPP_DEBUG_STREAM(get_logger(), "Sleeping for " <<
                    // residual_period_usec << "usec" );
                    std::this_thread::sleep_for(
                        std::chrono::microseconds(residual_period_usec));
                }
            }
        }

        sl_tools::StopWatch elabTimer;

        if (!mSvoPause) {
            // ZED grab
            mGrabStatus = mZed.grab(mRunParams);

            // ----> Check SVO status
            if (mSvoMode && mGrabStatus == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
                if (mSvoLoop) {
                    mZed.setSVOPosition(0);
                    RCLCPP_WARN(get_logger(),
                        "SVO reached the end and has been restarted.");
                    std::this_thread::sleep_for(std::chrono::milliseconds(
                        static_cast<long int>(mGrabPeriodMean_sec->getMean() * 1000.f)));
                    continue;
                } else {
                    RCLCPP_WARN(get_logger(),
                        "SVO reached the end. The node has been stopped.");
                    break;
                }
            }
            // <---- Check SVO status

            // ----> Grab errors?
            // Note: disconnection are automatically handled by the ZED SDK
            if (mGrabStatus != sl::ERROR_CODE::SUCCESS) {
                RCLCPP_ERROR_STREAM(
                    get_logger(), "Camera error: " << sl::toString(mGrabStatus).c_str());
                break; // TODO verify what to do in case of grab errors
            }
            // <---- Grab errors?

            mFrameCount++;

            // ----> Timestamp
            if (mSvoMode) {
                mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::CURRENT));
            } else {
                mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::IMAGE));
            }
            // <---- Timestamp

            // ----> Check recording status
            mRecMutex.lock();
            if (mRecording) {
                mRecStatus = mZed.getRecordingStatus();

                if (!mRecStatus.status) {
                    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), steady_clock, 1.0, "Error saving frame to SVO");
                }

                // TODO update disgnostic
                // mDiagUpdater.force_update();
            }
            mRecMutex.unlock();
            // <---- Check recording status
        }

        if (!mDepthDisabled) {
            if (mPosTrackingStarted) {
                if (!mSvoPause) {
                    processPose();
                    processOdometry();
                }

                if (mCamRealModel == sl::MODEL::ZED || !mPublishImuTF || mSvoMode) {
                    publishTFs(mFrameTimestamp);
                }
            }
        }

        // Publish the point cloud if someone has subscribed to
        if (!mDepthDisabled) {
            size_t cloudSubnumber = 0;
            try {
                cloudSubnumber = count_subscribers(mPubCloud->get_topic_name());
            } catch (...) {
                rcutils_reset_error();
                RCLCPP_DEBUG(get_logger(),
                    "threadFunc_zedGrab: Exception while "
                    "counting point cloud subscribers");
                return;
            }

            if (cloudSubnumber > 0) {
                // Run the point cloud conversion asynchronously to avoid slowing down
                // all the program
                // Retrieve raw pointCloud data if latest Pointcloud is ready
                std::unique_lock<std::mutex> lock(mPcMutex, std::defer_lock);

                if (lock.try_lock()) {
                    RCLCPP_DEBUG(get_logger(),"Retrieving point cloud");
                    mZed.retrieveMeasure(
                        mMatCloud, sl::MEASURE::XYZBGRA, sl::MEM::CPU, mMatResolDepth);

                    // Signal Pointcloud thread that a new pointcloud is ready
                    mPcDataReadyCondVar.notify_one();
                    mPcDataReady = true;
                    mPcPublishing = true;
                }
            } else {
                mPcPublishing = false;
            }
        }

        if (!mDepthDisabled) {
            mObjDetMutex.lock();
            if (mObjDetRunning) {
                processDetectedObjects(mFrameTimestamp);
            }
            mObjDetMutex.unlock();
        }

        // Diagnostic statistics update
        double mean_elab_sec = mElabPeriodMean_sec->addValue(elabTimer.toc());
    }

    RCLCPP_DEBUG(get_logger(), "Grab thread finished");
}

rclcpp::Time
ZedCamera::publishSensorsData(rclcpp::Time t)
{
    // ----> Subscribers count
    RCLCPP_DEBUG_ONCE(get_logger(), "Sensors callback: counting subscribers");

    size_t imu_SubNumber = 0;
    size_t imu_RawSubNumber = 0;
    size_t imu_TempSubNumber = 0;
    size_t imu_MagSubNumber = 0;
    size_t pressSubNumber = 0;
    size_t tempLeftSubNumber = 0;
    size_t tempRightSubNumber = 0;

    try {
        imu_SubNumber = count_subscribers(mPubImu->get_topic_name());
        imu_RawSubNumber = count_subscribers(mPubImuRaw->get_topic_name());
        imu_TempSubNumber = 0;
        imu_MagSubNumber = 0;
        pressSubNumber = 0;
        tempLeftSubNumber = 0;
        tempRightSubNumber = 0;

        if (sl_tools::isZED2OrZED2i(mCamRealModel)) {
            imu_TempSubNumber = count_subscribers(mPubImuTemp->get_topic_name());
            imu_MagSubNumber = count_subscribers(mPubImuMag->get_topic_name());
            pressSubNumber = count_subscribers(mPubPressure->get_topic_name());
            tempLeftSubNumber = count_subscribers(mPubTempL->get_topic_name());
            tempRightSubNumber = count_subscribers(mPubTempR->get_topic_name());
        }
    } catch (...) {
        rcutils_reset_error();
        RCLCPP_DEBUG(get_logger(),
            "pubSensorsData: Exception while counting subscribers");
        return TIMEZERO_ROS;
    }
    // <---- Subscribers count

    int totSub = imu_SubNumber + imu_RawSubNumber + imu_TempSubNumber + imu_MagSubNumber + pressSubNumber + tempLeftSubNumber + tempRightSubNumber;

    // ----> Grab data and setup timestamps
    RCLCPP_DEBUG_ONCE(get_logger(),
        "Sensors callback: Grab data and setup timestamps");
    rclcpp::Time ts_imu;
    rclcpp::Time ts_baro;
    rclcpp::Time ts_mag;

    rclcpp::Time now = get_clock()->now();

    static rclcpp::Time lastTs_imu = now;
    static rclcpp::Time lastTs_baro = now;
    static rclcpp::Time lastTs_mag = now;

    sl::SensorsData sens_data;

    if (mSvoMode || mSensCameraSync) {
        if (mZed.getSensorsData(sens_data, sl::TIME_REFERENCE::IMAGE) != sl::ERROR_CODE::SUCCESS) {
            return TIMEZERO_ROS;
        }
    } else {
        if (mZed.getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT) != sl::ERROR_CODE::SUCCESS) {
            return TIMEZERO_ROS;
        }
    }

    if (mSensCameraSync) {
        ts_imu = t;
        ts_baro = t;
        ts_mag = t;
    } else {
        ts_imu = sl_tools::slTime2Ros(sens_data.imu.timestamp);
        ts_baro = sl_tools::slTime2Ros(sens_data.barometer.timestamp);
        ts_mag = sl_tools::slTime2Ros(sens_data.magnetometer.timestamp);
    }
    // <---- Grab data and setup timestamps

    // ----> Check for duplicated data
    bool new_imu_data = ts_imu != lastTs_imu;
    double sens_period = 1. / mSensPubRate;
    double dT = ts_imu.seconds() - lastTs_imu.seconds();

    /*if(dT<sens_period) {
      new_imu_data = false;
  }*/

    bool new_baro_data = ts_baro != lastTs_baro;
    lastTs_baro = ts_baro;
    bool new_mag_data = ts_mag != lastTs_mag;
    lastTs_mag = ts_mag;

    if (!new_imu_data && !new_baro_data && !new_mag_data) {
        return TIMEZERO_ROS;
    }
    // <---- Check for duplicated data

    lastTs_imu = ts_imu;

    if(mDebugSensors) 
    {
        RCLCPP_DEBUG_STREAM(
        get_logger(), "SENSOR LAST PERIOD: " << dT << " sec @" << 1. / dT << " Hz");
    }

    RCLCPP_DEBUG_ONCE(get_logger(), "Sensors callback: temperatures");

    if (sl_tools::isZED2OrZED2i(mCamRealModel)) {
        // Update temperatures for Diagnostic
        sens_data.temperature.get(
            sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT,
            mTempLeft);
        sens_data.temperature.get(
            sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_RIGHT,
            mTempRight);
    }

    // ----> Sensors freq for diagnostic
    if (new_imu_data) {
        RCLCPP_DEBUG_ONCE(get_logger(), "Sensors callback: IMU FREQ");         
        static sl_tools::StopWatch freqTimer;

        double mean = mImuPeriodMean_sec->addValue(freqTimer.toc());
        freqTimer.tic();

        if(mDebugSensors)  RCLCPP_DEBUG_STREAM(get_logger(), "IMU MEAN freq: " << 1e6 / mean);
    }

    if (new_baro_data) {
        RCLCPP_DEBUG_ONCE(get_logger(), "Sensors callback: BARO FREQ");  
        static sl_tools::StopWatch freqTimer;

        double mean = mBaroPeriodMean_sec->addValue(freqTimer.toc());
        freqTimer.tic();
        if(mDebugSensors)  RCLCPP_DEBUG_STREAM(get_logger(), "Barometer freq: " << 1e6/mean);
    }

    if (new_mag_data) {
        RCLCPP_DEBUG_ONCE(get_logger(), "Sensors callback: MAG FREQ");
        static sl_tools::StopWatch freqTimer;

        double mean = mMagPeriodMean_sec->addValue(freqTimer.toc());
        freqTimer.tic();

        if(mDebugSensors)  RCLCPP_DEBUG_STREAM(get_logger(), "Magnetometer freq: " << 1e6/mean);
    }
    // <---- Sensors freq for diagnostic

    // if (!totSub < 0)
    // {
    //   return TIMEZERO_ROS;  // Nothing to publish
    // }

    // ----> Sensors data publishing
    if (new_imu_data) {
        if (imu_SubNumber > 0) {
            mImuPublishing = true;

            imuMsgPtr imuMsg = std::make_unique<sensor_msgs::msg::Imu>();

            imuMsg->header.stamp = ts_imu;
            imuMsg->header.frame_id = mImuFrameId;

            imuMsg->orientation.x = sens_data.imu.pose.getOrientation()[0];
            imuMsg->orientation.y = sens_data.imu.pose.getOrientation()[1];
            imuMsg->orientation.z = sens_data.imu.pose.getOrientation()[2];
            imuMsg->orientation.w = sens_data.imu.pose.getOrientation()[3];

            imuMsg->angular_velocity.x = sens_data.imu.angular_velocity[0] * DEG2RAD;
            imuMsg->angular_velocity.y = sens_data.imu.angular_velocity[1] * DEG2RAD;
            imuMsg->angular_velocity.z = sens_data.imu.angular_velocity[2] * DEG2RAD;

            imuMsg->linear_acceleration.x = sens_data.imu.linear_acceleration[0];
            imuMsg->linear_acceleration.y = sens_data.imu.linear_acceleration[1];
            imuMsg->linear_acceleration.z = sens_data.imu.linear_acceleration[2];

            // ----> Covariances copy
            // Note: memcpy not allowed because ROS2 uses double and ZED SDK uses
            // float
            for (int i = 0; i < 3; ++i) {
                int r = 0;

                if (i == 0) {
                    r = 0;
                } else if (i == 1) {
                    r = 1;
                } else {
                    r = 2;
                }

                imuMsg->orientation_covariance[i * 3 + 0] = sens_data.imu.pose_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
                imuMsg->orientation_covariance[i * 3 + 1] = sens_data.imu.pose_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
                imuMsg->orientation_covariance[i * 3 + 2] = sens_data.imu.pose_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;

                imuMsg->linear_acceleration_covariance[i * 3 + 0] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
                imuMsg->linear_acceleration_covariance[i * 3 + 1] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
                imuMsg->linear_acceleration_covariance[i * 3 + 2] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];

                imuMsg->angular_velocity_covariance[i * 3 + 0] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
                imuMsg->angular_velocity_covariance[i * 3 + 1] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
                imuMsg->angular_velocity_covariance[i * 3 + 2] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
            }
            // <---- Covariances copy

            RCLCPP_DEBUG_STREAM(get_logger(), "Publishing IMU message");
            mPubImu->publish(std::move(imuMsg));
        } else {
            mImuPublishing = false;
        }

        if (imu_RawSubNumber > 0) {
            mImuPublishing = true;

            imuMsgPtr imuRawMsg = std::make_unique<sensor_msgs::msg::Imu>();

            imuRawMsg->header.stamp = ts_imu;
            imuRawMsg->header.frame_id = mImuFrameId;

            imuRawMsg->angular_velocity.x = sens_data.imu.angular_velocity[0] * DEG2RAD;
            imuRawMsg->angular_velocity.y = sens_data.imu.angular_velocity[1] * DEG2RAD;
            imuRawMsg->angular_velocity.z = sens_data.imu.angular_velocity[2] * DEG2RAD;

            imuRawMsg->linear_acceleration.x = sens_data.imu.linear_acceleration[0];
            imuRawMsg->linear_acceleration.y = sens_data.imu.linear_acceleration[1];
            imuRawMsg->linear_acceleration.z = sens_data.imu.linear_acceleration[2];

            // ----> Covariances copy
            // Note: memcpy not allowed because ROS2 uses double and ZED SDK uses
            // float
            for (int i = 0; i < 3; ++i) {
                int r = 0;

                if (i == 0) {
                    r = 0;
                } else if (i == 1) {
                    r = 1;
                } else {
                    r = 2;
                }

                imuRawMsg->linear_acceleration_covariance[i * 3 + 0] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
                imuRawMsg->linear_acceleration_covariance[i * 3 + 1] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
                imuRawMsg->linear_acceleration_covariance[i * 3 + 2] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];

                imuRawMsg->angular_velocity_covariance[i * 3 + 0] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
                imuRawMsg->angular_velocity_covariance[i * 3 + 1] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
                imuRawMsg->angular_velocity_covariance[i * 3 + 2] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
            }
            // <---- Covariances copy

            RCLCPP_DEBUG_STREAM(get_logger(), "Publishing IMU RAW message");
            mPubImuRaw->publish(std::move(imuRawMsg));
        }

        if (imu_TempSubNumber > 0) {
            mImuPublishing = true;

            tempMsgPtr imuTempMsg = std::make_unique<sensor_msgs::msg::Temperature>();

            imuTempMsg->header.stamp = ts_imu;

            imuTempMsg->header.frame_id = mImuFrameId;
            float imu_temp;
            sens_data.temperature.get(
                sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU, imu_temp);
            imuTempMsg->temperature = static_cast<double>(imu_temp);
            imuTempMsg->variance = 0.0;

            RCLCPP_DEBUG_STREAM(get_logger(), "Publishing IMU TEMP message");
            mPubImuTemp->publish(std::move(imuTempMsg));
        }
    }

    if (sens_data.barometer.is_available && new_baro_data) {
        if (pressSubNumber > 0) {
            mBaroPublishing = true;

            pressMsgPtr pressMsg = std::make_unique<sensor_msgs::msg::FluidPressure>();

            pressMsg->header.stamp = ts_baro;
            pressMsg->header.frame_id = mBaroFrameId;
            pressMsg->fluid_pressure = sens_data.barometer.pressure * 1e2; // Pascal
            pressMsg->variance = 1.0585e-2;

            RCLCPP_DEBUG_STREAM(get_logger(), "Publishing PRESS message");
            mPubPressure->publish(std::move(pressMsg));
        } else {
            mBaroPublishing = false;
        }

        if (tempLeftSubNumber > 0) {
            mBaroPublishing = true;

            tempMsgPtr leftTempMsg = std::make_unique<sensor_msgs::msg::Temperature>();

            leftTempMsg->header.stamp = ts_baro;

            leftTempMsg->header.frame_id = mTempLeftFrameId;
            leftTempMsg->temperature = static_cast<double>(mTempLeft);
            leftTempMsg->variance = 0.0;

            mPubTempL->publish(std::move(leftTempMsg));
        }

        if (tempRightSubNumber > 0) {
            mBaroPublishing = true;

            tempMsgPtr rightTempMsg = std::make_unique<sensor_msgs::msg::Temperature>();

            rightTempMsg->header.stamp = ts_baro;

            rightTempMsg->header.frame_id = mTempRightFrameId;
            rightTempMsg->temperature = static_cast<double>(mTempRight);
            rightTempMsg->variance = 0.0;

            RCLCPP_DEBUG_STREAM(get_logger(), "Publishing RIGHT TEMP message");
            mPubTempR->publish(std::move(rightTempMsg));
        }
    }

    if (sens_data.magnetometer.is_available && new_mag_data) {
        if (imu_MagSubNumber > 0) {
            mMagPublishing = true;

            magMsgPtr magMsg = std::make_unique<sensor_msgs::msg::MagneticField>();

            magMsg->header.stamp = ts_mag;
            magMsg->header.frame_id = mMagFrameId;
            magMsg->magnetic_field.x = sens_data.magnetometer.magnetic_field_calibrated.x * 1e-6; // Tesla
            magMsg->magnetic_field.y = sens_data.magnetometer.magnetic_field_calibrated.y * 1e-6; // Tesla
            magMsg->magnetic_field.z = sens_data.magnetometer.magnetic_field_calibrated.z * 1e-6; // Tesla
            magMsg->magnetic_field_covariance[0] = 0.039e-6;
            magMsg->magnetic_field_covariance[1] = 0.0f;
            magMsg->magnetic_field_covariance[2] = 0.0f;
            magMsg->magnetic_field_covariance[3] = 0.0f;
            magMsg->magnetic_field_covariance[4] = 0.037e-6;
            magMsg->magnetic_field_covariance[5] = 0.0f;
            magMsg->magnetic_field_covariance[6] = 0.0f;
            magMsg->magnetic_field_covariance[7] = 0.0f;
            magMsg->magnetic_field_covariance[8] = 0.047e-6;

            RCLCPP_DEBUG_STREAM(get_logger(), "Publishing MAG message");
            mPubImuMag->publish(std::move(magMsg));
        } else {
            mMagPublishing = false;
        }
    }
    // <---- Sensors data publishing

    return ts_imu;
}

void ZedCamera::publishTFs(rclcpp::Time t)
{
    // RCLCPP_DEBUG(get_logger(), "publishTFs");

    // RCLCPP_INFO_STREAM(get_logger(), "publishTFs - t type:" <<
    // t.get_clock_type());

    if (!mPosTrackingReady) {
        return;
    }

    if (t == TIMEZERO_ROS) {
        RCLCPP_DEBUG(get_logger(), "Time zero: not publishing TFs");
        return;
    }

    // Publish pose tf only if enabled
    if (mDepthQuality != sl::DEPTH_MODE::NONE && mPublishTF) {
        publishOdomTF(t); // publish the base Frame in odometry frame

        if (mPublishMapTF) {
            publishPoseTF(t); // publish the odometry Frame in map frame
        }

        if (mCamRealModel != sl::MODEL::ZED) {
            if (mPublishImuTF && !mStaticImuFramePublished) {
                publishStaticImuFrameAndTopic();
            }
        }
    }
}

void ZedCamera::publishOdomTF(rclcpp::Time t)
{
    // RCLCPP_DEBUG(get_logger(), "publishOdomTF");

    // ----> Avoid duplicated TF publishing
    static rclcpp::Time last_stamp = TIMEZERO_ROS;

    if (t == last_stamp) {
        return;
    }
    last_stamp = t;
    // <---- Avoid duplicated TF publishing

    if (!mSensor2BaseTransfValid) {
        getSens2BaseTransform();
    }

    if (!mSensor2CameraTransfValid) {
        getSens2CameraTransform();
    }

    if (!mCamera2BaseTransfValid) {
        getCamera2BaseTransform();
    }

    transfMsgPtr transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>();

    transformStamped->header.stamp = t;
    transformStamped->header.frame_id = mOdomFrameId;
    transformStamped->child_frame_id = mBaseFrameId;
    // conversion from Tranform to message
    tf2::Vector3 translation = mOdom2BaseTransf.getOrigin();
    tf2::Quaternion quat = mOdom2BaseTransf.getRotation();
    transformStamped->transform.translation.x = translation.x();
    transformStamped->transform.translation.y = translation.y();
    transformStamped->transform.translation.z = translation.z();
    transformStamped->transform.rotation.x = quat.x();
    transformStamped->transform.rotation.y = quat.y();
    transformStamped->transform.rotation.z = quat.z();
    transformStamped->transform.rotation.w = quat.w();

    // Publish transformation
    mTfBroadcaster->sendTransform(*(transformStamped.get()));
}

void ZedCamera::publishPoseTF(rclcpp::Time t)
{
    // RCLCPP_DEBUG(get_logger(), "publishPoseTF");

    // ----> Avoid duplicated TF publishing
    static rclcpp::Time last_stamp = TIMEZERO_ROS;

    if (t == last_stamp) {
        return;
    }
    last_stamp = t;
    // <---- Avoid duplicated TF publishing

    if (!mSensor2BaseTransfValid) {
        getSens2BaseTransform();
    }

    if (!mSensor2CameraTransfValid) {
        getSens2CameraTransform();
    }

    if (!mCamera2BaseTransfValid) {
        getCamera2BaseTransform();
    }

    transfMsgPtr transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>();

    transformStamped->header.stamp = t;
    transformStamped->header.frame_id = mMapFrameId;
    transformStamped->child_frame_id = mOdomFrameId;
    // conversion from Tranform to message
    tf2::Vector3 translation = mMap2OdomTransf.getOrigin();
    tf2::Quaternion quat = mMap2OdomTransf.getRotation();
    transformStamped->transform.translation.x = translation.x();
    transformStamped->transform.translation.y = translation.y();
    transformStamped->transform.translation.z = translation.z();
    transformStamped->transform.rotation.x = quat.x();
    transformStamped->transform.rotation.y = quat.y();
    transformStamped->transform.rotation.z = quat.z();
    transformStamped->transform.rotation.w = quat.w();

    // Publish transformation
    mTfBroadcaster->sendTransform(*(transformStamped.get()));
}

void ZedCamera::threadFunc_pointcloudElab()
{
    std::unique_lock<std::mutex> lock(mPcMutex);

    RCLCPP_DEBUG(get_logger(), "Point Cloud thread started");

    while (1) {
        if (!rclcpp::ok()) {
            RCLCPP_DEBUG(get_logger(),
                "Ctrl+C received: stopping point cloud thread");
            break;
        }

        // RCLCPP_DEBUG(get_logger(), "pointcloudThreadFunc -> mPcDataReady value:
        // %s", mPcDataReady ? "TRUE" : "FALSE");

        while (!mPcDataReady) { // loop to avoid spurious wakeups
            if (mPcDataReadyCondVar.wait_for(lock, std::chrono::milliseconds(500)) == std::cv_status::timeout) {
                // Check thread stopping
                if (!rclcpp::ok()) {
                    RCLCPP_DEBUG(get_logger(),
                        "Ctrl+C received: stopping point cloud thread");
                    mThreadStop = true;
                    break;
                }
                if (mThreadStop) {
                    RCLCPP_DEBUG(
                        get_logger(),
                        "threadFunc_pointcloudElab (2): Point Cloud thread stopped");
                    break;
                } else {
                    // RCLCPP_DEBUG(get_logger(), "pointcloudThreadFunc -> WAIT FOR CLOUD
                    // DATA");
                    continue;
                }
            }
        }

        if (mThreadStop) {
            RCLCPP_DEBUG(get_logger(),
                "threadFunc_pointcloudElab (1): Point Cloud thread stopped");
            break;
        }

        publishPointCloud();

        // ----> Check publishing frequency
        double pc_period_msec = 1000.0 / mPcPubRate;

        static sl_tools::StopWatch freqTimer;
        double elapsed_msec = freqTimer.toc() * 1e3;

        if (elapsed_msec < pc_period_msec) {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<long int>(pc_period_msec - elapsed_msec)));
        }

        freqTimer.tic();
        // <---- Check publishing frequency

        mPcDataReady = false;
        // RCLCPP_DEBUG(get_logger(), "pointcloudThreadFunc -> mPcDataReady FALSE")
    }

    RCLCPP_DEBUG(get_logger(), "Pointcloud thread finished");
}

void ZedCamera::callback_pubVideoDepth()
{
    RCLCPP_DEBUG_ONCE(get_logger(), "Video Depth callback called");

    static rclcpp::Time prev_ts = TIMEZERO_ROS;

    std::lock_guard<std::mutex> lock(mCloseZedMutex);
    mPublishingData = false;

    rclcpp::Time pub_ts;
    mPublishingData = publishVideoDepth(pub_ts);

    //RCLCPP_DEBUG_ONCE(get_logger(), "Video Depth published");

    // RCLCPP_INFO_STREAM(get_logger(), "callback_pubVideoDepth - pub_ts type:" <<
    // pub_ts.get_clock_type());

    if (mCamRealModel != sl::MODEL::ZED && mPublishingData && pub_ts != TIMEZERO_ROS) {
        if (mSensCameraSync || mSvoMode) {
            publishSensorsData(pub_ts);
        }
    }
}

void ZedCamera::callback_pubSensorsData()
{

    RCLCPP_DEBUG_ONCE(get_logger(), "Sensors callback called");

    std::lock_guard<std::mutex> lock(mCloseZedMutex);
    if (!mZed.isOpened()) {
        return;
    }

    rclcpp::Time sens_ts = publishSensorsData();

    // RCLCPP_INFO_STREAM(get_logger(), "callback_pubSensorsData - sens_ts type:"
    // << sens_ts.get_clock_type());

    // Publish TF at the same frequency of IMU data, so they are always
    // synchronized
    if (sens_ts != TIMEZERO_ROS) {
        publishTFs(sens_ts);
    }
}

bool ZedCamera::publishVideoDepth(rclcpp::Time& out_pub_ts)
{
    sl_tools::StopWatch elabTimer;

    static sl::Timestamp lastZedTs = 0; // Used to calculate stable publish frequency

    static size_t rgbSubnumber = 0;
    static size_t rgbRawSubnumber = 0;
    static size_t rgbGraySubnumber = 0;
    static size_t rgbGrayRawSubnumber = 0;
    static size_t leftSubnumber = 0;
    static size_t leftRawSubnumber = 0;
    static size_t leftGraySubnumber = 0;
    static size_t leftGrayRawSubnumber = 0;
    static size_t rightSubnumber = 0;
    static size_t rightRawSubnumber = 0;
    static size_t rightGraySubnumber = 0;
    static size_t rightGrayRawSubnumber = 0;
    static size_t stereoSubnumber = 0;
    static size_t stereoRawSubnumber = 0;
    static size_t depthSubnumber = 0;
    static size_t confMapSubnumber = 0;
    static size_t disparitySubnumber = 0;
    static size_t depthInfoSubnumber = 0;

    try {
        rgbSubnumber = mPubRgb.getNumSubscribers();
        rgbRawSubnumber = mPubRawRgb.getNumSubscribers();
        rgbGraySubnumber = mPubRgbGray.getNumSubscribers();
        rgbGrayRawSubnumber = mPubRawRgbGray.getNumSubscribers();
        leftSubnumber = mPubLeft.getNumSubscribers();
        leftRawSubnumber = mPubRawLeft.getNumSubscribers();
        leftGraySubnumber = mPubLeftGray.getNumSubscribers();
        leftGrayRawSubnumber = mPubRawLeftGray.getNumSubscribers();
        rightSubnumber = mPubRight.getNumSubscribers();
        rightRawSubnumber = mPubRawRight.getNumSubscribers();
        rightGraySubnumber = mPubRightGray.getNumSubscribers();
        rightGrayRawSubnumber = mPubRawRightGray.getNumSubscribers();
        stereoSubnumber = mPubStereo.getNumSubscribers();
        stereoRawSubnumber = mPubRawStereo.getNumSubscribers();
        if(!mDepthDisabled)
        {
            depthSubnumber = mPubDepth.getNumSubscribers();
            depthInfoSubnumber = count_subscribers(mPubDepthInfo->get_topic_name());
            confMapSubnumber = count_subscribers(mPubConfMap->get_topic_name());
            disparitySubnumber = count_subscribers(mPubDisparity->get_topic_name());
        }
    } catch (...) {
        rcutils_reset_error();
        RCLCPP_DEBUG(get_logger(),
            "publishImages: Exception while counting subscribers");
        return false;
    }

    bool retrieved = false;

    static sl::Mat mat_left, mat_left_raw;
    static sl::Mat mat_right, mat_right_raw;
    static sl::Mat mat_left_gray, mat_left_raw_gray;
    static sl::Mat mat_right_gray, mat_right_raw_gray;
    static sl::Mat mat_depth, mat_disp, mat_conf;

    static sl::Timestamp ts_rgb = 0; // used to check RGB/Depth sync
    static sl::Timestamp ts_depth = 0; // used to check RGB/Depth sync
    static sl::Timestamp grab_ts = 0;

    float min_depth = 0.0f, max_depth = 0.0f;

    // ----> Retrieve all required data
    std::unique_lock<std::timed_mutex> lock(mCamDataMutex, std::defer_lock);

    if (lock.try_lock_for(std::chrono::milliseconds(1000 / mCamGrabFrameRate))) {
        //RCLCPP_DEBUG(get_logger(), "Retrieving Video Data");
        if (rgbSubnumber + leftSubnumber + stereoSubnumber > 0) {
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.retrieveImage(mat_left, sl::VIEW::LEFT, sl::MEM::CPU, mMatResolVideo);
            ts_rgb = mat_left.timestamp;
            grab_ts = mat_left.timestamp;
        }
        if (rgbRawSubnumber + leftRawSubnumber + stereoRawSubnumber > 0) {
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.retrieveImage(mat_left_raw, sl::VIEW::LEFT_UNRECTIFIED, sl::MEM::CPU, mMatResolVideo);
            grab_ts = mat_left_raw.timestamp;
        }
        if (rightSubnumber + stereoSubnumber > 0) {
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.retrieveImage(mat_right, sl::VIEW::RIGHT, sl::MEM::CPU, mMatResolVideo);
            grab_ts = mat_right.timestamp;
        }
        if (rightRawSubnumber + stereoRawSubnumber > 0) {
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.retrieveImage(mat_right_raw, sl::VIEW::RIGHT_UNRECTIFIED, sl::MEM::CPU, mMatResolVideo);
            grab_ts = mat_right_raw.timestamp;
        }
        if (rgbGraySubnumber + leftGraySubnumber > 0) {
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.retrieveImage(mat_left_gray, sl::VIEW::LEFT_GRAY, sl::MEM::CPU, mMatResolVideo);
            grab_ts = mat_left_gray.timestamp;
        }
        if (rgbGrayRawSubnumber + leftGrayRawSubnumber > 0) {
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.retrieveImage(mat_left_raw_gray, sl::VIEW::LEFT_UNRECTIFIED_GRAY, sl::MEM::CPU, mMatResolVideo);
            grab_ts = mat_left_raw_gray.timestamp;
        }
        if (rightGraySubnumber > 0) {
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.retrieveImage(mat_right_gray, sl::VIEW::RIGHT_GRAY, sl::MEM::CPU, mMatResolVideo);
            grab_ts = mat_right_gray.timestamp;
        }
        if (rightGrayRawSubnumber > 0) {
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.retrieveImage(mat_right_raw_gray, sl::VIEW::RIGHT_UNRECTIFIED_GRAY, sl::MEM::CPU, mMatResolVideo);
            grab_ts = mat_right_raw_gray.timestamp;
        }
        //RCLCPP_DEBUG(get_logger(), "Video Data retrieved");
        //RCLCPP_DEBUG(get_logger(), "Retrieving Depth Data");
        if (depthSubnumber > 0 || depthInfoSubnumber > 0) {
            RCLCPP_DEBUG(get_logger(), "Retrieving Depth");
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, mMatResolDepth);
            grab_ts = mat_depth.timestamp;

            ts_depth = mat_depth.timestamp;

            if (ts_rgb.data_ns != 0 && (ts_depth.data_ns != ts_rgb.data_ns)) {
                RCLCPP_WARN_STREAM(get_logger(),
                    "!!!!! DEPTH/RGB ASYNC !!!!! - Delta: "
                        << 1e-9 * static_cast<double>(ts_depth - ts_rgb)
                        << " sec");
            }
        }
        if (disparitySubnumber > 0) {
            RCLCPP_DEBUG(get_logger(), "Retrieving Disparity");
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.retrieveMeasure(mat_disp, sl::MEASURE::DISPARITY, sl::MEM::CPU, mMatResolDepth);
            grab_ts = mat_disp.timestamp;
        }
        if (confMapSubnumber > 0) {
            RCLCPP_DEBUG(get_logger(), "Retrieving Confidence");
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.retrieveMeasure(mat_conf, sl::MEASURE::CONFIDENCE, sl::MEM::CPU, mMatResolDepth);
            grab_ts = mat_conf.timestamp;
        }
        if (depthInfoSubnumber > 0) {
            retrieved |= sl::ERROR_CODE::SUCCESS == mZed.getCurrentMinMaxDepth(min_depth, max_depth);
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION == 7 && ZED_SDK_PATCH_VERSION == 0 // Units bug workaround
            min_depth *= 0.001f;
            max_depth *= 0.001f;
#endif
        }
        //RCLCPP_DEBUG(get_logger(), "Depth Data retrieved");
    } else {
        RCLCPP_DEBUG(get_logger(), "Lock timeout");
    }
    // <---- Retrieve all required data

    // ----> Notify grab thread that all data are synchronized and a new grab can
    // be done
    mRgbDepthDataRetrievedCondVar.notify_one();
    mRgbDepthDataRetrieved = true;
    // <---- Notify grab thread that all data are synchronized and a new grab can
    // be done

    if (!retrieved) {
        lastZedTs = 0;
        out_pub_ts = TIMEZERO_ROS;
        return false;
    }

    // ----> Check if a grab has been done before publishing the same images
    if (grab_ts.data_ns == lastZedTs.data_ns) {
        out_pub_ts = TIMEZERO_ROS;
        // Data not updated by a grab calling in the grab thread
        return true;
    }

    if (lastZedTs.data_ns != 0) {
        double period_sec = static_cast<double>(grab_ts.data_ns - lastZedTs.data_ns) / 1e9;
        RCLCPP_DEBUG_STREAM(get_logger(),
            "VIDEO/DEPTH PUB LAST PERIOD: "
                << period_sec << " sec @" << 1. / period_sec
                << " Hz");

        mVideoDepthPeriodMean_sec->addValue(period_sec);
        RCLCPP_DEBUG_STREAM(get_logger(),
            "VIDEO/DEPTH PUB MEAN PERIOD: "
                << mVideoDepthPeriodMean_sec->getMean() << " sec @"
                << 1. / mVideoDepthPeriodMean_sec->getMean()
                << " Hz");
    }
    lastZedTs = grab_ts;
    // <---- Check if a grab has been done before publishing the same images

    static rclcpp::Time timeStamp;
    if (!mSvoMode) {
        timeStamp = sl_tools::slTime2Ros(grab_ts, get_clock()->get_clock_type());
    } else {
        timeStamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::CURRENT),
            get_clock()->get_clock_type());
    }

    out_pub_ts = timeStamp;

    // ----> Publish the left=rgb image if someone has subscribed to
    if (leftSubnumber > 0) {
        publishImageWithInfo(
            mat_left, mPubLeft, mLeftCamInfoMsg, mLeftCamOptFrameId, timeStamp);
    }
    if (rgbSubnumber > 0) {
        publishImageWithInfo(
            mat_left, mPubRgb, mRgbCamInfoMsg, mDepthOptFrameId, timeStamp);
    }
    // <---- Publish the left=rgb image if someone has subscribed to

    // ----> Publish the left_raw=rgb_raw image if someone has subscribed to
    if (leftRawSubnumber > 0) {
        publishImageWithInfo(mat_left_raw,
            mPubRawLeft,
            mLeftCamInfoRawMsg,
            mLeftCamOptFrameId,
            timeStamp);
    }
    if (rgbRawSubnumber > 0) {
        publishImageWithInfo(
            mat_left_raw, mPubRawRgb, mRgbCamInfoRawMsg, mDepthOptFrameId, timeStamp);
    }
    // <---- Publish the left_raw=rgb_raw image if someone has subscribed to

    // ----> Publish the left_gray=rgb_gray image if someone has subscribed to
    if (leftGraySubnumber > 0) {
        publishImageWithInfo(mat_left_gray,
            mPubLeftGray,
            mLeftCamInfoMsg,
            mLeftCamOptFrameId,
            timeStamp);
    }
    if (rgbGraySubnumber > 0) {
        publishImageWithInfo(
            mat_left_gray, mPubRgbGray, mRgbCamInfoMsg, mDepthOptFrameId, timeStamp);
    }
    // <---- Publish the left_raw=rgb_raw image if someone has subscribed to

    // ----> Publish the left_raw_gray=rgb_raw_gray image if someone has
    // subscribed to
    if (leftGrayRawSubnumber > 0) {
        publishImageWithInfo(mat_left_raw_gray,
            mPubRawLeftGray,
            mLeftCamInfoRawMsg,
            mLeftCamOptFrameId,
            timeStamp);
    }
    if (rgbGrayRawSubnumber > 0) {
        publishImageWithInfo(mat_left_raw_gray,
            mPubRawRgbGray,
            mRgbCamInfoRawMsg,
            mDepthOptFrameId,
            timeStamp);
    }
    // ----> Publish the left_raw_gray=rgb_raw_gray image if someone has
    // subscribed to

    // ----> Publish the right image if someone has subscribed to
    if (rightSubnumber > 0) {
        publishImageWithInfo(
            mat_right, mPubRight, mRightCamInfoMsg, mRightCamOptFrameId, timeStamp);
    }
    // <---- Publish the right image if someone has subscribed to

    // ----> Publish the right raw image if someone has subscribed to
    if (rightRawSubnumber > 0) {
        publishImageWithInfo(mat_right_raw,
            mPubRawRight,
            mRightCamInfoRawMsg,
            mRightCamOptFrameId,
            timeStamp);
    }
    // <---- Publish the right raw image if someone has subscribed to

    // ----> Publish the right gray image if someone has subscribed to
    if (rightGraySubnumber > 0) {
        publishImageWithInfo(mat_right_gray,
            mPubRightGray,
            mRightCamInfoMsg,
            mRightCamOptFrameId,
            timeStamp);
    }
    // <---- Publish the right gray image if someone has subscribed to

    // ----> Publish the right raw gray image if someone has subscribed to
    if (rightGrayRawSubnumber > 0) {
        publishImageWithInfo(mat_right_raw_gray,
            mPubRawRightGray,
            mRightCamInfoRawMsg,
            mRightCamOptFrameId,
            timeStamp);
    }
    // <---- Publish the right raw gray image if someone has subscribed to

    // ----> Publish the side-by-side image if someone has subscribed to
    if (stereoSubnumber > 0) {
        auto combined = sl_tools::imagesToROSmsg(mat_left, mat_right, mCameraFrameId, timeStamp);
        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing SIDE-BY-SIDE message");
        mPubStereo.publish(combined);
    }
    // <---- Publish the side-by-side image if someone has subscribed to

    // ----> Publish the side-by-side image if someone has subscribed to
    if (stereoRawSubnumber > 0) {
        auto combined = sl_tools::imagesToROSmsg(
            mat_left_raw, mat_right_raw, mCameraFrameId, timeStamp);
        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing SIDE-BY-SIDE RAW message");
        mPubRawStereo.publish(combined);
    }
    // <---- Publish the side-by-side image if someone has subscribed to

    // ---->  Publish the depth image if someone has subscribed to
    if (depthSubnumber > 0) {
        publishDepthMapWithInfo(mat_depth, timeStamp);
    }
    // <----  Publish the depth image if someone has subscribed to

    // ---->  Publish the confidence image and map if someone has subscribed to
    if (confMapSubnumber > 0) {
        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing CONF MAP message");
        mPubConfMap->publish(
            *sl_tools::imageToROSmsg(mat_conf, mDepthOptFrameId, timeStamp));
    }
    // <----  Publish the confidence image and map if someone has subscribed to

    // ----> Publish the disparity image if someone has subscribed to
    if (disparitySubnumber > 0) {
        publishDisparity(mat_disp, timeStamp);
    }
    // <---- Publish the disparity image if someone has subscribed to

    // ----> Publish the depth info if someone has subscribed to
    if (depthInfoSubnumber > 0) {
        depthInfoMsgPtr depthInfoMsg = std::make_unique<zed_interfaces::msg::DepthInfoStamped>();
        depthInfoMsg->header.stamp = timeStamp;
        depthInfoMsg->header.frame_id = mDepthOptFrameId;
        depthInfoMsg->min_depth = min_depth;
        depthInfoMsg->max_depth = max_depth;

        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing DEPTH INFO message");
        mPubDepthInfo->publish(std::move(depthInfoMsg));
    }
    // <---- Publish the depth info if someone has subscribed to

    // Diagnostic statistic
    mVideoDepthElabMean_sec->addValue(elabTimer.toc());

    return true;
}

void ZedCamera::publishImageWithInfo(sl::Mat& img,
    image_transport::CameraPublisher& pubImg,
    camInfoMsgPtr& camInfoMsg,
    std::string imgFrameId,
    rclcpp::Time t)
{
    auto image = sl_tools::imageToROSmsg(img, imgFrameId, t);
    camInfoMsg->header.stamp = t;
    RCLCPP_DEBUG_STREAM(get_logger(), "Publishing IMAGE message");
    pubImg.publish(image, camInfoMsg); // TODO CHECK FOR ZERO-COPY
}

void ZedCamera::processOdometry()
{
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    if (!mSensor2BaseTransfValid) {
        getSens2BaseTransform();
    }

    if (!mSensor2CameraTransfValid) {
        getSens2CameraTransform();
    }

    if (!mCamera2BaseTransfValid) {
        getCamera2BaseTransform();
    }

    if (!mInitOdomWithPose) {
        sl::Pose deltaOdom;

        mPosTrackingStatus = mZed.getPosition(deltaOdom, sl::REFERENCE_FRAME::CAMERA);

        sl::Translation translation = deltaOdom.getTranslation();
        sl::Orientation quat = deltaOdom.getOrientation();

#if 0
        RCLCPP_DEBUG(get_logger(), "delta ODOM [%s] - %.2f,%.2f,%.2f %.2f,%.2f,%.2f,%.2f",
                     sl::toString(mPosTrackingStatus).c_str(),
                     translation(0), translation(1), translation(2),
                     quat(0), quat(1), quat(2), quat(3));
#endif

        if (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK || mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::SEARCHING || mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW) {
            // Transform ZED delta odom pose in TF2 Transformation
            tf2::Transform deltaOdomTf;
            deltaOdomTf.setOrigin(
                tf2::Vector3(translation(0), translation(1), translation(2)));
            // w at the end in the constructor
            deltaOdomTf.setRotation(
                tf2::Quaternion(quat(0), quat(1), quat(2), quat(3)));

            // delta odom from sensor to base frame
            tf2::Transform deltaOdomTf_base = mSensor2BaseTransf.inverse() * deltaOdomTf * mSensor2BaseTransf;

            // Propagate Odom transform in time
            mOdom2BaseTransf = mOdom2BaseTransf * deltaOdomTf_base;

            if (mTwoDMode) {
                tf2::Vector3 tr_2d = mOdom2BaseTransf.getOrigin();
                tr_2d.setZ(mFixedZValue);
                mOdom2BaseTransf.setOrigin(tr_2d);

                double roll, pitch, yaw;
                tf2::Matrix3x3(mOdom2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

                tf2::Quaternion quat_2d;
                quat_2d.setRPY(0.0, 0.0, yaw);

                mOdom2BaseTransf.setRotation(quat_2d);
            }

#if 0
            double roll, pitch, yaw;
            tf2::Matrix3x3(mOdom2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

            RCLCPP_DEBUG(get_logger(), "+++ Odometry [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                         mOdomFrameId.c_str(), mBaseFrameId.c_str(),
                         mOdom2BaseTransf.getOrigin().x(),
                         mOdom2BaseTransf.getOrigin().y(),
                         mOdom2BaseTransf.getOrigin().z(),
                         roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif

            // Publish odometry message
            publishOdom(mOdom2BaseTransf, deltaOdom, mFrameTimestamp);
            mPosTrackingReady = true;
        }
    } else if (mFloorAlignment) {
        RCLCPP_DEBUG_THROTTLE(get_logger(),
            steady_clock,
            5.0,
            "Odometry will be published as soon as the floor as "
            "been detected for the first time");
    }
}

void ZedCamera::publishOdom(tf2::Transform& odom2baseTransf,
    sl::Pose& slPose,
    rclcpp::Time t)
{
    odomMsgPtr odomMsg = std::make_unique<nav_msgs::msg::Odometry>();

    odomMsg->header.stamp = t;
    odomMsg->header.frame_id = mOdomFrameId; // frame
    odomMsg->child_frame_id = mBaseFrameId; // camera_frame

    // Add all value in odometry message
    odomMsg->pose.pose.position.x = odom2baseTransf.getOrigin().x();
    odomMsg->pose.pose.position.y = odom2baseTransf.getOrigin().y();
    odomMsg->pose.pose.position.z = odom2baseTransf.getOrigin().z();
    odomMsg->pose.pose.orientation.x = odom2baseTransf.getRotation().x();
    odomMsg->pose.pose.orientation.y = odom2baseTransf.getRotation().y();
    odomMsg->pose.pose.orientation.z = odom2baseTransf.getRotation().z();
    odomMsg->pose.pose.orientation.w = odom2baseTransf.getRotation().w();

    // Odometry pose covariance
    for (size_t i = 0; i < odomMsg->pose.covariance.size(); i++) {
        odomMsg->pose.covariance[i] = static_cast<double>(slPose.pose_covariance[i]);

        if (mTwoDMode) {
            if (i == 14 || i == 21 || i == 28) {
                odomMsg->pose.covariance[i] = 1e-9; // Very low covariance if 2D mode
            } else if ((i >= 2 && i <= 4) || (i >= 8 && i <= 10) || (i >= 12 && i <= 13) || (i >= 15 && i <= 16) || (i >= 18 && i <= 20) || (i == 22) || (i >= 24 && i <= 27)) {
                odomMsg->pose.covariance[i] = 0.0;
            }
        }
    }

    // Publish odometry message
    RCLCPP_DEBUG_STREAM(get_logger(), "Publishing ODOM message");
    mPubOdom->publish(std::move(odomMsg));
}

void ZedCamera::processPose()
{
    if (!mSensor2BaseTransfValid) {
        getSens2BaseTransform();
    }

    if (!mSensor2CameraTransfValid) {
        getSens2CameraTransform();
    }

    if (!mCamera2BaseTransfValid) {
        getCamera2BaseTransform();
    }

    size_t odomSub = 0;
    try {
        odomSub = count_subscribers(mOdomTopic); // mPubOdom subscribers
    } catch (...) {
        rcutils_reset_error();
        RCLCPP_DEBUG(get_logger(),
            "processPose: Exception while counting subscribers");
        return;
    }

    static sl::POSITIONAL_TRACKING_STATE oldStatus;
    mPosTrackingStatus = mZed.getPosition(mLastZedPose, sl::REFERENCE_FRAME::WORLD);

    sl::Translation translation = mLastZedPose.getTranslation();
    sl::Orientation quat = mLastZedPose.getOrientation();

    if (quat.sum() == 0) {
        return;
    }

#if 0 // Enable for TF checking
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(quat.ox, quat.oy, quat.oz, quat.ow)).getRPY(roll, pitch, yaw);

    RCLCPP_DEBUG(get_logger(), "Sensor POSE [%s -> %s] - {%.2f,%.2f,%.2f} {%.2f,%.2f,%.2f}",
                 mLeftCamFrameId.c_str(), mMapFrameId.c_str(),
                 translation.x, translation.y, translation.z,
                 roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

    RCLCPP_DEBUG(get_logger(), "MAP -> Tracking Status: %s", sl::toString(mTrackingStatus).c_str());
#endif

    if (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK || mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::SEARCHING) {
        tf2::Transform map_to_sens_transf;
        map_to_sens_transf.setOrigin(
            tf2::Vector3(translation(0), translation(1), translation(2)));
        map_to_sens_transf.setRotation(
            tf2::Quaternion(quat(0), quat(1), quat(2), quat(3)));

        mMap2BaseTransf = map_to_sens_transf * mSensor2BaseTransf; // Base position in map frame

        if (mTwoDMode) {
            tf2::Vector3 tr_2d = mMap2BaseTransf.getOrigin();
            tr_2d.setZ(mFixedZValue);
            mMap2BaseTransf.setOrigin(tr_2d);

            double roll, pitch, yaw;
            tf2::Matrix3x3(mMap2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

            tf2::Quaternion quat_2d;
            quat_2d.setRPY(0.0, 0.0, yaw);

            mMap2BaseTransf.setRotation(quat_2d);
        }

#if 0 // Enable for TF checking
        double roll, pitch, yaw;
        tf2::Matrix3x3(mMap2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

        RCLCPP_DEBUG(get_logger(), "*** Base POSE [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                     mMapFrameId.c_str(), mBaseFrameId.c_str(),
                     mMap2BaseTransf.getOrigin().x(), mMap2BaseTransf.getOrigin().y(), mMap2BaseTransf.getOrigin().z(),
                     roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif

        bool initOdom = false;

        if (!(mFloorAlignment)) {
            initOdom = mInitOdomWithPose;
        } else {
            initOdom = mInitOdomWithPose & (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK);
        }

        if (initOdom || mResetOdom) {
            RCLCPP_INFO(get_logger(), "Odometry aligned to last tracking pose");

            // Propagate Odom transform in time
            mOdom2BaseTransf = mMap2BaseTransf;
            mMap2BaseTransf.setIdentity();

            if (odomSub > 0) {
                // Publish odometry message
                publishOdom(mOdom2BaseTransf, mLastZedPose, mFrameTimestamp);
            }

            mInitOdomWithPose = false;
            mResetOdom = false;
        } else {
            // Transformation from map to odometry frame
            mMap2OdomTransf = mMap2BaseTransf * mOdom2BaseTransf.inverse();

#if 0 // Enable for TF checking
            double roll, pitch, yaw;
            tf2::Matrix3x3(mMap2OdomTransf.getRotation()).getRPY(roll, pitch, yaw);

            RCLCPP_DEBUG(get_logger(), "+++ Diff [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                         mMapFrameId.c_str(), mOdomFrameId.c_str(),
                         mMap2OdomTransf.getOrigin().x(), mMap2OdomTransf.getOrigin().y(), mMap2OdomTransf.getOrigin().z(),
                         roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif
        }

        // Publish Pose message
        publishPose();
        mPosTrackingReady = true;
    }

    oldStatus = mPosTrackingStatus;
}

void ZedCamera::publishPose()
{
    size_t poseSub = 0;
    size_t poseCovSub = 0;

    try {
        poseSub = count_subscribers(mPoseTopic); // mPubPose subscribers
        poseCovSub = count_subscribers(mPoseCovTopic); // mPubPoseCov subscribers
    } catch (...) {
        rcutils_reset_error();
        RCLCPP_DEBUG(get_logger(),
            "publishPose: Exception while counting subscribers");
        return;
    }

    tf2::Transform base_pose;
    base_pose.setIdentity();

    base_pose = mMap2BaseTransf;

    std_msgs::msg::Header header;
    header.stamp = mFrameTimestamp;
    header.frame_id = mMapFrameId; // frame

    geometry_msgs::msg::Pose pose;

    // Add all value in Pose message
    pose.position.x = mMap2BaseTransf.getOrigin().x();
    pose.position.y = mMap2BaseTransf.getOrigin().y();
    pose.position.z = mMap2BaseTransf.getOrigin().z();
    pose.orientation.x = mMap2BaseTransf.getRotation().x();
    pose.orientation.y = mMap2BaseTransf.getRotation().y();
    pose.orientation.z = mMap2BaseTransf.getRotation().z();
    pose.orientation.w = mMap2BaseTransf.getRotation().w();

    if (poseSub > 0) {
        poseMsgPtr poseNoCov = std::make_unique<geometry_msgs::msg::PoseStamped>();

        poseNoCov->header = header;
        poseNoCov->pose = pose;

        // Publish pose stamped message
        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing POSE NO COV message");
        mPubPose->publish(std::move(poseNoCov));
    }

    if (mPublishPoseCov) {
        if (poseCovSub > 0) {
            poseCovMsgPtr poseCov = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();

            poseCov->header = header;
            poseCov->pose.pose = pose;

            // Odometry pose covariance if available

            for (size_t i = 0; i < poseCov->pose.covariance.size(); i++) {
                poseCov->pose.covariance[i] = static_cast<double>(mLastZedPose.pose_covariance[i]);

                if (mTwoDMode) {
                    if ((i >= 2 && i <= 4) || (i >= 8 && i <= 10) || (i >= 12 && i <= 29) || (i >= 32 && i <= 34)) {
                        poseCov->pose.covariance[i] = 1e-9; // Very low covariance if 2D mode
                    }
                }
            }

            // Publish pose with covariance stamped message
            RCLCPP_DEBUG_STREAM(get_logger(), "Publishing POSE COV message");
            mPubPoseCov->publish(std::move(poseCov));
        }
    }
}

void ZedCamera::processDetectedObjects(rclcpp::Time t)
{
    size_t objdet_sub_count = 0;

    try {
        objdet_sub_count = count_subscribers(mPubObjDet->get_topic_name());
    } catch (...) {
        rcutils_reset_error();
        RCLCPP_DEBUG(
            get_logger(),
            "processDetectedObjects: Exception while counting subscribers");
        return;
    }

    if (objdet_sub_count < 1) {
        mObjDetSubscribed = false;
        return;
    }

    sl_tools::StopWatch elabTimer;
    static sl_tools::StopWatch freqTimer;

    mObjDetSubscribed = true;

    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;

    // ----> Process realtime dynamic parameters
    objectTracker_parameters_rt.detection_confidence_threshold = mObjDetConfidence;
    mObjDetFilter.clear();
    if (mObjDetPeopleEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::PERSON);
    }
    if (mObjDetVehiclesEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::VEHICLE);
    }
    if (mObjDetBagsEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::BAG);
    }
    if (mObjDetAnimalsEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::ANIMAL);
    }
    if (mObjDetElectronicsEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::ELECTRONICS);
    }
    if (mObjDetFruitsEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::FRUIT_VEGETABLE);
    }
    if (mObjDetSportEnable) {
        mObjDetFilter.push_back(sl::OBJECT_CLASS::SPORT);
    }
    objectTracker_parameters_rt.object_class_filter = mObjDetFilter;
    // <---- Process realtime dynamic parameters

    sl::Objects objects;

    sl::ERROR_CODE objDetRes = mZed.retrieveObjects(objects, objectTracker_parameters_rt);

    if (objDetRes != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(get_logger(),
            "Object Detection error: " << sl::toString(objDetRes));
        return;
    }

    if (!objects.is_new) // Async object detection. Update data only if new
    // detection is available
    {
        return;
    }

    // RCLCPP_DEBUG_STREAM(get_logger(), "Detected " << objects.object_list.size()
    // << " objects");

    size_t objCount = objects.object_list.size();

    objDetMsgPtr objMsg = std::make_unique<zed_interfaces::msg::ObjectsStamped>();

    objMsg->header.stamp = t;
    objMsg->header.frame_id = mLeftCamFrameId;

    objMsg->objects.resize(objCount);

    size_t idx = 0;
    for (auto data : objects.object_list) {
        objMsg->objects[idx].label = sl::toString(data.label).c_str();
        objMsg->objects[idx].sublabel = sl::toString(data.sublabel).c_str();
        objMsg->objects[idx].label_id = data.id;
        objMsg->objects[idx].confidence = data.confidence;

        memcpy(&(objMsg->objects[idx].position[0]),
            &(data.position[0]),
            3 * sizeof(float));
        memcpy(&(objMsg->objects[idx].position_covariance[0]),
            &(data.position_covariance[0]),
            6 * sizeof(float));
        memcpy(&(objMsg->objects[idx].velocity[0]),
            &(data.velocity[0]),
            3 * sizeof(float));

        objMsg->objects[idx].tracking_available = mObjDetTracking;
        objMsg->objects[idx].tracking_state = static_cast<int8_t>(data.tracking_state);
        objMsg->objects[idx].action_state = static_cast<int8_t>(data.action_state);

        if (data.bounding_box_2d.size() == 4) {
            memcpy(&(objMsg->objects[idx].bounding_box_2d.corners[0]),
                &(data.bounding_box_2d[0]),
                8 * sizeof(unsigned int));
        }
        if (data.bounding_box.size() == 8) {
            memcpy(&(objMsg->objects[idx].bounding_box_3d.corners[0]),
                &(data.bounding_box[0]),
                24 * sizeof(float));
        }

        memcpy(&(objMsg->objects[idx].dimensions_3d[0]),
            &(data.dimensions[0]),
            3 * sizeof(float));

        objMsg->objects[idx].body_format = static_cast<uint8_t>(mObjDetBodyFmt);

        if (mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE ||
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 5
            mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_MEDIUM ||
#endif
            mObjDetModel == sl::DETECTION_MODEL::HUMAN_BODY_FAST) {
            objMsg->objects[idx].skeleton_available = true;

            if (data.head_bounding_box_2d.size() == 4) {
                memcpy(&(objMsg->objects[idx].head_bounding_box_2d.corners[0]),
                    &(data.head_bounding_box_2d[0]),
                    8 * sizeof(unsigned int));
            }
            if (data.head_bounding_box.size() == 8) {
                memcpy(&(objMsg->objects[idx].head_bounding_box_3d.corners[0]),
                    &(data.head_bounding_box[0]),
                    24 * sizeof(float));
            }
            memcpy(&(objMsg->objects[idx].head_position[0]),
                &(data.head_position[0]),
                3 * sizeof(float));

            uint8_t kp_size = data.keypoint_2d.size();
            if (kp_size == 18 || kp_size == 34) {

                memcpy(&(objMsg->objects[idx].skeleton_2d.keypoints[0]),
                    &(data.keypoint_2d[0]),
                    2 * kp_size * sizeof(float));

                memcpy(&(objMsg->objects[idx].skeleton_3d.keypoints[0]),
                    &(data.keypoint[0]),
                    3 * kp_size * sizeof(float));
            }
        } else {
            objMsg->objects[idx].skeleton_available = false;
        }

        // at the end of the loop
        idx++;
    }

    RCLCPP_DEBUG_STREAM(get_logger(), "Publishing OBJ DET message");
    mPubObjDet->publish(std::move(objMsg));

    // ----> Diagnostic information update
    mObjDetElabMean_sec->addValue(elabTimer.toc());
    mObjDetPeriodMean_sec->addValue(freqTimer.toc());
    freqTimer.tic();
    // <---- Diagnostic information update
}

bool ZedCamera::isDepthRequired()
{
    //RCLCPP_DEBUG_STREAM(get_logger(), "isDepthRequired called");

    if (mDepthDisabled) {
        return false;
    }

    size_t tot_sub = 0;
    size_t depthSub = 0;
    size_t confMapSub = 0;
    size_t dispSub = 0;
    size_t pcSub = 0;
    size_t depthInfoSub = 0;

    try {
        depthSub = mPubDepth.getNumSubscribers();
        confMapSub = count_subscribers(mPubConfMap->get_topic_name());
        dispSub = count_subscribers(mPubDisparity->get_topic_name());
        pcSub = count_subscribers(mPubCloud->get_topic_name());
        depthInfoSub = count_subscribers(mPubDepthInfo->get_topic_name());

        tot_sub = depthSub + confMapSub + dispSub + pcSub + depthInfoSub;
    } catch (...) {
        rcutils_reset_error();
        RCLCPP_DEBUG(get_logger(),
            "isDepthRequired: Exception while counting subscribers");
        return false;
    }

    return tot_sub > 0 || isPosTrackingRequired();
}

void ZedCamera::applyDepthSettings()
{
    if (isDepthRequired()) {
        mDynParMutex.lock();
        mRunParams.confidence_threshold = mDepthConf; // Update depth confidence if changed
        mRunParams.texture_confidence_threshold = mDepthTextConf; // Update depth texture confidence if changed
        mRunParams.remove_saturated_areas = mRemoveSatAreas;
        mDynParMutex.unlock();

        RCLCPP_DEBUG_ONCE(get_logger(), "Depth extraction enabled");
        mRunParams.enable_depth = true;
    } else {
        RCLCPP_DEBUG_ONCE(get_logger(), "Depth extraction disabled");
        mRunParams.enable_depth = false;
    }
}

void ZedCamera::applyVideoSettings()
{
    if (!mSvoMode && mFrameCount % 5 == 0) {
        mDynParMutex.lock();
        if (mCamAutoExpGain) {
            if (mTriggerAutoExpGain) {
                mZed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 1);
                mTriggerAutoExpGain = false;
            }
        } else {
            int exposure = mZed.getCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE);
            if (exposure != mCamExposure) {
                mZed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, mCamExposure);
            }

            int gain = mZed.getCameraSettings(sl::VIDEO_SETTINGS::GAIN);
            if (gain != mCamGain) {
                mZed.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, mCamGain);
            }
        }
        if (mCamAutoWB) {
            if (mTriggerAutoWB) {
                mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 1);
                mTriggerAutoWB = false;
            }
        } else {
            int wb = mZed.getCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE);
            if (wb != mCamWBTemp) {
                mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE,
                    mCamWBTemp);
            }
        }
        int brgt = mZed.getCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS);
        if (brgt != mCamBrightness) {
            mZed.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, mCamBrightness);
        }
        int contr = mZed.getCameraSettings(sl::VIDEO_SETTINGS::CONTRAST);
        if (contr != mCamContrast) {
            mZed.setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, mCamContrast);
        }
        int hue = mZed.getCameraSettings(sl::VIDEO_SETTINGS::HUE);
        if (hue != mCamHue) {
            mZed.setCameraSettings(sl::VIDEO_SETTINGS::HUE, mCamHue);
        }
        int sat = mZed.getCameraSettings(sl::VIDEO_SETTINGS::SATURATION);
        if (sat != mCamSaturation) {
            mZed.setCameraSettings(sl::VIDEO_SETTINGS::HUE, mCamSaturation);
        }
        int sharp = mZed.getCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS);
        if (sharp != mCamSharpness) {
            mZed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, mCamSharpness);
        }
        int gamma = mZed.getCameraSettings(sl::VIDEO_SETTINGS::GAMMA);
        if (gamma != mCamGamma) {
            mZed.setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, mCamGamma);
        }

        mDynParMutex.unlock();
    }
}

bool ZedCamera::isPosTrackingRequired()
{
    if (mDepthDisabled) {
        return false;
    }

    if (mPosTrackingEnabled) {
        return true;
    }

    if (mPublishTF) {
        return true;
    }

    if (mDepthStabilization) {
        return true;
    }

    if (mMappingEnabled || mObjDetEnabled) {
        return true;
    }

    size_t topics_sub = 0;
    try {
        topics_sub = count_subscribers(mPubPose->get_topic_name()) + +count_subscribers(mPubPoseCov->get_topic_name()) + count_subscribers(mPubPosePath->get_topic_name()) + count_subscribers(mPubOdom->get_topic_name()) + count_subscribers(mPubOdomPath->get_topic_name());
    } catch (...) {
        rcutils_reset_error();
        RCLCPP_DEBUG(get_logger(),
            "isPosTrackingRequired: Exception while counting subscribers");
        return false;
    }

    if (topics_sub > 0) {
        return true;
    }

    return false;
}

void ZedCamera::publishDepthMapWithInfo(sl::Mat& depth, rclcpp::Time t)
{
    mDepthCamInfoMsg->header.stamp = t;

    if (!mOpenniDepthMode) {
        auto depth_img = sl_tools::imageToROSmsg(depth, mDepthOptFrameId, t);
        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing DEPTH message");
        mPubDepth.publish(depth_img, mDepthCamInfoMsg); // TODO CHECK FOR ZERO-COPY
        return;
    }

    // OPENNI CONVERSION (meter -> millimeters - float32 -> uint16)
    std::shared_ptr<sensor_msgs::msg::Image> openniDepthMsg = std::make_shared<sensor_msgs::msg::Image>();

    openniDepthMsg->header.stamp = t;
    openniDepthMsg->header.frame_id = mDepthOptFrameId;
    openniDepthMsg->height = depth.getHeight();
    openniDepthMsg->width = depth.getWidth();

    int num = 1; // for endianness detection
    openniDepthMsg->is_bigendian = !(*(char*)&num == 1);

    openniDepthMsg->step = openniDepthMsg->width * sizeof(uint16_t);
    openniDepthMsg->encoding = sensor_msgs::image_encodings::MONO16;

    size_t size = openniDepthMsg->step * openniDepthMsg->height;
    openniDepthMsg->data.resize(size);

    uint16_t* data = (uint16_t*)(&openniDepthMsg->data[0]);

    int dataSize = openniDepthMsg->width * openniDepthMsg->height;
    sl::float1* depthDataPtr = depth.getPtr<sl::float1>();

    for (int i = 0; i < dataSize; i++) {
        *(data++) = static_cast<uint16_t>(
            std::round(*(depthDataPtr++) * 1000)); // in mm, rounded
    }

    RCLCPP_DEBUG_STREAM(get_logger(), "Publishing OPENNI DEPTH message");
    mPubDepth.publish(openniDepthMsg, mDepthCamInfoMsg);
}

void ZedCamera::publishDisparity(sl::Mat disparity, rclcpp::Time t)
{
    sl::CameraInformation zedParam = mZed.getCameraInformation(mMatResolDepth);

    std::shared_ptr<sensor_msgs::msg::Image> disparity_image = sl_tools::imageToROSmsg(disparity, mDepthOptFrameId, t);

    dispMsgPtr disparityMsg = std::make_unique<stereo_msgs::msg::DisparityImage>();
    disparityMsg->image = *disparity_image.get();
    disparityMsg->header = disparityMsg->image.header;
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    disparityMsg->f = zedParam.calibration_parameters.left_cam.fx;
    disparityMsg->t = zedParam.calibration_parameters.T.x;
#else
    disparityMsg->f = zedParam.camera_configuration.calibration_parameters.left_cam.fx;
    disparityMsg->t = zedParam.camera_configuration.calibration_parameters.getCameraBaseline();
#endif
    disparityMsg->min_disparity = disparityMsg->f * disparityMsg->t / mZed.getInitParameters().depth_minimum_distance;
    disparityMsg->max_disparity = disparityMsg->f * disparityMsg->t / mZed.getInitParameters().depth_maximum_distance;

    RCLCPP_DEBUG_STREAM(get_logger(), "Publishing DISPARITY message");
    mPubDisparity->publish(std::move(disparityMsg));
}

void ZedCamera::publishPointCloud()
{
    sl_tools::StopWatch elabTimer;

    pointcloudMsgPtr pcMsg = std::make_unique<sensor_msgs::msg::PointCloud2>();

    // Initialize Point Cloud message
    // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h

    int width = mMatResolDepth.width;
    int height = mMatResolDepth.height;

    int ptsCount = width * height;

    if (!mSvoMode) {
        pcMsg->header.stamp = sl_tools::slTime2Ros(mMatCloud.timestamp);
    } else {
        pcMsg->header.stamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::CURRENT));
        ;
    }

    if (pcMsg->width != width || pcMsg->height != height) {
        pcMsg->header.frame_id = mPointCloudFrameId; // Set the header values of the ROS message

        pcMsg->is_bigendian = false;
        pcMsg->is_dense = false;

        pcMsg->width = width;
        pcMsg->height = height;

        sensor_msgs::PointCloud2Modifier modifier(*(pcMsg.get()));
        modifier.setPointCloud2Fields(4,
            "x",
            1,
            sensor_msgs::msg::PointField::FLOAT32,
            "y",
            1,
            sensor_msgs::msg::PointField::FLOAT32,
            "z",
            1,
            sensor_msgs::msg::PointField::FLOAT32,
            "rgb",
            1,
            sensor_msgs::msg::PointField::FLOAT32);
    }

    sl::Vector4<float>* cpu_cloud = mMatCloud.getPtr<sl::float4>();

    // Data copy
    float* ptCloudPtr = (float*)(&pcMsg->data[0]);
    memcpy(ptCloudPtr, (float*)cpu_cloud, ptsCount * 4 * sizeof(float));

    // Pointcloud publishing
    RCLCPP_DEBUG_STREAM(get_logger(), "Publishing POINT CLOUD message");
    mPubCloud->publish(std::move(pcMsg));

    // Publish freq calculation
    static sl_tools::StopWatch freqTimer;
    double mean = mPcPeriodMean_sec->addValue(freqTimer.toc());
    freqTimer.tic();

    // Point cloud elaboration time
    mPcProcMean_sec->addValue(elabTimer.toc());
    // RCLCPP_INFO_STREAM(get_logger(), "Point cloud freq: " << 1e6/mean);
}

void ZedCamera::callback_pubFusedPc()
{
    RCLCPP_DEBUG_ONCE(get_logger(), "Mapping callback called");

    static rclcpp::Time prev_ts = TIMEZERO_ROS;

    pointcloudMsgPtr pointcloudFusedMsg = std::make_unique<sensor_msgs::msg::PointCloud2>();

    uint32_t fusedCloudSubnumber = 0;
    try {
        fusedCloudSubnumber = count_subscribers(mPubFusedCloud->get_topic_name());
    } catch (...) {
        rcutils_reset_error();
        RCLCPP_DEBUG(get_logger(),
            "pubFusedPc: Exception while counting subscribers");
        return;
    }

    if (fusedCloudSubnumber == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(mCloseZedMutex);

    if (!mZed.isOpened()) {
        return;
    }

    mZed.requestSpatialMapAsync();

    while (mZed.getSpatialMapRequestStatusAsync() == sl::ERROR_CODE::FAILURE) {
        // Mesh is still generating
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    sl::ERROR_CODE res = mZed.retrieveSpatialMapAsync(mFusedPC);

    if (res != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
            get_logger(),
            "Fused point cloud not extracted: " << sl::toString(res).c_str());
        return;
    }

    size_t ptsCount = mFusedPC.getNumberOfPoints();
    bool resized = false;

    if (pointcloudFusedMsg->width != ptsCount || pointcloudFusedMsg->height != 1) {
        // Initialize Point Cloud message
        // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h
        pointcloudFusedMsg->header.frame_id = mMapFrameId; // Set the header values of the ROS message
        pointcloudFusedMsg->is_bigendian = false;
        pointcloudFusedMsg->is_dense = false;
        pointcloudFusedMsg->width = ptsCount;
        pointcloudFusedMsg->height = 1;

        sensor_msgs::PointCloud2Modifier modifier(*pointcloudFusedMsg);
        modifier.setPointCloud2Fields(4,
            "x",
            1,
            sensor_msgs::msg::PointField::FLOAT32,
            "y",
            1,
            sensor_msgs::msg::PointField::FLOAT32,
            "z",
            1,
            sensor_msgs::msg::PointField::FLOAT32,
            "rgb",
            1,
            sensor_msgs::msg::PointField::FLOAT32);

        resized = true;
    }

    int index = 0;
    float* ptCloudPtr = (float*)(&pointcloudFusedMsg->data[0]);
    int updated = 0;

    for (int c = 0; c < mFusedPC.chunks.size(); c++) {
        if (mFusedPC.chunks[c].has_been_updated || resized) {
            updated++;
            size_t chunkSize = mFusedPC.chunks[c].vertices.size();

            if (chunkSize > 0) {
                float* cloud_pts = (float*)(mFusedPC.chunks[c].vertices.data());
                memcpy(ptCloudPtr, cloud_pts, 4 * chunkSize * sizeof(float));
                ptCloudPtr += 4 * chunkSize;
                pointcloudFusedMsg->header.stamp = sl_tools::slTime2Ros(mFusedPC.chunks[c].timestamp);
            }
        } else {
            index += mFusedPC.chunks[c].vertices.size();
        }
    }

    rclcpp::Time ros_ts = get_clock()->now();

    // RCLCPP_INFO_STREAM(get_logger(), "callback_pubFusedPc - prev_ts type:" <<
    // prev_ts.get_clock_type());

    if (prev_ts != TIMEZERO_ROS) {
        double mean = mPubFusedCloudPeriodMean_sec->addValue((ros_ts - prev_ts).seconds());
        // RCLCPP_INFO_STREAM( get_logger(),"Fused Cloud Pub freq: " << 1.0/mean );
    }
    prev_ts = ros_ts;

    // Pointcloud publishing
    RCLCPP_DEBUG_STREAM(get_logger(), "Publishing FUSED POINT CLOUD message");
    mPubFusedCloud->publish(std::move(pointcloudFusedMsg));
}

void ZedCamera::callback_pubPaths()
{
    uint32_t mapPathSub = 0;
    uint32_t odomPathSub = 0;

    try {
        mapPathSub = count_subscribers(mMapPathTopic);
        odomPathSub = count_subscribers(mOdomPathTopic);
    } catch (...) {
        rcutils_reset_error();
        RCLCPP_DEBUG(get_logger(),
            "pubPaths: Exception while counting subscribers");
        return;
    }

    geometry_msgs::msg::PoseStamped odomPose;
    geometry_msgs::msg::PoseStamped mapPose;

    odomPose.header.stamp = mFrameTimestamp;
    odomPose.header.frame_id = mMapFrameId; // map_frame
    odomPose.pose.position.x = mOdom2BaseTransf.getOrigin().x();
    odomPose.pose.position.y = mOdom2BaseTransf.getOrigin().y();
    odomPose.pose.position.z = mOdom2BaseTransf.getOrigin().z();
    odomPose.pose.orientation.x = mOdom2BaseTransf.getRotation().x();
    odomPose.pose.orientation.y = mOdom2BaseTransf.getRotation().y();
    odomPose.pose.orientation.z = mOdom2BaseTransf.getRotation().z();
    odomPose.pose.orientation.w = mOdom2BaseTransf.getRotation().w();

    mapPose.header.stamp = mFrameTimestamp;
    mapPose.header.frame_id = mMapFrameId; // map_frame
    mapPose.pose.position.x = mMap2BaseTransf.getOrigin().x();
    mapPose.pose.position.y = mMap2BaseTransf.getOrigin().y();
    mapPose.pose.position.z = mMap2BaseTransf.getOrigin().z();
    mapPose.pose.orientation.x = mMap2BaseTransf.getRotation().x();
    mapPose.pose.orientation.y = mMap2BaseTransf.getRotation().y();
    mapPose.pose.orientation.z = mMap2BaseTransf.getRotation().z();
    mapPose.pose.orientation.w = mMap2BaseTransf.getRotation().w();

    // Circular vector
    if (mPathMaxCount != -1) {
        if (mOdomPath.size() == mPathMaxCount) {
            RCLCPP_DEBUG(get_logger(), "Path vectors full: rotating ");
            std::rotate(mOdomPath.begin(), mOdomPath.begin() + 1, mOdomPath.end());
            std::rotate(mMapPath.begin(), mMapPath.begin() + 1, mMapPath.end());

            mMapPath[mPathMaxCount - 1] = mapPose;
            mOdomPath[mPathMaxCount - 1] = odomPose;
        } else {
            // RCLCPP_DEBUG(get_logger(), "Path vectors adding last available poses");
            mMapPath.push_back(mapPose);
            mOdomPath.push_back(odomPose);
        }
    } else {
        // RCLCPP_DEBUG(get_logger(), "No limit path vectors, adding last available
        // poses");
        mMapPath.push_back(mapPose);
        mOdomPath.push_back(odomPose);
    }

    if (mapPathSub > 0) {
        pathMsgPtr mapPath = std::make_unique<nav_msgs::msg::Path>();
        mapPath->header.frame_id = mMapFrameId;
        mapPath->header.stamp = mFrameTimestamp;
        mapPath->poses = mMapPath;

        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing MAP PATH message");
        mPubPosePath->publish(std::move(mapPath));
    }

    if (odomPathSub > 0) {
        pathMsgPtr odomPath = std::make_unique<nav_msgs::msg::Path>();
        odomPath->header.frame_id = mMapFrameId;
        odomPath->header.stamp = mFrameTimestamp;
        odomPath->poses = mOdomPath;

        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing ODOM PATH message");
        mPubOdomPath->publish(std::move(odomPath));
    }
}

void ZedCamera::callback_resetOdometry(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res)
{
    (void)request_header;
    (void)req;

    RCLCPP_INFO(get_logger(), "** Reset Odometry service called **");
    mResetOdom = true;
    res->message = "Odometry reset OK";
    res->success = true;
}

void ZedCamera::callback_resetPosTracking(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res)
{
    (void)request_header;
    (void)req;

    RCLCPP_INFO(get_logger(), "** Reset Pos. Tracking service called **");

    if (!mPosTrackingStarted) {
        RCLCPP_WARN(get_logger(), "Pos. Tracking was not started");
        res->message = "Positional tracking not active";
        res->success = false;
        return;
    }

    /*if (!setPose(mInitialBasePose[0],
            mInitialBasePose[1],
            mInitialBasePose[2],
            mInitialBasePose[3],
            mInitialBasePose[4],
            mInitialBasePose[5])) {
        res->message = "Error setting initial pose";
        RCLCPP_WARN(get_logger(), "Error setting initial pose");
        res->success = false;
        return;
    }*/

    std::lock_guard<std::mutex> lock(mPosTrkMutex);

    //mInitOdomWithPose = true;

    // Disable tracking
    //mPosTrackingStarted = false;
    //mZed.disablePositionalTracking();

    // Restart tracking
    startPosTracking();

    res->message = "Positional tracking reset OK";
    res->success = true;
}

void ZedCamera::callback_setPose(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_interfaces::srv::SetPose_Request> req,
    std::shared_ptr<zed_interfaces::srv::SetPose_Response> res)
{
    (void)request_header;

    RCLCPP_INFO(get_logger(), "** Set Pose service called **");
    RCLCPP_INFO_STREAM(get_logger(),
        "New pose: [" << req->pos[0] << "," << req->pos[1] << ","
                      << req->pos[2] << ", " << req->orient[0]
                      << "," << req->orient[1] << ","
                      << req->orient[2] << "]");

    if (!mPosTrackingStarted) {
        RCLCPP_WARN(get_logger(), "Pos. Tracking was not active");
        res->message = "Positional tracking not active";
        res->success = false;
        return;
    }

    //mInitOdomWithPose = true;

    mInitialBasePose[0] = req->pos[0];
    mInitialBasePose[1] = req->pos[1];
    mInitialBasePose[2] = req->pos[2];

    mInitialBasePose[3] = req->orient[0];
    mInitialBasePose[4] = req->orient[1];
    mInitialBasePose[5] = req->orient[2];

    /*if (!setPose(mInitialBasePose[0],
            mInitialBasePose[1],
            mInitialBasePose[2],
            mInitialBasePose[3],
            mInitialBasePose[4],
            mInitialBasePose[5])) {
        res->message = "Error setting initial pose";
        RCLCPP_WARN(get_logger(), "Error setting initial pose");
        res->success = false;
        return;
    }*/

    //std::lock_guard<std::mutex> lock(mPosTrkMutex);

    // Disable tracking
    //mPosTrackingStarted = false;
    //mZed.disablePositionalTracking();

    // Restart tracking
    startPosTracking();

    res->message = "Positional Tracking new pose OK";
    res->success = true;
}

void ZedCamera::callback_enableObjDet(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
    std::shared_ptr<std_srvs::srv::SetBool_Response> res)
{
    (void)request_header;

    RCLCPP_INFO(get_logger(), "** Enable Object Detection service called **");

    std::lock_guard<std::mutex> lock(mObjDetMutex);

    if (mCamRealModel == sl::MODEL::ZED || mCamRealModel == sl::MODEL::ZED_M) {
        RCLCPP_WARN(get_logger(),
            "Object Detection not available for ZED or ZED Mini");
        res->message = "Object Detection not available for ZED or ZED Mini";
        res->success = false;
        return;
    }

    if (req->data) {
        RCLCPP_INFO(get_logger(), "Starting Object Detection");
        // Start
        if (mObjDetEnabled && mObjDetRunning) {
            RCLCPP_WARN(get_logger(), "Object Detection is just running");
            res->message = "Object Detection is just running";
            res->success = false;
            return;
        }

        mObjDetEnabled = true;

        if (startObjDetect()) {
            res->message = "Object Detection started";
            res->success = true;
            return;
        } else {
            res->message = "Error occurred starting Object Detection. See log for more info";
            res->success = false;
            return;
        }
    } else {
        RCLCPP_INFO(get_logger(), "Stopping Object Detection");
        // Stop
        if (!mObjDetEnabled || !mObjDetRunning) {
            RCLCPP_WARN(get_logger(), "Object Detection was not running");
            res->message = "Object Detection was not running";
            res->success = false;
            return;
        }

        stopObjDetect();

        res->message = "Object Detection stopped";
        res->success = true;
        return;
    }
}

void ZedCamera::callback_enableMapping(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
    std::shared_ptr<std_srvs::srv::SetBool_Response> res)
{
    (void)request_header;

    RCLCPP_INFO(get_logger(), "** Enable Spatial Mapping service called **");

    std::lock_guard<std::mutex> lock(mMappingMutex);

    if (req->data) {
        RCLCPP_INFO(get_logger(), "Starting Spatial Mapping");
        // Start
        if (mMappingEnabled && mMappingRunning) {
            RCLCPP_WARN(get_logger(), "Spatial Mapping is just running");
            res->message = "Spatial Mapping is just running";
            res->success = false;
            return;
        }

        mMappingEnabled = true;

        if (start3dMapping()) {
            res->message = "Spatial Mapping started";
            res->success = true;
            return;
        } else {
            res->message = "Error occurred starting Spatial Mapping. See log for more info";
            res->success = false;
            return;
        }
    } else {
        RCLCPP_INFO(get_logger(), "Stopping Spatial Mapping");
        // Stop
        if (!mMappingEnabled || !mMappingRunning) {
            RCLCPP_WARN(get_logger(), "Spatial Mapping was not running");
            res->message = "Spatial Mapping was not running";
            res->success = false;
            return;
        }

        stop3dMapping();

        res->message = "Spatial Mapping stopped";
        res->success = true;
        return;
    }
}

void ZedCamera::callback_startSvoRec(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_interfaces::srv::StartSvoRec_Request> req,
    std::shared_ptr<zed_interfaces::srv::StartSvoRec_Response> res)
{
    (void)request_header;

    RCLCPP_INFO(get_logger(), "** Start SVO Recording service called **");

    if (mSvoMode) {
        RCLCPP_WARN(get_logger(),
            "Cannot start SVO recording while playing SVO as input");
        res->message = "Cannot start SVO recording while playing SVO as input";
        res->success = false;
        return;
    }

    std::lock_guard<std::mutex> lock(mRecMutex);

    if (mRecording) {
        RCLCPP_WARN(get_logger(), "SVO Recording is just enabled");
        res->message = "SVO Recording is just enabled";
        res->success = false;
        return;
    }

    mSvoRecBitrate = req->bitrate;
    if ((mSvoRecBitrate != 0) && ((mSvoRecBitrate < 1000) || (mSvoRecBitrate > 60000))) {
        RCLCPP_WARN(get_logger(),
            "'bitrate' value not valid. Please use a value "
            "in range [1000,60000], or 0 for default");
        res->message = "'bitrate' value not valid. Please use a value in range "
                       "[1000,60000], or 0 for default";
        res->success = false;
        return;
    }
    mSvoRecCompr = static_cast<sl::SVO_COMPRESSION_MODE>(req->compression_mode);
    if (mSvoRecCompr >= sl::SVO_COMPRESSION_MODE::LAST) {
        RCLCPP_WARN(
            get_logger(),
            "'compression_mode' mode not valid. Please use a value in range [0,2]");
        res->message = "'compression_mode' mode not valid. Please use a value in range [0,2]";
        res->success = false;
        return;
    }
    mSvoRecFramerate = req->target_framerate;
    mSvoRecTranscode = req->input_transcode;
    mSvoRecFilename = req->svo_filename;

    if (mSvoRecFilename.empty()) {
        mSvoRecFilename = "zed.svo";
    }

    std::string err;

    if (!startSvoRecording(err)) {
        res->message = "Error starting SVO recording: " + err;
        res->success = false;
        return;
    }

    RCLCPP_INFO(get_logger(), "SVO Recording started: ");
    RCLCPP_INFO_STREAM(get_logger(), " * Bitrate: " << mSvoRecBitrate);
    RCLCPP_INFO_STREAM(get_logger(), " * Compression: " << mSvoRecCompr);
    RCLCPP_INFO_STREAM(get_logger(), " * Framerate: " << mSvoRecFramerate);
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * Input Transcode: " << (mSvoRecTranscode ? "TRUE" : "FALSE"));
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * Filename: " << (mSvoRecFilename.empty() ? "zed.svo" : mSvoRecFilename));

    res->message = "SVO Recording started";
    res->success = true;
}

void ZedCamera::callback_stopSvoRec(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res)
{
    (void)request_header;
    (void)req;

    RCLCPP_INFO(get_logger(), "** Stop SVO Recording service called **");

    std::lock_guard<std::mutex> lock(mRecMutex);

    if (!mRecording) {
        RCLCPP_WARN(get_logger(), "SVO Recording is NOT enabled");
        res->message = "SVO Recording is NOT enabled";
        res->success = false;
        return;
    }

    stopSvoRecording();

    RCLCPP_WARN(get_logger(), "SVO Recording stopped");
    res->message = "SVO Recording stopped";
    res->success = true;
}

void ZedCamera::callback_pauseSvoInput(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res)
{
    (void)request_header;

    RCLCPP_INFO(get_logger(), "** Pause SVO Input service called **");

    std::lock_guard<std::mutex> lock(mRecMutex);

    if (!mSvoMode) {
        RCLCPP_WARN(get_logger(), "The node is not using an SVO as input");
        res->message = "The node is not using an SVO as inpu";
        res->success = false;
        return;
    }

    if (mSvoRealtime) {
        RCLCPP_WARN(get_logger(),
            "SVO input can be paused only if SVO is not in RealTime mode");
        res->message = "SVO input can be paused only if SVO is not in RealTime mode";
        res->success = false;
        mSvoPause = false;
        return;
    }

    if (!mSvoPause) {
        RCLCPP_WARN(get_logger(), "SVO is paused");
        res->message = "SVO is paused";
        mSvoPause = true;
    } else {
        RCLCPP_WARN(get_logger(), "SVO is playing");
        res->message = "SVO is playing";
        mSvoPause = false;
    }
    res->success = true;
}

void ZedCamera::callback_updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (mConnStatus != sl::ERROR_CODE::SUCCESS) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
            sl::toString(mConnStatus).c_str());
        return;
    }

    // if (mGrabActive)
    {
        if (mGrabStatus == sl::ERROR_CODE::SUCCESS /*|| mGrabStatus == sl::ERROR_CODE::NOT_A_NEW_FRAME*/) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                "Camera grabbing");

            double freq = 1. / mGrabPeriodMean_sec->getMean();
            double freq_perc = 100. * freq / mCamGrabFrameRate;
            stat.addf("Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
            stat.addf("Capture",
                "Tot. Processing Time: %.3f sec (Max. %.3f sec)",
                mElabPeriodMean_sec->getMean(),
                1. / mCamGrabFrameRate);

            if (mPublishingData) {
                freq = 1. / mVideoDepthPeriodMean_sec->getMean();
                freq_perc = 100. * freq / mPubFrameRate;
                stat.addf(
                    "Video/Depth", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
                stat.addf("Video/Depth",
                    "Processing Time: %.3f sec (Max. %.3f sec)",
                    mVideoDepthElabMean_sec->getMean(),
                    1. / mCamGrabFrameRate);
            }

            if (mSvoMode) {
                int frame = mZed.getSVOPosition();
                int totFrames = mZed.getSVONumberOfFrames();
                double svo_perc = 100. * (static_cast<double>(frame) / totFrames);

                stat.addf(
                    "Playing SVO", "Frame: %d/%d (%.1f%%)", frame, totFrames, svo_perc);
            }

            if (isDepthRequired()) {
                stat.add("Depth status", "ACTIVE");

                if (mPcPublishing) {
                    double freq = 1. / mPcPeriodMean_sec->getMean();
                    double freq_perc = 100. * freq / mPcPubRate;
                    stat.addf(
                        "Point Cloud", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
                    stat.addf("Point Cloud",
                        "Processing Time: %.3f sec (Max. %.3f sec)",
                        mPcProcMean_sec->getMean(),
                        1. / mPcPubRate);
                } else {
                    stat.add("Point Cloud", "Topic not subscribed");
                }

                if (mFloorAlignment) {
                    if (mInitOdomWithPose) {
                        stat.add("Floor Detection", "NOT INITIALIZED");
                    } else {
                        stat.add("Floor Detection", "INITIALIZED");
                    }
                }

                if (mPosTrackingStarted) {
                    stat.addf(
                        "Tracking status", "%s", sl::toString(mPosTrackingStatus).c_str());
                } else {
                    stat.add("Pos. Tracking status", "INACTIVE");
                }

                if (mObjDetRunning) {
                    if (mObjDetSubscribed) {
                        double freq = 1. / mObjDetPeriodMean_sec->getMean();
                        double freq_perc = 100. * freq / mPubFrameRate;
                        stat.addf("Object detection",
                            "Mean Frequency: %.3f Hz  (%.1f%%)",
                            freq,
                            freq_perc);
                        stat.addf("Object detection",
                            "Processing Time: %.3f sec (Max. %.3f sec)",
                            mObjDetElabMean_sec->getMean(),
                            1. / mCamGrabFrameRate);
                    } else {
                        stat.add("Object Detection", "Active, topic not subscribed");
                    }
                } else {
                    stat.add("Object Detection", "INACTIVE");
                }
            } else {
                stat.add("Depth status", "INACTIVE");
            }
        } else {
            stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                "Camera error: %s",
                sl::toString(mGrabStatus).c_str());
        }
    }
    /*else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Waiting for data
  subscriber"); stat.add("Capture", "INACTIVE");
  }*/

    if (mImuPublishing) {
        double freq = 1. / mImuPeriodMean_sec->getMean();
        stat.addf("IMU", "Mean Frequency: %.1f Hz", freq);
    } else {
        stat.add("IMU Sensor", "Topics not subscribed");
    }

    if (mMagPublishing) {
        double freq = 1. / mMagPeriodMean_sec->getMean();
        stat.addf("Magnetometer", "Mean Frequency: %.1f Hz", freq);
    } else {
        stat.add("Magnetometer Sensor", "Topics not subscribed");
    }

    if (mBaroPublishing) {
        double freq = 1. / mBaroPeriodMean_sec->getMean();
        stat.addf("Barometer", "Mean Frequency: %.1f Hz", freq);
    } else {
        stat.add("Barometer Sensor", "Topics not subscribed");
    }

    if (sl_tools::isZED2OrZED2i(mCamRealModel)) {
        stat.addf("Left CMOS Temp.", "%.1f °C", mTempLeft);
        stat.addf("Right CMOS Temp.", "%.1f °C", mTempRight);

        if (mTempLeft > 70.f || mTempRight > 70.f) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                "Camera temperature");
        }
    } else {
        stat.add("Left CMOS Temp.", "N/A");
        stat.add("Right CMOS Temp.", "N/A");
    }

    if (mRecording) {
        if (!mRecStatus.status) {
            // if (mGrabActive)
            {
                stat.add("SVO Recording", "ERROR");
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                    "Error adding frames to SVO file while recording. "
                    "Check "
                    "free disk space");
            }
            /*else
      {
        stat.add("SVO Recording", "WAITING");
      }*/
        } else {
            stat.add("SVO Recording", "ACTIVE");
            stat.addf(
                "SVO compression time", "%g msec", mRecStatus.average_compression_time);
            stat.addf("SVO compression ratio",
                "%.1f%%",
                mRecStatus.average_compression_ratio);
        }
    } else {
        stat.add("SVO Recording", "NOT ACTIVE");
    }
}

void ZedCamera::callback_clickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // ----> Check for result subscribers
    size_t markerSubNumber = 0;
    size_t planeSubNumber = 0;
    try {
        markerSubNumber = count_subscribers(mPubMarker->get_topic_name());
        planeSubNumber = count_subscribers(mPubPlane->get_topic_name());
    } catch (...) {
        rcutils_reset_error();
        RCLCPP_DEBUG(get_logger(),
            "threadFunc_zedGrab: Exception while "
            "counting point plane subscribers");
        return;
    }

    if ((markerSubNumber + planeSubNumber) == 0) {
        return;
    }
    // <---- Check for result subscribers

    rclcpp::Time ts = get_clock()->now();

    float X = msg->point.x;
    float Y = msg->point.y;
    float Z = msg->point.z;

    RCLCPP_INFO_STREAM(get_logger(), "Clicked 3D point [X FW, Y LF, Z UP]: [" << X << "," << Y << "," << Z << "]");

    // ----> Transform the point from `map` frame to `left_camera_optical_frame`
    double camX, camY, camZ;
    try {
        // Save the transformation
        geometry_msgs::msg::TransformStamped m2o = mTfBuffer->lookupTransform(mLeftCamOptFrameId, msg->header.frame_id, TIMEZERO_SYS, rclcpp::Duration(0.1));
        
        RCLCPP_INFO(get_logger(), "'%s' -> '%s': {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f,%.3f}", msg->header.frame_id.c_str(), mLeftCamOptFrameId.c_str(),
            m2o.transform.translation.x, m2o.transform.translation.y, m2o.transform.translation.z,
            m2o.transform.rotation.x, m2o.transform.rotation.y, m2o.transform.rotation.z, m2o.transform.rotation.w);

        // Get the TF2 transformation
        geometry_msgs::msg::PointStamped ptCam;

        tf2::doTransform(*(msg.get()), ptCam, m2o);

        camX = ptCam.point.x;
        camY = ptCam.point.y;
        camZ = ptCam.point.z;

        RCLCPP_INFO(get_logger(), "Point in camera coordinates [Z FW, X RG, Y DW]: {%.3f,%.3f,%.3f}", camX, camY, camZ);
    } catch (tf2::TransformException& ex) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_DEBUG_THROTTLE(
            get_logger(), steady_clock, 1.0, "Transform error: %s", ex.what());
        RCLCPP_WARN_THROTTLE(get_logger(),
            steady_clock,
            1.0,
            "The tf from '%s' to '%s' is not available.",
            msg->header.frame_id.c_str(),
            mLeftCamOptFrameId.c_str());

        return;
    }
    // <---- Transform the point from `map` frame to `left_camera_optical_frame`

    // ----> Project the point into 2D image coordinates
    sl::CalibrationParameters zedParam;
    zedParam = mZed.getCameraInformation(mMatResolVideo).calibration_parameters; // ok

    float f_x = zedParam.left_cam.fx;
    float f_y = zedParam.left_cam.fy;
    float c_x = zedParam.left_cam.cx;
    float c_y = zedParam.left_cam.cy;

    float u = ((camX / camZ) * f_x + c_x) / mImgDownsampleFactor;
    float v = ((camY / camZ) * f_y + c_y) / mImgDownsampleFactor;
    RCLCPP_INFO_STREAM(get_logger(), "Clicked point image coordinates: [" << u << "," << v << "]");
    // <---- Project the point into 2D image coordinates

    // ----> Extract plane from clicked point
    sl::Plane plane;
    sl::ERROR_CODE err = mZed.findPlaneAtHit(sl::uint2(u, v), plane);
    if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN(get_logger(), "Error extracting plane at point [%.3f,%.3f,%.3f]: %s", X, Y, Z, sl::toString(err).c_str());
        return;
    }

    sl::float3 center = plane.getCenter();
    sl::float2 dims = plane.getExtents();

    if (dims[0] == 0 || dims[1] == 0) {
        RCLCPP_INFO(get_logger(), "Plane not found at point [%.3f,%.3f,%.3f]", X, Y, Z);
        return;
    }

    RCLCPP_INFO(get_logger(), "Found plane at point [%.3f,%.3f,%.3f] -> Center: [%.3f,%.3f,%.3f], Dims: %.3fx%.3f", X, Y, Z, center.x, center.y, center.z, dims[0], dims[1]);
    // <---- Extract plane from clicked point

    if (markerSubNumber > 0) {
        // ----> Publish a blue sphere in the clicked point        
        markerMsgPtr pt_marker = std::make_unique<visualization_msgs::msg::Marker>();
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        static int hit_pt_id
            = 0;
        pt_marker->header.stamp = ts;
        // Set the marker action.  Options are ADD and DELETE
        pt_marker->action = visualization_msgs::msg::Marker::ADD;
        pt_marker->lifetime = rclcpp::Duration(0);

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        pt_marker->ns = "plane_hit_points";
        pt_marker->id = hit_pt_id++;
        pt_marker->header.frame_id = mMapFrameId;

        // Set the marker type.
        pt_marker->type = visualization_msgs::msg::Marker::SPHERE;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        pt_marker->pose.position.x = X;
        pt_marker->pose.position.y = Y;
        pt_marker->pose.position.z = Z;
        pt_marker->pose.orientation.x = 0.0;
        pt_marker->pose.orientation.y = 0.0;
        pt_marker->pose.orientation.z = 0.0;
        pt_marker->pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        pt_marker->scale.x = 0.025;
        pt_marker->scale.y = 0.025;
        pt_marker->scale.z = 0.025;

        // Set the color -- be sure to set alpha to something non-zero!
        pt_marker->color.r = 0.2f;
        pt_marker->color.g = 0.1f;
        pt_marker->color.b = 0.75f;
        pt_marker->color.a = 0.8;

        // Publish the marker
        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing PT MARKER message");
        mPubMarker->publish(std::move(pt_marker));
        // ----> Publish a blue sphere in the clicked point

        // ----> Publish the plane as green mesh
        markerMsgPtr plane_marker = std::make_unique<visualization_msgs::msg::Marker>();
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        static int plane_mesh_id = 0;
        plane_marker->header.stamp = ts;
        // Set the marker action.  Options are ADD and DELETE
        plane_marker->action = visualization_msgs::msg::Marker::ADD;
        plane_marker->lifetime = rclcpp::Duration(0);

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        plane_marker->ns = "plane_meshes";
        plane_marker->id = plane_mesh_id++;
        plane_marker->header.frame_id = mLeftCamFrameId;

        // Set the marker type.
        plane_marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

        // Set the pose of the marker.  This isplane_marker
        plane_marker->pose.orientation.w = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        plane_marker->color.r = 0.10f;
        plane_marker->color.g = 0.75f;
        plane_marker->color.b = 0.20f;
        plane_marker->color.a = 0.75;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        plane_marker->scale.x = 1.0;
        plane_marker->scale.y = 1.0;
        plane_marker->scale.z = 1.0;

        sl::Mesh mesh = plane.extractMesh();
        size_t triangCount = mesh.getNumberOfTriangles();
        size_t ptCount = triangCount * 3;
        plane_marker->points.resize(ptCount);
        plane_marker->colors.resize(ptCount);

        size_t ptIdx = 0;
        for (size_t t = 0; t < triangCount; t++) {
            for (int p = 0; p < 3; p++) {
                uint vIdx = mesh.triangles[t][p];
                plane_marker->points[ptIdx].x = mesh.vertices[vIdx][0];
                plane_marker->points[ptIdx].y = mesh.vertices[vIdx][1];
                plane_marker->points[ptIdx].z = mesh.vertices[vIdx][2];

                // Set the color -- be sure to set alpha to something non-zero!
                plane_marker->colors[ptIdx].r = 0.10f;
                plane_marker->colors[ptIdx].g = 0.75f;
                plane_marker->colors[ptIdx].b = 0.20f;
                plane_marker->colors[ptIdx].a = 0.75;

                ptIdx++;
            }
        }

        // Publish the marker
        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing PLANE MARKER message");
        mPubMarker->publish(std::move(plane_marker));
        // <---- Publish the plane as green mesh
    }

    if (planeSubNumber > 0) {
        // ----> Publish the plane as custom message

        planeMsgPtr planeMsg = std::make_unique<zed_interfaces::msg::PlaneStamped>();
        planeMsg->header.stamp = ts;
        planeMsg->header.frame_id = mLeftCamFrameId;

        // Plane equation
        sl::float4 sl_coeff = plane.getPlaneEquation();
        planeMsg->coefficients.coef[0] = static_cast<double>(sl_coeff[0]);
        planeMsg->coefficients.coef[1] = static_cast<double>(sl_coeff[1]);
        planeMsg->coefficients.coef[2] = static_cast<double>(sl_coeff[2]);
        planeMsg->coefficients.coef[3] = static_cast<double>(sl_coeff[3]);

        // Plane Normal
        sl::float3 sl_normal = plane.getNormal();
        planeMsg->normal.x = sl_normal[0];
        planeMsg->normal.y = sl_normal[1];
        planeMsg->normal.z = sl_normal[2];

        // Plane Center
        sl::float3 sl_center = plane.getCenter();
        planeMsg->center.x = sl_center[0];
        planeMsg->center.y = sl_center[1];
        planeMsg->center.z = sl_center[2];

        // Plane extents
        sl::float3 sl_extents = plane.getExtents();
        planeMsg->extents[0] = sl_extents[0];
        planeMsg->extents[1] = sl_extents[1];

        // Plane pose
        sl::Pose sl_pose = plane.getPose();
        sl::Orientation sl_rot = sl_pose.getOrientation();
        sl::Translation sl_tr = sl_pose.getTranslation();

        planeMsg->pose.rotation.x = sl_rot.ox;
        planeMsg->pose.rotation.y = sl_rot.oy;
        planeMsg->pose.rotation.z = sl_rot.oz;
        planeMsg->pose.rotation.w = sl_rot.ow;

        planeMsg->pose.translation.x = sl_tr.x;
        planeMsg->pose.translation.y = sl_tr.y;
        planeMsg->pose.translation.z = sl_tr.z;

        // Plane Bounds
        std::vector<sl::float3> sl_bounds = plane.getBounds();
        planeMsg->bounds.points.resize(sl_bounds.size());
        memcpy(planeMsg->bounds.points.data(), sl_bounds.data(), 3 * sl_bounds.size() * sizeof(float));

        // Plane mesh
        sl::Mesh sl_mesh = plane.extractMesh();
        size_t triangCount = sl_mesh.triangles.size();
        size_t ptsCount = sl_mesh.vertices.size();
        planeMsg->mesh.triangles.resize(triangCount);
        planeMsg->mesh.vertices.resize(ptsCount);

        // memcpy not allowed because data types are different
        for (size_t i = 0; i < triangCount; i++) {
            planeMsg->mesh.triangles[i].vertex_indices[0] = sl_mesh.triangles[i][0];
            planeMsg->mesh.triangles[i].vertex_indices[1] = sl_mesh.triangles[i][1];
            planeMsg->mesh.triangles[i].vertex_indices[2] = sl_mesh.triangles[i][2];
        }

        // memcpy not allowed because data types are different
        for (size_t i = 0; i < ptsCount; i++) {
            planeMsg->mesh.vertices[i].x = sl_mesh.vertices[i][0];
            planeMsg->mesh.vertices[i].y = sl_mesh.vertices[i][1];
            planeMsg->mesh.vertices[i].z = sl_mesh.vertices[i][2];
        }

        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing PLANE message");
        mPubPlane->publish(std::move(planeMsg));
        // <---- Publish the plane as custom message
    }
}

} // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCamera)
