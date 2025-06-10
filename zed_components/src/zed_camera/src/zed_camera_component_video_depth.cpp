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

#include "zed_camera_component.hpp"
#include "sl_logging.hpp"
#include "sl_tools.hpp"

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace stereolabs
{
void ZedCamera::getVideoParams()
{
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "=== VIDEO parameters ===");

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  if (!sl_tools::isZEDX(mCamUserModel)) {
    sl_tools::getParam(
      shared_from_this(), "video.brightness", mCamBrightness,
      mCamBrightness, " * Brightness: ", true, 0, 8);
    sl_tools::getParam(
      shared_from_this(), "video.contrast", mCamContrast,
      mCamContrast, " * Contrast: ", true, 0, 8);
    sl_tools::getParam(
      shared_from_this(), "video.hue", mCamHue, mCamHue,
      " * Hue: ", true, 0, 11);
  }

  sl_tools::getParam(
    shared_from_this(), "video.saturation", mCamSaturation,
    mCamSaturation, " * Saturation: ", true, 0, 8);
  sl_tools::getParam(
    shared_from_this(), "video.sharpness", mCamSharpness,
    mCamSharpness, " * Sharpness: ", true, 0, 8);
  sl_tools::getParam(
    shared_from_this(), "video.gamma", mCamGamma, mCamGamma,
    " * Gamma: ", true, 1, 9);
  sl_tools::getParam(
    shared_from_this(), "video.auto_exposure_gain",
    mCamAutoExpGain, mCamAutoExpGain,
    " * Auto Exposure/Gain: ", true);
  if (mCamAutoExpGain) {
    mTriggerAutoExpGain = true;
  }
  sl_tools::getParam(
    shared_from_this(), "video.exposure", mCamExposure,
    mCamExposure, " * Exposure: ", true, 0, 100);
  sl_tools::getParam(
    shared_from_this(), "video.gain", mCamGain, mCamGain,
    " * Gain: ", true, 0, 100);
  sl_tools::getParam(
    shared_from_this(), "video.auto_whitebalance", mCamAutoWB,
    mCamAutoWB, " * Auto White Balance: ", true);
  if (mCamAutoWB) {
    mTriggerAutoWB = true;
  }
  int wb = 42;
  sl_tools::getParam(
    shared_from_this(), "video.whitebalance_temperature", wb,
    wb, " * White Balance Temperature: ", true, 28, 65);
  mCamWBTemp = wb * 100;

  if (sl_tools::isZEDX(mCamUserModel)) {
    sl_tools::getParam(
      shared_from_this(), "video.exposure_time", mGmslExpTime,
      mGmslExpTime, " * ZED X Exposure time: ", true, 28,
      66000);
    sl_tools::getParam(
      shared_from_this(), "video.auto_exposure_time_range_min",
      mGmslAutoExpTimeRangeMin, mGmslAutoExpTimeRangeMin,
      " * ZED X Auto Exp. time range min: ", true, 28, 66000);
    sl_tools::getParam(
      shared_from_this(), "video.auto_exposure_time_range_max",
      mGmslAutoExpTimeRangeMax, mGmslAutoExpTimeRangeMax,
      " * ZED X Auto Exp. time range max: ", true, 28, 66000);
    sl_tools::getParam(
      shared_from_this(), "video.exposure_compensation",
      mGmslExposureComp, mGmslExposureComp,
      " * ZED X Exposure comp.: ", true, 0, 100);
    sl_tools::getParam(
      shared_from_this(), "video.analog_gain", mGmslAnalogGain,
      mGmslAnalogGain, " * ZED X Analog Gain: ", true, 1000,
      16000);
    sl_tools::getParam(
      shared_from_this(), "video.auto_analog_gain_range_min",
      mGmslAnalogGainRangeMin, mGmslAnalogGainRangeMin,
      " * ZED X Auto Analog Gain range min: ", true, 1000,
      16000);
    sl_tools::getParam(
      shared_from_this(), "video.auto_analog_gain_range_max",
      mGmslAnalogGainRangeMax, mGmslAnalogGainRangeMax,
      " * ZED X Auto Analog Gain range max: ", true, 1000,
      16000);
    sl_tools::getParam(
      shared_from_this(), "video.digital_gain",
      mGmslDigitalGain, mGmslDigitalGain,
      " * ZED X Digital Gain: ", true, 1, 256);
    sl_tools::getParam(
      shared_from_this(), "video.auto_digital_gain_range_min",
      mGmslAutoDigitalGainRangeMin,
      mGmslAutoDigitalGainRangeMin,
      " * ZED X Auto Digital Gain range min: ", true, 1, 256);
    sl_tools::getParam(
      shared_from_this(), "video.auto_digital_gain_range_max",
      mGmslAutoDigitalGainRangeMax,
      mGmslAutoDigitalGainRangeMax,
      " * ZED X Auto Digital Gain range max: ", true, 1, 256);
    sl_tools::getParam(
      shared_from_this(), "video.denoising", mGmslDenoising,
      mGmslDenoising,
      " * ZED X Auto Digital Gain range max: ", true, 0, 100);
  }
}

void ZedCamera::getDepthParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "=== DEPTH parameters ===");

  std::string depth_mode_str = sl::toString(mDepthMode).c_str();
  sl_tools::getParam(
    shared_from_this(), "depth.depth_mode", depth_mode_str,
    depth_mode_str);

  bool matched = false;
  for (int mode = static_cast<int>(sl::DEPTH_MODE::NONE);
    mode < static_cast<int>(sl::DEPTH_MODE::LAST); ++mode)
  {
    std::string test_str =
      sl::toString(static_cast<sl::DEPTH_MODE>(mode)).c_str();
    std::replace(
      test_str.begin(), test_str.end(), ' ',
      '_');    // Replace spaces with underscores to match the YAML setting
    if (test_str == depth_mode_str) {
      matched = true;
      mDepthMode = static_cast<sl::DEPTH_MODE>(mode);
      break;
    }
  }

  if (!matched) {
    mDepthMode = sl::DEPTH_MODE::PERFORMANCE;
    if (depth_mode_str != "NEURAL_LIGHT") {
      RCLCPP_WARN(
        get_logger(),
        "The parameter 'depth.depth_mode' contains a not valid string. "
        "Please check it in 'common_stereo.yaml'.");
      RCLCPP_WARN_STREAM(get_logger(), "Using default value: " << sl::toString(mDepthMode).c_str());
    }
  }

  if (mDepthMode == sl::DEPTH_MODE::NONE) {
    mDepthDisabled = true;
    mDepthStabilization = 0;
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Depth mode: " << sl::toString(mDepthMode).c_str()
                        << " - DEPTH DISABLED");
  } else {
    mDepthDisabled = false;
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Depth mode: " << sl::toString(mDepthMode).c_str()
                        << " [" << static_cast<int>(mDepthMode)
                        << "]");
  }

  if (!mDepthDisabled) {
    sl_tools::getParam(
      shared_from_this(), "depth.min_depth", mCamMinDepth,
      mCamMinDepth, " * Min depth [m]: ", false, 0.1, 100.0);
    sl_tools::getParam(
      shared_from_this(), "depth.max_depth", mCamMaxDepth,
      mCamMaxDepth, " * Max depth [m]: ", false, 0.1, 100.0);

    sl_tools::getParam(
      shared_from_this(), "depth.depth_stabilization",
      mDepthStabilization, mDepthStabilization,
      " * Depth Stabilization: ", false, 0, 100);


    sl_tools::getParam(
      shared_from_this(), "depth.openni_depth_mode",
      mOpenniDepthMode, mOpenniDepthMode,
      " * OpenNI mode (16bit point cloud): ");

    sl_tools::getParam(
      shared_from_this(), "depth.point_cloud_freq", mPcPubRate,
      mPcPubRate, "", true, 0.1, static_cast<double>(mCamGrabFrameRate));
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Point cloud rate [Hz]: " << mPcPubRate);

    std::string out_resol = "COMPACT";
    sl_tools::getParam(
      shared_from_this(), "depth.point_cloud_res", out_resol,
      out_resol);
    if (out_resol == toString(PcRes::PUB)) {
      mPcResolution = PcRes::PUB;
    } else if (out_resol == toString(PcRes::FULL)) {
      mPcResolution = PcRes::FULL;
    } else if (out_resol == toString(PcRes::COMPACT)) {
      mPcResolution = PcRes::COMPACT;
    } else if (out_resol == toString(PcRes::REDUCED)) {
      mPcResolution = PcRes::REDUCED;
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Not valid 'depth.point_cloud_res' value: '%s'. Using default "
        "setting instead.",
        out_resol.c_str());
      out_resol = "COMPACT -> Fix the value in YAML!";
      mPcResolution = PcRes::COMPACT;
    }
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Point cloud resolution: " << out_resol.c_str());

    sl_tools::getParam(
      shared_from_this(), "depth.depth_confidence", mDepthConf,
      mDepthConf, " * Depth Confidence: ", true, 0, 100);
    sl_tools::getParam(
      shared_from_this(), "depth.depth_texture_conf",
      mDepthTextConf, mDepthTextConf,
      " * Depth Texture Confidence: ", true, 0, 100);
    sl_tools::getParam(
      shared_from_this(), "depth.remove_saturated_areas",
      mRemoveSatAreas, mRemoveSatAreas,
      " * Remove saturated areas: ", true);
    // ------------------------------------------
  }
}

void ZedCamera::fillCamInfo(
  const std::shared_ptr<sl::Camera> zed,
  const std::shared_ptr<sensor_msgs::msg::CameraInfo> & leftCamInfoMsg,
  const std::shared_ptr<sensor_msgs::msg::CameraInfo> & rightCamInfoMsg,
  const std::string & leftFrameId, const std::string & rightFrameId,
  bool rawParam /*= false*/)
{
  sl::CalibrationParameters zedParam;

  if (rawParam) {
    zedParam = zed->getCameraInformation(mMatResol)
      .camera_configuration.calibration_parameters_raw;
  } else {
    zedParam = zed->getCameraInformation(mMatResol)
      .camera_configuration.calibration_parameters;
  }

  float baseline = zedParam.getCameraBaseline();

  // ----> Distortion models
  // ZED SDK params order: [ k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]
  // Radial (k1, k2, k3, k4, k5, k6), Tangential (p1,p2) and Prism (s1, s2, s3,
  // s4) distortion. Prism not currently used.

  // ROS2 order (OpenCV) -> k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
  switch (mCamRealModel) {
    case sl::MODEL::ZED:  // PLUMB_BOB
      leftCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::PLUMB_BOB;
      rightCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::PLUMB_BOB;
      leftCamInfoMsg->d.resize(5);
      rightCamInfoMsg->d.resize(5);
      leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
      leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
      leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
      leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
      leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
      rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
      rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
      rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
      rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
      rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
      break;

    case sl::MODEL::ZED2:    // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED2i:   // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED_X:   // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED_XM:  // RATIONAL_POLYNOMIAL
    case sl::MODEL::VIRTUAL_ZED_X:  // RATIONAL_POLYNOMIAL
      leftCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
      rightCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
      leftCamInfoMsg->d.resize(8);
      rightCamInfoMsg->d.resize(8);
      leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
      leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
      leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
      leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
      leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
      leftCamInfoMsg->d[5] = zedParam.left_cam.disto[5];    // k4
      leftCamInfoMsg->d[6] = zedParam.left_cam.disto[6];    // k5
      leftCamInfoMsg->d[7] = zedParam.left_cam.disto[7];    // k6
      rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
      rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
      rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
      rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
      rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
      rightCamInfoMsg->d[5] = zedParam.right_cam.disto[5];  // k4
      rightCamInfoMsg->d[6] = zedParam.right_cam.disto[6];  // k5
      rightCamInfoMsg->d[7] = zedParam.right_cam.disto[7];  // k6
      break;

    case sl::MODEL::ZED_M:
      if (zedParam.left_cam.disto[5] != 0 &&   // k4!=0
        zedParam.right_cam.disto[2] == 0 &&    // p1==0
        zedParam.right_cam.disto[3] == 0)      // p2==0
      {
        leftCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::EQUIDISTANT;
        rightCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::EQUIDISTANT;

        leftCamInfoMsg->d.resize(4);
        rightCamInfoMsg->d.resize(4);
        leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
        leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
        leftCamInfoMsg->d[2] = zedParam.left_cam.disto[4];    // k3
        leftCamInfoMsg->d[3] = zedParam.left_cam.disto[5];    // k4
        rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
        rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
        rightCamInfoMsg->d[2] = zedParam.right_cam.disto[4];  // k3
        rightCamInfoMsg->d[3] = zedParam.right_cam.disto[5];  // k4
      } else {
        leftCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::PLUMB_BOB;
        rightCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::PLUMB_BOB;
        leftCamInfoMsg->d.resize(5);
        rightCamInfoMsg->d.resize(5);
        leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
        leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
        leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
        leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
        leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
        rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
        rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
        rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
        rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
        rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
      }
  }

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
    // ROS frame (X forward, Z up, Y left)
    for (int i = 0; i < 9; i++) {
      rightCamInfoMsg->r[i] =
        zedParam.stereo_transform.getRotationMatrix().r[i];
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
  rightCamInfoMsg->p[3] =
    static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
  rightCamInfoMsg->p[0] = static_cast<double>(zedParam.right_cam.fx);
  rightCamInfoMsg->p[2] = static_cast<double>(zedParam.right_cam.cx);
  rightCamInfoMsg->p[5] = static_cast<double>(zedParam.right_cam.fy);
  rightCamInfoMsg->p[6] = static_cast<double>(zedParam.right_cam.cy);
  rightCamInfoMsg->p[10] = 1.0;
  leftCamInfoMsg->width = rightCamInfoMsg->width =
    static_cast<uint32_t>(mMatResol.width);
  leftCamInfoMsg->height = rightCamInfoMsg->height =
    static_cast<uint32_t>(mMatResol.height);
  leftCamInfoMsg->header.frame_id = leftFrameId;
  rightCamInfoMsg->header.frame_id = rightFrameId;
}

bool ZedCamera::areVideoDepthSubscribed()
{
  mRgbSubCount = 0;
  mRgbRawSubCount = 0;
  mRgbGraySubCount = 0;
  mRgbGrayRawSubCount = 0;
  mLeftSubCount = 0;
  mLeftRawSubCount = 0;
  mLeftGraySubCount = 0;
  mLeftGrayRawSubCount = 0;
  mRightSubCount = 0;
  mRightRawSubCount = 0;
  mRightGraySubCount = 0;
  mRightGrayRawSubCount = 0;
  mStereoSubCount = 0;
  mStereoRawSubCount = 0;
  mDepthSubCount = 0;
  mConfMapSubCount = 0;
  mDisparitySubCount = 0;
  mDepthInfoSubCount = 0;

  try {
    mRgbSubCount = mPubRgb.getNumSubscribers();
    mRgbRawSubCount = mPubRawRgb.getNumSubscribers();
    mRgbGraySubCount = mPubRgbGray.getNumSubscribers();
    mRgbGrayRawSubCount = mPubRawRgbGray.getNumSubscribers();
    mLeftSubCount = mPubLeft.getNumSubscribers();
    mLeftRawSubCount = mPubRawLeft.getNumSubscribers();
    mLeftGraySubCount = mPubLeftGray.getNumSubscribers();
    mLeftGrayRawSubCount = mPubRawLeftGray.getNumSubscribers();
    mRightSubCount = mPubRight.getNumSubscribers();
    mRightRawSubCount = mPubRawRight.getNumSubscribers();
    mRightGraySubCount = mPubRightGray.getNumSubscribers();
    mRightGrayRawSubCount = mPubRawRightGray.getNumSubscribers();
    mStereoSubCount = mPubStereo.getNumSubscribers();
    mStereoRawSubCount = mPubRawStereo.getNumSubscribers();

    if (!mDepthDisabled) {
      mDepthSubCount = mPubDepth.getNumSubscribers();
      mDepthInfoSubCount = count_subscribers(mPubDepthInfo->get_topic_name());
      mConfMapSubCount = count_subscribers(mPubConfMap->get_topic_name());
      mDisparitySubCount = count_subscribers(mPubDisparity->get_topic_name());
    }
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_VD(" * [areVideoDepthSubscribed] Exception while counting subscribers");
    return false;
  }

  return (mRgbSubCount + mRgbRawSubCount + mRgbGraySubCount +
         mRgbGrayRawSubCount + mLeftSubCount + mLeftRawSubCount +
         mLeftGraySubCount + mLeftGrayRawSubCount + mRightSubCount +
         mRightRawSubCount + mRightGraySubCount + mRightGrayRawSubCount +
         mStereoSubCount + mStereoRawSubCount + mDepthSubCount +
         mConfMapSubCount + mDisparitySubCount + mDepthInfoSubCount) > 0;
}

bool ZedCamera::isDepthRequired()
{
  // DEBUG_STREAM_COMM( "isDepthRequired called");

  if (mDepthDisabled) {
    return false;
  }

  size_t tot_sub = 0;

  try {
    size_t depthSub = 0;
    size_t confMapSub = 0;
    size_t dispSub = 0;
    size_t pcSub = 0;
    size_t depthInfoSub = 0;
    depthSub = mPubDepth.getNumSubscribers();
    confMapSub = count_subscribers(mPubConfMap->get_topic_name());
    dispSub = count_subscribers(mPubDisparity->get_topic_name());
#ifndef FOUND_FOXY
    pcSub = mPubCloud.getNumSubscribers();
#else
    pcSub = count_subscribers(mPubCloud->get_topic_name());
#endif
    depthInfoSub = count_subscribers(mPubDepthInfo->get_topic_name());

    tot_sub = depthSub + confMapSub + dispSub + pcSub + depthInfoSub;
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_VD(" * [isDepthRequired] Exception while counting subscribers");
    return false;
  }

  return tot_sub > 0 || isPosTrackingRequired();
}

void ZedCamera::applyDepthSettings()
{
  if (isDepthRequired()) {
    std::lock_guard<std::mutex> lock(mDynParMutex);
    mRunParams.confidence_threshold =
      mDepthConf;      // Update depth confidence if changed
    mRunParams.texture_confidence_threshold =
      mDepthTextConf;      // Update depth texture confidence if changed
    mRunParams.remove_saturated_areas = mRemoveSatAreas;

    DEBUG_STREAM_COMM_ONCE("Depth extraction enabled");
    mRunParams.enable_depth = true;
  } else {
    DEBUG_STREAM_COMM_ONCE("Depth extraction disabled");
    mRunParams.enable_depth = false;
  }
}

void ZedCamera::applyVideoSettings()
{
  if (!mSvoMode && mFrameCount % 10 == 0) {
    std::lock_guard<std::mutex> lock(mDynParMutex);

    applyAutoExposureGainSettings();
    applyExposureGainSettings();
    applyWhiteBalanceSettings();
    applyBrightnessContrastHueSettings();
    applySaturationSharpnessGammaSettings();
    applyZEDXSettings();
  }
}

// Helper: Auto Exposure/Gain
void ZedCamera::applyAutoExposureGainSettings()
{
  sl::ERROR_CODE err;
  sl::VIDEO_SETTINGS setting;

  if (mTriggerAutoExpGain) {
    setting = sl::VIDEO_SETTINGS::AEC_AGC;
    err = mZed->setCameraSettings(setting, (mCamAutoExpGain ? 1 : 0));
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting AEC_AGC: "
          << sl::toString(err).c_str());
    } else {
      mTriggerAutoExpGain = false;
      DEBUG_STREAM_CTRL(
        "New setting for " << sl::toString(setting).c_str()
                           << ": "
                           << (mCamAutoExpGain ? 1 : 0));
    }
  }
}

// Helper: Exposure and Gain
void ZedCamera::applyExposureGainSettings()
{
  sl::ERROR_CODE err;
  sl::VIDEO_SETTINGS setting;

  if (!mCamAutoExpGain) {
    int value;
    err = mZed->getCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamExposure) {
      mZed->setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, mCamExposure);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(sl::VIDEO_SETTINGS::EXPOSURE).c_str()
          << ": "
          << sl::toString(err).c_str());
    }

    err = mZed->getCameraSettings(sl::VIDEO_SETTINGS::GAIN, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamGain) {
      err = mZed->setCameraSettings(sl::VIDEO_SETTINGS::GAIN, mCamGain);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(sl::VIDEO_SETTINGS::GAIN).c_str()
          << ": "
          << sl::toString(err).c_str());
    }
  }
}

// Helper: White Balance
void ZedCamera::applyWhiteBalanceSettings()
{
  sl::ERROR_CODE err;
  sl::VIDEO_SETTINGS setting;

  if (mTriggerAutoWB) {
    setting = sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO;
    err = mZed->setCameraSettings(setting, (mCamAutoWB ? 1 : 0));
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    } else {
      mTriggerAutoWB = false;
      DEBUG_STREAM_CTRL(
        "New setting for " << sl::toString(setting).c_str()
                           << ": " << (mCamAutoWB ? 1 : 0));
    }
  }

  if (!mCamAutoWB) {
    int value;
    setting = sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamWBTemp) {
      err = mZed->setCameraSettings(setting, mCamWBTemp);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }
  }
}

// Helper: Brightness, Contrast, Hue (not for ZED X)
void ZedCamera::applyBrightnessContrastHueSettings()
{
  if (!sl_tools::isZEDX(mCamRealModel)) {
    sl::ERROR_CODE err;
    sl::VIDEO_SETTINGS setting;
    int value;

    setting = sl::VIDEO_SETTINGS::BRIGHTNESS;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamBrightness) {
      mZed->setCameraSettings(setting, mCamBrightness);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }

    setting = sl::VIDEO_SETTINGS::CONTRAST;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamContrast) {
      err = mZed->setCameraSettings(setting, mCamContrast);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }

    setting = sl::VIDEO_SETTINGS::HUE;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamHue) {
      mZed->setCameraSettings(setting, mCamHue);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }
  }
}

// Helper: Saturation, Sharpness, Gamma
void ZedCamera::applySaturationSharpnessGammaSettings()
{
  sl::ERROR_CODE err;
  sl::VIDEO_SETTINGS setting;
  int value;

  setting = sl::VIDEO_SETTINGS::SATURATION;
  err = mZed->getCameraSettings(setting, value);
  if (err == sl::ERROR_CODE::SUCCESS && value != mCamSaturation) {
    mZed->setCameraSettings(setting, mCamSaturation);
  }

  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Error setting "
        << sl::toString(setting).c_str()
        << ": "
        << sl::toString(err).c_str());
  }

  setting = sl::VIDEO_SETTINGS::SHARPNESS;
  err = mZed->getCameraSettings(setting, value);
  if (err == sl::ERROR_CODE::SUCCESS && value != mCamSharpness) {
    mZed->setCameraSettings(setting, mCamSharpness);
  }

  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Error setting "
        << sl::toString(setting).c_str()
        << ": "
        << sl::toString(err).c_str());
  }

  setting = sl::VIDEO_SETTINGS::GAMMA;
  err = mZed->getCameraSettings(setting, value);
  if (err == sl::ERROR_CODE::SUCCESS && value != mCamGamma) {
    err = mZed->setCameraSettings(setting, mCamGamma);
  }

  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Error setting "
        << sl::toString(setting).c_str()
        << ": "
        << sl::toString(err).c_str());
  }
}

// Helper: ZED X specific settings
void ZedCamera::applyZEDXSettings()
{
  if (!sl_tools::isZEDX(mCamRealModel)) {
    return;
  }

  applyZEDXExposureSettings();
  applyZEDXAutoExposureTimeRange();
  applyZEDXExposureCompensation();
  applyZEDXAnalogDigitalGain();
  applyZEDXAutoAnalogGainRange();
  applyZEDXAutoDigitalGainRange();
  applyZEDXDenoising();
}

void ZedCamera::applyZEDXExposureSettings()
{
  if (!mCamAutoExpGain) {
    sl::ERROR_CODE err;
    sl::VIDEO_SETTINGS setting = sl::VIDEO_SETTINGS::EXPOSURE_TIME;
    int value;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mGmslExpTime) {
      err = mZed->setCameraSettings(setting, mGmslExpTime);
      DEBUG_STREAM_CTRL(
        "New setting for "
          << sl::toString(setting).c_str() << ": "
          << mGmslExpTime << " [Old " << value << "]");
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Error setting " << sl::toString(setting).c_str()
                         << ": "
                         << sl::toString(err).c_str());
    }
  }
}

void ZedCamera::applyZEDXAutoExposureTimeRange()
{
  sl::ERROR_CODE err;
  int value_min, value_max;
  err = mZed->getCameraSettings(
    sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE, value_min,
    value_max);
  if (err == sl::ERROR_CODE::SUCCESS &&
    (value_min != mGmslAutoExpTimeRangeMin || value_max !=
    mGmslAutoExpTimeRangeMax))
  {
    err = mZed->setCameraSettings(
      sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE,
      mGmslAutoExpTimeRangeMin, mGmslAutoExpTimeRangeMax);
  }

  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Error setting " << sl::toString(sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE).c_str()
                       << ": "
                       << sl::toString(err).c_str());
  }
}

void ZedCamera::applyZEDXExposureCompensation()
{
  if (!mStreamMode) {
    sl::ERROR_CODE err;
    sl::VIDEO_SETTINGS setting = sl::VIDEO_SETTINGS::EXPOSURE_COMPENSATION;
    int value;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mGmslExposureComp) {
      err = mZed->setCameraSettings(setting, mGmslExposureComp);
      DEBUG_STREAM_CTRL(
        "New setting for " << sl::toString(setting).c_str()
                           << ": " << mGmslExposureComp
                           << " [Old " << value << "]");
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }
  }
}

void ZedCamera::applyZEDXAnalogDigitalGain()
{
  sl::ERROR_CODE err;
  sl::VIDEO_SETTINGS setting = sl::VIDEO_SETTINGS::ANALOG_GAIN;

  if (!mCamAutoExpGain) {
    int value;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mGmslAnalogGain) {
      err = mZed->setCameraSettings(setting, mGmslAnalogGain);
      DEBUG_STREAM_CTRL(
        "New setting for "
          << sl::toString(setting).c_str() << ": "
          << mGmslAnalogGain << " [Old " << value << "]");
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Error setting " << sl::toString(setting).c_str()
                         << ": "
                         << sl::toString(err).c_str());
    }

    setting = sl::VIDEO_SETTINGS::DIGITAL_GAIN;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mGmslDigitalGain) {
      err = mZed->setCameraSettings(setting, mGmslDigitalGain);
      DEBUG_STREAM_CTRL(
        "New setting for "
          << sl::toString(setting).c_str() << ": "
          << mGmslDigitalGain << " [Old " << value << "]");
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Error setting " << sl::toString(setting).c_str()
                         << ": "
                         << sl::toString(err).c_str());
    }
  }
}

void ZedCamera::applyZEDXAutoAnalogGainRange()
{
  sl::ERROR_CODE err;
  int value_min, value_max;
  err =
    mZed->getCameraSettings(
    sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE,
    value_min, value_max);
  if (err == sl::ERROR_CODE::SUCCESS &&
    (value_min != mGmslAnalogGainRangeMin || value_max !=
    mGmslAnalogGainRangeMax))
  {
    err = mZed->setCameraSettings(
      sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE,
      mGmslAnalogGainRangeMin, mGmslAnalogGainRangeMax);

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Error setting " << sl::toString(sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE).c_str()
                         << ": "
                         << sl::toString(err).c_str());
    }
  }

  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Error setting " << sl::toString(sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE).c_str()
                       << ": "
                       << sl::toString(err).c_str());
  }
}

void ZedCamera::applyZEDXAutoDigitalGainRange()
{
  sl::ERROR_CODE err;
  int value_min, value_max;
  err =
    mZed->getCameraSettings(
    sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE,
    value_min, value_max);
  if (err == sl::ERROR_CODE::SUCCESS &&
    (value_min != mGmslAutoDigitalGainRangeMin || value_max !=
    mGmslAutoDigitalGainRangeMax))
  {
    err = mZed->setCameraSettings(
      sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE,
      mGmslAutoDigitalGainRangeMin, mGmslAnalogGainRangeMax);
  }

  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Error setting " << sl::toString(sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE).c_str()
                       << ": "
                       << sl::toString(err).c_str());
  }
}

void ZedCamera::applyZEDXDenoising()
{
  if (!mStreamMode) {
    sl::ERROR_CODE err;
    sl::VIDEO_SETTINGS setting = sl::VIDEO_SETTINGS::DENOISING;
    int value;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mGmslDenoising) {
      err = mZed->setCameraSettings(setting, mGmslDenoising);
      DEBUG_STREAM_CTRL(
        "New setting for " << sl::toString(setting).c_str()
                           << ": " << mGmslDenoising
                           << " [Old " << value << "]");
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }
  }
}

void ZedCamera::processVideoDepth()
{
  DEBUG_VD("=== Process Video/Depth ===");

  // If no subscribers, do not retrieve data
  if (areVideoDepthSubscribed()) {
    DEBUG_VD(" * [processVideoDepth] vd_lock -> defer");
    std::unique_lock<std::mutex> vd_lock(mVdMutex, std::defer_lock);

    DEBUG_VD(" * [processVideoDepth] vd_lock -> try_lock");
    if (vd_lock.try_lock()) {
      retrieveVideoDepth();

      // Signal Video/Depth thread that a new pointcloud is ready
      mVdDataReadyCondVar.notify_one();
      mVdDataReady = true;
      mVdPublishing = true;
    } else {
      DEBUG_VD(" * [processVideoDepth] vd_lock not locked");
    }
  } else {
    mVdPublishing = false;
    DEBUG_VD(" * [processVideoDepth] No video/depth subscribers");
  }
  DEBUG_VD("=== Process Video/Depth done ===");
}

void ZedCamera::retrieveVideoDepth()
{
  DEBUG_VD(" *** Retrieving Video/Depth Data ***");
  mRgbSubscribed = false;
  bool retrieved = false;

  DEBUG_STREAM_VD(" *** Retrieving Video Data ***");
  retrieved |= retrieveLeftImage();
  retrieved |= retrieveLeftRawImage();
  retrieved |= retrieveRightImage();
  retrieved |= retrieveRightRawImage();
  retrieved |= retrieveLeftGrayImage();
  retrieved |= retrieveLeftRawGrayImage();
  retrieved |= retrieveRightGrayImage();
  retrieved |= retrieveRightRawGrayImage();

  if (retrieved) {
    DEBUG_STREAM_VD(" *** Video Data retrieved ***");
  }

  retrieved = false;
  DEBUG_STREAM_VD(" *** Retrieving Depth Data ***");
  retrieved |= retrieveDepthMap();
  retrieved |= retrieveDisparity();
  retrieved |= retrieveConfidence();
  retrieved |= retrieveDepthInfo();

  if (retrieved) {
    DEBUG_STREAM_VD(" *** Depth Data retrieved ***");
  }

  DEBUG_VD(" *** Retrieving Video/Depth Data DONE ***");
}

// Helper functions for retrieveVideoDepth()

bool ZedCamera::retrieveLeftImage()
{
  if (mRgbSubCount + mLeftSubCount + mStereoSubCount > 0) {
    DEBUG_VD(" * Retrieving Left image");
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(mMatLeft, sl::VIEW::LEFT, sl::MEM::CPU, mMatResol);
    if (ok) {
      mSdkGrabTS = mMatLeft.timestamp;
      mRgbSubscribed = true;
      DEBUG_VD(" * Left image retrieved");
    }
    return ok;
  }
  return false;
}

bool ZedCamera::retrieveLeftRawImage()
{
  if (mRgbRawSubCount + mLeftRawSubCount + mStereoRawSubCount > 0) {
    DEBUG_VD(" * Retrieving Left raw image");
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatLeftRaw, sl::VIEW::LEFT_UNRECTIFIED,
      sl::MEM::CPU, mMatResol);
    if (ok) {
      mSdkGrabTS = mMatLeftRaw.timestamp;
      DEBUG_VD(" * Left raw image retrieved");
    }
    return ok;
  }
  return false;
}

bool ZedCamera::retrieveRightImage()
{
  if (mRightSubCount + mStereoSubCount > 0) {
    DEBUG_VD(" * Retrieving Right image");
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(mMatRight, sl::VIEW::RIGHT, sl::MEM::CPU, mMatResol);
    if (ok) {
      mSdkGrabTS = mMatRight.timestamp;
      DEBUG_VD(" * Right image retrieved");
    }
    return ok;
  }
  return false;
}

bool ZedCamera::retrieveRightRawImage()
{
  if (mRightRawSubCount + mStereoRawSubCount > 0) {
    DEBUG_VD(" * Retrieving Right raw image");
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatRightRaw, sl::VIEW::RIGHT_UNRECTIFIED,
      sl::MEM::CPU, mMatResol);
    if (ok) {
      mSdkGrabTS = mMatRightRaw.timestamp;
      DEBUG_VD(" * Right raw image retrieved");
    }
    return ok;
  }
  return false;
}

bool ZedCamera::retrieveLeftGrayImage()
{
  if (mRgbGraySubCount + mLeftGraySubCount > 0) {
    DEBUG_VD(" * Retrieving Left gray image");
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatLeftGray, sl::VIEW::LEFT_GRAY,
      sl::MEM::CPU, mMatResol);
    if (ok) {
      mSdkGrabTS = mMatLeftGray.timestamp;
      DEBUG_VD(" * Left gray image retrieved");
    }
    return ok;
  }
  return false;
}

bool ZedCamera::retrieveLeftRawGrayImage()
{
  if (mRgbGrayRawSubCount + mLeftGrayRawSubCount > 0) {
    DEBUG_VD(" * Retrieving Left gray raw image");
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatLeftRawGray, sl::VIEW::LEFT_UNRECTIFIED_GRAY,
      sl::MEM::CPU, mMatResol);
    if (ok) {
      mSdkGrabTS = mMatLeftRawGray.timestamp;
      DEBUG_VD(" * Left gray raw image retrieved");
    }
    return ok;
  }
  return false;
}

bool ZedCamera::retrieveRightGrayImage()
{
  if (mRightGraySubCount > 0) {
    DEBUG_VD(" * Retrieving Right gray image");
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatRightGray, sl::VIEW::RIGHT_GRAY,
      sl::MEM::CPU, mMatResol);
    if (ok) {
      mSdkGrabTS = mMatRightGray.timestamp;
      DEBUG_VD(" * Right gray image retrieved");
    }
    return ok;
  }
  return false;
}

bool ZedCamera::retrieveRightRawGrayImage()
{
  if (mRightGrayRawSubCount > 0) {
    DEBUG_VD(" * Retrieving Right gray raw image");
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatRightRawGray, sl::VIEW::RIGHT_UNRECTIFIED_GRAY,
      sl::MEM::CPU, mMatResol);
    if (ok) {
      mSdkGrabTS = mMatRightRawGray.timestamp;
      DEBUG_VD(" * Right gray raw image retrieved");
    }
    return ok;
  }
  return false;
}

bool ZedCamera::retrieveDepthMap()
{
  if (mDepthSubCount > 0 || mDepthInfoSubCount > 0) {
    DEBUG_STREAM_VD(" * Retrieving Depth Map");
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveMeasure(
      mMatDepth, sl::MEASURE::DEPTH,
      sl::MEM::CPU, mMatResol);
    if (ok) {
      mSdkGrabTS = mMatDepth.timestamp;
      DEBUG_VD(" * Depth map retrieved");
    }
    return ok;
  }
  return false;
}

bool ZedCamera::retrieveDisparity()
{
  if (mDisparitySubCount > 0) {
    DEBUG_STREAM_VD(" * Retrieving Disparity");
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveMeasure(
      mMatDisp, sl::MEASURE::DISPARITY,
      sl::MEM::CPU, mMatResol);
    if (ok) {
      mSdkGrabTS = mMatDisp.timestamp;
      DEBUG_VD(" * Disparity map retrieved");
    }
    return ok;
  }
  return false;
}

bool ZedCamera::retrieveConfidence()
{
  if (mConfMapSubCount > 0) {
    DEBUG_STREAM_VD(" * Retrieving Confidence");
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveMeasure(
      mMatConf, sl::MEASURE::CONFIDENCE,
      sl::MEM::CPU, mMatResol);
    if (ok) {
      mSdkGrabTS = mMatConf.timestamp;
      DEBUG_VD(" * Confidence map retrieved");
    }
    return ok;
  }
  return false;
}

bool ZedCamera::retrieveDepthInfo()
{
  if (mDepthInfoSubCount > 0) {
    bool ok = sl::ERROR_CODE::SUCCESS ==
      mZed->getCurrentMinMaxDepth(mMinDepth, mMaxDepth);
    if (ok) {
      mSdkGrabTS = mMatConf.timestamp;
      DEBUG_VD(" * Depth info retrieved");
    }
    return ok;
  }
  return false;
}

void ZedCamera::publishVideoDepth(rclcpp::Time & out_pub_ts)
{
  DEBUG_VD("=== Publish Video and Depth topics === ");
  sl_tools::StopWatch vdElabTimer(get_clock());

  checkRgbDepthSync();

  vdElabTimer.tic();

  if (!checkGrabAndUpdateTimestamp(out_pub_ts)) {
    return;
  }

  rclcpp::Time timeStamp = out_pub_ts;

  publishLeftAndRgbImages(timeStamp);
  publishLeftRawAndRgbRawImages(timeStamp);
  publishLeftGrayAndRgbGrayImages(timeStamp);
  publishLeftRawGrayAndRgbRawGrayImages(timeStamp);
  publishRightImages(timeStamp);
  publishRightRawImages(timeStamp);
  publishRightGrayImages(timeStamp);
  publishRightRawGrayImages(timeStamp);
  publishStereoImages(timeStamp);
  publishStereoRawImages(timeStamp);
  publishDepthImage(timeStamp);
  publishConfidenceMap(timeStamp);
  publishDisparityImage(timeStamp);
  publishDepthInfo(timeStamp);

  mVideoDepthElabMean_sec->addValue(vdElabTimer.toc());

  DEBUG_VD("=== Video and Depth topics published === ");
}

// Helper functions for publishVideoDepth

void ZedCamera::checkRgbDepthSync()
{
  sl::Timestamp ts_rgb = 0;
  sl::Timestamp ts_depth = 0;

  if (mRgbSubscribed && (mDepthSubCount > 0 || mDepthInfoSubCount > 0)) {
    ts_rgb = mMatLeft.timestamp;
    ts_depth = mMatDepth.timestamp;

    if (mRgbSubscribed &&
      (ts_rgb.data_ns != 0 && (ts_depth.data_ns != ts_rgb.data_ns)))
    {
      RCLCPP_WARN_STREAM(
        get_logger(),
        " !!!!! DEPTH/RGB ASYNC !!!!! - Delta: "
          << 1e-9 * static_cast<double>(ts_depth - ts_rgb)
          << " sec");
      RCLCPP_WARN(
        get_logger(),
        " NOTE: this should never happen, please contact the node "
        "maintainer in case you get this warning.");
    }
  }
}

bool ZedCamera::checkGrabAndUpdateTimestamp(rclcpp::Time & out_pub_ts)
{
  if (mSdkGrabTS.getNanoseconds() == mLastTs_grab.getNanoseconds()) {
    out_pub_ts = TIMEZERO_ROS;
    DEBUG_VD(" * publishVideoDepth: ignoring not update data");
    DEBUG_STREAM_VD(
      " * Latest Ts: " << mLastTs_grab.getNanoseconds() << " - New Ts: " <<
        mSdkGrabTS.getNanoseconds());
    return false;
  }

  if (mSdkGrabTS.data_ns != 0) {
    if (!mSvoMode) {
      double period_sec =
        static_cast<double>(mSdkGrabTS.data_ns - mLastTs_grab.data_ns) / 1e9;
      DEBUG_STREAM_VD(
        " * VIDEO/DEPTH PUB LAST PERIOD: "
          << period_sec << " sec @" << 1. / period_sec << " Hz / Expected: " << 1. / mVdPubRate << " sec @" << mVdPubRate <<
          " Hz");

      mVideoDepthPeriodMean_sec->addValue(period_sec);
      DEBUG_STREAM_VD(
        " * VIDEO/DEPTH PUB MEAN PERIOD: "
          << mVideoDepthPeriodMean_sec->getAvg() << " sec @"
          << 1. / mVideoDepthPeriodMean_sec->getAvg() << " Hz / Expected: " << 1. / mVdPubRate << " sec @" << mVdPubRate <<
          " Hz");
      mLastTs_grab = mSdkGrabTS;
    }
  }

  if (mSvoMode) {
    out_pub_ts = mFrameTimestamp;
  } else if (mSimMode) {
    if (mUseSimTime) {
      out_pub_ts = get_clock()->now();
    } else {
      out_pub_ts =
        sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
    }
  } else {
    out_pub_ts = sl_tools::slTime2Ros(mSdkGrabTS, get_clock()->get_clock_type());
  }
  return true;
}

void ZedCamera::publishLeftAndRgbImages(const rclcpp::Time & t)
{
  if (mLeftSubCount > 0) {
    DEBUG_STREAM_VD(" * mLeftSubCount: " << mLeftSubCount);
    publishImageWithInfo(
      mMatLeft, mPubLeft, mLeftCamInfoMsg,
      mLeftCamOptFrameId, t);
  }

  if (mRgbSubCount > 0) {
    DEBUG_STREAM_VD(" * mRgbSubCount: " << mRgbSubCount);
    publishImageWithInfo(
      mMatLeft, mPubRgb, mRgbCamInfoMsg, mDepthOptFrameId,
      t);
  }
}

void ZedCamera::publishLeftRawAndRgbRawImages(const rclcpp::Time & t)
{
  if (mLeftRawSubCount > 0) {
    DEBUG_STREAM_VD(" * mLeftRawSubCount: " << mLeftRawSubCount);
    publishImageWithInfo(
      mMatLeftRaw, mPubRawLeft, mLeftCamInfoRawMsg,
      mLeftCamOptFrameId, t);
  }
  if (mRgbRawSubCount > 0) {
    DEBUG_STREAM_VD(" * mRgbRawSubCount: " << mRgbRawSubCount);
    publishImageWithInfo(
      mMatLeftRaw, mPubRawRgb, mRgbCamInfoRawMsg,
      mDepthOptFrameId, t);
  }
}

void ZedCamera::publishLeftGrayAndRgbGrayImages(const rclcpp::Time & t)
{
  if (mLeftGraySubCount > 0) {
    DEBUG_STREAM_VD(" * mLeftGraySubCount: " << mLeftGraySubCount);
    publishImageWithInfo(
      mMatLeftGray, mPubLeftGray, mLeftCamInfoMsg,
      mLeftCamOptFrameId, t);
  }
  if (mRgbGraySubCount > 0) {
    DEBUG_STREAM_VD(" * mRgbGraySubCount: " << mRgbGraySubCount);
    publishImageWithInfo(
      mMatLeftGray, mPubRgbGray, mRgbCamInfoMsg,
      mDepthOptFrameId, t);
  }
}

void ZedCamera::publishLeftRawGrayAndRgbRawGrayImages(const rclcpp::Time & t)
{
  if (mLeftGrayRawSubCount > 0) {
    DEBUG_STREAM_VD(" * mLeftGrayRawSubCount: " << mLeftGrayRawSubCount);
    publishImageWithInfo(
      mMatLeftRawGray, mPubRawLeftGray, mLeftCamInfoRawMsg,
      mLeftCamOptFrameId, t);
  }
  if (mRgbGrayRawSubCount > 0) {
    DEBUG_STREAM_VD(" * mRgbGrayRawSubCount: " << mRgbGrayRawSubCount);
    publishImageWithInfo(
      mMatLeftRawGray, mPubRawRgbGray, mRgbCamInfoRawMsg,
      mDepthOptFrameId, t);
  }
}

void ZedCamera::publishRightImages(const rclcpp::Time & t)
{
  if (mRightSubCount > 0) {
    DEBUG_STREAM_VD(" * mRightSubCount: " << mRightSubCount);
    publishImageWithInfo(
      mMatRight, mPubRight, mRightCamInfoMsg,
      mRightCamOptFrameId, t);
  }
}

void ZedCamera::publishRightRawImages(const rclcpp::Time & t)
{
  if (mRightRawSubCount > 0) {
    DEBUG_STREAM_VD(" * mRightRawSubCount: " << mRightRawSubCount);
    publishImageWithInfo(
      mMatRightRaw, mPubRawRight, mRightCamInfoRawMsg,
      mRightCamOptFrameId, t);
  }
}

void ZedCamera::publishRightGrayImages(const rclcpp::Time & t)
{
  if (mRightGraySubCount > 0) {
    DEBUG_STREAM_VD(" * mRightGraySubCount: " << mRightGraySubCount);
    publishImageWithInfo(
      mMatRightGray, mPubRightGray, mRightCamInfoMsg,
      mRightCamOptFrameId, t);
  }
}

void ZedCamera::publishRightRawGrayImages(const rclcpp::Time & t)
{
  if (mRightGrayRawSubCount > 0) {
    DEBUG_STREAM_VD(" * mRightGrayRawSubCount: " << mRightGrayRawSubCount);
    publishImageWithInfo(
      mMatRightRawGray, mPubRawRightGray,
      mRightCamInfoRawMsg, mRightCamOptFrameId, t);
  }
}

void ZedCamera::publishStereoImages(const rclcpp::Time & t)
{
  if (mStereoSubCount > 0) {
    DEBUG_STREAM_VD(" * mStereoSubCount: " << mStereoSubCount);
    auto combined = sl_tools::imagesToROSmsg(
      mMatLeft, mMatRight,
      mCameraFrameId, t);
    DEBUG_STREAM_VD(" * Publishing SIDE-BY-SIDE message");
    try {
      mPubStereo.publish(std::move(combined));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM(" * Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM(" * Message publishing generic ecception: ");
    }
  }
}

void ZedCamera::publishStereoRawImages(const rclcpp::Time & t)
{
  if (mStereoRawSubCount > 0) {
    DEBUG_STREAM_VD(" * mStereoRawSubCount: " << mStereoRawSubCount);
    auto combined = sl_tools::imagesToROSmsg(
      mMatLeftRaw, mMatRightRaw,
      mCameraFrameId, t);
    DEBUG_STREAM_VD(" * Publishing SIDE-BY-SIDE RAW message");
    try {
      mPubRawStereo.publish(std::move(combined));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM(" * Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM(" * Message publishing generic ecception: ");
    }
  }
}

void ZedCamera::publishDepthImage(const rclcpp::Time & t)
{
  if (mDepthSubCount > 0) {
    publishDepthMapWithInfo(mMatDepth, t);
  }
}

void ZedCamera::publishConfidenceMap(const rclcpp::Time & t)
{
  if (mConfMapSubCount > 0) {
    DEBUG_STREAM_VD(" * Publishing CONF MAP message");
    try {
      mPubConfMap->publish(
        *sl_tools::imageToROSmsg(mMatConf, mDepthOptFrameId, t));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM(" * Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM(" * Message publishing generic ecception: ");
    }
  }
}

void ZedCamera::publishDisparityImage(const rclcpp::Time & t)
{
  if (mDisparitySubCount > 0) {
    publishDisparity(mMatDisp, t);
  }
}

void ZedCamera::publishDepthInfo(const rclcpp::Time & t)
{
  if (mDepthInfoSubCount > 0) {
    auto depthInfoMsg = std::make_unique<zed_msgs::msg::DepthInfoStamped>();
    depthInfoMsg->header.stamp = t;
    depthInfoMsg->header.frame_id = mDepthOptFrameId;
    depthInfoMsg->min_depth = mMinDepth;
    depthInfoMsg->max_depth = mMaxDepth;

    DEBUG_STREAM_VD(" * Publishing DEPTH INFO message");
    try {
      mPubDepthInfo->publish(std::move(depthInfoMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM(" * Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM(" * Message publishing generic ecception: ");
    }
  }
}

void ZedCamera::publishImageWithInfo(
  sl::Mat & img,
  image_transport::CameraPublisher & pubImg,
  camInfoMsgPtr & camInfoMsg,
  std::string imgFrameId, rclcpp::Time t)
{
  auto image = sl_tools::imageToROSmsg(img, imgFrameId, t);
  camInfoMsg->header.stamp = t;
  DEBUG_STREAM_VD(" * Publishing IMAGE message: " << t.nanoseconds() << " nsec");
  try {
    pubImg.publish(std::move(image), camInfoMsg);
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM(" * Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM(" * Message publishing generic ecception: ");
  }
}

void ZedCamera::publishDepthMapWithInfo(sl::Mat & depth, rclcpp::Time t)
{
  mDepthCamInfoMsg->header.stamp = t;

  if (!mOpenniDepthMode) {
    auto depth_img = sl_tools::imageToROSmsg(depth, mDepthOptFrameId, t);
    DEBUG_STREAM_VD(
      " * Publishing DEPTH message: " << t.nanoseconds()
                                      << " nsec");
    try {
      mPubDepth.publish(std::move(depth_img), mDepthCamInfoMsg);
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM(" * Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM(" * Message publishing generic ecception: ");
    }
    return;
  }

  // OPENNI CONVERSION (meter -> millimeters - float32 -> uint16)
  auto openniDepthMsg = std::make_unique<sensor_msgs::msg::Image>();

  openniDepthMsg->header.stamp = t;
  openniDepthMsg->header.frame_id = mDepthOptFrameId;
  openniDepthMsg->height = depth.getHeight();
  openniDepthMsg->width = depth.getWidth();

  int num = 1;    // for endianness detection
  openniDepthMsg->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

  openniDepthMsg->step = openniDepthMsg->width * sizeof(uint16_t);
  openniDepthMsg->encoding = sensor_msgs::image_encodings::MONO16;

  size_t size = openniDepthMsg->step * openniDepthMsg->height;
  openniDepthMsg->data.resize(size);

  uint16_t * data = reinterpret_cast<uint16_t *>(&openniDepthMsg->data[0]);

  int dataSize = openniDepthMsg->width * openniDepthMsg->height;
  sl::float1 * depthDataPtr = depth.getPtr<sl::float1>();

  for (int i = 0; i < dataSize; i++) {
    *(data++) = static_cast<uint16_t>(
      std::round(*(depthDataPtr++) * 1000));      // in mm, rounded
  }

  DEBUG_STREAM_VD(" * Publishing OPENNI DEPTH message");
  try {
    mPubDepth.publish(std::move(openniDepthMsg), mDepthCamInfoMsg);
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM(" * Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM(" * Message publishing generic ecception: ");
  }
}

void ZedCamera::publishDisparity(sl::Mat disparity, rclcpp::Time t)
{
  sl::CameraInformation zedParam = mZed->getCameraInformation(mMatResol);

  std::unique_ptr<sensor_msgs::msg::Image> disparity_image =
    sl_tools::imageToROSmsg(disparity, mDepthOptFrameId, t);

  auto disparityMsg = std::make_unique<stereo_msgs::msg::DisparityImage>();
  disparityMsg->image = *disparity_image.get();
  disparityMsg->header = disparityMsg->image.header;
  disparityMsg->f =
    zedParam.camera_configuration.calibration_parameters.left_cam.fx;
  disparityMsg->t = zedParam.camera_configuration.calibration_parameters
    .getCameraBaseline();
  disparityMsg->min_disparity =
    disparityMsg->f * disparityMsg->t /
    mZed->getInitParameters().depth_minimum_distance;
  disparityMsg->max_disparity =
    disparityMsg->f * disparityMsg->t /
    mZed->getInitParameters().depth_maximum_distance;

  DEBUG_STREAM_VD(" * Publishing DISPARITY message");
  try {
    mPubDisparity->publish(std::move(disparityMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM(" * Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM(" * Message publishing generic ecception: ");
  }
}

void ZedCamera::processPointCloud()
{
  DEBUG_PC("=== Process Point Cloud ===");

  if (isPointCloudSubscribed()) {
    // Run the point cloud conversion asynchronously to avoid slowing down
    // all the program
    // Retrieve raw pointCloud data if latest Pointcloud is ready
    DEBUG_PC(" * [processPointCloud] pc_lock -> defer");
    std::unique_lock<std::mutex> pc_lock(mPcMutex, std::defer_lock);

    DEBUG_PC(" * [processPointCloud] pc_lock -> try_lock");
    if (pc_lock.try_lock()) {
      DEBUG_STREAM_PC(
        " * [processPointCloud] Retrieving point cloud size: " << mPcResol.width << "x" <<
          mPcResol.height);
      mZed->retrieveMeasure(
        mMatCloud, sl::MEASURE::XYZBGRA, sl::MEM::CPU,
        mPcResol);
      DEBUG_STREAM_PC(
        " * [processPointCloud] Retrieved point cloud size: " << mMatCloud.getWidth() << "x" <<
          mMatCloud.getHeight());

      // Signal Pointcloud thread that a new pointcloud is ready
      mPcDataReadyCondVar.notify_one();
      mPcDataReady = true;
      mPcPublishing = true;

      DEBUG_STREAM_PC(
        " * [processPointCloud] Extracted point cloud: " << mMatCloud.getInfos().c_str() );
    } else {
      DEBUG_PC(" * [processPointCloud] pc_lock not locked");
    }
  } else {
    mPcPublishing = false;
    DEBUG_PC(" * [processPointCloud] No point cloud subscribers");
  }

  DEBUG_PC("=== Process Point Cloud done ===");
}

bool ZedCamera::isPointCloudSubscribed()
{
  size_t cloudSubCount = 0;
  try {
#ifndef FOUND_FOXY
    cloudSubCount = mPubCloud.getNumSubscribers();
#else
    cloudSubCount = count_subscribers(mPubCloud->get_topic_name());
#endif
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_PC(
      " * [isPointCloudSubscribed] Exception while counting point cloud "
      "subscribers");
    return false;
  }

  return cloudSubCount > 0;
}

void ZedCamera::publishPointCloud()
{
  sl_tools::StopWatch pcElabTimer(get_clock());

  auto pcMsg = std::make_unique<sensor_msgs::msg::PointCloud2>();

  // Initialize Point Cloud message
  // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h

  int width = mPcResol.width;
  int height = mPcResol.height;

  int ptsCount = width * height;

  if (mSvoMode) {
    pcMsg->header.stamp = mFrameTimestamp;
  } else if (mSimMode) {
    if (mUseSimTime) {
      pcMsg->header.stamp = mFrameTimestamp;
    } else {
      pcMsg->header.stamp = sl_tools::slTime2Ros(mMatCloud.timestamp);
    }
  } else {
    pcMsg->header.stamp = sl_tools::slTime2Ros(mMatCloud.timestamp);
  }

  // ---> Check that `pcMsg->header.stamp` is not the same of the latest
  // published pointcloud Avoid to publish the same old data
  if (mLastTs_pc == pcMsg->header.stamp) {
    // Data not updated by a grab calling in the grab thread
    DEBUG_STREAM_PC(" * [publishPointCloud] ignoring not update data");
    return;
  }
  mLastTs_pc = pcMsg->header.stamp;
  // <--- Check that `pcMsg->header.stamp` is not the same of the latest
  // published pointcloud

  if (pcMsg->width != width || pcMsg->height != height) {
    pcMsg->header.frame_id =
      mPointCloudFrameId;      // Set the header values of the ROS message

    pcMsg->is_bigendian = false;
    pcMsg->is_dense = false;

    pcMsg->width = width;
    pcMsg->height = height;

    sensor_msgs::PointCloud2Modifier modifier(*(pcMsg.get()));
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "rgb", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  }

  sl::Vector4<float> * cpu_cloud = mMatCloud.getPtr<sl::float4>();

  // Data copy
  float * ptCloudPtr = reinterpret_cast<float *>(&pcMsg->data[0]);
  memcpy(
    ptCloudPtr, reinterpret_cast<float *>(cpu_cloud),
    ptsCount * 4 * sizeof(float));

  // Pointcloud publishing
  DEBUG_PC(" * [publishPointCloud] Publishing POINT CLOUD message");
#ifndef FOUND_FOXY
  try {
    mPubCloud.publish(std::move(pcMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_PC(" * [publishPointCloud] Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_PC(" * [publishPointCloud] Message publishing generic ecception");
  }
#else
  try {
    mPubCloud->publish(std::move(pcMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_PC(" * [publishPointCloud] Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_PC(" * [publishPointCloud] Message publishing generic ecception");
  }
#endif

  // Publish freq calculation
  double mean = mPcPeriodMean_sec->addValue(mPcFreqTimer.toc());
  mPcFreqTimer.tic();

  // Point cloud elaboration time
  mPcProcMean_sec->addValue(pcElabTimer.toc());
  DEBUG_STREAM_PC(" * [publishPointCloud] Point cloud freq: " << 1. / mean);
}

void ZedCamera::threadFunc_videoDepthElab()
{
  DEBUG_STREAM_VD("Video Depth thread started");
  setupVideoDepthThread();

  mVdDataReady = false;
  std::unique_lock<std::mutex> lock(mVdMutex);

  while (1) {
    if (!rclcpp::ok()) {
      DEBUG_VD(" * [threadFunc_videoDepthElab] Ctrl+C received: stopping video depth thread");
      break;
    }
    if (mThreadStop) {
      DEBUG_VD(
        " * [threadFunc_videoDepthElab] Video/Depth thread stopped");
      break;
    }

    if (!waitForVideoDepthData(lock)) {
      break;
    }

    handleVideoDepthPublishing();

    mVdDataReady = false;
  }

  DEBUG_STREAM_VD("Video/Depth thread finished");
}

// Helper: Setup thread scheduling and debug info
void ZedCamera::setupVideoDepthThread()
{
  DEBUG_STREAM_ADV("Video/Depth thread settings");
  if (_debugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * Default Video/Depth thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par.sched_priority);
    }
  }

  sched_param par;
  par.sched_priority =
    (mThreadSchedPolicy == "SCHED_FIFO" ||
    mThreadSchedPolicy == "SCHED_RR") ? mThreadPrioPointCloud : 0;

  int sched_policy = SCHED_OTHER;
  if (mThreadSchedPolicy == "SCHED_OTHER") {
    sched_policy = SCHED_OTHER;
  } else if (mThreadSchedPolicy == "SCHED_BATCH") {
    sched_policy = SCHED_BATCH;
  } else if (mThreadSchedPolicy == "SCHED_FIFO") {
    sched_policy = SCHED_FIFO;
  } else if (mThreadSchedPolicy == "SCHED_RR") {sched_policy = SCHED_RR;} else {
    RCLCPP_WARN_STREAM(
      get_logger(), " ! Failed to set thread params! - Policy not supported");
    return;
  }

  if (pthread_setschedparam(pthread_self(), sched_policy, &par)) {
    RCLCPP_WARN_STREAM(
      get_logger(), " ! Failed to set thread params! - "
        << std::strerror(errno));
  }

  if (_debugAdvanced) {
    int policy;
    sched_param par2;
    if (pthread_getschedparam(pthread_self(), &policy, &par2)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * New Video/Depth thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par2.sched_priority);
    }
  }
}

// Helper: Wait for video/depth data or thread stop
bool ZedCamera::waitForVideoDepthData(std::unique_lock<std::mutex> & lock)
{
  while (!mVdDataReady) { // loop to avoid spurious wakeups
    if (mVdDataReadyCondVar.wait_for(lock, std::chrono::milliseconds(500)) ==
      std::cv_status::timeout)
    {
      // Check thread stopping
      if (!rclcpp::ok()) {
        DEBUG_VD("[waitForVideoDepthData] Ctrl+C received: stopping video/depth thread");
        mThreadStop = true;
        return false;
      }
      if (mThreadStop) {
        DEBUG_VD(
          "[waitForVideoDepthData] Video/Depth thread stopped");
        return false;
      }

      DEBUG_VD(" * [waitForVideoDepthData] Waiting for Video/Depth data");
    }
  }
  DEBUG_VD(" * [waitForVideoDepthData] Video/Depth data ready to be published");
  DEBUG_STREAM_VD(
    " * [waitForVideoDepthData] mVdMutex: " <<
      (lock.owns_lock() ? "Locked" : "Unlocked"));
  return true;
}

// Helper: Handle publishing and frequency control
void ZedCamera::handleVideoDepthPublishing()
{
  rclcpp::Time pub_ts;
  publishVideoDepth(pub_ts);

  if (!sl_tools::isZED(mCamRealModel) && mVdPublishing &&
    pub_ts != TIMEZERO_ROS)
  {
    if (mSensCameraSync) {
      publishSensorsData(pub_ts);
    }
  }

  // ----> Check publishing frequency
  double vd_period_usec = 1e6 / mVdPubRate;
  double elapsed_usec = mVdPubFreqTimer.toc() * 1e6;

  DEBUG_STREAM_VD(" * [handleVideoDepthPublishing] elapsed_usec " << elapsed_usec);

  int wait_usec = 100;
  if (elapsed_usec < vd_period_usec) {
    wait_usec = static_cast<int>(vd_period_usec - elapsed_usec);
    rclcpp::sleep_for(std::chrono::microseconds(wait_usec));
    DEBUG_STREAM_VD(" * [handleVideoDepthPublishing] wait_usec " << wait_usec);
  } else {
    rclcpp::sleep_for(std::chrono::microseconds(wait_usec));
  }
  DEBUG_STREAM_VD(" * [handleVideoDepthPublishing] sleeped for " << wait_usec << " µsec");

  mVdPubFreqTimer.tic();
  // <---- Check publishing frequency
}

void ZedCamera::setupPointCloudThread()
{
  DEBUG_STREAM_ADV("Point Cloud thread settings");
  if (_debugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * Default Point Cloud thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par.sched_priority);
    }
  }

  if (mThreadSchedPolicy == "SCHED_OTHER") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_BATCH") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_BATCH, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_FIFO") {
    sched_param par;
    par.sched_priority = mThreadPrioPointCloud;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_RR") {
    sched_param par;
    par.sched_priority = mThreadPrioPointCloud;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(), " ! Failed to set thread params! - Policy not supported");
  }

  if (_debugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * New Point Cloud thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par.sched_priority);
    }
  }
}

bool ZedCamera::waitForPointCloudData(std::unique_lock<std::mutex> & lock)
{
  while (!mPcDataReady) {  // loop to avoid spurious wakeups
    if (mPcDataReadyCondVar.wait_for(lock, std::chrono::milliseconds(500)) ==
      std::cv_status::timeout)
    {
      // Check thread stopping
      if (!rclcpp::ok()) {
        DEBUG_PC(" * [waitForPointCloudData] Ctrl+C received: stopping point cloud thread");
        mThreadStop = true;
        return false;
      }
      if (mThreadStop) {
        DEBUG_PC(
          " * [waitForPointCloudData] Point Cloud thread stopped");
        return false;
      }

      DEBUG_PC(" * [waitForPointCloudData] Waiting for point cloud data");
    }
  }
  DEBUG_PC(" * [waitForPointCloudData] Point Cloud ready to be published");
  DEBUG_STREAM_PC(
    " * [waitForPointCloudData] mPcMutex: " <<
      (lock.owns_lock() ? "Locked" : "Unlocked"));
  return true;
}

void ZedCamera::handlePointCloudPublishing()
{
  publishPointCloud();

  // ----> Check publishing frequency
  double pc_period_usec = 1e6 / mPcPubRate;

  double elapsed_usec = mPcPubFreqTimer.toc() * 1e6;

  DEBUG_STREAM_PC(" * [handlePointCloudPublishing] elapsed_usec " << elapsed_usec);

  int wait_usec = 100;
  if (elapsed_usec < pc_period_usec) {
    wait_usec = static_cast<int>(pc_period_usec - elapsed_usec);
    rclcpp::sleep_for(std::chrono::microseconds(wait_usec));
    DEBUG_STREAM_PC(" * [handlePointCloudPublishing] wait_usec " << wait_usec);
  } else {
    rclcpp::sleep_for(std::chrono::microseconds(wait_usec));
  }
  DEBUG_STREAM_PC(" * [handlePointCloudPublishing] sleeped for " << wait_usec << " µsec");

  mPcPubFreqTimer.tic();
  // <---- Check publishing frequency

  mPcDataReady = false;
}

void ZedCamera::threadFunc_pointcloudElab()
{
  DEBUG_STREAM_PC("Point Cloud thread started");
  setupPointCloudThread();

  mPcDataReady = false;

  std::unique_lock<std::mutex> lock(mPcMutex);

  while (1) {
    if (!rclcpp::ok()) {
      DEBUG_PC(" * [threadFunc_pointcloudElab] Ctrl+C received: stopping point cloud thread");
      break;
    }
    if (mThreadStop) {
      DEBUG_STREAM_PC(
        " * [threadFunc_pointcloudElab] Point Cloud thread stopped");
      break;
    }

    if (!waitForPointCloudData(lock)) {
      break;
    }

    handlePointCloudPublishing();
  }

  DEBUG_STREAM_PC("Pointcloud thread finished");
}

bool ZedCamera::handleVideoDepthDynamicParams(
  const rclcpp::Parameter & param,
  rcl_interfaces::msg::SetParametersResult & result)
{
  DEBUG_VD("handleVideoDepthDynamicParams");

  // Split handling into smaller helper functions for maintainability
  if (sl_tools::isZEDX(mCamRealModel)) {
    if (handleGmsl2Params(param, result)) {return true;}
  } else {
    if (handleUsb3Params(param, result)) {return true;}
  }
  if (handleCommonVideoParams(param, result)) {return true;}
  if (handleDepthParams(param, result)) {return true;}

  // If parameter was not handled, return true (no error, but not handled)
  return true;
}

// Helper for GMSL2 (ZED X) parameters
bool ZedCamera::handleGmsl2Params(
  const rclcpp::Parameter & param,
  rcl_interfaces::msg::SetParametersResult & result)
{
  const std::string & name = param.get_name();
  if (name == "video.exposure_time" ||
    name == "video.auto_exposure_time_range_min" ||
    name == "video.auto_exposure_time_range_max" ||
    name == "video.exposure_compensation" ||
    name == "video.analog_gain" ||
    name == "video.auto_analog_gain_range_min" ||
    name == "video.auto_analog_gain_range_max" ||
    name == "video.digital_gain" ||
    name == "video.auto_digital_gain_range_min" ||
    name == "video.auto_digital_gain_range_max" ||
    name == "video.denoising")
  {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason = name + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return true;
    }
    int val = param.as_int();
    if (name == "video.exposure_time") {
      mGmslExpTime = val;
      mCamAutoExpGain = false;
    } else if (name == "video.auto_exposure_time_range_min") {
      mGmslAutoExpTimeRangeMin = val;
    } else if (name == "video.auto_exposure_time_range_max") {
      mGmslAutoExpTimeRangeMax = val;
    } else if (name == "video.exposure_compensation") {
      mGmslExposureComp = val;
    } else if (name == "video.analog_gain") {
      mGmslAnalogGain = val;
      mCamAutoExpGain = false;
    } else if (name == "video.auto_analog_gain_range_min") {
      mGmslAnalogGainRangeMin = val;
    } else if (name == "video.auto_analog_gain_range_max") {
      mGmslAnalogGainRangeMax = val;
    } else if (name == "video.digital_gain") {
      mGmslDigitalGain = val;
      mCamAutoExpGain = false;
    } else if (name == "video.auto_digital_gain_range_min") {
      mGmslAutoDigitalGainRangeMin = val;
    } else if (name == "video.auto_digital_gain_range_max") {
      mGmslAutoDigitalGainRangeMax = val;
    } else if (name == "video.denoising") {
      mGmslDenoising = val;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << name << "' correctly set to " << val);
    return true;
  }
  return false;
}

// Helper for USB3 (ZED/ZED2) parameters
bool ZedCamera::handleUsb3Params(
  const rclcpp::Parameter & param,
  rcl_interfaces::msg::SetParametersResult & result)
{
  const std::string & name = param.get_name();
  if (name == "video.brightness" || name == "video.contrast" || name == "video.hue") {
    if (sl_tools::isZEDX(mCamRealModel)) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Parameter '" << name << "' not available for " << sl::toString(mCamRealModel).c_str());
      return true;
    }
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason = name + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return true;
    }
    int val = param.as_int();
    if (name == "video.brightness") {
      mCamBrightness = val;
    } else if (name == "video.contrast") {
      mCamContrast = val;
    } else if (name == "video.hue") {
      mCamHue = val;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << name << "' correctly set to " << val);
    return true;
  }
  return false;
}

// Helper for common video parameters
bool ZedCamera::handleCommonVideoParams(
  const rclcpp::Parameter & param,
  rcl_interfaces::msg::SetParametersResult & result)
{
  const std::string & name = param.get_name();
  if (name == "video.saturation" || name == "video.sharpness" || name == "video.gamma" ||
    name == "video.exposure" || name == "video.gain" || name == "video.whitebalance_temperature")
  {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason = name + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return true;
    }
    int val = param.as_int();
    if (name == "video.saturation") {
      mCamSaturation = val;
    } else if (name == "video.sharpness") {
      mCamSharpness = val;
    } else if (name == "video.gamma") {
      mCamGamma = val;
    } else if (name == "video.exposure") {
      mCamExposure = val;
      mCamAutoExpGain = false;
    } else if (name == "video.gain") {
      mCamGain = val;
      mCamAutoExpGain = false;
    } else if (name == "video.whitebalance_temperature") {
      mCamWBTemp = val * 100;
      mCamAutoWB = false;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << name << "' correctly set to " << val);
    return true;
  } else if (name == "video.auto_exposure_gain") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason = name + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return true;
    }
    bool val = param.as_bool();
    if (val != mCamAutoExpGain) {
      mTriggerAutoExpGain = true;
    }
    mCamAutoExpGain = val;
    RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << name << "' correctly set to " << val);
    return true;
  } else if (name == "video.auto_whitebalance") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason = name + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return true;
    }
    bool val = param.as_bool();
    if (val != mCamAutoWB) {
      mTriggerAutoWB = true;
    }
    mCamAutoWB = val;
    RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << name << "' correctly set to " << val);
    return true;
  } else if (name == "general.pub_frame_rate") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason = name + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return true;
    }
    double val = param.as_double();
    if ((val <= 0.0) || (val > mCamGrabFrameRate)) {
      result.successful = false;
      result.reason = name + " must be positive and minor of `grab_frame_rate`";
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return true;
    }
    mVdPubRate = val;
    RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << name << "' correctly set to " << val);
    return true;
  }
  return false;
}

// Helper for depth-related parameters
bool ZedCamera::handleDepthParams(
  const rclcpp::Parameter & param,
  rcl_interfaces::msg::SetParametersResult & result)
{
  const std::string & name = param.get_name();
  if (name == "depth.point_cloud_freq") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason = name + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return true;
    }
    double val = param.as_double();
    if ((val <= 0.0) || (val > mCamGrabFrameRate)) {
      result.successful = false;
      result.reason = name + " must be positive and minor of `grab_frame_rate`";
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return true;
    }
    mPcPubRate = val;
    RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << name << "' correctly set to " << val);
    return true;
  } else if (name == "depth.depth_confidence" || name == "depth.depth_texture_conf") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason = name + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return true;
    }
    int val = param.as_int();
    if (name == "depth.depth_confidence") {
      mDepthConf = val;
    } else if (name == "depth.depth_texture_conf") {
      mDepthTextConf = val;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << name << "' correctly set to " << val);
    return true;
  } else if (name == "depth.remove_saturated_areas") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason = name + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return true;
    }
    mRemoveSatAreas = param.as_bool();
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '" << name << "' correctly set to " << (mRemoveSatAreas ? "TRUE" : "FALSE"));
    return true;
  }
  return false;
}

} // namespace stereolabs
