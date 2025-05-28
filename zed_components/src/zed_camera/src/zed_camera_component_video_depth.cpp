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
    sl_tools::getParam(shared_from_this(), "video.brightness", mCamBrightness,
                       mCamBrightness, " * Brightness: ", true, 0, 8);
    sl_tools::getParam(shared_from_this(), "video.contrast", mCamContrast,
                       mCamContrast, " * Contrast: ", true, 0, 8);
    sl_tools::getParam(shared_from_this(), "video.hue", mCamHue, mCamHue,
                       " * Hue: ", true, 0, 11);
  }

  sl_tools::getParam(shared_from_this(), "video.saturation", mCamSaturation,
                     mCamSaturation, " * Saturation: ", true, 0, 8);
  sl_tools::getParam(shared_from_this(), "video.sharpness", mCamSharpness,
                     mCamSharpness, " * Sharpness: ", true, 0, 8);
  sl_tools::getParam(shared_from_this(), "video.gamma", mCamGamma, mCamGamma,
                     " * Gamma: ", true, 1, 9);
  sl_tools::getParam(shared_from_this(), "video.auto_exposure_gain",
                     mCamAutoExpGain, mCamAutoExpGain,
                     " * Auto Exposure/Gain: ", true);
  if (mCamAutoExpGain) {
    mTriggerAutoExpGain = true;
  }
  sl_tools::getParam(shared_from_this(), "video.exposure", mCamExposure,
                     mCamExposure, " * Exposure: ", true, 0, 100);
  sl_tools::getParam(shared_from_this(), "video.gain", mCamGain, mCamGain,
                     " * Gain: ", true, 0, 100);
  sl_tools::getParam(shared_from_this(), "video.auto_whitebalance", mCamAutoWB,
                     mCamAutoWB, " * Auto White Balance: ", true);
  if (mCamAutoWB) {
    mTriggerAutoWB = true;
  }
  int wb = 42;
  sl_tools::getParam(shared_from_this(), "video.whitebalance_temperature", wb,
                     wb, " * White Balance Temperature: ", true, 28, 65);
  mCamWBTemp = wb * 100;

  if (sl_tools::isZEDX(mCamUserModel)) {
    sl_tools::getParam(shared_from_this(), "video.exposure_time", mGmslExpTime,
                       mGmslExpTime, " * ZED X Exposure time: ", true, 28,
                       66000);
    sl_tools::getParam(shared_from_this(), "video.auto_exposure_time_range_min",
                       mGmslAutoExpTimeRangeMin, mGmslAutoExpTimeRangeMin,
                       " * ZED X Auto Exp. time range min: ", true, 28, 66000);
    sl_tools::getParam(shared_from_this(), "video.auto_exposure_time_range_max",
                       mGmslAutoExpTimeRangeMax, mGmslAutoExpTimeRangeMax,
                       " * ZED X Auto Exp. time range max: ", true, 28, 66000);
    sl_tools::getParam(shared_from_this(), "video.exposure_compensation",
                       mGmslExposureComp, mGmslExposureComp,
                       " * ZED X Exposure comp.: ", true, 0, 100);
    sl_tools::getParam(shared_from_this(), "video.analog_gain", mGmslAnalogGain,
                       mGmslAnalogGain, " * ZED X Analog Gain: ", true, 1000,
                       16000);
    sl_tools::getParam(shared_from_this(), "video.auto_analog_gain_range_min",
                       mGmslAnalogGainRangeMin, mGmslAnalogGainRangeMin,
                       " * ZED X Auto Analog Gain range min: ", true, 1000,
                       16000);
    sl_tools::getParam(shared_from_this(), "video.auto_analog_gain_range_max",
                       mGmslAnalogGainRangeMax, mGmslAnalogGainRangeMax,
                       " * ZED X Auto Analog Gain range max: ", true, 1000,
                       16000);
    sl_tools::getParam(shared_from_this(), "video.digital_gain",
                       mGmslDigitalGain, mGmslDigitalGain,
                       " * ZED X Digital Gain: ", true, 1, 256);
    sl_tools::getParam(shared_from_this(), "video.auto_digital_gain_range_min",
                       mGmslAutoDigitalGainRangeMin,
                       mGmslAutoDigitalGainRangeMin,
                       " * ZED X Auto Digital Gain range min: ", true, 1, 256);
    sl_tools::getParam(shared_from_this(), "video.auto_digital_gain_range_max",
                       mGmslAutoDigitalGainRangeMax,
                       mGmslAutoDigitalGainRangeMax,
                       " * ZED X Auto Digital Gain range max: ", true, 1, 256);
    sl_tools::getParam(shared_from_this(), "video.denoising", mGmslDenoising,
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
      mPcPubRate, "", true, 0.1, 120.0);
    if (mSvoMode && !mSvoRealtime) {
      if (mPcPubRate > 30.0) {
        RCLCPP_WARN(
          get_logger(),
          "'point_cloud_freq' cannot be bigger than '30' in SVO Mode");
        mPcPubRate = 30.0;
      }
    } else {
      if (mPcPubRate > mPubFrameRate) {
        RCLCPP_WARN(
          get_logger(),
          "'point_cloud_freq' cannot be bigger than 'pub_frame_rate'");
        mPcPubRate = mPubFrameRate;
      }
    }
    if (mPcPubRate < 0.1) {
      RCLCPP_WARN(
        get_logger(),
        "'point_cloud_freq' cannot be lower than 0.1 Hz or negative.");
      mPcPubRate = 0.1;
    }
    RCLCPP_INFO_STREAM(get_logger(),
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

    sl_tools::getParam(shared_from_this(), "depth.depth_confidence", mDepthConf,
                       mDepthConf, " * Depth Confidence: ", true, 0, 100);
    sl_tools::getParam(shared_from_this(), "depth.depth_texture_conf",
                       mDepthTextConf, mDepthTextConf,
                       " * Depth Texture Confidence: ", true, 0, 100);
    sl_tools::getParam(shared_from_this(), "depth.remove_saturated_areas",
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
    DEBUG_STREAM_VD("publishImages: Exception while counting subscribers");
    return false;
  }

  return (mRgbSubCount + mRgbRawSubCount + mRgbGraySubCount +
         mRgbGrayRawSubCount + mLeftSubCount + mLeftRawSubCount +
         mLeftGraySubCount + mLeftGrayRawSubCount + mRightSubCount +
         mRightRawSubCount + mRightGraySubCount + mRightGrayRawSubCount +
         mStereoSubCount + mStereoRawSubCount + mDepthSubCount +
         mConfMapSubCount + mDisparitySubCount + mDepthInfoSubCount) > 0;
}

void ZedCamera::retrieveVideoDepth()
{
  mRgbSubscribed = false;
  bool retrieved = false;

  // ----> Retrieve all required data
  DEBUG_VD("Retrieving Video Data");
  if (mRgbSubCount + mLeftSubCount + mStereoSubCount > 0) {
    retrieved |=
      sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(mMatLeft, sl::VIEW::LEFT, sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatLeft.timestamp;
    mRgbSubscribed = true;
    DEBUG_VD("Left image retrieved");
  }
  if (mRgbRawSubCount + mLeftRawSubCount + mStereoRawSubCount > 0) {
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatLeftRaw, sl::VIEW::LEFT_UNRECTIFIED,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatLeftRaw.timestamp;
    DEBUG_VD("Left raw image retrieved");
  }
  if (mRightSubCount + mStereoSubCount > 0) {
    retrieved |=
      sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(mMatRight, sl::VIEW::RIGHT, sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatRight.timestamp;
    DEBUG_VD("Right image retrieved");
  }
  if (mRightRawSubCount + mStereoRawSubCount > 0) {
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatRightRaw, sl::VIEW::RIGHT_UNRECTIFIED,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatRightRaw.timestamp;
    DEBUG_VD("Right raw image retrieved");
  }
  if (mRgbGraySubCount + mLeftGraySubCount > 0) {
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatLeftGray, sl::VIEW::LEFT_GRAY,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatLeftGray.timestamp;
    DEBUG_VD("Left gray image retrieved");
  }
  if (mRgbGrayRawSubCount + mLeftGrayRawSubCount > 0) {
    retrieved |=
      sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatLeftRawGray, sl::VIEW::LEFT_UNRECTIFIED_GRAY,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatLeftRawGray.timestamp;
    DEBUG_VD("Left gray raw image retrieved");
  }
  if (mRightGraySubCount > 0) {
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatRightGray, sl::VIEW::RIGHT_GRAY,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatRightGray.timestamp;
    DEBUG_VD("Right gray image retrieved");
  }
  if (mRightGrayRawSubCount > 0) {
    retrieved |=
      sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveImage(
      mMatRightRawGray, sl::VIEW::RIGHT_UNRECTIFIED_GRAY,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatRightRawGray.timestamp;
    DEBUG_VD("Right gray raw image retrieved");
  }
  if (retrieved) {
    DEBUG_STREAM_VD("Video Data retrieved");
  }

  retrieved = false;
  DEBUG_STREAM_VD("Retrieving Depth Data");
  if (mDepthSubCount > 0 || mDepthInfoSubCount > 0) {
    DEBUG_STREAM_VD("Retrieving Depth");
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveMeasure(
      mMatDepth, sl::MEASURE::DEPTH,
      sl::MEM::CPU, mMatResol);

    mSdkGrabTS = mMatDepth.timestamp;
    DEBUG_STREAM_VD("Depth map retrieved: " << mMatDepth.getInfos().c_str());
  }
  if (mDisparitySubCount > 0) {
    DEBUG_STREAM_VD("Retrieving Disparity");
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveMeasure(
      mMatDisp, sl::MEASURE::DISPARITY,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatDisp.timestamp;
    DEBUG_VD("Disparity map retrieved");
  }
  if (mConfMapSubCount > 0) {
    DEBUG_STREAM_VD("Retrieving Confidence");
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->retrieveMeasure(
      mMatConf, sl::MEASURE::CONFIDENCE,
      sl::MEM::CPU, mMatResol);
    mSdkGrabTS = mMatConf.timestamp;
    DEBUG_VD("Confidence map retrieved");
  }
  if (mDepthInfoSubCount > 0) {
    retrieved |= sl::ERROR_CODE::SUCCESS ==
      mZed->getCurrentMinMaxDepth(mMinDepth, mMaxDepth);
    mSdkGrabTS = mMatConf.timestamp;
    DEBUG_VD("Depth info retrieved");
  }
  if (retrieved) {
    DEBUG_STREAM_VD("Depth Data retrieved");
  }
  // <---- Retrieve all required data
}
  
void ZedCamera::publishVideoDepth(rclcpp::Time & out_pub_ts)
{
  DEBUG_VD("=== Publish Video and Depth topics === ");
  sl_tools::StopWatch vdElabTimer(get_clock());

  // ----> Check RGB/Depth sync
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
        "!!!!! DEPTH/RGB ASYNC !!!!! - Delta: "
          << 1e-9 * static_cast<double>(ts_depth - ts_rgb)
          << " sec");
      RCLCPP_WARN(
        get_logger(),
        "NOTE: this should never happen, please contact the node "
        "maintainer in case you get this warning.");
    }
  }
  // <---- Check RGB/Depth sync

  // Start processing timer for diagnostic
  vdElabTimer.tic();

  // ----> Check if a grab has been done before publishing the same images
  if (mSdkGrabTS.getNanoseconds() == mLastTs_grab.getNanoseconds()) {
    out_pub_ts = TIMEZERO_ROS;
    // Data not updated by a grab calling in the grab thread
    DEBUG_VD("publishVideoDepth: ignoring not update data");
    DEBUG_STREAM_VD(
      "Latest Ts: " << mLastTs_grab.getNanoseconds() << " - New Ts: " <<
        mSdkGrabTS.getNanoseconds());
    return;
  }

  if (mSdkGrabTS.data_ns != 0) {
    if (!mSvoMode) {
      double period_sec =
        static_cast<double>(mSdkGrabTS.data_ns - mLastTs_grab.data_ns) / 1e9;
      DEBUG_STREAM_VD(
        "VIDEO/DEPTH PUB LAST PERIOD: "
          << period_sec << " sec @" << 1. / period_sec << " Hz");

      mVideoDepthPeriodMean_sec->addValue(period_sec);
      DEBUG_STREAM_VD(
        "VIDEO/DEPTH PUB MEAN PERIOD: "
          << mVideoDepthPeriodMean_sec->getAvg() << " sec @"
          << 1. / mVideoDepthPeriodMean_sec->getAvg() << " Hz");
      mLastTs_grab = mSdkGrabTS;
    }
  }

  // <---- Check if a grab has been done before publishing the same images

  rclcpp::Time timeStamp;
  if (mSvoMode) {
    timeStamp = mFrameTimestamp;
  } else if (mSimMode) {
    if (mUseSimTime) {
      timeStamp = get_clock()->now();
    } else {
      timeStamp =
        sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
    }
  } else {
    timeStamp = sl_tools::slTime2Ros(mSdkGrabTS, get_clock()->get_clock_type());
  }

  out_pub_ts = timeStamp;

  // ----> Publish the left=rgb image if someone has subscribed to
  if (mLeftSubCount > 0) {
    DEBUG_STREAM_VD("mLeftSubCount: " << mLeftSubCount);
    publishImageWithInfo(
      mMatLeft, mPubLeft, mLeftCamInfoMsg,
      mLeftCamOptFrameId, out_pub_ts);
  }

  if (mRgbSubCount > 0) {
    DEBUG_STREAM_VD("mRgbSubCount: " << mRgbSubCount);
    publishImageWithInfo(
      mMatLeft, mPubRgb, mRgbCamInfoMsg, mDepthOptFrameId,
      out_pub_ts);
  }
  // <---- Publish the left=rgb image if someone has subscribed to

  // ----> Publish the left_raw=rgb_raw image if someone has subscribed to
  if (mLeftRawSubCount > 0) {
    DEBUG_STREAM_VD("mLeftRawSubCount: " << mLeftRawSubCount);
    publishImageWithInfo(
      mMatLeftRaw, mPubRawLeft, mLeftCamInfoRawMsg,
      mLeftCamOptFrameId, out_pub_ts);
  }
  if (mRgbRawSubCount > 0) {
    DEBUG_STREAM_VD("mRgbRawSubCount: " << mRgbRawSubCount);
    publishImageWithInfo(
      mMatLeftRaw, mPubRawRgb, mRgbCamInfoRawMsg,
      mDepthOptFrameId, out_pub_ts);
  }
  // <---- Publish the left_raw=rgb_raw image if someone has subscribed to

  // ----> Publish the left_gray=rgb_gray image if someone has subscribed to
  if (mLeftGraySubCount > 0) {
    DEBUG_STREAM_VD("mLeftGraySubCount: " << mLeftGraySubCount);
    publishImageWithInfo(
      mMatLeftGray, mPubLeftGray, mLeftCamInfoMsg,
      mLeftCamOptFrameId, out_pub_ts);
  }
  if (mRgbGraySubCount > 0) {
    DEBUG_STREAM_VD("mRgbGraySubCount: " << mRgbGraySubCount);
    publishImageWithInfo(
      mMatLeftGray, mPubRgbGray, mRgbCamInfoMsg,
      mDepthOptFrameId, out_pub_ts);
  }
  // <---- Publish the left_raw=rgb_raw image if someone has subscribed to

  // ----> Publish the left_raw_gray=rgb_raw_gray image if someone has
  // subscribed to
  if (mLeftGrayRawSubCount > 0) {
    DEBUG_STREAM_VD("mLeftGrayRawSubCount: " << mLeftGrayRawSubCount);
    publishImageWithInfo(
      mMatLeftRawGray, mPubRawLeftGray, mLeftCamInfoRawMsg,
      mLeftCamOptFrameId, out_pub_ts);
  }
  if (mRgbGrayRawSubCount > 0) {
    DEBUG_STREAM_VD("mRgbGrayRawSubCount: " << mRgbGrayRawSubCount);
    publishImageWithInfo(
      mMatLeftRawGray, mPubRawRgbGray, mRgbCamInfoRawMsg,
      mDepthOptFrameId, out_pub_ts);
  }
  // ----> Publish the left_raw_gray=rgb_raw_gray image if someone has
  // subscribed to

  // ----> Publish the right image if someone has subscribed to
  if (mRightSubCount > 0) {
    DEBUG_STREAM_VD("mRightSubCount: " << mRightSubCount);
    publishImageWithInfo(
      mMatRight, mPubRight, mRightCamInfoMsg,
      mRightCamOptFrameId, out_pub_ts);
  }
  // <---- Publish the right image if someone has subscribed to

  // ----> Publish the right raw image if someone has subscribed to
  if (mRightRawSubCount > 0) {
    DEBUG_STREAM_VD("mRightRawSubCount: " << mRightRawSubCount);
    publishImageWithInfo(
      mMatRightRaw, mPubRawRight, mRightCamInfoRawMsg,
      mRightCamOptFrameId, out_pub_ts);
  }
  // <---- Publish the right raw image if someone has subscribed to

  // ----> Publish the right gray image if someone has subscribed to
  if (mRightGraySubCount > 0) {
    DEBUG_STREAM_VD("mRightGraySubCount: " << mRightGraySubCount);
    publishImageWithInfo(
      mMatRightGray, mPubRightGray, mRightCamInfoMsg,
      mRightCamOptFrameId, out_pub_ts);
  }
  // <---- Publish the right gray image if someone has subscribed to

  // ----> Publish the right raw gray image if someone has subscribed to
  if (mRightGrayRawSubCount > 0) {
    DEBUG_STREAM_VD("mRightGrayRawSubCount: " << mRightGrayRawSubCount);
    publishImageWithInfo(
      mMatRightRawGray, mPubRawRightGray,
      mRightCamInfoRawMsg, mRightCamOptFrameId, out_pub_ts);
  }
  // <---- Publish the right raw gray image if someone has subscribed to

  // ----> Publish the side-by-side image if someone has subscribed to
  if (mStereoSubCount > 0) {
    DEBUG_STREAM_VD("mStereoSubCount: " << mStereoSubCount);
    auto combined = sl_tools::imagesToROSmsg(
      mMatLeft, mMatRight,
      mCameraFrameId, out_pub_ts);
    DEBUG_STREAM_VD("Publishing SIDE-BY-SIDE message");
    try {
      mPubStereo.publish(std::move(combined));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
  // <---- Publish the side-by-side image if someone has subscribed to

  // ----> Publish the side-by-side image if someone has subscribed to
  if (mStereoRawSubCount > 0) {
    DEBUG_STREAM_VD("mStereoRawSubCount: " << mStereoRawSubCount);
    auto combined = sl_tools::imagesToROSmsg(
      mMatLeftRaw, mMatRightRaw,
      mCameraFrameId, out_pub_ts);
    DEBUG_STREAM_VD("Publishing SIDE-BY-SIDE RAW message");
    try {
      mPubRawStereo.publish(std::move(combined));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
  // <---- Publish the side-by-side image if someone has subscribed to

  // ---->  Publish the depth image if someone has subscribed to
  if (mDepthSubCount > 0) {
    publishDepthMapWithInfo(mMatDepth, out_pub_ts);
  }
  // <----  Publish the depth image if someone has subscribed to

  // ---->  Publish the confidence image and map if someone has subscribed to
  if (mConfMapSubCount > 0) {
    DEBUG_STREAM_VD("Publishing CONF MAP message");
    try {
      mPubConfMap->publish(
        *sl_tools::imageToROSmsg(mMatConf, mDepthOptFrameId, out_pub_ts));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
  // <----  Publish the confidence image and map if someone has subscribed to

  // ----> Publish the disparity image if someone has subscribed to
  if (mDisparitySubCount > 0) {
    publishDisparity(mMatDisp, out_pub_ts);
  }
  // <---- Publish the disparity image if someone has subscribed to

  // ----> Publish the depth info if someone has subscribed to
  if (mDepthInfoSubCount > 0) {
    auto depthInfoMsg = std::make_unique<zed_msgs::msg::DepthInfoStamped>();
    depthInfoMsg->header.stamp = timeStamp;
    depthInfoMsg->header.frame_id = mDepthOptFrameId;
    depthInfoMsg->min_depth = mMinDepth;
    depthInfoMsg->max_depth = mMaxDepth;

    DEBUG_STREAM_VD("Publishing DEPTH INFO message");
    try {
      mPubDepthInfo->publish(std::move(depthInfoMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
  }
  // <---- Publish the depth info if someone has subscribed to

  // Diagnostic statistic
  mVideoDepthElabMean_sec->addValue(vdElabTimer.toc());

  /*/ ----> Check publishing frequency
  double vd_period_usec = 1e6 / mPubFrameRate;

  double elapsed_usec = mVdPubFreqTimer.toc() * 1e6;

  if (elapsed_usec < vd_period_usec) {
    rclcpp::sleep_for(
      std::chrono::microseconds(
        static_cast<int>(vd_period_usec - elapsed_usec)));
  }

  mVdPubFreqTimer.tic();
  // <---- Check publishing frequency */

  DEBUG_VD("=== Video and Depth topics published === ");
}

void ZedCamera::publishImageWithInfo(
  sl::Mat & img,
  image_transport::CameraPublisher & pubImg,
  camInfoMsgPtr & camInfoMsg,
  std::string imgFrameId, rclcpp::Time t)
{
  auto image = sl_tools::imageToROSmsg(img, imgFrameId, t);
  camInfoMsg->header.stamp = t;
  DEBUG_STREAM_VD("Publishing IMAGE message: " << t.nanoseconds() << " nsec");
  try {
    pubImg.publish(std::move(image), camInfoMsg);
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
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
    DEBUG_STREAM_PC("publishPointCloud: ignoring not update data");
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
  DEBUG_STREAM_PC("Publishing POINT CLOUD message");
#ifndef FOUND_FOXY
  try {
    mPubCloud.publish(std::move(pcMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
#else
  try {
    mPubCloud->publish(std::move(pcMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
#endif

  // Publish freq calculation
  double mean = mPcPeriodMean_sec->addValue(mPcFreqTimer.toc());
  mPcFreqTimer.tic();

  // Point cloud elaboration time
  mPcProcMean_sec->addValue(pcElabTimer.toc());
  DEBUG_STREAM_PC("Point cloud freq: " << 1. / mean);
}

void ZedCamera::publishDepthMapWithInfo(sl::Mat & depth, rclcpp::Time t)
{
  mDepthCamInfoMsg->header.stamp = t;

  if (!mOpenniDepthMode) {
    auto depth_img = sl_tools::imageToROSmsg(depth, mDepthOptFrameId, t);
    DEBUG_STREAM_VD(
      "Publishing DEPTH message: " << t.nanoseconds()
                                   << " nsec");
    try {
      mPubDepth.publish(std::move(depth_img), mDepthCamInfoMsg);
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
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

  DEBUG_STREAM_VD("Publishing OPENNI DEPTH message");
  try {
    mPubDepth.publish(std::move(openniDepthMsg), mDepthCamInfoMsg);
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
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

  DEBUG_STREAM_VD("Publishing DISPARITY message");
  try {
    mPubDisparity->publish(std::move(disparityMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
}

} // namespace stereolabs