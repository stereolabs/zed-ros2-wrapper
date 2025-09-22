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

#include "zed_camera_one_component.hpp"
#include "sl_logging.hpp"

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace stereolabs
{

void ZedCameraOne::getVideoParams()
{
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "=== CAMERA CONTROL parameters ===");

  sl_tools::getParam(
    shared_from_this(), "video.enable_hdr", _enableHDR,
    _enableHDR, " * Enable HDR: ");

  sl_tools::getParam(
    shared_from_this(), "video.saturation", _camSaturation,
    _camSaturation, " * Saturation: ", true, 0, 8);
  _camDynParMapChanged["video.saturation"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.sharpness", _camSharpness,
    _camSharpness, " * Sharpness: ", true, 0, 8);
  _camDynParMapChanged["video.sharpness"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.gamma", _camGamma, _camGamma,
    " * Gamma: ", true, 1, 9);
  _camDynParMapChanged["video.gamma"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.auto_whitebalance", _camAutoWB,
    _camAutoWB, " * Auto White Balance: ", true);
  _camDynParMapChanged["video.auto_whitebalance"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.whitebalance_temperature",
    _camWBTemp, _camWBTemp,
    " * White Balance Temp (x100): ", true, 28, 65);
  _camDynParMapChanged["video.whitebalance_temperature"] = true;

  sl_tools::getParam(
    shared_from_this(), "video.auto_exposure",
    _camAutoExposure, _camAutoExposure,
    " * Auto Exposure: ", true);
  _camDynParMapChanged["video.auto_exposure"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.exposure_time", _camExpTime,
    _camExpTime, " * Exposure (us): ", true, 28, 30000);
  _camDynParMapChanged["video.exposure_time"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.auto_exposure_time_range_min",
    _camAutoExpTimeRangeMin, _camAutoExpTimeRangeMin,
    " * Auto Exp Time Min (us): ", true, 28, 30000);
  _camDynParMapChanged["video.auto_exposure_time_range_min"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.auto_exposure_time_range_max",
    _camAutoExpTimeRangeMax, _camAutoExpTimeRangeMax,
    " * Auto Exp Time Max (us): ", true, 28, 30000);
  _camDynParMapChanged["video.auto_exposure_time_range_max"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.exposure_compensation",
    _camExposureComp, _camExposureComp,
    " * Exposure Compensation: ", true, 0, 100);
  _camDynParMapChanged["video.exposure_compensation"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.auto_analog_gain",
    _camAutoAnalogGain, _camAutoAnalogGain,
    " * Auto Analog Gain: ", true);
  _camDynParMapChanged["video.auto_analog_gain"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.analog_gain", _camAnalogGain,
    _camAnalogGain, " * Analog Gain: ", true, 1000, 16000);
  _camDynParMapChanged["video.analog_gain"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.auto_analog_gain_range_min",
    _camAutoAnalogGainRangeMin, _camAutoAnalogGainRangeMin,
    " * Analog Gain Min: ", true, 1000, 16000);
  _camDynParMapChanged["video.auto_analog_gain_range_min"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.auto_analog_gain_range_max",
    _camAutoAnalogGainRangeMax, _camAutoAnalogGainRangeMax,
    " * Analog Gain Max: ", true, 1000, 16000);
  _camDynParMapChanged["video.auto_analog_gain_range_max"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.auto_digital_gain",
    _camAutoDigitalGain, _camAutoDigitalGain,
    " * Auto Digital Gain: ", true);
  _camDynParMapChanged["video.auto_digital_gain"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.digital_gain", _camDigitalGain,
    _camDigitalGain, " * Digital Gain: ", true, 1, 256);
  _camDynParMapChanged["video.digital_gain"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.auto_digital_gain_range_min",
    _camAutoDigitalGainRangeMin, _camAutoDigitalGainRangeMin,
    " * Digital Gain Min: ", true, 1, 256);
  _camDynParMapChanged["video.auto_digital_gain_range_min"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.auto_digital_gain_range_max",
    _camAutoDigitalGainRangeMax, _camAutoDigitalGainRangeMax,
    " * Digital Gain Max: ", true, 1, 256);
  _camDynParMapChanged["video.auto_digital_gain_range_max"] = true;
  sl_tools::getParam(
    shared_from_this(), "video.denoising", _camDenoising,
    _camDenoising, " * Denoising: ", true, 0, 100);
  _camDynParMapChanged["video.denoising"] = true;

  _triggerUpdateDynParams = true;
}

void ZedCameraOne::fillCamInfo(
  sensor_msgs::msg::CameraInfo::SharedPtr camInfoMsg,
  const std::string & frameId, bool rawParam)
{
  sl::CameraParameters zedParam;

  if (rawParam) {
    zedParam =
      _zed->getCameraInformation(_matResol).camera_configuration.calibration_parameters_raw;
  } else {
    zedParam = _zed->getCameraInformation(_matResol).camera_configuration.calibration_parameters;
  }

  // https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html

  // ----> Distortion models
  // ZED SDK params order: [ k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]
  // Radial (k1, k2, k3, k4, k5, k6), Tangential (p1,p2) and Prism (s1, s2, s3,
  // s4) distortion. Prism not currently used.

  // ROS2 order (OpenCV) -> k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
  switch (_camRealModel) {
    case sl::MODEL::ZED_XONE_GS: // RATIONAL_POLYNOMIAL

      camInfoMsg->distortion_model =
        sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

      camInfoMsg->d.resize(8);
      camInfoMsg->d[0] = zedParam.disto[0];    // k1
      camInfoMsg->d[1] = zedParam.disto[1];    // k2
      camInfoMsg->d[2] = zedParam.disto[2];    // p1
      camInfoMsg->d[3] = zedParam.disto[3];    // p2
      camInfoMsg->d[4] = zedParam.disto[4];    // k3
      camInfoMsg->d[5] = zedParam.disto[5];    // k4
      camInfoMsg->d[6] = zedParam.disto[6];    // k5
      camInfoMsg->d[7] = zedParam.disto[7];    // k6
      break;

    case sl::MODEL::ZED_XONE_UHD: // RATIONAL_POLYNOMIAL

      camInfoMsg->distortion_model =
        sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

      camInfoMsg->d.resize(8);
      camInfoMsg->d[0] = zedParam.disto[0];    // k1
      camInfoMsg->d[1] = zedParam.disto[1];    // k2
      camInfoMsg->d[2] = zedParam.disto[2];    // p1
      camInfoMsg->d[3] = zedParam.disto[3];    // p2
      camInfoMsg->d[4] = zedParam.disto[4];    // k3
      camInfoMsg->d[5] = zedParam.disto[5];    // k4
      camInfoMsg->d[6] = zedParam.disto[6];    // k5
      camInfoMsg->d[7] = zedParam.disto[7];    // k6
      break;
  }

  // Intrinsic
  camInfoMsg->k.fill(0.0);
  camInfoMsg->k[0] = static_cast<double>(zedParam.fx);
  camInfoMsg->k[2] = static_cast<double>(zedParam.cx);
  camInfoMsg->k[4] = static_cast<double>(zedParam.fy);
  camInfoMsg->k[5] = static_cast<double>(zedParam.cy);
  camInfoMsg->k[8] = 1.0;

  // Rectification
  camInfoMsg->r.fill(0.0);
  for (size_t i = 0; i < 3; i++) {
    // identity
    camInfoMsg->r[i + i * 3] = 1.0;
  }

  // Projection/camera matrix
  camInfoMsg->p.fill(0.0);
  camInfoMsg->p[0] = static_cast<double>(zedParam.fx);
  camInfoMsg->p[2] = static_cast<double>(zedParam.cx);
  camInfoMsg->p[5] = static_cast<double>(zedParam.fy);
  camInfoMsg->p[6] = static_cast<double>(zedParam.cy);
  camInfoMsg->p[10] = 1.0;

  // Image size
  camInfoMsg->width = static_cast<uint32_t>(_matResol.width);
  camInfoMsg->height = static_cast<uint32_t>(_matResol.height);
  camInfoMsg->header.frame_id = frameId;
}

bool ZedCameraOne::areImageTopicsSubscribed()
{
  _colorSubCount = 0;
  _colorRawSubCount = 0;
#if ENABLE_GRAY_IMAGE
  _graySubCount = 0;
  _grayRawSubCount = 0;
#endif

  try {
    _colorSubCount = _pubColorImg.getNumSubscribers();
    _colorRawSubCount = _pubColorRawImg.getNumSubscribers();
#if ENABLE_GRAY_IMAGE
    _graySubCount = _pubGrayImg.getNumSubscribers();
    _grayRawSubCount = _pubGrayRawImg.getNumSubscribers();
#endif
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_VD("publishImages: Exception while counting subscribers");
    return false;
  }

  return (_colorSubCount + _colorRawSubCount
#if ENABLE_GRAY_IMAGE
         + _graySubCount + _grayRawSubCount
#endif
  ) > 0;
}

void ZedCameraOne::retrieveImages()
{
  bool retrieved = false;

  // ----> Retrieve all required data
  DEBUG_VD("Retrieving Image Data");
  if (_colorSubCount > 0) {
    retrieved |=
      (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(_matColor, sl::VIEW::LEFT, sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matColor.timestamp;
    DEBUG_STREAM_VD(
      "Color image " << _matResol.width << "x" << _matResol.height << " retrieved - timestamp: " <<
        _sdkGrabTS.getNanoseconds() <<
        " nsec");
  }
  if (_colorRawSubCount > 0) {
    retrieved |= (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(
        _matColorRaw, sl::VIEW::LEFT_UNRECTIFIED,
        sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matColorRaw.timestamp;
    DEBUG_STREAM_VD(
      "Color raw image " << _matResol.width << "x" << _matResol.height <<
        " retrieved - timestamp: " << _sdkGrabTS.getNanoseconds() <<
        " nsec");
  }
#if ENABLE_GRAY_IMAGE
  if (_graySubCount > 0) {
    retrieved |= (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(
        _matGray, sl::VIEW::LEFT_GRAY,
        sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matGray.timestamp;
    DEBUG_STREAM_VD(
      "Gray image " << _matResol.width << "x" << _matResol.height << " retrieved - timestamp: " <<
        _sdkGrabTS.getNanoseconds() <<
        " nsec");
  }
  if (_grayRawSubCount > 0) {
    retrieved |=
      (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(
        _matGrayRaw, sl::VIEW::LEFT_UNRECTIFIED_GRAY,
        sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matGrayRaw.timestamp;
    DEBUG_STREAM_VD(
      "Gray raw image " << _matResol.width << "x" << _matResol.height <<
        " retrieved - timestamp: " << _sdkGrabTS.getNanoseconds() <<
        " nsec");
  }
#endif
  if (retrieved) {
    DEBUG_VD("Image Data retrieved");
  }
  // <---- Retrieve all required data
}

void ZedCameraOne::publishImages()
{
  DEBUG_VD("=== Publish Image topics === ");
  sl_tools::StopWatch vdElabTimer(get_clock());

  // Start processing timer for diagnostic
  vdElabTimer.tic();

  // ----> Check if a grab has been done before publishing the same images
  if (_sdkGrabTS.getNanoseconds() == _lastTs_grab.getNanoseconds()) {
    // Data not updated by a grab calling in the grab thread
    DEBUG_VD("publishImages: ignoring not update data");
    DEBUG_STREAM_VD(
      "Latest Ts: " << _lastTs_grab.getNanoseconds()
                    << " - New Ts: "
                    << _sdkGrabTS.getNanoseconds());
    return;
  }

  if (_sdkGrabTS.data_ns != 0) {
    double period_sec =
      static_cast<double>(_sdkGrabTS.data_ns - _lastTs_grab.data_ns) / 1e9;
    DEBUG_STREAM_VD(
      "IMAGE PUB LAST PERIOD: " << period_sec << " sec @"
                                << 1. / period_sec << " Hz");

    _imagePeriodMean_sec->addValue(period_sec);
    DEBUG_STREAM_VD(
      "IMAGE PUB MEAN PERIOD: "
        << _imagePeriodMean_sec->getAvg() << " sec @"
        << 1. / _imagePeriodMean_sec->getAvg() << " Hz");
  }
  _lastTs_grab = _sdkGrabTS;
  // <---- Check if a grab has been done before publishing the same images

  // ----> Timestamp
  rclcpp::Time timeStamp;
  if (_svoMode) {
    timeStamp = _frameTimestamp;
  } else {
    timeStamp = sl_tools::slTime2Ros(_sdkGrabTS, get_clock()->get_clock_type());
  }
  // <---- Timestamp

  // ----> Publish the COLOR image if someone has subscribed to
  if (_colorSubCount > 0) {
    DEBUG_STREAM_VD("_colorSubCount: " << _colorSubCount);
    publishImageWithInfo(
      _matColor, _pubColorImg, _camInfoMsg, _camOptFrameId,
      timeStamp);
  }
  // <---- Publish the COLOR image if someone has subscribed to

  // ----> Publish the COLOR RAW image if someone has subscribed to
  if (_colorRawSubCount > 0) {
    DEBUG_STREAM_VD("_colorRawSubCount: " << _colorRawSubCount);
    publishImageWithInfo(
      _matColorRaw, _pubColorRawImg, _camInfoRawMsg,
      _camOptFrameId, timeStamp);
  }
  // <---- Publish the COLOR RAW image if someone has subscribed to

#if ENABLE_GRAY_IMAGE
  // ----> Publish the GRAY image if someone has subscribed to
  if (_graySubCount > 0) {
    DEBUG_STREAM_VD("_graySubCount: " << _graySubCount);
    publishImageWithInfo(
      _matGray, _pubGrayImg, _camInfoMsg, _camOptFrameId,
      timeStamp);
  }
  // <---- Publish the GRAY image if someone has subscribed to

  // ----> Publish the GRAY RAW image if someone has subscribed to
  if (_grayRawSubCount > 0) {
    DEBUG_STREAM_VD("_grayRawSubCount: " << _grayRawSubCount);
    publishImageWithInfo(
      _matGrayRaw, _pubGrayRawImg, _camInfoRawMsg,
      _camOptFrameId, timeStamp);
  }
  // <---- Publish the GRAY RAW image if someone has subscribed to
#endif

  // Diagnostic statistic
  _imageElabMean_sec->addValue(vdElabTimer.toc());

  // ----> Check publishing frequency
  double vd_period_usec = 1e6 / _camGrabFrameRate;

  double elapsed_usec = _imgPubFreqTimer.toc() * 1e6;

  if (elapsed_usec < vd_period_usec) {
    rclcpp::sleep_for(
      std::chrono::microseconds(
        static_cast<int>(vd_period_usec - elapsed_usec)));
  }

  _imgPubFreqTimer.tic();
  // <---- Check publishing frequency

  DEBUG_VD("=== Video and Depth topics published === ");
}

void ZedCameraOne::publishImageWithInfo(
  const sl::Mat & img, const image_transport::CameraPublisher & pubImg,
  camInfoMsgPtr & camInfoMsg, const std::string & imgFrameId,
  const rclcpp::Time & t)
{
  auto image = sl_tools::imageToROSmsg(img, imgFrameId, t, false);
  camInfoMsg->header.stamp = t;
  DEBUG_STREAM_VD("Publishing IMAGE message: " << t.nanoseconds() << " nsec");
  try {
    pubImg.publish(std::move(image), std::move(camInfoMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }
}

}
