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

#include <image_transport/camera_common.hpp>

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

void ZedCameraOne::initVideoPublishers()
{
  RCLCPP_INFO(get_logger(), " +++ IMAGE TOPICS +++");

  // ----> Advertised topics
  const std::string sensor = "rgb/";
  const std::string rect_prefix = "rect/";
  const std::string raw_prefix = "raw/";
  const std::string color_prefix = "color/";
  const std::string gray_prefix = "gray/";
  const std::string image_topic = "image";

  // Helper to build topic names
  auto make_topic =
    [&](const std::string & sensor, const std::string & color_mode, const std::string & rect_raw,
      const std::string & type) {
      std::string topic = _topicRoot + sensor + color_mode + rect_raw + type;
      return get_node_topics_interface()->resolve_topic_name(topic);
    };

  _imgColorTopic = make_topic(sensor, color_prefix, rect_prefix, image_topic);
  _imgColorRawTopic = make_topic(sensor, color_prefix, raw_prefix, image_topic);
  _imgGrayTopic = make_topic(sensor, gray_prefix, rect_prefix, image_topic);
  _imgRawGrayTopic = make_topic(sensor, gray_prefix, raw_prefix, image_topic);
  // <---- Advertised topics

  // ----> Create publishers
  auto qos = _qos.get_rmw_qos_profile();

  // Publishers logging
  auto log_cam_pub = [&](const auto & pub) {
      RCLCPP_INFO_STREAM(
        get_logger(),
        "  * Advertised on topic: " << pub.getTopic());
      auto transports = image_transport::getLoadableTransports();
      for (const auto & transport : transports) {
        std::string transport_copy = transport;
        auto pos = transport_copy.find('/');
        if (pos != std::string::npos) {
          transport_copy.erase(0, pos);
        }
        RCLCPP_INFO_STREAM(
          get_logger(), " * Advertised on topic: "
            << pub.getTopic() << transport_copy
            << " [image_transport]");
      }
    };

  // Camera publishers
  if (_nitrosDisabled) {
    if (_publishImgRgb) {
      _pubColorImg = image_transport::create_publisher(this, _imgColorTopic, qos);
      log_cam_pub(_pubColorImg);

      if (_publishImgRaw) {
        _pubColorRawImg = image_transport::create_publisher(this, _imgColorRawTopic, qos);
        log_cam_pub(_pubColorRawImg);
      }
    }

    if (_publishImgGray) {
      _pubGrayImg = image_transport::create_publisher(this, _imgGrayTopic, qos);
      log_cam_pub(_pubGrayImg);

      if (_publishImgRaw) {
        _pubGrayRawImg = image_transport::create_publisher(this, _imgRawGrayTopic, qos);
        log_cam_pub(_pubGrayRawImg);
      }
    }


  } else {
#ifdef FOUND_ISAAC_ROS_NITROS
    // Nitros publishers lambda
    auto make_nitros_img_pub = [&](const std::string & topic) {
        auto ret = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
              nvidia::isaac_ros::nitros::NitrosImage>>(
          this, topic, nvidia::isaac_ros::nitros::nitros_image_bgra8_t::supported_type_name,
          nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(), _qos);
        RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << topic);
        RCLCPP_INFO_STREAM(
          get_logger(), "  * Advertised on topic: " << topic + "/nitros [isaac_ros_nitros]");
        return ret;
      };
    if (_publishImgRgb) {
      _nitrosPubColorImg = make_nitros_img_pub(_imgColorTopic);

      if (_publishImgRaw) {
        _nitrosPubColorRawImg = make_nitros_img_pub(_imgColorRawTopic);
      }
    }

    if (_publishImgGray) {
      _nitrosPubGrayImg = make_nitros_img_pub(_imgGrayTopic);
      if (_publishImgRaw) {
        _nitrosPubGrayRawImg = make_nitros_img_pub(_imgRawGrayTopic);
      }
    }
#endif
  }
  // <---- Create publishers

  // ----> Camera Info publishers
  // Lambda to create and log CameraInfo publishers
  auto make_cam_info_pub = [&](const std::string & topic) {
      std::string info_topic = image_transport::getCameraInfoTopic(topic);
      auto pub = create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, _qos);
      RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << pub->get_topic_name());
      return pub;
    };

  // Lambda to create and log CameraInfo publishers for image_transport or nitros
  auto make_cam_info_trans_pub = [&](const std::string & topic) {
      std::string info_topic = topic + "/camera_info";
      auto pub = create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, _qos);
      RCLCPP_INFO_STREAM(
        get_logger(),
        "  * Advertised on topic: " << pub->get_topic_name());
      return pub;
    };

  if (_publishImgRgb) {
    _pubColorImgInfo = make_cam_info_pub(_imgColorTopic);
    _pubColorImgInfoTrans = make_cam_info_trans_pub(_imgColorTopic);

    if (_publishImgRaw) {
      _pubColorRawImgInfo = make_cam_info_pub(_imgColorRawTopic);
      _pubColorRawImgInfoTrans = make_cam_info_trans_pub(_imgColorRawTopic);
    }
  }

  if (_publishImgGray) {
    _pubGrayImgInfo = make_cam_info_pub(_imgGrayTopic);
    _pubGrayImgInfoTrans = make_cam_info_trans_pub(_imgGrayTopic);

    if (_publishImgRaw) {
      _pubGrayRawImgInfo = make_cam_info_pub(_imgRawGrayTopic);
      _pubGrayRawImgInfoTrans = make_cam_info_trans_pub(_imgRawGrayTopic);
    }
  }
  // <---- Camera Info publishers
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

    case sl::MODEL::ZED_XONE_HDR:  // RATIONAL_POLYNOMIAL

      camInfoMsg->distortion_model =
        sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

      camInfoMsg->d.resize(8);
      camInfoMsg->d[0] = zedParam.disto[0];  // k1
      camInfoMsg->d[1] = zedParam.disto[1];  // k2
      camInfoMsg->d[2] = zedParam.disto[2];  // p1
      camInfoMsg->d[3] = zedParam.disto[3];  // p2
      camInfoMsg->d[4] = zedParam.disto[4];  // k3
      camInfoMsg->d[5] = zedParam.disto[5];  // k4
      camInfoMsg->d[6] = zedParam.disto[6];  // k5
      camInfoMsg->d[7] = zedParam.disto[7];  // k6
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

void ZedCameraOne::setupCameraInfoMessages()
{
  _camInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  _camInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();

  fillCamInfo(_camInfoMsg, _camOptFrameId, false);
  fillCamInfo(_camInfoRawMsg, _camOptFrameId, true);
}

bool ZedCameraOne::areImageTopicsSubscribed()
{
  _colorSubCount = 0;
  _colorRawSubCount = 0;
  _graySubCount = 0;
  _grayRawSubCount = 0;

  try {
    if (_nitrosDisabled) {
      _colorSubCount = _pubColorImg.getNumSubscribers();
      _colorRawSubCount = _pubColorRawImg.getNumSubscribers();
      _graySubCount = _pubGrayImg.getNumSubscribers();
      _grayRawSubCount = _pubGrayRawImg.getNumSubscribers();
    } else {
#ifdef FOUND_ISAAC_ROS_NITROS
      _colorSubCount = count_subscribers(_imgColorTopic) + count_subscribers(
        _imgColorTopic + "/nitros");
      _colorRawSubCount = count_subscribers(_imgColorRawTopic) + count_subscribers(
        _imgColorRawTopic + "/nitros");
      _graySubCount = count_subscribers(_imgGrayTopic) +
        count_subscribers(_imgGrayTopic + "/nitros");
      _grayRawSubCount = count_subscribers(_imgRawGrayTopic) + count_subscribers(
        _imgRawGrayTopic + "/nitros");
#endif
    }
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_VD("publishImages: Exception while counting subscribers");
    return false;
  }

  return (_colorSubCount + _colorRawSubCount +
         _graySubCount + _grayRawSubCount
  ) > 0;
}

void ZedCameraOne::handleImageRetrievalAndPublishing()
{
  _imageSubscribed = areImageTopicsSubscribed();
  if (_imageSubscribed) {
    DEBUG_STREAM_VD("Retrieving video data");

    bool gpu = false;
#ifdef FOUND_ISAAC_ROS_NITROS
    if (!_nitrosDisabled) {
      gpu = true;
    }
#endif
    retrieveImages(gpu);
    publishImages();
    _videoPublishing = true;
  } else {
    _videoPublishing = false;
  }
}

void ZedCameraOne::retrieveImages(bool gpu)
{
  bool retrieved = false;

  // ----> Retrieve all required data
  DEBUG_VD("Retrieving Image Data");
  if (_colorSubCount > 0) {
    retrieved |=
      (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(_matColor, sl::VIEW::LEFT, gpu ? sl::MEM::GPU : sl::MEM::CPU, _matResol));
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
        gpu ? sl::MEM::GPU : sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matColorRaw.timestamp;
    DEBUG_STREAM_VD(
      "Color raw image " << _matResol.width << "x" << _matResol.height <<
        " retrieved - timestamp: " << _sdkGrabTS.getNanoseconds() <<
        " nsec");
  }
  if (_graySubCount > 0) {
    retrieved |= (sl::ERROR_CODE::SUCCESS ==
      _zed->retrieveImage(
        _matGray, sl::VIEW::LEFT_GRAY,
        gpu ? sl::MEM::GPU : sl::MEM::CPU, _matResol));
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
        gpu ? sl::MEM::GPU : sl::MEM::CPU, _matResol));
    _sdkGrabTS = _matGrayRaw.timestamp;
    DEBUG_STREAM_VD(
      "Gray raw image " << _matResol.width << "x" << _matResol.height <<
        " retrieved - timestamp: " << _sdkGrabTS.getNanoseconds() <<
        " nsec");
  }
  if (retrieved) {
    DEBUG_VD("Image Data retrieved");
  }
  // <---- Retrieve all required data
}

void ZedCameraOne::publishImages()
{
  DEBUG_VD("=== Publish Image topics === ");
  sl_tools::StopWatch vdElabTimer(get_clock());
  vdElabTimer.tic();

  if (_sdkGrabTS.getNanoseconds() == _lastTs_grab.getNanoseconds()) {
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

  rclcpp::Time timeStamp;
  if (_svoMode) {
    timeStamp = _frameTimestamp;
  } else {
    timeStamp = sl_tools::slTime2Ros(_sdkGrabTS, get_clock()->get_clock_type());
  }

  publishColorImage(timeStamp);
  publishColorRawImage(timeStamp);
  publishGrayImage(timeStamp);
  publishGrayRawImage(timeStamp);

  _imageElabMean_sec->addValue(vdElabTimer.toc());

  double vd_period_usec = 1e6 / _camGrabFrameRate;
  double elapsed_usec = _imgPubFreqTimer.toc() * 1e6;

  if (elapsed_usec < vd_period_usec) {
    rclcpp::sleep_for(
      std::chrono::microseconds(
        static_cast<int>(vd_period_usec - elapsed_usec)));
  }

  _imgPubFreqTimer.tic();

  DEBUG_VD("=== Video and Depth topics published === ");
}

void ZedCameraOne::publishColorImage(const rclcpp::Time & timeStamp)
{
  if (_colorSubCount > 0) {
    DEBUG_STREAM_VD("_colorSubCount: " << _colorSubCount);
    if (_nitrosDisabled) {
      publishImageWithInfo(
        _matColor, _pubColorImg, _pubColorImgInfo, _pubColorImgInfoTrans,
        _camInfoMsg, _camOptFrameId, timeStamp);
    } else {
#ifdef FOUND_ISAAC_ROS_NITROS
      publishImageWithInfo(
        _matColor, _nitrosPubColorImg, _pubColorImgInfo, _pubColorImgInfoTrans,
        _camInfoMsg, _camOptFrameId, timeStamp);
#endif
    }
  }
}

void ZedCameraOne::publishColorRawImage(const rclcpp::Time & timeStamp)
{
  if (_colorRawSubCount > 0) {
    DEBUG_STREAM_VD("_colorRawSubCount: " << _colorRawSubCount);
    if (_nitrosDisabled) {
      publishImageWithInfo(
        _matColorRaw, _pubColorRawImg, _pubColorRawImgInfo,
        _pubColorRawImgInfoTrans, _camInfoRawMsg, _camOptFrameId, timeStamp);
    } else {
#ifdef FOUND_ISAAC_ROS_NITROS
      publishImageWithInfo(
        _matColorRaw, _nitrosPubColorRawImg, _pubColorRawImgInfo, _pubColorRawImgInfoTrans,
        _camInfoRawMsg, _camOptFrameId, timeStamp);
#endif
    }
  }
}

void ZedCameraOne::publishGrayImage(const rclcpp::Time & timeStamp)
{
  if (_graySubCount > 0) {
    DEBUG_STREAM_VD("_graySubCount: " << _graySubCount);
    if (_nitrosDisabled) {
      publishImageWithInfo(
        _matGray, _pubGrayImg, _pubGrayImgInfo, _pubGrayImgInfoTrans,
        _camInfoMsg, _camOptFrameId, timeStamp);
    } else {
#ifdef FOUND_ISAAC_ROS_NITROS
      publishImageWithInfo(
        _matGray, _nitrosPubGrayImg, _pubGrayImgInfo, _pubGrayImgInfoTrans,
        _camInfoMsg, _camOptFrameId, timeStamp);
#endif
    }
  }
}

void ZedCameraOne::publishGrayRawImage(const rclcpp::Time & timeStamp)
{
  if (_grayRawSubCount > 0) {
    DEBUG_STREAM_VD("_grayRawSubCount: " << _grayRawSubCount);
    if (_nitrosDisabled) {
      publishImageWithInfo(
        _matGrayRaw, _pubGrayRawImg, _pubGrayRawImgInfo,
        _pubGrayRawImgInfoTrans, _camInfoRawMsg, _camOptFrameId, timeStamp);
    } else {
#ifdef FOUND_ISAAC_ROS_NITROS
      publishImageWithInfo(
        _matGrayRaw, _nitrosPubGrayRawImg, _pubGrayRawImgInfo, _pubGrayRawImgInfoTrans,
        _camInfoRawMsg, _camOptFrameId, timeStamp);
#endif
    }
  }
}

void ZedCameraOne::publishCameraInfo(
  const camInfoPub & infoPub,
  camInfoMsgPtr & camInfoMsg,
  const rclcpp::Time & t)
{
  camInfoMsg->header.stamp = _usePubTimestamps ? get_clock()->now() : t;
  DEBUG_STREAM_VD(
    " * Publishing Camera Info message: " << camInfoMsg->header.stamp.nanosec
                                          << " nsec");

  infoPub->publish(*camInfoMsg);
}

void ZedCameraOne::publishImageWithInfo(
  const sl::Mat & img,
  const image_transport::Publisher & pubImg,
  const camInfoPub & infoPub,
  const camInfoPub & infoPubTrans,
  camInfoMsgPtr & camInfoMsg,
  const std::string & imgFrameId,
  const rclcpp::Time & t)
{
  auto image = sl_tools::imageToROSmsg(img, imgFrameId, t, _usePubTimestamps);
  DEBUG_STREAM_VD("Publishing IMAGE message: " << t.nanoseconds() << " nsec");
  try {
    publishCameraInfo(infoPub, camInfoMsg, image->header.stamp);
    publishCameraInfo(infoPubTrans, camInfoMsg, image->header.stamp);
    pubImg.publish(std::move(image));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic exception: ");
  }
}
#ifdef FOUND_ISAAC_ROS_NITROS
void ZedCameraOne::publishImageWithInfo(
  const sl::Mat & img,
  const nitrosImgPub & nitrosPubImg,
  const camInfoPub & infoPub,
  const camInfoPub & infoPubTrans,
  camInfoMsgPtr & camInfoMsg,
  const std::string & imgFrameId,
  const rclcpp::Time & t)
{
  DEBUG_STREAM_VD(" * Publishing NITROS IMAGE message: " << t.nanoseconds() << " nsec");
  try {
    size_t dpitch = img.getWidthBytes();
    size_t spitch = img.getStepBytes(sl::MEM::GPU); // SL Mat can be padded

    size_t dbuffer_size{spitch * img.getHeight()};
    void * dbuffer;
    CUDA_CHECK(cudaMalloc(&dbuffer, dbuffer_size));

    DEBUG_NITROS("Sent CUDA Image buffer with memory at: %p", dbuffer);

    // Copy data bytes to CUDA buffer
    CUDA_CHECK(
      cudaMemcpy2D(
        dbuffer,
        dpitch,
        img.getPtr<sl::uchar1>(sl::MEM::GPU),
        spitch,
        img.getWidth() * img.getPixelBytes(), img.getHeight(),
        cudaMemcpyDeviceToDevice));

    // Adding header data
    std_msgs::msg::Header header;
    header.stamp = _usePubTimestamps ? get_clock()->now() : t;
    header.frame_id = imgFrameId;

    auto encoding = img_encodings::BGRA8; // Default encoding
    if (img.getDataType() == sl::MAT_TYPE::U8_C1) {
      encoding = img_encodings::MONO8; // Mono image
    } else if (img.getDataType() == sl::MAT_TYPE::U8_C3) {
      encoding = img_encodings::BGR8; // BGR image
    } else if (img.getDataType() == sl::MAT_TYPE::F32_C1) {
      encoding = img_encodings::TYPE_32FC1; // Float image
    }

    // Create NitrosImage wrapping CUDA buffer
    nvidia::isaac_ros::nitros::NitrosImage nitros_image =
      nvidia::isaac_ros::nitros::NitrosImageBuilder()
      .WithHeader(header)
      .WithEncoding(encoding)
      .WithDimensions(img.getHeight(), img.getWidth())
      .WithGpuData(dbuffer)
      //.WithGpuData(img.getPtr<sl::uchar4>(sl::MEM::GPU)) // TODO: Enable direct GPU memory sharing when supported by Isaac ROS.
      .Build();

    nitrosPubImg->publish(nitros_image);
    publishCameraInfo(infoPub, camInfoMsg, header.stamp);
    publishCameraInfo(infoPubTrans, camInfoMsg, header.stamp);
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM(" * Message publishing exception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM(" * Message publishing generic exception: ");
  }
}
#endif

void ZedCameraOne::applyDynamicSettings()
{
  DEBUG_STREAM_COMM("=== Applying dynamic settings ===");

  if (_debugCommon) {
    DEBUG_COMM("Settings to apply: ");
    for (auto & param: _camDynParMapChanged) {
      if (param.second) {DEBUG_STREAM_COMM(" * " << param.first);}
    }
  }

  applySaturationSharpnessGamma();
  applyWhiteBalance();
  applyExposure();
  applyAnalogGain();
  applyDigitalGain();
  applyExposureCompensationAndDenoising();

  DEBUG_COMM("Settings yet to apply: ");
  int count = 0;
  for (auto & param: _camDynParMapChanged) {
    if (param.second) {
      count++;
      DEBUG_STREAM_COMM(" * " << param.first);
    }
  }
  if (count == 0) {
    DEBUG_COMM(" * NONE");
  }

  _triggerUpdateDynParams = (count > 0);

  DEBUG_STREAM_COMM("=== Dynamic settings applied ===");
}

// Helper function for saturation, sharpness, gamma
void ZedCameraOne::applySaturationSharpnessGamma()
{
  auto setVideoSetting = [this](sl::VIDEO_SETTINGS setting, int value,
      const std::string & settingName) {
      if (this->_camDynParMapChanged[settingName]) {
        sl::ERROR_CODE ret_code = _zed->setCameraSettings(setting, value);
        if (ret_code != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting "
              << settingName << ": "
              << sl::toString(ret_code));
        } else {
          this->_camDynParMapChanged[settingName] = false;
          DEBUG_STREAM_COMM("Set " << settingName << " to " << value);
        }
      }
    };

  setVideoSetting(sl::VIDEO_SETTINGS::SATURATION, _camSaturation, "video.saturation");
  setVideoSetting(sl::VIDEO_SETTINGS::SHARPNESS, _camSharpness, "video.sharpness");
  setVideoSetting(sl::VIDEO_SETTINGS::GAMMA, _camGamma, "video.gamma");
}

// Helper function for white balance
void ZedCameraOne::applyWhiteBalance()
{
  auto setVideoSetting = [this](sl::VIDEO_SETTINGS setting, int value,
      const std::string & settingName) {
      if (this->_camDynParMapChanged[settingName]) {
        sl::ERROR_CODE ret_code = _zed->setCameraSettings(setting, value);
        if (ret_code != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting "
              << settingName << ": "
              << sl::toString(ret_code));
        } else {
          this->_camDynParMapChanged[settingName] = false;
          DEBUG_STREAM_COMM("Set " << settingName << " to " << value);
        }
      }
    };

  setVideoSetting(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, _camAutoWB, "video.auto_whitebalance");
  if (!_camAutoWB) {
    setVideoSetting(
      sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, _camWBTemp * 100,
      "video.whitebalance_temperature");
    set_parameter(rclcpp::Parameter("video.auto_whitebalance", false));
  } else {
    _camDynParMapChanged["video.whitebalance_temperature"] = false;
  }
}

// Helper function for exposure
void ZedCameraOne::applyExposure()
{
  auto setVideoSetting = [this](sl::VIDEO_SETTINGS setting, int value,
      const std::string & settingName) {
      if (this->_camDynParMapChanged[settingName]) {
        sl::ERROR_CODE ret_code = _zed->setCameraSettings(setting, value);
        if (ret_code != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting "
              << settingName << ": "
              << sl::toString(ret_code));
        } else {
          this->_camDynParMapChanged[settingName] = false;
          DEBUG_STREAM_COMM("Set " << settingName << " to " << value);
        }
      }
    };

  auto setVideoSettingRange = [this](sl::VIDEO_SETTINGS setting, int value_min,
      int value_max, const std::string & settingName_min, const std::string & settingName_max) {
      if (this->_camDynParMapChanged[settingName_min] ||
        this->_camDynParMapChanged[settingName_max])
      {
        sl::ERROR_CODE ret_code = _zed->setCameraSettings(setting, value_min, value_max);
        if (ret_code != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting the range of ["
              << settingName_min << "," << settingName_max << ": "
              << sl::toString(ret_code));
          return false;
        } else {
          this->_camDynParMapChanged[settingName_min] = false;
          this->_camDynParMapChanged[settingName_max] = false;
          DEBUG_STREAM_COMM("Set " << settingName_min << " to " << value_min);
          DEBUG_STREAM_COMM("Set " << settingName_max << " to " << value_max);
          return true;
        }
        return false;
      }
      return true;
    };

  if (_camAutoExposure) {
    if (_camDynParMapChanged["video.auto_exposure"]) {
      set_parameters(
        {
          rclcpp::Parameter("video.auto_exposure_time_range_min", 28),
          rclcpp::Parameter("video.auto_exposure_time_range_max", 30000)
        });
      DEBUG_STREAM_COMM("Forced exposure range to [" << 28 << "," << 30000 << "]");
    }
    DEBUG_STREAM_COMM(
      "Set video.auto_exposure to "
        << (_camAutoExposure ? "TRUE" : "FALSE"));
  } else {
    setVideoSetting(
      sl::VIDEO_SETTINGS::EXPOSURE_TIME, _camExpTime,
      "video.exposure_time");
    set_parameters(
      {
        rclcpp::Parameter("video.auto_exposure", false),
        rclcpp::Parameter("video.auto_exposure_time_range_min", _camExpTime),
        rclcpp::Parameter("video.auto_exposure_time_range_max", _camExpTime)
      });
    DEBUG_STREAM_COMM(
      "Forced video.auto_exposure to false and exposure range to [" << _camExpTime << "," <<
        _camExpTime <<
        "]");
  }
  _camDynParMapChanged["video.auto_exposure"] = false;
  _camDynParMapChanged["video.exposure_time"] = false;

  if (setVideoSettingRange(
      sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE,
      _camAutoExpTimeRangeMin, _camAutoExpTimeRangeMax,
      "video.auto_exposure_time_range_min",
      "video.auto_exposure_time_range_max"))
  {
    if (_camAutoExpTimeRangeMin == _camAutoExpTimeRangeMax) {
      set_parameters(
        {
          rclcpp::Parameter("video.auto_exposure", false),
          rclcpp::Parameter("video.exposure_time", _camAutoExpTimeRangeMin)
        });
      DEBUG_STREAM_COMM(
        "Forced video.auto_exposure to false and video.exposure_time to " <<
          _camAutoExpTimeRangeMin);
    } else {
      set_parameter(rclcpp::Parameter("video.auto_exposure", true));
      DEBUG_STREAM_COMM("Forced video.auto_exposure to true");
    }
  }
}

// Helper function for analog gain
void ZedCameraOne::applyAnalogGain()
{
  auto setVideoSetting = [this](sl::VIDEO_SETTINGS setting, int value,
      const std::string & settingName) {
      if (this->_camDynParMapChanged[settingName]) {
        sl::ERROR_CODE ret_code = _zed->setCameraSettings(setting, value);
        if (ret_code != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting "
              << settingName << ": "
              << sl::toString(ret_code));
        } else {
          this->_camDynParMapChanged[settingName] = false;
          DEBUG_STREAM_COMM("Set " << settingName << " to " << value);
        }
      }
    };

  auto setVideoSettingRange = [this](sl::VIDEO_SETTINGS setting, int value_min,
      int value_max, const std::string & settingName_min, const std::string & settingName_max) {
      if (this->_camDynParMapChanged[settingName_min] ||
        this->_camDynParMapChanged[settingName_max])
      {
        sl::ERROR_CODE ret_code = _zed->setCameraSettings(setting, value_min, value_max);
        if (ret_code != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting the range of ["
              << settingName_min << "," << settingName_max << ": "
              << sl::toString(ret_code));
          return false;
        } else {
          this->_camDynParMapChanged[settingName_min] = false;
          this->_camDynParMapChanged[settingName_max] = false;
          DEBUG_STREAM_COMM("Set " << settingName_min << " to " << value_min);
          DEBUG_STREAM_COMM("Set " << settingName_max << " to " << value_max);
          return true;
        }
        return false;
      }
      return true;
    };

  if (_camAutoAnalogGain) {
    if (_camDynParMapChanged["video.auto_analog_gain"]) {
      set_parameters(
        {
          rclcpp::Parameter("video.auto_analog_gain_range_min", 1000),
          rclcpp::Parameter("video.auto_analog_gain_range_max", 16000)
        });
      DEBUG_STREAM_COMM("Forced analog gain range to [" << 1000 << "," << 16000 << "]");
    }
    DEBUG_STREAM_COMM(
      "Set video.auto_analog_gain to "
        << (_camAutoAnalogGain ? "TRUE" : "FALSE"));
  } else {
    setVideoSetting(
      sl::VIDEO_SETTINGS::ANALOG_GAIN, _camAnalogGain, "video.analog_gain");
    set_parameters(
      {
        rclcpp::Parameter("video.auto_analog_gain", false),
        rclcpp::Parameter("video.auto_analog_gain_range_min", _camAnalogGain),
        rclcpp::Parameter("video.auto_analog_gain_range_max", _camAnalogGain)
      });
    DEBUG_STREAM_COMM(
      "Forced video.auto_analog_gain to false and analog gain range to [" << _camAnalogGain <<
        "," << _camAnalogGain <<
        "]");
  }
  _camDynParMapChanged["video.auto_analog_gain"] = false;
  _camDynParMapChanged["video.analog_gain"] = false;

  if (setVideoSettingRange(
      sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE,
      _camAutoAnalogGainRangeMin, _camAutoAnalogGainRangeMax,
      "video.auto_analog_gain_range_min",
      "video.auto_analog_gain_range_max"))
  {
    if (_camAutoAnalogGainRangeMin == _camAutoAnalogGainRangeMax) {
      set_parameters(
        {
          rclcpp::Parameter("video.auto_analog_gain", false),
          rclcpp::Parameter("video.analog_gain", _camAutoAnalogGainRangeMin)
        });
      DEBUG_STREAM_COMM(
        "Forced video.auto_analog_gain to false and video.analog_gain to " <<
          _camAutoAnalogGainRangeMin);
    } else {
      set_parameter(rclcpp::Parameter("video.auto_analog_gain", true));
      DEBUG_STREAM_COMM("Forced video.auto_analog_gain to true");
    }
  }
}

// Helper function for digital gain
void ZedCameraOne::applyDigitalGain()
{
  auto setVideoSetting = [this](sl::VIDEO_SETTINGS setting, int value,
      const std::string & settingName) {
      if (this->_camDynParMapChanged[settingName]) {
        sl::ERROR_CODE ret_code = _zed->setCameraSettings(setting, value);
        if (ret_code != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting "
              << settingName << ": "
              << sl::toString(ret_code));
        } else {
          this->_camDynParMapChanged[settingName] = false;
          DEBUG_STREAM_COMM("Set " << settingName << " to " << value);
        }
      }
    };

  auto setVideoSettingRange = [this](sl::VIDEO_SETTINGS setting, int value_min,
      int value_max, const std::string & settingName_min, const std::string & settingName_max) {
      if (this->_camDynParMapChanged[settingName_min] ||
        this->_camDynParMapChanged[settingName_max])
      {
        sl::ERROR_CODE ret_code = _zed->setCameraSettings(setting, value_min, value_max);
        if (ret_code != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting the range of ["
              << settingName_min << "," << settingName_max << ": "
              << sl::toString(ret_code));
          return false;
        } else {
          this->_camDynParMapChanged[settingName_min] = false;
          this->_camDynParMapChanged[settingName_max] = false;
          DEBUG_STREAM_COMM("Set " << settingName_min << " to " << value_min);
          DEBUG_STREAM_COMM("Set " << settingName_max << " to " << value_max);
          return true;
        }
        return false;
      }
      return true;
    };

  if (_camAutoDigitalGain) {
    if (_camDynParMapChanged["video.auto_digital_gain"]) {
      set_parameters(
        {
          rclcpp::Parameter("video.auto_digital_gain_range_min", 1),
          rclcpp::Parameter("video.auto_digital_gain_range_max", 256)
        });
      DEBUG_STREAM_COMM("Forced digital gain range to [" << 1 << "," << 256 << "]");
    }
    DEBUG_STREAM_COMM(
      "Set video.auto_digital_gain to "
        << (_camAutoDigitalGain ? "TRUE" : "FALSE"));
  } else {
    setVideoSetting(
      sl::VIDEO_SETTINGS::DIGITAL_GAIN, _camDigitalGain, "video.digital_gain");
    set_parameters(
      {
        rclcpp::Parameter("video.auto_digital_gain", false),
        rclcpp::Parameter("video.auto_digital_gain_range_min", _camDigitalGain),
        rclcpp::Parameter("video.auto_digital_gain_range_max", _camDigitalGain)
      });
    DEBUG_STREAM_COMM(
      "Forced video.auto_digital_gain to false and digital gain range to [" << _camDigitalGain <<
        "," << _camDigitalGain <<
        "]");
  }
  _camDynParMapChanged["video.auto_digital_gain"] = false;
  _camDynParMapChanged["video.digital_gain"] = false;

  if (setVideoSettingRange(
      sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE,
      _camAutoDigitalGainRangeMin, _camAutoDigitalGainRangeMax,
      "video.auto_digital_gain_range_min",
      "video.auto_digital_gain_range_max"))
  {
    if (_camAutoDigitalGainRangeMin == _camAutoDigitalGainRangeMax) {
      set_parameters(
        {
          rclcpp::Parameter("video.auto_digital_gain", false),
          rclcpp::Parameter("video.digital_gain", _camAutoDigitalGainRangeMin)
        });
      DEBUG_STREAM_COMM(
        "Forced video.auto_digital_gain to false and video.digital_gain to " <<
          _camAutoDigitalGainRangeMin);
    } else {
      set_parameter(rclcpp::Parameter("video.auto_digital_gain", true));
      DEBUG_STREAM_COMM("Forced video.auto_digital_gain to true");
    }
  }
}

// Helper function for exposure compensation and denoising
void ZedCameraOne::applyExposureCompensationAndDenoising()
{
  auto setVideoSetting = [this](sl::VIDEO_SETTINGS setting, int value,
      const std::string & settingName) {
      if (this->_camDynParMapChanged[settingName]) {
        sl::ERROR_CODE ret_code = _zed->setCameraSettings(setting, value);
        if (ret_code != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Error setting "
              << settingName << ": "
              << sl::toString(ret_code));
        } else {
          this->_camDynParMapChanged[settingName] = false;
          DEBUG_STREAM_COMM("Set " << settingName << " to " << value);
        }
      }
    };

  setVideoSetting(
    sl::VIDEO_SETTINGS::EXPOSURE_COMPENSATION, _camExposureComp,
    "video.exposure_compensation");

  setVideoSetting(sl::VIDEO_SETTINGS::DENOISING, _camDenoising, "video.denoising");
}

}
