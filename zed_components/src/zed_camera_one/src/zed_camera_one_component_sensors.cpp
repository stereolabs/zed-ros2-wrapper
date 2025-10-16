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

using namespace std::chrono_literals;

namespace stereolabs
{

void ZedCameraOne::getSensorsParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "=== SENSORS parameters ===");

  sl_tools::getParam(
    shared_from_this(), "sensors.publish_imu_tf",
    _publishSensImuTF, _publishSensImuTF, " * Publish IMU TF: ");
  sl_tools::getParam(
    shared_from_this(), "sensors.sensors_pub_rate",
    _sensPubRate, _sensPubRate,
    " * Sensors publishing rate [Hz]: ", true, 1.0, 400.0);
}

void ZedCameraOne::initSensorPublishers()
{
  RCLCPP_INFO(get_logger(), " +++ SENSORS TOPICS +++");

  // ----> Advertised topics
  const std::string imu_topic_root = "imu/";
  const std::string imu_topic = imu_topic_root + "data";
  const std::string imu_raw_topic = imu_topic_root + "data_raw";
  const std::string temp_topic = "temperature";

  // Helper to build topic names
  auto make_topic =
    [&](const std::string & type) {
      std::string topic = _topicRoot + type;
      return get_node_topics_interface()->resolve_topic_name(topic);
    };

  _sensImuTopic = make_topic(imu_topic);
  _sensImuRawTopic = make_topic(imu_raw_topic);
  _sensTempTopic = make_topic(temp_topic);
  // <---- Advertised topics

  // ----> Create publishers

  // Sensors publishers
  if (_publishSensImu) {
    _pubImu = this->create_publisher<sensor_msgs::msg::Imu>(_sensImuTopic, _qos, _pubOpt);
    RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubImu->get_topic_name());
  }

  if (_publishSensImuRaw) {
    _pubImuRaw = this->create_publisher<sensor_msgs::msg::Imu>(_sensImuRawTopic, _qos, _pubOpt);
    RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubImuRaw->get_topic_name());
  }

  if (_publishSensTemp) {
    _pubTemp = this->create_publisher<sensor_msgs::msg::Temperature>(_sensTempTopic, _qos, _pubOpt);
    RCLCPP_INFO_STREAM(get_logger(), "  * Advertised on topic: " << _pubTemp->get_topic_name());
  }
  // <---- Create publishers
}

void ZedCameraOne::threadFunc_pubSensorsData()
{
  DEBUG_STREAM_SENS("Sensors thread started");
  setupSensorThreadScheduling();

  DEBUG_STREAM_SENS("Sensors thread loop starting...");
  _lastTs_imu = TIMEZERO_ROS;

  while (true) {
    if (handleSensorThreadInterruption()) {break;}
    if (!waitForCameraOpen()) {continue;}
    if (!waitForSensorSubscribers()) {continue;}
    if (!handleSensorPublishing()) {continue;}
    adjustSensorPublishingFrequency();
  }

  DEBUG_STREAM_SENS("Sensors thread finished");
}

// Helper: Setup thread scheduling for sensors thread
void ZedCameraOne::setupSensorThreadScheduling()
{
  DEBUG_STREAM_ADV("Sensors thread settings");
  if (_debugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        " ! Failed to get thread policy! - " << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * Default Sensors thread (#" << pthread_self() << ") settings - Policy: "
                                       << sl_tools::threadSched2Str(
          policy).c_str() << " - Priority: " << par.sched_priority);
    }
  }

  sched_param par;
  par.sched_priority =
    (_threadSchedPolicy == "SCHED_FIFO" ||
    _threadSchedPolicy == "SCHED_RR") ? _threadPrioSens : 0;
  int sched_policy = SCHED_OTHER;
  if (_threadSchedPolicy == "SCHED_BATCH") {
    sched_policy = SCHED_BATCH;
  } else if (_threadSchedPolicy == "SCHED_FIFO") {
    sched_policy = SCHED_FIFO;
  } else if (_threadSchedPolicy == "SCHED_RR") {sched_policy = SCHED_RR;}

  if (pthread_setschedparam(pthread_self(), sched_policy, &par)) {
    RCLCPP_WARN_STREAM(get_logger(), " ! Failed to set thread params! - " << std::strerror(errno));
  }

  if (_debugAdvanced) {
    int policy;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        " ! Failed to get thread policy! - " << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * New Sensors thread (#" << pthread_self() << ") settings - Policy: "
                                   << sl_tools::threadSched2Str(
          policy).c_str() << " - Priority: " << par.sched_priority);
    }
  }
}

// Helper: Handle thread interruption and shutdown
bool ZedCameraOne::handleSensorThreadInterruption()
{
  try {
    if (!rclcpp::ok()) {
      DEBUG_STREAM_SENS("Ctrl+C received: stopping sensors thread");
      _threadStop = true;
      return true;
    }
    if (_threadStop) {
      DEBUG_STREAM_SENS("[threadFunc_pubSensorsData] (2): Sensors thread stopped");
      return true;
    }
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_COMM("[threadFunc_pubSensorsData] Generic exception.");
    return false;
  }
  return false;
}

// Helper: Wait for camera to be open
bool ZedCameraOne::waitForCameraOpen()
{
  if (!_zed->isOpened()) {
    DEBUG_STREAM_SENS("[threadFunc_pubSensorsData] the camera is not open");
    rclcpp::sleep_for(200ms);
    return false;
  }
  return true;
}

// Helper: Wait for sensor topic subscribers
bool ZedCameraOne::waitForSensorSubscribers()
{
  _imuPublishing = areSensorsTopicsSubscribed();
  if (!_imuPublishing && !_publishSensImuTF) {
    rclcpp::sleep_for(200ms);
    return false;
  }
  return true;
}

// Helper: Handle sensor publishing and sleep if needed
bool ZedCameraOne::handleSensorPublishing()
{
  if (!publishSensorsData()) {
    auto sleep_msec = static_cast<int>(_sensRateComp * (1000. / _sensPubRate));
    sleep_msec = std::max(1, sleep_msec);
    DEBUG_STREAM_SENS("[threadFunc_pubSensorsData] Thread sleep: " << sleep_msec << " msec");
    rclcpp::sleep_for(std::chrono::milliseconds(sleep_msec));
    return false;
  }
  return true;
}

// Helper: Adjust publishing frequency compensation
void ZedCameraOne::adjustSensorPublishingFrequency()
{
  double avg_freq = 1. / _imuPeriodMean_sec->getAvg();
  double err = std::fabs(_sensPubRate - avg_freq);
  const double COMP_P_GAIN = 0.0005;

  if (avg_freq < _sensPubRate) {
    _sensRateComp -= COMP_P_GAIN * err;
  } else if (avg_freq > _sensPubRate) {
    _sensRateComp += COMP_P_GAIN * err;
  }

  _sensRateComp = std::max(0.05, _sensRateComp);
  _sensRateComp = std::min(2.0, _sensRateComp);
  DEBUG_STREAM_SENS("[threadFunc_pubSensorsData] _sensRateComp: " << _sensRateComp);
}

void ZedCameraOne::startTempPubTimer()
{
  if (_tempPubTimer != nullptr) {
    _tempPubTimer->cancel();
  }

  std::chrono::milliseconds pubPeriod_msec(static_cast<int>(1000.0));
  _tempPubTimer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(pubPeriod_msec),
    std::bind(&ZedCameraOne::callback_pubTemp, this));
}

void ZedCameraOne::callback_pubTemp()
{
  DEBUG_STREAM_ONCE_SENS("Temperatures callback called");

  if (_grabStatus != sl::ERROR_CODE::SUCCESS) {
    DEBUG_SENS("Camera not ready");
    return;
  }

  // ----> Always update temperature values for diagnostic
  sl::SensorsData sens_data;
  sl::ERROR_CODE err = _zed->getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT);
  if (err != sl::ERROR_CODE::SUCCESS) {
    DEBUG_STREAM_SENS(
      "[callback_pubTemp] sl::getSensorsData error: "
        << sl::toString(err).c_str());
    return;
  }

  sens_data.temperature.get(
    sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU, _tempImu);
  DEBUG_STREAM_SENS("Camera temperature: " << _tempImu << "°C");
  // <---- Always update temperature values for diagnostic

  // ----> Subscribers count
  size_t tempSubCount = 0;

  try {
    if (_pubTemp) {
      tempSubCount = count_subscribers(_pubTemp->get_topic_name());
      DEBUG_STREAM_SENS("Temperature subscribers: " << static_cast<int>(tempSubCount));
    }
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_SENS(
      "callback_pubTemp: Exception while counting subscribers");
    return;
  }
  // <---- Subscribers count

  // ----> Publish temperature
  if (tempSubCount > 0) {
    auto imuTempMsg = std::make_unique<sensor_msgs::msg::Temperature>();

    imuTempMsg->header.stamp = get_clock()->now();

    imuTempMsg->header.frame_id = _imuFrameId;
    imuTempMsg->temperature = static_cast<double>(_tempImu);
    imuTempMsg->variance = 0.0;

    DEBUG_SENS("Publishing IMU TEMP message");
    try {
      if (_pubTemp) {
        _pubTemp->publish(std::move(imuTempMsg));
      }
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
    }
  }
  // <---- Publish temperature
}

bool ZedCameraOne::publishSensorsData()
{
  if (_grabStatus != sl::ERROR_CODE::SUCCESS) {
    DEBUG_SENS("Camera not ready");
    return false;
  }

  sl::SensorsData sens_data;
  sl::ERROR_CODE err = _zed->getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT);
  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "[publishSensorsData] sl::getSensorsData error: " << sl::toString(err).c_str());
    return false;
  }

  rclcpp::Time ts_imu = sl_tools::slTime2Ros(sens_data.imu.timestamp);
  double dT = ts_imu.seconds() - _lastTs_imu.seconds();
  _lastTs_imu = ts_imu;
  bool new_imu_data = (dT > 0.0);

  if (!new_imu_data) {
    DEBUG_STREAM_SENS("[publishSensorsData] No new sensors data");
    return false;
  }

  updateImuFreqDiagnostics(dT);

  publishImuFrameAndTopic();

  if (_imuSubCount > 0) {
    publishImuMsg(ts_imu, sens_data);
  }

  if (_imuRawSubCount > 0) {
    publishImuRawMsg(ts_imu, sens_data);
  }

  return true;
}

void ZedCameraOne::updateImuFreqDiagnostics(double dT)
{
  DEBUG_STREAM_SENS("SENSOR LAST PERIOD: " << dT << " sec @" << 1. / dT << " Hz");
  auto elapsed = _imuFreqTimer.toc();
  _imuFreqTimer.tic();
  double mean = _imuPeriodMean_sec->addValue(elapsed);
  _pubImu_sec->addValue(mean);
  DEBUG_STREAM_SENS("IMU MEAN freq: " << 1. / mean);
}

void ZedCameraOne::publishImuMsg(const rclcpp::Time & ts_imu, const sl::SensorsData & sens_data)
{
  if (!_pubImu) {
    DEBUG_STREAM_SENS("[publishSensorsData] _pubImu is null");
    return;
  }

  DEBUG_STREAM_SENS("[publishSensorsData] IMU subscribers: " << static_cast<int>(_imuSubCount));
  auto imuMsg = std::make_unique<sensor_msgs::msg::Imu>();
  imuMsg->header.stamp = ts_imu;
  imuMsg->header.frame_id = _imuFrameId;

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

  for (int i = 0; i < 3; ++i) {
    int r = i;
    imuMsg->orientation_covariance[i * 3 + 0] = sens_data.imu.pose_covariance.r[r * 3 + 0] *
      DEG2RAD * DEG2RAD;
    imuMsg->orientation_covariance[i * 3 + 1] = sens_data.imu.pose_covariance.r[r * 3 + 1] *
      DEG2RAD * DEG2RAD;
    imuMsg->orientation_covariance[i * 3 + 2] = sens_data.imu.pose_covariance.r[r * 3 + 2] *
      DEG2RAD * DEG2RAD;

    imuMsg->linear_acceleration_covariance[i * 3 +
      0] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
    imuMsg->linear_acceleration_covariance[i * 3 +
      1] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
    imuMsg->linear_acceleration_covariance[i * 3 +
      2] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];

    imuMsg->angular_velocity_covariance[i * 3 +
      0] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
    imuMsg->angular_velocity_covariance[i * 3 +
      1] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
    imuMsg->angular_velocity_covariance[i * 3 +
      2] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
  }

  try {
    _pubImu->publish(std::move(imuMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic exception: ");
  }
}

void ZedCameraOne::publishImuRawMsg(const rclcpp::Time & ts_imu, const sl::SensorsData & sens_data)
{
  if (!_pubImuRaw) {
    DEBUG_STREAM_SENS("[publishSensorsData] _pubImuRaw is null");
    return;
  }

  DEBUG_STREAM_SENS(
    "[publishSensorsData] IMU RAW subscribers: " << static_cast<int>(_imuRawSubCount));
  auto imuRawMsg = std::make_unique<sensor_msgs::msg::Imu>();
  imuRawMsg->header.stamp = ts_imu;
  imuRawMsg->header.frame_id = _imuFrameId;

  imuRawMsg->angular_velocity.x = sens_data.imu.angular_velocity_uncalibrated[0] * DEG2RAD;
  imuRawMsg->angular_velocity.y = sens_data.imu.angular_velocity_uncalibrated[1] * DEG2RAD;
  imuRawMsg->angular_velocity.z = sens_data.imu.angular_velocity_uncalibrated[2] * DEG2RAD;

  imuRawMsg->linear_acceleration.x = sens_data.imu.linear_acceleration_uncalibrated[0];
  imuRawMsg->linear_acceleration.y = sens_data.imu.linear_acceleration_uncalibrated[1];
  imuRawMsg->linear_acceleration.z = sens_data.imu.linear_acceleration_uncalibrated[2];

  for (int i = 0; i < 3; ++i) {
    int r = i;
    imuRawMsg->linear_acceleration_covariance[i * 3 +
      0] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
    imuRawMsg->linear_acceleration_covariance[i * 3 +
      1] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
    imuRawMsg->linear_acceleration_covariance[i * 3 +
      2] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];

    imuRawMsg->angular_velocity_covariance[i * 3 +
      0] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
    imuRawMsg->angular_velocity_covariance[i * 3 +
      1] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
    imuRawMsg->angular_velocity_covariance[i * 3 +
      2] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
  }

  try {
    _pubImuRaw->publish(std::move(imuRawMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic exception: ");
  }
}

void ZedCameraOne::publishImuFrameAndTopic()
{
  if (!_publishSensImuTF && !_publishSensImuTransf) {
    return;
  }

  sl::Orientation sl_rot = _slCamImuTransf.getOrientation();
  sl::Translation sl_tr = _slCamImuTransf.getTranslation();

  auto cameraImuTransfMsg = std::make_unique<geometry_msgs::msg::TransformStamped>();

  cameraImuTransfMsg->header.stamp = get_clock()->now();
  cameraImuTransfMsg->header.frame_id = _camImgFrameId;
  cameraImuTransfMsg->child_frame_id = _imuFrameId;

  cameraImuTransfMsg->transform.rotation.x = sl_rot.ox;
  cameraImuTransfMsg->transform.rotation.y = sl_rot.oy;
  cameraImuTransfMsg->transform.rotation.z = sl_rot.oz;
  cameraImuTransfMsg->transform.rotation.w = sl_rot.ow;

  cameraImuTransfMsg->transform.translation.x = sl_tr.x;
  cameraImuTransfMsg->transform.translation.y = sl_tr.y;
  cameraImuTransfMsg->transform.translation.z = sl_tr.z;

  // ----> Publish CAM/IMU Transform
  if (_publishSensImuTransf) {
    try {
      size_t sub_count = 0;
      if (_pubCamImuTransf) {
        sub_count = count_subscribers(_pubCamImuTransf->get_topic_name());
        DEBUG_STREAM_SENS("Camera-IMU Transform subscribers: " << static_cast<int>(sub_count));
      }

      if (sub_count && _pubCamImuTransf) {
        _pubCamImuTransf->publish(std::move(cameraImuTransfMsg));
      }
    } catch (const std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception.");
    }
  }
  // <---- Publish CAM/IMU Transform

  // ----> Broadcast CAM/IMU TF
  if (!_publishSensImuTF) {
    return;
  }

  auto transformStamped = std::make_unique<geometry_msgs::msg::TransformStamped>();

  transformStamped->header.stamp = get_clock()->now();
  transformStamped->header.frame_id = _camImgFrameId;
  transformStamped->child_frame_id = _imuFrameId;

  transformStamped->transform.rotation.x = sl_rot.ox;
  transformStamped->transform.rotation.y = sl_rot.oy;
  transformStamped->transform.rotation.z = sl_rot.oz;
  transformStamped->transform.rotation.w = sl_rot.ow;

  transformStamped->transform.translation.x = sl_tr.x;
  transformStamped->transform.translation.y = sl_tr.y;
  transformStamped->transform.translation.z = sl_tr.z;

  _tfBroadcaster->sendTransform(*transformStamped);

  double elapsed_sec = _imuTfFreqTimer.toc();
  _pubImuTF_sec->addValue(elapsed_sec);
  _imuTfFreqTimer.tic();
  // <---- Broadcast CAM/IMU TF
}

bool ZedCameraOne::areSensorsTopicsSubscribed()
{
  try {
    if (_pubImu) {
      _imuSubCount = _pubImu->get_subscription_count();
    } else {
      _imuSubCount = 0;
    }
    if (_pubImuRaw) {
      _imuRawSubCount = _pubImuRaw->get_subscription_count();
    } else {
      _imuRawSubCount = 0;
    }
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_SENS(
      "areSensorsTopicsSubscribed: Exception while counting subscribers");
    return false;
  }

  DEBUG_STREAM_SENS(
    "[areSensorsTopicsSubscribed] IMU subscribers: " << _imuSubCount);
  DEBUG_STREAM_SENS(
    "[areSensorsTopicsSubscribed] IMU RAW subscribers: " << _imuRawSubCount);

  return (_imuSubCount + _imuRawSubCount) > 0;
}

} // namespace stereolabs
