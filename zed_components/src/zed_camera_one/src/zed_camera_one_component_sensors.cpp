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

  // Set the name of the pubSensorsData thread for easier identification in
  // system monitors
  pthread_setname_np(pthread_self(), (get_name() + std::string("_pubSensorsData")).c_str());
  setupSensorThreadScheduling();

  DEBUG_STREAM_SENS("Sensors thread loop starting...");
  _lastTs_imu = TIMEZERO_ROS;

  constexpr auto SVO_PAUSE_POLL_INTERVAL =
    100ms;    // Poll interval when SVO is paused

  while (true) {
    if (handleSensorThreadInterruption()) {break;}

    if (_svoMode && _svoPause) {
      if (!_grabImuOnce) {
        rclcpp::sleep_for(SVO_PAUSE_POLL_INTERVAL);
        continue;
      } else {
        _grabImuOnce = false;  // Reset the flag and grab the IMU data
      }
    }

    if (!waitForCameraOpen()) {continue;}
    if (!waitForSensorSubscribers()) {continue;}
    handleSensorPublishing();
  }

  DEBUG_STREAM_SENS("Sensors thread finished");
}

// Helper: Setup thread scheduling for sensors thread
void ZedCameraOne::setupSensorThreadScheduling()
{
  if (_changeThreadSched) {
    DEBUG_STREAM_ADV("Sensors thread settings");
    if (_debugAdvanced) {
      int policy;
      sched_param par;
      if (pthread_getschedparam(pthread_self(), &policy, &par)) {
        RCLCPP_WARN_STREAM(
          get_logger(), " ! Failed to get thread policy! - "
            << std::strerror(errno));
      } else {
        DEBUG_STREAM_ADV(
          " * Default Sensors thread (#"
            << pthread_self() << ") settings - Policy: "
            << sl_tools::threadSched2Str(policy).c_str()
            << " - Priority: " << par.sched_priority);
      }
    }

    sched_param par;
    par.sched_priority =
      (_threadSchedPolicy == "SCHED_FIFO" || _threadSchedPolicy == "SCHED_RR") ?
      _threadPrioSens :
      0;
    int sched_policy = SCHED_OTHER;
    if (_threadSchedPolicy == "SCHED_BATCH") {
      sched_policy = SCHED_BATCH;
    } else if (_threadSchedPolicy == "SCHED_FIFO") {
      sched_policy = SCHED_FIFO;
    } else if (_threadSchedPolicy == "SCHED_RR") {
      sched_policy = SCHED_RR;
    }

    if (pthread_setschedparam(pthread_self(), sched_policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }

    if (_debugAdvanced) {
      int policy;
      if (pthread_getschedparam(pthread_self(), &policy, &par)) {
        RCLCPP_WARN_STREAM(
          get_logger(), " ! Failed to get thread policy! - "
            << std::strerror(errno));
      } else {
        DEBUG_STREAM_ADV(
          " * New Sensors thread (#"
            << pthread_self() << ") settings - Policy: "
            << sl_tools::threadSched2Str(policy).c_str()
            << " - Priority: " << par.sched_priority);
      }
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
  // The sensors thread polls at several kHz: refresh the subscriber counts on a
  // timer rather than on every iteration. While nothing is subscribed the thread
  // sleeps 200 ms below, so a new subscriber is still picked up promptly.
  auto sub_count_now = std::chrono::steady_clock::now();
  if (!_sensSubCountInit ||
    std::chrono::duration<double>(sub_count_now - _sensSubCountLastCheck).count() >=
    SENS_SUB_COUNT_REFRESH_SEC)
  {
    _imuPublishing = areSensorsTopicsSubscribed();
    _sensSubCountLastCheck = sub_count_now;
    _sensSubCountInit = true;
  }
  if (!_imuPublishing && !_publishSensImuTF) {
    rclcpp::sleep_for(200ms);
    return false;
  }
  return true;
}

// Helper: Drain the IMU FIFO and sleep a fixed poll period
bool ZedCameraOne::handleSensorPublishing()
{
  publishSensorsData();

  // In live mode publishSensorsData() reads the newest sample with
  // getSensorsData(TIME_REFERENCE::CURRENT), so the poll must run FASTER than
  // the hardware ODR to catch every sample: GMSL cameras deliver the IMU in
  // tight bursts and a poll at the ODR loses most of the samples for good
  // (the cause of the unstable rate in issues #249 and #445). Oversampling by
  // IMU_POLL_OVERSAMPLING captures the whole stream and keeps the publish delay
  // around one millisecond, independently of the grab rate.
  // SVO and simulation still drain the FIFO with getSensorsDataBatch(), for
  // which one poll per sample period is enough.
  // The output rate is set by decimation, not by this period.
  double poll_rate;
  if (!_svoMode && !_simMode && _imuOdr > 0.0) {
    poll_rate = std::min(_imuOdr * IMU_POLL_OVERSAMPLING, IMU_POLL_MAX_HZ);
  } else {
    poll_rate = (_imuOdr > 0.0) ? _imuOdr : _sensPubRate;
  }
  int poll_usec = static_cast<int>(1000000. / poll_rate);
  poll_usec = std::max(100, poll_usec);
  DEBUG_STREAM_SENS("[threadFunc_pubSensorsData] Poll period: " << poll_usec << " usec");
  rclcpp::sleep_for(std::chrono::microseconds(poll_usec));
  return true;
}

void ZedCameraOne::startTempPubTimer()
{
  if (_tempPubTimer != nullptr) {
    _tempPubTimer->cancel();
  }

  std::chrono::milliseconds pubPeriod_msec(TEMP_PUB_INTERVAL_MS);
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
    // Only warn if not in SVO mode or if the error is not a benign sensor
    // unavailability
    if (!_svoMode || err != sl::ERROR_CODE::SENSORS_NOT_AVAILABLE) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "[callback_pubTemp] sl::getSensorsData error: "
          << sl::toString(err).c_str());
    }
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
      tempSubCount = _pubTemp->get_subscription_count();
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

  // ----> Live mode: read the IMU decoupled from grab()
  // getSensorsDataBatch() only returns the samples attached to the most recent
  // grabbed frame, so its latency is tied to the grab cadence rather than to the
  // IMU's own rate: with `general.grab_frame_rate` at 15 the samples arrived in
  // 15 Hz bursts, ~100 ms late, even though the IMU keeps running at its own ODR
  // (this node has no compute capping; the stereo node, which does, was hit much
  // harder). getSensorsData(TIME_REFERENCE::CURRENT) reads the newest sample
  // straight from the sensors stream, so the publish delay stays around one
  // millisecond whatever the grab rate is. Every sample is still captured: the
  // sensors thread polls far above the ODR (see IMU_POLL_OVERSAMPLING), because
  // GMSL cameras deliver the IMU in bursts and a slow poll drops most of them -
  // the unstable rate reported in issues #249 and #445.
  if (!_svoMode && !_simMode) {
    sl::SensorsData sens_data;
    sl::ERROR_CODE err = _zed->getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "[publishSensorsData] sl::getSensorsData error: " << sl::toString(err).c_str());
      return false;
    }

    rclcpp::Time ts_imu = sl_tools::slTime2Ros(sens_data.imu.timestamp);

    // The poll runs faster than the IMU ODR, so the same sample is read back
    // several times in a row: only a brand new hardware timestamp feeds the
    // decimator, otherwise the accumulator would count one sample many times.
    if (_lastSeenTs_imu != TIMEZERO_ROS &&
      ts_imu.seconds() <= _lastSeenTs_imu.seconds())
    {
      return false;
    }
    // Track the real interval between samples. The ODR advertised by the SDK is
    // not always the rate the IMU actually delivers (a ZED X One GS reports
    // 400 Hz and delivers 200 Hz), and decimating against the advertised value
    // then halves the output rate, so `sensors.sensors_pub_rate` is not honored.
    if (_lastSeenTs_imu != TIMEZERO_ROS) {
      double dt = ts_imu.seconds() - _lastSeenTs_imu.seconds();
      if (dt > 0.0 && dt < 1.0) {
        _imuSamplePeriod =
          (_imuSamplePeriod > 0.0) ? (0.99 * _imuSamplePeriod + 0.01 * dt) : dt;
      }
    }
    _lastSeenTs_imu = ts_imu;

    // Decimate against the measured rate, falling back to the advertised ODR
    // until enough samples have been seen to measure it.
    double sample_rate = (_imuSamplePeriod > 0.0) ? (1.0 / _imuSamplePeriod) : _imuOdr;
    double decim_ratio = 1.0;
    if (sample_rate > 0.0 && _sensPubRate > 0.0 && _sensPubRate < sample_rate) {
      decim_ratio = _sensPubRate / sample_rate;
    }

    _imuDecimAccum += decim_ratio;
    if (_imuDecimAccum < 1.0) {
      return false;
    }
    _imuDecimAccum -= 1.0;

    double dT = ts_imu.seconds() - _lastTs_imu.seconds();
    _lastTs_imu = ts_imu;

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
  // <---- Live mode: read the IMU decoupled from grab()

  // ----> SVO and simulation: drain the whole IMU FIFO
  // Neither is subject to the grab-compute capping, so draining the batch is
  // both correct and cheap here, and it guarantees no sample is dropped.
  std::vector<sl::SensorsData> sens_data_batch;
  sl::ERROR_CODE err = _zed->getSensorsDataBatch(sens_data_batch);
  if (err != sl::ERROR_CODE::SUCCESS) {
    // Only warn if the input is a live camera or if the error is not a benign
    // sensor unavailability
    if ((!_svoMode && !_simMode) || err != sl::ERROR_CODE::SENSORS_NOT_AVAILABLE) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "[publishSensorsData] sl::getSensorsDataBatch error: " << sl::toString(err).c_str());
    }
    return false;
  }

  if (sens_data_batch.empty()) {
    DEBUG_STREAM_SENS("[publishSensorsData] No new sensors data");
    return false;
  }

  // In simulation with `use_sim_time`, the timestamps carried by the stream do
  // not belong to the simulation timeline, so every sample must be stamped with
  // the current ROS (simulation) time. All the samples drained by a single call
  // would then share the same stamp, so keep only the most recent one: the
  // duplicate and decimation gates below cannot tell apart samples that have no
  // distinct timestamps.
  if (_simMode && _useSimTime && sens_data_batch.size() > 1) {
    sens_data_batch.erase(sens_data_batch.begin(), sens_data_batch.end() - 1);
  }

  // Decimate the drained IMU stream down to the requested
  // `sensors.sensors_pub_rate`. The FIFO is filled at the camera's hardware ODR;
  // a fractional accumulator selects samples as uniformly as possible so the
  // average output rate matches _sensPubRate (capped at the hardware ODR), while
  // every published sample keeps its real hardware timestamp. If the ODR is
  // unknown or the requested rate is >= ODR, every sample is published.
  double decim_ratio = 1.0;
  if (_imuOdr > 0.0 && _sensPubRate > 0.0 && _sensPubRate < _imuOdr) {
    decim_ratio = _sensPubRate / _imuOdr;
  }

  bool published = false;
  for (const auto & sens_data : sens_data_batch) {
    rclcpp::Time ts_imu = (_simMode && _useSimTime) ?
      get_clock()->now() :
      sl_tools::slTime2Ros(sens_data.imu.timestamp);
    double dT = ts_imu.seconds() - _lastTs_imu.seconds();

    // Skip duplicated / out-of-order IMU samples (defensive: the FIFO is
    // already ordered and de-duplicated).
    if (_lastTs_imu != TIMEZERO_ROS && dT <= 0.0) {
      continue;
    }

    // Decimation gate
    _imuDecimAccum += decim_ratio;
    if (_imuDecimAccum < 1.0) {
      continue;
    }
    _imuDecimAccum -= 1.0;

    _lastTs_imu = ts_imu;

    updateImuFreqDiagnostics(dT);

    publishImuFrameAndTopic();

    if (_imuSubCount > 0) {
      publishImuMsg(ts_imu, sens_data);
    }

    if (_imuRawSubCount > 0) {
      publishImuRawMsg(ts_imu, sens_data);
    }

    published = true;
  }

  return published;
  // <---- SVO and simulation: drain the whole IMU FIFO
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
    DEBUG_STREAM_SENS("[publishImuMsg] _pubImu is null");
    return;
  }

  DEBUG_STREAM_SENS(
    "[publishImuMsg] IMU subscribers: " << static_cast<int>(_imuSubCount));
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
    DEBUG_STREAM_SENS("[publishImuRawMsg] _pubImuRaw is null");
    return;
  }

  DEBUG_STREAM_SENS(
    "[publishImuRawMsg] IMU RAW subscribers: "
      << static_cast<int>(_imuRawSubCount));
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

  if (!_usingIPC && _staticImuTfPublished) {
    DEBUG_ONCE_TF(
      "Static Imu TF and Transient Local message already published");
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
        sub_count = _pubCamImuTransf->get_subscription_count();
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

  if (_usingIPC) {
    _tfBroadcaster->sendTransform(*transformStamped);
    DEBUG_STREAM_TF(
      "Broadcasted new dynamic transform: "
        << transformStamped->header.frame_id << " -> " << transformStamped->child_frame_id);
  } else {
    _staticTfBroadcaster->sendTransform(*transformStamped);
    DEBUG_STREAM_TF(
      "Broadcasted new static transform: "
        << transformStamped->header.frame_id << " -> " << transformStamped->child_frame_id);
  }

  double elapsed_sec = _imuTfFreqTimer.toc();
  _pubImuTF_sec->addValue(elapsed_sec);
  _imuTfFreqTimer.tic();
  // <---- Broadcast CAM/IMU TF

  // Debug info
  if (_debugTf) {
    double roll, pitch, yaw;
    tf2::Matrix3x3(
      tf2::Quaternion(
        transformStamped->transform.rotation.x,
        transformStamped->transform.rotation.y,
        transformStamped->transform.rotation.z,
        transformStamped->transform.rotation.w))
    .getRPY(roll, pitch, yaw);
    DEBUG_STREAM_TF(
      "TF [" << transformStamped->header.frame_id << " -> "
             << transformStamped->child_frame_id << "] Position: ("
             << transformStamped->transform.translation.x << ", "
             << transformStamped->transform.translation.y << ", "
             << transformStamped->transform.translation.z
             << ") - Orientation RPY: (" << roll * RAD2DEG << ", "
             << pitch * RAD2DEG << ", " << yaw * RAD2DEG << ")");
  }

  _staticImuTfPublished = true;
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
