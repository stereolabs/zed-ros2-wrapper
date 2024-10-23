// Copyright 2024 Stereolabs
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

#ifndef ZED_CAMERA_ONE_COMPONENT_HPP_
#define ZED_CAMERA_ONE_COMPONENT_HPP_

#include <atomic>
#include <sl/CameraOne.hpp>

#include "sl_tools.hpp"
#include "sl_types.hpp"
#include "visibility_control.hpp"

namespace stereolabs
{

class ZedCameraOne : public rclcpp::Node
{
public:
  ZED_COMPONENTS_PUBLIC
  explicit ZedCameraOne(const rclcpp::NodeOptions & options);

  virtual ~ZedCameraOne();

protected:
  // ----> Initialization functions
  void init();
  void initParameters();
  void initServices();
  void initThreads();

  void getDebugParams();

  bool startCamera();
  // <---- Initialization functions

  // ----> Callbacks
  rcl_interfaces::msg::SetParametersResult callback_paramChange(
    std::vector<rclcpp::Parameter> parameters);
  void callback_updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  // <---- Callbacks

private:
  // ----> ZED SDK
  std::shared_ptr<sl::CameraOne> _zed;
  sl::InitParametersOne _initParams;
  // <---- ZED SDK

  // ----> Threads and Timers
  std::thread _grabThread;        // Main grab thread
  std::thread _videoThread;       // RGB data publish thread
  std::thread _sensThread;        // Sensors data publish thread

  std::atomic<bool> _threadStop;
  rclcpp::TimerBase::SharedPtr _initTimer;
  rclcpp::TimerBase::SharedPtr _tempPubTimer;    // Timer to retrieve and publish camera temperature
  // <---- Threads and Timers

  // ----> Debug variables
  bool _debugCommon = false;
  bool _debugVideoDepth = false;
  bool _debugSensors = false;
  // <---- Debug variables

  // ----> QoS
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
  rclcpp::QoS _qos;
  rclcpp::PublisherOptions _pubOpt;
  rclcpp::SubscriptionOptions _subOpt;
  // <---- QoS

  // ----> Parameters
  std::string _cameraName = "zed_one";  // Name of the camera
  int _camGrabFrameRate = 0; // Grab frame rate
  sl::RESOLUTION _camResol = sl::RESOLUTION::HD1080; // Default resolution: RESOLUTION_HD1080
  PubRes _pubResolution = PubRes::NATIVE; // Use native grab resolution by default
  double _customDownscaleFactor = 1.0;  // Used to rescale data with user factor
  bool _cameraFlip = false; // Camera flipped?
  bool _enableHDR = false; // Enable HDR if supported?
  float _openTimeout_sec = 5; // Camera open timeout
  std::string _opencvCalibFile; // Custom OpenCV calibration file
  int _sdkVerbose = 0; // SDK verbose level

  int _camSerialNumber = 0; // Camera serial number

  std::string _svoFilepath = ""; // SVO input
  bool _svoRealtime = true; // SVO playback with real time

  std::string _streamAddr = ""; // Address for local streaming input
  int _streamPort = 10000;
  // <---- Parameters

  // ----> Dynamic params
  OnSetParametersCallbackHandle::SharedPtr _paramChangeCallbackHandle;
  // <---- Dynamic params

  // ----> Diagnostic
  diagnostic_updater::Updater _diagUpdater;
  // <---- Diagnostic

  // ----> Running status
  bool _svoMode = false;        // Input from SVO?
  bool _streamMode = false;     // Expecting local streaming data?
  sl::ERROR_CODE _connStatus = sl::ERROR_CODE::LAST; // Connection status
  sl::ERROR_CODE _grabStatus = sl::ERROR_CODE::LAST; // Grab status
  // <---- Running status

};

}  // namespace stereolabs

#endif  // ZED_CAMERA_ONE_COMPONENT_HPP_
