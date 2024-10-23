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
  // <---- Debug variables

  // ----> QoS
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings  
  rclcpp::QoS _qos;
  rclcpp::PublisherOptions _pubOpt;
  rclcpp::SubscriptionOptions _subOpt;
  // <---- QoS

  // ----> Dynamic params
  OnSetParametersCallbackHandle::SharedPtr _aramChangeCallbackHandle;
  // <---- Dynamic params

};

}  // namespace stereolabs

#endif  // ZED_CAMERA_ONE_COMPONENT_HPP_
