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

#ifndef SL_LOGGING_HPP
#define SL_LOGGING_HPP

#include <rcutils/logging_macros.h>

// ----> DEBUG MACROS
// Common
#define DEBUG_COMM(...) \
  if (_debugCommon) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_COMM(stream_arg) \
  if (_debugCommon) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)
#define DEBUG_STREAM_COMM_ONCE(stream_arg) \
  if (_debugCommon) RCLCPP_DEBUG_STREAM_ONCE(get_logger(), stream_arg)

// Simulation
#define DEBUG_SIM(...) \
  if (_debugSim) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_ONCE_SIM(...) \
  if (_debugSim) RCLCPP_DEBUG_ONCE(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_SIM(stream_arg) \
  if (_debugSim) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)
#define DEBUG_STREAM_THROTTLE_SIM(duration, stream_arg) \
  if (_debugSim) { \
    rclcpp::Clock steady_clock(RCL_STEADY_TIME); \
    RCLCPP_DEBUG_STREAM_THROTTLE( \
      get_logger(), steady_clock, duration, \
      stream_arg); \
  }

// Advanced
#define DEBUG_ADV(...) \
  if (_debugAdvanced) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_ONCE_ADV(...) \
  if (_debugAdvanced) RCLCPP_DEBUG_ONCE(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_ADV(stream_arg) \
  if (_debugAdvanced) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)
#define DEBUG_STREAM_THROTTLE_ADV(duration, stream_arg) \
  if (_debugAdvanced) { \
    rclcpp::Clock steady_clock(RCL_STEADY_TIME); \
    RCLCPP_DEBUG_STREAM_THROTTLE( \
      get_logger(), steady_clock, duration, \
      stream_arg); \
  }

// Video Depth
#define DEBUG_VD(...) \
  if (_debugVideoDepth) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_VD(stream_arg) \
  if (_debugVideoDepth) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)

// Camera Controls settings
#define DEBUG_CTRL(...) \
  if (_debugCamCtrl) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_CTRL(stream_arg) \
  if (_debugCamCtrl) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)

// Point Cloud
#define DEBUG_PC(...) \
  if (_debugPointCloud) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_PC(stream_arg) \
  if (_debugPointCloud) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)

// Positional Tracking
#define DEBUG_PT(...) \
  if (_debugPosTracking) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_ONCE_PT(...) \
  if (_debugPosTracking) RCLCPP_DEBUG_ONCE(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_PT(stream_arg) \
  if (_debugPosTracking) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)
#define DEBUG_STREAM_THROTTLE_PT(duration, stream_arg) \
  if (_debugPosTracking) { \
    rclcpp::Clock steady_clock(RCL_STEADY_TIME); \
    RCLCPP_DEBUG_STREAM_THROTTLE( \
      get_logger(), steady_clock, duration, \
      stream_arg); \
  }

// GNSS integration
#define DEBUG_GNSS(...) \
  if (_debugGnss) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_GNSS(stream_arg) \
  if (_debugGnss) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)
#define DEBUG_STREAM_THROTTLE_GNSS(duration, stream_arg) \
  if (_debugGnss) { \
    rclcpp::Clock steady_clock(RCL_STEADY_TIME); \
    RCLCPP_DEBUG_STREAM_THROTTLE( \
      get_logger(), steady_clock, duration, \
      stream_arg); \
  }

// Sensors
#define DEBUG_SENS(...) \
  if (_debugSensors) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_SENS(stream_arg) \
  if (_debugSensors) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)
#define DEBUG_STREAM_ONCE_SENS(stream_arg) \
  if (_debugSensors) RCLCPP_DEBUG_STREAM_ONCE(get_logger(), stream_arg)

// Mapping
#define DEBUG_MAP(...) \
  if (_debugMapping) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_MAP(stream_arg) \
  if (_debugMapping) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)
#define DEBUG_STREAM_ONCE_MAP(stream_arg) \
  if (_debugMapping) RCLCPP_DEBUG_STREAM_ONCE(get_logger(), stream_arg)

// Object Detection
#define DEBUG_OD(...) \
  if (_debugObjectDet) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_OD(stream_arg) \
  if (_debugObjectDet) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)

// Body Tracking
#define DEBUG_BT(...) \
  if (_debugBodyTrk) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_BT(stream_arg) \
  if (_debugBodyTrk) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)

// Region of Interest
#define DEBUG_ROI(...) \
  if (_debugRoi) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_ONCE_ROI(...) \
  if (_debugRoi) RCLCPP_DEBUG_ONCE(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_ROI(stream_arg) \
  if (_debugRoi) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)
#define DEBUG_STREAM_THROTTLE_ROI(duration, stream_arg) \
  if (_debugRoi) { \
    rclcpp::Clock steady_clock(RCL_STEADY_TIME); \
    RCLCPP_DEBUG_STREAM_THROTTLE( \
      get_logger(), steady_clock, duration, \
      stream_arg); \
  }

// Streaming
#define DEBUG_STR(...) \
  if (_debugStreaming) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_ONCE_STR(...) \
  if (_debugStreaming) RCLCPP_DEBUG_ONCE(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_STR(stream_arg) \
  if (_debugStreaming) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)
#define DEBUG_STREAM_THROTTLE_STR(duration, stream_arg) \
  if (_debugStreaming) { \
    rclcpp::Clock steady_clock(RCL_STEADY_TIME); \
    RCLCPP_DEBUG_STREAM_THROTTLE( \
      get_logger(), steady_clock, duration, \
      stream_arg); \
  }

// Nitros
#define DEBUG_NITROS(...) \
  if (_debugNitros) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define DEBUG_ONCE_NITROS(...) \
  if (_debugNitros) RCLCPP_DEBUG_ONCE(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_NITROS(stream_arg) \
  if (_debugNitros) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)
#define DEBUG_STREAM_THROTTLE_NITROS(duration, stream_arg) \
  if (_debugNitros) { \
    rclcpp::Clock steady_clock(RCL_STEADY_TIME); \
    RCLCPP_DEBUG_STREAM_THROTTLE( \
      get_logger(), steady_clock, duration, \
      stream_arg); \
  }
// <---- DEBUG MACROS

#endif  // SL_LOGGING_HPP_
