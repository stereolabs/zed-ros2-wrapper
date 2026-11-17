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

#ifndef SL_TOOLS_HPP_
#define SL_TOOLS_HPP_

#include <algorithm>
#include <cctype>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sl/Camera.hpp>
#include <string>
#include <type_traits>
#include <vector>
#include <sstream>
#include <stdexcept>

#include "gnss_replay.hpp"
#include "sl_win_avg.hpp"

#include <rcutils/logging_macros.h>

// Used to enable Positional Tracking lock check
//#define ENABLE_PT_LOCK_CHECK

// CUDA includes and macros
#ifdef FOUND_ISAAC_ROS_NITROS
  #include "isaac_ros_nitros_image_type/nitros_image_builder.hpp"

  #define CUDA_CHECK(status) \
  if (status != cudaSuccess) \
  { \
    RCLCPP_ERROR_STREAM( \
      get_logger(), "Internal CUDA ERROR encountered: {" << std::string( \
        cudaGetErrorName( \
          status)) << "} {" << std::string(cudaGetErrorString(status)) << "}"); \
    std::abort(); \
  }
#endif

namespace sl_tools
{

/*! \brief Error that stops the node, but not the process it runs in
 *
 * Thrown during the node initialization (invalid configuration, camera that
 * cannot be configured): the node catches it and stops on its own, so the other
 * components of the same container keep running. Never call `exit()` from a
 * component: it tears the whole container down, with the cameras of the other
 * nodes still grabbing (SDK-515).
 */
class NodeFatalError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/*! \brief Get a parameter from the node
 * \param node : the node to get the parameter from
 * \param paramName : the name of the parameter
 * \param defValue : the default value of the parameter
 * \param outVal : the output value of the parameter
 * \param log_info : the info to log
 * \param dynamic : if the parameter is dynamic
 * \param min : the minimum value of the parameter
 * \param max : the maximum value of the parameter
 * \tparam T : the type of the parameter
 */
template<typename T>
void getParam(
  const std::shared_ptr<rclcpp::Node> node,
  std::string paramName, T defValue, T & outVal,
  std::string log_info = std::string(), bool dynamic = false,
  T minVal = std::numeric_limits<T>::lowest(), T maxVal = std::numeric_limits<T>::max());

/*! \brief Convert a sl::float3 to a vector of float
 * \param r : the sl::float3 to convert
 * \return a vector of float
 */
std::vector<float> convertRodrigues(sl::float3 r);

/*!
 * @brief Get the full file path from a relative file name
 * @param file_name the relative file name
 * @return the full file path
 */
std::string getFullFilePath(const std::string & file_name);

/*! \brief Get Stereolabs SDK version
 * \param major : major value for version
 * \param minor : minor value for version
 * \param sub_minor _ sub_minor value for version
 */
std::string getSDKVersion(int & major, int & minor, int & sub_minor);

/*! \brief Convert a Stereolabs timestamp to ROS timestamp.
 *
 *  Single conversion entry point: works for any source (live grab, SVO
 *  replay, sim-injected, /clock topic, etc.). The live-time offset configured
 *  via \ref setSdkLiveTimeOffsetNs is applied automatically, except when
 *  replay mode is active (set via \ref setSdkReplayMode), in which case the
 *  timestamp passes through unchanged so file-recorded / sim-injected stamps
 *  retain their original ROS-epoch value.
 *
 *  \param t : Stereolabs timestamp to be converted
 *  \param clock_type : ROS 2 clock type
 */
rclcpp::Time slTime2Ros(sl::Timestamp t, rcl_clock_type_t clock_type = RCL_ROS_TIME);

/*! \brief Convert a ROS timestamp to a Stereolabs timestamp: the inverse of
 *         \ref slTime2Ros, live-time offset included.
 *  \param t : ROS timestamp to be converted
 */
sl::Timestamp rosTime2Sl(const rclcpp::Time & t);

/*! \brief Set the offset (nanoseconds) added to live SDK timestamps in
 *         \ref slTime2Ros.
 *
 *  Process-wide. Set once after a successful \p Camera::open() when the SDK is
 *  configured with a monotonic \p TIMESTAMP_CLOCK (SDK >= 5.3): pass
 *  \p ros_now_ns - sl::getCurrentTimeStamp_ns so live monotonic stamps land
 *  in the ROS epoch. Pass 0 to disable the shim. Default: 0.
 *  Has no effect when replay mode is active (see \ref setSdkReplayMode).
 */
void setSdkLiveTimeOffsetNs(int64_t offset_ns);

/*! \brief Get the current live-time offset (nanoseconds). */
int64_t getSdkLiveTimeOffsetNs();

/*! \brief Mark the process as running in a replay/sim input mode.
 *
 *  Process-wide. Set once at component startup based on the camera input
 *  mode: \p true for SVO playback or simulation, \p false for a live camera.
 *  When \p true, \ref slTime2Ros bypasses the live-time offset so file-
 *  recorded / sim-injected timestamps pass through unchanged. Default: false.
 *  Multi-camera compositions with conflicting modes are not supported (the
 *  underlying \p sl::Camera::setTimestampClock is also process-wide).
 */
void setSdkReplayMode(bool isReplay);

/*! \brief Get the current replay-mode flag. */
bool getSdkReplayMode();

/*! \brief check if ZED
 * \param camModel the model to check
 */
bool isZED(sl::MODEL camModel);

/*! \brief check if ZED Mini
 * \param camModel the model to check
 */
bool isZEDM(sl::MODEL camModel);

/*! \brief check if ZED2 or ZED2i
 * \param camModel the model to check
 */
bool isZED2OrZED2i(sl::MODEL camModel);

/*! \brief check if ZED-X or ZED-X Mini
 * \param camModel the model to check
 */
bool isZEDX(sl::MODEL camModel);

/*! \brief check if ZED X HDR, ZED X HDR Mini or ZED X HDR Max
 * \param camModel the model to check
 */
bool isZEDXHDR(sl::MODEL camModel);

/*! \brief the `camera_model` launch argument value of a camera model
 * \param camModel the model of the camera
 * \return the value, or an empty string if the model has none
 */
std::string cameraModelLaunchValue(sl::MODEL camModel);

/*! \brief look for a camera in a device list by serial number
 * \param devices the list returned by `getDeviceList()`
 * \param serial the serial number to look for
 * \param model set to the model of the camera when it is found
 * \return true if the camera is in the list
 */
bool findDeviceBySerial(
  const std::vector<sl::DeviceProperties> & devices, int serial,
  sl::MODEL & model);

/*! \brief check if a `setCameraSettings` error cannot be fixed by retrying:
 * the setting is not supported by the camera or the input, or the camera
 * refused the value. Other errors (camera gone or restarting) can be retried.
 * \param err the error returned by `setCameraSettings`
 */
bool isVideoSettingRejected(sl::ERROR_CODE err);

/*! \brief check if a depth mode is one of the NEURAL depth modes
 * \param mode the depth mode to check
 */
bool isNeuralDepthMode(sl::DEPTH_MODE mode);

/*! \brief check if Object Detection is available
 * \param camModel the camera model to check
 */
bool isObjDetAvailable(sl::MODEL camModel);

/*! \brief sl::Mat to ros message conversion
 * \param img : the image to publish
 * \param frameId : the id of the reference frame of the image
 * \param t : rclcpp ros::Time to stamp the image
 * \param use_pub_timestamp : if true use the current time as timestamp instead of \ref t
 */
std::unique_ptr<sensor_msgs::msg::Image> imageToROSmsg(
  const sl::Mat & img, const std::string & frameId, const rclcpp::Time & t, bool use_pub_timestamp);

/*! \brief sl::Mat to ros message conversion
 * \param left : the left image to convert and stitch
 * \param right : the right image to convert and stitch
 * \param frameId : the id of the reference frame of the image
 * \param t : rclcpp rclcpp::Time to stamp the image
 * \param use_pub_timestamp : if true use the current time as timestamp instead of \ref t
 */
std::unique_ptr<sensor_msgs::msg::Image> imagesToROSmsg(
  const sl::Mat & left, const sl::Mat & right, const std::string & frameId,
  const rclcpp::Time & t, bool use_pub_timestamp);

/*! \brief qos value to string
 * \param qos the value to convert
 */
std::string qos2str(rmw_qos_history_policy_t qos);

/*! \brief qos value to string
 * \param qos the value to convert
 */
std::string qos2str(rmw_qos_reliability_policy_t qos);

/*! \brief qos value to string
 * \param qos the value to convert
 */
std::string qos2str(rmw_qos_durability_policy_t qos);

/*! \brief Creates an sl::Mat containing a ROI from a polygon
 *  \param poly the ROI polygon. Coordinates must be normalized from 0.0 to 1.0
 *  \param out_roi the `sl::Mat` containing the ROI
 */
bool generateROI(const std::vector<sl::float2> & poly, sl::Mat & out_roi);

/*! \brief Parse a vector of vector of floats from a string.
 *  \param input
 *  \param error_return
 *  Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...] */
std::vector<std::vector<float>> parseStringMultiVector_float(
  const std::string & input, std::string & error_return);

/*! \brief Parse a vector of integer values from a string.
 *  \param input
 *  \param error_return
 *  Syntax is [1, 2, 3, ...] */
std::vector<int> parseStringVector_int(
  const std::string & input,
  std::string & error_return);

/*!
 * @brief Convert thread policy to string
 * @param thread_sched_policy
 * @return policy string
 */
std::string threadSched2Str(int thread_sched_policy);

/*!
 * @brief check if root is available
 * @return true if root
 */
bool checkRoot();

/*!
 * @brief Convert seconds to string in format DD:HH:MM:SS
 * @param sec seconds
 * @return string
 */
std::string seconds2str(double sec);

/*!
 * @brief read custom OD labels from a COCO-Like YAML file
 * @param label_file label file full path
 * @param out_labels the map containing the labels. The map idx corresponds to the class ID
 * @return true if successfull
 */
bool ReadCocoYaml(
  const std::string & label_file, std::unordered_map<std::string,
  std::string> & out_labels);

/**
 * @brief Stop Timer used to measure time intervals
 *
 */
class StopWatch
{
public:
  explicit StopWatch(rclcpp::Clock::SharedPtr clock);
  ~StopWatch() {}

  void tic();    //!< Set the reference time point to the current time
  double toc(std::string func_name = std::string() );  //!< Returns the seconds elapsed from the last tic in ROS clock reference (it works also in simulation)

private:
  rclcpp::Time mStartTime;  // Reference time point
  rclcpp::Clock::SharedPtr mClockPtr;  // Node clock interface
};

/**
 * @brief Tracks the disconnections of a camera, written by the grab thread and
 * read by the diagnostic callback. Uses the steady clock: a disconnection is a
 * live camera event, measured in real time also with `use_sim_time`.
 */
class CameraDisconnection
{
public:
  bool start();  //!< Marks the camera as disconnected. Returns true on the first call of a disconnection
  bool stop();  //!< Marks the camera as connected. Returns true if it was disconnected
  bool active() const {return mActive;}  //!< True while the camera is disconnected
  double elapsedSec() const;  //!< Seconds since the current disconnection started (0 if connected)
  int count() const {return mCount;}  //!< Disconnections in this session
  std::string lastReconnection() const;  //!< Local time of the last reconnection, empty if none

private:
  std::atomic<bool> mActive{false};
  std::atomic<int64_t> mStartNs{0};  // Steady clock, start of the current disconnection
  std::atomic<int> mCount{0};
  std::atomic<int64_t> mLastReconnSec{0};  // System clock, last reconnection (0: none)
};

/**
 * @brief Tracks the camera reboots requested with the `~/reboot_camera`
 * service, written by the service callback and read by the diagnostic
 * callback. Uses the steady clock for the durations.
 */
class CameraReboot
{
public:
  void start();  //!< Marks a reboot as in progress
  void stop(const std::string & result);  //!< Ends the reboot in progress with its result
  bool active() const {return mActive;}  //!< True while a reboot is in progress
  double elapsedSec() const;  //!< Seconds since the reboot in progress started (0 if none)
  int count() const {return mCount;}  //!< Reboots in this session
  std::string last() const;  //!< Local time and result of the last reboot, empty if none

private:
  std::atomic<bool> mActive{false};
  std::atomic<int64_t> mStartNs{0};  // Steady clock, start of the reboot in progress
  std::atomic<int> mCount{0};
  mutable std::mutex mLastMutex;
  std::string mLast;  // Local time and result of the last reboot
};

// ----> Utility functions

/*! \brief Convert a string to uppercase (ASCII)
 * \param s : the string to convert
 * \return the uppercase string
 */
inline std::string toUpper(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return std::toupper(c);});
  return s;
}

// ----> Template functions definitions

// Match a YAML string (with underscores) to an sl:: SDK enum value.
// Case-insensitive. Iterates from `first` up to (excluding) `last`,
// comparing sl::toString() output (spaces replaced by underscores).
// Returns true on match.
template<typename EnumT>
bool matchSdkEnum(
  const std::string & str, EnumT first, EnumT last, EnumT & outVal)
{
  const std::string upperStr = toUpper(str);

  for (int idx = static_cast<int>(first);
    idx < static_cast<int>(last); ++idx)
  {
    EnumT candidate = static_cast<EnumT>(idx);
    std::string candidate_str = sl::toString(candidate).c_str();
    std::replace(candidate_str.begin(), candidate_str.end(), ' ', '_');
    if (upperStr == toUpper(candidate_str)) {
      outVal = candidate;
      return true;
    }
  }
  return false;
}

/*! \brief Read a ROS parameter and match it to an SDK enum value.
 *  Combines getParam + matchSdkEnum + warning on mismatch + info log.
 *  \param node : the node to get the parameter from
 *  \param paramName : the ROS parameter name
 *  \param defaultStr : the default string value for the parameter
 *  \param first : first enum value to iterate from
 *  \param last : sentinel (exclusive) enum value
 *  \param outVal : receives the matched enum value (unchanged on mismatch)
 *  \param logLabel : label prefix for the INFO log (empty to skip logging)
 *  \return true if the parameter matched a valid enum value
 */
template<typename EnumT>
bool getEnumParam(
  const std::shared_ptr<rclcpp::Node> node,
  const std::string & paramName,
  const std::string & defaultStr,
  EnumT first, EnumT last,
  EnumT & outVal,
  const std::string & logLabel = std::string())
{
  std::string str = defaultStr;
  getParam(node, paramName, str, str);
  bool ok = matchSdkEnum(str, first, last, outVal);
  if (!ok) {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "The value of the parameter '" << paramName << "' is not valid: '"
                                     << str << "'. Using the default value.");
  }
  if (!logLabel.empty()) {
    RCLCPP_INFO_STREAM(
      node->get_logger(),
      logLabel << sl::toString(outVal).c_str());
  }
  return ok;
}

template<typename T>
void getParam(
  const std::shared_ptr<rclcpp::Node> node,
  std::string paramName, T defValue, T & outVal,
  std::string log_info, bool dynamic,
  T minVal, T maxVal)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = !dynamic;

  std::stringstream ss;
  if constexpr (std::is_same<T, bool>::value) {
    ss << "Default value: " << (defValue ? "TRUE" : "FALSE");
  } else {
    ss << "Default value: " << defValue;
  }
  descriptor.description = ss.str();

  if constexpr (std::is_same<T, double>::value) {
    descriptor.additional_constraints = "Range: [" + std::to_string(minVal) + ", " + std::to_string(
      maxVal) + "]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = minVal;
    range.to_value = maxVal;
    descriptor.floating_point_range.push_back(range);
  } else if constexpr (std::is_same<T, int>::value) {
    descriptor.additional_constraints = "Range: [" + std::to_string(minVal) + ", " + std::to_string(
      maxVal) + "]";
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = minVal;
    range.to_value = maxVal;
    descriptor.integer_range.push_back(range);
  }

  // A value set by the user (YAML file, launch override) that is out of range or
  // of the wrong type makes declare_parameter() throw, which would abort the
  // whole node and its container: convert it to a valid value instead and warn.
  // A number of the other numeric type is converted (`5` for a double, `5.5`
  // rounded to the nearest integer), then clamped to the range.
  auto declareValidValue = [&](bool wrongType) {
      T value = defValue;
      bool converted = false;
      std::string userValue;
      double v = 0.0;  // The user value, as a number
      const auto & overrides =
        node->get_node_parameters_interface()->get_parameter_overrides();
      auto it = overrides.find(paramName);
      if (it != overrides.end()) {
        if (it->second.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          std::stringstream ss;
          ss << it->second.get<double>();  // `6.6`, not `6.600000`
          userValue = ss.str();
        } else {
          userValue = rclcpp::to_string(it->second);
        }
      }
      if constexpr (std::is_same<T, double>::value || std::is_same<T, int>::value) {
        if (it != overrides.end()) {
          if (it->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            v = static_cast<double>(it->second.get<int64_t>());
            converted = true;
          } else if (it->second.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            v = it->second.get<double>();
            converted = std::isfinite(v);
            if (std::is_same<T, int>::value) {
              v = std::round(v);
            }
          }
          if (converted) {
            value = static_cast<T>(
              std::clamp(v, static_cast<double>(minVal), static_cast<double>(maxVal)));
          }
        }
      }

      std::stringstream msg;
      msg << std::boolalpha << "The value " << (userValue.empty() ? "" : "`" + userValue + "` ") <<
        "of the parameter '" << paramName << "' ";
      if (!converted) {
        msg << (wrongType ? "has the wrong type" : "is not valid") <<
          ": using the default value " << defValue;
      } else if constexpr (std::is_same<T, double>::value || std::is_same<T, int>::value) {
        if (wrongType) {
          msg << "has the wrong type (" <<
            (std::is_same<T, double>::value ? "a decimal number" : "an integer") << " is expected)";
        }
        // `v` is already rounded for an integer: only the range can change it
        if (static_cast<double>(value) != v) {
          msg << (wrongType ? " and " : "") << "is out of the range [" << minVal << ", " <<
            maxVal << "]";
        }
        msg << ": using " << value;
      }
      RCLCPP_WARN_STREAM(node->get_logger(), msg.str());
      node->declare_parameter(paramName, rclcpp::ParameterValue(value), descriptor, true);
    };

  try {
    node->declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);
  } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
    declareValidValue(false);
  }
#ifndef FOUND_FOXY
  catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
    declareValidValue(true);
  }
#endif

  if (!node->get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << defValue);
  }

  if (!log_info.empty()) {
    std::stringstream ss;
    ss << log_info;
    if constexpr (std::is_same<T, bool>::value) {
      ss << (outVal ? "TRUE" : "FALSE");
    } else {
      ss << outVal;
    }
    if (dynamic) {
      ss << " [DYNAMIC]";
    }
    RCLCPP_INFO_STREAM(node->get_logger(), ss.str());
  }
}
// Validate and assign a dynamic parameter within [minVal, maxVal].
// Returns true on success. On failure, sets result.successful = false with reason.
template<typename T>
bool checkParamRange(
  const rclcpp::Parameter & param,
  T & outVal,
  T minVal, T maxVal,
  rcl_interfaces::msg::SetParametersResult & result,
  const rclcpp::Logger & logger)
{
  T val;
  if constexpr (std::is_same_v<T, double>) {
    val = param.as_double();
  } else if constexpr (std::is_same_v<T, float>) {
    val = static_cast<float>(param.as_double());
  } else if constexpr (std::is_same_v<T, int>) {
    val = param.as_int();
  } else {
    static_assert(
      std::is_same_v<T, double>|| std::is_same_v<T, float>|| std::is_same_v<T, int>,
      "checkParamRange only supports double, float and int");
  }

  if (val < minVal || val > maxVal) {
    result.successful = false;
    result.reason = param.get_name() + " must be in range [" +
      std::to_string(minVal) + ", " + std::to_string(maxVal) + "]";
    RCLCPP_WARN_STREAM(logger, result.reason);
    return false;
  }
  outVal = val;
  return true;
}
// <---- Template functions definitions

/*! \brief Fill the distortion model and the distortion coefficients of a
 *  `CameraInfo` message from the calibration parameters reported by the ZED SDK.
 *
 *  The model is taken from `sl::CameraParameters::lens_distortion_model` when the
 *  ZED SDK provides it (>= 5.4), instead of being guessed from the camera model:
 *  the lens, not the camera, decides it, and a fisheye ZED X One or ZED Mini is
 *  not distinguishable from its siblings by model alone.
 *
 *  \param zed_params : the raw or rectified calibration parameters of one camera
 *  \param cam_info : the message to fill
 */
void fillCamInfoDistortion(
  const sl::CameraParameters & zed_params,
  sensor_msgs::msg::CameraInfo & cam_info);

}  // namespace sl_tools

#endif  // SL_TOOLS_HPP_
