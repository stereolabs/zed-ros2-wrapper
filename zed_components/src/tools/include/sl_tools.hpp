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

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sl/Camera.hpp>
#include <string>
#include <vector>
#include <sstream>

#include "gnss_replay.hpp"
#include "sl_win_avg.hpp"

#include <rcutils/logging_macros.h>

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
  T minVal = std::numeric_limits<T>::min(), T maxVal = std::numeric_limits<T>::max());

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

/*! \brief Convert StereoLabs timestamp to ROS timestamp
 *  \param t : Stereolabs timestamp to be converted
 *  \param t : ROS 2 clock type
 */
rclcpp::Time slTime2Ros(sl::Timestamp t, rcl_clock_type_t clock_type = RCL_ROS_TIME);

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
std::vector<std::vector<float>> parseStringVector(
  const std::string & input, std::string & error_return);

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

// ----> Template functions definitions
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

  node->declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

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
// <---- Template functions definitions

}  // namespace sl_tools

#endif  // SL_TOOLS_HPP_
