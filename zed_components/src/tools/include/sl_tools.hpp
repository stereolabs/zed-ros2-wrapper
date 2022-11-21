// Copyright 2022 Stereolabs
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
#include <string>
#include <vector>
#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sl/Camera.hpp>

namespace sl_tools
{
/*! \brief Check if a ZED camera is ready
 * \param serial_number : the serial number of the camera to be checked
 */
int checkCameraReady(unsigned int serial_number);

/*! \brief Get ZED camera properties
 * \param serial_number : the serial number of the camera
 */
sl::DeviceProperties getZEDFromSN(unsigned int serial_number);

std::vector<float> convertRodrigues(sl::float3 r);

/*! \brief Test if a file exist
 * \param name : the path to the file
 */
bool file_exist(const std::string & name);

/*! \brief Get Stereolabs SDK version
 * \param major : major value for version
 * \param minor : minor value for version
 * \param sub_minor _ sub_minor value for version
 */
std::string getSDKVersion(int & major, int & minor, int & sub_minor);

/*! \brief Convert StereoLabs timestamp to ROS timestamp
 *  \param t : Stereolabs timestamp to be converted
 *  \param t : ROS2 clock type
 */
rclcpp::Time slTime2Ros(sl::Timestamp t, rcl_clock_type_t clock_type = RCL_ROS_TIME);

/*! \brief check if ZED2 or ZED2i
 * \param camModel the model to check
 */
bool isZED2OrZED2i(sl::MODEL camModel);

/*! \brief check if Object Detection is available
 * \param camModel the camera model to check
 */
bool isObjDetAvailable(sl::MODEL camModel);

/*! \brief sl::Mat to ros message conversion
 * \param img : the image to publish
 * \param frameId : the id of the reference frame of the image
 * \param t : rclcpp ros::Time to stamp the image
 */
std::shared_ptr<sensor_msgs::msg::Image> imageToROSmsg(
  sl::Mat & img, std::string frameId, rclcpp::Time t);

/*! \brief sl::Mat to ros message conversion
 * \param left : the left image to convert and stitch
 * \param right : the right image to convert and stitch
 * \param frameId : the id of the reference frame of the image
 * \param t : rclcpp rclcpp::Time to stamp the image
 */
std::shared_ptr<sensor_msgs::msg::Image> imagesToROSmsg(
  sl::Mat & left, sl::Mat & right, std::string frameId, rclcpp::Time t);

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
 * \brief The CSmartMean class is used to
 * make a mobile window mean of a sequence of values
 * and reject outliers.
 * Tutorial:
 * https://www.myzhar.com/blog/tutorials/tutorial-exponential-weighted-average-good-moving-windows-average/
 */
class SmartMean
{
public:
  explicit SmartMean(int winSize);

  int getValCount()
  {
    return mValCount;  ///< Return the number of values in the sequence
  }

  double getMean()
  {
    return mMean;  ///< Return the updated mean
  }

  /*!
     * \brief addValue
     * Add a value to the sequence
     * \param val value to be added
     * \return mean value
     */
  double addValue(double val);

private:
  int mWinSize;   ///< The size of the window (number of values ti evaluate)
  int mValCount;  ///< The number of values in sequence

  double mMeanCorr;  ///< Used for bias correction
  double mMean;      ///< The mean of the last \ref mWinSize values

  double mGamma;  ///< Weight value
};

/**
 * @brief Stop Timer used to measure time intervals
 *
 */
class StopWatch
{
public:
  StopWatch();
  ~StopWatch() {}

  void tic();    //!< Set the beginning time
  double toc();  //!< Returns the seconds elapsed from the last tic

private:
  std::chrono::steady_clock::time_point mStartTime;
};

}  // namespace sl_tools

#endif  // SL_TOOLS_HPP_
