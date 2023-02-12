/********************************************************************************
 * MIT License
 *
 * Copyright (c) 2020 Stereolabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ********************************************************************************/

#ifndef SL_TOOLS_H
#define SL_TOOLS_H

///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sl/Camera.hpp>
#include <string>
#include <chrono>

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
bool file_exist(const std::string& name);

/*! \brief Get Stereolabs SDK version
 * \param major : major value for version
 * \param minor : minor value for version
 * \param sub_minor _ sub_minor value for version
 */
std::string getSDKVersion(int& major, int& minor, int& sub_minor);

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
std::shared_ptr<sensor_msgs::msg::Image> imageToROSmsg(sl::Mat& img, std::string frameId, rclcpp::Time t);

/*! \brief sl::Mat to ros message conversion
 * \param left : the left image to convert and stitch
 * \param right : the right image to convert and stitch
 * \param frameId : the id of the reference frame of the image
 * \param t : rclcpp rclcpp::Time to stamp the image
 */
std::shared_ptr<sensor_msgs::msg::Image> imagesToROSmsg(sl::Mat& left, sl::Mat& right, std::string frameId,
                                                        rclcpp::Time t);

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
  SmartMean(int winSize);

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
  ~StopWatch(){};

  void tic();    //!< Set the beginning time
  double toc();  //!< Returns the seconds elapsed from the last tic

private:
  std::chrono::steady_clock::time_point mStartTime;
};

}  // namespace sl_tools

#endif  // SL_TOOLS_H
