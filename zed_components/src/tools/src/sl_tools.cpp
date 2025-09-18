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

#include <float.h>
#include <sys/stat.h>
#include <unistd.h> // getuid

#include <algorithm>
#include <string>
#include <sstream>
#include <vector>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "sl_tools.hpp"

namespace sl_tools
{
std::vector<float> convertRodrigues(sl::float3 r)
{
  float theta = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);

  std::vector<float> R = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

  if (theta < DBL_EPSILON) {
    return R;
  } else {
    float c = cos(theta);
    float s = sin(theta);
    float c1 = 1.f - c;
    float itheta = theta ? 1.f / theta : 0.f;

    r *= itheta;

    std::vector<float> rrt = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

    float * p = rrt.data();
    p[0] = r.x * r.x;
    p[1] = r.x * r.y;
    p[2] = r.x * r.z;
    p[3] = r.x * r.y;
    p[4] = r.y * r.y;
    p[5] = r.y * r.z;
    p[6] = r.x * r.z;
    p[7] = r.y * r.z;
    p[8] = r.z * r.z;

    std::vector<float> r_x = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    p = r_x.data();
    p[0] = 0;
    p[1] = -r.z;
    p[2] = r.y;
    p[3] = r.z;
    p[4] = 0;
    p[5] = -r.x;
    p[6] = -r.y;
    p[7] = r.x;
    p[8] = 0;

    // R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]

    sl::Matrix3f eye;
    eye.setIdentity();

    sl::Matrix3f sl_R(R.data());
    sl::Matrix3f sl_rrt(rrt.data());
    sl::Matrix3f sl_r_x(r_x.data());

    sl_R = eye * c + sl_rrt * c1 + sl_r_x * s;

    R[0] = sl_R.r00;
    R[1] = sl_R.r01;
    R[2] = sl_R.r02;
    R[3] = sl_R.r10;
    R[4] = sl_R.r11;
    R[5] = sl_R.r12;
    R[6] = sl_R.r20;
    R[7] = sl_R.r21;
    R[8] = sl_R.r22;
  }

  return R;
}

std::string getFullFilePath(const std::string & file_name)
{
  std::string new_filename;
  if (file_name.front() == '~') {
    std::string home_path = std::getenv("HOME");
    if (!home_path.empty()) {
      new_filename = home_path;
      new_filename += file_name.substr(1, file_name.size() - 1);
    }
  } else {
    new_filename = file_name;
  }

  std::filesystem::path path(new_filename);
  auto abs_path = std::filesystem::absolute(path);
  return abs_path.string();
}

std::string getSDKVersion(int & major, int & minor, int & sub_minor)
{
  std::string ver = sl::Camera::getSDKVersion().c_str();
  std::vector<std::string> strings;
  std::istringstream f(ver);
  std::string s;

  while (getline(f, s, '.')) {
    strings.push_back(s);
  }

  major = 0;
  minor = 0;
  sub_minor = 0;

  switch (strings.size()) {
    case 3:
      sub_minor = std::stoi(strings[2]);

    case 2:
      minor = std::stoi(strings[1]);

    case 1:
      major = std::stoi(strings[0]);
  }

  return ver;
}

rclcpp::Time slTime2Ros(sl::Timestamp t, rcl_clock_type_t clock_type)
{
  uint64_t ts_nsec = t.getNanoseconds();
  uint32_t sec = static_cast<uint32_t>(ts_nsec / 1000000000);
  uint32_t nsec = static_cast<uint32_t>(ts_nsec % 1000000000);
  return rclcpp::Time(sec, nsec, clock_type);
}

std::unique_ptr<sensor_msgs::msg::Image> imageToROSmsg(
  const sl::Mat & img, const std::string & frameId, const rclcpp::Time & t, bool use_pub_timestamp)
{
  std::unique_ptr<sensor_msgs::msg::Image> imgMessage = std::make_unique<sensor_msgs::msg::Image>();

  imgMessage->header.stamp = use_pub_timestamp ? rclcpp::Clock().now() : t;
  imgMessage->header.frame_id = frameId;
  imgMessage->height = img.getHeight();
  imgMessage->width = img.getWidth();

  int num = 1;  // for endianness detection
  imgMessage->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

  imgMessage->step = img.getStepBytes();

  size_t size = imgMessage->step * imgMessage->height;

  uint8_t * data_ptr = nullptr;

  sl::MAT_TYPE dataType = img.getDataType();

  switch (dataType) {
    case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
      imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float1>());
      imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
      imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float2>());
      imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
      imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float3>());
      imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
      imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float4>());
      imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
      imgMessage->encoding = sensor_msgs::image_encodings::MONO8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar1>());
      imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
      imgMessage->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar2>());
      imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
      imgMessage->encoding = sensor_msgs::image_encodings::BGR8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar3>());
      imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
      imgMessage->encoding = sensor_msgs::image_encodings::BGRA8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar4>());
      imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;
  }

  return imgMessage;
}

std::unique_ptr<sensor_msgs::msg::Image> imagesToROSmsg(
  const sl::Mat & left, const sl::Mat & right, const std::string & frameId,
  const rclcpp::Time & t, bool use_pub_timestamp)
{
  std::unique_ptr<sensor_msgs::msg::Image> imgMsgPtr = std::make_unique<sensor_msgs::msg::Image>();

  if (
    left.getWidth() != right.getWidth() || left.getHeight() != right.getHeight() ||
    left.getChannels() != right.getChannels() || left.getDataType() != right.getDataType())
  {
    return imgMsgPtr;
  }

  imgMsgPtr->header.stamp = use_pub_timestamp ? rclcpp::Clock().now() : t;
  imgMsgPtr->header.frame_id = frameId;
  imgMsgPtr->height = left.getHeight();
  imgMsgPtr->width = 2 * left.getWidth();

  int num = 1;  // for endianness detection
  imgMsgPtr->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

  imgMsgPtr->step = 2 * left.getStepBytes();

  size_t size = imgMsgPtr->step * imgMsgPtr->height;
  imgMsgPtr->data.resize(size);

  sl::MAT_TYPE dataType = left.getDataType();

  char * srcL;
  char * srcR;

  switch (dataType) {
    case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
      imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      srcL = reinterpret_cast<char *>(left.getPtr<sl::float1>());
      srcR = reinterpret_cast<char *>(right.getPtr<sl::float1>());
      break;

    case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
      imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
      srcL = reinterpret_cast<char *>(left.getPtr<sl::float2>());
      srcR = reinterpret_cast<char *>(right.getPtr<sl::float2>());
      break;

    case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
      imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
      srcL = reinterpret_cast<char *>(left.getPtr<sl::float3>());
      srcR = reinterpret_cast<char *>(right.getPtr<sl::float3>());
      break;

    case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
      imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
      srcL = reinterpret_cast<char *>(left.getPtr<sl::float4>());
      srcR = reinterpret_cast<char *>(right.getPtr<sl::float4>());
      break;

    case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
      imgMsgPtr->encoding = sensor_msgs::image_encodings::MONO8;
      srcL = reinterpret_cast<char *>(left.getPtr<sl::uchar1>());
      srcR = reinterpret_cast<char *>(right.getPtr<sl::uchar1>());
      break;

    case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
      imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
      srcL = reinterpret_cast<char *>(left.getPtr<sl::uchar2>());
      srcR = reinterpret_cast<char *>(right.getPtr<sl::uchar2>());
      break;

    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
      imgMsgPtr->encoding = sensor_msgs::image_encodings::BGR8;
      srcL = reinterpret_cast<char *>(left.getPtr<sl::uchar3>());
      srcR = reinterpret_cast<char *>(right.getPtr<sl::uchar3>());
      break;

    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
      imgMsgPtr->encoding = sensor_msgs::image_encodings::BGRA8;
      srcL = reinterpret_cast<char *>(left.getPtr<sl::uchar4>());
      srcR = reinterpret_cast<char *>(right.getPtr<sl::uchar4>());
      break;
  }

  char * dest = reinterpret_cast<char *>((&imgMsgPtr->data[0]));

  for (int i = 0; i < left.getHeight(); i++) {
    memcpy(dest, srcL, left.getStepBytes());
    dest += left.getStepBytes();
    memcpy(dest, srcR, right.getStepBytes());
    dest += right.getStepBytes();

    srcL += left.getStepBytes();
    srcR += right.getStepBytes();
  }

  return imgMsgPtr;
}

std::string qos2str(rmw_qos_history_policy_t qos)
{
  if (qos == RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    return "KEEP_LAST";
  }

  if (qos == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
    return "KEEP_ALL";
  }

  return "Unknown QoS value";
}

std::string qos2str(rmw_qos_reliability_policy_t qos)
{
  if (qos == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
    return "RELIABLE";
  }

  if (qos == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
    return "BEST_EFFORT";
  }

  return "Unknown QoS value";
}

std::string qos2str(rmw_qos_durability_policy_t qos)
{
  if (qos == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    return "TRANSIENT_LOCAL";
  }

  if (qos == RMW_QOS_POLICY_DURABILITY_VOLATILE) {
    return "VOLATILE";
  }

  return "Unknown QoS value";
}

inline bool contains(const std::vector<sl::float2> & poly, sl::float2 test)
{
  int i, j;
  bool c = false;
  const int nvert = poly.size();
  for (i = 0, j = nvert - 1; i < nvert; j = i++) {
    if (
      ((poly[i].y > test.y) != (poly[j].y > test.y)) &&
      (test.x <
      (poly[j].x - poly[i].x) * (test.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x))
    {
      c = !c;
    }
  }
  return c;
}

bool generateROI(const std::vector<sl::float2> & poly, sl::Mat & out_roi)
{
  if (poly.size() < 3) {
    out_roi = sl::Mat();
    return false;
  }

  // Set each pixel to valid
  //std::cerr << "Setting ROI mask to full valid" << std::endl;
  out_roi.setTo<sl::uchar1>(255, sl::MEM::CPU);

  // ----> De-normalize coordinates
  size_t w = out_roi.getWidth();
  size_t h = out_roi.getHeight();

  //std::cerr << "De-normalize coordinates" << std::endl;
  //std::cerr << "Image resolution: " << w << "x" << h << std::endl;
  //std::cerr << "Polygon size: " << poly.size() << std::endl;
  std::vector<sl::float2> poly_img;
  size_t idx = 0;
  for (auto & it : poly) {
    sl::float2 pt;
    pt.x = it.x * w;
    pt.y = it.y * h;

    if (pt.x >= w) {
      pt.x = (w - 1);
    }
    if (pt.y >= h) {
      pt.y = (h - 1);
    }

    poly_img.push_back(pt);

    //std::cerr << "Pushed pt #: " << idx << std::endl;
    ++idx;
  }
  // <---- De-normalize coordinates

  // ----> Unset ROI pixels outside the polygon
  //std::cerr << "Unset ROI pixels outside the polygon" << std::endl;
  //std::cerr << "Set mask" << std::endl;
  for (int v = 0; v < h; v++) {
    for (int u = 0; u < w; u++) {
      if (!contains(poly_img, sl::float2(u, v))) {
        out_roi.setValue<sl::uchar1>(u, v, 0, sl::MEM::CPU);
      }
    }
  }
  //std::cerr << "Mask ready" << std::endl;
  //std::cerr << "ROI resolution: " << w << "x" << h << std::endl;
  // <---- Unset ROI pixels outside the polygon

  return true;
}

std::vector<std::vector<float>> parseStringVector(
  const std::string & input, std::string & error_return)
{
  std::vector<std::vector<float>> result;

  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
  while (!!input_ss && !input_ss.eof()) {
    switch (input_ss.peek()) {
      case EOF:
        break;
      case '[':
        depth++;
        if (depth > 2) {
          error_return = "Array depth greater than 2";
          return result;
        }
        input_ss.get();
        current_vector.clear();
        break;
      case ']':
        depth--;
        if (depth < 0) {
          error_return = "More close ] than open [";
          return result;
        }
        input_ss.get();
        if (depth == 1) {
          result.push_back(current_vector);
        }
        break;
      case ',':
      case ' ':
      case '\t':
        input_ss.get();
        break;
      default:  // All other characters should be part of the numbers.
        if (depth != 2) {
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
          error_return = err_ss.str();
          return result;
        }
        float value;
        input_ss >> value;
        if (!!input_ss) {
          current_vector.push_back(value);
        }
        break;
    }
  }

  if (depth != 0) {
    error_return = "Unterminated vector string.";
  } else {
    error_return = "";
  }

  return result;
}

std::string threadSched2Str(int thread_sched_policy)
{
  switch (thread_sched_policy) {
    case SCHED_OTHER:
      return "SCHED_OTHER";
    case SCHED_FIFO:
      return "SCHED_FIFO";
    case SCHED_RR:
      return "SCHED_RR";
#ifdef __USE_GNU
    case SCHED_BATCH:
      return "SCHED_BATCH";
    case SCHED_ISO:
      return "SCHED_ISO";
    case SCHED_IDLE:
      return "SCHED_IDLE";
    case SCHED_DEADLINE:
      return "SCHED_DEADLINE";
#endif
    default:
      return "";
  }
}

bool checkRoot()
{
  if (getuid()) {
    return false;
  } else {
    return true;
  }
}

bool ReadCocoYaml(
  const std::string & label_file, std::unordered_map<std::string,
  std::string> & out_labels)
{
  // Open the YAML file
  std::ifstream file(label_file.c_str());
  if (!file.is_open()) {
    return false;
  }

  // Read the file line by line
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(file, line)) {
    lines.push_back(line);
  }

  // Find the start and end of the names section
  std::size_t start = 0;
  std::size_t end = 0;
  for (std::size_t i = 0; i < lines.size(); i++) {
    if (lines[i].find("names:") != std::string::npos) {
      start = i + 1;
    } else if (start > 0 && lines[i].find(':') == std::string::npos) {
      end = i;
      break;
    }
  }

  // Extract the labels
  for (std::size_t i = start; i < end; i++) {
    std::stringstream ss(lines[i]);
    std::string class_id, label;
    std::getline(ss, class_id, ':'); // Extract the number before the delimiter
    // ---> remove heading spaces and tabs
    class_id.erase(remove(class_id.begin(), class_id.end(), ' '), class_id.end());
    class_id.erase(remove(class_id.begin(), class_id.end(), '\t'), class_id.end());
    // <--- remove heading spaces and tabs
    std::getline(ss, label); // Extract the string after the delimiter
    out_labels[class_id] = label;
  }

  return true;
}


bool isZED(sl::MODEL camModel)
{
  if (camModel == sl::MODEL::ZED) {
    return true;
  }
  return false;
}

bool isZEDM(sl::MODEL camModel)
{
  if (camModel == sl::MODEL::ZED_M) {
    return true;
  }
  return false;
}

bool isZED2OrZED2i(sl::MODEL camModel)
{
  if (camModel == sl::MODEL::ZED2) {
    return true;
  }
  if (camModel == sl::MODEL::ZED2i) {
    return true;
  }
  return false;
}

bool isZEDX(sl::MODEL camModel)
{
  if (camModel == sl::MODEL::ZED_X) {
    return true;
  }
  if (camModel == sl::MODEL::ZED_XM) {
    return true;
  }
  if (camModel == sl::MODEL::VIRTUAL_ZED_X) {
    return true;
  }
  return false;
}

bool isObjDetAvailable(sl::MODEL camModel)
{
  if (camModel != sl::MODEL::ZED) {
    return true;
  }
  return false;
}

std::string seconds2str(double sec)
{
  int days = sec / 86400;
  sec -= days * 86400;
  int hours = sec / 3600;
  sec -= hours * 3600;
  int minutes = sec / 60;
  sec -= minutes * 60;

  std::stringstream ss;
  ss << days << " days, " << hours << " hours, " << minutes << " min, " << sec << " sec";

  return ss.str();
}

StopWatch::StopWatch(rclcpp::Clock::SharedPtr clock)
: mStartTime(0, 0, RCL_ROS_TIME),
  mClockPtr(clock)
{
  tic();  // Start the timer at creation
}

void StopWatch::tic()
{
  mStartTime = mClockPtr->now();  // Reset the start time point
}

double StopWatch::toc(std::string func_name)
{
  auto now = mClockPtr->now();

  double elapsed_nsec = (now - mStartTime).nanoseconds();
  if (!func_name.empty()) {
    std::cerr << func_name << " -> toc elapsed_sec: " << elapsed_nsec / 1e9 << std::endl <<
      std::flush;
  }

  return elapsed_nsec / 1e9;  // Returns elapsed time in seconds
}

}  // namespace sl_tools
