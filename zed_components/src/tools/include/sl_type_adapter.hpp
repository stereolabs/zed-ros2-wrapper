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

#ifndef SL_TYPE_ADAPTER_HPP_
#define SL_TYPE_ADAPTER_HPP_

#include <rclcpp/type_adapter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sl/Camera.hpp>

#include <rclcpp/logging.hpp>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace stereolabs
{

/// Custom type wrapping sl::Mat with ROS header metadata.
/// Used as the "custom_type" in the TypeAdapter so that intra-process
/// subscribers can receive sl::Mat data directly without serialization.
struct StampedSlMat
{
  sl::Mat mat;              // ZED SDK image (CPU memory)
  std::string frame_id;     // TF frame
  rclcpp::Time stamp;       // Message timestamp
};

}  // namespace stereolabs

template<>
struct rclcpp::TypeAdapter<stereolabs::StampedSlMat, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = stereolabs::StampedSlMat;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    const auto & img = source.mat;

    destination.header.stamp = source.stamp;
    destination.header.frame_id = source.frame_id;
    destination.height = img.getHeight();
    destination.width = img.getWidth();

    int num = 1;
    destination.is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

    destination.step = img.getStepBytes();

    size_t size = destination.step * destination.height;

    uint8_t * data_ptr = nullptr;

    switch (img.getDataType()) {
      case sl::MAT_TYPE::F32_C1:
        destination.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        data_ptr = reinterpret_cast<uint8_t *>(
          const_cast<sl::Mat &>(img).getPtr<sl::float1>());
        break;
      case sl::MAT_TYPE::F32_C2:
        destination.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
        data_ptr = reinterpret_cast<uint8_t *>(
          const_cast<sl::Mat &>(img).getPtr<sl::float2>());
        break;
      case sl::MAT_TYPE::F32_C3:
        destination.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
        data_ptr = reinterpret_cast<uint8_t *>(
          const_cast<sl::Mat &>(img).getPtr<sl::float3>());
        break;
      case sl::MAT_TYPE::F32_C4:
        destination.encoding = sensor_msgs::image_encodings::TYPE_32FC4;
        data_ptr = reinterpret_cast<uint8_t *>(
          const_cast<sl::Mat &>(img).getPtr<sl::float4>());
        break;
      case sl::MAT_TYPE::U8_C1:
        destination.encoding = sensor_msgs::image_encodings::MONO8;
        data_ptr = reinterpret_cast<uint8_t *>(
          const_cast<sl::Mat &>(img).getPtr<sl::uchar1>());
        break;
      case sl::MAT_TYPE::U8_C2:
        destination.encoding = sensor_msgs::image_encodings::TYPE_8UC2;
        data_ptr = reinterpret_cast<uint8_t *>(
          const_cast<sl::Mat &>(img).getPtr<sl::uchar2>());
        break;
      case sl::MAT_TYPE::U8_C3:
        destination.encoding = sensor_msgs::image_encodings::BGR8;
        data_ptr = reinterpret_cast<uint8_t *>(
          const_cast<sl::Mat &>(img).getPtr<sl::uchar3>());
        break;
      case sl::MAT_TYPE::U8_C4:
        destination.encoding = sensor_msgs::image_encodings::BGRA8;
        data_ptr = reinterpret_cast<uint8_t *>(
          const_cast<sl::Mat &>(img).getPtr<sl::uchar4>());
        break;
      default:
        RCLCPP_ERROR(
          rclcpp::get_logger("sl_type_adapter"),
          "convert_to_ros_message: unsupported sl::MAT_TYPE (%d)",
          static_cast<int>(img.getDataType()));
        return;
    }

    if (data_ptr != nullptr) {
      destination.data.assign(data_ptr, data_ptr + size);
    }
  }

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination.frame_id = source.header.frame_id;
    destination.stamp = source.header.stamp;

    // Determine sl::MAT_TYPE from encoding
    sl::MAT_TYPE mat_type;
    if (source.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      mat_type = sl::MAT_TYPE::F32_C1;
    } else if (source.encoding == sensor_msgs::image_encodings::TYPE_32FC2) {
      mat_type = sl::MAT_TYPE::F32_C2;
    } else if (source.encoding == sensor_msgs::image_encodings::TYPE_32FC3) {
      mat_type = sl::MAT_TYPE::F32_C3;
    } else if (source.encoding == sensor_msgs::image_encodings::TYPE_32FC4) {
      mat_type = sl::MAT_TYPE::F32_C4;
    } else if (source.encoding == sensor_msgs::image_encodings::MONO8) {
      mat_type = sl::MAT_TYPE::U8_C1;
    } else if (source.encoding == sensor_msgs::image_encodings::TYPE_8UC2) {
      mat_type = sl::MAT_TYPE::U8_C2;
    } else if (source.encoding == sensor_msgs::image_encodings::BGR8) {
      mat_type = sl::MAT_TYPE::U8_C3;
    } else if (source.encoding == sensor_msgs::image_encodings::BGRA8) {
      mat_type = sl::MAT_TYPE::U8_C4;
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("sl_type_adapter"),
        "convert_to_custom: unsupported encoding '%s'",
        source.encoding.c_str());
      return;
    }

    // Create sl::Mat wrapping the data (setFrom copies data)
    sl::Resolution res(source.width, source.height);
    destination.mat.alloc(res, mat_type);
    auto * dst_ptr = destination.mat.getPtr<sl::uchar1>();
    if (dst_ptr != nullptr && !source.data.empty()) {
      std::memcpy(dst_ptr, source.data.data(), source.data.size());
    }
  }
};

#endif  // SL_TYPE_ADAPTER_HPP_
