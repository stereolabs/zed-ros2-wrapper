// Copyright 2026 Stereolabs
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

#ifndef SL_CUDA_BUFFER_HPP_
#define SL_CUDA_BUFFER_HPP_

// `rosidl::Buffer` CUDA zero-copy transport (ROS 2 Lyrical and newer).
//
// Generated messages store `uint8[]` fields in a `rosidl::Buffer<uint8_t>`
// whose memory can live on the GPU. The `cuda` buffer backend shares that GPU
// memory with the subscribers that run on the same host and GPU and accept the
// `cuda` backend; the RMW automatically serializes the data on CPU for all the
// other subscribers.
//
// The `cuda_buffer` headers are only included by `sl_cuda_buffer.cpp`, because
// they define a `CUDA_CHECK` macro that conflicts with the one used by the
// NITROS code.

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sl/Camera.hpp>

#ifdef FOUND_CUDA_BUFFER
#include <rosidl_buffer/buffer.hpp>
#endif

namespace sl_tools
{

// Name of the `rosidl::Buffer` backend plugin providing the CUDA zero-copy transport
constexpr auto CUDA_BUFFER_BACKEND = "cuda";

// How to deliver a message to the subscribers of a publisher
struct BufferRoute
{
  bool cuda = false;  // Publish a message with a CUDA-backed `rosidl::Buffer`
  bool cpu = false;   // Publish a message with a CPU `rosidl::Buffer`
};

// Choose how to deliver a message to the subscribers of `pub`.
// A CUDA-backed message is published only when all the subscribers are in
// other processes: the RMW negotiates the buffer backend with each of them and
// falls back to CPU serialization when the `cuda` backend cannot be used.
// rclcpp intra-process delivery bypasses this negotiation and would hand GPU
// memory to subscribers that cannot read it, so the CPU message is used when
// intra-process subscribers exist.
// The CUDA zero-copy transport does not support `TRANSIENT_LOCAL` durability.
template<typename PublisherT>
BufferRoute getBufferRoute(const PublisherT & pub)
{
  BufferRoute route;
  if (!pub || pub->get_subscription_count() == 0) {
    return route;
  }
  if (pub->get_intra_process_subscription_count() > 0 ||
    pub->get_actual_qos().durability() == rclcpp::DurabilityPolicy::TransientLocal)
  {
    route.cpu = true;
  } else {
    route.cuda = true;
  }
  return route;
}

#ifdef FOUND_CUDA_BUFFER

// Check that the RMW implementation `rmw` supports the `rosidl::Buffer`
// backends: `rmw_fastrtps_cpp`, or `rmw_zenoh_cpp` v0.10.5 or newer (Lyrical).
// `reason` receives the reason when it is not supported.
bool isCudaBufferRmwSupported(const std::string & rmw, std::string & reason);

// Check that the `cuda` buffer backend plugin is installed.
// `installed` receives the names of all the installed buffer backends.
bool isCudaBufferBackendInstalled(std::vector<std::string> & installed);

// Copy the GPU data retrieved by the ZED SDK into CUDA-backed `rosidl::Buffer`
// messages.
// All the copies are performed in the CUDA context of the ZED SDK on a
// dedicated stream. Each copy completes before the function returns, so the
// source `sl::Mat` can be overwritten by the next retrieve call right after.
// All the functions are thread-safe.
class CudaBufferTransport
{
public:
  CudaBufferTransport() = default;
  ~CudaBufferTransport();

  CudaBufferTransport(const CudaBufferTransport &) = delete;
  CudaBufferTransport & operator=(const CudaBufferTransport &) = delete;

  // Create the CUDA resources. Call it after opening the camera.
  // `sdk_stream` is the CUDA stream of the ZED SDK. `sdk_ctx` is the CUDA
  // context of the ZED SDK, or nullptr to use the context of `sdk_stream`.
  // `retrieve_on_stream`:
  //  - true: the caller retrieves the data on `stream()`, which orders the
  //    copies after the retrieve without depending on the GPU work that the
  //    SDK queues afterwards (e.g. the next `grab()`).
  //  - false: the data are retrieved on the default stream of the SDK, so each
  //    copy waits for all the GPU work queued on the SDK stream and on the
  //    legacy default stream. Use it when the retrieve API has no stream
  //    parameter, or when the SDK can reuse the source memory of a retrieve
  //    before the copies run on `stream()`. The retrieved `sl::Mat` must not
  //    be retrieved again before the copies are done.
  bool init(
    CUcontext sdk_ctx, CUstream sdk_stream, bool retrieve_on_stream,
    std::string & err);
  // Release the CUDA resources. Call it before closing the camera.
  void reset();
  bool isReady() const;

  // The CUDA stream to pass to the retrieve functions of the ZED SDK when
  // initialized with `retrieve_on_stream`
  cudaStream_t stream() const;

  // Fill `msg` with the GPU data of `img`. `msg.data` becomes a CUDA-backed
  // buffer containing the image without the row padding of the `sl::Mat`.
  // The header is not modified.
  bool toImageMsg(const sl::Mat & img, sensor_msgs::msg::Image & msg, std::string & err);

  // Fill `msg` with the side-by-side composition of the GPU data of `left`
  // and `right`. The header is not modified.
  bool toStereoImageMsg(
    const sl::Mat & left, const sl::Mat & right,
    sensor_msgs::msg::Image & msg, std::string & err);

  // Fill `msg.data` with the GPU data of the XYZBGRA point cloud `cloud`.
  // All the other fields are not modified.
  bool toPointCloudData(
    const sl::Mat & cloud, sensor_msgs::msg::PointCloud2 & msg,
    std::string & err);

  // Copy the GPU data of `gpu` to the CPU `sl::Mat` `cpu` (allocated if required)
  bool download(const sl::Mat & gpu, sl::Mat & cpu, std::string & err);

  // Copy the GPU data of `gpu` to the CPU memory pointed by `dst`, with a row
  // pitch of `dst_pitch` bytes.
  bool download(const sl::Mat & gpu, void * dst, size_t dst_pitch, std::string & err);

private:
  // A 2D copy from GPU memory to the destination buffer
  struct Copy2D
  {
    const void * src;
    size_t src_pitch;
    size_t dst_offset;
    size_t width_bytes;
    size_t height;
  };

  // Make the ZED SDK context current and, without `retrieve_on_stream`, make
  // the transport stream wait for the GPU work queued on the ZED SDK stream.
  // Call it with `mMutex` locked.
  bool prepare(std::string & err);
  // Create a CUDA-backed buffer of `size` bytes and fill it with `copies`
  bool fillBuffer(
    rosidl::Buffer<uint8_t> & data, size_t size, size_t dst_pitch,
    const std::vector<Copy2D> & copies, std::string & err);

  mutable std::mutex mMutex;
  CUcontext mSdkCtx = nullptr;
  CUstream mSdkStream = nullptr;
  cudaStream_t mStream = nullptr;
  cudaEvent_t mSdkEvent = nullptr;
};

#endif  // FOUND_CUDA_BUFFER

}  // namespace sl_tools

#endif  // SL_CUDA_BUFFER_HPP_
