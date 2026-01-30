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

#include <rclcpp/rclcpp.hpp>

#define DEBUG_MONOCULAR

#if defined(DEBUG_MONOCULAR)
#include <zed_components/zed_camera_one_component.hpp>
#else
#include <zed_components/zed_camera_component.hpp>
#endif

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);

#if defined(DEBUG_MONOCULAR)
  auto zed_component =
    std::make_shared<stereolabs::ZedCameraOne>(options);
#else
  auto zed_component =
    std::make_shared<stereolabs::ZedCamera>(options);
#endif

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(zed_component);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
