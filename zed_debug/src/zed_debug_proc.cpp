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
#include <zed_components/zed_camera_component.hpp>
#include <zed_components/zed_camera_one_component.hpp>

int main(int argc, char ** argv)
{
  // Disable stdout buffering for better logging visibility
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Disable intra-process communication
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);

  // Check for monocular mode argument
  bool monocular_mode = false;
  for (int i = 1; i < argc; ++i) {
    std::string mode_arg = argv[i];
    if (mode_arg == "--monocular") {
      monocular_mode = true;
      break;
    }
  }

  // Create the appropriate ZED camera node (monocular or stereo)
  rclcpp::Node::SharedPtr zed_component;
  if (monocular_mode) {
    RCLCPP_INFO(
      rclcpp::get_logger("zed_debug_proc"),
      "Debugging ZED Camera One (monocular) node...");
    zed_component = std::make_shared<stereolabs::ZedCameraOne>(options);
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("zed_debug_proc"),
      "Debugging ZED Camera (stereo) node...");
    zed_component = std::make_shared<stereolabs::ZedCamera>(options);
  }

  // Create single-threaded executor and spin the node
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(zed_component);
  executor.spin();

  // Shutdown ROS 2
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
