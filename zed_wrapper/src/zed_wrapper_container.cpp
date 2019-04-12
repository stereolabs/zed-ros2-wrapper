// /////////////////////////////////////////////////////////////////////////
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
// /////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_map.hpp>

#include "zed_component.hpp"
#include "zed_it_broadcaster.hpp"
#include "zed_tf_broadcaster.hpp"

#include <rcl_yaml_param_parser/parser.h>

#include <map>
#include <memory>
#include <vector>
#include <iostream>

// A function to parse YAML parameters is required since the "IT" and "TF" nodes must have a different
// name respect to the main "zed_node". This fact requires to set "use_global_arguments" to "false"
// so "__ns", "__name" and "__params" arguments are ignored.
std::vector<rclcpp::Parameter> createParamsListFromYAMLs(int argc, char* argv[], std::string ros_namespace,
        std::string nodename);

int main(int argc, char* argv[]) {

    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    // Local context
    auto context = rclcpp::contexts::default_context::get_global_default_context();

    // Executor args
    rclcpp::executor::ExecutorArgs execArgs;
    execArgs.context = context;

    rclcpp::executors::MultiThreadedExecutor multiExec(execArgs);

    // namespace: zed - node_name: zed_node - intra-process communication: true
    std::string lcNamespace = "zed";
    std::string lcNodeName = "zed_node";
    bool intraProcComm = false;

    // ZED main component
    // Note: use the constructor to get node_name and namespace from the launch file
    auto lc_node = std::make_shared<stereolabs::ZedCameraComponent>(lcNodeName, lcNamespace, intraProcComm);
    multiExec.add_node(lc_node->get_node_base_interface());

    // Overwrite the default values with the effective ones
    lcNamespace = lc_node->get_namespace();
    lcNodeName = lc_node->get_name();

    // This is useful to use the same parameter of the Lifecyle node in the IT and TF broadcasters even
    // if the node names are not the same (ROS2 requires that namespace+node_name in the YAML file match
    // namespace+node_name of the node where it is loaded)
    std::vector<rclcpp::Parameter> params = createParamsListFromYAMLs(argc, argv, lcNamespace, lcNodeName);

    // Note: image topics published by the main component do not support the ROS standard for `camera_info`
    //       topics to be compatible with the `camera view` plugin of `RVIZ2`.
    //       See
    //          * https://answers.ros.org/question/312930/ros2-image_transport-and-rviz2-camera-something-wrong/
    //          * https://github.com/ros2/rviz/issues/207

    // ZED Image Transport broadcaster
    // Note: this is required since `image_transport` stack in ROS Crystal Clemmys does not support
    //       Lifecycle nodes. The component subscribes to image and depth topics from the main component
    //       and re-publish them using `image_transport`
    auto it_node = std::make_shared<stereolabs::ZedItBroadcaster>(
                       lcNodeName + "_it", lcNamespace, lcNodeName,
                       context,
                       std::vector<std::string>(),
                       params,
                       false,
                       intraProcComm);
    multiExec.add_node(it_node->get_node_base_interface());

    // ZED TF broadcaster
    // Note: this is required since `tf2_ros::TransformBroadcaster` in ROS Crystal Clemmys does not support
    //       Lifecycle nodes. The component subscribes to ODOM and POSE topics from the main component
    //       and re-publish them using `TransformBroadcaster`
    auto tf_node = std::make_shared<stereolabs::ZedTfBroadcaster>(
                       lcNodeName + "_tf", lcNamespace, lcNodeName,
                       context,
                       std::vector<std::string>(),
                       params,
                       false,
                       intraProcComm);
    multiExec.add_node(tf_node->get_node_base_interface());

    multiExec.spin();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

std::vector<rclcpp::Parameter> createParamsListFromYAMLs(int argc, char* argv[], std::string ros_namespace,
        std::string nodename) {
    std::vector<rclcpp::Parameter> res;

    std::vector<std::string> yaml_paths;

    for (int i = 0; i < argc; i++) {
        std::string argvStr(argv[i]);

        std::string prefix = "__params:=";

        if (argvStr.find(prefix) == std::string::npos) {
            continue;
        }

        std::string yaml_path = argvStr.erase(0, prefix.size());
        yaml_paths.push_back(yaml_path);

        std::cout << "YAML config: " << yaml_path << std::endl;
    }

    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    std::map<std::string, rclcpp::Parameter> parameters;
    std::string combined_name_;

    if ('/' == ros_namespace.at(ros_namespace.size() - 1)) {
        combined_name_ = ros_namespace + nodename;
    } else {
        combined_name_ = ros_namespace + '/' + nodename;
    }

    if ('/' != combined_name_.at(0)) {
        combined_name_ = '/'  + combined_name_;
    }

    std::cout << "Parsing parameters for " << combined_name_ << std::endl;

    for (const std::string& yaml_path : yaml_paths) {
        rcl_params_t* yaml_params = rcl_yaml_node_struct_init(allocator);

        if (nullptr == yaml_params) {
            throw std::bad_alloc();
        }

        if (!rcl_parse_yaml_file(yaml_path.c_str(), yaml_params)) {
            std::ostringstream ss;
            ss << "Failed to parse parameters from file '" << yaml_path << "': " <<
               rcl_get_error_string().str;
            rcl_reset_error();
            throw std::runtime_error(ss.str());
        }

        rclcpp::ParameterMap initial_map = rclcpp::parameter_map_from(yaml_params);
        rcl_yaml_node_struct_fini(yaml_params);
        auto iter = initial_map.find(combined_name_);

        if (initial_map.end() == iter) {
            continue;
        }

        // Combine parameter yaml files, overwriting values in older ones
        for (auto& param : iter->second) {
            parameters[param.get_name()] = param;
        }
    }
    
    res.reserve(parameters.size());

    for (auto& kv : parameters) {
        res.emplace_back(kv.second);
    }

    return res;
}
