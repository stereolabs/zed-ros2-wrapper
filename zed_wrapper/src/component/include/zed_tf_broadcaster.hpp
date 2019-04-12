#ifndef ZED_TF_BROADCASTER_HPP
#define ZED_TF_BROADCASTER_HPP

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

#include "visibility_control.h"

#include <rclcpp/node.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace stereolabs {

    typedef rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

    class ZedTfBroadcaster : public rclcpp::Node {
      public:
        /// Create a new node with the specified name.
        /**
         * \param[in] node_name Name of the node.
         * \param[in] namespace_ Namespace of the node.
         * \param[in] use_intra_process_comms True to use the optimized intra-process communication
         * pipeline to pass messages between nodes in the same process using shared memory.
         */
        ZED_PUBLIC
        explicit ZedTfBroadcaster(const std::string& node_name = "zed_tf_broadcaster",
                                  const std::string& ros_namespace = "zed",
                                  const std::string& main_node = "zed_node",
                                  bool intra_process_comms = true);

        /// Create a node based on the node name and a rclcpp::Context.
        /**
         * \param[in] node_name Name of the node.
         * \param[in] ros_namespace Namespace of the node.
         * \param[in] context The context for the node (usually represents the state of a process).
         * \param[in] arguments Command line arguments that should apply only to this node.
         * \param[in] initial_parameters a list of initial values for parameters on the node.
         * This can be used to provide remapping rules that only affect one instance.
         * \param[in] use_global_arguments False to prevent node using arguments passed to the process.
         * \param[in] use_intra_process_comms True to use the optimized intra-process communication
         * pipeline to pass messages between nodes in the same process using shared memory.
         * \param[in] start_parameter_services True to setup ROS interfaces for accessing parameters
         * in the node.
         */
        ZED_PUBLIC
        explicit ZedTfBroadcaster(const std::string& node_name,
                                  const std::string& ros_namespace,
                                  const std::string& main_node,
                                  rclcpp::Context::SharedPtr context,
                                  const std::vector<std::string>& arguments,
                                  const std::vector<rclcpp::Parameter>& initial_parameters,
                                  bool use_global_arguments = true,
                                  bool use_intra_process_comms = false,
                                  bool start_parameter_services = true);

      protected:
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void mapOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void initSubscribers();
        void initParameters();
        void initBroadcasters();

      private:
        // Params
        bool mPublishTf = false;
        bool mPublishMapTf = false;
        std::string mOdomTopic = "odom";
        std::string mMapOdomTopic = "map2odom";

        // Subscribers
        odomSub mOdomSub;
        odomSub mMapOdomSub;

        // QoS profiles
        // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
        rmw_qos_profile_t mPoseQos = rmw_qos_profile_default;

        // Messages
        nav_msgs::msg::Odometry mOdomMsg;
        nav_msgs::msg::Odometry mMapOdomMsg;

        // Broadcasters
        std::shared_ptr<tf2_ros::TransformBroadcaster> mOdomBroadcaster;
        std::shared_ptr<tf2_ros::TransformBroadcaster> mMapOdomBroadcaster;

        bool mBroadcasterInitialized = false;

        std::string mMainNode;
    };

}  // namespace stereolabs

#endif // #define ZED_TF_BROADCASTER_HPP
