#ifndef ZED_IT_BROADCASTER_HPP
#define ZED_IT_BROADCASTER_HPP

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
#include <image_transport/image_transport.h>

namespace stereolabs {

    typedef rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub;
    typedef rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoSub;

    class ZedItBroadcaster : public rclcpp::Node {
      public:
        /// Create a new node with the specified name.
        /**
         * \param[in] node_name Name of the node.
         * \param[in] namespace_ Namespace of the node.
         * \param[in] use_intra_process_comms True to use the optimized intra-process communication
         * pipeline to pass messages between nodes in the same process using shared memory.
         */
        ZED_PUBLIC
        explicit ZedItBroadcaster(const std::string& node_name = "zed_node_it",
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
        explicit ZedItBroadcaster(const std::string& node_name,
                                  const std::string& ros_namespace,
                                  const std::string& main_node,
                                  rclcpp::Context::SharedPtr context,
                                  const std::vector<std::string>& arguments,
                                  const std::vector<rclcpp::Parameter>& initial_parameters,
                                  bool use_global_arguments = true,
                                  bool use_intra_process_comms = false,
                                  bool start_parameter_services = true);

      protected:
        void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rgbInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void rightCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rightInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void leftCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void leftInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        void rgbRawCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rgbInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void rightRawCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rightInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void leftRawCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void leftInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        void initSubscribers();
        void initParameters();
        void initPublishers();

        void checkSubscribersCallback();

        bool checkVideoSubs(std::string topic, std::string camInfoTopic);
        bool checkDepthSubs(std::string topic, std::string camInfoTopic);

      private:
        // Params
        bool mOpenniDepthMode = false;
        std::string mLeftTopicRoot = "left";
        std::string mRightTopicRoot = "right";
        std::string mRgbTopicRoot = "rgb";
        std::string mDepthTopicRoot = "depth";

        // Check Subscribers Timer
        rclcpp::TimerBase::SharedPtr mSubTimer = nullptr;

        // Subscribers
        imgSub mRgbSub;
        camInfoSub mRgbInfoSub;
        imgSub mRightSub;
        camInfoSub mRightInfoSub;
        imgSub mLeftSub;
        camInfoSub mLeftInfoSub;
        imgSub mRawRgbSub;
        camInfoSub mRawRgbInfoSub;
        imgSub mRawRightSub;
        camInfoSub mRawRightInfoSub;
        imgSub mRawLeftSub;
        camInfoSub mRawLeftInfoSub;
        imgSub mDepthSub;
        camInfoSub mDepthInfoSub;

        // QoS profiles
        // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
        rmw_qos_profile_t mVideoQos = rmw_qos_profile_default;
        rmw_qos_profile_t mDepthQos = rmw_qos_profile_default;
        rmw_qos_profile_t mImuQos = rmw_qos_profile_default;

        // Publisher topics
        std::string mRgbTopic;
        std::string mRightTopic;
        std::string mLeftTopic;
        std::string mRawRgbTopic;
        std::string mRawRightTopic;
        std::string mRawLeftTopic;
        std::string mDepthTopic;

        // Camera Info Messages
        sensor_msgs::msg::CameraInfo mRgbInfoMsg;
        sensor_msgs::msg::CameraInfo mRightInfoMsg;
        sensor_msgs::msg::CameraInfo mLeftInfoMsg;
        sensor_msgs::msg::CameraInfo mRawRgbInfoMsg;
        sensor_msgs::msg::CameraInfo mRawRightInfoMsg;
        sensor_msgs::msg::CameraInfo mRawLeftInfoMsg;
        sensor_msgs::msg::CameraInfo mDepthInfoMsg;

        // Camera Publishers
        bool mPubInitialized = false;
        image_transport::CameraPublisher mRgbPub;
        image_transport::CameraPublisher mRightPub;
        image_transport::CameraPublisher mLeftPub;
        image_transport::CameraPublisher mRawRgbPub;
        image_transport::CameraPublisher mRawRightPub;
        image_transport::CameraPublisher mRawLeftPub;
        image_transport::CameraPublisher mDepthPub;

        std::string mMainNode;
    };

}  // namespace stereolabs

#endif // #define ZED_IT_BROADCASTER_HPP
