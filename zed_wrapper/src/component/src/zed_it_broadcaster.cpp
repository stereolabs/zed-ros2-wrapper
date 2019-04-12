#include "zed_it_broadcaster.hpp"
#include <chrono>
#include <thread>

using namespace std::placeholders;

namespace stereolabs {

    ZedItBroadcaster::ZedItBroadcaster(const std::string& node_name /*= "zed_it_broadcaster"*/,
                                       const std::string& ros_namespace /*= "zed"*/,
                                       const std::string& main_node,
                                       bool intra_process_comms /*= true*/)
        : Node(node_name, ros_namespace, intra_process_comms) {

#ifndef NDEBUG
        rcutils_ret_t res = rcutils_logging_set_logger_level(get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting DEBUG logger");
        }

#endif

        RCLCPP_INFO(get_logger(), "***************************************************");
        RCLCPP_INFO(get_logger(), " ZED Image Transport Broadcaster Component created");
        RCLCPP_INFO(get_logger(), "  * namespace: %s", get_namespace());
        RCLCPP_INFO(get_logger(), "  * node name: %s", get_name());
        RCLCPP_INFO(get_logger(), "***************************************************");

        // Topics
        std::string topicPrefix = get_namespace();

        if (topicPrefix.length() > 1) {
            topicPrefix += "/";
        }

        mMainNode = main_node;

        topicPrefix += mMainNode;
        topicPrefix += "/";

        // Parameters
        initParameters();

        // Video topics
        std::string img_topic = "/image_rect_color";
        std::string img_raw_topic = "/image_raw_color";
        std::string raw_suffix = "_raw";
        std::string it_prefix = "it_";
        // Depth topics
        std::string depth_topic = "/depth_registered";
        std::string depth_openni_topic = "/depth_raw_registered";

        // Image Transport output topic names
        mRgbTopic = topicPrefix + it_prefix + mRgbTopicRoot + img_topic;
        mRightTopic = topicPrefix + it_prefix + mRightTopicRoot + img_topic;
        mLeftTopic = topicPrefix + it_prefix + mLeftTopicRoot + img_topic;
        mRawRgbTopic = topicPrefix + it_prefix + mRgbTopicRoot + raw_suffix + img_raw_topic;
        mRawRightTopic = topicPrefix + it_prefix + mRightTopicRoot + raw_suffix + img_raw_topic;
        mRawLeftTopic = topicPrefix + it_prefix + mLeftTopicRoot + raw_suffix + img_raw_topic;
        mDepthTopic = topicPrefix + it_prefix + mDepthTopicRoot + (mOpenniDepthMode ? depth_openni_topic :
                      depth_topic);

        // Advertise publishers
        initPublishers();

        // Subscribers checking
        std::chrono::milliseconds checkPeriodMsec(100);

        mSubTimer = create_wall_timer(std::chrono::duration_cast<std::chrono::microseconds>(checkPeriodMsec),
                                      std::bind(&ZedItBroadcaster::checkSubscribersCallback, this));
    }

    ZedItBroadcaster::ZedItBroadcaster(
        const std::string& node_name,
        const std::string& ros_namespace,
        const std::string& main_node,
        rclcpp::Context::SharedPtr context,
        const std::vector<std::string>& arguments,
        const std::vector<rclcpp::Parameter>& initial_parameters,
        bool use_global_arguments /*= true*/,
        bool use_intra_process_comms /*= false*/,
        bool start_parameter_services /*= true*/)
        : Node(node_name, ros_namespace, context, arguments, initial_parameters,
               use_global_arguments, use_intra_process_comms, start_parameter_services) {

#ifndef NDEBUG
        rcutils_ret_t res = rcutils_logging_set_logger_level(get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting DEBUG logger");
        }

#endif

        RCLCPP_INFO(get_logger(), "ZED Image Transport Broadcaster Component created");

        RCLCPP_INFO(get_logger(), "ZED Image Transport Broadcaster namespace: %s", get_namespace());
        RCLCPP_INFO(get_logger(), "ZED Image Transport Broadcaster node: %s", get_name());

        // Topics
        std::string topicPrefix = get_namespace();

        if (topicPrefix.length() > 1) {
            topicPrefix += "/";
        }

        mMainNode = main_node;

        topicPrefix += mMainNode;
        topicPrefix += "/";

        // Parameters
        initParameters();

        // Video topics
        std::string img_topic = "/image_rect_color";
        std::string img_raw_topic = "/image_raw_color";
        std::string raw_suffix = "_raw";
        std::string it_prefix = "it_";
        // Depth topics
        std::string depth_topic = "/depth_registered";
        std::string depth_openni_topic = "/depth_raw_registered";

        // Image Transport output topic names
        mRgbTopic = topicPrefix + it_prefix + mRgbTopicRoot + img_topic;
        mRightTopic = topicPrefix + it_prefix + mRightTopicRoot + img_topic;
        mLeftTopic = topicPrefix + it_prefix + mLeftTopicRoot + img_topic;
        mRawRgbTopic = topicPrefix + it_prefix + mRgbTopicRoot + raw_suffix + img_raw_topic;
        mRawRightTopic = topicPrefix + it_prefix + mRightTopicRoot + raw_suffix + img_raw_topic;
        mRawLeftTopic = topicPrefix + it_prefix + mLeftTopicRoot + raw_suffix + img_raw_topic;
        mDepthTopic = topicPrefix + it_prefix + mDepthTopicRoot + (mOpenniDepthMode ? depth_openni_topic :
                      depth_topic);

        // Advertise publishers
        initPublishers();

        // Subscribers checking
        std::chrono::milliseconds checkPeriodMsec(100);

        mSubTimer = create_wall_timer(std::chrono::duration_cast<std::chrono::microseconds>(checkPeriodMsec),
                                      std::bind(&ZedItBroadcaster::checkSubscribersCallback, this));
    }

    void ZedItBroadcaster::initParameters() {
        rclcpp::Parameter paramVal;
        std::string paramName;

        paramName = "video.rgb_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mRgbTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.left_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mLeftTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.right_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mRightTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.openni_depth_mode";

        if (get_parameter(paramName, paramVal)) {
            mOpenniDepthMode = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.depth_topic_root";

        if (get_parameter(paramName, paramVal)) {
            mDepthTopicRoot = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.qos_history";

        if (get_parameter(paramName, paramVal)) {
            mVideoQos.history = paramVal.as_int() == 0 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.qos_depth";

        if (get_parameter(paramName, paramVal)) {
            mVideoQos.depth = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.qos_reliability";

        if (get_parameter(paramName, paramVal)) {
            mVideoQos.reliability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT :
                                    RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "video.qos_durability";

        if (get_parameter(paramName, paramVal)) {
            mVideoQos.durability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL :
                                   RMW_QOS_POLICY_DURABILITY_VOLATILE;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.qos_history";

        if (get_parameter(paramName, paramVal)) {
            mDepthQos.history = paramVal.as_int() == 0 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.qos_depth";

        if (get_parameter(paramName, paramVal)) {
            mDepthQos.depth = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.qos_reliability";

        if (get_parameter(paramName, paramVal)) {
            mDepthQos.reliability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT :
                                    RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "depth.qos_durability";

        if (get_parameter(paramName, paramVal)) {
            mDepthQos.durability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL :
                                   RMW_QOS_POLICY_DURABILITY_VOLATILE;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
    }

    void ZedItBroadcaster::checkSubscribersCallback() {
        initSubscribers();
    }

    void ZedItBroadcaster::initSubscribers() {

        // Topics
        std::string topicPrefix = get_namespace();

        if (topicPrefix.length() > 1) {
            topicPrefix += "/";
        }

        topicPrefix += get_name();
        topicPrefix += "/";

        // Video topics
        std::string img_topic = "/image_rect_color";
        std::string img_raw_topic = "/image_raw_color";
        std::string cam_info_topic = "/camera_info";
        std::string raw_suffix = "_raw";
        // Depth topics
        std::string depth_topic = "/depth_registered";
        std::string depth_openni_topic = "/depth_raw_registered";

        // Set the default input topic names
        std::string leftTopic = topicPrefix + mLeftTopicRoot + img_topic;
        std::string leftCamInfoTopic = leftTopic + cam_info_topic;
        std::string leftRawTopic = topicPrefix + mLeftTopicRoot + raw_suffix + img_raw_topic;
        std::string leftCamInfoRawTopic = leftRawTopic + cam_info_topic;

        std::string rightTopic = topicPrefix + mRightTopicRoot + img_topic;
        std::string rightCamInfoTopic = rightTopic + cam_info_topic;
        std::string rightRawTopic = topicPrefix + mRightTopicRoot + raw_suffix + img_raw_topic;
        std::string rightCamInfoRawTopic = rightRawTopic + cam_info_topic;

        std::string rgbTopic = topicPrefix + mRgbTopicRoot + img_topic;
        std::string rgbCamInfoTopic = rgbTopic + cam_info_topic;
        std::string rgbRawTopic = topicPrefix + mRgbTopicRoot + raw_suffix + img_raw_topic;
        std::string rgbCamInfoRawTopic = rgbRawTopic + cam_info_topic;

        std::string depthTopic = topicPrefix + mDepthTopicRoot + (mOpenniDepthMode ? depth_openni_topic :
                                 depth_topic);
        std::string depthCamInfoTopic = depthTopic + cam_info_topic;

        // Subscribers QoS
        bool rgbSub = checkVideoSubs(mRgbTopic, mRgbPub.getInfoTopic());
        bool rgbRawSub = checkVideoSubs(mRawRgbTopic, mRawRgbPub.getInfoTopic());
        bool leftSub = checkVideoSubs(mLeftTopic, mLeftPub.getInfoTopic());
        bool leftRawSub = checkVideoSubs(mRawLeftTopic, mRawLeftPub.getInfoTopic());
        bool rightSub = checkVideoSubs(mRightTopic, mRightPub.getInfoTopic());
        bool rightRawSub = checkVideoSubs(mRawRightTopic, mRawRightPub.getInfoTopic());
        bool depthSub = checkVideoSubs(mDepthTopic, mDepthPub.getInfoTopic());

        // Video Subscribers
        if (rgbSub  && (!mRgbSub || !mRgbInfoSub)) {
            mRgbSub = create_subscription<sensor_msgs::msg::Image>(
                          rgbTopic,
                          std::bind(&ZedItBroadcaster::rgbCallback, this, _1),
                          mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRgbSub->get_topic_name());

            mRgbInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                              rgbCamInfoTopic,
                              std::bind(&ZedItBroadcaster::rgbInfoCallback, this, _1),
                              mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRgbInfoSub->get_topic_name());
        } else {
            if (!rgbSub  && (mRgbSub || mRgbInfoSub)) {
                mRgbSub.reset();
                mRgbInfoSub.reset();

                RCLCPP_INFO(get_logger(), " * RGB Unsubscribed");
            }
        }

        if (rightSub  && (!mRightSub || !mRightInfoSub)) {
            mRightSub = create_subscription<sensor_msgs::msg::Image>(
                            rightTopic,
                            std::bind(&ZedItBroadcaster::rightCallback, this, _1),
                            mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRightSub->get_topic_name());

            mRightInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                                rightCamInfoTopic,
                                std::bind(&ZedItBroadcaster::rightInfoCallback, this, _1),
                                mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRightInfoSub->get_topic_name());
        } else {
            if (!rightSub  && (mRightSub || mRightInfoSub)) {
                mRightSub.reset();
                mRightInfoSub.reset();

                RCLCPP_DEBUG(get_logger(), " * Right Unsubscribed");
            }
        }

        if (leftSub  && (!mLeftSub || !mLeftInfoSub)) {
            mLeftSub = create_subscription<sensor_msgs::msg::Image>(
                           leftTopic,
                           std::bind(&ZedItBroadcaster::leftCallback, this, _1),
                           mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mLeftSub->get_topic_name());


            mLeftInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                               leftCamInfoTopic,
                               std::bind(&ZedItBroadcaster::leftInfoCallback, this, _1),
                               mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mLeftInfoSub->get_topic_name());
        } else {
            if (!leftSub  && (mLeftSub || mLeftInfoSub)) {
                mLeftSub.reset();
                mLeftInfoSub.reset();

                RCLCPP_DEBUG(get_logger(), " * Left Unsubscribed");
            }
        }

        if (rgbRawSub  && (!mRawRgbSub || !mRawRgbInfoSub)) {
            mRawRgbSub = create_subscription<sensor_msgs::msg::Image>(
                             rgbRawTopic,
                             std::bind(&ZedItBroadcaster::rgbRawCallback, this, _1),
                             mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRgbSub->get_topic_name());

            mRawRgbInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                                 rgbCamInfoRawTopic,
                                 std::bind(&ZedItBroadcaster::rgbInfoRawCallback, this, _1),
                                 mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRgbInfoSub->get_topic_name());
        } else {
            if (!rgbRawSub  && (mRawRgbSub || mRawRgbInfoSub)) {
                mRawRgbSub.reset();
                mRawRgbInfoSub.reset();

                RCLCPP_DEBUG(get_logger(), " * Raw RGB Unsubscribed");
            }
        }

        if (rightRawSub  && (!mRawRightSub || !mRawRightInfoSub)) {
            mRawRightSub = create_subscription<sensor_msgs::msg::Image>(
                               rightRawTopic,
                               std::bind(&ZedItBroadcaster::rightRawCallback, this, _1),
                               mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRightSub->get_topic_name());

            mRawRightInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                                   rightCamInfoRawTopic,
                                   std::bind(&ZedItBroadcaster::rightInfoRawCallback, this, _1),
                                   mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawRightInfoSub->get_topic_name());
        } else {
            if (!rightRawSub  && (mRawRightSub || mRawRightInfoSub)) {
                mRawRightSub.reset();
                mRawRightInfoSub.reset();

                RCLCPP_INFO(get_logger(), " * Raw Right Unsubscribed");
            }
        }

        if (leftRawSub  && (!mRawLeftSub || !mRawLeftInfoSub)) {
            mRawLeftSub = create_subscription<sensor_msgs::msg::Image>(
                              leftRawTopic,
                              std::bind(&ZedItBroadcaster::leftRawCallback, this, _1),
                              mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawLeftSub->get_topic_name());


            mRawLeftInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                                  leftCamInfoRawTopic,
                                  std::bind(&ZedItBroadcaster::leftInfoRawCallback, this, _1),
                                  mVideoQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mRawLeftInfoSub->get_topic_name());
        } else {
            if (!leftRawSub  && (mRawLeftSub || mRawLeftInfoSub)) {
                mRawLeftSub.reset();
                mRawLeftInfoSub.reset();

                RCLCPP_DEBUG(get_logger(), " * Raw Left Unsubscribed");
            }
        }

        if (depthSub  && (!mDepthSub || !mDepthInfoSub)) {
            // Depth Subsubscribers
            mDepthSub = create_subscription<sensor_msgs::msg::Image>(
                            depthTopic,
                            std::bind(&ZedItBroadcaster::depthCallback, this, _1),
                            mDepthQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mDepthSub->get_topic_name());

            mDepthInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                                depthCamInfoTopic,
                                std::bind(&ZedItBroadcaster::depthInfoCallback, this, _1),
                                mDepthQos);
            RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mDepthInfoSub->get_topic_name());
        } else {
            if (!depthSub  && (mDepthSub || mDepthInfoSub)) {
                mDepthSub.reset();
                mDepthInfoSub.reset();

                RCLCPP_INFO(get_logger(), " * Depth Unsubscribed");
            }
        }
    }

    void ZedItBroadcaster::initPublishers() {
        if (mPubInitialized) {
            return;
        }

        mRgbPub = image_transport::create_camera_publisher(this, mRgbTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mRgbPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mRgbPub.getInfoTopic().c_str());

        mRightPub = image_transport::create_camera_publisher(this, mRightTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mRightPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mRightPub.getInfoTopic().c_str());

        mLeftPub = image_transport::create_camera_publisher(this, mLeftTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mLeftPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mLeftPub.getInfoTopic().c_str());

        mRawRgbPub = image_transport::create_camera_publisher(this, mRawRgbTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mRawRgbPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mRawRgbPub.getInfoTopic().c_str());

        mRawRightPub = image_transport::create_camera_publisher(this, mRawRightTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mRawRightPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mRawRightPub.getInfoTopic().c_str());

        mRawLeftPub = image_transport::create_camera_publisher(this, mRawLeftTopic, mVideoQos);
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mRawLeftPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mRawLeftPub.getInfoTopic().c_str());

        mDepthPub = image_transport::create_camera_publisher(this, mDepthTopic, mDepthQos);
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mDepthPub.getTopic().c_str());
        RCLCPP_INFO(get_logger(), " * IT Advertised '%s'", mDepthPub.getInfoTopic().c_str());

        mPubInitialized = true;
    }

    void ZedItBroadcaster::rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        mRgbPub.publish(*msg, mRgbInfoMsg);
    }

    void ZedItBroadcaster::rgbInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPublishers();

        mRgbInfoMsg = *msg;
    }

    void ZedItBroadcaster::rightCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        mRightPub.publish(*msg, mRightInfoMsg);
    }

    void ZedItBroadcaster::rightInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPublishers();

        mRightInfoMsg = *msg;
    }

    void ZedItBroadcaster::leftCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        mLeftPub.publish(*msg, mLeftInfoMsg);
    }

    void ZedItBroadcaster::leftInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPublishers();

        mLeftInfoMsg = *msg;
    }

    void ZedItBroadcaster::rgbRawCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        mRawRgbPub.publish(*msg, mRawRgbInfoMsg);
    }

    void ZedItBroadcaster::rgbInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPublishers();

        mRawRgbInfoMsg = *msg;
    }

    void ZedItBroadcaster::rightRawCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        mRawRightPub.publish(*msg, mRawRightInfoMsg);
    }

    void ZedItBroadcaster::rightInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPublishers();

        mRawRightInfoMsg = *msg;
    }

    void ZedItBroadcaster::leftRawCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        mRawLeftPub.publish(*msg, mRawLeftInfoMsg);
    }

    void ZedItBroadcaster::leftInfoRawCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPublishers();

        mRawLeftInfoMsg = *msg;
    }

    void ZedItBroadcaster::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mPubInitialized) {
            return;
        }

        mDepthPub.publish(*msg, mDepthInfoMsg);
    }

    void ZedItBroadcaster::depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        initPublishers();

        mDepthInfoMsg = *msg;
    }

    bool ZedItBroadcaster::checkVideoSubs(std::string topic, std::string camInfoTopic) {
        size_t subs = 0;

        subs += count_subscribers(topic);        // Image topic subscribers
        subs += count_subscribers(camInfoTopic); // Camera Info topic subscribers

        subs += count_subscribers(topic + "/compressed"); // Compressed image subscribers
        subs += count_subscribers(topic + "/theora");     // Compressed Theora image subscribers

        return (subs > 0);
    }

    bool ZedItBroadcaster::checkDepthSubs(std::string topic, std::string camInfoTopic) {
        size_t subs = 0;

        subs += count_subscribers(topic);        // Image topic subscribers
        subs += count_subscribers(camInfoTopic); // Camera Info topic subscribers

        subs += count_subscribers(topic + "/compressedDepth"); // Compressed image subscribers

        return (subs > 0);
    }

}  // namespace stereolabs
