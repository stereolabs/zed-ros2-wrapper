#include "zed_tf_broadcaster.hpp"

using namespace std::placeholders;

namespace stereolabs {

    ZedTfBroadcaster::ZedTfBroadcaster(const std::string& node_name /*= "zed_node_tf"*/,
                                       const std::string& ros_namespace /*= "zed"*/,
                                       const std::string& main_node /*= "zed_node"*/,
                                       bool intra_process_comms /*= true*/)
        : Node(node_name, ros_namespace, intra_process_comms) {

#ifndef NDEBUG
        rcutils_ret_t res = rcutils_logging_set_logger_level(get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        if (res != RCUTILS_RET_OK) {
            RCLCPP_INFO(get_logger(), "Error setting DEBUG logger");
        }

#endif

        RCLCPP_INFO(get_logger(), "ZED TF Broadcaster Component created");

        RCLCPP_INFO(get_logger(), "ZED TF Broadcaster namespace: %s", get_namespace());
        RCLCPP_INFO(get_logger(), "ZED TF Broadcaster node: %s", get_name());

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

        // Initialize subscribers
        initSubscribers();
    }

    ZedTfBroadcaster::ZedTfBroadcaster(
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

        RCLCPP_INFO(get_logger(), "**************************************");
        RCLCPP_INFO(get_logger(), " ZED TF Broadcaster Component created");
        RCLCPP_INFO(get_logger(), "  * namespace: %s", get_namespace());
        RCLCPP_INFO(get_logger(), "  * node name: %s", get_name());
        RCLCPP_INFO(get_logger(), "************************************");

        mMainNode = main_node;

        // Parameters
        initParameters();

        // Initialize subscribers
        initSubscribers();
    }

    void ZedTfBroadcaster::initParameters() {
        rclcpp::Parameter paramVal;
        std::string paramName;

        paramName = "tracking.odometry_topic";

        if (get_parameter(paramName, paramVal)) {
            mOdomTopic = paramVal.as_string();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "tracking.publish_tf";

        if (get_parameter(paramName, paramVal)) {
            mPublishTf = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "tracking.publish_map_tf";

        if (get_parameter(paramName, paramVal)) {
            mPublishMapTf = paramVal.as_bool();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "tracking.qos_history";

        if (get_parameter(paramName, paramVal)) {
            mPoseQos.history = paramVal.as_int() == 0 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "tracking.qos_depth";

        if (get_parameter(paramName, paramVal)) {
            mPoseQos.depth = paramVal.as_int();
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "tracking.qos_reliability";

        if (get_parameter(paramName, paramVal)) {
            mPoseQos.reliability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT :
                                   RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }

        paramName = "tracking.qos_durability";

        if (get_parameter(paramName, paramVal)) {
            mPoseQos.durability = paramVal.as_int() == 0 ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL :
                                  RMW_QOS_POLICY_DURABILITY_VOLATILE;
        } else {
            RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
        }
    }

    void ZedTfBroadcaster::initSubscribers() {

        // Topics
        std::string topicPrefix = get_namespace();

        if (topicPrefix.length() > 1) {
            topicPrefix += "/";
        }

        if ('/' != topicPrefix.at(0)) {
            topicPrefix = '/'  + topicPrefix;
        }

        topicPrefix += mMainNode;
        topicPrefix += "/";

        // Topics
        mOdomTopic = topicPrefix + mOdomTopic;
        mMapOdomTopic = topicPrefix + mMapOdomTopic;

        // Subscribers
        mOdomSub = create_subscription<nav_msgs::msg::Odometry>(
                       mOdomTopic,
                       std::bind(&ZedTfBroadcaster::odomCallback, this, _1),
                       mPoseQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mOdomSub->get_topic_name());

        mMapOdomSub = create_subscription<nav_msgs::msg::Odometry>(
                          mMapOdomTopic,
                          std::bind(&ZedTfBroadcaster::mapOdomCallback, this, _1),
                          mPoseQos);
        RCLCPP_INFO(get_logger(), " * Subscribed to '%s'", mMapOdomSub->get_topic_name());
    }

    void ZedTfBroadcaster::initBroadcasters() {
        if (mBroadcasterInitialized) {
            return;
        }

        mOdomBroadcaster.reset(new tf2_ros::TransformBroadcaster(shared_from_this()));
        mMapOdomBroadcaster.reset(new tf2_ros::TransformBroadcaster(shared_from_this()));

        mBroadcasterInitialized = true;
    }

    void ZedTfBroadcaster::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!mBroadcasterInitialized) {
            initBroadcasters();
        }

        geometry_msgs::msg::TransformStamped tf;

        rclcpp::Time delayed(msg->header.stamp.sec, msg->header.stamp.nanosec + 33000000);

        tf.header.frame_id = msg->header.frame_id;
        tf.header.stamp = delayed;

        tf.child_frame_id = msg->child_frame_id;
        tf.transform.translation.x = msg->pose.pose.position.x;
        tf.transform.translation.y = msg->pose.pose.position.y;
        tf.transform.translation.z = msg->pose.pose.position.z;
        tf.transform.rotation.x = msg->pose.pose.orientation.x;
        tf.transform.rotation.y = msg->pose.pose.orientation.y;
        tf.transform.rotation.z = msg->pose.pose.orientation.z;
        tf.transform.rotation.w = msg->pose.pose.orientation.w;

        mOdomBroadcaster->sendTransform(tf);
    }

    void ZedTfBroadcaster::mapOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!mBroadcasterInitialized) {
            initBroadcasters();
        }

        geometry_msgs::msg::TransformStamped tf;

        rclcpp::Time delayed(msg->header.stamp.sec, msg->header.stamp.nanosec + 33000000);

        tf.header.frame_id = msg->header.frame_id;
        tf.header.stamp = delayed;

        tf.child_frame_id = msg->child_frame_id;
        tf.transform.translation.x = msg->pose.pose.position.x;
        tf.transform.translation.y = msg->pose.pose.position.y;
        tf.transform.translation.z = msg->pose.pose.position.z;
        tf.transform.rotation.x = msg->pose.pose.orientation.x;
        tf.transform.rotation.y = msg->pose.pose.orientation.y;
        tf.transform.rotation.z = msg->pose.pose.orientation.z;
        tf.transform.rotation.w = msg->pose.pose.orientation.w;

        mMapOdomBroadcaster->sendTransform(tf);
    }
}  // namespace stereolabs
