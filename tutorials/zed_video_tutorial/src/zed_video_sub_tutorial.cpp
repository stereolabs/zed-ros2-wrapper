///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
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
///////////////////////////////////////////////////////////////////////////

/**
 * This tutorial demonstrates simple receipt of ZED video messages over the ROS system.
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs//msg/image.hpp"

rclcpp::Node::SharedPtr g_node = nullptr;

/**
 * Subscriber callbacks. The argument of the callback is a constant pointer to the received message
 */

void imageRightRectifiedCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(g_node->get_logger(), "Right Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
}

void imageLeftRectifiedCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(g_node->get_logger(), "Left Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    g_node = rclcpp::Node::make_shared("zed_video_tutorial");

    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
    rmw_qos_profile_t camera_qos_profile = rmw_qos_profile_sensor_data;

    /* Note: it is very important to use a QOS profile for the subscriber that is compatible
     * with the QOS profile of the publisher.
     * The ZED node uses a "rmw_qos_profile_sensor_data" profile for depth data,
     * so reliability is "BEST_EFFORT" and durability is "VOLATILE".
     * To be able to receive the subscribed topic the subscriber must use the
     * same parameters, so setting the QOS to "rmw_qos_profile_sensor_data" as the publisher
     * is the better solution.
     */

    auto sub_right = g_node->create_subscription<sensor_msgs::msg::Image>
                     ("/zed/zed_node/right/image_rect_color", imageRightRectifiedCallback,
                      rmw_qos_profile_sensor_data);

    auto sub_left = g_node->create_subscription<sensor_msgs::msg::Image>
                    ("/zed/zed_node/left/image_rect_color", imageLeftRectifiedCallback,
                     rmw_qos_profile_sensor_data);

    rclcpp::spin(g_node);
    rclcpp::shutdown();

    // It would be better to remove both of these nullptr
    // assignments and let the destructors handle it, but we can't because of
    // https://github.com/eProsima/Fast-RTPS/issues/235 .  Once that is fixed
    // we should probably look at removing these two assignments.
    //subscription = nullptr;
    //g_node = nullptr;

    return 0;
}
