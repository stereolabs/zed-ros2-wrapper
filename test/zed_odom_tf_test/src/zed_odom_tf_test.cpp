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

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>

#include <string>

using namespace std::placeholders;

class OdomTFTest : public rclcpp::Node {
  public:
    OdomTFTest()
        : Node("zed_odom_tf_test") {

        // ----> TF2 Transform
        mTfBuffer.reset(new tf2_ros::Buffer(this->get_clock()));
        mTfListener.reset(new tf2_ros::TransformListener(*mTfBuffer));
        // <---- TF2 Transform

        // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
        rmw_qos_profile_t topic_qos_profile = rmw_qos_profile_sensor_data;

        mSub = create_subscription<nav_msgs::msg::Odometry>(
                   "/zed/zed_node/map2odom",
                   std::bind(&OdomTFTest::odomCallback, this, _1),
                   topic_qos_profile);
    }

  protected:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped tf;

        try {
            tf = mTfBuffer->lookupTransform(msg->header.frame_id, msg->child_frame_id, tf2::TimePointZero, std::chrono::seconds(2));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "The tf from '%s' to '%s' does not seem to be available, "
                        "will assume it as identity!",
                        msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
            RCLCPP_WARN(get_logger(), "Transform error: %s", ex.what());

            return;
        }

        double msg_nsec = static_cast<double>(rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).nanoseconds());
        double now_nsec = static_cast<double>(this->now().nanoseconds());
        double tf_nsec = static_cast<double>(rclcpp::Time(tf.header.stamp.sec, tf.header.stamp.nanosec).nanoseconds());

        double msgDelay = (now_nsec - msg_nsec) / 1000000000.;
        double tfDelay = (msg_nsec - tf_nsec) / 1000000000.;

        RCLCPP_INFO(get_logger(), "[%s -> %s] Msg delay: %g sec - TF delay: %g sec", msg->header.frame_id.c_str(),
                    msg->child_frame_id.c_str(), msgDelay, tfDelay);
    }

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mSub;

    // initialization Transform listener
    std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> mTfListener;
};

// The main function
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto test_node = std::make_shared<OdomTFTest>();

    rclcpp::spin(test_node);
    rclcpp::shutdown();
    return 0;
}
