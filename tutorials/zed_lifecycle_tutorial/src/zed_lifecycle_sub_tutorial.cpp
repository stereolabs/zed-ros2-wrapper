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
 * This tutorial demonstrates simple receipt of ZED lifecycle transition messages over the ROS system.
 */

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

class MinimalLifecycleSubscriber : public rclcpp::Node {
  public:
    MinimalLifecycleSubscriber()
        : Node("zed_lifecycle_tutorial") {

        std::string topic = "/zed/zed_node/transition_event";

        mSub = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
                   topic,
        [this](lifecycle_msgs::msg::TransitionEvent::UniquePtr msgPtr) {
            lifecycle_msgs::msg::TransitionEvent msg = *msgPtr.get();

            RCLCPP_INFO(get_logger(), "Transition received: ");

            RCLCPP_INFO(get_logger(), "\t'%s [%d]' -> '%s [%d]'",
                        msg.start_state.label.c_str(), msg.start_state.id,
                        msg.goal_state.label.c_str(), msg.goal_state.id);
        });

        RCLCPP_INFO(get_logger(), "Waiting for transition messages on topic `%s`... ",
                    topic.c_str());
    }

  private:
    rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr mSub;
};

// The main function
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalLifecycleSubscriber>());
    rclcpp::shutdown();
    return 0;
}
