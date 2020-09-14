
#include <rclcpp/rclcpp.hpp>
#include "zed_components/zed_camera_component.hpp"

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize any global resources needed by the middleware and the client library.
    // This will also parse command line arguments one day (as of Beta 1 they are not used).
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // Create an executor that will be responsible for execution of callbacks for a set of nodes.
    // With this version, all callbacks will be called from within this thread (the main one).
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    // Add zed_camera node
    auto zed_node = std::make_shared<stereolabs::ZedCamera>(options);
    exec.add_node(zed_node);

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    exec.spin();

    rclcpp::shutdown();

    return 0;
}

