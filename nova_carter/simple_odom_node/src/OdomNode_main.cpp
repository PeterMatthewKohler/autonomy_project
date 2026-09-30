#include "odom_node/OdomNode_impl.hpp"
#include <cstdio>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto pOdomNode = std::make_shared<odom_node::OdomNode>(options);
    executor.add_node(pOdomNode);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}