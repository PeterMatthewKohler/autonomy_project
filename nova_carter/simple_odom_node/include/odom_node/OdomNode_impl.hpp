#ifndef ODOM_NODE_HPP
#define ODOM_NODE_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace odom_node {
    class OdomNode : public rclcpp::Node
    {
        public:
        OdomNode(const rclcpp::NodeOptions& options);

        private:
        void initPubSubs(); // Helper
        // Publishers
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr odomPub_;
        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
        void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    };
}   // namespace odom_node


#endif // ODOM_NODE_HPP