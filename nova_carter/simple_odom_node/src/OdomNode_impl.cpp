#include "odom_node/OdomNode_impl.hpp"

namespace odom_node {
  OdomNode::OdomNode(const rclcpp::NodeOptions& options) : Node("odom_node", options) {
    // Init pub/subs
    initPubSubs();
  }

  void OdomNode::initPubSubs() {
    // Publishers
    odomPub_ = this->create_publisher<std_msgs::msg::String>("odom_output",
                                                             10);
    // Subscribers
    odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>("/chassis/odom",
                                                                  10,
                                                                  std::bind(&OdomNode::odomCallback, 
                                                                            this,
                                                                            std::placeholders::_1));
  }

  void OdomNode::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    // Pull the x, y, and forward velocity components, and publish them as a string
    std_msgs::msg::String strMsg;
    strMsg.data = "x: " + std::to_string(msg->pose.pose.position.x) +
                  ", y: " + std::to_string(msg->pose.pose.position.y) +
                  ", x_vel: " + std::to_string(msg->twist.twist.linear.x);
    odomPub_->publish(strMsg);
  }

} // namespace odom_node
