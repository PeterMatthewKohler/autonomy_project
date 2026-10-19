#ifndef MISSION_NODE_HPP
#define MISSION_NODE_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <queue>
#include <future>
#include <mutex>
#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nova_carter_interfaces/srv/goal.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mission_node
{
enum class NodeState
{
  WAITING_FOR_SERVICE,
  READY_TO_SEND,
  WAITING_FOR_RESPONSE,
  TRACKING,
  COMPLETE
};

class MissionNode : public rclcpp::Node
{
public:
  explicit MissionNode(const rclcpp::NodeOptions & options);

private:
  using Goal = nova_carter_interfaces::srv::Goal;

  void initParameters();
  void initPubSubs();
  void initServices();
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void cmdVelCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void missionLoop();
  void sendGoal(double x, double y);
  bool checkOdomTolerance(const nav_msgs::msg::Odometry odom,
                          const Goal::Request request,
                          double tol);

  rclcpp::Client<Goal>::SharedPtr goalClient_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  rclcpp::TimerBase::SharedPtr missionTimer_;
  std::mutex mutex_;

  std::string odomFrame_;
  double goalTolerance_{};
  double commandTimeoutS_{};
  static constexpr int MISSION_LOOP_PERIOD_MS = 200;
  std::optional<nav_msgs::msg::Odometry> currOdom_;
  std::optional<geometry_msgs::msg::Twist> currCmd_;
  std::optional<rclcpp::Time> lastCmdTime_;
  std::optional<rclcpp::Time> acceptedCmdTime_;
  std::queue<Goal::Request> goalQueue_;
  std::optional<Goal::Request> currGoal_;
  int failures_;
  static constexpr int MAX_FAILURES_ALLOWED = 1;
  NodeState state_;
  std::optional<rclcpp::Client<Goal>::Future> goalFuture_;

};
}  // namespace mission_node

#endif  // MISSION_NODE_HPP
