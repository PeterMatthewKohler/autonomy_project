#ifndef WAYPOINT_NODE_HPP
#define WAYPOINT_NODE_HPP

#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simple_waypoint_node/srv/goal.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace waypoint_node
{
class PID
{
public:
  PID(double kp, double ki, double kd, double fc, double Ts)
  : kp(kp), ki(ki), kd(kd), alpha(calcAlphaEMA(fc * Ts)), Ts(Ts) {}

  static double calcAlphaEMA(double fn);

  static double wrappedError(double referenceRad, double measuredRad)
  {
    const double difference = referenceRad - measuredRad;
    return std::atan2(std::sin(difference), std::cos(difference));
  }

  // A logger and clock are sufficient; the PID does not need a node pointer.
  void setLogging(
    const rclcpp::Logger & logger, rclcpp::Clock::SharedPtr clock,
    const std::string & label)
  {
    logger_ = logger;
    clock_ = clock;
    label_ = label;
  }

  void reset()
  {
    integral = 0;
    old_ef = 0;
  }

  double update(double reference, double measured)
  {
    return computeControlSignal(reference - measured, reference, measured, false);
  }

  double updateAngle(double referenceRad, double measuredRad)
  {
    return computeControlSignal(
      wrappedError(referenceRad, measuredRad), referenceRad, measuredRad, true);
  }

private:
  double kp, ki, kd, alpha, Ts;
  double integral = 0;
  double old_ef = 0;
  std::optional<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string label_;

  double computeControlSignal(
    double error, double reference, double measured, bool angular)
  {
    const double ef = alpha * error + (1 - alpha) * old_ef;
    const double derivative = (ef - old_ef) / Ts;
    const double new_integral = integral + error * Ts;
    const double control_u = kp * error + ki * integral + kd * derivative;
    integral = new_integral;
    old_ef = ef;

    if (logger_ && clock_) {
      const double displayScale = angular ? 180.0 / M_PI : 1.0;
      RCLCPP_INFO_STREAM_THROTTLE(
        *logger_, *clock_, 1000,
        label_ << " PID: reference=" << reference * displayScale <<
          ", measured=" << measured * displayScale <<
          ", error=" << error * displayScale << (angular ? " deg" : " m") <<
          ", output=" << control_u * displayScale << (angular ? " deg/s" : " m/s"));
    }
    return control_u;
  }
};

class WaypointNode : public rclcpp::Node
{
public:
  explicit WaypointNode(const rclcpp::NodeOptions & options);

private:
  void initParameters();
  void initPubSubs();
  void initServices();
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void initializeGoal(
    std::shared_ptr<simple_waypoint_node::srv::Goal::Request> request,
    std::shared_ptr<simple_waypoint_node::srv::Goal::Response> response);
  void controlLoop();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Service<simple_waypoint_node::srv::Goal>::SharedPtr goalService_;
  rclcpp::TimerBase::SharedPtr controlTimer_;

  // Declare the buffer first so the listener is destroyed before it.
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;
  std::mutex mutex_;

  std::string odomFrame_;
  std::string baseFrame_;
  double goalDistance_{};
  double goalTolerance_{};
  double headingTolerance_{};
  double maxLinearSpeed_{};
  double maxAngularSpeed_{};

  // Positions are meters; yaw is radians.
  std::optional<double> currX_;
  std::optional<double> currY_;
  std::optional<double> currYawRad_;
  std::optional<double> goalX_;
  std::optional<double> goalY_;
  bool haveGoal_{false};

  PID latPID;
  PID rotPID;
  static constexpr int CONTROL_LOOP_PERIOD_MS = 50;
};
}  // namespace waypoint_node

#endif  // WAYPOINT_NODE_HPP
