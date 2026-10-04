#include "waypoint_node/WaypointNode_impl.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include <rclcpp/create_timer.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace waypoint_node
{
WaypointNode::WaypointNode(const rclcpp::NodeOptions & options)
: Node("waypoint_node", options),
  latPID(1.0, 0.0, 0.0, 10.0, CONTROL_LOOP_PERIOD_MS / 1000.0),
  rotPID(1.0, 0.0, 0.0, 10.0, CONTROL_LOOP_PERIOD_MS / 1000.0)
{
  initParameters();
  initPubSubs();
  initServices();
  latPID.setLogging(get_logger(), get_clock(), "Linear");
  rotPID.setLogging(get_logger(), get_clock(), "Angular");

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_, this, false);
  controlTimer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(CONTROL_LOOP_PERIOD_MS),
    std::bind(&WaypointNode::controlLoop, this));
}

void WaypointNode::initParameters()
{
  odomFrame_ = declare_parameter<std::string>("odom_frame", "odom");
  baseFrame_ = declare_parameter<std::string>("base_frame", "base_link");
  goalDistance_ = declare_parameter<double>("goal_distance", 1.0);
  goalTolerance_ = declare_parameter<double>("goal_tolerance", 0.15);
  headingTolerance_ = declare_parameter<double>("heading_tolerance", 0.25);
  maxLinearSpeed_ = declare_parameter<double>("max_linear_speed", 0.3);
  maxAngularSpeed_ = declare_parameter<double>("max_angular_speed", 0.6);
}

void WaypointNode::initPubSubs()
{
  cmdVelPub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/chassis/odom", 10,
    std::bind(&WaypointNode::odomCallback, this, std::placeholders::_1));
}

void WaypointNode::initServices()
{
  goalService_ = create_service<simple_waypoint_node::srv::Goal>(
    "goal_service",
    std::bind(
      &WaypointNode::initializeGoal, this,
      std::placeholders::_1, std::placeholders::_2));
}

void WaypointNode::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (msg->header.frame_id != odomFrame_ || msg->child_frame_id != baseFrame_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Expected odometry frames " << odomFrame_ << " -> " << baseFrame_ <<
        ", received " << msg->header.frame_id << " -> " << msg->child_frame_id);
    return;
  }

  const auto & position = msg->pose.pose.position;
  const auto & orientation = msg->pose.pose.orientation;
  tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(quat.length2()) || quat.length2() <= 0.0)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid odometry pose");
    return;
  }
  double rollRad, pitchRad, yawRad;
  tf2::Matrix3x3(quat).getRPY(rollRad, pitchRad, yawRad);

  std::lock_guard<std::mutex> lock(mutex_);
  currX_ = position.x;
  currY_ = position.y;
  currYawRad_ = yawRad;
}

void WaypointNode::initializeGoal(
  std::shared_ptr<simple_waypoint_node::srv::Goal::Request> request,
  std::shared_ptr<simple_waypoint_node::srv::Goal::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (haveGoal_) {
    response->success = false;
    response->message = "Already working on a different goal. Please wait.";
    return;
  }
  if (!std::isfinite(request->x) || !std::isfinite(request->y)) {
    response->success = false;
    response->message = "Goal coordinates must be finite.";
    return;
  }

  goalX_ = request->x;
  goalY_ = request->y;
  latPID.reset();
  rotPID.reset();
  haveGoal_ = true;
  response->success = true;
  response->message = "Goal set.";
  RCLCPP_INFO_STREAM(
    get_logger(), "Goal accepted in " << odomFrame_ <<
      ": x=" << *goalX_ << " m, y=" << *goalY_ << " m");
}

void WaypointNode::controlLoop()
{
  // Keep pose, goal, and PID state consistent with the other callbacks.
  std::lock_guard<std::mutex> lock(mutex_);
  if (!haveGoal_) {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  if (!currX_ || !currY_ || !currYawRad_ || !goalX_ || !goalY_) {
    cmdVelPub_->publish(cmd);
    return;
  }

  const double dx = *goalX_ - *currX_;
  const double dy = *goalY_ - *currY_;
  const double distanceError = std::hypot(dx, dy);
  // At arrival, heading toward the point is irrelevant (and may be undefined).
  const double desiredHeading = distanceError > goalTolerance_ ?
    std::atan2(dy, dx) : *currYawRad_;
  const double headingError = PID::wrappedError(desiredHeading, *currYawRad_);

  if (distanceError <= goalTolerance_) {
    haveGoal_ = false;
    latPID.reset();
    rotPID.reset();
    RCLCPP_INFO_STREAM(get_logger(), "Goal reached; distance error=" << distanceError << " m");
  } else {
    // The reference is zero remaining distance. Forward motion reduces that
    // measurement, so map the PID's negative output to positive forward speed.
    cmd.linear.x = std::clamp(
      -latPID.update(0.0, distanceError), 0.0, maxLinearSpeed_);
    cmd.angular.z = std::clamp(
      rotPID.updateAngle(desiredHeading, *currYawRad_),
      -maxAngularSpeed_, maxAngularSpeed_);
  }

  RCLCPP_INFO_STREAM_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Traversal: distance error=" << distanceError <<
      " m, heading error=" << headingError * (180.0 / M_PI) <<
      " deg, aligned=" << (std::abs(headingError) <= headingTolerance_) <<
      ", v=" << cmd.linear.x <<
      " m/s, w=" << cmd.angular.z * (180.0 / M_PI) << " deg/s");
  cmdVelPub_->publish(cmd);
}

double PID::calcAlphaEMA(double fn)
{
  if (fn <= 0) {
    return 1;
  }
  const double c = std::cos(2 * M_PI * fn);
  return c - 1 + std::sqrt(c * c - 4 * c + 3);
}
}  // namespace waypoint_node
