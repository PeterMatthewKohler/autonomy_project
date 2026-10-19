#include "mission_node/MissionNode_impl.hpp"

#include <chrono>
#include <functional>

#include <rclcpp/create_timer.hpp>

namespace mission_node
{
MissionNode::MissionNode(const rclcpp::NodeOptions & options)
: Node("mission_node", options)
{
  initParameters();
  initPubSubs();
  initServices();

  missionTimer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(MISSION_LOOP_PERIOD_MS),
    std::bind(&MissionNode::missionLoop, this));
  
  Goal::Request goal;
  goal.x = 1.0;
  goal.y = 1.0;
  goalQueue_.push(goal);
  goal.x += 2.0;
  goal.y -= 2.0;
  goalQueue_.push(goal);
  goal.x = 0.0;
  goal.y = 0.0;
  goalQueue_.push(goal);
  // Init to WAITING_FOR_SERVICE
  state_ = NodeState::WAITING_FOR_SERVICE;
  failures_ = 0;
  RCLCPP_INFO_STREAM(
    get_logger(), "WAITING_FOR_SERVICE: " << goalQueue_.size() <<
      " goals in " << odomFrame_ << ", tolerance=" << goalTolerance_ <<
      " m, attempt timeout=" << commandTimeoutS_ << " s; waiting for /goal_service.");
}

void MissionNode::initParameters()
{
  odomFrame_ = declare_parameter<std::string>("odom_frame", "odom");
  goalTolerance_ = declare_parameter<double>("goal_tolerance", 0.15);
  commandTimeoutS_ = declare_parameter<double>("command_timeout_s", 10);
}

void MissionNode::initPubSubs()
{
  odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/chassis/odom", 10,
    std::bind(&MissionNode::odomCallback, this, std::placeholders::_1));
  cmdVelSub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&MissionNode::cmdVelCallback, this, std::placeholders::_1));
}

void MissionNode::initServices()
{
  goalClient_ = create_client<Goal>("/goal_service");
}

void MissionNode::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (msg->header.frame_id != odomFrame_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Expected odometry frames " << odomFrame_ << ", received " 
      << msg->header.frame_id << " -> " << msg->child_frame_id);
    return;
  }
  std::lock_guard<std::mutex> lock_guard(mutex_);
  currOdom_.emplace(*msg);
}

void MissionNode::cmdVelCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  const auto receiptTime = now();
  std::lock_guard<std::mutex> lock_guard(mutex_);
  currCmd_ = *msg;
  lastCmdTime_ = receiptTime;
}

void MissionNode::missionLoop()
{
  // TODO: Implement your mission states, transitions, arrival checks, and retries.
  switch(state_)
  {
    case NodeState::WAITING_FOR_SERVICE :
    {
      // Poll service until it's ready, then transition to READY_TO_SEND
      if(goalClient_->service_is_ready())
      {
        std::lock_guard<std::mutex> lock_guard(mutex_);
        state_ = NodeState::READY_TO_SEND;
        RCLCPP_INFO_STREAM(
          get_logger(), "WAITING_FOR_SERVICE -> READY_TO_SEND: /goal_service available.");
      }
      break;
    }
    case NodeState::READY_TO_SEND :
    {
      // Handle failure with a retry
      Goal::Request goal;
      if (currGoal_.has_value())
      {
        goal = currGoal_.value();
      }
      else if (!goalQueue_.empty())
      {
        goal = goalQueue_.front();
        goalQueue_.pop();
        currGoal_.emplace(goal);
      }
      else // we're done
      {
        std::lock_guard<std::mutex> lock_guard(mutex_);
        state_ = NodeState::COMPLETE;
        RCLCPP_INFO_STREAM(
          get_logger(), "READY_TO_SEND -> COMPLETE: no queued goals remain; no further requests.");
        return;  
      }
      sendGoal(goal.x, goal.y);
      std::lock_guard<std::mutex> lock_guard(mutex_);
      state_ = NodeState::WAITING_FOR_RESPONSE;
      RCLCPP_INFO_STREAM(
        get_logger(), "READY_TO_SEND -> WAITING_FOR_RESPONSE: sending goal=(" <<
          goal.x << ", " << goal.y << ") m, retry=" << failures_ << "/" <<
          MAX_FAILURES_ALLOWED << ", remaining queued=" << goalQueue_.size());
      break;
    }
    case NodeState::WAITING_FOR_RESPONSE :
    {
      if (goalFuture_.has_value() &&
          goalFuture_.value().valid() &&
          goalFuture_.value().wait_for(std::chrono::seconds(0)) == std::future_status::ready)
      {
        const auto response = goalFuture_.value().get();
        // If accepted, start tracking
        if(response->success)
        {
          state_ = NodeState::TRACKING;
          acceptedCmdTime_.emplace(now());
          RCLCPP_INFO_STREAM(
            get_logger(), "WAITING_FOR_RESPONSE -> TRACKING: goal accepted; response=\"" <<
              response->message << "\"");
        }
        else // If request was rejected, retry the current goal
        {
          state_ = NodeState::READY_TO_SEND;
          RCLCPP_WARN_STREAM(
            get_logger(), "WAITING_FOR_RESPONSE -> READY_TO_SEND: goal rejected; " <<
              "retrying the same goal. response=\"" << response->message << "\"");
        }
      }
      break;
    }
    case NodeState::TRACKING :
    {
      // 3 cases to leave:
      // - we are inside our goal distance tolerance (success)
      // - controller commands are zero (if not success, failure)
      // - timeout occurs (failure)
      bool timedOut = false;
      bool arrived = false;
      bool controllerStopped = false;
      // Check for timeout
      if(acceptedCmdTime_.has_value())
      {
        timedOut = now().seconds() - acceptedCmdTime_.value().seconds() >= commandTimeoutS_;
      }
      // Have we arrived at our goal?
      nav_msgs::msg::Odometry odom;
      rclcpp::Time lastCmdTime, acceptedCmdTime;
      geometry_msgs::msg::Twist currCmd;
      if(currOdom_.has_value())
      {
        {
          std::lock_guard<std::mutex> lock_guard(mutex_);
          odom = currOdom_.value();
        }
        arrived = checkOdomTolerance(odom, currGoal_.value(), goalTolerance_);
      }
      // Only check for zero controller commands on recent valid commands
      if(currCmd_.has_value() && lastCmdTime_.has_value() && acceptedCmdTime_.has_value())
      {
        {
          std::lock_guard<std::mutex> lock_guard(mutex_);
          currCmd = currCmd_.value();
          lastCmdTime = lastCmdTime_.value();
          acceptedCmdTime = acceptedCmdTime_.value();
        }
        if(lastCmdTime.seconds() > acceptedCmdTime.seconds())
        {
          controllerStopped = (currCmd.linear.x == 0.0 && currCmd.angular.z == 0);
        }
      }
      // If we've arrived, we can move on to next goal
      if(arrived)
      {
        RCLCPP_INFO_STREAM(
          get_logger(), "TRACKING -> READY_TO_SEND: reached goal=(" << currGoal_->x <<
            ", " << currGoal_->y << ") m at position=(" << odom.pose.pose.position.x <<
            ", " << odom.pose.pose.position.y << ") m.");
        currGoal_.reset();
        failures_ = 0;
        state_ = NodeState::READY_TO_SEND;
      }
      else if(timedOut || controllerStopped)
      {
        if(failures_ >= MAX_FAILURES_ALLOWED)
        {
          RCLCPP_WARN_STREAM(
            get_logger(), "TRACKING -> READY_TO_SEND: skipping goal=(" << currGoal_->x <<
              ", " << currGoal_->y << ") m; retries exhausted (" << failures_ << "/" <<
              MAX_FAILURES_ALLOWED << "), timed_out=" << timedOut <<
              ", controller_stopped=" << controllerStopped);
          currGoal_.reset();
          failures_ = 0;
        }
        else
        {
          failures_++;
          RCLCPP_WARN_STREAM(
            get_logger(), "TRACKING -> READY_TO_SEND: retrying goal=(" << currGoal_->x <<
              ", " << currGoal_->y << ") m, retry=" << failures_ << "/" <<
              MAX_FAILURES_ALLOWED << ", timed_out=" << timedOut <<
              ", controller_stopped=" << controllerStopped);
        }
        state_ = NodeState::READY_TO_SEND;
      }
      break;
    }
    case NodeState::COMPLETE : {break;}
    default : {break;}
  }
}

void MissionNode::sendGoal(double x, double y)
{
  auto request = std::make_shared<Goal::Request>();
  request->x = x;
  request->y = y;
  auto pending = goalClient_->async_send_request(request);
  goalFuture_.emplace(std::move(pending.future));
}

bool MissionNode::checkOdomTolerance(const nav_msgs::msg::Odometry odom,
                                     const Goal::Request request,
                                     double tol)
{
  // Calc euclidean distance from current position to request
  const double dx = std::abs(odom.pose.pose.position.x - request.x);
  const double dy = std::abs(odom.pose.pose.position.y - request.y);
  const double dist = std::sqrt((dx * dx) + (dy * dy));
  return (dist <= tol);
}

}  // namespace mission_node
