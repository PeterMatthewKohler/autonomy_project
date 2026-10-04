#include "waypoint_node/WaypointNode_impl.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto pNode = std::make_shared<waypoint_node::WaypointNode>(options);
  executor.add_node(pNode);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
