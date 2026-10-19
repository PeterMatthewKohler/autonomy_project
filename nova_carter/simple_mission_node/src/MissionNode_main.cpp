#include "mission_node/MissionNode_impl.hpp"

#include <memory>

#include <rclcpp/executors/static_single_threaded_executor.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto node = std::make_shared<mission_node::MissionNode>(options);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
