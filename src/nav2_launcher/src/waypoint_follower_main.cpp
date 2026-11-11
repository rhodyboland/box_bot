// SPDX-License-Identifier: Apache-2.0
#include <memory>
#include "nav2_waypoint_follower/waypoint_follower.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<nav2_waypoint_follower::WaypointFollower>(rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
