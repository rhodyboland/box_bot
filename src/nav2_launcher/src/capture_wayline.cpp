// SPDX-License-Identifier: Apache-2.0
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_launcher
{

class CaptureWayline : public BT::DecoratorNode
{
public:
  CaptureWayline(const std::string & name, const BT::NodeConfig & config)
  : BT::DecoratorNode(name, config)
  {
    auto node = config.blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
    tf_ = config.blackboard->get<nav2::TransformBuffer::SharedPtr>("tf_buffer");
    node->get_parameter_or("global_frame", global_frame_, std::string("map"));
    node->get_parameter_or("robot_base_frame", base_frame_, std::string("base_link"));
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("reference")
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped goal;
    const auto update = getInputStamped("goal", goal);
    if (!update) {
      return BT::NodeStatus::FAILURE;
    }
    if (goal_sequence_ != update->seq) {
      geometry_msgs::msg::PoseStamped anchor;
      if (!nav2_util::getCurrentPose(anchor, *tf_, global_frame_, base_frame_, 0.0)) {
        return BT::NodeStatus::RUNNING;
      }
      setOutput("reference", std::vector<geometry_msgs::msg::PoseStamped>{anchor});
      goal_sequence_ = update->seq;
    }
    return child_node_->executeTick();
  }

  void halt() override
  {
    goal_sequence_.reset();
    BT::DecoratorNode::halt();
  }

private:
  nav2::TransformBuffer::SharedPtr tf_;
  std::string global_frame_;
  std::string base_frame_;
  std::optional<uint64_t> goal_sequence_;
};

}  // namespace nav2_launcher

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_launcher::CaptureWayline>("CaptureWayline");
}
