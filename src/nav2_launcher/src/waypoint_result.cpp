// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2026 Boxbot contributors
// SPDX-License-Identifier: Apache-2.0
// Nav2 1.5 resultCallback compatibility fix. The executable exports this member
// in place of the library implementation, retaining the rest of Nav2 unchanged.
#include <chrono>
#include "nav2_waypoint_follower/waypoint_follower.hpp"

namespace nav2_waypoint_follower
{
void WaypointFollower::resultCallback(
  const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult & result)
{
  // A prior goal's result can arrive while a replacement goal's acknowledgement
  // is pending. Blocking here prevents this same executor handling that reply.
  if (!future_goal_handle_.valid() ||
    future_goal_handle_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    RCLCPP_DEBUG(get_logger(), "Ignoring old navigation result while goal acknowledgement is pending");
    return;
  }
  const auto handle = future_goal_handle_.get();
  if (!handle || result.goal_id != handle->get_goal_id()) {
    RCLCPP_DEBUG(get_logger(), "Ignoring navigation result for a previous goal");
    return;
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      current_goal_status_.status = ActionStatus::SUCCEEDED;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      current_goal_status_.status = ActionStatus::FAILED;
      current_goal_status_.error_code = result.result->error_code;
      current_goal_status_.error_msg = result.result->error_msg;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      current_goal_status_.status = ActionStatus::FAILED;
      return;
    default:
      current_goal_status_.status = ActionStatus::UNKNOWN;
      current_goal_status_.error_code = nav2_msgs::action::FollowWaypoints::Result::UNKNOWN;
      current_goal_status_.error_msg = "Received an UNKNOWN result code from navigation action!";
      RCLCPP_ERROR(get_logger(), "%s", current_goal_status_.error_msg.c_str());
      return;
  }
}
}  // namespace nav2_waypoint_follower
