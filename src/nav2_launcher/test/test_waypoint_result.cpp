// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <future>
#include <memory>
#include "gtest/gtest.h"
#include "nav2_waypoint_follower/waypoint_follower.hpp"

class ResultHarness : public nav2_waypoint_follower::WaypointFollower
{
public:
  ResultHarness() : WaypointFollower(rclcpp::NodeOptions()) {}
  using WaypointFollower::resultCallback;
  using WaypointFollower::future_goal_handle_;
};

TEST(WaypointResult, PendingReplacementAcknowledgementCannotBlockOldResult)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<ResultHarness>();
    std::promise<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> promise;
    node->future_goal_handle_ = promise.get_future().share();
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result;
    result.code = rclcpp_action::ResultCode::CANCELED;
    auto start = std::chrono::steady_clock::now();
    node->resultCallback(result);
    EXPECT_LT(std::chrono::steady_clock::now()-start, std::chrono::milliseconds(100));
    promise.set_value(nullptr);
    EXPECT_NO_THROW(node->resultCallback(result));
  }
  rclcpp::shutdown();
}
