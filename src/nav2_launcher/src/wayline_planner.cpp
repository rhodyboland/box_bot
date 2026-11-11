// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <mutex>
#include <optional>
#include <utility>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_launcher
{

class WaylinePlanner : public nav2_navfn_planner::NavfnPlanner
{
public:
  void configure(
    const nav2::LifecycleNode::WeakPtr & parent, std::string name,
    nav2::TransformBuffer::SharedPtr tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    NavfnPlanner::configure(parent, std::move(name), std::move(tf), costmap_ros);
    live_map_ = costmap_ros->getCostmap();
    costmap_ = &planning_map_;
    auto node = parent.lock();
    cost_per_metre_ = node->declare_or_get_parameter(name_ + ".wayline_cost_per_metre", 45.0);
    maximum_cost_ = node->declare_or_get_parameter(name_ + ".wayline_maximum_cost", 100.0);
    if (!std::isfinite(cost_per_metre_) || cost_per_metre_ < 0.0 ||
      !std::isfinite(maximum_cost_) || maximum_cost_ < 0.0 || maximum_cost_ > 252.0)
    {
      throw std::invalid_argument("Invalid wayline costs");
    }
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & reference,
    std::function<bool()> cancel_checker) override
  {
    nav2_costmap_2d::Costmap2D obstacle_map;
    {
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live_map_->getMutex());
      obstacle_map = *live_map_;
    }
    const auto & target = goal.pose.position;
    const double resolution = obstacle_map.getResolution();
    // The GPS tree supplies the fixed row start. The planner server supplies
    // the current robot pose separately, including during obstacle detours.
    if (reference.size() != 1 || reference.front().header.frame_id != global_frame_) {
      planning_map_ = obstacle_map;
      return NavfnPlanner::createPlan(start, goal, {}, std::move(cancel_checker));
    }

    const Segment row{reference.front().pose.position, target};
    if (!std::isfinite(row.length) || row.length < resolution) {
      planning_map_ = obstacle_map;
      return NavfnPlanner::createPlan(start, goal, {}, std::move(cancel_checker));
    }
    if (cancel_checker()) {
      throw nav2_core::PlannerCancelled("Wayline planning cancelled");
    }

    // Only straighten through free cells on the original inflated costmap.
    // The cross-track penalty and the grid search are only needed for detours.
    const auto clear_from = clearSuffixStart(obstacle_map, row);
    if (clear_from) {
      auto path = newPath(start);
      if (appendClearSuffix(obstacle_map, row, *clear_from, goal, path)) {
        return path;
      }
    }

    planning_map_ = obstacle_map;
    biasTowardSegment(row.origin, target);
    auto path = NavfnPlanner::createPlan(start, goal, {}, std::move(cancel_checker));
    if (clear_from && !path.poses.empty()) {
      nav_msgs::msg::Path straightened;
      straightened.header = path.header;
      for (const auto & pose : path.poses) {
        straightened.poses.push_back(pose);
        if (appendClearSuffix(obstacle_map, row, *clear_from, goal, straightened)) {
          return straightened;
        }
      }
    }
    return path;
  }

private:
  using Point = geometry_msgs::msg::Point;
  using Costmap = nav2_costmap_2d::Costmap2D;

  struct Segment
  {
    Segment(const Point & start, const Point & end)
    : origin(start), dx(end.x - start.x), dy(end.y - start.y), length(std::hypot(dx, dy)) {}

    Point at(double fraction) const
    {
      Point point = origin;
      point.x += fraction * dx;
      point.y += fraction * dy;
      return point;
    }

    double projection(const Point & point) const
    {
      return ((point.x - origin.x) * dx + (point.y - origin.y) * dy) / (length * length);
    }

    Point origin;
    double dx;
    double dy;
    double length;
  };

  static bool freeAround(const Costmap & map, const Point & point, double radius)
  {
    // The sample squares overlap between consecutive points, including at
    // grid corners. Each square spans at most two cells on each axis.
    for (double dx : {-radius, radius}) {
      for (double dy : {-radius, radius}) {
        unsigned int mx, my;
        if (!map.worldToMap(point.x + dx, point.y + dy, mx, my) ||
          map.getCost(mx, my) != nav2_costmap_2d::FREE_SPACE)
        {
          return false;
        }
      }
    }
    return true;
  }

  static std::optional<double> clearSuffixStart(const Costmap & map, const Segment & row)
  {
    const double spacing = map.getResolution() * 0.5;
    const double map_diagonal = map.getResolution() *
      std::hypot(map.getSizeInCellsX(), map.getSizeInCellsY());
    const double scan_length = std::min(row.length, map_diagonal);
    const auto steps = static_cast<std::size_t>(std::ceil(scan_length / spacing));
    const double step_fraction = (scan_length / row.length) / steps;
    for (std::size_t i = 0; i <= steps; ++i) {
      const double fraction = std::max(0.0, 1.0 - i * step_fraction);
      if (!freeAround(map, row.at(fraction), spacing * 0.5)) {
        if (i == 0) {
          return std::nullopt;
        }
        return std::min(1.0, fraction + step_fraction);
      }
    }
    return 1.0 - scan_length / row.length;
  }

  nav_msgs::msg::Path newPath(const geometry_msgs::msg::PoseStamped & start) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = clock_->now();
    path.poses.push_back(start);
    path.poses.back().header = path.header;
    return path;
  }

  static void appendSegment(
    const Segment & segment, double spacing, nav_msgs::msg::Path & path)
  {
    const auto steps = static_cast<std::size_t>(std::ceil(segment.length / spacing));
    const double yaw = std::atan2(segment.dy, segment.dx);
    for (std::size_t i = 1; i <= steps; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = segment.at(static_cast<double>(i) / steps);
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      path.poses.push_back(pose);
    }
  }

  static bool appendClearSuffix(
    const Costmap & map, const Segment & row, double clear_from,
    const geometry_msgs::msg::PoseStamped & goal, nav_msgs::msg::Path & path)
  {
    const auto & start = path.poses.back().pose.position;
    const double resolution = map.getResolution();
    const double along = row.projection(start);
    const auto projected = row.at(along);
    const double offset = std::hypot(start.x - projected.x, start.y - projected.y);
    if (!std::isfinite(along) || !std::isfinite(offset) ||
      along < clear_from - resolution / row.length || along > 1.0 || offset > resolution)
    {
      return false;
    }
    // Give the robot room to merge forward instead of making a sideways step.
    const double join = std::clamp(
      along + std::max(resolution, 2.0 * offset) / row.length, clear_from, 1.0);
    const Segment merge{start, row.at(join)};
    const double spacing = resolution * 0.5;
    const auto steps = std::max<std::size_t>(1, std::ceil(merge.length / spacing));
    for (std::size_t i = 0; i <= steps; ++i) {
      if (!freeAround(map, merge.at(static_cast<double>(i) / steps), spacing * 0.5)) {
        return false;
      }
    }
    appendSegment(merge, spacing, path);
    appendSegment(Segment{row.at(join), goal.pose.position}, spacing, path);
    path.poses.back().pose = goal.pose;
    return true;
  }

  void biasTowardSegment(
    const geometry_msgs::msg::Point & origin, const geometry_msgs::msg::Point & goal)
  {
    const double dx = goal.x - origin.x;
    const double dy = goal.y - origin.y;
    const double length = std::hypot(dx, dy);
    const double resolution = planning_map_.getResolution();
    if (!std::isfinite(length) || length < resolution ||
      cost_per_metre_ == 0.0 || maximum_cost_ == 0.0)
    {
      return;
    }
    const double ux = dx / length;
    const double uy = dy / length;
    const double first_x = planning_map_.getOriginX() + resolution * 0.5 - origin.x;
    const double first_y = planning_map_.getOriginY() + resolution * 0.5 - origin.y;
    const auto width = planning_map_.getSizeInCellsX();
    const auto height = planning_map_.getSizeInCellsY();
    auto * costs = planning_map_.getCharMap();
    const double maximum_distance = maximum_cost_ / cost_per_metre_;
    constexpr auto maximum_traversable = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
    for (unsigned int y = 0; y < height; ++y) {
      const double py = first_y + y * resolution;
      for (unsigned int x = 0; x < width; ++x, ++costs) {
        if (*costs >= maximum_traversable) {
          continue;
        }
        const double px = first_x + x * resolution;
        const double cross_track = px * uy - py * ux;
        const double along_track = px * ux + py * uy;
        const double past_end = std::max({0.0, -along_track, along_track - length});
        const double distance_sq = cross_track * cross_track + past_end * past_end;
        const double penalty = distance_sq >= maximum_distance * maximum_distance ?
          maximum_cost_ : cost_per_metre_ * std::sqrt(distance_sq);
        *costs = static_cast<unsigned char>(
          std::min(maximum_traversable, *costs + static_cast<int>(std::round(penalty))));
      }
    }
  }

  nav2_costmap_2d::Costmap2D * live_map_ = nullptr;
  nav2_costmap_2d::Costmap2D planning_map_;
  double cost_per_metre_ = 0.0;
  double maximum_cost_ = 0.0;
};

}  // namespace nav2_launcher

PLUGINLIB_EXPORT_CLASS(nav2_launcher::WaylinePlanner, nav2_core::GlobalPlanner)
