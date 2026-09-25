#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
: logger_(logger), grid_(kWidth * kHeight, 0) {}

nav_msgs::msg::OccupancyGrid CostmapCore::buildCostmap(const sensor_msgs::msg::LaserScan& scan)
{
  // 1. Each costmap is a fresh snapshot
  std::fill(grid_.begin(), grid_.end(), 0);

  // 2. Mark all obstacles first, 3. then inflate, so inflated cells are never treated as obstacles
  const auto obstacles = markObstacles(scan);
  inflateObstacles(obstacles);

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = scan.header.stamp;
  msg.header.frame_id = scan.header.frame_id;
  msg.info.resolution = kResolution;
  msg.info.width = kWidth;
  msg.info.height = kHeight;
  // Position of cell (0, 0) relative to the robot, which sits at the grid center
  msg.info.origin.position.x = -(kWidth * kResolution) / 2.0;
  msg.info.origin.position.y = -(kHeight * kResolution) / 2.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data = grid_;
  return msg;
}

std::vector<std::pair<int, int>> CostmapCore::markObstacles(const sensor_msgs::msg::LaserScan& scan)
{
  std::vector<std::pair<int, int>> obstacles;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range <= scan.range_min || range >= scan.range_max) {
      continue;
    }

    const double angle = scan.angle_min + i * scan.angle_increment;
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);

    // std::floor so negative coordinates round toward -infinity, not toward zero
    const int x_index = static_cast<int>(std::floor(x / kResolution)) + kWidth / 2;
    const int y_index = static_cast<int>(std::floor(y / kResolution)) + kHeight / 2;
    if (!inBounds(x_index, y_index)) {
      continue;
    }

    grid_[y_index * kWidth + x_index] = kMaxCost;
    obstacles.emplace_back(x_index, y_index);
  }

  return obstacles;
}

void CostmapCore::inflateObstacles(const std::vector<std::pair<int, int>>& obstacles)
{
  const int radius_in_cells = static_cast<int>(std::ceil(kInflationRadius / kResolution));

  for (const auto& [obstacle_x, obstacle_y] : obstacles) {
    for (int dy = -radius_in_cells; dy <= radius_in_cells; ++dy) {
      for (int dx = -radius_in_cells; dx <= radius_in_cells; ++dx) {
        const int x_index = obstacle_x + dx;
        const int y_index = obstacle_y + dy;
        if (!inBounds(x_index, y_index)) {
          continue;
        }

        const double distance = std::hypot(dx, dy) * kResolution;
        if (distance > kInflationRadius) {
          continue;
        }

        const int8_t cost = static_cast<int8_t>(
          std::floor(kMaxCost * (1.0 - distance / kInflationRadius)));
        int8_t& cell = grid_[y_index * kWidth + x_index];
        if (cost > cell) {
          cell = cost;
        }
      }
    }
  }
}

bool CostmapCore::inBounds(int x_index, int y_index) const
{
  return x_index >= 0 && x_index < kWidth && y_index >= 0 && y_index < kHeight;
}

}
