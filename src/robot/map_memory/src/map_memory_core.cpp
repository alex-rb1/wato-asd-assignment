#include <algorithm>
#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger), global_map_(kSize * kSize, 0) {}

double MapMemoryCore::yawFromQuaternion(double z, double w)
{
  return 2.0 * std::atan2(z, w);
}

void MapMemoryCore::fuseCostmap(
  const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw)
{
  const int width = static_cast<int>(costmap.info.width);
  const int height = static_cast<int>(costmap.info.height);
  const double costmap_resolution = costmap.info.resolution;
  const double costmap_origin_x = costmap.info.origin.position.x;
  const double costmap_origin_y = costmap.info.origin.position.y;
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      // Only obstacle and halo cells are fused; 0 means "no information"
      const int8_t cost = costmap.data[j * width + i];
      if (cost <= 0) {
        continue;
      }

      // Costmap cell center -> robot-frame meters
      const double a = costmap_origin_x + (i + 0.5) * costmap_resolution;
      const double b = costmap_origin_y + (j + 0.5) * costmap_resolution;

      // Robot-frame meters -> world meters
      const double world_x = robot_x + a * cos_yaw - b * sin_yaw;
      const double world_y = robot_y + a * sin_yaw + b * cos_yaw;

      // World meters -> global cell; std::floor so negative values round toward -infinity
      const int gx = static_cast<int>(std::floor((world_x - kOriginX) / kResolution));
      const int gy = static_cast<int>(std::floor((world_y - kOriginY) / kResolution));
      if (gx < 0 || gx >= kSize || gy < 0 || gy >= kSize) {
        continue;
      }

      int8_t& cell = global_map_[gy * kSize + gx];
      cell = std::max(cell, cost);
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getMap(const rclcpp::Time& stamp) const
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = kFrameId;
  msg.info.resolution = kResolution;
  msg.info.width = kSize;
  msg.info.height = kSize;
  msg.info.origin.position.x = kOriginX;
  msg.info.origin.position.y = kOriginY;
  msg.info.origin.orientation.w = 1.0;
  msg.data = global_map_;
  return msg;
}

} 
