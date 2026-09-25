#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Yaw from a flat-ground orientation quaternion (x and y are 0)
    static double yawFromQuaternion(double z, double w);

    // Merges a robot-frame costmap into the global map, given the robot pose in the world frame
    void fuseCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw);

    // Returns the current global map, stamped with the given time
    nav_msgs::msg::OccupancyGrid getMap(const rclcpp::Time& stamp) const;

  private:
    // Global map parameters
    static constexpr int kSize = 400;               // cells (width and height)
    static constexpr double kResolution = 0.1;      // meters per cell
    static constexpr double kOriginX = -(kSize * kResolution) / 2.0;
    static constexpr double kOriginY = -(kSize * kResolution) / 2.0;
    static constexpr const char* kFrameId = "sim_world";  // frame /odom/filtered is reported in

    rclcpp::Logger logger_;

    // Row-major grid: index = gy * kSize + gx
    std::vector<int8_t> global_map_;
};

}  

#endif  
