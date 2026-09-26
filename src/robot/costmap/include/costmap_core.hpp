#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Builds a fresh costmap (no memory of previous scans) from a single laser scan
    nav_msgs::msg::OccupancyGrid buildCostmap(const sensor_msgs::msg::LaserScan& scan);

  private:
    // Costmap parameters
    static constexpr int kWidth = 600;               // cells
    static constexpr int kHeight = 600;              // cells
    static constexpr double kResolution = 0.05;      // meters per cell
    static constexpr double kInflationRadius = 2.0;  // meters
    static constexpr int kMaxCost = 100;

    // Marks every valid laser hit as an obstacle and returns the (x, y) index of each marked cell
    std::vector<std::pair<int, int>> markObstacles(const sensor_msgs::msg::LaserScan& scan);

    // Spreads decreasing cost around each obstacle cell out to the inflation radius
    void inflateObstacles(const std::vector<std::pair<int, int>>& obstacles);

    bool inBounds(int x_index, int y_index) const;

    rclcpp::Logger logger_;

    // Row-major grid: index = y_index * kWidth + x_index
    std::vector<int8_t> grid_;
};

}  

#endif  
