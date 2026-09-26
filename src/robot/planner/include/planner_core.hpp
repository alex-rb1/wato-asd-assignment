#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstddef>
#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

// Outcome of a planning attempt; the node decides state transitions from it
enum class PlanResult {
  kOffMap,       // start or goal cell is outside the map
  kGoalBlocked,  // goal cell cost is above the blocked threshold
  kNoPath,       // A* could not reach the goal
  kFound,        // path found
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Plans from the robot center to the goal (both in map-frame meters).
    // On kFound, path holds one pose per cell at the cell center; otherwise path.poses is empty.
    PlanResult plan(
      const nav_msgs::msg::OccupancyGrid& map, double start_x, double start_y,
      double goal_x, double goal_y, nav_msgs::msg::Path& path) const;

  private:
    static constexpr int kBlockedThreshold = 30;  // cells with cost > this are blocked
    static constexpr double kCostWeight = 5.0;
    static constexpr int kMaxCost = 100;

    // A* over the map's 8-connected grid; returns start -> goal cells, or empty if unreachable
    std::vector<CellIndex> aStar(
      const nav_msgs::msg::OccupancyGrid& map, const CellIndex& start, const CellIndex& goal) const;

    static CellIndex worldToCell(const nav_msgs::msg::OccupancyGrid& map, double x, double y);
    static bool inBounds(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell);
    // Cell cost, with unknown (-1) treated as 0
    static int cellCost(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell);

    rclcpp::Logger logger_;
};

}  

#endif  
