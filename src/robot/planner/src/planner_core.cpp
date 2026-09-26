#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

PlanResult PlannerCore::plan(
  const nav_msgs::msg::OccupancyGrid& map, double start_x, double start_y,
  double goal_x, double goal_y, nav_msgs::msg::Path& path) const
{
  path.poses.clear();

  const CellIndex start = worldToCell(map, start_x, start_y);
  const CellIndex goal = worldToCell(map, goal_x, goal_y);
  if (!inBounds(map, start) || !inBounds(map, goal)) {
    RCLCPP_WARN(logger_, "Start or goal is off the map; not planning");
    return PlanResult::kOffMap;
  }

  if (cellCost(map, goal) > kBlockedThreshold) {
    RCLCPP_WARN(logger_, "Goal cell is blocked (cost %d); rejecting goal", cellCost(map, goal));
    return PlanResult::kGoalBlocked;
  }

  const std::vector<CellIndex> cells = aStar(map, start, goal);
  if (cells.empty()) {
    RCLCPP_WARN(logger_, "A* found no path to the goal");
    return PlanResult::kNoPath;
  }

  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  for (const auto& cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    // Center of the cell
    pose.pose.position.x = origin_x + (cell.x + 0.5) * resolution;
    pose.pose.position.y = origin_y + (cell.y + 0.5) * resolution;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return PlanResult::kFound;
}

std::vector<CellIndex> PlannerCore::aStar(
  const nav_msgs::msg::OccupancyGrid& map, const CellIndex& start, const CellIndex& goal) const
{
  // Straight-line distance to the goal, in cells
  const auto heuristic = [&goal](const CellIndex& cell) {
    return std::hypot(cell.x - goal.x, cell.y - goal.y);
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start));

  while (!open_set.empty()) {
    const CellIndex current = open_set.top().index;
    open_set.pop();

    // The queue can hold stale duplicates of cells already expanded
    if (closed_set.count(current)) {
      continue;
    }
    closed_set.insert(current);

    if (current == goal) {
      std::vector<CellIndex> path{current};
      CellIndex cell = current;
      while (cell != start) {
        cell = came_from.at(cell);
        path.push_back(cell);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    const int current_cost = cellCost(map, current);

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }

        const CellIndex neighbor(current.x + dx, current.y + dy);
        if (!inBounds(map, neighbor) || closed_set.count(neighbor)) {
          continue;
        }

        const int neighbor_cost = cellCost(map, neighbor);
        const bool free = neighbor_cost <= kBlockedThreshold;
        // Lets a robot that starts too close to an obstacle plan its way out of the blocked zone
        const bool escaping = current_cost > kBlockedThreshold &&
          neighbor_cost < current_cost && neighbor_cost < kMaxCost;
        if (!free && !escaping) {
          continue;
        }

        const double step_length = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
        const double step_cost = step_length * (1.0 + kCostWeight * neighbor_cost / 100.0);
        const double tentative_g = g_score[current] + step_cost;

        const auto it = g_score.find(neighbor);
        if (it == g_score.end() || tentative_g < it->second) {
          g_score[neighbor] = tentative_g;
          came_from[neighbor] = current;
          open_set.emplace(neighbor, tentative_g + heuristic(neighbor));
        }
      }
    }
  }

  return {};
}

CellIndex PlannerCore::worldToCell(const nav_msgs::msg::OccupancyGrid& map, double x, double y)
{
  const double resolution = map.info.resolution;
  return CellIndex(
    static_cast<int>(std::floor((x - map.info.origin.position.x) / resolution)),
    static_cast<int>(std::floor((y - map.info.origin.position.y) / resolution)));
}

bool PlannerCore::inBounds(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell)
{
  return cell.x >= 0 && cell.x < static_cast<int>(map.info.width) &&
         cell.y >= 0 && cell.y < static_cast<int>(map.info.height);
}

int PlannerCore::cellCost(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell)
{
  const int cost = map.data[cell.y * map.info.width + cell.x];
  return cost < 0 ? 0 : cost;
}

} 
