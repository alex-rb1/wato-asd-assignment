#include <algorithm>
#include <limits>

#include "control_core.hpp"

namespace robot
{

namespace
{

double distanceTo(const geometry_msgs::msg::PoseStamped& pose, double x, double y)
{
  return std::hypot(pose.pose.position.x - x, pose.pose.position.y - y);
}

}  // namespace

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

bool ControlCore::isFinished(const nav_msgs::msg::Path& path, double robot_x, double robot_y) const
{
  return path.poses.empty() || distanceTo(path.poses.back(), robot_x, robot_y) < kGoalTolerance;
}

geometry_msgs::msg::Twist ControlCore::computeCommand(
  const nav_msgs::msg::Path& path, double robot_x, double robot_y, double robot_yaw) const
{
  const auto& target = path.poses[findLookaheadIndex(path, robot_x, robot_y)].pose.position;

  // Lookahead point in the robot's frame (rotate by -yaw)
  const double dx = target.x - robot_x;
  const double dy = target.y - robot_y;
  const double local_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  const double angle_to_target = std::atan2(local_y, local_x);

  geometry_msgs::msg::Twist cmd;
  if (std::abs(angle_to_target) > kTurnInPlaceAngle) {
    // Turn in place
    cmd.linear.x = 0.0;
    cmd.angular.z = std::copysign(kMaxAngularSpeed, angle_to_target);
  } else {
    // Pure Pursuit. L is never 0: isFinished stops the robot within kGoalTolerance of the last pose.
    const double L = std::hypot(local_x, local_y);
    const double curvature = 2.0 * local_y / (L * L);
    cmd.linear.x = kLinearSpeed;
    cmd.angular.z = std::clamp(kLinearSpeed * curvature, -kMaxAngularSpeed, kMaxAngularSpeed);
  }
  return cmd;
}

std::size_t ControlCore::findLookaheadIndex(
  const nav_msgs::msg::Path& path, double robot_x, double robot_y) const
{
  std::size_t closest = 0;
  double closest_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double distance = distanceTo(path.poses[i], robot_x, robot_y);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest = i;
    }
  }

  for (std::size_t i = closest; i < path.poses.size(); ++i) {
    if (distanceTo(path.poses[i], robot_x, robot_y) >= kLookahead) {
      return i;
    }
  }
  return path.poses.size() - 1;
}

}  
