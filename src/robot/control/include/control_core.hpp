#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <cmath>
#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    // True if the path is empty or the robot center is within the goal tolerance of its last pose
    bool isFinished(const nav_msgs::msg::Path& path, double robot_x, double robot_y) const;

    // Turn-in-place or Pure Pursuit command toward the lookahead point on a non-empty path
    geometry_msgs::msg::Twist computeCommand(
      const nav_msgs::msg::Path& path, double robot_x, double robot_y, double robot_yaw) const;

  private:
    static constexpr double kLookahead = 1.5;                        // meters
    static constexpr double kLinearSpeed = 0.5;                      // m/s
    static constexpr double kMaxAngularSpeed = 1.0;                  // rad/s
    static constexpr double kGoalTolerance = 0.5;                    // meters
    static constexpr double kTurnInPlaceAngle = 60.0 * M_PI / 180.0; // radians

    // Index of the path pose to steer toward
    std::size_t findLookaheadIndex(const nav_msgs::msg::Path& path, double robot_x, double robot_y) const;

    rclcpp::Logger logger_;
};

} 

#endif 
