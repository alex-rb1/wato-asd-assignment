#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    static constexpr double kLidarOffset = 0.8;    // meters the lidar sits in front of the robot center
    static constexpr double kGoalTolerance = 0.5;  // meters

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // Every 500 ms: check whether the robot has reached the goal
    void timerCallback();

    // Plans to the stored goal (map and odometry must be present) and updates the state from the result
    void planToGoal();

    void publishEmptyPath();

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::WAITING_FOR_GOAL;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;  // nullptr until the first map arrives
    geometry_msgs::msg::PointStamped goal_;

    // Robot center (not the lidar) in the odometry frame
    bool odom_received_ = false;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
};

#endif 
