#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = msg;
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  // Replan. The map is always present here, so only odometry can be missing.
  if (!odom_received_) {
    RCLCPP_WARN(this->get_logger(), "No odometry yet; dropping goal");
    state_ = State::WAITING_FOR_GOAL;
    return;
  }
  planToGoal();
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  if (!latest_map_ || !odom_received_) {
    // Keep the goal; the next /map message triggers a replan
    RCLCPP_WARN(this->get_logger(), "No map or odometry yet; will plan when the next map arrives");
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    return;
  }
  planToGoal();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // /odom/filtered reports the lidar's position; shift back to the robot center
  const double yaw = 2.0 * std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  robot_x_ = msg->pose.pose.position.x - kLidarOffset * std::cos(yaw);
  robot_y_ = msg->pose.pose.position.y - kLidarOffset * std::sin(yaw);
  odom_received_ = true;
}

void PlannerNode::timerCallback()
{
  // Without odometry the robot position is unknown, so it can't have reached the goal
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL || !odom_received_) {
    return;
  }

  const double distance = std::hypot(goal_.point.x - robot_x_, goal_.point.y - robot_y_);
  if (distance < kGoalTolerance) {
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    publishEmptyPath();
    state_ = State::WAITING_FOR_GOAL;
  }
}

void PlannerNode::planToGoal()
{
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = latest_map_->header.frame_id;
  const robot::PlanResult result = planner_.plan(
    *latest_map_, robot_x_, robot_y_, goal_.point.x, goal_.point.y, path);

  switch (result) {
    case robot::PlanResult::kFound:
      path_pub_->publish(path);
      state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
      break;
    case robot::PlanResult::kNoPath:
      path_pub_->publish(path);  // empty
      state_ = State::WAITING_FOR_GOAL;
      break;
    case robot::PlanResult::kOffMap:
    case robot::PlanResult::kGoalBlocked:
      state_ = State::WAITING_FOR_GOAL;
      break;
  }
}

void PlannerNode::publishEmptyPath()
{
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  if (latest_map_) {
    path.header.frame_id = latest_map_->header.frame_id;
  }
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
