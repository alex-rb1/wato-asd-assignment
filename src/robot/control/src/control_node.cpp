#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger()))
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  latest_path_ = msg;
  if (!msg->poses.empty()) {
    stop_sent_ = false;
  }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // /odom/filtered reports the lidar's position; shift back to the robot center
  robot_yaw_ = 2.0 * std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  robot_x_ = msg->pose.pose.position.x - kLidarOffset * std::cos(robot_yaw_);
  robot_y_ = msg->pose.pose.position.y - kLidarOffset * std::sin(robot_yaw_);
  odom_received_ = true;
}

void ControlNode::timerCallback()
{
  if (!odom_received_ || !latest_path_) {
    return;
  }

  if (control_.isFinished(*latest_path_, robot_x_, robot_y_)) {
    if (!stop_sent_) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      stop_sent_ = true;
    }
    return;
  }

  cmd_vel_pub_->publish(control_.computeCommand(*latest_path_, robot_x_, robot_y_, robot_yaw_));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
