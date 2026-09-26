#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    static constexpr double kLidarOffset = 0.8;  // meters the lidar sits in front of the robot center

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // Control loop, every 100 ms
    void timerCallback();

    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path::SharedPtr latest_path_;  // nullptr until the first path arrives

    // Robot center (not the lidar) in the odometry frame
    bool odom_received_ = false;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    // Set once the stop command is sent, so the controller stays silent while idle
    // (the Teleop panel also publishes to /cmd_vel)
    bool stop_sent_ = false;
};

#endif
