#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "control_core.hpp"
#include "rclcpp/rclcpp.hpp"


class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    // Declaring callback functions for incoming/outgoing messages (or timer events)
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void publishCommand();

    // Declaring an instance of the ControlCore class
    robot::ControlCore control_;

    // Declaring subscribers for path and odometry topics
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
