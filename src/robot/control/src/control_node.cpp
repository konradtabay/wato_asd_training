#include "control_node.hpp"

#include <chrono>
#include <memory>

ControlNode::ControlNode()
: Node("control_node"), control_(robot::ControlCore(this->get_logger()))
{
  this->declare_parameter("lookahead_distance", 1.0);
  this->declare_parameter("goal_tolerance", 0.1);
  this->declare_parameter("linear_speed", 0.5);
  this->declare_parameter("max_angular_speed", 1.2);
  this->declare_parameter("rotate_threshold", 0.8);

  control_.configure(
    this->get_parameter("lookahead_distance").as_double(),
    this->get_parameter("goal_tolerance").as_double(),
    this->get_parameter("linear_speed").as_double(),
    this->get_parameter("max_angular_speed").as_double(),
    this->get_parameter("rotate_threshold").as_double());

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  control_.updatePath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  control_.updateOdometry(*msg);
}

void ControlNode::controlLoop()
{
  cmd_vel_pub_->publish(control_.computeVelocity());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
