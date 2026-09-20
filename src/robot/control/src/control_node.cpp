#include "control_node.hpp"

#include <chrono>
#include <memory>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {

  //Creating publishers and subscribers declared in .hpp file
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, 
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::timerCallback, this));

}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  control_.updatePath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  control_.updateOdometry(*msg);
}

void ControlNode::timerCallback()
{
  publishCommand();
}

void ControlNode::publishCommand()
{
  auto cmd_vel = control_.computeVelocity();
  cmd_vel_pub_->publish(cmd_vel); // Gets a velocity command from the controller and publishes it
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
