#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    // Updating information
    void updatePath(const nav_msgs::msg::Path& path);
    void updateOdometry(const nav_msgs::msg::Odometry& odom);

    // Compute the velocity command based on the current path and odometry
    geometry_msgs::msg::Twist computeVelocity();
  
  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::Path path_; // The current path for the robot to follow
    nav_msgs::msg::Odometry odom_; // The current odometry of the robot

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    bool has_path_ = false;
    bool has_odometry_ = false;

    double lookahead_distance_ = 1.0;
    double goal_tolerance_ = 0.1;
    double linear_speed_ = 0.5;

    // Find the lookahead point on the path.
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const; 

    // Compute the velocity command to move towards the target point.
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target) const;

    // Compute the distance between two points.
    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const;

    // Extract yaw from quaternion (get the robot's rotation around the vertical axis).
    double extractYaw(const geometry_msgs::msg::Quaternion& quat) const; 
};

} 

#endif 
