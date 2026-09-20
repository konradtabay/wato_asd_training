#include "control_core.hpp"

#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

  // Update functions for path and odometry
  void ControlCore::updatePath(const nav_msgs::msg::Path& path)
  {
    path_ = path;
    has_path_ = !path_.poses.empty();
  }

  void ControlCore::updateOdometry(const nav_msgs::msg::Odometry& odom)
  {
    robot_x_ = odom.pose.pose.position.x;
    robot_y_ = odom.pose.pose.position.y;
    robot_yaw_ = extractYaw(odom.pose.pose.orientation);

    has_odometry_ = true;
  }

  // Compute the velocity command based on the current path and odometry
  double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;

    return std::hypot(dx, dy);
  }

  // Extract yaw from quaternion
  double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) const
  {
    return std::atan2(
      2.0 * (quat.w * quat.z + quat.x * quat.y),
      1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
  }

  std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() const
  {
    if (!has_path_ || !has_odometry_ || path_.poses.empty()) {
      return std::nullopt; // Return empty if there is no path or odometry data.
    }

    geometry_msgs::msg::Point robot_position;
    robot_position.x = robot_x_;
    robot_position.y = robot_y_;
    robot_position.z = 0.0;

    // Check whether the goal is already within the tolerance
    const auto& goal = path_.poses.back().pose.position;

    if (computeDistance(robot_position, goal) < goal_tolerance_) {
      return path_.poses.back();
    }

    // Find the first point at least the lookahead distance away
    for (const auto& pose : path_.poses) {
      const double distance = computeDistance(robot_position, pose.pose.position);

      if (distance >= lookahead_distance_) {
        return pose;
      }
    }

    // If no point is far enough away, use the final point
    return path_.poses.back();
  }

  // Compute the velocity command based on the current path and odometry
  geometry_msgs::msg::Twist ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped& target) const
  {
    geometry_msgs::msg::Twist cmd_vel;

    geometry_msgs::msg::Point robot_position;
    robot_position.x = robot_x_;
    robot_position.y = robot_y_;
    robot_position.z = 0.0;

    // Calculate the distance to the target point
    const double distance = computeDistance(robot_position, target.pose.position);

    // Stop when we reach the goal.
    if (distance < goal_tolerance_) {
      return cmd_vel;
    }

    const double dx = target.pose.position.x - robot_x_;
    const double dy = target.pose.position.y - robot_y_;

    // Angle from robot to lookahead point
    const double target_angle = std::atan2(dy, dx);

    // Difference between where robot is facing and where it should face
    double angle_error = target_angle - robot_yaw_;

    // Normalize angle to [-pi, pi]
    while (angle_error > M_PI) {
      angle_error -= 2.0 * M_PI;
    }

    while (angle_error < -M_PI) {
      angle_error += 2.0 * M_PI;
    }

    // Pure Pursuit curvature.
    const double curvature = (2.0 * std::sin(alpha)) / distance;

    // Set forward and turning velocities
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = linear_speed_ * curvature;

    return cmd_vel;
  }

  // Coordinate change in velocity
  geometry_msgs::msg::Twist ControlCore::computeVelocity()
  {
    geometry_msgs::msg::Twist cmd_vel;

    if (!has_path_ || !has_odometry_) {
      return cmd_vel; // Return zero velocity if there is no path or odometry data
    }

    auto lookahead_point = findLookaheadPoint();

    if (!lookahead_point) {
      return cmd_vel;
    }

    return computeVelocity(*lookahead_point);
  }

}  