#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
: logger_(logger) {}

void ControlCore::configure(
  double lookahead_distance,
  double goal_tolerance,
  double linear_speed,
  double max_angular_speed,
  double rotate_threshold,
  double base_offset,
  double min_speed,
  double slowdown_distance)
{
  lookahead_distance_ = lookahead_distance;
  goal_tolerance_ = goal_tolerance;
  linear_speed_ = linear_speed;
  min_speed_ = min_speed;
  slowdown_distance_ = slowdown_distance;
  max_angular_speed_ = max_angular_speed;
  rotate_threshold_ = rotate_threshold;
  base_offset_ = base_offset;
}

void ControlCore::updatePath(const nav_msgs::msg::Path& path)
{
  path_ = path;
  has_path_ = !path_.poses.empty();
}

// Odometry is the lidar at the front of the chassis; track the chassis centre so the
// body follows the planned path instead of the rear cutting inside corners.
void ControlCore::updateOdometry(const nav_msgs::msg::Odometry& odom)
{
  robot_yaw_ = extractYaw(odom.pose.pose.orientation);
  robot_x_ = odom.pose.pose.position.x - base_offset_ * std::cos(robot_yaw_);
  robot_y_ = odom.pose.pose.position.y - base_offset_ * std::sin(robot_yaw_);
  has_odometry_ = true;
}

double ControlCore::computeDistance(
  const geometry_msgs::msg::Point& a,
  const geometry_msgs::msg::Point& b) const
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) const
{
  return std::atan2(
    2.0 * (quat.w * quat.z + quat.x * quat.y),
    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

double ControlCore::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double ControlCore::clampAngular(double angular) const
{
  return std::clamp(angular, -max_angular_speed_, max_angular_speed_);
}

// First path point at least `distance` away, searching forward from the closest point.
std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findPointAhead(double distance) const
{
  if (!has_path_ || !has_odometry_ || path_.poses.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point robot;
  robot.x = robot_x_;
  robot.y = robot_y_;

  const auto& goal = path_.poses.back().pose.position;
  if (computeDistance(robot, goal) < goal_tolerance_) {
    return path_.poses.back();
  }

  size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path_.poses.size(); ++i) {
    const double distance = computeDistance(robot, path_.poses[i].pose.position);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }

  for (size_t i = closest_index; i < path_.poses.size(); ++i) {
    if (computeDistance(robot, path_.poses[i].pose.position) >= distance) {
      return path_.poses[i];
    }
  }

  return path_.poses.back();
}

double ControlCore::headingErrorTo(const geometry_msgs::msg::Point& point) const
{
  const double target_angle = std::atan2(point.y - robot_y_, point.x - robot_x_);
  return normalizeAngle(target_angle - robot_yaw_);
}

// Full speed only when the path is straight for the next slowdown_distance and the
// goal is still far away; otherwise scale down towards min_speed.
double ControlCore::computeSpeed(double angle_error) const
{
  geometry_msgs::msg::Point robot;
  robot.x = robot_x_;
  robot.y = robot_y_;

  double preview_error = 0.0;
  const auto preview = findPointAhead(slowdown_distance_);
  if (preview) {
    preview_error = headingErrorTo(preview->pose.position);
  }

  const double turn = std::max(std::abs(angle_error), std::abs(preview_error));
  const double turn_scale = std::clamp(1.0 - turn / rotate_threshold_, 0.0, 1.0);

  const double goal_distance = computeDistance(robot, path_.poses.back().pose.position);
  const double goal_scale = std::clamp(goal_distance / slowdown_distance_, 0.0, 1.0);

  return min_speed_ + (linear_speed_ - min_speed_) * std::min(turn_scale, goal_scale);
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const geometry_msgs::msg::PoseStamped& target) const
{
  geometry_msgs::msg::Twist cmd_vel;

  geometry_msgs::msg::Point robot;
  robot.x = robot_x_;
  robot.y = robot_y_;

  const double distance = computeDistance(robot, target.pose.position);
  if (distance < goal_tolerance_) {
    return cmd_vel;
  }

  const double angle_error = headingErrorTo(target.pose.position);

  if (std::abs(angle_error) > rotate_threshold_) {
    cmd_vel.angular.z = clampAngular(1.5 * angle_error);
    return cmd_vel;
  }

  const double speed = computeSpeed(angle_error);
  const double lookahead = std::max(distance, 1e-3);
  const double curvature = (2.0 * std::sin(angle_error)) / lookahead;
  cmd_vel.linear.x = speed;
  cmd_vel.angular.z = clampAngular(speed * curvature);
  return cmd_vel;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity() const
{
  geometry_msgs::msg::Twist cmd_vel;
  if (!has_path_ || !has_odometry_) {
    return cmd_vel;
  }

  const auto lookahead_point = findPointAhead(lookahead_distance_);
  if (!lookahead_point) {
    return cmd_vel;
  }

  return computeVelocity(*lookahead_point);
}

}  // namespace robot
