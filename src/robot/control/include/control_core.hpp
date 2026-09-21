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
    explicit ControlCore(const rclcpp::Logger& logger);

    void configure(
      double lookahead_distance,
      double goal_tolerance,
      double linear_speed,
      double max_angular_speed,
      double rotate_threshold);

    void updatePath(const nav_msgs::msg::Path& path);
    void updateOdometry(const nav_msgs::msg::Odometry& odom);
    geometry_msgs::msg::Twist computeVelocity() const;

  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::Path path_;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    bool has_path_ = false;
    bool has_odometry_ = false;

    double lookahead_distance_ = 1.0;
    double goal_tolerance_ = 0.1;
    double linear_speed_ = 0.5;
    double max_angular_speed_ = 1.2;
    double rotate_threshold_ = 0.8;

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;
    geometry_msgs::msg::Twist computeVelocity(
      const geometry_msgs::msg::PoseStamped& target) const;
    double computeDistance(
      const geometry_msgs::msg::Point& a,
      const geometry_msgs::msg::Point& b) const;
    double extractYaw(const geometry_msgs::msg::Quaternion& quat) const;
    static double normalizeAngle(double angle);
    double clampAngular(double angular) const;
};

}  // namespace robot

#endif
