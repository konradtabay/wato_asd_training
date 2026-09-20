#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_core.hpp"
#include "rclcpp/rclcpp.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    // Declaring callback functions for incoming/outgoing messages (or timer events)
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void publishPath();

    // Declaring an instance of the PlannerCore class
    robot::PlannerCore planner_;

    // Declaring subscribers for map, goal, and odometry topics
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Declaring publisher for the path topic
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Declaring timer for periodic path publishing
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
