#include <chrono>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
: Node("planner_node"), planner_(robot::PlannerCore(this->get_logger()))
{
  this->declare_parameter("occupied_threshold", 50); // Cell is considered occupied if its value is >= 50
  this->declare_parameter("goal_tolerance", 0.5); // Goal considered reached within 0.5m
  this->declare_parameter("replan_progress_threshold", 0.2); // Progress threshold for 500ms timer increments
  this->declare_parameter("replan_timeout_sec", 3.0); // Timeout for replanning
  this->declare_parameter("map_frame", "sim_world"); // Frame ID for the map

  planner_.configure(
    this->get_parameter("occupied_threshold").as_int(),
    this->get_parameter("goal_tolerance").as_double(),
    this->get_parameter("replan_progress_threshold").as_double(),
    this->get_parameter("replan_timeout_sec").as_double(),
    this->get_parameter("map_frame").as_string());

  
  //Creating publishers and subscribers declared in .hpp file
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", map_qos,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  planner_.updateMap(*msg);
  if (planner_.isWaitingForRobotToReachGoal()) { 
    publishPath(); // Update the path if a new map arrives while travelling
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  planner_.updateGoal(*msg);
  publishPath(); // Plan a path when a goal is received
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  planner_.updateOdometry(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y); // Update robot position
}

void PlannerNode::timerCallback()
{
  if (!planner_.isWaitingForRobotToReachGoal()) { // Do nothing if not travelling to goal
    return;
  }

  if (planner_.goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    planner_.markGoalReached(); 
    return;
  }

  const double now_sec = this->now().seconds();
  planner_.updateProgressAnchor(now_sec);
  if (planner_.shouldReplan(now_sec)) { //Check if new path is needed due to timeout or lack of progress
    RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or lack of progress");
    publishPath();
  }
}

void PlannerNode::publishPath()
{
  if (!planner_.hasMap() || !planner_.hasGoal()) {
    return; // Cannot do anything if map or goal is missing
  }

  auto path = planner_.planPath();
  path.header.stamp = this->now();
  for (auto& pose : path.poses) {
    pose.header.stamp = path.header.stamp;
  }
  path_pub_->publish(path); // After pathing, publish the path
  planner_.markReplan(
    planner_.robotX(),
    planner_.robotY(),
    this->now().seconds());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
