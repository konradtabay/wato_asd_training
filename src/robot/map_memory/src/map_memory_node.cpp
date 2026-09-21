#include <chrono>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory_node"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  this->declare_parameter("resolution", 0.2);
  this->declare_parameter("width", 150);
  this->declare_parameter("height", 150);
  this->declare_parameter("origin_x", -15.0);
  this->declare_parameter("origin_y", -15.0);
  this->declare_parameter("distance_threshold", 1.5);
  this->declare_parameter("map_frame", "sim_world");

  map_memory_.configure(
    this->get_parameter("resolution").as_double(),
    this->get_parameter("width").as_int(),
    this->get_parameter("height").as_int(),
    this->get_parameter("origin_x").as_double(),
    this->get_parameter("origin_y").as_double(),
    this->get_parameter("distance_threshold").as_double(),
    this->get_parameter("map_frame").as_string());

  const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::updateMapTimer, this));

  auto initial_map = map_memory_.createInitialMap();
  initial_map.header.stamp = this->now();
  map_pub_->publish(initial_map);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_memory_.updateCostmap(*msg);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  map_memory_.updateOdometry(*msg);
}

void MapMemoryNode::updateMapTimer()
{
  map_memory_.tryMerge();

  auto map = map_memory_.getGlobalMap();
  map.header.stamp = this->now();
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
