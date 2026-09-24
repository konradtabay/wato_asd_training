#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap_node"), costmap_(robot::CostmapCore(this->get_logger()))
{
  this->declare_parameter("resolution", 0.1);
  this->declare_parameter("width", 200);
  this->declare_parameter("height", 200);
  this->declare_parameter("origin_x", -10.0);
  this->declare_parameter("origin_y", -10.0);
  this->declare_parameter("inflation_radius", 2.2);
  this->declare_parameter("max_cost", 100);
  this->declare_parameter("min_obstacle_range", 0.4);

  costmap_.configure(
    this->get_parameter("resolution").as_double(),
    this->get_parameter("width").as_int(),
    this->get_parameter("height").as_int(),
    this->get_parameter("origin_x").as_double(),
    this->get_parameter("origin_y").as_double(),
    this->get_parameter("inflation_radius").as_double(),
    this->get_parameter("max_cost").as_int(),
    this->get_parameter("min_obstacle_range").as_double());

  costmap_pub_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  const auto costmap = costmap_.updateFromScan(*scan);
  costmap_pub_->publish(costmap);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
