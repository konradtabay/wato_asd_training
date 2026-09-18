#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void configure(
      double resolution,
      int width,
      int height,
      double origin_x,
      double origin_y,
      double inflation_radius,
      int max_cost);

    nav_msgs::msg::OccupancyGrid updateFromScan(const sensor_msgs::msg::LaserScan& scan);

  private:
    rclcpp::Logger logger_;

    double resolution_ = 0.1;
    int width_ = 200;
    int height_ = 200;
    double origin_x_ = -10.0;
    double origin_y_ = -10.0;
    double inflation_radius_ = 1.0;
    int max_cost_ = 100;

    std::vector<int8_t> grid_;

    void initializeCostmap();
    bool convertToGrid(double x, double y, int& x_cell, int& y_cell) const;
    void markObstacle(int x_cell, int y_cell);
    void inflateObstacles();
    nav_msgs::msg::OccupancyGrid buildOccupancyGrid(
      const std_msgs::msg::Header& header) const;
};

}  // namespace robot

#endif
