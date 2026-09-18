#include <algorithm>
#include <cmath>
#include <limits>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger)
{
  grid_.resize(static_cast<size_t>(width_ * height_), -1);
}

void CostmapCore::configure(
  double resolution,
  int width,
  int height,
  double origin_x,
  double origin_y,
  double inflation_radius,
  int max_cost)
{
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  inflation_radius_ = inflation_radius;
  max_cost_ = max_cost;
  grid_.assign(static_cast<size_t>(width_ * height_), -1);
}

void CostmapCore::initializeCostmap()
{
  std::fill(grid_.begin(), grid_.end(), static_cast<int8_t>(-1));
}

bool CostmapCore::convertToGrid(double x, double y, int& x_cell, int& y_cell) const
{
  x_cell = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  y_cell = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return x_cell >= 0 && x_cell < width_ && y_cell >= 0 && y_cell < height_;
}

void CostmapCore::markObstacle(int x_cell, int y_cell)
{
  grid_[static_cast<size_t>(y_cell * width_ + x_cell)] = static_cast<int8_t>(max_cost_);
}

void CostmapCore::inflateObstacles()
{
  const int inflation_cells =
    static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  std::vector<int8_t> inflated = grid_;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_[static_cast<size_t>(y * width_ + x)] != max_cost_) {
        continue;
      }

      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
            continue;
          }

          const double dist =
            std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * resolution_;
          if (dist > inflation_radius_) {
            continue;
          }

          const int cost = static_cast<int>(
            max_cost_ * (1.0 - dist / inflation_radius_));
          const size_t idx = static_cast<size_t>(ny * width_ + nx);
          if (cost > inflated[idx]) {
            inflated[idx] = static_cast<int8_t>(cost);
          }
        }
      }
    }
  }

  grid_ = std::move(inflated);
}

nav_msgs::msg::OccupancyGrid CostmapCore::buildOccupancyGrid(
  const std_msgs::msg::Header& header) const
{
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header = header;
  grid_msg.info.resolution = resolution_;
  grid_msg.info.width = static_cast<uint32_t>(width_);
  grid_msg.info.height = static_cast<uint32_t>(height_);
  grid_msg.info.origin.position.x = origin_x_;
  grid_msg.info.origin.position.y = origin_y_;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;
  grid_msg.data = grid_;
  return grid_msg;
}

nav_msgs::msg::OccupancyGrid CostmapCore::updateFromScan(
  const sensor_msgs::msg::LaserScan& scan)
{
  initializeCostmap();

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) ||
      range <= scan.range_min ||
      range >= scan.range_max)
    {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);

    int x_cell = 0;
    int y_cell = 0;
    if (convertToGrid(x, y, x_cell, y_cell)) {
      markObstacle(x_cell, y_cell);
    }
  }

  inflateObstacles();
  return buildOccupancyGrid(scan.header);
}

}  // namespace robot
