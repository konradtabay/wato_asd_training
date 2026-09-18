#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger)
{
  global_map_.assign(static_cast<size_t>(width_ * height_), -1);
}

void MapMemoryCore::configure(
  double resolution,
  int width,
  int height,
  double origin_x,
  double origin_y,
  double distance_threshold,
  const std::string& map_frame)
{
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  distance_threshold_ = distance_threshold;
  map_frame_ = map_frame;
  global_map_.assign(static_cast<size_t>(width_ * height_), -1);
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::createInitialMap() const
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = map_frame_;
  map.info.resolution = resolution_;
  map.info.width = static_cast<uint32_t>(width_);
  map.info.height = static_cast<uint32_t>(height_);
  map.info.origin.position.x = origin_x_;
  map.info.origin.position.y = origin_y_;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.data = global_map_;
  return map;
}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap)
{
  latest_costmap_ = costmap;
  has_costmap_ = true;
  costmap_updated_ = true;
}

void MapMemoryCore::updateOdometry(const nav_msgs::msg::Odometry& odom)
{
  robot_x_ = odom.pose.pose.position.x;
  robot_y_ = odom.pose.pose.position.y;
  robot_yaw_ = yawFromQuaternion(
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w);
  has_robot_pose_ = true;

  if (!has_last_merge_pose_) {
    should_update_map_ = true;
    return;
  }

  const double dx = robot_x_ - last_merge_x_;
  const double dy = robot_y_ - last_merge_y_;
  if (std::hypot(dx, dy) >= distance_threshold_) {
    should_update_map_ = true;
  }
}

bool MapMemoryCore::tryMerge()
{
  if (!should_update_map_ || !costmap_updated_ || !has_costmap_ || !has_robot_pose_) {
    return false;
  }

  mergeLatestCostmap();
  last_merge_x_ = robot_x_;
  last_merge_y_ = robot_y_;
  has_last_merge_pose_ = true;
  should_update_map_ = false;
  costmap_updated_ = false;
  return true;
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = map_frame_;
  map.info.resolution = resolution_;
  map.info.width = static_cast<uint32_t>(width_);
  map.info.height = static_cast<uint32_t>(height_);
  map.info.origin.position.x = origin_x_;
  map.info.origin.position.y = origin_y_;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.data = global_map_;
  return map;
}

double MapMemoryCore::yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

bool MapMemoryCore::worldToGrid(double wx, double wy, int& gx, int& gy) const
{
  gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

void MapMemoryCore::mergeLatestCostmap()
{
  const double local_resolution = latest_costmap_.info.resolution;
  const int local_width = static_cast<int>(latest_costmap_.info.width);
  const int local_height = static_cast<int>(latest_costmap_.info.height);
  const double local_origin_x = latest_costmap_.info.origin.position.x;
  const double local_origin_y = latest_costmap_.info.origin.position.y;

  const double cos_yaw = std::cos(robot_yaw_);
  const double sin_yaw = std::sin(robot_yaw_);

  for (int ly = 0; ly < local_height; ++ly) {
    for (int lx = 0; lx < local_width; ++lx) {
      const int8_t local_value =
        latest_costmap_.data[static_cast<size_t>(ly * local_width + lx)];
      if (local_value < 0) {
        continue;
      }

      const double x_local =
        local_origin_x + (static_cast<double>(lx) + 0.5) * local_resolution;
      const double y_local =
        local_origin_y + (static_cast<double>(ly) + 0.5) * local_resolution;

      const double wx = robot_x_ + x_local * cos_yaw - y_local * sin_yaw;
      const double wy = robot_y_ + x_local * sin_yaw + y_local * cos_yaw;

      int gx = 0;
      int gy = 0;
      if (worldToGrid(wx, wy, gx, gy)) {
        global_map_[static_cast<size_t>(gy * width_ + gx)] = local_value;
      }
    }
  }
}

}  // namespace robot
