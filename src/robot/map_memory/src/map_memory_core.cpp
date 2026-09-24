#include <algorithm>
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
  recent_costmaps_.push_back(costmap);
  while (recent_costmaps_.size() > 3) {
    recent_costmaps_.pop_front();
  }
  costmap_updated_ = true;
}

void MapMemoryCore::updateOdometry(const nav_msgs::msg::Odometry& odom)
{
  robot_x_ = odom.pose.pose.position.x;
  robot_y_ = odom.pose.pose.position.y;
  const double yaw = yawFromQuaternion(
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w);

  const double t = rclcpp::Time(odom.header.stamp).seconds();
  pose_history_.push_back({t, robot_x_, robot_y_, yaw});
  while (pose_history_.front().t < t - 2.0) {
    pose_history_.pop_front();
  }

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

// Merge the newest costmap whose scan time is covered by the odometry history, so the
// scan is placed with the pose the robot had when it was taken.
bool MapMemoryCore::tryMerge()
{
  if (!should_update_map_ || !costmap_updated_) {
    return false;
  }

  for (auto it = recent_costmaps_.rbegin(); it != recent_costmaps_.rend(); ++it) {
    StampedPose pose{};
    if (!poseAt(rclcpp::Time(it->header.stamp).seconds(), pose)) {
      continue;
    }

    mergeCostmap(*it, pose);
    last_merge_x_ = pose.x;
    last_merge_y_ = pose.y;
    has_last_merge_pose_ = true;
    should_update_map_ = false;
    costmap_updated_ = false;
    return true;
  }
  return false;
}

// Interpolated pose at time t. Fails if t is outside the history or the robot jumped
// (teleport or glitch) between the bracketing samples.
bool MapMemoryCore::poseAt(double t, StampedPose& pose) const
{
  if (pose_history_.empty() || t < pose_history_.front().t || t > pose_history_.back().t) {
    return false;
  }
  if (pose_history_.size() == 1) {
    pose = pose_history_.front();
    return true;
  }

  for (size_t i = 1; i < pose_history_.size(); ++i) {
    const StampedPose& b = pose_history_[i];
    if (b.t < t) {
      continue;
    }
    const StampedPose& a = pose_history_[i - 1];
    if (std::hypot(b.x - a.x, b.y - a.y) > 1.0) {
      return false;
    }

    const double span = b.t - a.t;
    const double s = span > 0.0 ? (t - a.t) / span : 0.0;
    const double dyaw = std::atan2(std::sin(b.yaw - a.yaw), std::cos(b.yaw - a.yaw));
    pose = {t, a.x + s * (b.x - a.x), a.y + s * (b.y - a.y), a.yaw + s * dyaw};
    return true;
  }
  return false;
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

void MapMemoryCore::mergeCostmap(
  const nav_msgs::msg::OccupancyGrid& costmap, const StampedPose& pose)
{
  const double local_resolution = costmap.info.resolution;
  const int local_width = static_cast<int>(costmap.info.width);
  const int local_height = static_cast<int>(costmap.info.height);
  const double local_origin_x = costmap.info.origin.position.x;
  const double local_origin_y = costmap.info.origin.position.y;

  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);

  // Several local cells land in one global cell; keep the highest cost so free cells
  // never erase an obstacle observed in the same scan.
  std::vector<int8_t> update(global_map_.size(), -1);

  for (int ly = 0; ly < local_height; ++ly) {
    for (int lx = 0; lx < local_width; ++lx) {
      const int8_t local_value =
        costmap.data[static_cast<size_t>(ly * local_width + lx)];
      if (local_value < 0) {
        continue;
      }

      const double x_local =
        local_origin_x + (static_cast<double>(lx) + 0.5) * local_resolution;
      const double y_local =
        local_origin_y + (static_cast<double>(ly) + 0.5) * local_resolution;

      const double wx = pose.x + x_local * cos_yaw - y_local * sin_yaw;
      const double wy = pose.y + x_local * sin_yaw + y_local * cos_yaw;

      int gx = 0;
      int gy = 0;
      if (worldToGrid(wx, wy, gx, gy)) {
        int8_t& cell = update[static_cast<size_t>(gy * width_ + gx)];
        cell = std::max(cell, local_value);
      }
    }
  }

  for (size_t i = 0; i < update.size(); ++i) {
    if (update[i] >= 0) {
      global_map_[i] = update[i];
    }
  }
}

}  // namespace robot
