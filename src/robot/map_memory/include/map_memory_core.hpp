#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void configure(
      double resolution,
      int width,
      int height,
      double origin_x,
      double origin_y,
      double distance_threshold,
      const std::string& map_frame);

    nav_msgs::msg::OccupancyGrid createInitialMap() const;

    void updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap);
    void updateOdometry(const nav_msgs::msg::Odometry& odom);
    bool tryMerge();
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;

  private:
    rclcpp::Logger logger_;

    double resolution_ = 0.2;
    int width_ = 150;
    int height_ = 150;
    double origin_x_ = -15.0;
    double origin_y_ = -15.0;
    double distance_threshold_ = 0.5;
    std::string map_frame_ = "sim_world";

    struct StampedPose {
      double t;
      double x;
      double y;
      double yaw;
    };

    std::vector<int8_t> global_map_;

    std::deque<nav_msgs::msg::OccupancyGrid> recent_costmaps_;
    std::deque<StampedPose> pose_history_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    double last_merge_x_ = 0.0;
    double last_merge_y_ = 0.0;
    bool has_last_merge_pose_ = false;

    bool should_update_map_ = false;
    bool costmap_updated_ = false;

    static double yawFromQuaternion(
      double x, double y, double z, double w);
    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
    bool poseAt(double t, StampedPose& pose) const;
    void mergeCostmap(const nav_msgs::msg::OccupancyGrid& costmap, const StampedPose& pose);
};

}  // namespace robot

#endif
