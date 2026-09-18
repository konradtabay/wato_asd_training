#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const
  {
    return x == other.x && y == other.y;
  }

  bool operator!=(const CellIndex& other) const
  {
    return !(*this == other);
  }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) const
  {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void configure(
      int occupied_threshold,
      double goal_tolerance,
      double replan_progress_threshold,
      double replan_timeout_sec,
      const std::string& map_frame);

    void updateMap(const nav_msgs::msg::OccupancyGrid& map);
    void updateGoal(const geometry_msgs::msg::PointStamped& goal);
    void updateOdometry(double x, double y);

    nav_msgs::msg::Path planPath();
    bool goalReached() const;
    bool shouldReplan(double now_sec) const;
    void markReplan(double robot_x, double robot_y, double now_sec);
    void markGoalReached();
    void updateProgressAnchor(double now_sec);

    bool hasMap() const { return has_map_; }
    bool hasGoal() const { return has_goal_; }
    double robotX() const { return robot_x_; }
    double robotY() const { return robot_y_; }
    bool isWaitingForGoal() const;
    bool isWaitingForRobotToReachGoal() const;
    void setWaitingForGoal();
    void setWaitingForRobotToReachGoal();

  private:
    rclcpp::Logger logger_;

    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_ = State::WAITING_FOR_GOAL;

    nav_msgs::msg::OccupancyGrid map_;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    bool has_map_ = false;
    bool has_goal_ = false;

    int occupied_threshold_ = 50;
    double goal_tolerance_ = 0.5;
    double replan_progress_threshold_ = 0.2;
    double replan_timeout_sec_ = 3.0;
    std::string map_frame_ = "sim_world";

    double last_replan_x_ = 0.0;
    double last_replan_y_ = 0.0;
    double last_replan_time_sec_ = 0.0;
    bool has_replan_anchor_ = false;

    bool worldToGrid(double wx, double wy, int& cx, int& cy) const;
    void gridToWorld(int cx, int cy, double& wx, double& wy) const;
    bool isOccupied(int cx, int cy) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;
    nav_msgs::msg::Path runAStar(int start_x, int start_y, int goal_x, int goal_y) const;
};

}  // namespace robot

#endif
