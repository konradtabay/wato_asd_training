#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

void PlannerCore::configure(
  int occupied_threshold,
  double cost_weight,
  double goal_tolerance,
  double base_offset,
  double replan_progress_threshold,
  double replan_timeout_sec,
  const std::string& map_frame)
{
  occupied_threshold_ = occupied_threshold;
  cost_weight_ = cost_weight;
  goal_tolerance_ = goal_tolerance;
  base_offset_ = base_offset;
  replan_progress_threshold_ = replan_progress_threshold;
  replan_timeout_sec_ = replan_timeout_sec;
  map_frame_ = map_frame;
}

void PlannerCore::updateMap(const nav_msgs::msg::OccupancyGrid& map)
{
  map_ = map;
  has_map_ = true;
}

void PlannerCore::updateGoal(const geometry_msgs::msg::PointStamped& goal)
{
  goal_x_ = goal.point.x;
  goal_y_ = goal.point.y;
  has_goal_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
}

// Odometry is the lidar at the front of the chassis; plan for the chassis centre
// so the obstacle clearance applies to the whole body.
void PlannerCore::updateOdometry(const nav_msgs::msg::Odometry& odom)
{
  const auto& q = odom.pose.pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  robot_x_ = odom.pose.pose.position.x - base_offset_ * std::cos(yaw);
  robot_y_ = odom.pose.pose.position.y - base_offset_ * std::sin(yaw);
}

bool PlannerCore::isWaitingForGoal() const
{
  return state_ == State::WAITING_FOR_GOAL;
}

bool PlannerCore::isWaitingForRobotToReachGoal() const
{
  return state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
}

void PlannerCore::setWaitingForGoal()
{
  state_ = State::WAITING_FOR_GOAL;
  has_goal_ = false;
  has_replan_anchor_ = false;
}

void PlannerCore::setWaitingForRobotToReachGoal()
{
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
}

bool PlannerCore::goalReached() const
{
  if (!has_goal_) {
    return false;
  }
  const double dx = goal_x_ - robot_x_;
  const double dy = goal_y_ - robot_y_;
  return std::hypot(dx, dy) < goal_tolerance_;
}

bool PlannerCore::shouldReplan(double now_sec) const
{
  if (!has_replan_anchor_) {
    return true;
  }

  const double progress = std::hypot(
    robot_x_ - last_replan_x_,
    robot_y_ - last_replan_y_);
  if (progress >= replan_progress_threshold_) {
    return false;
  }

  return (now_sec - last_replan_time_sec_) >= replan_timeout_sec_;
}

void PlannerCore::markReplan(double robot_x, double robot_y, double now_sec)
{
  last_replan_x_ = robot_x;
  last_replan_y_ = robot_y;
  last_replan_time_sec_ = now_sec;
  has_replan_anchor_ = true;
}

void PlannerCore::markGoalReached()
{
  setWaitingForGoal();
}

void PlannerCore::updateProgressAnchor(double now_sec)
{
  if (!has_replan_anchor_) {
    return;
  }

  const double progress = std::hypot(
    robot_x_ - last_replan_x_,
    robot_y_ - last_replan_y_);
  if (progress >= replan_progress_threshold_) {
    markReplan(robot_x_, robot_y_, now_sec);
  }
}

bool PlannerCore::worldToGrid(double wx, double wy, int& cx, int& cy) const
{
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;
  const double resolution = map_.info.resolution;
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);

  cx = static_cast<int>(std::floor((wx - origin_x) / resolution));
  cy = static_cast<int>(std::floor((wy - origin_y) / resolution));
  return cx >= 0 && cx < width && cy >= 0 && cy < height;
}

void PlannerCore::gridToWorld(int cx, int cy, double& wx, double& wy) const
{
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;
  const double resolution = map_.info.resolution;
  wx = origin_x + (static_cast<double>(cx) + 0.5) * resolution;
  wy = origin_y + (static_cast<double>(cy) + 0.5) * resolution;
}

bool PlannerCore::isOccupied(int cx, int cy) const
{
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
    return true;
  }

  const int8_t value = map_.data[static_cast<size_t>(cy * width + cx)];
  if (value < 0) {
    return false;
  }
  return value >= occupied_threshold_;
}

// Extra step cost for inflated cells so paths stay centred between obstacles
// instead of skimming the occupied threshold.
double PlannerCore::cellPenalty(int cx, int cy) const
{
  const int width = static_cast<int>(map_.info.width);
  const int8_t value = map_.data[static_cast<size_t>(cy * width + cx)];
  if (value <= 0) {
    return 1.0;
  }
  return 1.0 + cost_weight_ * static_cast<double>(value) / 100.0;
}

// Closest free cell (Euclidean) within max_distance; ties go to the cell nearest the robot.
bool PlannerCore::findNearestFree(
  int start_x, int start_y, double max_distance, int& out_x, int& out_y) const
{
  out_x = start_x;
  out_y = start_y;
  if (!isOccupied(start_x, start_y)) {
    return true;
  }

  int robot_x = start_x;
  int robot_y = start_y;
  worldToGrid(robot_x_, robot_y_, robot_x, robot_y);

  const int max_radius = static_cast<int>(std::ceil(max_distance / map_.info.resolution));
  double best_dist = std::numeric_limits<double>::max();
  double best_robot_dist = std::numeric_limits<double>::max();
  bool found = false;

  for (int dy = -max_radius; dy <= max_radius; ++dy) {
    for (int dx = -max_radius; dx <= max_radius; ++dx) {
      const double dist = std::hypot(dx, dy);
      if (dist > max_radius || dist > best_dist + 1e-9) {
        continue;
      }

      const int nx = start_x + dx;
      const int ny = start_y + dy;
      if (isOccupied(nx, ny)) {
        continue;
      }

      const double robot_dist = std::hypot(nx - robot_x, ny - robot_y);
      if (dist < best_dist - 1e-9 || robot_dist < best_robot_dist) {
        best_dist = dist;
        best_robot_dist = robot_dist;
        out_x = nx;
        out_y = ny;
        found = true;
      }
    }
  }
  return found;
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const
{
  return std::hypot(
    static_cast<double>(a.x - b.x),
    static_cast<double>(a.y - b.y));
}

nav_msgs::msg::Path PlannerCore::runAStar(
  int start_x, int start_y, int goal_x, int goal_y) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame_;

  const CellIndex start(start_x, start_y);
  const CellIndex goal(goal_x, goal_y);

  if (isOccupied(goal_x, goal_y)) {
    RCLCPP_WARN(logger_, "Goal cell is occupied");
    return path;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_set<CellIndex, CellIndexHash> closed;

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start, goal));

  static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static const double step_cost[8] = {
    1.0, 1.0, 1.0, 1.0,
    std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)};

  while (!open.empty()) {
    const CellIndex current = open.top().index;
    open.pop();

    if (closed.count(current) > 0) {
      continue;
    }
    closed.insert(current);

    if (current == goal) {
      std::vector<CellIndex> cells;
      CellIndex trace = current;
      cells.push_back(trace);
      while (came_from.count(trace) > 0) {
        trace = came_from.at(trace);
        cells.push_back(trace);
      }
      std::reverse(cells.begin(), cells.end());

      for (const CellIndex& cell : cells) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = map_frame_;
        gridToWorld(cell.x, cell.y, pose.pose.position.x, pose.pose.position.y);
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
      return path;
    }

    for (int i = 0; i < 8; ++i) {
      const int nx = current.x + dx[i];
      const int ny = current.y + dy[i];
      if (isOccupied(nx, ny)) {
        continue;
      }

      // No diagonal squeezing between two blocked cells: the robot cannot cut a corner.
      const bool diagonal = dx[i] != 0 && dy[i] != 0;
      if (diagonal && (isOccupied(current.x + dx[i], current.y) ||
        isOccupied(current.x, current.y + dy[i])))
      {
        continue;
      }

      const CellIndex neighbor(nx, ny);
      if (closed.count(neighbor) > 0) {
        continue;
      }

      const double tentative_g = g_score[current] + step_cost[i] * cellPenalty(nx, ny);
      if (g_score.count(neighbor) == 0 || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        open.emplace(neighbor, tentative_g + heuristic(neighbor, goal));
      }
    }
  }

  RCLCPP_WARN(logger_, "A* failed to find a path");
  return path;
}

nav_msgs::msg::Path PlannerCore::planPath()
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = map_frame_;

  if (!has_map_ || !has_goal_ || map_.data.empty()) {
    RCLCPP_WARN(logger_, "Cannot plan path: missing map or goal");
    return empty_path;
  }

  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;

  if (!worldToGrid(robot_x_, robot_y_, start_x, start_y)) {
    RCLCPP_WARN(logger_, "Robot position is outside the map");
    return empty_path;
  }

  if (!worldToGrid(goal_x_, goal_y_, goal_x, goal_y)) {
    RCLCPP_WARN(logger_, "Goal position is outside the map");
    return empty_path;
  }

  int traversable_x = start_x;
  int traversable_y = start_y;
  if (!findNearestFree(start_x, start_y, 2.0, traversable_x, traversable_y)) {
    RCLCPP_WARN(logger_, "Robot is inside inflated cells; planning from the robot cell");
  }

  // A goal clicked just inside the inflation zone moves to the nearest reachable cell,
  // and the goal itself is updated so goalReached() agrees with where the path ends.
  int free_goal_x = goal_x;
  int free_goal_y = goal_y;
  if (findNearestFree(goal_x, goal_y, 1.0, free_goal_x, free_goal_y) &&
    (free_goal_x != goal_x || free_goal_y != goal_y))
  {
    gridToWorld(free_goal_x, free_goal_y, goal_x_, goal_y_);
    RCLCPP_INFO(logger_, "Goal too close to an obstacle; moved to (%.2f, %.2f)", goal_x_, goal_y_);
    goal_x = free_goal_x;
    goal_y = free_goal_y;
  }

  return runAStar(traversable_x, traversable_y, goal_x, goal_y);
}

}  // namespace robot
