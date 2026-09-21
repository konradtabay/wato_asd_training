#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {} // Constructor

// Store parameters within the class
void PlannerCore::configure(
  int occupied_threshold,
  double goal_tolerance,
  double replan_progress_threshold,
  double replan_timeout_sec,
  const std::string& map_frame)
{
  occupied_threshold_ = occupied_threshold;
  goal_tolerance_ = goal_tolerance;
  replan_progress_threshold_ = replan_progress_threshold;
  replan_timeout_sec_ = replan_timeout_sec;
  map_frame_ = map_frame;
}

// Update the map, goal, and odometry information
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

void PlannerCore::updateOdometry(double x, double y)
{
  robot_x_ = x;
  robot_y_ = y;
}

// State functions (getters/setters)
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
  return std::hypot(dx, dy) < goal_tolerance_; // Check if the robot is within the goal tolerance
}

bool PlannerCore::shouldReplan(double now_sec) const
{
  if (!has_replan_anchor_) { // If there is no previous replan position/time, we replan
    return true; 
  }

  const double progress = std::hypot( // Calculate movement since last replan
    robot_x_ - last_replan_x_, 
    robot_y_ - last_replan_y_);
  if (progress >= replan_progress_threshold_) { // If robot progressed enough, do not replan
    return false;
  }

  return (now_sec - last_replan_time_sec_) >= replan_timeout_sec_; // If robot has not progressed enough and timeout has passed, replan
}

// Retain values for a replan based on the new robot position and time
void PlannerCore::markReplan(double robot_x, double robot_y, double now_sec)
{
  last_replan_x_ = robot_x; // Update the last replan position and time
  last_replan_y_ = robot_y;
  last_replan_time_sec_ = now_sec; // Update the last replan time
  has_replan_anchor_ = true; // Mark that we now have a replan anchor
}

void PlannerCore::markGoalReached() // After reaching the goal, reset to waiting for a new goal
{
  setWaitingForGoal();
}

void PlannerCore::updateProgressAnchor(double now_sec)
{
  if (!has_replan_anchor_) {
    return;
  }

  // Calculate the distance moved since the last replan anchor and update the anchor if the robot has moved enough
  const double progress = std::hypot(
    robot_x_ - last_replan_x_,
    robot_y_ - last_replan_y_);
  if (progress >= replan_progress_threshold_) {
    markReplan(robot_x_, robot_y_, now_sec);
  }
}

// Convert world coordinates to grid coordinates and check if the point is within the map bounds
bool PlannerCore::worldToGrid(double wx, double wy, int& cx, int& cy) const
{
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;
  const double resolution = map_.info.resolution; // Size of each cell in the occupancy grid
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);

  cx = static_cast<int>(std::floor((wx - origin_x) / resolution));
  cy = static_cast<int>(std::floor((wy - origin_y) / resolution));
  return cx >= 0 && cx < width && cy >= 0 && cy < height;
}

// Convert grid coordinates back to world coordinates
void PlannerCore::gridToWorld(int cx, int cy, double& wx, double& wy) const
{
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;
  const double resolution = map_.info.resolution;
  wx = origin_x + (static_cast<double>(cx) + 0.5) * resolution; // Puts positions at the centre of the cell
  wy = origin_y + (static_cast<double>(cy) + 0.5) * resolution;
}

bool PlannerCore::isOccupied(int cx, int cy) const
{
  // Check if the cell is within the map bounds. If not, it is considered occupied.
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
    return true;
  }

  // Gets the occupancy value of the cell.
  const int8_t value = map_.data[static_cast<size_t>(cy * width + cx)];
  if (value < 0) { // Unknown occupancy value is treated as unoccupied.
    return false;
  }
  return value >= occupied_threshold_; // Cell is considered occupied if its value is >= threshold (50).
}

bool PlannerCore::findTraversableStart(int start_x, int start_y, int& out_x, int& out_y) const
{
  if (!isOccupied(start_x, start_y)) {
    out_x = start_x;
    out_y = start_y;
    return true;
  }

  const int max_radius = static_cast<int>(std::ceil(2.0 / map_.info.resolution));
  for (int radius = 1; radius <= max_radius; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius) {
          continue;
        }

        const int nx = start_x + dx;
        const int ny = start_y + dy;
        if (!isOccupied(nx, ny)) {
          out_x = nx;
          out_y = ny;
          return true;
        }
      }
    }
  }

  out_x = start_x;
  out_y = start_y;
  return false;
}

// Calculate the straight-line distance between position and goal
double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const
{
  return std::hypot(
    static_cast<double>(a.x - b.x),
    static_cast<double>(a.y - b.y));
}

// Main A* pathfinding algorithm.
nav_msgs::msg::Path PlannerCore::runAStar(
  int start_x, int start_y, int goal_x, int goal_y) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame_; // Creates an empty path.

  const CellIndex start(start_x, start_y); // Create start and goal cells.
  const CellIndex goal(goal_x, goal_y);

  if (isOccupied(goal_x, goal_y)) { // Checks if the goal cell is occupied. If it is, it returns an empty path and logs a warning.
    RCLCPP_WARN(logger_, "Goal cell is occupied");
    return path;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open; // List for open cells.
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from; // Remembers which cell the current cell came from.
  std::unordered_map<CellIndex, double, CellIndexHash> g_score; // Cost from start to the current cell.
  std::unordered_set<CellIndex, CellIndexHash> closed; // List for closed/processed cells.

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start, goal));

  static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1}; // dx and dy arrays represent the 8 possible movements.
  static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static const double step_cost[8] = { // Associated movement costs for each direction (1.0 for horizontal/vertical, sqrt(2) for diagonal).
    1.0, 1.0, 1.0, 1.0,
    std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)};

  while (!open.empty()) { // Start of A* algorithm. Keep searching as long as there are cells in the open list.
    const CellIndex current = open.top().index; //The priority queue gives A* the cell with the lowest estimated total cost.
    open.pop();

    if (closed.count(current) > 0) { // Skip processed cells.
      continue;
    }
    closed.insert(current);

    // If goal is reached -----------------------------------------------
    if (current == goal) { // If the goal is reached, reconstruct the path by backtracking through the came_from map.
      std::vector<CellIndex> cells;
      CellIndex trace = current;
      cells.push_back(trace); // Start at goal.
      while (came_from.count(trace) > 0) { // Backtrack to the start cell.
        trace = came_from.at(trace);
        cells.push_back(trace);
      }
      std::reverse(cells.begin(), cells.end()); // Reverse the path to get it from start to goal.

      for (const CellIndex& cell : cells) {
        geometry_msgs::msg::PoseStamped pose; // Set the pose for each cell in the path. 
        pose.header.frame_id = map_frame_; // Set the frame ID for the pose.
        gridToWorld(cell.x, cell.y, pose.pose.position.x, pose.pose.position.y); // Convert grid coordinates to world coordinates for the path. Default to center of the cell.
        pose.pose.position.z = 0.0; // Set z position to 0 for a 2D path.
        pose.pose.orientation.w = 1.0; // Set orientation to default (no rotation).
        path.poses.push_back(pose); // Add the pose to the path.
      }
      return path;
    }

    // If goal is not reached -----------------------------------------------
    for (int i = 0; i < 8; ++i) {
      const int nx = current.x + dx[i]; // Calculate the neighbor cell's coordinates based on the current cell and the movement direction.
      const int ny = current.y + dy[i]; 
      if (isOccupied(nx, ny) && CellIndex(nx, ny) != start) {
        continue;
      }

      const CellIndex neighbor(nx, ny);
      if (closed.count(neighbor) > 0) { // Skip already processed cells.
        continue;
      }

      const double tentative_g = g_score[current] + step_cost[i]; // Calculate the cost. 
      if (g_score.count(neighbor) == 0 || tentative_g < g_score[neighbor]) { // For new neighbours or if the new cost is lower than the previous cost, update the path and costs.
        came_from[neighbor] = current; // Remember current cell in the path.
        g_score[neighbor] = tentative_g; // Set the cost of the neighbor cell. 
        open.emplace(neighbor, tentative_g + heuristic(neighbor, goal)); // Add the neighbour in the priority queue with its estimated cost.
      }
    }
  }

  RCLCPP_WARN(logger_, "A* failed to find a path"); // No path exists
  return path;
}

nav_msgs::msg::Path PlannerCore::planPath()
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = map_frame_;

  // Validation steps
  if (!has_map_ || !has_goal_ || map_.data.empty()) {
    RCLCPP_WARN(logger_, "Cannot plan path: missing map or goal");
    return empty_path;
  }

  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;

  if (!worldToGrid(robot_x_, robot_y_, start_x, start_y)) { // Converts the robot's world coordinates to grid coordinates.
    RCLCPP_WARN(logger_, "Robot position is outside the map"); 
    return empty_path;
  }

  if (!worldToGrid(goal_x_, goal_y_, goal_x, goal_y)) { // Converts the goal's world coordinates to grid coordinates.
    RCLCPP_WARN(logger_, "Goal position is outside the map");
    return empty_path;
  }

  int traversable_x = start_x;
  int traversable_y = start_y;
  if (!findTraversableStart(start_x, start_y, traversable_x, traversable_y)) {
    RCLCPP_WARN(logger_, "Robot is inside inflated cells; planning from the robot cell");
  }

  return runAStar(traversable_x, traversable_y, goal_x, goal_y);
}

} 
