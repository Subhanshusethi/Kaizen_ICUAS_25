#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <array>
#include <tuple>
#include <algorithm>
#include <sstream>
#include <fstream>  // For CSV output

using namespace std::chrono_literals;

class AStarPathfinder3DNode : public rclcpp::Node
{
public:
  AStarPathfinder3DNode() : Node("astar_pathfinder_3d_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("octomap_file", "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/city_1/meshes/city_1.binvox.bt");
    this->declare_parameter<double>("grid_resolution", 0.5);
    this->declare_parameter<int>("dilation_iterations", 2);

    // Retrieve parameters
    octomap_file_ = this->get_parameter("octomap_file").as_string();
    grid_resolution_ = this->get_parameter("grid_resolution").as_double();
    dilation_iterations_ = this->get_parameter("dilation_iterations").as_int();

    // Load OctoMap
    octree_ = std::make_shared<octomap::OcTree>(octomap_file_);
    if (!octree_ || octree_->size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load OctoMap file: %s", octomap_file_.c_str());
      rclcpp::shutdown();
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Loaded OctoMap from: %s", octomap_file_.c_str());
    }

    // Get the OctoMap bounding box
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    bb_min_[0] = min_x; bb_min_[1] = min_y; bb_min_[2] = min_z;
    bb_max_[0] = max_x; bb_max_[1] = max_y; bb_max_[2] = max_z;

    // Determine grid dimensions
    grid_size_x_ = static_cast<int>(std::ceil((bb_max_[0] - bb_min_[0]) / grid_resolution_));
    grid_size_y_ = static_cast<int>(std::ceil((bb_max_[1] - bb_min_[1]) / grid_resolution_));
    grid_size_z_ = static_cast<int>(std::ceil((bb_max_[2] - bb_min_[2]) / grid_resolution_));

    RCLCPP_INFO(this->get_logger(),
                "OctoMap bounding box: min=(%.2f,%.2f,%.2f) max=(%.2f,%.2f,%.2f)",
                bb_min_[0], bb_min_[1], bb_min_[2],
                bb_max_[0], bb_max_[1], bb_max_[2]);
    RCLCPP_INFO(this->get_logger(),
                "Grid dimensions: %d x %d x %d (res=%.2f)",
                grid_size_x_, grid_size_y_, grid_size_z_, grid_resolution_);

    // Build the safe grid from the OctoMap
    buildSafeGrid();
    dilateGrid();

    // Create publishers for each drone’s path and A* result.
    for (int i = 1; i <= 5; i++) {
      std::string path_topic = "/uav_" + std::to_string(i) + "_path";
      auto path_pub = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
      path_publishers_[i] = path_pub;

      std::string result_topic = "/uav_" + std::to_string(i) + "_path_result";
      auto result_pub = this->create_publisher<std_msgs::msg::Bool>(result_topic, 10);
      path_result_publishers_[i] = result_pub;
    }

    // Store the subscription objects so they persist
    for (int i = 1; i <= 5; i++) {
      // Subscribe to 3D goals (PoseStamped)
      std::string goal_topic = "/drone" + std::to_string(i) + "_goal";
      RCLCPP_INFO(this->get_logger(), "Subscribing to %s", goal_topic.c_str());
      goal_subscriptions_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_topic, 10,
          [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            std::array<double, 3> goal = {
              msg->pose.position.x,
              msg->pose.position.y,
              msg->pose.position.z
            };
            drone_goals_[i] = goal;
            RCLCPP_INFO(this->get_logger(), "Drone %d received new goal: (%.2f, %.2f, %.2f)",
                        i, goal[0], goal[1], goal[2]);
          });

      // Subscribe to odometry topics
      std::string odom_topic = "/cf_" + std::to_string(i) + "/odom";
      odom_subscriptions_[i] = this->create_subscription<nav_msgs::msg::Odometry>(
          odom_topic, 10,
          [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::array<double, 3> pos = {
              msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z
            };
            drone_positions_[i] = pos;
          });
    }

    // Timer for planning (e.g. 0.5 sec interval)
    timer_ = this->create_wall_timer(500ms, std::bind(&AStarPathfinder3DNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "AStarPathfinder3DNode initialized.");
  }

private:
  // Parameters
  std::string octomap_file_;
  double grid_resolution_;
  int dilation_iterations_;

  // OctoMap pointer
  std::shared_ptr<octomap::OcTree> octree_;

  // Bounding box (min and max for x,y,z)
  double bb_min_[3], bb_max_[3];
  int grid_size_x_, grid_size_y_, grid_size_z_;

  // The safe grid (1 = free, 0 = obstacle)
  // index = x + grid_size_x_ * (y + grid_size_y_ * z)
  std::vector<int> safe_grid_;

  // Drone state: current positions and goals (for drones 1..5)
  std::unordered_map<int, std::array<double, 3>> drone_positions_;
  std::unordered_map<int, std::array<double, 3>> drone_goals_;

  // Publishers for each drone’s planned path and A* result (Bool)
  std::unordered_map<int, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_publishers_;
  std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> path_result_publishers_;

  // Subscriptions must be stored to remain active
  std::unordered_map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> goal_subscriptions_;
  std::unordered_map<int, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscriptions_;

  // Timer for planning
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Grid utility functions ---
  inline int indexFromCoord(int x, int y, int z) const {
    return x + grid_size_x_ * (y + grid_size_y_ * z);
  }

  void coordFromIndex(int idx, int &x, int &y, int &z) const {
    x = idx % grid_size_x_;
    int rem = idx / grid_size_x_;
    y = rem % grid_size_y_;
    z = rem / grid_size_y_;
  }

  // Convert grid indices back to world coordinates (center of cell)
  std::array<double, 3> gridToWorld(int x, int y, int z) const {
    double wx = bb_min_[0] + (x + 0.5) * grid_resolution_;
    double wy = bb_min_[1] + (y + 0.5) * grid_resolution_;
    double wz = bb_min_[2] + (z + 0.5) * grid_resolution_;
    return {wx, wy, wz};
  }

  // Convert a world coordinate (x,y,z) to grid indices (gx,gy,gz)
  std::tuple<int,int,int> worldToGrid(const std::array<double,3>& pos) const {
    int gx = static_cast<int>(std::floor((pos[0] - bb_min_[0]) / grid_resolution_));
    int gy = static_cast<int>(std::floor((pos[1] - bb_min_[1]) / grid_resolution_));
    int gz = static_cast<int>(std::floor((pos[2] - bb_min_[2]) / grid_resolution_));
    return std::make_tuple(gx, gy, gz);
  }

  // --- Build safe grid from OctoMap ---
  void buildSafeGrid()
  {
    int total = grid_size_x_ * grid_size_y_ * grid_size_z_;
    safe_grid_.resize(total, 1);  // assume free initially

    for (int z = 0; z < grid_size_z_; z++) {
      for (int y = 0; y < grid_size_y_; y++) {
        for (int x = 0; x < grid_size_x_; x++) {
          std::array<double, 3> world_pt = gridToWorld(x, y, z);
          // Query OctoMap: if no node found, treat as free; 
          // if node exists, check occupancy.
          octomap::OcTreeNode* node = octree_->search(world_pt[0], world_pt[1], world_pt[2]);
          int idx = indexFromCoord(x, y, z);
          if (node && octree_->isNodeOccupied(node)) {
            safe_grid_[idx] = 0;  // obstacle
          } else {
            safe_grid_[idx] = 1;  // free
          }
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "Safe grid built from OctoMap.");
  }

  // --- Perform binary dilation on the safe grid to expand obstacles ---
  void dilateGrid()
  {
    std::vector<int> dilated = safe_grid_; // copy current
    for (int iter = 0; iter < dilation_iterations_; iter++) {
      std::vector<int> temp = dilated; // copy current
      for (int z = 0; z < grid_size_z_; z++) {
        for (int y = 0; y < grid_size_y_; y++) {
          for (int x = 0; x < grid_size_x_; x++) {
            int idx = indexFromCoord(x, y, z);
            if (dilated[idx] == 1) {
              // Check neighbors in 3x3x3
              for (int dz = -1; dz <= 1; dz++) {
                for (int dy = -1; dy <= 1; dy++) {
                  for (int dx_ = -1; dx_ <= 1; dx_++) {
                    int nx = x + dx_, ny = y + dy, nz = z + dz;
                    if (nx < 0 || nx >= grid_size_x_ ||
                        ny < 0 || ny >= grid_size_y_ ||
                        nz < 0 || nz >= grid_size_z_)
                    {
                      continue;
                    }
                    int nidx = indexFromCoord(nx, ny, nz);
                    if (dilated[nidx] == 0) {
                      // If a neighbor is obstacle,
                      // this cell becomes obstacle in the next iteration
                      temp[idx] = 0;
                      goto nextCell;
                    }
                  }
                }
              }
            }
          nextCell:;
          }
        }
      }
      dilated = temp;
    }
    safe_grid_ = dilated;
    RCLCPP_INFO(this->get_logger(), "Safe grid dilated with %d iterations.", dilation_iterations_);
  }

  // --- Check if a grid cell is free (within bounds and safe) ---
  bool isFree(int x, int y, int z) const {
    if (x < 0 || x >= grid_size_x_ ||
        y < 0 || y >= grid_size_y_ ||
        z < 0 || z >= grid_size_z_)
    {
      return false;
    }
    int idx = indexFromCoord(x, y, z);
    return (safe_grid_[idx] == 1);
  }

  // A continuous line-of-sight check in the grid. 
  // Steps along the line at a fraction of grid_resolution_ to ensure no collisions.
  bool isLineFree(const std::array<double,3>& startW, 
                  const std::array<double,3>& endW) const 
  {
    // Simple approach: sample points between startW and endW at ~ half a cell resolution
    const double step_size = grid_resolution_ * 0.5;
    double dx = endW[0] - startW[0];
    double dy = endW[1] - startW[1];
    double dz = endW[2] - startW[2];
    double length = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (length < 1e-6) {
      // The same point, trivial
      return true;
    }

    int steps = static_cast<int>(std::ceil(length / step_size));
    // Unit direction
    double ux = dx / length;
    double uy = dy / length;
    double uz = dz / length;

    for (int i = 0; i <= steps; i++) {
      double t = (double)i / (double)steps;
      double px = startW[0] + t * dx;
      double py = startW[1] + t * dy;
      double pz = startW[2] + t * dz;
      // Convert to grid
      auto [gx, gy, gz] = worldToGrid({px, py, pz});
      if (!isFree(gx, gy, gz)) {
        return false;
      }
    }
    return true;
  }

  // --- A* search in 3D ---
  std::vector<int> astarSearch(const std::tuple<int,int,int>& start,
                               const std::tuple<int,int,int>& goal)
  {
    auto [sx, sy, sz] = start;
    auto [gx, gy, gz] = goal;
    int start_idx = indexFromCoord(sx, sy, sz);
    int goal_idx  = indexFromCoord(gx, gy, gz);

    using PQElement = std::pair<double, int>; // (f_score, cell_index)
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> open_set;
    open_set.push({0.0, start_idx});

    std::vector<double> g_score(safe_grid_.size(), std::numeric_limits<double>::infinity());
    g_score[start_idx] = 0.0;

    std::vector<int> came_from(safe_grid_.size(), -1);

    auto heuristic = [this, goal](int idx) {
      int cx, cy, cz;
      coordFromIndex(idx, cx, cy, cz);
      auto [gx, gy, gz] = goal;
      double dx = (cx - gx);
      double dy = (cy - gy);
      double dz = (cz - gz);
      return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    std::vector<double> f_score(safe_grid_.size(), std::numeric_limits<double>::infinity());
    f_score[start_idx] = heuristic(start_idx);

    while (!open_set.empty()) {
      auto [current_f, current] = open_set.top();
      open_set.pop();

      if (current == goal_idx) {
        // Reconstruct path
        std::vector<int> path;
        int curr = goal_idx;
        while (curr != -1) {
          path.push_back(curr);
          curr = came_from[curr];
        }
        std::reverse(path.begin(), path.end());
        return path;
      }

      // Skip stale queue entries
      if (current_f > f_score[current]) {
        continue;
      }

      int cx, cy, cz;
      coordFromIndex(current, cx, cy, cz);

      // Explore neighbors
      for (int dz = -1; dz <= 1; dz++) {
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx_ = -1; dx_ <= 1; dx_++) {
            if (dx_ == 0 && dy == 0 && dz == 0) {
              continue;
            }
            int nx = cx + dx_, ny = cy + dy, nz = cz + dz;

            if (!isFree(nx, ny, nz)) {
              continue;
            }

            int neighbor_idx = indexFromCoord(nx, ny, nz);
            double dist = std::sqrt((double)(dx_*dx_ + dy*dy + dz*dz));
            double tentative_g = g_score[current] + dist;
            if (tentative_g < g_score[neighbor_idx]) {
              came_from[neighbor_idx] = current;
              g_score[neighbor_idx]   = tentative_g;
              double f = tentative_g + heuristic(neighbor_idx);
              f_score[neighbor_idx] = f;
              open_set.push({f, neighbor_idx});
            }
          }
        }
      }
    }
    // Return empty if no path found
    return std::vector<int>{};
  }
  std::vector<std::array<double, 3>> subdivideLine(
      const std::array<double, 3>& startW,
      const std::array<double, 3>& endW,
      double step)
  {
      std::vector<std::array<double, 3>> result;
      // If step <= 0, fallback to returning just start->end
      if (step <= 0.0) {
          result.push_back(startW);
          result.push_back(endW);
          return result;
      }
      double dx = endW[0] - startW[0];
      double dy = endW[1] - startW[1];
      double dz = endW[2] - startW[2];
      double length = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (length < 1e-6) {
          // The same point
          result.push_back(startW);
          return result;
      }
  
      int steps = static_cast<int>(std::floor(length / step));
      // Add each intermediate
      for (int i = 0; i <= steps; i++) {
          double t = (double)i / (double)steps;
          std::array<double, 3> pt = {
              startW[0] + t * dx,
              startW[1] + t * dy,
              startW[2] + t * dz
          };
          result.push_back(pt);
      }
  
      // Make sure we end exactly at endW, 
      // though the loop above with i=steps does this
      // result.push_back(endW);
      return result;
  }
  // --- Shortcut-based path smoothing ---
  std::vector<std::array<double,3>> smoothPath(
      const std::vector<std::array<double,3>>& raw_path,
      double subdivision_step)
  {
      if (raw_path.size() < 2) {
          return raw_path;  // trivial or no path
      }
  
      std::vector<std::array<double,3>> smoothed;
      smoothed.reserve(raw_path.size());  // just a guess
  
      // We keep the first waypoint
      std::array<double,3> firstPt = raw_path.front();
      smoothed.push_back(firstPt);
  
      // We'll do a "jump forward" smoothing:
      int i = 0;  // index in raw_path
      while (i < (int)raw_path.size() - 1) {
          // Attempt the furthest j we can skip to while line-of-sight is free
          int furthest = (int)raw_path.size() - 1;
          for (int j = (int)raw_path.size() - 1; j > i; j--) {
              if (isLineFree(raw_path[i], raw_path[j])) {
                  furthest = j;
                  break;
              }
          }
  
          // Now subdivide from raw_path[i] to raw_path[furthest]
          // But we skip the "start" of each segment if it's already in the smoothed path
          std::array<double,3> startW = raw_path[i];
          std::array<double,3> endW   = raw_path[furthest];
  
          // This returns a vector of [start, ..., end]
          std::vector<std::array<double,3>> segment = subdivideLine(startW, endW, subdivision_step);
          // We already have "start" from previous iteration, so pop it to avoid duplication
          if (segment.size() > 1) {
              segment.erase(segment.begin());
          }
          // Append to smoothed
          smoothed.insert(smoothed.end(), segment.begin(), segment.end());
  
          // Move i to 'furthest' so we can proceed
          i = furthest;
      }
  
      return smoothed;
  }

  // Publish the computed path as a nav_msgs::msg::Path message
  // and save the waypoints in CSV format.
  void publishPath(int drone_id, const std::vector<std::array<double,3>>& path)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "world";

    for (const auto &pt : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = pt[0];
      pose.pose.position.y = pt[1];
      pose.pose.position.z = pt[2];
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    path_publishers_[drone_id]->publish(path_msg);

    // Save the waypoints to CSV
    savePathToCSV(drone_id, path);
  }

  // --- Helper function: Save waypoints to a CSV file ---
  void savePathToCSV(int drone_id, const std::vector<std::array<double,3>>& path)
  {
    std::string file_name = "/root/CrazySim/ros2_ws/src/icuas25_competition/startup/csv_save/drone_" + std::to_string(drone_id) + "_waypoints.csv";
    std::ofstream file(file_name);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file %s for writing", file_name.c_str());
      return;
    }
    // Write CSV header (optional)
    file << "x,y,z\n";
    for (const auto &pt : path) {
      file << pt[0] << "," << pt[1] << "," << pt[2] << "\n";
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), "Saved path to %s", file_name.c_str());
  }

  // --- Timer callback: For each drone, plan a 3D path using A* and then smooth it ---
  void timerCallback()
  {
    for (int drone_id = 1; drone_id <= 5; drone_id++) {
      // Skip if we haven't received both position and goal
      if (drone_goals_.find(drone_id) == drone_goals_.end() ||
          drone_positions_.find(drone_id) == drone_positions_.end())
      {
        continue;
      }

      std::array<double,3> current = drone_positions_[drone_id];
      std::array<double,3> goal = drone_goals_[drone_id];

      // Convert world to grid
      auto start = worldToGrid(current);
      auto end   = worldToGrid(goal);

      // --- PRE-CHECK: is start in-bounds and free? ---
      {
        auto [sx, sy, sz] = start;
        if (sx < 0 || sx >= grid_size_x_ ||
            sy < 0 || sy >= grid_size_y_ ||
            sz < 0 || sz >= grid_size_z_)
        {
          RCLCPP_WARN(this->get_logger(),
                      "Drone %d start (%.2f,%.2f,%.2f) => grid (%d,%d,%d) out-of-bounds!",
                      drone_id, current[0], current[1], current[2], sx, sy, sz);
          continue;
        }
        if (!isFree(sx, sy, sz)) {
          RCLCPP_WARN(this->get_logger(),
                      "Drone %d start is in obstacle or unknown space!", drone_id);
          continue;
        }
      }

      // --- PRE-CHECK: is goal in-bounds and free? ---
      {
        auto [gx, gy, gz] = end;
        if (gx < 0 || gx >= grid_size_x_ ||
            gy < 0 || gy >= grid_size_y_ ||
            gz < 0 || gz >= grid_size_z_)
        {
          RCLCPP_WARN(this->get_logger(),
                      "Drone %d goal (%.2f,%.2f,%.2f) => grid (%d,%d,%d) out-of-bounds!",
                      drone_id, goal[0], goal[1], goal[2], gx, gy, gz);
          continue;
        }
        if (!isFree(gx, gy, gz)) {
          RCLCPP_WARN(this->get_logger(),
                      "Drone %d goal is in obstacle or unknown space!", drone_id);
          continue;
        }
      }

      // Run A*
      std::vector<int> path_indices = astarSearch(start, end);

      std_msgs::msg::Bool result_msg;
      if (!path_indices.empty()) {
        // Convert path indices to world coordinates
        std::vector<std::array<double,3>> world_path;
        world_path.reserve(path_indices.size());
        for (int idx : path_indices) {
          int gx, gy, gz;
          coordFromIndex(idx, gx, gy, gz);
          world_path.push_back(gridToWorld(gx, gy, gz));
        }

        double subdivision_step = 2.5; // or 0.1, etc.

        // Smooth the path
        std::vector<std::array<double,3>> final_path = smoothPath(world_path, subdivision_step);

        // Publish and save path
        publishPath(drone_id, final_path);

        result_msg.data = true;
        RCLCPP_INFO(this->get_logger(), 
                    "A* path found & smoothed for Drone %d (raw length=%zu, smoothed=%zu)",
                    drone_id, world_path.size(), final_path.size());
      } else {
        result_msg.data = false;
        RCLCPP_WARN(this->get_logger(), "No valid A* path for Drone %d", drone_id);
      }

      // Publish True/False for path availability
      path_result_publishers_[drone_id]->publish(result_msg);

      // Optionally clear the goal to avoid repeated replanning
      drone_goals_.erase(drone_id);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AStarPathfinder3DNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/path.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <octomap/octomap.h>
// #include <octomap/OcTree.h>

// #include <vector>
// #include <queue>
// #include <cmath>
// #include <limits>
// #include <unordered_map>
// #include <array>
// #include <tuple>
// #include <algorithm>
// #include <sstream>
// #include <fstream>  // For CSV output

// using namespace std::chrono_literals;

// class AStarPathfinder3DNode : public rclcpp::Node
// {
// public:
//   AStarPathfinder3DNode() : Node("astar_pathfinder_3d_node")
//   {
//     // Declare parameters
//     this->declare_parameter<std::string>("octomap_file", "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/city_1/meshes/city_1.binvox.bt");
//     this->declare_parameter<double>("grid_resolution", 0.5);
//     this->declare_parameter<int>("dilation_iterations", 2);

//     // Retrieve parameters
//     octomap_file_ = this->get_parameter("octomap_file").as_string();
//     grid_resolution_ = this->get_parameter("grid_resolution").as_double();
//     dilation_iterations_ = this->get_parameter("dilation_iterations").as_int();

//     // Load OctoMap
//     octree_ = std::make_shared<octomap::OcTree>(octomap_file_);
//     if (!octree_ || octree_->size() == 0) {
//       RCLCPP_ERROR(this->get_logger(), "Failed to load OctoMap file: %s", octomap_file_.c_str());
//       rclcpp::shutdown();
//       return;
//     } else {
//       RCLCPP_INFO(this->get_logger(), "Loaded OctoMap from: %s", octomap_file_.c_str());
//     }

//     // Get the OctoMap bounding box
//     double min_x, min_y, min_z;
//     double max_x, max_y, max_z;
//     octree_->getMetricMin(min_x, min_y, min_z);
//     octree_->getMetricMax(max_x, max_y, max_z);
//     bb_min_[0] = min_x; bb_min_[1] = min_y; bb_min_[2] = min_z;
//     bb_max_[0] = max_x; bb_max_[1] = max_y; bb_max_[2] = max_z;

//     // Determine grid dimensions
//     grid_size_x_ = static_cast<int>(std::ceil((bb_max_[0] - bb_min_[0]) / grid_resolution_));
//     grid_size_y_ = static_cast<int>(std::ceil((bb_max_[1] - bb_min_[1]) / grid_resolution_));
//     grid_size_z_ = static_cast<int>(std::ceil((bb_max_[2] - bb_min_[2]) / grid_resolution_));

//     RCLCPP_INFO(this->get_logger(),
//                 "OctoMap bounding box: min=(%.2f,%.2f,%.2f) max=(%.2f,%.2f,%.2f)",
//                 bb_min_[0], bb_min_[1], bb_min_[2],
//                 bb_max_[0], bb_max_[1], bb_max_[2]);
//     RCLCPP_INFO(this->get_logger(),
//                 "Grid dimensions: %d x %d x %d (res=%.2f)",
//                 grid_size_x_, grid_size_y_, grid_size_z_, grid_resolution_);

//     // Build the safe grid from the OctoMap
//     buildSafeGrid();
//     dilateGrid();

//     // Create publishers for each drone’s path and A* result.
//     for (int i = 1; i <= 5; i++) {
//       std::string path_topic = "/uav_" + std::to_string(i) + "_path";
//       auto path_pub = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
//       path_publishers_[i] = path_pub;

//       std::string result_topic = "/uav_" + std::to_string(i) + "_path_result";
//       auto result_pub = this->create_publisher<std_msgs::msg::Bool>(result_topic, 10);
//       path_result_publishers_[i] = result_pub;
//     }

//     // Store the subscription objects so they persist
//     for (int i = 1; i <= 5; i++) {
//       // Subscribe to 3D goals (PoseStamped)
//       std::string goal_topic = "/drone" + std::to_string(i) + "_goal";
//       RCLCPP_INFO(this->get_logger(), "Subscribing to %s", goal_topic.c_str());
//       goal_subscriptions_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//           goal_topic, 10,
//           [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//             std::array<double, 3> goal = {
//               msg->pose.position.x,
//               msg->pose.position.y,
//               msg->pose.position.z
//             };
//             drone_goals_[i] = goal;
//             RCLCPP_INFO(this->get_logger(), "Drone %d received new goal: (%.2f, %.2f, %.2f)",
//                         i, goal[0], goal[1], goal[2]);
//           });

//       // Subscribe to odometry topics
//       std::string odom_topic = "/cf_" + std::to_string(i) + "/odom";
//       odom_subscriptions_[i] = this->create_subscription<nav_msgs::msg::Odometry>(
//           odom_topic, 10,
//           [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
//             std::array<double, 3> pos = {
//               msg->pose.pose.position.x,
//               msg->pose.pose.position.y,
//               msg->pose.pose.position.z
//             };
//             drone_positions_[i] = pos;
//           });
//     }

//     // Timer for planning (e.g. 0.5 sec interval)
//     timer_ = this->create_wall_timer(500ms, std::bind(&AStarPathfinder3DNode::timerCallback, this));

//     RCLCPP_INFO(this->get_logger(), "AStarPathfinder3DNode initialized.");
//   }

// private:
//   // Parameters
//   std::string octomap_file_;
//   double grid_resolution_;
//   int dilation_iterations_;

//   // OctoMap pointer
//   std::shared_ptr<octomap::OcTree> octree_;

//   // Bounding box (min and max for x,y,z)
//   double bb_min_[3], bb_max_[3];
//   int grid_size_x_, grid_size_y_, grid_size_z_;

//   // The safe grid (1 = free, 0 = obstacle)
//   // index = x + grid_size_x_ * (y + grid_size_y_ * z)
//   std::vector<int> safe_grid_;

//   // Drone state: current positions and goals (for drones 1..5)
//   std::unordered_map<int, std::array<double, 3>> drone_positions_;
//   std::unordered_map<int, std::array<double, 3>> drone_goals_;

//   // Publishers for each drone’s planned path and A* result (Bool)
//   std::unordered_map<int, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_publishers_;
//   std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> path_result_publishers_;

//   // Subscriptions must be stored to remain active
//   std::unordered_map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> goal_subscriptions_;
//   std::unordered_map<int, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscriptions_;

//   // Timer for planning
//   rclcpp::TimerBase::SharedPtr timer_;

//   // --- Grid utility functions ---
//   inline int indexFromCoord(int x, int y, int z) const {
//     return x + grid_size_x_ * (y + grid_size_y_ * z);
//   }

//   void coordFromIndex(int idx, int &x, int &y, int &z) const {
//     x = idx % grid_size_x_;
//     int rem = idx / grid_size_x_;
//     y = rem % grid_size_y_;
//     z = rem / grid_size_y_;
//   }

//   // Convert grid indices back to world coordinates (center of cell)
//   std::array<double, 3> gridToWorld(int x, int y, int z) const {
//     double wx = bb_min_[0] + (x + 0.5) * grid_resolution_;
//     double wy = bb_min_[1] + (y + 0.5) * grid_resolution_;
//     double wz = bb_min_[2] + (z + 0.5) * grid_resolution_;
//     return {wx, wy, wz};
//   }

//   // Convert a world coordinate (x,y,z) to grid indices (gx,gy,gz)
//   std::tuple<int,int,int> worldToGrid(const std::array<double,3>& pos) const {
//     int gx = static_cast<int>(std::floor((pos[0] - bb_min_[0]) / grid_resolution_));
//     int gy = static_cast<int>(std::floor((pos[1] - bb_min_[1]) / grid_resolution_));
//     int gz = static_cast<int>(std::floor((pos[2] - bb_min_[2]) / grid_resolution_));
//     return std::make_tuple(gx, gy, gz);
//   }

//   // --- Build safe grid from OctoMap ---
//   void buildSafeGrid()
//   {
//     int total = grid_size_x_ * grid_size_y_ * grid_size_z_;
//     safe_grid_.resize(total, 1);  // assume free initially

//     for (int z = 0; z < grid_size_z_; z++) {
//       for (int y = 0; y < grid_size_y_; y++) {
//         for (int x = 0; x < grid_size_x_; x++) {
//           std::array<double, 3> world_pt = gridToWorld(x, y, z);
//           // Query OctoMap: if no node found, treat as free; 
//           // if node exists, check occupancy.
//           octomap::OcTreeNode* node = octree_->search(world_pt[0], world_pt[1], world_pt[2]);
//           int idx = indexFromCoord(x, y, z);
//           if (node && octree_->isNodeOccupied(node)) {
//             safe_grid_[idx] = 0;  // obstacle
//           } else {
//             safe_grid_[idx] = 1;  // free
//           }
//         }
//       }
//     }
//     RCLCPP_INFO(this->get_logger(), "Safe grid built from OctoMap.");
//   }

//   // --- Perform binary dilation on the safe grid to expand obstacles ---
//   void dilateGrid()
//   {
//     std::vector<int> dilated = safe_grid_; // copy current
//     for (int iter = 0; iter < dilation_iterations_; iter++) {
//       std::vector<int> temp = dilated; // copy current
//       for (int z = 0; z < grid_size_z_; z++) {
//         for (int y = 0; y < grid_size_y_; y++) {
//           for (int x = 0; x < grid_size_x_; x++) {
//             int idx = indexFromCoord(x, y, z);
//             if (dilated[idx] == 1) {
//               // Check neighbors in 3x3x3
//               for (int dz = -1; dz <= 1; dz++) {
//                 for (int dy = -1; dy <= 1; dy++) {
//                   for (int dx_ = -1; dx_ <= 1; dx_++) {
//                     int nx = x + dx_, ny = y + dy, nz = z + dz;
//                     if (nx < 0 || nx >= grid_size_x_ ||
//                         ny < 0 || ny >= grid_size_y_ ||
//                         nz < 0 || nz >= grid_size_z_)
//                     {
//                       continue;
//                     }
//                     int nidx = indexFromCoord(nx, ny, nz);
//                     if (dilated[nidx] == 0) {
//                       // If a neighbor is obstacle,
//                       // this cell becomes obstacle in the next iteration
//                       temp[idx] = 0;
//                       goto nextCell;
//                     }
//                   }
//                 }
//               }
//             }
//           nextCell:;
//           }
//         }
//       }
//       dilated = temp;
//     }
//     safe_grid_ = dilated;
//     RCLCPP_INFO(this->get_logger(), "Safe grid dilated with %d iterations.", dilation_iterations_);
//   }

//   // --- Check if a grid cell is free (within bounds and safe) ---
//   bool isFree(int x, int y, int z) const {
//     if (x < 0 || x >= grid_size_x_ ||
//         y < 0 || y >= grid_size_y_ ||
//         z < 0 || z >= grid_size_z_)
//     {
//       return false;
//     }
//     int idx = indexFromCoord(x, y, z);
//     return (safe_grid_[idx] == 1);
//   }

//   // A continuous line-of-sight check in the grid. 
//   // Steps along the line at a fraction of grid_resolution_ to ensure no collisions.
//   bool isLineFree(const std::array<double,3>& startW, 
//                   const std::array<double,3>& endW) const 
//   {
//     // Sample points between startW and endW at roughly half a cell resolution.
//     const double step_size = grid_resolution_ * 0.5;
//     double dx = endW[0] - startW[0];
//     double dy = endW[1] - startW[1];
//     double dz = endW[2] - startW[2];
//     double length = std::sqrt(dx*dx + dy*dy + dz*dz);
//     if (length < 1e-6) {
//       return true;
//     }

//     int steps = static_cast<int>(std::ceil(length / step_size));
//     for (int i = 0; i <= steps; i++) {
//       double t = static_cast<double>(i) / steps;
//       double px = startW[0] + t * dx;
//       double py = startW[1] + t * dy;
//       double pz = startW[2] + t * dz;
//       auto [gx, gy, gz] = worldToGrid({px, py, pz});
//       if (!isFree(gx, gy, gz)) {
//         return false;
//       }
//     }
//     return true;
//   }

//   // --- A* search in 3D ---
//   std::vector<int> astarSearch(const std::tuple<int,int,int>& start,
//                                const std::tuple<int,int,int>& goal)
//   {
//     auto [sx, sy, sz] = start;
//     auto [gx, gy, gz] = goal;
//     int start_idx = indexFromCoord(sx, sy, sz);
//     int goal_idx  = indexFromCoord(gx, gy, gz);

//     using PQElement = std::pair<double, int>; // (f_score, cell_index)
//     std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> open_set;
//     open_set.push({0.0, start_idx});

//     std::vector<double> g_score(safe_grid_.size(), std::numeric_limits<double>::infinity());
//     g_score[start_idx] = 0.0;

//     std::vector<int> came_from(safe_grid_.size(), -1);

//     auto heuristic = [this, goal](int idx) {
//       int cx, cy, cz;
//       coordFromIndex(idx, cx, cy, cz);
//       auto [gx, gy, gz] = goal;
//       double dx = (cx - gx);
//       double dy = (cy - gy);
//       double dz = (cz - gz);
//       return std::sqrt(dx*dx + dy*dy + dz*dz);
//     };

//     std::vector<double> f_score(safe_grid_.size(), std::numeric_limits<double>::infinity());
//     f_score[start_idx] = heuristic(start_idx);

//     while (!open_set.empty()) {
//       auto [current_f, current] = open_set.top();
//       open_set.pop();

//       if (current == goal_idx) {
//         std::vector<int> path;
//         int curr = goal_idx;
//         while (curr != -1) {
//           path.push_back(curr);
//           curr = came_from[curr];
//         }
//         std::reverse(path.begin(), path.end());
//         return path;
//       }

//       if (current_f > f_score[current]) {
//         continue;
//       }

//       int cx, cy, cz;
//       coordFromIndex(current, cx, cy, cz);

//       // Explore neighbors in a 3x3x3 neighborhood.
//       for (int dz = -1; dz <= 1; dz++) {
//         for (int dy = -1; dy <= 1; dy++) {
//           for (int dx_ = -1; dx_ <= 1; dx_++) {
//             if (dx_ == 0 && dy == 0 && dz == 0) continue;
//             int nx = cx + dx_, ny = cy + dy, nz = cz + dz;
//             if (!isFree(nx, ny, nz)) continue;
//             int neighbor_idx = indexFromCoord(nx, ny, nz);
//             double dist = std::sqrt(static_cast<double>(dx_*dx_ + dy*dy + dz*dz));
//             double tentative_g = g_score[current] + dist;
//             if (tentative_g < g_score[neighbor_idx]) {
//               came_from[neighbor_idx] = current;
//               g_score[neighbor_idx]   = tentative_g;
//               double f = tentative_g + heuristic(neighbor_idx);
//               f_score[neighbor_idx] = f;
//               open_set.push({f, neighbor_idx});
//             }
//           }
//         }
//       }
//     }
//     return std::vector<int>{};  // No path found
//   }

//   // --- Original Shortcut-based path smoothing (optional) ---
//   std::vector<std::array<double,3>> smoothPathShortcut(
//       const std::vector<std::array<double,3>>& raw_path) 
//   {
//     if (raw_path.size() < 2) {
//       return raw_path;
//     }
//     std::vector<std::array<double,3>> smoothed;
//     smoothed.push_back(raw_path.front());
//     int i = 0;
//     while (i < static_cast<int>(raw_path.size()) - 1) {
//       int furthest = raw_path.size() - 1;
//       for (int j = raw_path.size() - 1; j > i; j--) {
//         if (isLineFree(raw_path[i], raw_path[j])) {
//           furthest = j;
//           break;
//         }
//       }
//       smoothed.push_back(raw_path[furthest]);
//       i = furthest;
//     }
//     return smoothed;
//   }

//   // --- Alternative smoothing using Catmull–Rom spline ---
//   std::vector<std::array<double,3>> smoothPathCatmullRom(
//       const std::vector<std::array<double,3>>& raw_path, int segments_per_curve = 10)
//   {
//     std::vector<std::array<double,3>> smoothed_path;
//     int n = raw_path.size();
//     if (n < 2) {
//       return raw_path;
//     }

//     // Extend the raw path for boundary conditions by duplicating the first and last waypoints.
//     std::vector<std::array<double,3>> extended;
//     extended.push_back(raw_path.front());
//     extended.insert(extended.end(), raw_path.begin(), raw_path.end());
//     extended.push_back(raw_path.back());

//     // Generate spline segments between each pair of original points.
//     for (size_t i = 1; i < extended.size() - 2; i++) {
//       const auto& P0 = extended[i - 1];
//       const auto& P1 = extended[i];
//       const auto& P2 = extended[i + 1];
//       const auto& P3 = extended[i + 2];

//       for (int j = 0; j < segments_per_curve; j++) {
//         double t = static_cast<double>(j) / segments_per_curve;
//         std::array<double,3> point;
//         for (int dim = 0; dim < 3; dim++) {
//           point[dim] = 0.5 * ((2.0 * P1[dim]) +
//                               (-P0[dim] + P2[dim]) * t +
//                               (2.0 * P0[dim] - 5.0 * P1[dim] + 4.0 * P2[dim] - P3[dim]) * t * t +
//                               (-P0[dim] + 3.0 * P1[dim] - 3.0 * P2[dim] + P3[dim]) * t * t * t);
//         }
//         smoothed_path.push_back(point);
//       }
//     }
//     // Ensure the final waypoint is included.
//     smoothed_path.push_back(raw_path.back());
//     return smoothed_path;
//   }

//   // Publish the computed path as a nav_msgs::msg::Path message and save to CSV.
//   void publishPath(int drone_id, const std::vector<std::array<double,3>>& path)
//   {
//     nav_msgs::msg::Path path_msg;
//     path_msg.header.stamp = this->now();
//     path_msg.header.frame_id = "world";

//     for (const auto &pt : path) {
//       geometry_msgs::msg::PoseStamped pose;
//       pose.header = path_msg.header;
//       pose.pose.position.x = pt[0];
//       pose.pose.position.y = pt[1];
//       pose.pose.position.z = pt[2];
//       pose.pose.orientation.w = 1.0;
//       path_msg.poses.push_back(pose);
//     }

//     path_publishers_[drone_id]->publish(path_msg);
//     savePathToCSV(drone_id, path);
//   }

//   // --- Helper function: Save waypoints to a CSV file ---
//   void savePathToCSV(int drone_id, const std::vector<std::array<double,3>>& path)
//   {
//     std::string file_name = "/root/CrazySim/ros2_ws/src/icuas25_competition/startup/csv_save/drone_" + std::to_string(drone_id) + "_waypoints.csv";
//     std::ofstream file(file_name);
//     if (!file.is_open()) {
//       RCLCPP_ERROR(this->get_logger(), "Failed to open file %s for writing", file_name.c_str());
//       return;
//     }
//     file << "x,y,z\n";
//     for (const auto &pt : path) {
//       file << pt[0] << "," << pt[1] << "," << pt[2] << "\n";
//     }
//     file.close();
//     RCLCPP_INFO(this->get_logger(), "Saved path to %s", file_name.c_str());
//   }

//   // --- Timer callback: For each drone, plan a 3D path using A* and then smooth it ---
//   void timerCallback()
//   {
//     for (int drone_id = 1; drone_id <= 5; drone_id++) {
//       // Skip if we haven't received both position and goal
//       if (drone_goals_.find(drone_id) == drone_goals_.end() ||
//           drone_positions_.find(drone_id) == drone_positions_.end())
//       {
//         continue;
//       }

//       std::array<double,3> current = drone_positions_[drone_id];
//       std::array<double,3> goal = drone_goals_[drone_id];

//       // Convert world coordinates to grid indices.
//       auto start = worldToGrid(current);
//       auto end   = worldToGrid(goal);

//       // --- PRE-CHECK: Verify that start is in bounds and free ---
//       {
//         auto [sx, sy, sz] = start;
//         if (sx < 0 || sx >= grid_size_x_ ||
//             sy < 0 || sy >= grid_size_y_ ||
//             sz < 0 || sz >= grid_size_z_)
//         {
//           RCLCPP_WARN(this->get_logger(),
//                       "Drone %d start (%.2f,%.2f,%.2f) => grid (%d,%d,%d) out-of-bounds!",
//                       drone_id, current[0], current[1], current[2], sx, sy, sz);
//           continue;
//         }
//         if (!isFree(sx, sy, sz)) {
//           RCLCPP_WARN(this->get_logger(), "Drone %d start is in obstacle or unknown space!", drone_id);
//           continue;
//         }
//       }

//       // --- PRE-CHECK: Verify that goal is in bounds and free ---
//       {
//         auto [gx, gy, gz] = end;
//         if (gx < 0 || gx >= grid_size_x_ ||
//             gy < 0 || gy >= grid_size_y_ ||
//             gz < 0 || gz >= grid_size_z_)
//         {
//           RCLCPP_WARN(this->get_logger(),
//                       "Drone %d goal (%.2f,%.2f,%.2f) => grid (%d,%d,%d) out-of-bounds!",
//                       drone_id, goal[0], goal[1], goal[2], gx, gy, gz);
//           continue;
//         }
//         if (!isFree(gx, gy, gz)) {
//           RCLCPP_WARN(this->get_logger(), "Drone %d goal is in obstacle or unknown space!", drone_id);
//           continue;
//         }
//       }

//       // Run A* search to obtain the path as grid indices.
//       std::vector<int> path_indices = astarSearch(start, end);

//       std_msgs::msg::Bool result_msg;
//       if (!path_indices.empty()) {
//         // Convert grid indices to world coordinates.
//         std::vector<std::array<double,3>> world_path;
//         world_path.reserve(path_indices.size());
//         for (int idx : path_indices) {
//           int gx, gy, gz;
//           coordFromIndex(idx, gx, gy, gz);
//           world_path.push_back(gridToWorld(gx, gy, gz));
//         }

//         // Smooth the path using the Catmull-Rom spline method.
//         std::vector<std::array<double,3>> smoothed_path = smoothPathCatmullRom(world_path, 10);

//         // Publish and save the smoothed path.
//         publishPath(drone_id, smoothed_path);

//         result_msg.data = true;
//         RCLCPP_INFO(this->get_logger(), 
//                     "A* path found & smoothed for Drone %d (raw length=%zu, smoothed=%zu)",
//                     drone_id, world_path.size(), smoothed_path.size());
//       } else {
//         result_msg.data = false;
//         RCLCPP_WARN(this->get_logger(), "No valid A* path for Drone %d", drone_id);
//       }

//       // Publish the path availability result.
//       path_result_publishers_[drone_id]->publish(result_msg);
//       // Optionally clear the goal to avoid repeated replanning.
//       drone_goals_.erase(drone_id);
//     }
//   }
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<AStarPathfinder3DNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
