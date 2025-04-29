#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
 
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
 
#include <vector>
#include <string>
#include <unordered_map>
#include <limits>
#include <queue>
#include <cmath>
#include <memory>
#include <algorithm>
 
class AStar2DInOctoMapSlice : public rclcpp::Node
{
public:
  AStar2DInOctoMapSlice()
  : Node("astar_2d_in_octomap_slice")
  {
    // params
    this->declare_parameter<std::string>(
      "octomap_file",
      "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/city_1/meshes/city_1.binvox.bt"
    );
    this->declare_parameter<double>("z_slice", 1.0);
    this->declare_parameter<double>("grid_resolution", 0.5);
    this->declare_parameter<double>("z_clearance", 10.0);
    // NEW: inflation parameter to make obstacles appear bigger
    this->declare_parameter<double>("inflation_distance", 1.2);
 
    octomap_file_ = this->get_parameter("octomap_file").as_string();
    z_slice_      = this->get_parameter("z_slice").as_double();
    grid_res_     = this->get_parameter("grid_resolution").as_double();
    z_clearance_  = this->get_parameter("z_clearance").as_double();
    inflation_distance_ = this->get_parameter("inflation_distance").as_double();
 
    // load octomap
    octree_ = std::make_shared<octomap::OcTree>(octomap_file_);
    if(!octree_ || octree_->size()==0){
      RCLCPP_ERROR(this->get_logger(),"Failed to load OctoMap: %s", octomap_file_.c_str());
      rclcpp::shutdown();
      return;
    }
 
    double minx, miny, minz;
    double maxx, maxy, maxz;
    octree_->getMetricMin(minx, miny, minz);
    octree_->getMetricMax(maxx, maxy, maxz);
 
    RCLCPP_INFO(this->get_logger(),
      "Loaded octomap => bounding box (%.2f..%.2f, %.2f..%.2f, %.2f..%.2f)",
      minx, maxx, miny, maxy, minz, maxz
    );
 
    // Clamp the slice to be within the octomap bounds.
    if(z_slice_ < minz) z_slice_ = minz;
    if(z_slice_ > maxz) z_slice_ = maxz;
 
    min_x_ = minx; 
    min_y_ = miny;
    max_x_ = maxx; 
    max_y_ = maxy;
    size_x_ = static_cast<int>(std::ceil((max_x_ - min_x_) / grid_res_));
    size_y_ = static_cast<int>(std::ceil((max_y_ - min_y_) / grid_res_));
    grid_.resize(size_x_ * size_y_, 1); // 1 = free, 0 = occupied
 
    // Fill the 2D grid considering a vertical band around z_slice_.
    for (int iy = 0; iy < size_y_; iy++) {
      for (int ix = 0; ix < size_x_; ix++) {
        double wx = min_x_ + (ix + 0.5) * grid_res_;
        double wy = min_y_ + (iy + 0.5) * grid_res_;
        // If any voxel in [z_slice_ - z_clearance_, z_slice_ + z_clearance_] is occupied,
        // treat the cell as occupied.
        if (!isFreeDilated(wx, wy, z_slice_, z_clearance_)) {
          grid_[index2d(ix, iy)] = 0;
        }
      }
    }
 
    RCLCPP_INFO(this->get_logger(),
      "2D grid at z=%.2f ± %.2f => size=(%d,%d), res=%.2f", 
      z_slice_, z_clearance_, size_x_, size_y_, grid_res_
    );
 
    // NEW: Obstacle inflation to increase safety margin
    inflateObstacles();
 
    // For each drone, set up subscriptions and publishers.
    for (int i = 1; i <= 5; i++) {
      // odom subscription
      std::string odom_topic = "/cf_" + std::to_string(i) + "/odom";
      odom_subs_.push_back(
        this->create_subscription<nav_msgs::msg::Odometry>(
          odom_topic, 10,
          [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;
            drone_positions_[i] = {x, y};
          }
        )
      );
      // goal subscription
      std::string goal_topic = "/drone" + std::to_string(i) + "_goal";
      goal_subs_.push_back(
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_topic, 10,
          [this, i](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            double gx = msg->pose.position.x;
            double gy = msg->pose.position.y;
            goals_[i] = {gx, gy};
            RCLCPP_INFO(this->get_logger(),
              "Drone %d => new goal => (%.2f, %.2f)", i, gx, gy);
          }
        )
      );
      // path publisher
      std::string path_topic = "/uav_" + std::to_string(i) + "_path";
      path_pubs_[i] = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
 
      // path result publisher
      std::string result_topic = "/uav_" + std::to_string(i) + "_path_result";
      path_result_pubs_[i] = this->create_publisher<std_msgs::msg::Bool>(result_topic, 10);
    }
 
    // Timer for running A* and publishing results.
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&AStar2DInOctoMapSlice::timerCallback, this)
    );
 
    RCLCPP_INFO(this->get_logger(), "AStar2DInOctoMapSlice node started.");
  }
 
private:
  // Parameters.
  std::string octomap_file_;
  double z_slice_;
  double grid_res_;
  double z_clearance_;
  double inflation_distance_;  // NEW: extra clearance around obstacles
 
  // Bounding box.
  double min_x_, min_y_;
  double max_x_, max_y_;
  int size_x_, size_y_;
 
  // 2D grid representation: 1 = free, 0 = obstacle.
  std::vector<int> grid_;
 
  // Octomap.
  std::shared_ptr<octomap::OcTree> octree_;
 
  // Drone positions and goals.
  std::unordered_map<int, std::array<double, 2>> drone_positions_;
  std::unordered_map<int, std::array<double, 2>> goals_;
 
  // Publishers and subscribers.
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> goal_subs_;
  std::unordered_map<int, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs_;
  std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> path_result_pubs_;
 
  rclcpp::TimerBase::SharedPtr timer_;
 
  // ----------------------------------------------------
  // Check the vertical band around z_slice.
  // ----------------------------------------------------
  bool isFreeDilated(double wx, double wy, double z_center, double z_clear) 
  {
    double minZ = z_center - z_clear;
    double maxZ = z_center + z_clear;
    static const int STEPS = 5; // number of samples along z
    for (int i = 0; i < STEPS; i++) {
      double alpha = static_cast<double>(i) / static_cast<double>(STEPS - 1);
      double z = minZ + alpha * (maxZ - minZ);
      auto node = octree_->search(wx, wy, z);
      if (node && octree_->isNodeOccupied(node)) {
        // If any voxel in the vertical band is occupied, mark as obstacle.
        return false;
      }
    }
    return true;
  }
 
  inline int index2d(int ix, int iy) const {
    return ix + iy * size_x_;
  }
 
  bool inBounds(int ix, int iy) const {
    return (ix >= 0 && ix < size_x_ && iy >= 0 && iy < size_y_);
  }
 
  bool isFreeCell(int ix, int iy) const {
    if (!inBounds(ix, iy)) return false;
    return (grid_[index2d(ix, iy)] == 1);
  }
 
  std::array<int, 2> worldToGrid(double wx, double wy) const {
    int gx = static_cast<int>(std::floor((wx - min_x_) / grid_res_));
    int gy = static_cast<int>(std::floor((wy - min_y_) / grid_res_));
    return {gx, gy};
  }
 
  std::array<double, 2> gridToWorld(int ix, int iy) const {
    double wx = min_x_ + (ix + 0.5) * grid_res_;
    double wy = min_y_ + (iy + 0.5) * grid_res_;
    return {wx, wy};
  }
 
  // ----------------------------------------------------
  // Inflate obstacles in the grid.
  // ----------------------------------------------------
  void inflateObstacles() {
    // Convert the inflation distance to grid cells.
    int inflation_radius = static_cast<int>(std::ceil(inflation_distance_ / grid_res_));
    RCLCPP_INFO(this->get_logger(), "Inflating obstacles with radius (cells): %d", inflation_radius);
 
    // Create a copy of the original grid.
    std::vector<int> inflated_grid = grid_;
 
    // For each cell that is an obstacle, mark neighboring cells within the radius as obstacles.
    for (int iy = 0; iy < size_y_; iy++) {
      for (int ix = 0; ix < size_x_; ix++) {
        if (grid_[index2d(ix, iy)] == 0) {
          for (int dy = -inflation_radius; dy <= inflation_radius; dy++) {
            for (int dx = -inflation_radius; dx <= inflation_radius; dx++) {
              int nx = ix + dx;
              int ny = iy + dy;
              if (inBounds(nx, ny)) {
                // Optionally, you can use a circular check to only inflate in a circle.
                // Here, we simply use a square neighborhood.
                inflated_grid[index2d(nx, ny)] = 0;
              }
            }
          }
        }
      }
    }
    // Replace the original grid with the inflated grid.
    grid_ = inflated_grid;
  }
 
  // ----------------------------------------------------
  // Timer callback: For each drone, if a goal exists, run A*.
  // ----------------------------------------------------
  void timerCallback() {
    for (int i = 1; i <= 5; i++) {
      // Ensure we have both a goal and current position.
      if (goals_.find(i) == goals_.end()) continue;
      if (drone_positions_.find(i) == drone_positions_.end()) continue;
 
      double sx = drone_positions_[i][0];
      double sy = drone_positions_[i][1];
      double gx = goals_[i][0];
      double gy = goals_[i][1];
 
      auto [six, siy] = worldToGrid(sx, sy);
      auto [gix, giy] = worldToGrid(gx, gy);
 
      if (!inBounds(six, siy) || !inBounds(gix, giy)) {
        RCLCPP_WARN(this->get_logger(), "Drone %d => start/goal out of bounds => fail path!", i);
        publishResult(i, false);
        goals_.erase(i);
        continue;
      }
      if (!isFreeCell(six, siy) || !isFreeCell(gix, giy)) {
        RCLCPP_WARN(this->get_logger(), "Drone %d => start/goal in obstacle => fail path!", i);
        publishResult(i, false);
        goals_.erase(i);
        continue;
      }
      auto path_cells = runAStar(six, siy, gix, giy);
      if (path_cells.empty()) {
        RCLCPP_WARN(this->get_logger(), "Drone %d => no path found => fail", i);
        publishResult(i, false);
        goals_.erase(i);
        continue;
      }
      // Convert grid path to world coordinates.
      auto path_world = convertPathToWorld(path_cells);
      publishPath(i, path_world);
      RCLCPP_INFO(this->get_logger(), "Drone %d => found path => length=%zu", i, path_world.size());
      publishResult(i, true);
 
      // Remove the goal so that we don't repeatedly re-plan.
      goals_.erase(i);
    }
  }
 
  void publishResult(int drone_id, bool ok) {
    std_msgs::msg::Bool bmsg;
    bmsg.data = ok;
    path_result_pubs_[drone_id]->publish(bmsg);
  }
 
  // ----------------------------------------------------
  // A* algorithm in 2D grid.
  // ----------------------------------------------------
  std::vector<std::array<int, 2>> runAStar(int sx, int sy, int gx, int gy) {
    int idxS = index2d(sx, sy);
    int idxG = index2d(gx, gy);
 
    std::vector<double> gscore(size_x_ * size_y_, std::numeric_limits<double>::infinity());
    std::vector<double> fscore(size_x_ * size_y_, std::numeric_limits<double>::infinity());
    std::vector<int> cameFrom(size_x_ * size_y_, -1);
 
    auto heuristic = [&](int idx) {
      int ix = idx % size_x_;
      int iy = idx / size_x_;
      double dx = ix - gx;
      double dy = iy - gy;
      return std::fabs(dx) + std::fabs(dy);  // Manhattan distance
    };
 
    auto cmp = [&](int lhs, int rhs) {
      return fscore[lhs] > fscore[rhs];
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> openSet(cmp);
 
    gscore[idxS] = 0.0;
    fscore[idxS] = heuristic(idxS);
    openSet.push(idxS);
 
    static const int dirs[8][2] = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };
 
    while (!openSet.empty()) {
      int current = openSet.top();
      openSet.pop();
      if (current == idxG) {
        // Reconstruct the path.
        std::vector<std::array<int, 2>> path;
        int c = idxG;
        while (c != -1) {
          int cx = c % size_x_;
          int cy = c / size_x_;
          path.push_back({cx, cy});
          c = cameFrom[c];
        }
        std::reverse(path.begin(), path.end());
        return path;
      }
      if (fscore[current] == std::numeric_limits<double>::infinity()) {
        // Skip stale entry.
        continue;
      }
      int cx = current % size_x_;
      int cy = current / size_x_;
      for (auto &d : dirs) {
        int nx = cx + d[0];
        int ny = cy + d[1];
        if (!inBounds(nx, ny)) continue;
        if (!isFreeCell(nx, ny)) continue;
        double step_cost = (std::abs(d[0]) + std::abs(d[1]) == 2) ? 1.4142 : 1.0;
        double cost = gscore[current] + step_cost;
        int nidx = index2d(nx, ny);
        if (cost < gscore[nidx]) {
          cameFrom[nidx] = current;
          gscore[nidx] = cost;
          fscore[nidx] = cost + std::fabs(nx - gx) + std::fabs(ny - gy);
          openSet.push(nidx);
        }
      }
    }
    return {};
  }
 
  std::vector<std::array<double, 2>> convertPathToWorld(
    const std::vector<std::array<int, 2>> &cells) 
  {
    std::vector<std::array<double, 2>> out;
    out.reserve(cells.size());
    for (auto &c : cells) {
      int ix = c[0];
      int iy = c[1];
      out.push_back(gridToWorld(ix, iy));
    }
    return out;
  }
 
  void publishPath(int drone_id, const std::vector<std::array<double, 2>> &path_world) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "world";
 
    for (auto &pt : path_world) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = pt[0];
      ps.pose.position.y = pt[1];
      ps.pose.position.z = z_slice_; // keep the z coordinate fixed
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }
    path_pubs_[drone_id]->publish(path_msg);
  }
};
 
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AStar2DInOctoMapSlice>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}