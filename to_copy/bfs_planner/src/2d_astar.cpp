#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <fstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <limits>
#include <queue>
#include <cmath>
#include <memory>
#include <algorithm>

struct GridCell
{
  bool free;
};

class AStarPathfinder3DNode : public rclcpp::Node
{
public:
  AStarPathfinder3DNode()
  : Node("astar_pathfinder_3d_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("octomap_file", "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/city_1/meshes/city_1.binvox.bt");
    this->declare_parameter<std::string>("output_csv_folder", "/root/CrazySim/ros2_ws/src/icuas25_competition/startup/csv_save");
    this->declare_parameter<double>("grid_resolution", 0.5);
    this->declare_parameter<double>("z_min", 4.0);
    this->declare_parameter<double>("z_max", 25.0);
    this->declare_parameter<bool>("treat_unknown_as_free", true);

    // Retrieve
    octomap_file_       = this->get_parameter("octomap_file").as_string();
    output_csv_folder_  = this->get_parameter("output_csv_folder").as_string();
    grid_res_           = this->get_parameter("grid_resolution").as_double();
    z_min_              = this->get_parameter("z_min").as_double();
    z_max_              = this->get_parameter("z_max").as_double();
    treat_unknown_      = this->get_parameter("treat_unknown_as_free").as_bool();

    // Load OctoMap
    octree_ = std::make_shared<octomap::OcTree>(octomap_file_);
    if (!octree_ || octree_->size() == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load OctoMap or it is empty: %s", octomap_file_.c_str());
      rclcpp::shutdown();
      return;
    }

    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);

    // clamp z if needed
    if (z_min_ > min_z) min_z = z_min_;
    if (z_max_ < max_z) max_z = z_max_;

    RCLCPP_INFO(this->get_logger(),
                "Loaded octomap: %s. BBox => [%.2f,%.2f,%.2f]..[%.2f,%.2f,%.2f]",
                octomap_file_.c_str(), min_x, min_y, min_z, max_x, max_y, max_z);

    // Build 3D grid
    min_bbx_ = {min_x, min_y, min_z};
    max_bbx_ = {max_x, max_y, max_z};

    size_x_ = static_cast<int>(std::ceil((max_x - min_x) / grid_res_));
    size_y_ = static_cast<int>(std::ceil((max_y - min_y) / grid_res_));
    size_z_ = static_cast<int>(std::ceil((max_z - min_z) / grid_res_));

    int total_cells = size_x_ * size_y_ * size_z_;
    grid_.resize(total_cells);

    for (int ix = 0; ix < size_x_; ix++) {
      for (int iy = 0; iy < size_y_; iy++) {
        for (int iz = 0; iz < size_z_; iz++) {
          int idx = index3D(ix, iy, iz);
          auto w = gridToWorld(ix, iy, iz);
          octomap::OcTreeNode* node = octree_->search(w[0], w[1], w[2]);
          if (node) {
            // known
            if (octree_->isNodeOccupied(node)) 
              grid_[idx].free = false;
            else
              grid_[idx].free = true;
          } else {
            // unknown
            grid_[idx].free = treat_unknown_;
          }
        }
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "3D grid built: %dx%dx%d, res=%.2f, unknown=free? %d",
                size_x_, size_y_, size_z_, grid_res_, (int)treat_unknown_);

    // For each drone 1..5, subscribe to odom and goal
    for (int i = 1; i <= 5; i++) {
      // Odom
      {
        std::string odom_topic = "/cf_" + std::to_string(i) + "/odom";
        auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
          odom_topic, 10,
          [this, i](nav_msgs::msg::Odometry::SharedPtr msg){
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;
            double z = msg->pose.pose.position.z;
            drone_poses_[i] = {x, y, z};
          }
        );
        subs_odom_.push_back(sub);
      }
      // Goal
      {
        std::string goal_topic = "/drone" + std::to_string(i) + "_goal";
        auto subg = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_topic, 10,
          [this, i](geometry_msgs::msg::PoseStamped::SharedPtr msg){
            this->goalCallback(msg, i);
          }
        );
        subs_goal_.push_back(subg);
      }
      // Path result publisher
      {
        std::string res_topic = "/uav_" + std::to_string(i) + "_path_result";
        path_result_pubs_[i] = this->create_publisher<std_msgs::msg::Bool>(res_topic, 10);
      }
    }

    RCLCPP_INFO(this->get_logger(), "AStarPathfinder3DNode ready.");
  }

private:
  // ---------------------------
  // Member variables
  // ---------------------------
  std::string octomap_file_;
  std::string output_csv_folder_;
  double grid_res_;
  double z_min_, z_max_;
  bool treat_unknown_;

  std::shared_ptr<octomap::OcTree> octree_;
  std::array<double,3> min_bbx_;
  std::array<double,3> max_bbx_;

  int size_x_, size_y_, size_z_;
  std::vector<GridCell> grid_;

  // drone => current (x,y,z)
  std::unordered_map<int, std::array<double,3>> drone_poses_;

  // subs & pubs
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subs_odom_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> subs_goal_;
  std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> path_result_pubs_;

  // ---------------------------
  // Indexing & free checks
  // ---------------------------
  int index3D(int ix, int iy, int iz) const {
    return ix + size_x_ * (iy + size_y_ * iz);
  }

  std::array<double,3> gridToWorld(int ix, int iy, int iz) const
  {
    double wx = min_bbx_[0] + (ix + 0.5)*grid_res_;
    double wy = min_bbx_[1] + (iy + 0.5)*grid_res_;
    double wz = min_bbx_[2] + (iz + 0.5)*grid_res_;
    return {wx, wy, wz};
  }

  std::tuple<int,int,int> worldToGrid(double wx, double wy, double wz) const
  {
    int ix = (int)std::floor((wx - min_bbx_[0]) / grid_res_);
    int iy = (int)std::floor((wy - min_bbx_[1]) / grid_res_);
    int iz = (int)std::floor((wz - min_bbx_[2]) / grid_res_);
    return {ix, iy, iz};
  }

  bool inBounds(int ix, int iy, int iz) const {
    return (ix >= 0 && ix < size_x_ &&
            iy >= 0 && iy < size_y_ &&
            iz >= 0 && iz < size_z_);
  }

  bool isFree(int ix, int iy, int iz) const {
    if (!inBounds(ix, iy, iz)) return false;
    int idx = index3D(ix, iy, iz);
    return grid_[idx].free;
  }

  void idxToCoord(int idx, int &x, int &y, int &z) const
  {
    x = idx % size_x_;
    int rem = idx / size_x_;
    y = rem % size_y_;
    z = rem / size_y_;
  }

  // ---------------------------
  // Goal callback => run A*
  // ---------------------------
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int drone_id)
  {
    // If no odom => fail
    if (drone_poses_.find(drone_id) == drone_poses_.end()) {
      RCLCPP_WARN(this->get_logger(), "Drone %d: no odom => cannot plan path", drone_id);
      publishPathResult(drone_id, false);
      return;
    }

    auto start = drone_poses_[drone_id];
    auto goal = std::array<double,3>{ msg->pose.position.x,
                                      msg->pose.position.y,
                                      msg->pose.position.z };

    auto [sx, sy, sz] = worldToGrid(start[0], start[1], start[2]);
    auto [gx, gy, gz] = worldToGrid(goal[0],  goal[1],  goal[2]);

    // Basic checks
    if (!inBounds(sx, sy, sz) || !inBounds(gx, gy, gz)) {
      RCLCPP_WARN(this->get_logger(), "Drone %d: start/goal OOB => fail", drone_id);
      publishPathResult(drone_id, false);
      return;
    }
    if (!isFree(sx, sy, sz) || !isFree(gx, gy, gz)) {
      RCLCPP_WARN(this->get_logger(), "Drone %d: start/goal in obstacle => fail", drone_id);
      publishPathResult(drone_id, false);
      return;
    }

    // A*
    auto raw_path = runAStar(sx, sy, sz, gx, gy, gz);
    if (raw_path.empty()) {
      RCLCPP_WARN(this->get_logger(), "Drone %d: A* no path => fail", drone_id);
      publishPathResult(drone_id, false);
      return;
    }

    // Smoothing
    auto smoothed = smoothPathIndices(raw_path);

    // Save CSV
    writePathCSV(drone_id, smoothed);

    // Publish success
    publishPathResult(drone_id, true);

    RCLCPP_INFO(this->get_logger(),
                "Drone %d: Path => raw=%zu, smoothed=%zu => CSV written.",
                drone_id, raw_path.size(), smoothed.size());
  }

  void publishPathResult(int drone_id, bool success)
  {
    std_msgs::msg::Bool msg;
    msg.data = success;
    path_result_pubs_[drone_id]->publish(msg);
  }

  // ---------------------------
  // A* search in 3D
  // ---------------------------
  std::vector<int> runAStar(int sx, int sy, int sz, int gx, int gy, int gz)
  {
    auto start_idx = index3D(sx, sy, sz);
    auto goal_idx  = index3D(gx, gy, gz);

    std::vector<double> gcost(grid_.size(), std::numeric_limits<double>::infinity());
    std::vector<double> fcost(grid_.size(), std::numeric_limits<double>::infinity());
    std::vector<int> came_from(grid_.size(), -1);

    auto heuristic = [&](int idx){
      int cx, cy, cz;
      idxToCoord(idx, cx, cy, cz);
      double dx = cx - gx;
      double dy = cy - gy;
      double dz = cz - gz;
      return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    auto cmp = [&](int lhs, int rhs){ return fcost[lhs] > fcost[rhs]; };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open_set(cmp);

    gcost[start_idx] = 0.0;
    fcost[start_idx] = heuristic(start_idx);
    open_set.push(start_idx);

    // 6-connected
    int neighbors[6][3] = {
      {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}
    };

    while(!open_set.empty()) {
      int current = open_set.top();
      open_set.pop();

      if (current == goal_idx) {
        // reconstruct
        std::vector<int> path;
        int p = goal_idx;
        while (p != -1) {
          path.push_back(p);
          p = came_from[p];
        }
        std::reverse(path.begin(), path.end());
        return path;
      }

      if (fcost[current] == std::numeric_limits<double>::infinity()) {
        continue; // stale
      }

      int cx, cy, cz;
      idxToCoord(current, cx, cy, cz);
      for (auto &off : neighbors) {
        int nx = cx + off[0];
        int ny = cy + off[1];
        int nz = cz + off[2];
        if (!inBounds(nx, ny, nz)) continue;
        if (!isFree(nx, ny, nz)) continue;

        int nidx = index3D(nx, ny, nz);
        double new_g = gcost[current] + 1.0;
        if (new_g < gcost[nidx]) {
          gcost[nidx] = new_g;
          came_from[nidx] = current;
          double f = new_g + heuristic(nidx);
          fcost[nidx] = f;
          open_set.push(nidx);
        }
      }
    }

    // no path
    return {};
  }

  // ---------------------------
  // Shortcut-based smoothing
  // ---------------------------
  std::vector<int> smoothPathIndices(const std::vector<int> &raw)
  {
    if (raw.size() < 2) {
      return raw;
    }
    std::vector<int> result;
    result.push_back(raw[0]);

    size_t i = 0;
    while (i < raw.size() - 1) {
      size_t furthest = raw.size()-1;
      for (size_t j = raw.size()-1; j > i; j--) {
        if (checkLOS(raw[i], raw[j])) {
          furthest = j;
          break;
        }
      }
      i = furthest;
      result.push_back(raw[furthest]);
    }
    return result;
  }

  bool checkLOS(int idxA, int idxB)
  {
    int ax, ay, az; idxToCoord(idxA, ax, ay, az);
    auto wA = gridToWorld(ax, ay, az);
    int bx, by, bz; idxToCoord(idxB, bx, by, bz);
    auto wB = gridToWorld(bx, by, bz);

    double dx = wB[0] - wA[0];
    double dy = wB[1] - wA[1];
    double dz = wB[2] - wA[2];
    double length = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (length < 1e-6) {
      return true;
    }

    double step = grid_res_*0.5;
    int steps = (int)std::ceil(length / step);

    for (int i = 0; i <= steps; i++){
      double t = (double)i / (double)steps;
      double px = wA[0] + t*dx;
      double py = wA[1] + t*dy;
      double pz = wA[2] + t*dz;
      auto [gx, gy, gz] = worldToGrid(px, py, pz);
      if (!inBounds(gx, gy, gz) || !isFree(gx, gy, gz)) {
        return false;
      }
    }
    return true;
  }

  // ---------------------------
  // Write path to CSV
  // ---------------------------
  void writePathCSV(int drone_id, const std::vector<int> &path_indices)
  {
    if (path_indices.empty()) return;
    std::string filename = output_csv_folder_ + "/drone_" + std::to_string(drone_id) + "_waypoints.csv";
    std::ofstream fout(filename);
    if(!fout.is_open()) {
      RCLCPP_ERROR(this->get_logger(),"Cannot open CSV: %s", filename.c_str());
      return;
    }
    fout << "x,y,z\n";
    for (auto idx : path_indices) {
      int ix, iy, iz;
      idxToCoord(idx, ix, iy, iz);
      auto w = gridToWorld(ix, iy, iz);
      fout << w[0] << "," << w[1] << "," << w[2] << "\n";
    }
    fout.close();
    RCLCPP_INFO(this->get_logger(),"Drone %d: wrote CSV => %s", drone_id, filename.c_str());
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AStarPathfinder3DNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}




// #include <rclcpp/rclcpp.hpp>
// #include <octomap/octomap.h>
// #include <octomap/OcTree.h>

// #include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <nav_msgs/msg/path.hpp>
// #include <std_msgs/msg/bool.hpp>

// #include <vector>
// #include <string>
// #include <unordered_map>
// #include <limits>
// #include <queue>
// #include <cmath>
// #include <memory>
// #include <algorithm>

// struct GridCell
// {
//   bool free;
// };

// class AStarPathfinder3DNode : public rclcpp::Node
// {
// public:
//   AStarPathfinder3DNode()
//   : Node("astar_pathfinder_3d_node")
//   {
//     // -----------------------------
//     // Parameters
//     // -----------------------------
//     this->declare_parameter<std::string>("octomap_file", "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/city_1/meshes/city_1.binvox.bt");
//     this->declare_parameter<double>("grid_resolution", 0.5);
//     this->declare_parameter<double>("z_min", 0.0);
//     this->declare_parameter<double>("z_max", 20.0);
//     this->declare_parameter<bool>("treat_unknown_as_free", true);

//     // Retrieve
//     octomap_file_   = this->get_parameter("octomap_file").as_string();
//     grid_res_       = this->get_parameter("grid_resolution").as_double();
//     z_min_          = this->get_parameter("z_min").as_double();
//     z_max_          = this->get_parameter("z_max").as_double();
//     treat_unknown_  = this->get_parameter("treat_unknown_as_free").as_bool();

//     // -----------------------------
//     // Load OctoMap
//     // -----------------------------
//     octree_ = std::make_shared<octomap::OcTree>(octomap_file_);
//     if (!octree_ || octree_->size() == 0)
//     {
//       RCLCPP_ERROR(this->get_logger(), 
//                    "Failed to load OctoMap or it is empty: %s", 
//                    octomap_file_.c_str());
//       rclcpp::shutdown();
//       return;
//     }

//     // Get bounding box
//     double min_x, min_y, min_z;
//     double max_x, max_y, max_z;
//     octree_->getMetricMin(min_x, min_y, min_z);
//     octree_->getMetricMax(max_x, max_y, max_z);

//     // Optionally clamp z
//     if (z_min_ > min_z) min_z = z_min_;
//     if (z_max_ < max_z) max_z = z_max_;

//     RCLCPP_INFO(this->get_logger(),
//                 "Loaded octomap: %s. Map bbox => [%.2f,%.2f,%.2f]..[%.2f,%.2f,%.2f]",
//                 octomap_file_.c_str(), min_x, min_y, min_z, max_x, max_y, max_z);

//     // -----------------------------
//     // Build a 3D grid from octomap
//     //   index: x + sizeX*(y + sizeY*z)
//     // -----------------------------
//     min_bbx_ = {min_x, min_y, min_z};
//     max_bbx_ = {max_x, max_y, max_z};

//     size_x_ = static_cast<int>(std::ceil((max_x - min_x) / grid_res_));
//     size_y_ = static_cast<int>(std::ceil((max_y - min_y) / grid_res_));
//     size_z_ = static_cast<int>(std::ceil((max_z - min_z) / grid_res_));

//     int total_cells = size_x_ * size_y_ * size_z_;
//     grid_.resize(total_cells);

//     // Fill in free/occupied
//     for (int ix = 0; ix < size_x_; ix++)
//     {
//       for (int iy = 0; iy < size_y_; iy++)
//       {
//         for (int iz = 0; iz < size_z_; iz++)
//         {
//           int idx = index3D(ix, iy, iz);
//           auto world = gridToWorld(ix, iy, iz);
//           // Check in octree
//           octomap::OcTreeNode* node = octree_->search(world[0], world[1], world[2]);
//           if (node)
//           {
//             // known cell => check occupancy
//             if (octree_->isNodeOccupied(node))
//               grid_[idx].free = false; // obstacle
//             else
//               grid_[idx].free = true;  // free
//           }
//           else
//           {
//             // unknown => treat as free or not
//             grid_[idx].free = treat_unknown_;
//           }
//         }
//       }
//     }
//     RCLCPP_INFO(this->get_logger(),
//                 "Built 3D grid: %dx%dx%d with resolution=%.2f, unknown=free?=%d",
//                 size_x_, size_y_, size_z_, grid_res_, (int)treat_unknown_);

//     // -----------------------------
//     // Create subs/pubs for each drone
//     // -----------------------------
//     for (int i = 1; i <= 5; i++)
//     {
//       // Pose subscription
//       std::string odom_topic = "/cf_" + std::to_string(i) + "/odom";
//       auto sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
//         odom_topic, 10,
//         [this, i](nav_msgs::msg::Odometry::SharedPtr msg){
//           double x = msg->pose.pose.position.x;
//           double y = msg->pose.pose.position.y;
//           double z = msg->pose.pose.position.z;
//           drone_poses_[i] = {x,y,z};
//         }
//       );
//       subs_odom_.push_back(sub_odom);

//       // Goal subscription
//       std::string goal_topic = "/drone" + std::to_string(i) + "_goal";
//       auto sub_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//         goal_topic, 10,
//         [this, i](geometry_msgs::msg::PoseStamped::SharedPtr msg){
//           this->goalCallback(msg, i);
//         }
//       );
//       subs_goal_.push_back(sub_goal);

//       // Path publisher: nav_msgs/Path
//       std::string path_topic = "/uav_" + std::to_string(i) + "_path";
//       auto pub_path = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
//       path_pubs_[i] = pub_path;

//       // Path result publisher: std_msgs/Bool
//       std::string result_topic = "/uav_" + std::to_string(i) + "_path_result";
//       auto pub_result = this->create_publisher<std_msgs::msg::Bool>(result_topic, 10);
//       path_result_pubs_[i] = pub_result;
//     }

//     // Done
//     RCLCPP_INFO(this->get_logger(), "AStarPathfinder3DNode is ready.");
//   }

// private:
//   // ------------------------------------------------------
//   // Parameters
//   // ------------------------------------------------------
//   std::string octomap_file_;
//   double grid_res_;
//   double z_min_, z_max_;
//   bool treat_unknown_;

//   // ------------------------------------------------------
//   // OctoMap + 3D grid
//   // ------------------------------------------------------
//   std::shared_ptr<octomap::OcTree> octree_;
//   std::array<double,3> min_bbx_;
//   std::array<double,3> max_bbx_;

//   int size_x_, size_y_, size_z_;
//   std::vector<GridCell> grid_; // each cell => free or occupied

//   // index -> cell
//   inline int index3D(int ix, int iy, int iz) const
//   {
//     return ix + size_x_ * (iy + size_y_ * iz);
//   }

//   // cell -> world
//   std::array<double,3> gridToWorld(int ix, int iy, int iz) const
//   {
//     double wx = min_bbx_[0] + (ix + 0.5) * grid_res_;
//     double wy = min_bbx_[1] + (iy + 0.5) * grid_res_;
//     double wz = min_bbx_[2] + (iz + 0.5) * grid_res_;
//     return {wx, wy, wz};
//   }

//   // world -> cell
//   std::tuple<int,int,int> worldToGrid(double wx, double wy, double wz) const
//   {
//     int ix = static_cast<int>(std::floor((wx - min_bbx_[0]) / grid_res_));
//     int iy = static_cast<int>(std::floor((wy - min_bbx_[1]) / grid_res_));
//     int iz = static_cast<int>(std::floor((wz - min_bbx_[2]) / grid_res_));
//     return std::make_tuple(ix, iy, iz);
//   }

//   // Check if inside grid
//   bool inBounds(int ix, int iy, int iz) const
//   {
//     return (ix >= 0 && ix < size_x_ &&
//             iy >= 0 && iy < size_y_ &&
//             iz >= 0 && iz < size_z_);
//   }

//   // Occupied check
//   bool isFree(int ix, int iy, int iz) const
//   {
//     if(!inBounds(ix, iy, iz)) return false;
//     int idx = index3D(ix, iy, iz);
//     return grid_[idx].free;
//   }

//   // ------------------------------------------------------
//   // Data for 5 drones
//   // ------------------------------------------------------
//   std::unordered_map<int, std::array<double,3>> drone_poses_; // current positions

//   // Pubs and subs
//   std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subs_odom_;
//   std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> subs_goal_;

//   std::unordered_map<int, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs_;
//   std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> path_result_pubs_;

//   // ------------------------------------------------------
//   // goalCallback => run A*
//   // ------------------------------------------------------
//   void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int drone_id)
//   {
//     // Start = drone's current pose
//     if (drone_poses_.find(drone_id) == drone_poses_.end()) {
//       RCLCPP_WARN(this->get_logger(),
//                   "Drone %d: no odom yet => cannot plan path!", drone_id);
//       // publish path_result = false
//       std_msgs::msg::Bool result_msg;
//       result_msg.data = false;
//       path_result_pubs_[drone_id]->publish(result_msg);
//       return;
//     }
//     auto start = drone_poses_[drone_id];
//     auto goal = std::array<double,3>{ msg->pose.position.x,
//                                       msg->pose.position.y,
//                                       msg->pose.position.z };

//     // Convert start & goal to grid coords
//     auto [sx, sy, sz] = worldToGrid(start[0], start[1], start[2]);
//     auto [gx, gy, gz] = worldToGrid(goal[0],  goal[1],  goal[2]);

//     // Basic checks
//     if (!inBounds(sx, sy, sz) || !inBounds(gx, gy, gz))
//     {
//       RCLCPP_WARN(this->get_logger(),
//                   "Drone %d: start or goal out of bounds => fail path!", 
//                   drone_id);
//       publishPathFail(drone_id);
//       return;
//     }
//     if (!isFree(sx, sy, sz) || !isFree(gx, gy, gz))
//     {
//       RCLCPP_WARN(this->get_logger(),
//                   "Drone %d: start or goal is in obstacle => fail path!", 
//                   drone_id);
//       publishPathFail(drone_id);
//       return;
//     }

//     // Run A*
//     auto path_indices = runAStar(sx, sy, sz, gx, gy, gz);
//     if (path_indices.empty()) {
//       RCLCPP_WARN(this->get_logger(),
//                   "Drone %d: no path found by A*! => fail path.",
//                   drone_id);
//       publishPathFail(drone_id);
//       return;
//     }

//     // Convert to nav_msgs/Path
//     nav_msgs::msg::Path path_msg;
//     path_msg.header.stamp = this->now();
//     path_msg.header.frame_id = "world";
//     path_msg.poses.reserve(path_indices.size());
//     for (auto idx : path_indices) {
//       int ix, iy, iz;
//       idxToCoord(idx, ix, iy, iz);
//       auto w = gridToWorld(ix, iy, iz);
//       geometry_msgs::msg::PoseStamped ps;
//       ps.header = path_msg.header;
//       ps.pose.position.x = w[0];
//       ps.pose.position.y = w[1];
//       ps.pose.position.z = w[2];
//       ps.pose.orientation.w = 1.0;
//       path_msg.poses.push_back(ps);
//     }

//     path_pubs_[drone_id]->publish(path_msg);

//     // publish success
//     std_msgs::msg::Bool result_msg;
//     result_msg.data = true;
//     path_result_pubs_[drone_id]->publish(result_msg);

//     RCLCPP_INFO(this->get_logger(),
//                 "Drone %d: A* path found with %zu waypoints.",
//                 drone_id, path_indices.size());
//   }

//   void publishPathFail(int drone_id)
//   {
//     std_msgs::msg::Bool result_msg;
//     result_msg.data = false;
//     path_result_pubs_[drone_id]->publish(result_msg);
//   }

//   // helper: convert linear index back to (ix,iy,iz)
//   void idxToCoord(int idx, int &x, int &y, int &z) const
//   {
//     x = idx % size_x_;
//     int rem = idx / size_x_;
//     y = rem % size_y_;
//     z = rem / size_y_;
//   }

//   // ------------------------------------------------------
//   // runAStar => standard 3D A*
//   // neighbors: +/-1 in x,y,z (6, 26, or 27 directions)
//   // here we do 6 or 26. We'll do 6 for simplicity.
//   // ------------------------------------------------------
//   std::vector<int> runAStar(int sx, int sy, int sz, 
//                             int gx, int gy, int gz)
//   {
//     auto start_idx = index3D(sx, sy, sz);
//     auto goal_idx  = index3D(gx, gy, gz);

//     // standard containers
//     std::vector<double> gcost(grid_.size(), std::numeric_limits<double>::infinity());
//     std::vector<double> fcost(grid_.size(), std::numeric_limits<double>::infinity());
//     std::vector<int> came_from(grid_.size(), -1);

//     auto heuristic = [this, gx, gy, gz](int idx){
//       int cx, cy, cz;
//       idxToCoord(idx, cx, cy, cz);
//       double dx = cx - gx;
//       double dy = cy - gy;
//       double dz = cz - gz;
//       return std::sqrt(dx*dx + dy*dy + dz*dz);
//     };

//     auto cmp = [&](int lhs, int rhs){
//       return fcost[lhs] > fcost[rhs];
//     };

//     std::priority_queue<int, std::vector<int>, decltype(cmp)> open_set(cmp);

//     gcost[start_idx] = 0.0;
//     fcost[start_idx] = heuristic(start_idx);
//     open_set.push(start_idx);

//     const int neighbor_offsets[6][3] = {
//       {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
//     };
//     // if you want 26 neighbors, expand this array.

//     while (!open_set.empty())
//     {
//       int current = open_set.top();
//       open_set.pop();

//       if (current == goal_idx)
//       {
//         // reconstruct path
//         std::vector<int> path;
//         int p = goal_idx;
//         while (p != -1) {
//           path.push_back(p);
//           p = came_from[p];
//         }
//         std::reverse(path.begin(), path.end());
//         return path;
//       }

//       // skip stale
//       if (fcost[current] == std::numeric_limits<double>::infinity()) {
//         // means we popped from queue after we've discovered a better path
//         continue;
//       }

//       int cx, cy, cz;
//       idxToCoord(current, cx, cy, cz);

//       for (auto &offset : neighbor_offsets) {
//         int nx = cx + offset[0];
//         int ny = cy + offset[1];
//         int nz = cz + offset[2];
//         if (!inBounds(nx, ny, nz)) continue;
//         if (!isFree(nx, ny, nz)) continue;  // obstacle skip

//         int nidx = index3D(nx, ny, nz);
//         double cost = gcost[current] + 1.0; // cost = 1 per step or sqrt( dx^2 + dy^2 + dz^2 ) if you prefer
//         if (cost < gcost[nidx]) {
//           came_from[nidx] = current;
//           gcost[nidx] = cost;
//           double f = cost + heuristic(nidx);
//           fcost[nidx] = f;
//           open_set.push(nidx);
//         }
//       }
//     }

//     // no path
//     return {};
//   }
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<AStarPathfinder3DNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
