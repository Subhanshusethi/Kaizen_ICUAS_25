#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <string>
#include <vector>
#include <deque>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <memory>
#include <algorithm>
#include <optional>

class BFSConnectivity3DPlanner : public rclcpp::Node
{
public:
  BFSConnectivity3DPlanner()
  : Node("bfs_connectivity_3d_planner")
  {
    // -----------------------------------------------------
    // Declare Parameters
    // -----------------------------------------------------
    this->declare_parameter<std::string>("octomap_file", "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/city_1/meshes/city_1.binvox.bt");
    this->declare_parameter<double>("z_min", 4.0);
    this->declare_parameter<double>("z_max", 25.0);
    this->declare_parameter<double>("downsample_step", 10.0);
    this->declare_parameter<double>("prune_dist", 10.0);
    this->declare_parameter<double>("max_los_dist", 70.0);
    this->declare_parameter<double>("arrival_threshold", 1.5);
    this->declare_parameter<double>("base_x", 0.0);
    this->declare_parameter<double>("base_y", 0.0);
    this->declare_parameter<double>("base_z", 1.0);
    this->declare_parameter<double>("fixed_alt", 2.0);

    // Battery thresholds
    this->declare_parameter<double>("battery_return_threshold", 0.35);
    this->declare_parameter<double>("battery_resume_threshold", 0.90);

    // -----------------------------------------------------
    // Get Parameters
    // -----------------------------------------------------
    octomap_file_ = this->get_parameter("octomap_file").as_string();
    z_min_        = this->get_parameter("z_min").as_double();
    z_max_        = this->get_parameter("z_max").as_double();
    ds_step_      = this->get_parameter("downsample_step").as_double();
    prune_dist_   = this->get_parameter("prune_dist").as_double();
    max_los_dist_ = this->get_parameter("max_los_dist").as_double();
    arrival_threshold_ = this->get_parameter("arrival_threshold").as_double();
    base_x_       = this->get_parameter("base_x").as_double();
    base_y_       = this->get_parameter("base_y").as_double();
    base_z_       = this->get_parameter("base_z").as_double();
    fixed_alt_    = this->get_parameter("fixed_alt").as_double();

    battery_return_th_  = this->get_parameter("battery_return_threshold").as_double();
    battery_resume_th_  = this->get_parameter("battery_resume_threshold").as_double();

    // -----------------------------------------------------
    // Load OctoMap
    // -----------------------------------------------------
    octree_ = std::make_shared<octomap::OcTree>(octomap_file_);
    if (!octree_ || octree_->size() == 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to load OctoMap or it's empty. File: %s",
                   octomap_file_.c_str());
      rclcpp::shutdown();
      return;
    }
    double minX, minY, minZ, maxX, maxY, maxZ;
    octree_->getMetricMin(minX, minY, minZ);
    octree_->getMetricMax(maxX, maxY, maxZ);
    RCLCPP_INFO(this->get_logger(),
                "Loaded OctoMap from: %s. Bounding box: (%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f)",
                octomap_file_.c_str(), minX, minY, minZ, maxX, maxY, maxZ);

    // -----------------------------------------------------
    // Drone Data
    // -----------------------------------------------------
    for (int i = 1; i <= 5; i++) {
      path_results_[i] = true; 
      battery_states_[i] = 1.0;  
      drone_returning_[i] = false;
      move_queues_[i] = std::deque<std::pair<std::array<double,3>, bool>>();
    }

    // Base vantage => ID=0
    current_vantage_[0] = std::array<double,3>{base_x_, base_y_, base_z_};
    for (int i = 1; i <= 5; i++) {
      current_vantage_[i] = std::nullopt;
    }

    // -----------------------------------------------------
    // Subscribers for each drone
    // -----------------------------------------------------
    for (int i = 1; i <= 5; i++) {
      // Odom
      {
        std::string odom_topic = "/cf_" + std::to_string(i) + "/odom";
        odom_subs_.push_back(
          this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this, i](nav_msgs::msg::Odometry::SharedPtr msg){
              double x = msg->pose.pose.position.x;
              double y = msg->pose.pose.position.y;
              double z = msg->pose.pose.position.z;
              drone_poses_[i] = {x, y, z};
            }
          )
        );
      }
      // Battery
      {
        std::string batt_topic = "/cf_" + std::to_string(i) + "/battery_status";
        battery_subs_.push_back(
          this->create_subscription<sensor_msgs::msg::BatteryState>(
            batt_topic, 10,
            [this, i](sensor_msgs::msg::BatteryState::SharedPtr msg){
              double perc = msg->percentage; // 0..1
              battery_states_[i] = perc;
              checkAndHandleBattery(i, perc);
            }
          )
        );
      }
      // Path result
      {
        std::string pathres_topic = "/uav_" + std::to_string(i) + "_path_result";
        pathres_subs_.push_back(
          this->create_subscription<std_msgs::msg::Bool>(
            pathres_topic, 10,
            [this, i](std_msgs::msg::Bool::SharedPtr msg){
              path_results_[i] = msg->data;
            }
          )
        );
      }
    }

    // -----------------------------------------------------
    // Publishers
    // -----------------------------------------------------
    for (int i = 1; i <= 5; i++) {
      // Pose goal
      {
        std::string goal_topic = "/drone" + std::to_string(i) + "_goal";
        drone_goal_pubs_[i] = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
      }
      // Returning bool
      {
        std::string ret_topic = "/drone" + std::to_string(i) + "_returning";
        drone_return_pubs_[i] = this->create_publisher<std_msgs::msg::Bool>(ret_topic, 10);
      }
    }

    vantage_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("bfs_3d_vantage_markers", 10);

    // -----------------------------------------------------
    // Build vantage points & BFS adjacency
    // -----------------------------------------------------
    buildCandidateVantagePoints();
    buildAdjacency();
    computeBFSOrder();
    assignVantagePoints();
    publishVantageMarkers();

    // Timer for assignment logic
    assignment_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&BFSConnectivity3DPlanner::assignmentTimerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "BFSConnectivity3DPlanner initialized.");
  }

private:
  // ======= Parameters =======
  std::string octomap_file_;
  double z_min_, z_max_;
  double ds_step_;
  double prune_dist_;
  double max_los_dist_;
  double arrival_threshold_;
  double base_x_, base_y_, base_z_;
  double fixed_alt_;

  double battery_return_th_;
  double battery_resume_th_;

  // ======= OctoMap =======
  std::shared_ptr<octomap::OcTree> octree_;

  // ======= Drone States =======
  std::unordered_map<int, std::array<double,3>> drone_poses_;
  std::unordered_map<int, double> battery_states_;
  std::unordered_map<int, bool> drone_returning_;
  std::unordered_map<int, bool> path_results_;

  // vantage currently "occupied" by drone i (or base=0)
  std::unordered_map<int, std::optional<std::array<double,3>>> current_vantage_;

  // vantage queue for each drone
  std::unordered_map<int, std::deque<std::pair<std::array<double,3>, bool>>> move_queues_;

  // ======= Vantage data =======
  std::vector<std::array<double,3>> vantage_points_;
  std::vector<std::vector<int>> adjacency_;
  std::vector<int> bfs_order_; // BFS vantage indices

  // ======= Publishers/Subscribers =======
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> pathres_subs_;

  std::unordered_map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> drone_goal_pubs_;
  std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> drone_return_pubs_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vantage_marker_pub_;
  rclcpp::TimerBase::SharedPtr assignment_timer_;

  // ---------------------------
  // Battery logic
  // ---------------------------
  void checkAndHandleBattery(int drone_id, double perc)
  {
    // if not returning yet, but below threshold => go home
    if (!drone_returning_[drone_id] && (perc < battery_return_th_)) {
      drone_returning_[drone_id] = true;
      move_queues_[drone_id].clear();
      std_msgs::msg::Bool msg;
      msg.data = true;
      drone_return_pubs_[drone_id]->publish(msg);

      publishGoal(drone_id, {base_x_, base_y_, base_z_});
      RCLCPP_WARN(this->get_logger(),
                  "Drone %d battery=%.2f => returning home!",
                  drone_id, perc);
    }
    // if returning, but above resume => normal coverage
    else if (drone_returning_[drone_id] && (perc > battery_resume_th_)) {
      drone_returning_[drone_id] = false;
      std_msgs::msg::Bool msg;
      msg.data = false;
      drone_return_pubs_[drone_id]->publish(msg);

      RCLCPP_INFO(this->get_logger(),
                  "Drone %d battery=%.2f => rejoining coverage!",
                  drone_id, perc);
      // optionally re-insert vantage points
    }
  }

  // ---------------------------
  // Timer callback
  // ---------------------------
  void assignmentTimerCallback()
  {
    for (int drone_id = 1; drone_id <= 5; drone_id++) {
      // skip if returning
      if (drone_returning_[drone_id]) {
        continue;
      }

      auto &dq = move_queues_[drone_id];
      if (dq.empty()) {
        continue;
      }

      auto &front = dq.front();
      auto vantage = front.first;
      bool published = front.second;

      // path fail => skip vantage
      if (!path_results_[drone_id]) {
        RCLCPP_WARN(this->get_logger(),
                    "Drone %d path fail => skip vantage (%.2f, %.2f, %.2f)",
                    drone_id, vantage[0], vantage[1], vantage[2]);
        dq.pop_front();
        path_results_[drone_id] = true;
        continue;
      }

      // no odom => skip vantage
      if (drone_poses_.find(drone_id) == drone_poses_.end()) {
        dq.pop_front();
        continue;
      }

      // arrival check
      auto pose = drone_poses_[drone_id];
      double dx = pose[0] - vantage[0];
      double dy = pose[1] - vantage[1];
      double dz = pose[2] - vantage[2];
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (dist < arrival_threshold_) {
        RCLCPP_INFO(this->get_logger(),
                    "Drone %d arrived vantage => (%.2f, %.2f, %.2f)",
                    drone_id, vantage[0], vantage[1], vantage[2]);
        dq.pop_front();
        current_vantage_[drone_id] = vantage;
        continue;
      }

      // LOS check => skip vantage if not in LOS
      if (!hasLOSWithAnyOccupied(vantage, drone_id)) {
        RCLCPP_WARN(this->get_logger(),
                    "Drone %d vantage => (%.2f, %.2f, %.2f) not in LOS => skip",
                    drone_id, vantage[0], vantage[1], vantage[2]);
        dq.pop_front();
        continue;
      }

      // if not published => publish
      if (!published) {
        publishGoal(drone_id, vantage);
        front.second = true;
        RCLCPP_INFO(this->get_logger(),
                    "Drone %d assigned vantage => (%.2f, %.2f, %.2f).",
                    drone_id, vantage[0], vantage[1], vantage[2]);
      }
    }
  }

  // ---------------------------
  // Check LOS with base or any other occupied vantage
  // ---------------------------
  bool hasLOSWithAnyOccupied(const std::array<double,3> &v, int drone_id)
  {
    for (auto &kv : current_vantage_) {
      int other_id = kv.first;
      if (other_id == drone_id) continue;
      if (!kv.second.has_value()) continue;
      auto ov = kv.second.value();
      if (lineOfSight3D(v, ov)) {
        return true;
      }
    }
    return false;
  }

  // ---------------------------
  // Publish goal
  // ---------------------------
  void publishGoal(int drone_id, const std::array<double,3> &v)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->now();
    ps.header.frame_id = "world";
    ps.pose.position.x = v[0];
    ps.pose.position.y = v[1];
    // either fix the altitude or use the vantage's z
    ps.pose.position.z = (v[2] < 0.1) ? fixed_alt_ : v[2];
    ps.pose.orientation.w = 1.0;
    drone_goal_pubs_[drone_id]->publish(ps);
  }

  // ---------------------------
  // Build vantage points
  // ---------------------------
  void buildCandidateVantagePoints()
  {
    vantage_points_.push_back({base_x_, base_y_, base_z_}); // index=0 => base

    double minX, minY, minZ, maxX, maxY, maxZ;
    octree_->getMetricMin(minX, minY, minZ);
    octree_->getMetricMax(maxX, maxY, maxZ);

    // clamp z
    if (z_min_ > minZ) minZ = z_min_;
    if (z_max_ < maxZ) maxZ = z_max_;

    // naive sampling
    std::vector<std::array<double,3>> raw;
    for (double x = minX; x <= maxX; x += ds_step_) {
      for (double y = minY; y <= maxY; y += ds_step_) {
        for (double z = minZ; z <= maxZ; z += ds_step_) {
          if (isFree3D(x, y, z)) {
            raw.push_back({x, y, z});
          }
        }
      }
    }

    // prune
    std::vector<std::array<double,3>> kept;
    for (auto &pt : raw) {
      bool too_close = false;
      for (auto &kp : kept) {
        double dx = pt[0] - kp[0];
        double dy = pt[1] - kp[1];
        double dz = pt[2] - kp[2];
        double dd = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dd < prune_dist_) {
          too_close = true;
          break;
        }
      }
      if (!too_close) {
        kept.push_back(pt);
      }
    }

    // vantage_points_ = [base] + kept
    for (auto &pt : kept) {
      vantage_points_.push_back(pt);
    }

    RCLCPP_INFO(this->get_logger(),
                "Collected %zu vantage points (after prune).",
                vantage_points_.size());
  }

  bool isFree3D(double x, double y, double z)
  {
    auto node = octree_->search(x, y, z);
    if (!node) {
      // unknown => free
      return true;
    }
    return !octree_->isNodeOccupied(node);
  }

  // ---------------------------
  // Build adjacency
  // ---------------------------
  void buildAdjacency()
  {
    adjacency_.resize(vantage_points_.size());
    for (auto &adj : adjacency_) {
      adj.clear();
    }

    size_t N = vantage_points_.size();
    for (size_t i = 0; i < N; i++) {
      for (size_t j = i+1; j < N; j++) {
        double dx = vantage_points_[j][0] - vantage_points_[i][0];
        double dy = vantage_points_[j][1] - vantage_points_[i][1];
        double dz = vantage_points_[j][2] - vantage_points_[i][2];
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist <= max_los_dist_) {
          if (lineOfSight3D(vantage_points_[i], vantage_points_[j])) {
            adjacency_[i].push_back(j);
            adjacency_[j].push_back(i);
          }
        }
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "Built adjacency among %zu vantage points.", N);
  }

  bool lineOfSight3D(const std::array<double,3> &p1, 
                     const std::array<double,3> &p2)
  {
    octomap::point3d origin(p1[0], p1[1], p1[2]);
    octomap::point3d target(p2[0], p2[1], p2[2]);
    octomap::point3d direction = target - origin;
    double maxRange = direction.norm();

    octomap::point3d hit;
    bool hitSomething = octree_->castRay(origin, direction, hit, true, maxRange);
    if (!hitSomething) {
      return true;
    } else {
      double distHit = (hit - origin).norm();
      return (distHit >= maxRange);
    }
  }

  // ---------------------------
  // BFS
  // ---------------------------
  void computeBFSOrder()
  {
    bfs_order_.clear();
    size_t N = vantage_points_.size();
    if (N <= 1) {
      RCLCPP_WARN(this->get_logger(),"No vantage beyond base => BFS trivial");
      return;
    }
    std::vector<bool> visited(N, false);
    std::queue<int> q;
    visited[0] = true; // base
    q.push(0);

    while (!q.empty()) {
      int c = q.front();
      q.pop();
      for (auto nbr : adjacency_[c]) {
        if (!visited[nbr]) {
          visited[nbr] = true;
          if (nbr != 0) {
            bfs_order_.push_back(nbr);
          }
          q.push(nbr);
        }
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "BFS found %zu vantage points from base.", 
                bfs_order_.size());
  }

  // ---------------------------
  // assign vantage round-robin
  // ---------------------------
  void assignVantagePoints()
  {
    for (int i = 1; i <= 5; i++) {
      move_queues_[i].clear();
    }
    for (size_t idx = 0; idx < bfs_order_.size(); idx++) {
      int drone_id = (idx % 5) + 1;
      int vp_idx = bfs_order_[idx];
      auto &coords = vantage_points_[vp_idx];
      move_queues_[drone_id].push_back({coords, false});
    }
    RCLCPP_INFO(this->get_logger(),"Assigned BFS vantage points in round-robin to 5 drones.");
  }

  void publishVantageMarkers()
  {
    visualization_msgs::msg::MarkerArray ma;
    auto now = this->now();
    for (size_t i = 0; i < vantage_points_.size(); i++) {
      const auto &vp = vantage_points_[i];
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = now;
      mk.ns = "vantage_3d";
      mk.id = i;
      mk.type = visualization_msgs::msg::Marker::SPHERE;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.pose.position.x = vp[0];
      mk.pose.position.y = vp[1];
      mk.pose.position.z = vp[2];
      mk.pose.orientation.w = 1.0;
      mk.scale.x = 1.3;
      mk.scale.y = 1.3;
      mk.scale.z = 1.3;
      mk.color.r = 0.0f;
      mk.color.g = 1.0f;
      mk.color.b = 1.0f;
      mk.color.a = 1.0f;
      ma.markers.push_back(mk);
    }
    vantage_marker_pub_->publish(ma);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BFSConnectivity3DPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <octomap/octomap.h>
// #include <octomap/OcTree.h>

// #include <nav_msgs/msg/odometry.hpp>
// #include <sensor_msgs/msg/battery_state.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <visualization_msgs/msg/marker.hpp>

// #include <string>
// #include <vector>
// #include <deque>
// #include <queue>
// #include <unordered_map>
// #include <cmath>
// #include <memory>
// #include <algorithm>

// class BFSConnectivity3DPlanner : public rclcpp::Node
// {
// public:
//   BFSConnectivity3DPlanner()
//   : Node("bfs_connectivity_3d_planner")
//   {
//     // -----------------------------------------------------
//     // Declare Parameters
//     // -----------------------------------------------------
//     this->declare_parameter<std::string>("octomap_file", "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/city_1/meshes/city_1.binvox.bt");
//     this->declare_parameter<double>("z_min", 4.0);
//     this->declare_parameter<double>("z_max", 25.0);
//     this->declare_parameter<double>("downsample_step", 10.0); //logic to be impleented
//     this->declare_parameter<double>("prune_dist", 10.0); //logic to be impleented
//     this->declare_parameter<double>("max_los_dist", 70.0);
//     this->declare_parameter<double>("arrival_threshold", 1.5);
//     this->declare_parameter<double>("base_x", 0.0);
//     this->declare_parameter<double>("base_y", 0.0);
//     this->declare_parameter<double>("base_z", 1.0);
//     this->declare_parameter<double>("fixed_alt", 2.0);

//     // Battery thresholds
//     this->declare_parameter<double>("battery_return_threshold", 0.35);
//     this->declare_parameter<double>("battery_resume_threshold", 0.90);

//     // -----------------------------------------------------
//     // Get Parameters
//     // -----------------------------------------------------
//     octomap_file_ = this->get_parameter("octomap_file").as_string();
//     z_min_        = this->get_parameter("z_min").as_double();
//     z_max_        = this->get_parameter("z_max").as_double();
//     ds_step_      = this->get_parameter("downsample_step").as_double();
//     prune_dist_   = this->get_parameter("prune_dist").as_double();
//     max_los_dist_ = this->get_parameter("max_los_dist").as_double();
//     arrival_threshold_ = this->get_parameter("arrival_threshold").as_double();
//     base_x_       = this->get_parameter("base_x").as_double();
//     base_y_       = this->get_parameter("base_y").as_double();
//     base_z_       = this->get_parameter("base_z").as_double();
//     fixed_alt_    = this->get_parameter("fixed_alt").as_double();

//     battery_return_th_  = this->get_parameter("battery_return_threshold").as_double();
//     battery_resume_th_  = this->get_parameter("battery_resume_threshold").as_double();

//     // -----------------------------------------------------
//     // Load OctoMap
//     // -----------------------------------------------------
//     octree_ = std::make_shared<octomap::OcTree>(octomap_file_);
//     if (!octree_ || octree_->size() == 0) {
//       RCLCPP_ERROR(this->get_logger(), 
//                    "Failed to load OctoMap or it's empty. File: %s", 
//                    octomap_file_.c_str());
//       // no sense continuing if empty
//       rclcpp::shutdown();
//       return;
//     }
//     // Retrieve bounding box from the octree
//     double minX, minY, minZ, maxX, maxY, maxZ;
//     octree_->getMetricMin(minX, minY, minZ);
//     octree_->getMetricMax(maxX, maxY, maxZ);
//     RCLCPP_INFO(this->get_logger(), 
//                 "Loaded OctoMap from: %s. Bounding box: (%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f)",
//                 octomap_file_.c_str(), minX, minY, minZ, maxX, maxY, maxZ);

//     // -----------------------------------------------------
//     // Drone Data Structures
//     // -----------------------------------------------------
//     for (int i = 1; i <= 5; i++) {
//       // Initialize path result
//       path_results_[i] = true;   // or std::nullopt
//       // Battery + returning
//       battery_states_[i] = 1.0;  // assume 100% start
//       drone_returning_[i] = false;
//       // Move queue
//       move_queues_[i] = std::deque<std::pair<std::array<double,3>, bool>>();
//     }

//     // For simplicity, assume the base station is drone_id=0 vantage
//     current_vantage_[0] = {base_x_, base_y_, base_z_};
//     for (int i = 1; i <= 5; i++) {
//       current_vantage_[i] = std::nullopt;
//     }

//     // -----------------------------------------------------
//     // Subscribers for each drone: odom, battery, path_result
//     // -----------------------------------------------------
//     for (int i = 1; i <= 5; i++) {
//       // Odom
//       std::string odom_topic = "/cf_" + std::to_string(i) + "/odom";
//       odom_subs_.push_back(
//         this->create_subscription<nav_msgs::msg::Odometry>(
//           odom_topic, 10,
//           [this, i](nav_msgs::msg::Odometry::SharedPtr msg){
//             double x = msg->pose.pose.position.x;
//             double y = msg->pose.pose.position.y;
//             double z = msg->pose.pose.position.z;
//             drone_poses_[i] = {x, y, z};
//           }
//         )
//       );

//       // Battery
//       std::string batt_topic = "/cf_" + std::to_string(i) + "/battery_status";
//       battery_subs_.push_back(
//         this->create_subscription<sensor_msgs::msg::BatteryState>(
//           batt_topic, 10,
//           [this, i](sensor_msgs::msg::BatteryState::SharedPtr msg){
//             double perc = msg->percentage; // [0..1], presumably
//             battery_states_[i] = perc;
//             checkAndHandleBattery(i, perc);
//           }
//         )
//       );

//       // Path result
//       std::string pathres_topic = "/uav_" + std::to_string(i) + "_path_result";
//       pathres_subs_.push_back(
//         this->create_subscription<std_msgs::msg::Bool>(
//           pathres_topic, 10,
//           [this, i](std_msgs::msg::Bool::SharedPtr msg){
//             path_results_[i] = msg->data;
//           }
//         )
//       );
//     }

//     // -----------------------------------------------------
//     // Publishers for each drone: goal PoseStamped, return Bool
//     // -----------------------------------------------------
//     for (int i = 1; i <= 5; i++) {
//       std::string goal_topic = "/drone" + std::to_string(i) + "_goal";
//       drone_goal_pubs_[i] = 
//         this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

//       std::string return_topic = "/drone" + std::to_string(i) + "_returning";
//       drone_return_pubs_[i] =
//         this->create_publisher<std_msgs::msg::Bool>(return_topic, 10);
//     }

//     // Marker pub for vantage points
//     vantage_marker_pub_ = 
//       this->create_publisher<visualization_msgs::msg::MarkerArray>("bfs_3d_vantage_markers", 10);

//     // -----------------------------------------------------
//     // Build vantage points & BFS adjacency once at startup
//     // (Alternatively, you might want to do this lazily or 
//     //  whenever you get a 2D projected map.)
//     // -----------------------------------------------------
//     buildCandidateVantagePoints();
//     buildAdjacency();
//     computeBFSOrder();

//     // Round-robin assignment
//     assignVantagePoints();

//     // Publish markers
//     publishVantageMarkers();

//     // -----------------------------------------------------
//     // Timer for assignment logic
//     // -----------------------------------------------------
//     assignment_timer_ = this->create_wall_timer(
//       std::chrono::seconds(1),
//       std::bind(&BFSConnectivity3DPlanner::assignmentTimerCallback, this)
//     );

//     RCLCPP_INFO(this->get_logger(), "BFSConnectivity3DPlanner initialized.");
//   }

// private:
//   // ======================================================
//   // Parameters
//   // ======================================================
//   std::string octomap_file_;
//   double z_min_, z_max_;
//   double ds_step_;
//   double prune_dist_;
//   double max_los_dist_;
//   double arrival_threshold_;
//   double base_x_, base_y_, base_z_;
//   double fixed_alt_;

//   double battery_return_th_;
//   double battery_resume_th_;

//   // ======================================================
//   // OctoMap
//   // ======================================================
//   std::shared_ptr<octomap::OcTree> octree_;

//   // ======================================================
//   // Drone States
//   // ======================================================
//   // Drone pose
//   std::unordered_map<int, std::array<double,3>> drone_poses_;
//   // Battery
//   std::unordered_map<int, double> battery_states_;
//   // If drone is returning or not
//   std::unordered_map<int, bool> drone_returning_;
//   // Path result: True => success, False => fail
//   std::unordered_map<int, bool> path_results_;

//   // Currently "occupied" vantage by each drone (or base=0)
//   std::unordered_map<int, std::optional<std::array<double,3>>> current_vantage_;

//   // Per-drone vantage queue
//   std::unordered_map<int, std::deque<std::pair<std::array<double,3>, bool>>> move_queues_;

//   // ======================================================
//   // Vantage Points & Adjacency
//   // ======================================================
//   std::vector<std::array<double,3>> vantage_points_; 
//   std::vector<std::vector<int>> adjacency_;

//   // BFS order excluding vantage #0 (which is base)
//   std::vector<int> bfs_order_;

//   // ======================================================
//   // Publishers/Subscribers
//   // ======================================================
//   std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
//   std::vector<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subs_;
//   std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> pathres_subs_;

//   std::unordered_map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> drone_goal_pubs_;
//   std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> drone_return_pubs_;

//   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vantage_marker_pub_;

//   rclcpp::TimerBase::SharedPtr assignment_timer_;

//   // ------------------------------------------------------
//   // checkAndHandleBattery: sets "returning" if < threshold
//   // ------------------------------------------------------
//   void checkAndHandleBattery(int drone_id, double perc)
//   {
//     if (!drone_returning_[drone_id] && (perc < battery_return_th_)) {
//       // Switch to returning
//       drone_returning_[drone_id] = true;
//       // Clear vantage queue
//       move_queues_[drone_id].clear();
//       // Publish returning = True
//       std_msgs::msg::Bool msg;
//       msg.data = true;
//       drone_return_pubs_[drone_id]->publish(msg);

//       // Command them to go to base
//       publishGoal(drone_id, {base_x_, base_y_, base_z_});
//       RCLCPP_WARN(this->get_logger(),
//                   "Drone %d battery=%.2f => returning home!", 
//                   drone_id, perc);
//     }
//     else if (drone_returning_[drone_id] && (perc > battery_resume_th_)) {
//       // Switch to normal coverage
//       drone_returning_[drone_id] = false;
//       std_msgs::msg::Bool msg;
//       msg.data = false;
//       drone_return_pubs_[drone_id]->publish(msg);

//       RCLCPP_INFO(this->get_logger(),
//                   "Drone %d battery=%.2f => rejoining coverage!", 
//                   drone_id, perc);
//       // Optionally re-insert vantage points, or let them be.
//       // For simplicity, this code does nothing immediately.
//     }
//   }

//   // ------------------------------------------------------
//   // assignmentTimerCallback: main logic loop
//   // ------------------------------------------------------
//   void assignmentTimerCallback()
//   {
//     // For each drone, handle vantage logic
//     for (int drone_id = 1; drone_id <= 5; drone_id++) {
//       // if returning => skip vantage logic
//       if (drone_returning_[drone_id]) {
//         continue;
//       }

//       auto &dq = move_queues_[drone_id];
//       if (dq.empty()) {
//         // no vantage assigned
//         continue;
//       }

//       auto &front = dq.front();
//       auto vantage = front.first;  // xyz
//       bool published = front.second;

//       // check path result => if fail => skip vantage
//       if (!path_results_[drone_id]) {
//         RCLCPP_WARN(this->get_logger(),
//                     "Drone %d path fail => skipping vantage (%.2f, %.2f, %.2f)",
//                     drone_id, vantage[0], vantage[1], vantage[2]);
//         dq.pop_front();
//         path_results_[drone_id] = true; // reset
//         continue;
//       }

//       // Must have odom
//       if (drone_poses_.find(drone_id) == drone_poses_.end()) {
//         // no odometry => skip vantage
//         dq.pop_front();
//         continue;
//       }

//       // check arrival
//       auto pose = drone_poses_[drone_id];
//       double dx = pose[0] - vantage[0];
//       double dy = pose[1] - vantage[1];
//       double dz = pose[2] - vantage[2];
//       double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
//       if (dist < arrival_threshold_) {
//         RCLCPP_INFO(this->get_logger(),
//                     "Drone %d arrived vantage => (%.2f, %.2f, %.2f)",
//                     drone_id, vantage[0], vantage[1], vantage[2]);
//         dq.pop_front();
//         current_vantage_[drone_id] = vantage;
//         continue;
//       }

//       // check LOS with the base or any *occupied* vantage
//       // If not, skip vantage
//       if (!hasLOSWithAnyOccupied(vantage, drone_id)) {
//         RCLCPP_WARN(this->get_logger(),
//                     "Drone %d vantage => (%.2f, %.2f, %.2f) not in LOS => skip.",
//                     drone_id, vantage[0], vantage[1], vantage[2]);
//         dq.pop_front();
//         continue;
//       }

//       // if not published => publish goal
//       if (!published) {
//         publishGoal(drone_id, vantage);
//         front.second = true;
//         RCLCPP_INFO(this->get_logger(),
//                     "Drone %d assigned vantage => (%.2f, %.2f, %.2f).",
//                     drone_id, vantage[0], vantage[1], vantage[2]);
//       }
//     }
//   }

//   // ------------------------------------------------------
//   // hasLOSWithAnyOccupied:
//   //   checks if vantage is in line-of-sight with base or
//   //   any vantage physically occupied by a different drone.
//   // ------------------------------------------------------
//   bool hasLOSWithAnyOccupied(const std::array<double,3> &v, int drone_id)
//   {
//     for (auto &kv : current_vantage_) {
//       int other_id = kv.first;
//       // skip self
//       if (other_id == drone_id) 
//         continue;
//       if (!kv.second.has_value()) 
//         continue; // no vantage
//       auto ov = kv.second.value();
//       if (lineOfSight3D(v, ov)) {
//         return true;
//       }
//     }
//     // No vantage found => fail
//     return false;
//   }

//   // ------------------------------------------------------
//   // publishGoal
//   // ------------------------------------------------------
//   void publishGoal(int drone_id, const std::array<double,3> &v)
//   {
//     geometry_msgs::msg::PoseStamped ps;
//     ps.header.stamp = this->now();
//     ps.header.frame_id = "world";
//     ps.pose.position.x = v[0];
//     ps.pose.position.y = v[1];
//     ps.pose.position.z = v[2] < 0.1 ? fixed_alt_ : v[2]; 
//     // or always use fixed_alt_ if you want
//     ps.pose.orientation.w = 1.0;
//     drone_goal_pubs_[drone_id]->publish(ps);
//   }

//   // ------------------------------------------------------
//   // Build candidate vantage points
//   //   naive: step through bounding box in x,y,z
//   // ------------------------------------------------------
//   void buildCandidateVantagePoints()
//   {
//     // first push base vantage as index=0
//     vantage_points_.push_back({base_x_, base_y_, base_z_});

//     // bounding box
//     double minX, minY, minZ, maxX, maxY, maxZ;
//     octree_->getMetricMin(minX, minY, minZ);
//     octree_->getMetricMax(maxX, maxY, maxZ);

//     // clamp z
//     if (z_min_ > minZ) minZ = z_min_;
//     if (z_max_ < maxZ) maxZ = z_max_;

//     // naive sampling
//     std::vector<std::array<double,3>> raw;
//     for (double x = minX; x <= maxX; x += ds_step_) {
//       for (double y = minY; y <= maxY; y += ds_step_) {
//         for (double z = minZ; z <= maxZ; z += ds_step_) {
//           if (isFree3D(x, y, z)) {
//             raw.push_back({x, y, z});
//           }
//         }
//       }
//     }

//     // prune
//     std::vector<std::array<double,3>> kept;
//     for (auto &pt : raw) {
//       bool too_close = false;
//       for (auto &kp : kept) {
//         double dx = pt[0] - kp[0];
//         double dy = pt[1] - kp[1];
//         double dz = pt[2] - kp[2];
//         double dd = std::sqrt(dx*dx + dy*dy + dz*dz);
//         if (dd < prune_dist_) {
//           too_close = true;
//           break;
//         }
//       }
//       if (!too_close) {
//         kept.push_back(pt);
//       }
//     }

//     // vantage_points_ = [ base ] + kept
//     for (auto &pt : kept) {
//       vantage_points_.push_back(pt);
//     }

//     RCLCPP_INFO(this->get_logger(),
//                 "Collected %zu vantage points (after prune).", 
//                 vantage_points_.size());
//   }

//   // ------------------------------------------------------
//   // Build adjacency among vantage_points_ via LOS + dist
//   // adjacency_[i] = vector of vantage indices that are 
//   // directly in line-of-sight with vantage i
//   // ------------------------------------------------------
//   void buildAdjacency()
//   {
//     adjacency_.resize(vantage_points_.size());
//     for (size_t i = 0; i < vantage_points_.size(); i++) {
//       adjacency_[i].clear();
//     }

//     size_t N = vantage_points_.size();
//     for (size_t i = 0; i < N; i++) {
//       for (size_t j = i+1; j < N; j++) {
//         double dx = vantage_points_[j][0] - vantage_points_[i][0];
//         double dy = vantage_points_[j][1] - vantage_points_[i][1];
//         double dz = vantage_points_[j][2] - vantage_points_[i][2];
//         double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
//         if (dist <= max_los_dist_) {
//           // check line-of-sight
//           if (lineOfSight3D(vantage_points_[i], vantage_points_[j])) {
//             adjacency_[i].push_back(j);
//             adjacency_[j].push_back(i);
//           }
//         }
//       }
//     }

//     RCLCPP_INFO(this->get_logger(),
//                 "Built adjacency among %zu vantage points.", N);
//   }

//   // ------------------------------------------------------
//   // BFS from vantage 0 => base vantage
//   // store the BFS order into bfs_order_ (excluding 0)
//   // ------------------------------------------------------
//   void computeBFSOrder()
//   {
//     bfs_order_.clear();
//     size_t N = vantage_points_.size();
//     if (N <= 1) {
//       RCLCPP_WARN(this->get_logger(), 
//                   "No vantage points beyond base => BFS is trivial.");
//       return;
//     }

//     std::vector<bool> visited(N, false);
//     std::queue<int> q;
//     // vantage 0 is base
//     visited[0] = true;
//     q.push(0);

//     while (!q.empty()) {
//       int cur = q.front();
//       q.pop();
//       for (auto nbr : adjacency_[cur]) {
//         if (!visited[nbr]) {
//           visited[nbr] = true;
//           // skip push base vantage into BFS
//           if (nbr != 0) {
//             bfs_order_.push_back(nbr);
//           }
//           q.push(nbr);
//         }
//       }
//     }

//     RCLCPP_INFO(this->get_logger(),
//                 "BFS found %zu vantage points reachable from base.", 
//                 bfs_order_.size());
//   }

//   // ------------------------------------------------------
//   // assign vantage points to each drone in round-robin
//   // ------------------------------------------------------
//   void assignVantagePoints()
//   {
//     // Clear existing queues
//     for (int i = 1; i <= 5; i++) {
//       move_queues_[i].clear();
//     }
//     // Round robin
//     for (size_t idx = 0; idx < bfs_order_.size(); idx++) {
//       int drone_id = (idx % 5) + 1;
//       int vp_idx = bfs_order_[idx];
//       auto &coords = vantage_points_[vp_idx];
//       move_queues_[drone_id].push_back({coords, false});
//     }
//     RCLCPP_INFO(this->get_logger(),
//                 "Assigned BFS vantage points in round-robin fashion to 5 drones.");
//   }

//   // ------------------------------------------------------
//   // isFree3D: checks if the point is in free space
//   // treat unknown as free
//   // ------------------------------------------------------
//   bool isFree3D(double x, double y, double z)
//   {
//     octomap::OcTreeNode* node = octree_->search(x, y, z);
//     if (!node) {
//       // unknown => treat free
//       return true;
//     }
//     return !octree_->isNodeOccupied(node);
//   }

//   // ------------------------------------------------------
//   // lineOfSight3D: uses OctoMap raycast
//   // returns true if no collision
//   // ------------------------------------------------------
//   bool lineOfSight3D(const std::array<double,3> &p1, 
//                      const std::array<double,3> &p2)
//   {
//     octomap::point3d origin(p1[0], p1[1], p1[2]);
//     octomap::point3d target(p2[0], p2[1], p2[2]);
//     octomap::point3d direction = target - origin;
//     double maxRange = direction.norm();

//     octomap::point3d hit;
//     bool hitSomething = octree_->castRay(origin, direction, hit, true, maxRange);
//     if (!hitSomething) {
//       // no hit => clear
//       return true;
//     }
//     else {
//       double distHit = (hit - origin).norm();
//       // if we hit something closer than the target => blocked
//       return (distHit >= maxRange);
//     }
//   }

//   // ------------------------------------------------------
//   // publishVantageMarkers
//   // ------------------------------------------------------
//   void publishVantageMarkers()
//   {
//     visualization_msgs::msg::MarkerArray ma;
//     rclcpp::Time now = this->now();
//     for (size_t i = 0; i < vantage_points_.size(); i++) {
//       const auto &vp = vantage_points_[i];
//       visualization_msgs::msg::Marker mk;
//       mk.header.frame_id = "world";
//       mk.header.stamp = now;
//       mk.ns = "vantage_3d";
//       mk.id = i;
//       mk.type = visualization_msgs::msg::Marker::SPHERE;
//       mk.action = visualization_msgs::msg::Marker::ADD;
//       mk.pose.position.x = vp[0];
//       mk.pose.position.y = vp[1];
//       mk.pose.position.z = vp[2];
//       mk.pose.orientation.w = 1.0;
//       mk.scale.x = 1.3;
//       mk.scale.y = 1.3;
//       mk.scale.z = 1.3;
//       mk.color.r = 0.0f;
//       mk.color.g = 1.0f;
//       mk.color.b = 1.0f;
//       mk.color.a = 1.0f;
//       ma.markers.push_back(mk);
//     }
//     vantage_marker_pub_->publish(ma);
//   }
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<BFSConnectivity3DPlanner>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
