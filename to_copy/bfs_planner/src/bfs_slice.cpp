#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

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
#include <optional>
#include <cmath>

class BFSConnectivity3DNode : public rclcpp::Node
{
public:
  BFSConnectivity3DNode()
    : Node("bfs_connectivity_3d_node")
  {
    // -------------------------------------------
    // 1) Parameters
    // -------------------------------------------
    this->declare_parameter<std::string>("octomap_file",
      "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/city_1/meshes/city_1.binvox.bt");
    this->declare_parameter<double>("z_min", 1.0);
    this->declare_parameter<double>("z_max", 25.0);
    this->declare_parameter<double>("downsample_step", 10.0);
    this->declare_parameter<double>("prune_dist", 5.0);
    this->declare_parameter<double>("max_los_dist", 70.0);
    this->declare_parameter<double>("plane_step", 2.0);

    // BFS arrival threshold not used if we rely on hover_done
    this->declare_parameter<double>("arrival_threshold", 1.5);

    // battery thresholds
    this->declare_parameter<double>("battery_return_threshold", 0.35);
    this->declare_parameter<double>("battery_resume_threshold", 0.90);

    // base vantage
    this->declare_parameter<double>("base_x", 0.0);
    this->declare_parameter<double>("base_y", 0.0);
    this->declare_parameter<double>("base_z", 1.0);

    // <<< NEW: Minimum separation among drones >>>
    this->declare_parameter<double>("min_drone_separation", 2.0);

    // Retrieve parameter values
    octomap_file_   = this->get_parameter("octomap_file").as_string();
    z_min_          = this->get_parameter("z_min").as_double();
    z_max_          = this->get_parameter("z_max").as_double();
    ds_step_        = this->get_parameter("downsample_step").as_double();
    prune_dist_     = this->get_parameter("prune_dist").as_double();
    max_los_dist_   = this->get_parameter("max_los_dist").as_double();
    plane_step_     = this->get_parameter("plane_step").as_double();

    batt_return_th_ = this->get_parameter("battery_return_threshold").as_double();
    batt_resume_th_ = this->get_parameter("battery_resume_threshold").as_double();
    base_x_         = this->get_parameter("base_x").as_double();
    base_y_         = this->get_parameter("base_y").as_double();
    base_z_         = this->get_parameter("base_z").as_double();

    min_drone_sep_  = this->get_parameter("min_drone_separation").as_double();

    // -------------------------------------------
    // 2) Load OctoMap
    // -------------------------------------------
    octree_ = std::make_shared<octomap::OcTree>(octomap_file_);
    if(!octree_ || octree_->size() == 0){
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to load or empty OctoMap: %s",
        octomap_file_.c_str()
      );
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded OctoMap from: %s",
      octomap_file_.c_str()
    );

    // -------------------------------------------
    // 3) Drone data structures
    // -------------------------------------------
    for(int i=1; i<=5; i++){
      battery_states_[i]   = 1.0;
      drone_returning_[i]  = false;
      path_results_[i]     = true;
      hover_done_[i]       = false;
      vantage_queues_[i]   = std::deque<std::pair<std::array<double,3>, bool>>();
      current_vantage_[i]  = std::nullopt;
    }
    // vantage #0 => base vantage
    current_vantage_[0] = std::array<double,3>{base_x_, base_y_, base_z_};

    // -------------------------------------------
    // 4) Sub/Pub for each drone
    // -------------------------------------------
    for(int i=1; i<=5; i++){
      // battery
      auto batt_topic = "/cf_" + std::to_string(i) + "/battery_status";
      battery_subs_.push_back(
        this->create_subscription<sensor_msgs::msg::BatteryState>(
          batt_topic, 10,
          [this,i](sensor_msgs::msg::BatteryState::SharedPtr msg){
            double perc = msg->percentage;
            battery_states_[i] = perc;
            handleBattery(i, perc);
          }
        )
      );

      // path_result
      auto pathres_topic = "/uav_" + std::to_string(i) + "_path_result";
      pathres_subs_.push_back(
        this->create_subscription<std_msgs::msg::Bool>(
          pathres_topic, 10,
          [this,i](std_msgs::msg::Bool::SharedPtr msg){
            path_results_[i] = msg->data;
          }
        )
      );

      // hover_done
      auto hover_topic = "/drone" + std::to_string(i) + "_hover_done";
      hoverdone_subs_.push_back(
        this->create_subscription<std_msgs::msg::Bool>(
          hover_topic, 10,
          [this,i](std_msgs::msg::Bool::SharedPtr msg){
            if(msg->data){
              hover_done_[i] = true;
              RCLCPP_INFO(this->get_logger(),
                "Drone %d => hover_done received", i
              );
            }
          }
        )
      );

      // goal publisher
      auto goal_topic = "/drone" + std::to_string(i) + "_goal";
      goal_pubs_[i] = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

      // returning bool
      auto ret_topic = "/drone" + std::to_string(i) + "_returning";
      return_pubs_[i] = this->create_publisher<std_msgs::msg::Bool>(ret_topic, 10);
    }

    // vantage marker
    vantage_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "bfs_3d_vantages", 10
    );

    // -------------------------------------------
    // 5) Multi-plane coverage setup
    // -------------------------------------------
    buildZLevels(); // build z_levels_ from z_min_..z_max_ in plane_step_
    current_plane_idx_ = 0;
    coverage_done_for_plane_ = false;
    waiting_for_return_ = false;

    if (z_levels_.empty()){
      RCLCPP_ERROR(this->get_logger(),
        "No valid z-levels => shutting down."
      );
      rclcpp::shutdown();
      return;
    }

    // Build vantage coverage for the first plane
    buildPlaneCoverage(current_plane_idx_);

    // Timer => checks vantage assignment, coverage completion
    assignment_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&BFSConnectivity3DNode::assignmentCallback, this)
    );

    RCLCPP_INFO(this->get_logger(),
      "BFSConnectivity3DNode ready."
    );
  }

private:
  // ---------- parameters ----------
  std::string octomap_file_;
  double z_min_, z_max_;
  double ds_step_;
  double prune_dist_;
  double max_los_dist_;
  double plane_step_;

  // multi-plane coverage
  std::vector<double> z_levels_;
  int current_plane_idx_;
  bool coverage_done_for_plane_;
  bool waiting_for_return_;

  double batt_return_th_, batt_resume_th_;
  double base_x_, base_y_, base_z_;

  // <<< NEW: Min separation >>>
  double min_drone_sep_;

  // octomap
  std::shared_ptr<octomap::OcTree> octree_;

  // vantage points for the current plane
  std::vector<std::array<double,3>> vantage_points_;
  std::vector<std::vector<int>> adjacency_;
  std::vector<int> bfs_order_;

  // drone data
  std::unordered_map<int, double> battery_states_;
  std::unordered_map<int, bool>   drone_returning_;
  std::unordered_map<int, bool>   path_results_;
  std::unordered_map<int, bool>   hover_done_;

  // vantage
  std::unordered_map<int, std::optional<std::array<double,3>>> current_vantage_;
  std::unordered_map<int, std::deque<std::pair<std::array<double,3>, bool>>> vantage_queues_;

  // pubs/subs
  std::vector<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> pathres_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> hoverdone_subs_;

  std::unordered_map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> goal_pubs_;
  std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> return_pubs_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vantage_marker_pub_;
  rclcpp::TimerBase::SharedPtr assignment_timer_;

  // --------------------------------------------------
  // build the list of z-levels
  // --------------------------------------------------
  void buildZLevels()
  {
    z_levels_.clear();
    double z = z_min_;
    while(z <= (z_max_ + 1e-3)){
      z_levels_.push_back(z);
      z += plane_step_;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Z-levels: total %zu planes from %.2f to %.2f with step=%.2f",
      z_levels_.size(), z_min_, z_max_, plane_step_
    );
  }

  // --------------------------------------------------
  // Build vantage coverage for a given plane index
  // --------------------------------------------------
  void buildPlaneCoverage(int plane_idx)
  {
    if((size_t)plane_idx >= z_levels_.size()){
      RCLCPP_WARN(this->get_logger(),
        "Requested plane_idx out of range!"
      );
      return;
    }
    double z_plane = z_levels_[plane_idx];
    RCLCPP_INFO(
      this->get_logger(),
      "=== Building vantage coverage for plane z=%.2f ===",
      z_plane
    );

    vantage_points_.clear();
    adjacency_.clear();
    bfs_order_.clear();

    buildVantagePointsForPlane(z_plane);
    buildAdjacency();
    computeBFSOrder();
    assignVantages();
    publishVantageMarkers();

    coverage_done_for_plane_ = false;
    waiting_for_return_      = false;
  }

  // --------------------------------------------------
  // Build vantage points in a single plane (with border checking)
  // --------------------------------------------------
  void buildVantagePointsForPlane(double z_plane)
  {
    vantage_points_.clear();
    // vantage_points_[0] => base vantage
    vantage_points_.push_back({base_x_, base_y_, base_z_});

    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree_->getMetricMin(minX, minY, minZ);
    octree_->getMetricMax(maxX, maxY, maxZ);

    std::vector<std::array<double,3>> raw;
    double border_margin = ds_step_; // border margin to skip points near the octomap boundaries

    for(double x = minX; x <= maxX; x += ds_step_){
      for(double y = minY; y <= maxY; y += ds_step_){
        // Skip points too close to the border
        if ((x - minX) < border_margin || (maxX - x) < border_margin ||
            (y - minY) < border_margin || (maxY - y) < border_margin) {
          continue;
        }
        // Check if free at the given z_plane
        if(isFree(x, y, z_plane)){
          raw.push_back({x, y, z_plane});
        }
      }
    }

    // Prune to keep vantage points at least prune_dist_ apart and not too close to the base vantage.
    std::vector<std::array<double,3>> kept;
    for(auto &pt : raw){
      double dxb = pt[0] - base_x_;
      double dyb = pt[1] - base_y_;
      double dzb = pt[2] - base_z_;
      double distB = std::sqrt(dxb*dxb + dyb*dyb + dzb*dzb);
      if(distB < prune_dist_){
        continue;
      }
      bool too_close = false;
      for(auto &kpt : kept){
        double dx = pt[0] - kpt[0];
        double dy = pt[1] - kpt[1];
        double dz = pt[2] - kpt[2];
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if(dist < prune_dist_){
          too_close = true;
          break;
        }
      }
      if(!too_close){
        kept.push_back(pt);
      }
    }

    // Add kept points (after the base) to vantage_points_
    for(auto &pt : kept){
      vantage_points_.push_back(pt);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Plane z=%.2f => vantage points after prune=%zu",
      z_plane, vantage_points_.size()
    );
  }

  bool isFree(double x, double y, double z)
  {
    auto node = octree_->search(x, y, z);
    // Treat unknown as free.
    if(!node) {
      return true;
    }
    return !octree_->isNodeOccupied(node);
  }

  // --------------------------------------------------
  // Build adjacency based on line-of-sight (LOS) between vantage points.
  // --------------------------------------------------
  void buildAdjacency()
  {
    adjacency_.resize(vantage_points_.size());
    for(auto &a : adjacency_){
      a.clear();
    }
    size_t N = vantage_points_.size();
    for(size_t i = 0; i < N; i++){
      for(size_t j = i + 1; j < N; j++){
        double dx = vantage_points_[j][0] - vantage_points_[i][0];
        double dy = vantage_points_[j][1] - vantage_points_[i][1];
        double dz = vantage_points_[j][2] - vantage_points_[i][2];
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if(dist < 1e-8) {
          continue;
        }
        if(dist <= max_los_dist_){
          if(hasLOS(vantage_points_[i], vantage_points_[j])){
            adjacency_[i].push_back(j);
            adjacency_[j].push_back(i);
          }
        }
      }
    }
  }

  bool hasLOS(const std::array<double,3> &p1,
              const std::array<double,3> &p2)
  {
    octomap::point3d origin(p1[0], p1[1], p1[2]);
    octomap::point3d target(p2[0], p2[1], p2[2]);
    octomap::point3d dir = target - origin;
    double maxRange = dir.norm();
    if(maxRange < 1e-8){
      return false;
    }
    octomap::point3d hit;
    bool hitSomething = octree_->castRay(origin, dir, hit, true, maxRange);
    if(!hitSomething){
      return true;
    } else {
      double distHit = (hit - origin).norm();
      return (distHit >= maxRange);
    }
  }

  // --------------------------------------------------
  // Compute BFS order from vantage point #0 (base)
  // --------------------------------------------------
  void computeBFSOrder()
  {
    bfs_order_.clear();
    size_t N = vantage_points_.size();
    if(N <= 1) return;
    std::vector<bool> visited(N, false);
    std::queue<int> q;
    visited[0] = true;
    q.push(0);
    while(!q.empty()){
      int c = q.front();
      q.pop();
      for(auto &nbr : adjacency_[c]){
        if(!visited[nbr]){
          visited[nbr] = true;
          if(nbr != 0)
            bfs_order_.push_back(nbr);
          q.push(nbr);
        }
      }
    }
  }

  // --------------------------------------------------
  // Assign BFS order of vantage points to each drone
  // --------------------------------------------------
  void assignVantages()
  {
    for(int i=1; i<=5; i++){
      vantage_queues_[i].clear();
    }
    for(size_t idx = 0; idx < bfs_order_.size(); idx++){
      int drone_id = (idx % 5) + 1;
      int vp_idx   = bfs_order_[idx];
      vantage_queues_[drone_id].push_back({vantage_points_[vp_idx], false});
    }
    RCLCPP_INFO(this->get_logger(),
      "Assigned BFS vantage points among 5 drones."
    );
  }

  // --------------------------------------------------
  // Timer callback to check vantage assignment and coverage status.
  // --------------------------------------------------
  void assignmentCallback()
  {
    // Check if all drones have finished coverage (i.e. their vantage queues are empty)
    bool all_empty = true;
    for(int i = 1; i <= 5; i++){
      if(!vantage_queues_[i].empty()){
        all_empty = false;
        break;
      }
    }
    coverage_done_for_plane_ = all_empty;

    // If coverage is done but not yet commanded to return
    if(coverage_done_for_plane_ && !waiting_for_return_){
      RCLCPP_INFO(this->get_logger(),
        "Coverage done => commanding all drones to return home."
      );
      for(int i = 1; i <= 5; i++){
        drone_returning_[i] = true;
        vantage_queues_[i].clear();
        std_msgs::msg::Bool ret;
        ret.data = true;
        return_pubs_[i]->publish(ret);
        // Send the drone to base
        publishGoal(i, base_x_, base_y_, base_z_);
      }
      waiting_for_return_ = true;
      return;
    }

    // If coverage is done and waiting, check if all drones are home
    if(coverage_done_for_plane_ && waiting_for_return_){
      if(allDronesHome()){
        RCLCPP_INFO(
          this->get_logger(),
          "All drones are home => move to next plane (if any)."
        );
        moveToNextPlane();
      }
    }

    // Normal vantage assignment
    if(!coverage_done_for_plane_){
      for(int i = 1; i <= 5; i++){
        if(drone_returning_[i]) continue; // Skip if drone is returning

        auto &dq = vantage_queues_[i];
        if(dq.empty()) continue;

        auto &front    = dq.front();
        auto vantage   = front.first;
        bool published = front.second;

        // If path failed, skip this vantage
        if(!path_results_[i]){
          RCLCPP_WARN(this->get_logger(),
            "Drone %d path fail => skip vantage (%.2f, %.2f, %.2f)",
            i, vantage[0], vantage[1], vantage[2]
          );
          dq.pop_front();
          path_results_[i] = true;
          continue;
        }
        // If hover is done, remove this vantage
        if(hover_done_[i]){
          RCLCPP_INFO(this->get_logger(),
            "Drone %d vantage done => (%.2f, %.2f, %.2f).",
            i, vantage[0], vantage[1], vantage[2]
          );
          dq.pop_front();
          hover_done_[i] = false;
          continue;
        }

        // Check line-of-sight (LOS) with base or other drone vantages
        if(!hasLOSWithBaseOrOthers(vantage, i)){
          RCLCPP_WARN(this->get_logger(),
            "Drone %d => vantage (%.2f, %.2f, %.2f) no LOS => skip.",
            i, vantage[0], vantage[1], vantage[2]
          );
          dq.pop_front();
          continue;
        }

        // Enforce minimum drone separation
        if(!checkDroneSeparation(vantage, i)){
          RCLCPP_WARN(this->get_logger(),
            "Drone %d => vantage (%.2f, %.2f, %.2f) is too close to another drone => skip.",
            i, vantage[0], vantage[1], vantage[2]
          );
          dq.pop_front();
          continue;
        }

        // Publish the vantage if not yet published
        if(!published){
          publishGoal(i, vantage[0], vantage[1], vantage[2]);
          front.second = true;
          RCLCPP_INFO(this->get_logger(),
            "Drone %d assigned vantage => (%.2f, %.2f, %.2f)",
            i, vantage[0], vantage[1], vantage[2]
          );
        }
      }
    }
  }

  // Check if the vantage is at least min_drone_sep_ away from other drones' current vantages.
  bool checkDroneSeparation(const std::array<double,3> &v, int self_id)
  {
    for(auto &kv : current_vantage_){
      int other_id = kv.first;
      if(other_id == 0 || other_id == self_id)
        continue;
      if(!kv.second.has_value())
        continue;
      auto ov = kv.second.value();
      double dx = v[0] - ov[0];
      double dy = v[1] - ov[1];
      double dz = v[2] - ov[2];
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      if(dist < min_drone_sep_){
        return false;
      }
    }
    return true;
  }

  bool allDronesHome()
  {
    // A drone is "home" if it is returning and has completed hover.
    for(int i=1; i<=5; i++){
      if(!drone_returning_[i]) {
        return false;
      }
      if(!hover_done_[i]) {
        return false;
      }
    }
    return true;
  }

  void moveToNextPlane()
  {
    for(int i=1; i<=5; i++){
      hover_done_[i]      = false;
      drone_returning_[i] = false;
      vantage_queues_[i].clear();
      path_results_[i]    = true;
      current_vantage_[i] = std::nullopt;
    }
    waiting_for_return_      = false;
    coverage_done_for_plane_ = false;

    current_plane_idx_++;
    if ((size_t)current_plane_idx_ >= z_levels_.size()){
      RCLCPP_INFO(this->get_logger(),
        "No more planes => coverage complete!"
      );
      return;
    }
    buildPlaneCoverage(current_plane_idx_);
  }

  bool hasLOSWithBaseOrOthers(const std::array<double,3> &v, int self_id)
  {
    if(current_vantage_[0].has_value()){
      auto baseV = current_vantage_[0].value();
      if(hasLOS3D(v, baseV)) return true;
    }
    for(auto &kv : current_vantage_){
      int other_id = kv.first;
      if(other_id == 0 || other_id == self_id) continue;
      if(!kv.second.has_value()) continue;
      auto ov = kv.second.value();
      if(hasLOS3D(v, ov)) return true;
    }
    return false;
  }

  bool hasLOS3D(const std::array<double,3> &p1,
                const std::array<double,3> &p2)
  {
    octomap::point3d origin(p1[0], p1[1], p1[2]);
    octomap::point3d target(p2[0], p2[1], p2[2]);
    octomap::point3d dir = target - origin;
    double maxDist = dir.norm();
    if(maxDist < 1e-8){
      return false;
    }
    octomap::point3d hit;
    bool hitSomething = octree_->castRay(origin, dir, hit, true, maxDist);
    if(!hitSomething){
      return true;
    } else {
      double distHit = (hit - origin).norm();
      return (distHit >= maxDist);
    }
  }

  // Publish a goal for drone (update current vantage)
  void publishGoal(int drone_id, double x, double y, double z)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->now();
    ps.header.frame_id = "world";
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
    ps.pose.orientation.w = 1.0;
    goal_pubs_[drone_id]->publish(ps);

    current_vantage_[drone_id] = std::array<double,3>{x, y, z};
  }

  // --------------------------------------------------
  // Battery handling: command return if battery below threshold.
  // --------------------------------------------------
  void handleBattery(int drone_id, double perc)
  {
    if(!drone_returning_[drone_id] && (perc < batt_return_th_)){
      drone_returning_[drone_id] = true;
      vantage_queues_[drone_id].clear();
      std_msgs::msg::Bool ret;
      ret.data = true;
      return_pubs_[drone_id]->publish(ret);
      publishGoal(drone_id, base_x_, base_y_, base_z_);
      RCLCPP_WARN(this->get_logger(),
        "Drone %d => battery=%.2f => returning home!",
        drone_id, perc
      );
    } else if(drone_returning_[drone_id] && (perc > batt_resume_th_)){
      drone_returning_[drone_id] = false;
      std_msgs::msg::Bool ret;
      ret.data = false;
      return_pubs_[drone_id]->publish(ret);
      RCLCPP_INFO(this->get_logger(),
        "Drone %d => battery=%.2f => rejoin coverage!",
        drone_id, perc
      );
    }
  }

  // --------------------------------------------------
  // Publish vantage markers for debugging.
  // --------------------------------------------------
  void publishVantageMarkers()
  {
    visualization_msgs::msg::MarkerArray ma;
    auto now = this->now();
    for(size_t i=0; i<vantage_points_.size(); i++){
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp    = now;
      mk.ns              = "bfs_3d";
      mk.id              = i;
      mk.type            = visualization_msgs::msg::Marker::SPHERE;
      mk.action          = visualization_msgs::msg::Marker::ADD;
      mk.pose.position.x = vantage_points_[i][0];
      mk.pose.position.y = vantage_points_[i][1];
      mk.pose.position.z = vantage_points_[i][2];
      mk.pose.orientation.w = 1.0;
      mk.scale.x = 0.3;
      mk.scale.y = 0.3;
      mk.scale.z = 0.3;
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
  auto node = std::make_shared<BFSConnectivity3DNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
