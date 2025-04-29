#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <array>

class UAV2DControllerNode : public rclcpp::Node
{
public:
  explicit UAV2DControllerNode(int drone_id)
  : Node("uav_2d_controller_" + std::to_string(drone_id))
  , drone_id_(drone_id)
  {
    // Publishers
    std::string cmd_vel_topic = "/cf_" + std::to_string(drone_id) + "/cmd_vel";
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    std::string hover_done_topic = "/drone" + std::to_string(drone_id) + "_hover_done";
    hover_done_pub_ = this->create_publisher<std_msgs::msg::Bool>(hover_done_topic, 10);

    // Subscriptions
    std::string odom_topic = "/cf_" + std::to_string(drone_id) + "/odom";
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        curr_x_ = msg->pose.pose.position.x;
        curr_y_ = msg->pose.pose.position.y;
      }
    );

    std::string path_topic = "/uav_" + std::to_string(drone_id) + "_path";
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic, 10,
      [this](nav_msgs::msg::Path::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        path_points_.clear();
        for(auto &ps: msg->poses){
          double px = ps.pose.position.x;
          double py = ps.pose.position.y;
          // We do not explicitly store pz here for 2D control,
          // but if you want a 3D controller, keep track of it.
          // We'll just keep it to maintain altitude passively.
          path_points_.push_back({px, py});
        }
        waypoint_idx_ = 0;
        final_reached_ = false;
        RCLCPP_INFO(this->get_logger(),
          "Drone %d => new path => %zu waypoints",
          drone_id_, path_points_.size());
      }
    );

    // Timer for control loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&UAV2DControllerNode::controlLoop, this)
    );

    // Store node startup time
    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "UAV2DControllerNode %d started.", drone_id_);
  }

private:
  int drone_id_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hover_done_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  double curr_x_ = 0.0, curr_y_ = 0.0;
  std::vector<std::array<double, 2>> path_points_;
  size_t waypoint_idx_ = 0;
  bool final_reached_ = false;

  // Control parameters
  double speed_ = 0.5;
  double waypoint_thresh_ = 1.0;

  // Start time for delaying flight start based on drone ID
  rclcpp::Time start_time_;

  void controlLoop()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Delay flight start based on drone_id (e.g., drone 2 waits 5 sec,
    // drone 3 waits 10 sec, etc.)
    auto elapsed = this->now() - start_time_;
    rclcpp::Duration required_delay = rclcpp::Duration::from_seconds((drone_id_ - 1) * 10.0);
    if (elapsed < required_delay) {
      // Not yet time to fly: publish zero velocity to remain stationary
      geometry_msgs::msg::Twist tw;
      cmd_vel_pub_->publish(tw);
      return;
    }

    // If final waypoint reached or no path available, stop the drone
    if(final_reached_ || path_points_.empty()){
      geometry_msgs::msg::Twist tw;
      cmd_vel_pub_->publish(tw);
      return;
    }

    // If all waypoints have been processed, stop and publish hover_done
    if(waypoint_idx_ >= path_points_.size()){
      geometry_msgs::msg::Twist tw;
      cmd_vel_pub_->publish(tw);
      if(!final_reached_){
        final_reached_ = true;
        std_msgs::msg::Bool bmsg;
        bmsg.data = true;
        hover_done_pub_->publish(bmsg);
        RCLCPP_INFO(this->get_logger(),
          "Drone %d => final WP reached => hover_done published",
          drone_id_);
      }
      return;
    }

    // Drive toward current waypoint
    auto &wp = path_points_[waypoint_idx_];
    double dx = wp[0] - curr_x_;
    double dy = wp[1] - curr_y_;
    double dist = std::hypot(dx, dy);
    if(dist < waypoint_thresh_){
      // Advance to next waypoint
      waypoint_idx_++;
      return;
    }
    double vx = (dx / dist) * speed_;
    double vy = (dy / dist) * speed_;
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd_vel_pub_->publish(cmd);
  }
};

#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Create multiple controller nodes, one for each drone (IDs 1 to 5)
  std::vector<std::shared_ptr<UAV2DControllerNode>> controllers;
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  for(int i = 1; i <= 5; i++){
    auto node = std::make_shared<UAV2DControllerNode>(i);
    controllers.push_back(node);
    executor->add_node(node);
  }
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
