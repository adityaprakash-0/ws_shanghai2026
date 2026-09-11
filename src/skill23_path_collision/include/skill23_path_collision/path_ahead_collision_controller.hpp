#ifndef SKILL23_PATH_COLLISION__PATH_AHEAD_COLLISION_CONTROLLER_HPP_
#define SKILL23_PATH_COLLISION__PATH_AHEAD_COLLISION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace skill23_path_collision
{

/**
 * @brief Thin nav2_core::Controller wrapper around an inner controller (RPP).
 *
 * Modes (path_collision_mode):
 *  - "path_ahead": sample path within lookahead; abort if footprint ≥ threshold
 *  - "realtime_footprint": each cycle, check CURRENT robot footprint on the
 *    live local costmap only (no path simulation). Abort if cost ≥ threshold
 *    (typically 253 inscribed). Same PlannerException → BT recovery as before.
 *
 * path_collision_disable_at_goal_yaw:
 *  Skips this wrapper's path-ahead / realtime checks near goal XY (yaw align).
 *  Does NOT change inner RPP collision detection by itself.
 *
 * path_collision_pause_rpp_cd_at_goal_yaw:
 *  Only needed when inner RPP use_collision_detection=true. Near goal XY, bypass
 *  RPP and rotate in place to goal yaw (Humble RPP ignores live CD param changes).
 */
class PathAheadCollisionController : public nav2_core::Controller
{
public:
  PathAheadCollisionController() = default;
  ~PathAheadCollisionController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  struct HitInfo
  {
    bool hit{false};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double footprint_cost{0.0};
    double center_cost{0.0};
    double hit_wx{0.0};
    double hit_wy{0.0};
    unsigned char hit_cell_cost{0};
    double along_path_m{0.0};
    std::string where{"?"};  // robot_pose | path_ahead | realtime_footprint
    std::string detail;
  };

  HitInfo footprintCostHitAt(
    double x, double y, double yaw, const char * where) const;

  HitInfo lethalOnPathAhead(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double speed) const;

  bool isInGoalYawAlignPhase(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  /** Rotate-in-place toward plan goal yaw (used when RPP CD must be muted). */
  geometry_msgs::msg::TwistStamped rotateToGoalYawCmd(
    const geometry_msgs::msg::PoseStamped & pose) const;

  void logHit(const HitInfo & hit, double horizon, double speed) const;

  bool isRealtimeMode() const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("PathAheadCollisionController")};
  rclcpp::Clock::SharedPtr clock_;

  pluginlib::ClassLoader<nav2_core::Controller> plugin_loader_{
    "nav2_core", "nav2_core::Controller"};
  nav2_core::Controller::Ptr inner_;

  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  nav_msgs::msg::Path global_plan_;

  bool enabled_{true};
  bool disable_at_goal_yaw_{true};
  // When near goal XY: bypass RPP (Humble cannot live-toggle use_collision_detection).
  bool pause_rpp_cd_at_goal_yaw_{false};
  bool rpp_cd_desired_{false};
  bool debug_log_{true};
  bool use_travel_yaw_{true};
  std::string mode_{"path_ahead"};  // path_ahead | realtime_footprint
  // Abort when footprint max cost ≥ this (253=inscribed, 254=lethal).
  double cost_threshold_{254.0};
  std::string primary_controller_;
  double lookahead_time_{1.0};
  double min_lookahead_dist_{0.15};
  double min_check_speed_{0.05};
  double sample_step_{0.05};
  double transform_tolerance_{0.5};
  double goal_xy_dist_{0.18};
  double rotate_to_heading_angular_vel_{0.2};
};

}  // namespace skill23_path_collision

#endif  // SKILL23_PATH_COLLISION__PATH_AHEAD_COLLISION_CONTROLLER_HPP_
