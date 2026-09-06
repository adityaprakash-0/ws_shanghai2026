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
 * @brief Thin nav2_core::Controller wrapper.
 *
 * Loads an inner controller (stock RPP by default) and, each cycle, checks the
 * footprint along the current path for ~lookahead_time of travel on the local
 * costmap. Lethal (254) → throw → FollowPath abort → BT recovery/replan.
 * Does not treat 253 as collision.
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
  bool footprintLethalAt(
    double x, double y, double yaw) const;

  /** @return true if lethal on path within horizon (should abort). */
  bool lethalOnPathAhead(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double speed) const;

  /**
   * True when robot is within goal XY of the path end (RPP final yaw-align
   * / rotate-to-goal-heading zone). Path-ahead collision is skipped here so
   * in-place yaw settle is not aborted; re-enabled automatically on the next
   * plan / when leaving the goal XY window.
   */
  bool isInGoalYawAlignPhase(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

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
  std::string primary_controller_;
  double lookahead_time_{1.0};
  double min_lookahead_dist_{0.15};
  double min_check_speed_{0.05};
  double sample_step_{0.05};
  double transform_tolerance_{0.5};
  double goal_xy_dist_{0.18};
};

}  // namespace skill23_path_collision

#endif  // SKILL23_PATH_COLLISION__PATH_AHEAD_COLLISION_CONTROLLER_HPP_
