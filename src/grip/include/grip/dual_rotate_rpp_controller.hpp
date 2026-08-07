// Copyright 2026 — grip dual-speed rotate-to-heading wrapper around Nav2 RPP.
// Revert: remove this plugin + restore nav2_grip.yaml FollowPath plugin to stock RPP.

#ifndef GRIP__DUAL_ROTATE_RPP_CONTROLLER_HPP_
#define GRIP__DUAL_ROTATE_RPP_CONTROLLER_HPP_

#include <string>
#include <memory>

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

namespace grip
{

/**
 * @brief Nav2 controller plugin: stock RPP + dual in-place rotate speeds.
 *
 * |yaw_error| > rotate_to_heading_fast_angle  → rotate_to_heading_angular_vel_fast
 * rotate_to_heading_min_angle < |yaw| ≤ fast  → rotate_to_heading_angular_vel_slow
 * |yaw| ≤ min_angle                           → normal Pure Pursuit (no in-place spin)
 */
class DualRotateRppController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  DualRotateRppController() = default;
  ~DualRotateRppController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

private:
  double rotate_to_heading_angular_vel_fast_{0.40};
  double rotate_to_heading_angular_vel_slow_{0.12};
  double rotate_to_heading_fast_angle_{0.50};
};

}  // namespace grip

#endif  // GRIP__DUAL_ROTATE_RPP_CONTROLLER_HPP_
