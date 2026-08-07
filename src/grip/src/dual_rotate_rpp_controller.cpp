// Copyright 2026 — grip dual-speed rotate-to-heading wrapper around Nav2 RPP.

#include "grip/dual_rotate_rpp_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace grip
{

void DualRotateRppController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Configure stock RPP first (declares all standard params).
  RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("DualRotateRppController: failed to lock node");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".rotate_to_heading_angular_vel_fast", rclcpp::ParameterValue(0.40));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".rotate_to_heading_angular_vel_slow", rclcpp::ParameterValue(0.12));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".rotate_to_heading_fast_angle", rclcpp::ParameterValue(0.50));

  node->get_parameter(name + ".rotate_to_heading_angular_vel_fast",
    rotate_to_heading_angular_vel_fast_);
  node->get_parameter(name + ".rotate_to_heading_angular_vel_slow",
    rotate_to_heading_angular_vel_slow_);
  node->get_parameter(name + ".rotate_to_heading_fast_angle",
    rotate_to_heading_fast_angle_);

  // Parent rotateToHeading uses rotate_to_heading_angular_vel_; seed it to fast.
  rotate_to_heading_angular_vel_ = rotate_to_heading_angular_vel_fast_;

  RCLCPP_INFO(
    logger_,
    "DualRotateRppController [%s]: fast=%.3f rad/s (err>%.3f rad), "
    "slow=%.3f rad/s (min_angle < err ≤ fast_angle)",
    name.c_str(),
    rotate_to_heading_angular_vel_fast_,
    rotate_to_heading_fast_angle_,
    rotate_to_heading_angular_vel_slow_);
}

geometry_msgs::msg::TwistStamped DualRotateRppController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // Pick fast vs slow target ω from current yaw error, then let stock RPP
  // run rotateToHeading (accel limit + stop-before-overshoot) with that target.
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (goal_checker) {
      geometry_msgs::msg::Pose pose_tolerance;
      geometry_msgs::msg::Twist vel_tolerance;
      if (goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
        goal_dist_tol_ = pose_tolerance.position.x;
      }
    }

    try {
      auto transformed_plan = transformGlobalPlan(pose);
      if (!transformed_plan.poses.empty()) {
        double lookahead_dist = getLookAheadDistance(velocity);
        auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);

        double angle_err = 0.0;
        if (shouldRotateToGoalHeading(carrot_pose)) {
          angle_err = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
        } else {
          angle_err = std::atan2(
            carrot_pose.pose.position.y, carrot_pose.pose.position.x);
        }

        const double abs_err = std::fabs(angle_err);
        if (use_rotate_to_heading_ && abs_err > rotate_to_heading_min_angle_) {
          rotate_to_heading_angular_vel_ =
            (abs_err > rotate_to_heading_fast_angle_)
            ? rotate_to_heading_angular_vel_fast_
            : rotate_to_heading_angular_vel_slow_;
        }
      }
    } catch (const std::exception &) {
      // Fall through — parent will throw the same if plan is invalid.
    }
  }

  return RegulatedPurePursuitController::computeVelocityCommands(
    pose, velocity, goal_checker);
}

}  // namespace grip

PLUGINLIB_EXPORT_CLASS(grip::DualRotateRppController, nav2_core::Controller)
