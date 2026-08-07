// Copyright 2026 — skill23_rpp
// Minimal GoalChecker without Costmap2DROS / LifecycleNode.

#ifndef SKILL23_RPP__SIMPLE_GOAL_CHECKER_HPP_
#define SKILL23_RPP__SIMPLE_GOAL_CHECKER_HPP_

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

namespace skill23_rpp
{

class SimpleGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & /*parent*/,
    const std::string & /*plugin_name*/,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/) override
  {
  }

  void configure(const rclcpp::Node::SharedPtr & node)
  {
    if (!node->has_parameter("xy_goal_tolerance")) {
      node->declare_parameter("xy_goal_tolerance", 0.12);
    }
    if (!node->has_parameter("yaw_goal_tolerance")) {
      node->declare_parameter("yaw_goal_tolerance", 0.15);
    }
    xy_goal_tolerance_ = node->get_parameter("xy_goal_tolerance").as_double();
    yaw_goal_tolerance_ = node->get_parameter("yaw_goal_tolerance").as_double();
    xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
  }

  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & /*velocity*/) override
  {
    const double dx = query_pose.position.x - goal_pose.position.x;
    const double dy = query_pose.position.y - goal_pose.position.y;
    if ((dx * dx + dy * dy) > xy_goal_tolerance_sq_) {
      return false;
    }
    double dyaw = tf2::getYaw(query_pose.orientation) - tf2::getYaw(goal_pose.orientation);
    while (dyaw > M_PI) {dyaw -= 2.0 * M_PI;}
    while (dyaw < -M_PI) {dyaw += 2.0 * M_PI;}
    return std::fabs(dyaw) <= yaw_goal_tolerance_;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override
  {
    // RPP uses pose_tolerance.position.x as goal_dist_tol_
    pose_tolerance.position.x = xy_goal_tolerance_;
    pose_tolerance.position.y = xy_goal_tolerance_;
    pose_tolerance.position.z = 0.0;
    pose_tolerance.orientation.w = 1.0;
    vel_tolerance = geometry_msgs::msg::Twist();
    return true;
  }

private:
  double xy_goal_tolerance_{0.12};
  double yaw_goal_tolerance_{0.15};
  double xy_goal_tolerance_sq_{0.0144};
};

}  // namespace skill23_rpp

#endif  // SKILL23_RPP__SIMPLE_GOAL_CHECKER_HPP_
