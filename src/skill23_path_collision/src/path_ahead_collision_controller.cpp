#include "skill23_path_collision/path_ahead_collision_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace skill23_path_collision
{

void PathAheadCollisionController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("PathAheadCollisionController: weak node lock failed");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".primary_controller",
    rclcpp::ParameterValue(
      std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_lookahead_time", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_min_lookahead_dist", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_min_check_speed", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_sample_step", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_disable_at_goal_yaw", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_goal_xy_dist", rclcpp::ParameterValue(0.18));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".transform_tolerance", rclcpp::ParameterValue(0.5));

  node->get_parameter(name_ + ".primary_controller", primary_controller_);
  node->get_parameter(name_ + ".path_collision_enabled", enabled_);
  node->get_parameter(name_ + ".path_collision_lookahead_time", lookahead_time_);
  node->get_parameter(name_ + ".path_collision_min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(name_ + ".path_collision_min_check_speed", min_check_speed_);
  node->get_parameter(name_ + ".path_collision_sample_step", sample_step_);
  node->get_parameter(name_ + ".path_collision_disable_at_goal_yaw", disable_at_goal_yaw_);
  node->get_parameter(name_ + ".path_collision_goal_xy_dist", goal_xy_dist_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);

  lookahead_time_ = std::max(0.0, lookahead_time_);
  min_lookahead_dist_ = std::max(0.0, min_lookahead_dist_);
  min_check_speed_ = std::max(0.0, min_check_speed_);
  sample_step_ = std::max(0.01, sample_step_);
  goal_xy_dist_ = std::max(0.0, goal_xy_dist_);

  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_);
  collision_checker_->setCostmap(costmap_);

  try {
    inner_ = plugin_loader_.createSharedInstance(primary_controller_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      logger_, "Failed to load primary controller '%s': %s",
      primary_controller_.c_str(), ex.what());
    throw;
  }

  // Same plugin name so inner RPP keeps reading FollowPath.* params.
  inner_->configure(parent, name_, tf_, costmap_ros_);

  RCLCPP_INFO(
    logger_,
    "[%s] path-ahead collision wrapper ready (enabled=%s lookahead=%.2fs min_dist=%.2f "
    "disable_at_goal_yaw=%s goal_xy=%.2f primary=%s)",
    name_.c_str(), enabled_ ? "true" : "false", lookahead_time_, min_lookahead_dist_,
    disable_at_goal_yaw_ ? "true" : "false", goal_xy_dist_,
    primary_controller_.c_str());
}

void PathAheadCollisionController::cleanup()
{
  if (inner_) {
    inner_->cleanup();
  }
  collision_checker_.reset();
  inner_.reset();
}

void PathAheadCollisionController::activate()
{
  if (inner_) {
    inner_->activate();
  }
}

void PathAheadCollisionController::deactivate()
{
  if (inner_) {
    inner_->deactivate();
  }
}

void PathAheadCollisionController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (inner_) {
    inner_->setPlan(path);
  }
}

void PathAheadCollisionController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  if (inner_) {
    inner_->setSpeedLimit(speed_limit, percentage);
  }
}

bool PathAheadCollisionController::footprintLethalAt(
  double x, double y, double yaw) const
{
  if (!costmap_ || !collision_checker_) {
    return false;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    // Outside local window: not a hard collision for this checker.
    return false;
  }

  const auto footprint = costmap_ros_->getRobotFootprint();
  const double footprint_cost = collision_checker_->footprintCostAtPose(
    x, y, yaw, footprint);

  if (footprint_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION)) {
    return false;
  }
  // Hard abort only on lethal walls (254+), not inscribed inflation (253).
  return footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

bool PathAheadCollisionController::lethalOnPathAhead(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double speed) const
{
  if (!enabled_ || global_plan_.poses.empty() || !tf_ || !costmap_ros_ || !costmap_) {
    return false;
  }

  auto * cmap = costmap_ros_->getCostmap();
  collision_checker_->setCostmap(cmap);
  const_cast<PathAheadCollisionController *>(this)->costmap_ = cmap;

  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  const double horizon = std::max(
    lookahead_time_ * std::max(std::abs(speed), min_check_speed_),
    min_lookahead_dist_);

  if (footprintLethalAt(
      robot_pose.pose.position.x, robot_pose.pose.position.y,
      tf2::getYaw(robot_pose.pose.orientation)))
  {
    return true;
  }

  // One TF: robot into plan frame to find closest index without transforming
  // the whole path every cycle.
  geometry_msgs::msg::PoseStamped robot_in_plan = robot_pose;
  if (robot_pose.header.frame_id != global_plan_.header.frame_id) {
    try {
      tf_->transform(
        robot_pose, robot_in_plan, global_plan_.header.frame_id,
        tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "[%s] path-ahead robot TF failed (%s); skipping check",
        name_.c_str(), ex.what());
      return false;
    }
  }

  const double rx_p = robot_in_plan.pose.position.x;
  const double ry_p = robot_in_plan.pose.position.y;
  size_t closest = 0;
  double closest_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const auto & p = global_plan_.poses[i].pose.position;
    const double dx = p.x - rx_p;
    const double dy = p.y - ry_p;
    const double d2 = dx * dx + dy * dy;
    if (d2 < closest_d2) {
      closest_d2 = d2;
      closest = i;
    }
  }

  geometry_msgs::msg::TransformStamped plan_to_costmap;
  try {
    plan_to_costmap = tf_->lookupTransform(
      costmap_frame, global_plan_.header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "[%s] path-ahead plan TF failed (%s); skipping check",
      name_.c_str(), ex.what());
    return false;
  }

  double traveled = 0.0;
  double last_x = 0.0;
  double last_y = 0.0;
  double last_yaw = 0.0;
  bool have_last = false;

  for (size_t i = closest; i < global_plan_.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped local;
    tf2::doTransform(global_plan_.poses[i], local, plan_to_costmap);
    const double x = local.pose.position.x;
    const double y = local.pose.position.y;
    const double yaw = tf2::getYaw(local.pose.orientation);

    if (!have_last) {
      if (footprintLethalAt(x, y, yaw)) {
        return true;
      }
      last_x = x;
      last_y = y;
      last_yaw = yaw;
      have_last = true;
      continue;
    }

    const double seg = std::hypot(x - last_x, y - last_y);
    if (seg < 1e-6) {
      continue;
    }

    const int n = std::max(1, static_cast<int>(std::ceil(seg / sample_step_)));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n);
      const double sx = last_x + t * (x - last_x);
      const double sy = last_y + t * (y - last_y);
      traveled += seg / static_cast<double>(n);
      if (footprintLethalAt(sx, sy, yaw)) {
        return true;
      }
      if (traveled >= horizon) {
        return false;
      }
    }

    last_x = x;
    last_y = y;
    last_yaw = yaw;
  }

  (void)last_yaw;
  return false;
}

bool PathAheadCollisionController::isInGoalYawAlignPhase(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  if (!disable_at_goal_yaw_ || global_plan_.poses.empty() || !tf_) {
    return false;
  }

  geometry_msgs::msg::PoseStamped goal = global_plan_.poses.back();
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = global_plan_.header.frame_id;
  }
  if (goal.header.frame_id.empty()) {
    return false;
  }

  double gx = goal.pose.position.x;
  double gy = goal.pose.position.y;

  if (goal.header.frame_id != robot_pose.header.frame_id) {
    try {
      geometry_msgs::msg::PoseStamped goal_tf;
      tf_->transform(
        goal, goal_tf, robot_pose.header.frame_id,
        tf2::durationFromSec(transform_tolerance_));
      gx = goal_tf.pose.position.x;
      gy = goal_tf.pose.position.y;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "[%s] goal-yaw gate TF failed (%s); keeping collision ON",
        name_.c_str(), ex.what());
      return false;
    }
  }

  const double dist = std::hypot(
    gx - robot_pose.pose.position.x,
    gy - robot_pose.pose.position.y);
  return dist <= goal_xy_dist_;
}

geometry_msgs::msg::TwistStamped PathAheadCollisionController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const double speed = std::hypot(velocity.linear.x, velocity.linear.y);
  const bool yaw_align = isInGoalYawAlignPhase(pose);

  // Final in-place yaw align at goal: skip path-ahead check so spinning to
  // goal orientation is not aborted. Mid-path tracking still checks. After
  // GoalChecker succeeds, FollowPath ends; the next setPlan is far from the
  // new goal end so checking is ON again automatically.
  if (enabled_ && !yaw_align && lethalOnPathAhead(pose, speed)) {
    throw nav2_core::PlannerException(
      "PathAheadCollision: footprint would hit lethal (254) on path within lookahead");
  }
  if (enabled_ && yaw_align) {
    RCLCPP_DEBUG_THROTTLE(
      logger_, *clock_, 2000,
      "[%s] path-ahead collision paused (goal XY / yaw-align)",
      name_.c_str());
  }

  return inner_->computeVelocityCommands(pose, velocity, goal_checker);
}

}  // namespace skill23_path_collision

PLUGINLIB_EXPORT_CLASS(
  skill23_path_collision::PathAheadCollisionController,
  nav2_core::Controller)
