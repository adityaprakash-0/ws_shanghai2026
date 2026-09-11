#include "skill23_path_collision/path_ahead_collision_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace skill23_path_collision
{

namespace
{
unsigned char cellCostAt(
  nav2_costmap_2d::Costmap2D * costmap, double wx, double wy, bool & ok)
{
  ok = false;
  if (!costmap) {
    return 0;
  }
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap->worldToMap(wx, wy, mx, my)) {
    return 0;
  }
  ok = true;
  return costmap->getCost(mx, my);
}

double shortestAngularDistance(double from, double to)
{
  double d = to - from;
  while (d > M_PI) {
    d -= 2.0 * M_PI;
  }
  while (d < -M_PI) {
    d += 2.0 * M_PI;
  }
  return d;
}

struct PathCursor
{
  size_t seg{0};       // segment start index into plan poses
  double t{0.0};       // [0,1] along segment seg → seg+1
  double along{0.0};   // arc length from plan start to this point
  double x{0.0};
  double y{0.0};
};

/** Closest point on polyline to (rx,ry); works for axis-aligned and diagonal legs. */
PathCursor closestOnPolyline(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  double rx, double ry)
{
  PathCursor best;
  if (poses.empty()) {
    return best;
  }
  if (poses.size() == 1) {
    best.x = poses[0].pose.position.x;
    best.y = poses[0].pose.position.y;
    return best;
  }

  double best_d2 = std::numeric_limits<double>::infinity();
  double cum = 0.0;
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const double x0 = poses[i].pose.position.x;
    const double y0 = poses[i].pose.position.y;
    const double x1 = poses[i + 1].pose.position.x;
    const double y1 = poses[i + 1].pose.position.y;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 1e-12) {
      t = ((rx - x0) * dx + (ry - y0) * dy) / len2;
      t = std::max(0.0, std::min(1.0, t));
    }
    const double px = x0 + t * dx;
    const double py = y0 + t * dy;
    const double d2 = (px - rx) * (px - rx) + (py - ry) * (py - ry);
    if (d2 < best_d2) {
      best_d2 = d2;
      best.seg = i;
      best.t = t;
      best.along = cum + t * std::sqrt(len2);
      best.x = px;
      best.y = py;
    }
    cum += std::sqrt(len2);
  }
  return best;
}

/** Advance cursor by `ds` meters along the plan; returns false if past end. */
bool advanceAlongPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  PathCursor & cur, double ds, double * yaw_out)
{
  if (poses.size() < 2 || ds < 0.0) {
    return false;
  }
  double remain = ds;
  while (remain > 1e-9) {
    if (cur.seg + 1 >= poses.size()) {
      return false;
    }
    const double x0 = poses[cur.seg].pose.position.x;
    const double y0 = poses[cur.seg].pose.position.y;
    const double x1 = poses[cur.seg + 1].pose.position.x;
    const double y1 = poses[cur.seg + 1].pose.position.y;
    const double seg_len = std::hypot(x1 - x0, y1 - y0);
    if (seg_len < 1e-9) {
      cur.seg += 1;
      cur.t = 0.0;
      continue;
    }
    const double left_on_seg = (1.0 - cur.t) * seg_len;
    if (remain <= left_on_seg) {
      cur.t += remain / seg_len;
      cur.along += remain;
      cur.x = x0 + cur.t * (x1 - x0);
      cur.y = y0 + cur.t * (y1 - y0);
      if (yaw_out) {
        *yaw_out = std::atan2(y1 - y0, x1 - x0);
      }
      return true;
    }
    remain -= left_on_seg;
    cur.along += left_on_seg;
    cur.seg += 1;
    cur.t = 0.0;
    cur.x = x1;
    cur.y = y1;
  }
  if (yaw_out && cur.seg + 1 < poses.size()) {
    const auto & a = poses[cur.seg].pose.position;
    const auto & b = poses[cur.seg + 1].pose.position;
    *yaw_out = std::atan2(b.y - a.y, b.x - a.x);
  }
  return true;
}
}  // namespace

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
    node, name_ + ".path_collision_mode",
    rclcpp::ParameterValue(std::string("path_ahead")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_cost_threshold",
    rclcpp::ParameterValue(254.0));
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
  // Default false: only needed when inner RPP CD is ON.
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_pause_rpp_cd_at_goal_yaw",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_goal_xy_dist", rclcpp::ParameterValue(0.18));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_collision_detection", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_debug", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_collision_use_travel_yaw", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".transform_tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(0.2));

  node->get_parameter(name_ + ".primary_controller", primary_controller_);
  node->get_parameter(name_ + ".path_collision_enabled", enabled_);
  node->get_parameter(name_ + ".path_collision_mode", mode_);
  node->get_parameter(name_ + ".path_collision_cost_threshold", cost_threshold_);
  node->get_parameter(name_ + ".path_collision_lookahead_time", lookahead_time_);
  node->get_parameter(name_ + ".path_collision_min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(name_ + ".path_collision_min_check_speed", min_check_speed_);
  node->get_parameter(name_ + ".path_collision_sample_step", sample_step_);
  node->get_parameter(name_ + ".path_collision_disable_at_goal_yaw", disable_at_goal_yaw_);
  node->get_parameter(
    name_ + ".path_collision_pause_rpp_cd_at_goal_yaw", pause_rpp_cd_at_goal_yaw_);
  node->get_parameter(name_ + ".path_collision_goal_xy_dist", goal_xy_dist_);
  node->get_parameter(name_ + ".path_collision_debug", debug_log_);
  node->get_parameter(name_ + ".path_collision_use_travel_yaw", use_travel_yaw_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".use_collision_detection", rpp_cd_desired_);
  node->get_parameter(name_ + ".rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);

  lookahead_time_ = std::max(0.0, lookahead_time_);
  min_lookahead_dist_ = std::max(0.0, min_lookahead_dist_);
  min_check_speed_ = std::max(0.0, min_check_speed_);
  sample_step_ = std::max(0.01, sample_step_);
  goal_xy_dist_ = std::max(0.0, goal_xy_dist_);
  cost_threshold_ = std::max(1.0, std::min(254.0, cost_threshold_));
  rotate_to_heading_angular_vel_ = std::max(0.05, rotate_to_heading_angular_vel_);
  if (mode_ != "realtime_footprint" && mode_ != "path_ahead") {
    RCLCPP_WARN(
      logger_,
      "[%s] unknown path_collision_mode='%s' — using path_ahead",
      name_.c_str(), mode_.c_str());
    mode_ = "path_ahead";
  }

  // path_ahead mode with RPP CD already off: pause_rpp_cd is a no-op — warn if set.
  if (pause_rpp_cd_at_goal_yaw_ && !rpp_cd_desired_) {
    RCLCPP_WARN(
      logger_,
      "[%s] path_collision_pause_rpp_cd_at_goal_yaw=true but use_collision_detection=false — "
      "pause is unused. Keep only path_collision_disable_at_goal_yaw for this mode.",
      name_.c_str());
  }

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

  inner_->configure(parent, name_, tf_, costmap_ros_);

  const double inscribed = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  const double circumscribed = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();

  RCLCPP_INFO(
    logger_,
    "[%s] collision wrapper ready mode=%s threshold=%.0f enabled=%s "
    "disable_at_goal_yaw=%s pause_rpp_cd_at_goal_yaw=%s rpp_cd_desired=%s "
    "debug=%s footprint inscribed=%.3f circumscribed=%.3f primary=%s",
    name_.c_str(), mode_.c_str(), cost_threshold_,
    enabled_ ? "true" : "false",
    disable_at_goal_yaw_ ? "true" : "false",
    pause_rpp_cd_at_goal_yaw_ ? "true" : "false",
    rpp_cd_desired_ ? "true" : "false",
    debug_log_ ? "true" : "false",
    inscribed, circumscribed, primary_controller_.c_str());
}

bool PathAheadCollisionController::isRealtimeMode() const
{
  return mode_ == "realtime_footprint";
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

PathAheadCollisionController::HitInfo PathAheadCollisionController::footprintCostHitAt(
  double x, double y, double yaw, const char * where) const
{
  HitInfo info;
  info.x = x;
  info.y = y;
  info.yaw = yaw;
  info.where = where ? where : "?";

  if (!costmap_ || !collision_checker_ || !costmap_ros_) {
    return info;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    info.detail = "pose outside local costmap window";
    return info;
  }

  bool center_ok = false;
  info.center_cost = static_cast<double>(cellCostAt(costmap_, x, y, center_ok));

  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.empty()) {
    info.detail = "empty footprint";
    return info;
  }

  const double footprint_cost = collision_checker_->footprintCostAtPose(
    x, y, yaw, footprint);
  info.footprint_cost = footprint_cost;

  if (footprint_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION)) {
    info.detail = "footprint on unknown (255) — ignored";
    return info;
  }

  if (footprint_cost < cost_threshold_) {
    return info;
  }

  info.hit = true;

  std::vector<geometry_msgs::msg::Point> oriented;
  nav2_costmap_2d::transformFootprint(x, y, yaw, footprint, oriented);
  double best_d2 = std::numeric_limits<double>::infinity();
  for (const auto & pt : oriented) {
    bool ok = false;
    const unsigned char c = cellCostAt(costmap_, pt.x, pt.y, ok);
    if (!ok) {
      continue;
    }
    if (c != nav2_costmap_2d::NO_INFORMATION &&
      static_cast<double>(c) >= cost_threshold_)
    {
      const double d2 = (pt.x - x) * (pt.x - x) + (pt.y - y) * (pt.y - y);
      if (d2 < best_d2) {
        best_d2 = d2;
        info.hit_wx = pt.x;
        info.hit_wy = pt.y;
        info.hit_cell_cost = c;
      }
    }
  }

  std::ostringstream ss;
  ss << "footprint_cost=" << static_cast<int>(footprint_cost)
     << " threshold=" << static_cast<int>(cost_threshold_)
     << " center_cost=" << static_cast<int>(info.center_cost);
  if (info.center_cost < cost_threshold_) {
    ss << " (CENTER OK — footprint edge/corner ≥ threshold)";
  } else {
    ss << " (center also ≥ threshold)";
  }
  if (best_d2 < std::numeric_limits<double>::infinity()) {
    ss << " hit_at=(" << info.hit_wx << "," << info.hit_wy << ") cell="
       << static_cast<int>(info.hit_cell_cost)
       << " from_center=" << std::sqrt(best_d2) << "m";
  }
  info.detail = ss.str();
  return info;
}

void PathAheadCollisionController::logHit(
  const HitInfo & hit, double horizon, double speed) const
{
  if (!debug_log_ && !hit.hit) {
    return;
  }
  RCLCPP_ERROR(
    logger_,
    "[%s] COLLISION REASON mode=%s where=%s along=%.2fm horizon=%.2fm speed=%.2f "
    "pose=(%.3f,%.3f) yaw=%.1f° %s",
    name_.c_str(), mode_.c_str(), hit.where.c_str(), hit.along_path_m, horizon, speed,
    hit.x, hit.y, hit.yaw * 180.0 / M_PI, hit.detail.c_str());
}

PathAheadCollisionController::HitInfo PathAheadCollisionController::lethalOnPathAhead(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double speed) const
{
  HitInfo miss;
  if (!enabled_ || global_plan_.poses.empty() || !tf_ || !costmap_ros_ || !costmap_) {
    return miss;
  }

  auto * cmap = costmap_ros_->getCostmap();
  collision_checker_->setCostmap(cmap);
  const_cast<PathAheadCollisionController *>(this)->costmap_ = cmap;

  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  const double horizon = std::max(
    lookahead_time_ * std::max(std::abs(speed), min_check_speed_),
    min_lookahead_dist_);

  HitInfo at_robot = footprintCostHitAt(
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    tf2::getYaw(robot_pose.pose.orientation), "robot_pose");
  at_robot.along_path_m = 0.0;
  if (at_robot.hit) {
    return at_robot;
  }

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
      return miss;
    }
  }

  const double rx_p = robot_in_plan.pose.position.x;
  const double ry_p = robot_in_plan.pose.position.y;
  PathCursor cursor = closestOnPolyline(global_plan_.poses, rx_p, ry_p);

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
    return miss;
  }

  // Sample from closest projection along the path for `horizon` meters.
  // Travel yaw follows segment direction (axis-aligned or diagonal).
  double checked = 0.0;
  bool first = true;
  while (checked <= horizon + 1e-9) {
    geometry_msgs::msg::PoseStamped plan_pt;
    plan_pt.header = global_plan_.header;
    plan_pt.pose.position.x = cursor.x;
    plan_pt.pose.position.y = cursor.y;
    plan_pt.pose.orientation.w = 1.0;

    double travel_yaw = 0.0;
    if (cursor.seg + 1 < global_plan_.poses.size()) {
      const auto & a = global_plan_.poses[cursor.seg].pose.position;
      const auto & b = global_plan_.poses[cursor.seg + 1].pose.position;
      travel_yaw = std::atan2(b.y - a.y, b.x - a.x);
    } else if (!global_plan_.poses.empty()) {
      travel_yaw = tf2::getYaw(global_plan_.poses.back().pose.orientation);
    }

    geometry_msgs::msg::PoseStamped local;
    tf2::doTransform(plan_pt, local, plan_to_costmap);
    const double x = local.pose.position.x;
    const double y = local.pose.position.y;

    double yaw = travel_yaw;
    if (!use_travel_yaw_) {
      // Interpolate path pose yaw on this segment.
      if (cursor.seg + 1 < global_plan_.poses.size()) {
        const double y0 = tf2::getYaw(global_plan_.poses[cursor.seg].pose.orientation);
        const double y1 = tf2::getYaw(global_plan_.poses[cursor.seg + 1].pose.orientation);
        yaw = y0 + cursor.t * shortestAngularDistance(y0, y1);
      } else {
        yaw = tf2::getYaw(global_plan_.poses.back().pose.orientation);
      }
    }
    // Rotate travel yaw into costmap frame (2D yaw of plan→costmap).
    {
      const double qx = plan_to_costmap.transform.rotation.x;
      const double qy = plan_to_costmap.transform.rotation.y;
      const double qz = plan_to_costmap.transform.rotation.z;
      const double qw = plan_to_costmap.transform.rotation.w;
      const double yaw_tf = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));
      yaw += yaw_tf;
    }

    HitInfo h = footprintCostHitAt(x, y, yaw, "path_ahead");
    h.along_path_m = checked;
    if (h.hit) {
      return h;
    }

    if (!first && checked >= horizon) {
      break;
    }
    first = false;

    double step_yaw = 0.0;
    if (!advanceAlongPath(global_plan_.poses, cursor, sample_step_, &step_yaw)) {
      break;
    }
    checked += sample_step_;
  }

  return miss;
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

geometry_msgs::msg::TwistStamped PathAheadCollisionController::rotateToGoalYawCmd(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;

  if (global_plan_.poses.empty() || !tf_) {
    return cmd;
  }

  geometry_msgs::msg::PoseStamped goal = global_plan_.poses.back();
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = global_plan_.header.frame_id;
  }

  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  if (!goal.header.frame_id.empty() &&
    goal.header.frame_id != pose.header.frame_id)
  {
    try {
      geometry_msgs::msg::PoseStamped goal_tf;
      tf_->transform(
        goal, goal_tf, pose.header.frame_id,
        tf2::durationFromSec(transform_tolerance_));
      goal_yaw = tf2::getYaw(goal_tf.pose.orientation);
    } catch (const tf2::TransformException &) {
      // Keep plan-frame yaw as best effort.
    }
  }

  const double cur = tf2::getYaw(pose.pose.orientation);
  const double err = shortestAngularDistance(cur, goal_yaw);
  const double w = std::copysign(
    std::min(std::abs(err) * 1.5, rotate_to_heading_angular_vel_), err);
  cmd.twist.angular.z = w;
  return cmd;
}

geometry_msgs::msg::TwistStamped PathAheadCollisionController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const double speed = std::hypot(velocity.linear.x, velocity.linear.y);
  const bool yaw_align = isInGoalYawAlignPhase(pose);

  // RPP CD while driving, muted only at goal yaw: Humble RPP does not apply live
  // use_collision_detection updates, so bypass RPP and rotate in place.
  if (yaw_align && pause_rpp_cd_at_goal_yaw_ && rpp_cd_desired_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "[%s] goal yaw-align — bypassing RPP (CD would stay on); rotate-in-place",
      name_.c_str());
    return rotateToGoalYawCmd(pose);
  }

  if (enabled_ && !yaw_align) {
    auto * cmap = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
    if (cmap && collision_checker_) {
      collision_checker_->setCostmap(cmap);
      costmap_ = cmap;
    }

    HitInfo hit;
    double horizon = 0.0;
    if (isRealtimeMode()) {
      hit = footprintCostHitAt(
        pose.pose.position.x, pose.pose.position.y,
        tf2::getYaw(pose.pose.orientation), "realtime_footprint");
      hit.along_path_m = 0.0;
    } else {
      horizon = std::max(
        lookahead_time_ * std::max(std::abs(speed), min_check_speed_),
        min_lookahead_dist_);
      hit = lethalOnPathAhead(pose, speed);
    }

    if (hit.hit) {
      logHit(hit, horizon, speed);
      std::ostringstream msg;
      msg << "PathAheadCollision[" << mode_ << "]: " << hit.where
          << " — " << hit.detail;
      throw nav2_core::PlannerException(msg.str());
    }
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
