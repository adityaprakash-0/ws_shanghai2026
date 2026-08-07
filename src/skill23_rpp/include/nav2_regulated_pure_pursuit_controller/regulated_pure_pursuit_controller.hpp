// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// skill23_rpp: vendored Nav2 Humble RPP as a plain rclcpp::Node helper
// (no LifecycleNode / Costmap2DROS / pluginlib). Stock rotate-to-heading only.

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_regulated_pure_pursuit_controller
{

/**
 * @class RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller (standalone, non-plugin)
 */
class RegulatedPurePursuitController
{
public:
  RegulatedPurePursuitController() = default;
  ~RegulatedPurePursuitController() = default;

  void configure(
    const rclcpp::Node::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    nav2_costmap_2d::Costmap2D * costmap,
    const std::string & robot_base_frame,
    const std::string & global_frame);

  void cleanup();
  void activate();
  void deactivate();

  /** Update costmap pointer after /custom_nav/local_grid refresh. */
  void setCostmap(nav2_costmap_2d::Costmap2D * costmap);

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker);

  void setPlan(const nav_msgs::msg::Path & path);

  void setSpeedLimit(const double & speed_limit, const bool & percentage);

protected:
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose);

  bool transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  double getLookAheadDistance(const geometry_msgs::msg::Twist &);

  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path);

  bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & carrot_pose);

  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);

  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped &,
    const double &, const double &,
    const double &);

  bool inCollision(
    const double & x,
    const double & y,
    const double & theta);

  double costAtPose(const double & x, const double & y);

  double approachVelocityScalingFactor(
    const nav_msgs::msg::Path & path
  ) const;

  void applyApproachVelocityScaling(
    const nav_msgs::msg::Path & path,
    double & linear_vel
  ) const;

  void applyConstraints(
    const double & curvature, const geometry_msgs::msg::Twist & speed,
    const double & pose_cost, const nav_msgs::msg::Path & path,
    double & linear_vel, double & sign);

  static geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double r);

  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

  double findVelocitySignChange(const nav_msgs::msg::Path & transformed_plan);

  double getCostmapMaxExtent() const;

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::string robot_base_frame_;
  std::string global_frame_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::vector<geometry_msgs::msg::Point> robot_footprint_;
  double inscribed_radius_{0.15};
  double circumscribed_radius_{0.22};
  rclcpp::Logger logger_{rclcpp::get_logger("RegulatedPurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  double desired_linear_vel_, base_desired_linear_vel_;
  double lookahead_dist_;
  double rotate_to_heading_angular_vel_;
  double max_lookahead_dist_;
  double min_lookahead_dist_;
  double lookahead_time_;
  bool use_velocity_scaled_lookahead_dist_;
  tf2::Duration transform_tolerance_;
  double min_approach_linear_velocity_;
  double approach_velocity_scaling_dist_;
  double control_duration_;
  double max_allowed_time_to_collision_up_to_carrot_;
  bool use_collision_detection_;
  bool use_regulated_linear_velocity_scaling_;
  bool use_cost_regulated_linear_velocity_scaling_;
  double cost_scaling_dist_;
  double cost_scaling_gain_;
  double inflation_cost_scaling_factor_;
  double regulated_linear_scaling_min_radius_;
  double regulated_linear_scaling_min_speed_;
  bool use_rotate_to_heading_;
  double max_angular_accel_;
  double rotate_to_heading_min_angle_;
  double goal_dist_tol_;
  bool allow_reversing_;
  double max_robot_pose_search_dist_;
  bool use_interpolation_;

  nav_msgs::msg::Path global_plan_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr carrot_arc_pub_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
