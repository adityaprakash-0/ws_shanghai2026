// Copyright 2026 — skill23_rpp host node.

#include "skill23_rpp/rpp_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace skill23_rpp
{

RppNode::RppNode()
: Node("rpp")
{
  declare_parameter("robot_base_frame", std::string("base_footprint"));
  declare_parameter("global_frame", std::string("map"));
  declare_parameter("odom_topic", std::string("/odometry/filtered"));
  declare_parameter("transform_tolerance", 1.0);
  declare_parameter("controller_frequency", 20.0);
  declare_parameter("xy_goal_tolerance", 0.12);
  declare_parameter("yaw_goal_tolerance", 0.15);

  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  global_frame_ = get_parameter("global_frame").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
}

void RppNode::start()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Placeholder costmap until first /custom_nav/local_grid (3x3m @ 0.03m).
  costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
    100, 100, 0.03, -1.5, -1.5, nav2_costmap_2d::FREE_SPACE);

  controller_ = std::make_unique<
    nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>();
  controller_->configure(
    weak_from_this(),
    "",  // flat params on this node
    tf_buffer_,
    costmap_.get(),
    robot_base_frame_,
    global_frame_);
  controller_->activate();

  goal_checker_ = std::make_unique<SimpleGoalChecker>();
  goal_checker_->configure(shared_from_this());

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  enable_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/custom_nav/controller/enable", 10,
    std::bind(&RppNode::onEnable, this, std::placeholders::_1));
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/custom_nav/path", 10,
    std::bind(&RppNode::onPath, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&RppNode::onPose, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RppNode::onOdom, this, std::placeholders::_1));
  grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/custom_nav/local_grid", 10,
    std::bind(&RppNode::onGrid, this, std::placeholders::_1));
  collision_sub_ = create_subscription<std_msgs::msg::String>(
    "/custom_nav/collision", 10,
    std::bind(&RppNode::onCollision, this, std::placeholders::_1));
  goal_sub_ = create_subscription<std_msgs::msg::String>(
    "/custom_nav/goal_status", 10,
    std::bind(&RppNode::onGoalStatus, this, std::placeholders::_1));

  const double freq = get_parameter("controller_frequency").as_double();
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / freq),
    std::bind(&RppNode::tick, this));

  RCLCPP_INFO(
    get_logger(),
    "skill23_rpp started (plain node). Waiting for enable + path + local_grid.");
}

void RppNode::onEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  enabled_ = msg->data;
  if (!enabled_) {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
  }
}

void RppNode::onPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (msg->poses.empty()) {
    has_plan_ = false;
    return;
  }
  controller_->setPlan(*msg);
  has_plan_ = true;
}

void RppNode::onPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_amcl_pose_ = *msg;
  has_amcl_pose_ = true;
}

void RppNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  robot_velocity_ = msg->twist.twist;
}

void RppNode::onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  auto next = std::make_unique<nav2_costmap_2d::Costmap2D>(*msg);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    costmap_ = std::move(next);
    controller_->setCostmap(costmap_.get());
  }
}

void RppNode::onCollision(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // Payload is JSON; treat any message containing "blocked":true as blocked.
  blocked_ = msg->data.find("\"blocked\": true") != std::string::npos ||
    msg->data.find("\"blocked\":true") != std::string::npos;
}

void RppNode::onGoalStatus(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  goal_status_ = msg->data;
}

bool RppNode::lookupRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    auto tf = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
    pose.header.stamp = tf.header.stamp;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    // Fall back to last AMCL pose if TF is briefly unavailable.
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_amcl_pose_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s→%s failed: %s", global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      return false;
    }
    pose.header = last_amcl_pose_.header;
    pose.pose = last_amcl_pose_.pose.pose;
    return true;
  }
}

void RppNode::tick()
{
  geometry_msgs::msg::Twist stop;

  bool enabled = false;
  bool blocked = false;
  bool has_plan = false;
  std::string goal_status;
  geometry_msgs::msg::Twist velocity;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    enabled = enabled_;
    blocked = blocked_;
    has_plan = has_plan_;
    goal_status = goal_status_;
    velocity = robot_velocity_;
  }

  if (!enabled || blocked || !has_plan || goal_status == "GOAL_REACHED") {
    cmd_pub_->publish(stop);
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  if (!lookupRobotPose(pose)) {
    cmd_pub_->publish(stop);
    return;
  }

  try {
    auto cmd = controller_->computeVelocityCommands(
      pose, velocity, goal_checker_.get());
    cmd_pub_->publish(cmd.twist);
  } catch (const nav2_core::PlannerException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "RPP: %s", ex.what());
    cmd_pub_->publish(stop);
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "RPP exception: %s", ex.what());
    cmd_pub_->publish(stop);
  }
}

}  // namespace skill23_rpp
