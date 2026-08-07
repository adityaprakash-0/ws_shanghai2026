// Copyright 2026 — skill23_rpp host node (AMCL-style plain rclcpp::Node).

#ifndef SKILL23_RPP__RPP_NODE_HPP_
#define SKILL23_RPP__RPP_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "skill23_rpp/simple_goal_checker.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace skill23_rpp
{

class RppNode : public rclcpp::Node
{
public:
  RppNode();
  void start();

private:
  void onEnable(const std_msgs::msg::Bool::SharedPtr msg);
  void onPath(const nav_msgs::msg::Path::SharedPtr msg);
  void onPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onCollision(const std_msgs::msg::String::SharedPtr msg);
  void onGoalStatus(const std_msgs::msg::String::SharedPtr msg);
  void tick();

  bool lookupRobotPose(geometry_msgs::msg::PoseStamped & pose);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::unique_ptr<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>
  controller_;
  std::unique_ptr<SimpleGoalChecker> goal_checker_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr collision_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex state_mutex_;
  bool enabled_{false};
  bool blocked_{false};
  bool has_plan_{false};
  std::string goal_status_{"NOT_REACHED"};
  geometry_msgs::msg::Twist robot_velocity_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_amcl_pose_;
  bool has_amcl_pose_{false};

  std::string robot_base_frame_;
  std::string global_frame_;
  std::string odom_topic_;
  double transform_tolerance_{1.0};
};

}  // namespace skill23_rpp

#endif  // SKILL23_RPP__RPP_NODE_HPP_
