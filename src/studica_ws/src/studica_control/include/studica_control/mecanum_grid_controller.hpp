// #ifndef MECANUM_GRID_CONTROLLER_HPP_
// #define MECANUM_GRID_CONTROLLER_HPP_

// #include <string>
// #include <memory>
// #include <vector>

// #include "rclcpp/rclcpp.hpp"
// #include "nav2_core/controller.hpp"
// #include "nav2_costmap_2d/footprint_collision_checker.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/twist_stamped.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "pluginlib/class_loader.hpp"
// #include "pluginlib/class_list_macros.hpp"
// #include "angles/angles.h"
// #include "tf2/utils.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// namespace nav2_mecanum_grid_controller
// {

// class MecanumGridController : public nav2_core::Controller
// {
// public:
//     MecanumGridController() = default;
//     ~MecanumGridController() override = default;

//     // --- Standard Nav2 Controller Interface ---
//     void configure(
//         const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//         std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
//         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

//     void cleanup() override;
//     void activate() override;
//     void deactivate() override;

//     void setPlan(const nav_msgs::msg::Path & path) override;

//     geometry_msgs::msg::TwistStamped computeVelocityCommands(
//         const geometry_msgs::msg::PoseStamped & pose,
//         const geometry_msgs::msg::Twist & velocity,
//         nav2_core::GoalChecker * goal_checker) override;

//     void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

// private:
//     // --- Custom Mecanum Logic ---
//     geometry_msgs::msg::PoseStamped getLookaheadPoint(const double & rx, const double & ry);
    
//     // Checks the Local Costmap to see if the immediate path is blocked
//     bool isCollisionFree(const geometry_msgs::msg::PoseStamped & robot_pose,
//                          const geometry_msgs::msg::TwistStamped & cmd_vel);

//     std::string name_;
//     std::shared_ptr<tf2_ros::Buffer> tf_;
//     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
//     nav2_costmap_2d::Costmap2D * costmap_;
//     rclcpp::Logger logger_{rclcpp::get_logger("MecanumGridController")};
    
//     // THE FIX: Add the <nav2_costmap_2d::Costmap2D *> template declaration
//     std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> collision_checker_;

//     nav_msgs::msg::Path global_plan_;

//     // --- ROS 2 Parameters ---
//     double kp_x_, kp_y_, kp_theta_;
//     double max_vel_x_, max_vel_y_, max_vel_theta_;
//     double min_vel_x_, min_vel_y_, min_vel_theta_;
//     double lookahead_dist_;
//     double xy_tolerance_, yaw_tolerance_;
//     double sim_time_; 
// };

// } // namespace nav2_mecanum_grid_controller

// #endif // MECANUM_GRID_CONTROLLER_HPP_
#pragma once

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_mecanum_grid_controller
{

class MecanumGridController : public nav2_core::Controller
{
public:
    MecanumGridController() = default;
    ~MecanumGridController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    void setPlan(const nav_msgs::msg::Path & path) override;
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D * costmap_;
    nav_msgs::msg::Path global_plan_;

    // Cleaned up parameters: Fixed speeds and tolerances instead of P-gains
    double speed_linear_;
    double speed_angular_;
    double lookahead_dist_;
    double goal_xy_tol_;
    double goal_yaw_tol_;
};

}  // namespace nav2_mecanum_grid_controller