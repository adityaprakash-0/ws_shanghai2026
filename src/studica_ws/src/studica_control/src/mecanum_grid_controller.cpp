#include "studica_control/mecanum_grid_controller.hpp" // Ensure this path matches your include folder structure!
#include <algorithm>
#include <cmath>
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace nav2_mecanum_grid_controller
{

void MecanumGridController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    auto node = node_.lock();
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    tf_ = tf;
    plugin_name_ = name;

    // Declare fixed speeds and tolerances
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".speed_linear", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".speed_angular", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_xy_tol", rclcpp::ParameterValue(0.05));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_yaw_tol", rclcpp::ParameterValue(0.1));

    node->get_parameter(plugin_name_ + ".speed_linear", speed_linear_);
    node->get_parameter(plugin_name_ + ".speed_angular", speed_angular_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".goal_xy_tol", goal_xy_tol_);
    node->get_parameter(plugin_name_ + ".goal_yaw_tol", goal_yaw_tol_);
}

void MecanumGridController::cleanup() {}
void MecanumGridController::activate() {}
void MecanumGridController::deactivate() {}

void MecanumGridController::setPlan(const nav_msgs::msg::Path & path) { global_plan_ = path; }
void MecanumGridController::setSpeedLimit(const double& /*speed_limit*/, const bool& /*percentage*/) {}

geometry_msgs::msg::TwistStamped MecanumGridController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & /*velocity*/,
    nav2_core::GoalChecker * /*goal_checker*/)
{
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.header.stamp = rclcpp::Clock().now();
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    if (global_plan_.poses.empty()) return cmd_vel;

    double rx = pose.pose.position.x;
    double ry = pose.pose.position.y;
    double yaw_robot = tf2::getYaw(pose.pose.orientation);

    // 1. Find the target point (Carrot) based on lookahead distance
    geometry_msgs::msg::PoseStamped target_pose = global_plan_.poses.back();
    for (const auto& path_pose : global_plan_.poses) {
        double d = std::hypot(rx - path_pose.pose.position.x, ry - path_pose.pose.position.y);
        if (d > lookahead_dist_) {
            target_pose = path_pose;
            break;
        }
    }

    // 2. Obstacle Check (Only 254 Lethal triggers replan)
    unsigned int mx, my;
    if (costmap_->worldToMap(target_pose.pose.position.x, target_pose.pose.position.y, mx, my)) {
        if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
            throw nav2_core::PlannerException("Path obstructed! Requesting Global Replan.");
        }
    }

    double dist_to_goal = std::hypot(rx - global_plan_.poses.back().pose.position.x, 
                                     ry - global_plan_.poses.back().pose.position.y);

    // 3. Goal Reached Logic (Final precise rotation to the actual goal yaw)
    // Fixes the 0.006 rad/s stalling bug.
    if (dist_to_goal <= goal_xy_tol_) {
        double target_yaw = tf2::getYaw(global_plan_.poses.back().pose.orientation);
        double yaw_error = atan2(sin(target_yaw - yaw_robot), cos(target_yaw - yaw_robot));

        if (std::abs(yaw_error) > goal_yaw_tol_) {
            cmd_vel.twist.angular.z = (yaw_error > 0.0) ? speed_angular_ : -speed_angular_;
        }
        return cmd_vel; // Stop translating entirely
    }

    // ====================================================================
    // 4. SNAP-TO-GRID AUTO ALIGNMENT (Recovery / Tilted Start Fix)
    // ====================================================================
    // Find the closest exact 90-degree angle (0, PI/2, PI, -PI/2)
    double nearest_grid_yaw = std::round(yaw_robot / (M_PI / 2.0)) * (M_PI / 2.0);
    double grid_yaw_error = atan2(sin(nearest_grid_yaw - yaw_robot), cos(nearest_grid_yaw - yaw_robot));

    // If the robot is tilted off the grid, rotate in place to fix it FIRST
    if (std::abs(grid_yaw_error) > goal_yaw_tol_) {
        cmd_vel.twist.angular.z = (grid_yaw_error > 0.0) ? speed_angular_ : -speed_angular_;
        return cmd_vel; // Return immediately to prevent driving while turning
    }
    // ====================================================================

    // 5. Transform Target Vector into Robot's Local Frame
    double dx = target_pose.pose.position.x - rx;
    double dy = target_pose.pose.position.y - ry;

    double local_x = dx * cos(-yaw_robot) - dy * sin(-yaw_robot);
    double local_y = dx * sin(-yaw_robot) + dy * cos(-yaw_robot);

    // 6. Strict 4-Way Bang-Bang Control
    if (std::abs(local_x) > std::abs(local_y)) {
        // Move Forward or Backward at fixed speed
        cmd_vel.twist.linear.x = (local_x > 0.0) ? speed_linear_ : -speed_linear_;
        cmd_vel.twist.linear.y = 0.0;
    } else {
        // Strafe Left or Right at fixed speed
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = (local_y > 0.0) ? speed_linear_ : -speed_linear_;
    }

    return cmd_vel;
}

}  // namespace nav2_mecanum_grid_controller

PLUGINLIB_EXPORT_CLASS(nav2_mecanum_grid_controller::MecanumGridController, nav2_core::Controller)