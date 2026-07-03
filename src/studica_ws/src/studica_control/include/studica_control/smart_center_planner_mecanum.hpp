#pragma once

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// FIXED: Added "_mecanum" to match the YAML/XML
namespace nav2_smart_center_planner_mecanum
{

struct AStarNode {
    unsigned int index;
    float f_score;
    bool operator>(const AStarNode& other) const {
        return f_score > other.f_score;
    }
};

class SmartCenterPlanner : public nav2_core::GlobalPlanner
{
public:
    SmartCenterPlanner() = default;
    ~SmartCenterPlanner() = default;

    // Nav2 Core overrides
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;

private:
    // Helper functions
    std::vector<unsigned int> getNeighbors(unsigned int current_index);
    double getHeuristic(unsigned int current_index, unsigned int goal_index);
    void smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path);
    void calculateOrientations(std::vector<geometry_msgs::msg::PoseStamped>& path, const geometry_msgs::msg::PoseStamped& goal);
    
    // Costmap translation helpers
    bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
    void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy);

    // Member variables
    std::string name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D * costmap_;
    rclcpp::Logger logger_{rclcpp::get_logger("SmartCenterPlanner")};

    unsigned int width_;
    unsigned int height_;

    // Plugin parameters
    double center_weight_;
    double turn_penalty_;
    int smooth_iterations_;
};

}  // namespace nav2_smart_center_planner_mecanum