#ifndef SMART_CENTER_PLANNER_HPP_
#define SMART_CENTER_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_smart_center_planner
{

// A simple structure to hold A* node data
struct AStarNode {
    unsigned int index;
    float f_score;
    
    // Priority queue needs to sort by lowest f_score
    bool operator>(const AStarNode& other) const {
        return f_score > other.f_score;
    }
};

class SmartCenterPlanner : public nav2_core::GlobalPlanner
{
public:
    SmartCenterPlanner() = default;
    ~SmartCenterPlanner() = default;

    // --- Nav2 Lifecycle Methods ---
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    // --- Core Planning Method ---
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;

private:
    // Helper Methods
    bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
    void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy);
    std::vector<unsigned int> getNeighbors(unsigned int current_index);
    double getHeuristic(unsigned int current_index, unsigned int goal_index);
    void smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path);
    void calculateOrientations(std::vector<geometry_msgs::msg::PoseStamped>& path, const geometry_msgs::msg::PoseStamped& goal);

    // Member Variables
    std::string name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D * costmap_;
    rclcpp::Logger logger_{rclcpp::get_logger("SmartCenterPlanner")};

    // Grid properties
    unsigned int width_;
    unsigned int height_;
    
    // Configurable Parameters
    double center_weight_;     // How strongly to push to the valley center
    int smooth_iterations_;    // Number of smoothing passes
    double turn_penalty_;      // Small penalty to discourage unnecessary wiggles
};

}  // namespace nav2_smart_center_planner

#endif  // SMART_CENTER_PLANNER_HPP_