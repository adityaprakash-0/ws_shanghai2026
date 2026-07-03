#include "studica_control/smart_center_planner_mecanum.hpp" // Ensure this path matches your setup!
#include <cmath>
#include <queue>
#include <algorithm>
#include <unordered_map>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

// FIXED: Added "_mecanum" to match the YAML/XML
namespace nav2_smart_center_planner_mecanum
{

void SmartCenterPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    auto node = parent.lock();

    nav2_util::declare_parameter_if_not_declared(node, name_ + ".center_weight", rclcpp::ParameterValue(4.0));
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".turn_penalty", rclcpp::ParameterValue(50.0));
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".smooth_iterations", rclcpp::ParameterValue(0));

    node->get_parameter(name_ + ".center_weight", center_weight_);
    node->get_parameter(name_ + ".turn_penalty", turn_penalty_);
    node->get_parameter(name_ + ".smooth_iterations", smooth_iterations_);

    RCLCPP_INFO(logger_, "Smart Center Planner Configured: weight=%.2f, turn_penalty=%.2f, smooth_iter=%d", 
                center_weight_, turn_penalty_, smooth_iterations_);
}

void SmartCenterPlanner::cleanup() { RCLCPP_INFO(logger_, "Cleaning up..."); }
void SmartCenterPlanner::activate() { RCLCPP_INFO(logger_, "Activating..."); }
void SmartCenterPlanner::deactivate() { RCLCPP_INFO(logger_, "Deactivating..."); }

nav_msgs::msg::Path SmartCenterPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = costmap_ros_->getGlobalFrameID();

    width_ = costmap_->getSizeInCellsX();
    height_ = costmap_->getSizeInCellsY();

    unsigned int start_x, start_y, goal_x, goal_y;
    if (!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
        !worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        RCLCPP_WARN(logger_, "Start or Goal is out of bounds!");
        return path;
    }

    unsigned int start_index = costmap_->getIndex(start_x, start_y);
    unsigned int goal_index = costmap_->getIndex(goal_x, goal_y);

    if (start_index == goal_index) {
        path.poses.push_back(start);
        path.poses.push_back(goal);
        return path;
    }

    std::unordered_map<unsigned int, float> g_score;
    std::unordered_map<unsigned int, unsigned int> came_from;
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

    g_score[start_index] = 0.0;
    open_set.push({start_index, static_cast<float>(getHeuristic(start_index, goal_index))});

    bool found_goal = false;

    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();

        if (current.index == goal_index) {
            found_goal = true;
            break;
        }

        unsigned int cx, cy;
        costmap_->indexToCells(current.index, cx, cy);

        for (unsigned int neighbor_index : getNeighbors(current.index)) {
            unsigned int nx, ny;
            costmap_->indexToCells(neighbor_index, nx, ny);
            unsigned char cost = costmap_->getCost(nx, ny);

            if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
                continue; 
            }

            float step_cost = 1.0f;
            float penalty = (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) ? 
                            (25.0f * center_weight_) : ((static_cast<float>(cost) / 252.0f) * center_weight_);

            float turn_cost = 0.0f;
            if (came_from.count(current.index)) {
                unsigned int px, py;
                costmap_->indexToCells(came_from[current.index], px, py);
                
                int dx_current = (int)cx - (int)px;
                int dy_current = (int)cy - (int)py;
                int dx_next = (int)nx - (int)cx;
                int dy_next = (int)ny - (int)cy;

                if (dx_current != dx_next || dy_current != dy_next) {
                    turn_cost = turn_penalty_;
                }
            }

            float tentative_g = g_score[current.index] + step_cost + penalty + turn_cost;

            if (!g_score.count(neighbor_index) || tentative_g < g_score[neighbor_index]) {
                came_from[neighbor_index] = current.index;
                g_score[neighbor_index] = tentative_g;
                float f_score = tentative_g + getHeuristic(neighbor_index, goal_index);
                open_set.push({neighbor_index, f_score});
            }
        }
    }

    if (!found_goal) {
        RCLCPP_WARN(logger_, "Failed to find path!");
        return path;
    }

    std::vector<unsigned int> reversed_path;
    unsigned int curr = goal_index;
    while (came_from.count(curr)) {
        reversed_path.push_back(curr);
        curr = came_from[curr];
    }
    reversed_path.push_back(start_index);
    std::reverse(reversed_path.begin(), reversed_path.end());

    for (unsigned int idx : reversed_path) {
        unsigned int mx, my;
        costmap_->indexToCells(idx, mx, my);
        double wx, wy;
        mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();
        pose.header.stamp = path.header.stamp;
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;
        path.poses.push_back(pose);
    }

    if (smooth_iterations_ > 0) {
        smoothPath(path.poses);
    }

    calculateOrientations(path.poses, goal);
    return path;
}

std::vector<unsigned int> SmartCenterPlanner::getNeighbors(unsigned int current_index)
{
    std::vector<unsigned int> neighbors;
    unsigned int mx, my;
    costmap_->indexToCells(current_index, mx, my);

    int dx[4] = {-1, 1, 0, 0};
    int dy[4] = {0, 0, -1, 1};

    for (int i = 0; i < 4; i++) {
        int nx = mx + dx[i];
        int ny = my + dy[i];

        if (nx >= 0 && nx < (int)width_ && ny >= 0 && ny < (int)height_) {
            neighbors.push_back(costmap_->getIndex(nx, ny));
        }
    }
    return neighbors;
}

double SmartCenterPlanner::getHeuristic(unsigned int current_index, unsigned int goal_index)
{
    unsigned int cx, cy, gx, gy;
    costmap_->indexToCells(current_index, cx, cy);
    costmap_->indexToCells(goal_index, gx, gy);
    return std::abs(static_cast<double>(cx) - static_cast<double>(gx)) + std::abs(static_cast<double>(cy) - static_cast<double>(gy));
}

void SmartCenterPlanner::smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path)
{
    if (path.size() < 3) return;
    std::vector<geometry_msgs::msg::PoseStamped> smoothed = path;
    for (int iter = 0; iter < smooth_iterations_; iter++) {
        for (size_t i = 1; i < path.size() - 1; i++) {
            smoothed[i].pose.position.x = 0.5 * path[i].pose.position.x + 0.25 * path[i-1].pose.position.x + 0.25 * path[i+1].pose.position.x;
            smoothed[i].pose.position.y = 0.5 * path[i].pose.position.y + 0.25 * path[i-1].pose.position.y + 0.25 * path[i+1].pose.position.y;
        }
        path = smoothed;
    }
}

void SmartCenterPlanner::calculateOrientations(std::vector<geometry_msgs::msg::PoseStamped>& path, const geometry_msgs::msg::PoseStamped& goal)
{
    if (path.empty()) return;
    
    auto start_orientation = path.front().pose.orientation;
    for (size_t i = 0; i < path.size() - 1; i++) {
        path[i].pose.orientation = start_orientation;
    }
    path.back().pose.orientation = goal.pose.orientation;
}

bool SmartCenterPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) { return costmap_->worldToMap(wx, wy, mx, my); }
void SmartCenterPlanner::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) { costmap_->mapToWorld(mx, my, wx, wy); }

}  // namespace nav2_smart_center_planner_mecanum

// FIXED: Macro explicitly references the new _mecanum namespace
PLUGINLIB_EXPORT_CLASS(nav2_smart_center_planner_mecanum::SmartCenterPlanner, nav2_core::GlobalPlanner)