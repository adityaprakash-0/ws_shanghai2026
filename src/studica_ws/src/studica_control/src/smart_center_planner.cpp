// #include "studica_control/smart_center_planner.hpp"
// #include <cmath>
// #include <queue>
// #include <algorithm>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include "nav2_util/node_utils.hpp"
// #include "pluginlib/class_list_macros.hpp"

// namespace nav2_smart_center_planner
// {

// void SmartCenterPlanner::configure(
//     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//     std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
//     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
// {
//     name_ = name;
//     costmap_ros_ = costmap_ros;
//     costmap_ = costmap_ros_->getCostmap();

//     auto node = parent.lock();

//     // Load ROS 2 parameters
//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".center_weight", rclcpp::ParameterValue(2.5));
//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".smooth_iterations", rclcpp::ParameterValue(3));
//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".turn_penalty", rclcpp::ParameterValue(0.1));

//     node->get_parameter(name_ + ".center_weight", center_weight_);
//     node->get_parameter(name_ + ".smooth_iterations", smooth_iterations_);
//     node->get_parameter(name_ + ".turn_penalty", turn_penalty_);

//     RCLCPP_INFO(logger_, "Configured Smart Center Planner: weight=%.2f, smoothing passes=%d", center_weight_, smooth_iterations_);
// }

// void SmartCenterPlanner::cleanup() { RCLCPP_INFO(logger_, "Cleaning up..."); }
// void SmartCenterPlanner::activate() { RCLCPP_INFO(logger_, "Activating..."); }
// void SmartCenterPlanner::deactivate() { RCLCPP_INFO(logger_, "Deactivating..."); }

// nav_msgs::msg::Path SmartCenterPlanner::createPlan(
//     const geometry_msgs::msg::PoseStamped & start,
//     const geometry_msgs::msg::PoseStamped & goal)
// {
//     nav_msgs::msg::Path path;
//     path.header.stamp = rclcpp::Clock().now();
//     path.header.frame_id = costmap_ros_->getGlobalFrameID();

//     // 1. Update Map Info
//     width_ = costmap_->getSizeInCellsX();
//     height_ = costmap_->getSizeInCellsY();

//     unsigned int start_x, start_y, goal_x, goal_y;
//     if (!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
//         RCLCPP_WARN(logger_, "Start is out of bounds!");
//         return path;
//     }
//     if (!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
//         RCLCPP_WARN(logger_, "Goal is out of bounds!");
//         return path;
//     }

//     unsigned int start_index = costmap_->getIndex(start_x, start_y);
//     unsigned int goal_index = costmap_->getIndex(goal_x, goal_y);

//     if (start_index == goal_index) {
//         path.poses.push_back(start);
//         return path;
//     }

//     // 2. Initialize A* Data Structures
//     unsigned int grid_size = width_ * height_;
//     std::vector<float> g_score(grid_size, std::numeric_limits<float>::infinity());
//     std::vector<unsigned int> came_from(grid_size, std::numeric_limits<unsigned int>::max());
//     std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

//     g_score[start_index] = 0.0;
//     open_set.push({start_index, static_cast<float>(getHeuristic(start_index, goal_index))});

//     bool found_goal = false;

//     // 3. 8-Connected A* Search
//     while (!open_set.empty()) {
//         AStarNode current = open_set.top();
//         open_set.pop();

//         if (current.index == goal_index) {
//             found_goal = true;
//             break;
//         }

//         unsigned int cx, cy;
//         costmap_->indexToCells(current.index, cx, cy);

//         for (unsigned int neighbor_index : getNeighbors(current.index)) {
//             unsigned int nx, ny;
//             costmap_->indexToCells(neighbor_index, nx, ny);

//             unsigned char cost = costmap_->getCost(nx, ny);

//             // HARD CONSTRAINT: 254 (Lethal) and 253 (Inscribed) are physically impossible.
//             if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
//                 continue; 
//             }

//             // CALCULATE GEOMETRIC COST (1.0 for straight, 1.414 for diagonal)
//             float step_cost = (cx == nx || cy == ny) ? 1.0f : 1.4142f;

//             // THE ELASTIC SQUEEZE & VALLEY MECHANIC:
//             // Convert Nav2 cost (0 to 252) into a normalized penalty.
//             // 0 cost = 0 penalty. 252 cost = 1.0 penalty * weight.
//             float penalty = (static_cast<float>(cost) / 252.0f) * center_weight_;

//             float tentative_g = g_score[current.index] + step_cost + penalty;

//             if (tentative_g < g_score[neighbor_index]) {
//                 came_from[neighbor_index] = current.index;
//                 g_score[neighbor_index] = tentative_g;
//                 float f_score = tentative_g + getHeuristic(neighbor_index, goal_index);
//                 open_set.push({neighbor_index, f_score});
//             }
//         }
//     }

//     if (!found_goal) {
//         RCLCPP_WARN(logger_, "Failed to find a path! Ensure gap is wider than robot footprint.");
//         return path;
//     }

//     // 4. Reconstruct Path
//     std::vector<unsigned int> reversed_path;
//     unsigned int curr = goal_index;
//     while (curr != std::numeric_limits<unsigned int>::max()) {
//         reversed_path.push_back(curr);
//         curr = came_from[curr];
//     }
//     std::reverse(reversed_path.begin(), reversed_path.end());

//     for (unsigned int idx : reversed_path) {
//         unsigned int mx, my;
//         costmap_->indexToCells(idx, mx, my);
//         double wx, wy;
//         mapToWorld(mx, my, wx, wy);

//         geometry_msgs::msg::PoseStamped pose;
//         pose.header.frame_id = costmap_ros_->getGlobalFrameID();
//         pose.header.stamp = path.header.stamp;
//         pose.pose.position.x = wx;
//         pose.pose.position.y = wy;
//         pose.pose.position.z = 0.0;
//         path.poses.push_back(pose);
//     }

//     // 5. Post-Processing
//     smoothPath(path.poses);
//     calculateOrientations(path.poses, goal);

//     return path;
// }

// std::vector<unsigned int> SmartCenterPlanner::getNeighbors(unsigned int current_index)
// {
//     std::vector<unsigned int> neighbors;
//     unsigned int mx, my;
//     costmap_->indexToCells(current_index, mx, my);

//     int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
//     int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

//     for (int i = 0; i < 8; i++) {
//         int nx = mx + dx[i];
//         int ny = my + dy[i];

//         if (nx >= 0 && nx < (int)width_ && ny >= 0 && ny < (int)height_) {
//             neighbors.push_back(costmap_->getIndex(nx, ny));
//         }
//     }
//     return neighbors;
// }

// double SmartCenterPlanner::getHeuristic(unsigned int current_index, unsigned int goal_index)
// {
//     unsigned int cx, cy, gx, gy;
//     costmap_->indexToCells(current_index, cx, cy);
//     costmap_->indexToCells(goal_index, gx, gy);

//     // OCTILE HEURISTIC: Perfect for 8-connected grid.
//     double dx = std::abs(static_cast<double>(cx) - static_cast<double>(gx));
//     double dy = std::abs(static_cast<double>(cy) - static_cast<double>(gy));
    
//     return std::max(dx, dy) + 0.4142 * std::min(dx, dy);
// }

// void SmartCenterPlanner::smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path)
// {
//     if (path.size() < 3 || smooth_iterations_ <= 0) return;

//     // A fast Simple Moving Average smoother
//     // It pulls the sharp 45/90 degree corners inward, creating fluid curves.
//     std::vector<geometry_msgs::msg::PoseStamped> smoothed = path;

//     for (int iter = 0; iter < smooth_iterations_; iter++) {
//         for (size_t i = 1; i < path.size() - 1; i++) {
//             smoothed[i].pose.position.x = 0.5 * path[i].pose.position.x + 0.25 * path[i-1].pose.position.x + 0.25 * path[i+1].pose.position.x;
//             smoothed[i].pose.position.y = 0.5 * path[i].pose.position.y + 0.25 * path[i-1].pose.position.y + 0.25 * path[i+1].pose.position.y;
//         }
//         path = smoothed;
//     }
// }

// void SmartCenterPlanner::calculateOrientations(std::vector<geometry_msgs::msg::PoseStamped>& path, const geometry_msgs::msg::PoseStamped& goal)
// {
//     if (path.empty()) return;

//     for (size_t i = 0; i < path.size() - 1; i++) {
//         double dx = path[i+1].pose.position.x - path[i].pose.position.x;
//         double dy = path[i+1].pose.position.y - path[i].pose.position.y;
//         double yaw = std::atan2(dy, dx);

//         tf2::Quaternion q;
//         q.setRPY(0, 0, yaw);
//         path[i].pose.orientation = tf2::toMsg(q);
//     }
//     // The final pose takes the requested goal orientation
//     path.back().pose.orientation = goal.pose.orientation;
// }

// bool SmartCenterPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
// {
//     return costmap_->worldToMap(wx, wy, mx, my);
// }

// void SmartCenterPlanner::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy)
// {
//     costmap_->mapToWorld(mx, my, wx, wy);
// }

// }  // namespace nav2_smart_center_planner

// // Export as a Nav2 Plugin
// PLUGINLIB_EXPORT_CLASS(nav2_smart_center_planner::SmartCenterPlanner, nav2_core::GlobalPlanner)

//updae correct

// #include "studica_control/smart_center_planner.hpp"
// #include <cmath>
// #include <queue>
// #include <algorithm>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include "nav2_util/node_utils.hpp"
// #include "pluginlib/class_list_macros.hpp"

// namespace nav2_smart_center_planner
// {

// void SmartCenterPlanner::configure(
//     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//     std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
//     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
// {
//     name_ = name;
//     costmap_ros_ = costmap_ros;
//     costmap_ = costmap_ros_->getCostmap();

//     auto node = parent.lock();

//     // Load ROS 2 parameters
//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".center_weight", rclcpp::ParameterValue(2.5));
//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".smooth_iterations", rclcpp::ParameterValue(3));
//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".turn_penalty", rclcpp::ParameterValue(0.1));

//     node->get_parameter(name_ + ".center_weight", center_weight_);
//     node->get_parameter(name_ + ".smooth_iterations", smooth_iterations_);
//     node->get_parameter(name_ + ".turn_penalty", turn_penalty_);

//     RCLCPP_INFO(logger_, "Configured Smart Center Planner: weight=%.2f, smoothing passes=%d", center_weight_, smooth_iterations_);
// }

// void SmartCenterPlanner::cleanup() { RCLCPP_INFO(logger_, "Cleaning up..."); }
// void SmartCenterPlanner::activate() { RCLCPP_INFO(logger_, "Activating..."); }
// void SmartCenterPlanner::deactivate() { RCLCPP_INFO(logger_, "Deactivating..."); }

// nav_msgs::msg::Path SmartCenterPlanner::createPlan(
//     const geometry_msgs::msg::PoseStamped & start,
//     const geometry_msgs::msg::PoseStamped & goal)
// {
//     nav_msgs::msg::Path path;
//     path.header.stamp = rclcpp::Clock().now();
//     path.header.frame_id = costmap_ros_->getGlobalFrameID();

//     // 1. Update Map Info
//     width_ = costmap_->getSizeInCellsX();
//     height_ = costmap_->getSizeInCellsY();

//     unsigned int start_x, start_y, goal_x, goal_y;
//     if (!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
//         RCLCPP_WARN(logger_, "Start is out of bounds!");
//         return path;
//     }
//     if (!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
//         RCLCPP_WARN(logger_, "Goal is out of bounds!");
//         return path;
//     }

//     unsigned int start_index = costmap_->getIndex(start_x, start_y);
//     unsigned int goal_index = costmap_->getIndex(goal_x, goal_y);

//     // ==============================================================
//     // FIX 1: Provide a valid path even if in the same cell
//     // ==============================================================
//     if (start_index == goal_index) {
//         path.poses.push_back(start);
        
//         geometry_msgs::msg::PoseStamped precise_goal = goal;
//         precise_goal.header.stamp = path.header.stamp;
//         precise_goal.header.frame_id = costmap_ros_->getGlobalFrameID();
//         precise_goal.pose.position.z = 0.0;
//         path.poses.push_back(precise_goal);
        
//         return path;
//     }

//     // 2. Initialize A* Data Structures
//     unsigned int grid_size = width_ * height_;
//     std::vector<float> g_score(grid_size, std::numeric_limits<float>::infinity());
//     std::vector<unsigned int> came_from(grid_size, std::numeric_limits<unsigned int>::max());
//     std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

//     g_score[start_index] = 0.0;
//     open_set.push({start_index, static_cast<float>(getHeuristic(start_index, goal_index))});

//     bool found_goal = false;

//     // 3. 8-Connected A* Search
//     while (!open_set.empty()) {
//         AStarNode current = open_set.top();
//         open_set.pop();

//         if (current.index == goal_index) {
//             found_goal = true;
//             break;
//         }

//         unsigned int cx, cy;
//         costmap_->indexToCells(current.index, cx, cy);

//         for (unsigned int neighbor_index : getNeighbors(current.index)) {
//             unsigned int nx, ny;
//             costmap_->indexToCells(neighbor_index, nx, ny);

//             unsigned char cost = costmap_->getCost(nx, ny);

//             if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
//                 continue; 
//             }

//             float step_cost = (cx == nx || cy == ny) ? 1.0f : 1.4142f;
//             float penalty = (static_cast<float>(cost) / 252.0f) * center_weight_;
//             float tentative_g = g_score[current.index] + step_cost + penalty;

//             if (tentative_g < g_score[neighbor_index]) {
//                 came_from[neighbor_index] = current.index;
//                 g_score[neighbor_index] = tentative_g;
//                 float f_score = tentative_g + getHeuristic(neighbor_index, goal_index);
//                 open_set.push({neighbor_index, f_score});
//             }
//         }
//     }

//     if (!found_goal) {
//         RCLCPP_WARN(logger_, "Failed to find a path! Ensure gap is wider than robot footprint.");
//         return path;
//     }

//     // 4. Reconstruct Path
//     std::vector<unsigned int> reversed_path;
//     unsigned int curr = goal_index;
//     while (curr != std::numeric_limits<unsigned int>::max()) {
//         reversed_path.push_back(curr);
//         curr = came_from[curr];
//     }
//     std::reverse(reversed_path.begin(), reversed_path.end());

//     for (unsigned int idx : reversed_path) {
//         unsigned int mx, my;
//         costmap_->indexToCells(idx, mx, my);
//         double wx, wy;
//         mapToWorld(mx, my, wx, wy);

//         geometry_msgs::msg::PoseStamped pose;
//         pose.header.frame_id = costmap_ros_->getGlobalFrameID();
//         pose.header.stamp = path.header.stamp;
//         pose.pose.position.x = wx;
//         pose.pose.position.y = wy;
//         pose.pose.position.z = 0.0;
//         path.poses.push_back(pose);
//     }

//     // ==============================================================
//     // FIX 2: Snap the first and last points to the exact coordinates
//     // ==============================================================
//     if (!path.poses.empty()) {
//         path.poses.front().pose.position.x = start.pose.position.x;
//         path.poses.front().pose.position.y = start.pose.position.y;
        
//         path.poses.back().pose.position.x = goal.pose.position.x;
//         path.poses.back().pose.position.y = goal.pose.position.y;
//     }

//     // 5. Post-Processing
//     smoothPath(path.poses);
//     calculateOrientations(path.poses, goal);

//     return path;
// }

// std::vector<unsigned int> SmartCenterPlanner::getNeighbors(unsigned int current_index)
// {
//     std::vector<unsigned int> neighbors;
//     unsigned int mx, my;
//     costmap_->indexToCells(current_index, mx, my);

//     int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
//     int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

//     for (int i = 0; i < 8; i++) {
//         int nx = mx + dx[i];
//         int ny = my + dy[i];

//         if (nx >= 0 && nx < (int)width_ && ny >= 0 && ny < (int)height_) {
//             neighbors.push_back(costmap_->getIndex(nx, ny));
//         }
//     }
//     return neighbors;
// }

// double SmartCenterPlanner::getHeuristic(unsigned int current_index, unsigned int goal_index)
// {
//     unsigned int cx, cy, gx, gy;
//     costmap_->indexToCells(current_index, cx, cy);
//     costmap_->indexToCells(goal_index, gx, gy);

//     double dx = std::abs(static_cast<double>(cx) - static_cast<double>(gx));
//     double dy = std::abs(static_cast<double>(cy) - static_cast<double>(gy));
    
//     return std::max(dx, dy) + 0.4142 * std::min(dx, dy);
// }

// void SmartCenterPlanner::smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path)
// {
//     if (path.size() < 3 || smooth_iterations_ <= 0) return;

//     std::vector<geometry_msgs::msg::PoseStamped> smoothed = path;

//     for (int iter = 0; iter < smooth_iterations_; iter++) {
//         for (size_t i = 1; i < path.size() - 1; i++) {
//             smoothed[i].pose.position.x = 0.5 * path[i].pose.position.x + 0.25 * path[i-1].pose.position.x + 0.25 * path[i+1].pose.position.x;
//             smoothed[i].pose.position.y = 0.5 * path[i].pose.position.y + 0.25 * path[i-1].pose.position.y + 0.25 * path[i+1].pose.position.y;
//         }
//         path = smoothed;
//     }
// }

// void SmartCenterPlanner::calculateOrientations(std::vector<geometry_msgs::msg::PoseStamped>& path, const geometry_msgs::msg::PoseStamped& goal)
// {
//     if (path.empty()) return;

//     for (size_t i = 0; i < path.size() - 1; i++) {
//         double dx = path[i+1].pose.position.x - path[i].pose.position.x;
//         double dy = path[i+1].pose.position.y - path[i].pose.position.y;
//         double yaw = std::atan2(dy, dx);

//         tf2::Quaternion q;
//         q.setRPY(0, 0, yaw);
//         path[i].pose.orientation = tf2::toMsg(q);
//     }
//     path.back().pose.orientation = goal.pose.orientation;
// }

// bool SmartCenterPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
// {
//     return costmap_->worldToMap(wx, wy, mx, my);
// }

// void SmartCenterPlanner::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy)
// {
//     costmap_->mapToWorld(mx, my, wx, wy);
// }

// }  // namespace nav2_smart_center_planner

// PLUGINLIB_EXPORT_CLASS(nav2_smart_center_planner::SmartCenterPlanner, nav2_core::GlobalPlanner)



//updated with allowing path to 253 and all movements

// #include "studica_control/smart_center_planner.hpp"
// #include <cmath>
// #include <queue>
// #include <algorithm>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include "nav2_util/node_utils.hpp"
// #include "pluginlib/class_list_macros.hpp"

// namespace nav2_smart_center_planner
// {

// void SmartCenterPlanner::configure(
//     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//     std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
//     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
// {
//     name_ = name;
//     costmap_ros_ = costmap_ros;
//     costmap_ = costmap_ros_->getCostmap();

//     auto node = parent.lock();

//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".center_weight", rclcpp::ParameterValue(2.5));
//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".smooth_iterations", rclcpp::ParameterValue(3));
//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".turn_penalty", rclcpp::ParameterValue(0.1));

//     node->get_parameter(name_ + ".center_weight", center_weight_);
//     node->get_parameter(name_ + ".smooth_iterations", smooth_iterations_);
//     node->get_parameter(name_ + ".turn_penalty", turn_penalty_);

//     RCLCPP_INFO(logger_, "Configured Smart Center Planner: weight=%.2f, smoothing passes=%d", center_weight_, smooth_iterations_);
// }

// void SmartCenterPlanner::cleanup() { RCLCPP_INFO(logger_, "Cleaning up..."); }
// void SmartCenterPlanner::activate() { RCLCPP_INFO(logger_, "Activating..."); }
// void SmartCenterPlanner::deactivate() { RCLCPP_INFO(logger_, "Deactivating..."); }

// nav_msgs::msg::Path SmartCenterPlanner::createPlan(
//     const geometry_msgs::msg::PoseStamped & start,
//     const geometry_msgs::msg::PoseStamped & goal)
// {
//     nav_msgs::msg::Path path;
//     path.header.stamp = rclcpp::Clock().now();
//     path.header.frame_id = costmap_ros_->getGlobalFrameID();

//     width_ = costmap_->getSizeInCellsX();
//     height_ = costmap_->getSizeInCellsY();

//     unsigned int start_x, start_y, goal_x, goal_y;
//     if (!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
//         !worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
//         RCLCPP_WARN(logger_, "Start or Goal is out of bounds!");
//         return path;
//     }

//     unsigned int start_index = costmap_->getIndex(start_x, start_y);
//     unsigned int goal_index = costmap_->getIndex(goal_x, goal_y);

//     if (start_index == goal_index) {
//         path.poses.push_back(start);
//         geometry_msgs::msg::PoseStamped precise_goal = goal;
//         precise_goal.header.stamp = path.header.stamp;
//         precise_goal.header.frame_id = costmap_ros_->getGlobalFrameID();
//         precise_goal.pose.position.z = 0.0;
//         path.poses.push_back(precise_goal);
//         return path;
//     }

//     unsigned int grid_size = width_ * height_;
//     std::vector<float> g_score(grid_size, std::numeric_limits<float>::infinity());
//     std::vector<unsigned int> came_from(grid_size, std::numeric_limits<unsigned int>::max());
//     std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

//     g_score[start_index] = 0.0;
//     open_set.push({start_index, static_cast<float>(getHeuristic(start_index, goal_index))});

//     bool found_goal = false;

//     while (!open_set.empty()) {
//         AStarNode current = open_set.top();
//         open_set.pop();

//         if (current.index == goal_index) {
//             found_goal = true;
//             break;
//         }

//         unsigned int cx, cy;
//         costmap_->indexToCells(current.index, cx, cy);

//         for (unsigned int neighbor_index : getNeighbors(current.index)) {
//             unsigned int nx, ny;
//             costmap_->indexToCells(neighbor_index, nx, ny);

//             unsigned char cost = costmap_->getCost(nx, ny);

//             // =====================================================================
//             // THE FIX: ONLY block 254 (Physical Walls) and 255 (Unknown Space).
//             // We now ALLOW 253 (Inscribed), meaning the robot can squeeze if it must!
//             // =====================================================================
//             if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
//                 continue; 
//             }

//             float step_cost = (cx == nx || cy == ny) ? 1.0f : 1.4142f;
//             float penalty = 0.0f;

//             if (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
//                 // DESPERATION MODE: The gap is tight. We apply a massive penalty 
//                 // so the robot avoids 253 unless there is absolutely no other choice.
//                 penalty = 25.0f * center_weight_; 
//             } else {
//                 // NORMAL MODE: Elastic squeeze towards the center
//                 penalty = (static_cast<float>(cost) / 252.0f) * center_weight_;
//             }

//             float tentative_g = g_score[current.index] + step_cost + penalty;

//             if (tentative_g < g_score[neighbor_index]) {
//                 came_from[neighbor_index] = current.index;
//                 g_score[neighbor_index] = tentative_g;
//                 float f_score = tentative_g + getHeuristic(neighbor_index, goal_index);
//                 open_set.push({neighbor_index, f_score});
//             }
//         }
//     }

//     if (!found_goal) {
//         RCLCPP_WARN(logger_, "Failed to find a path! Ensure the tip of your red goal arrow is NOT inside a wall.");
//         return path;
//     }

//     std::vector<unsigned int> reversed_path;
//     unsigned int curr = goal_index;
//     while (curr != std::numeric_limits<unsigned int>::max()) {
//         reversed_path.push_back(curr);
//         curr = came_from[curr];
//     }
//     std::reverse(reversed_path.begin(), reversed_path.end());

//     for (unsigned int idx : reversed_path) {
//         unsigned int mx, my;
//         costmap_->indexToCells(idx, mx, my);
//         double wx, wy;
//         mapToWorld(mx, my, wx, wy);

//         geometry_msgs::msg::PoseStamped pose;
//         pose.header.frame_id = costmap_ros_->getGlobalFrameID();
//         pose.header.stamp = path.header.stamp;
//         pose.pose.position.x = wx;
//         pose.pose.position.y = wy;
//         pose.pose.position.z = 0.0;
//         path.poses.push_back(pose);
//     }

//     if (!path.poses.empty()) {
//         path.poses.front().pose.position.x = start.pose.position.x;
//         path.poses.front().pose.position.y = start.pose.position.y;
//         path.poses.back().pose.position.x = goal.pose.position.x;
//         path.poses.back().pose.position.y = goal.pose.position.y;
//     }

//     smoothPath(path.poses);
//     calculateOrientations(path.poses, goal);

//     return path;
// }

// std::vector<unsigned int> SmartCenterPlanner::getNeighbors(unsigned int current_index)
// {
//     std::vector<unsigned int> neighbors;
//     unsigned int mx, my;
//     costmap_->indexToCells(current_index, mx, my);

//     int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
//     int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

//     for (int i = 0; i < 8; i++) {
//         int nx = mx + dx[i];
//         int ny = my + dy[i];

//         if (nx >= 0 && nx < (int)width_ && ny >= 0 && ny < (int)height_) {
//             neighbors.push_back(costmap_->getIndex(nx, ny));
//         }
//     }
//     return neighbors;
// }

// double SmartCenterPlanner::getHeuristic(unsigned int current_index, unsigned int goal_index)
// {
//     unsigned int cx, cy, gx, gy;
//     costmap_->indexToCells(current_index, cx, cy);
//     costmap_->indexToCells(goal_index, gx, gy);

//     double dx = std::abs(static_cast<double>(cx) - static_cast<double>(gx));
//     double dy = std::abs(static_cast<double>(cy) - static_cast<double>(gy));
    
//     return std::max(dx, dy) + 0.4142 * std::min(dx, dy);
// }

// void SmartCenterPlanner::smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path)
// {
//     if (path.size() < 3 || smooth_iterations_ <= 0) return;
//     std::vector<geometry_msgs::msg::PoseStamped> smoothed = path;
//     for (int iter = 0; iter < smooth_iterations_; iter++) {
//         for (size_t i = 1; i < path.size() - 1; i++) {
//             smoothed[i].pose.position.x = 0.5 * path[i].pose.position.x + 0.25 * path[i-1].pose.position.x + 0.25 * path[i+1].pose.position.x;
//             smoothed[i].pose.position.y = 0.5 * path[i].pose.position.y + 0.25 * path[i-1].pose.position.y + 0.25 * path[i+1].pose.position.y;
//         }
//         path = smoothed;
//     }
// }

// void SmartCenterPlanner::calculateOrientations(std::vector<geometry_msgs::msg::PoseStamped>& path, const geometry_msgs::msg::PoseStamped& goal)
// {
//     if (path.empty()) return;
//     for (size_t i = 0; i < path.size() - 1; i++) {
//         double dx = path[i+1].pose.position.x - path[i].pose.position.x;
//         double dy = path[i+1].pose.position.y - path[i].pose.position.y;
//         double yaw = std::atan2(dy, dx);
//         tf2::Quaternion q;
//         q.setRPY(0, 0, yaw);
//         path[i].pose.orientation = tf2::toMsg(q);
//     }
//     path.back().pose.orientation = goal.pose.orientation;
// }

// bool SmartCenterPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
// {
//     return costmap_->worldToMap(wx, wy, mx, my);
// }

// void SmartCenterPlanner::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy)
// {
//     costmap_->mapToWorld(mx, my, wx, wy);
// }

// }  // namespace nav2_smart_center_planner

// PLUGINLIB_EXPORT_CLASS(nav2_smart_center_planner::SmartCenterPlanner, nav2_core::GlobalPlanner)

//code with working all directions added turn penalty
#include "studica_control/smart_center_planner.hpp"
#include <cmath>
#include <queue>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_smart_center_planner
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

    nav2_util::declare_parameter_if_not_declared(node, name_ + ".center_weight", rclcpp::ParameterValue(2.5));
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".smooth_iterations", rclcpp::ParameterValue(3));
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".turn_penalty", rclcpp::ParameterValue(0.1));

    node->get_parameter(name_ + ".center_weight", center_weight_);
    node->get_parameter(name_ + ".smooth_iterations", smooth_iterations_);
    node->get_parameter(name_ + ".turn_penalty", turn_penalty_);

    RCLCPP_INFO(logger_, "Configured Smart Center Planner: weight=%.2f, smoothing passes=%d, turn_penalty=%.2f", 
                center_weight_, smooth_iterations_, turn_penalty_);
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
        geometry_msgs::msg::PoseStamped precise_goal = goal;
        precise_goal.header.stamp = path.header.stamp;
        precise_goal.header.frame_id = costmap_ros_->getGlobalFrameID();
        precise_goal.pose.position.z = 0.0;
        path.poses.push_back(precise_goal);
        return path;
    }

    unsigned int grid_size = width_ * height_;
    std::vector<float> g_score(grid_size, std::numeric_limits<float>::infinity());
    std::vector<unsigned int> came_from(grid_size, std::numeric_limits<unsigned int>::max());
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

            // =====================================================================
            // ONLY block 254 (Physical Walls) and 255 (Unknown Space).
            // We now ALLOW 253 (Inscribed), meaning the robot can squeeze if it must!
            // =====================================================================
            if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
                continue; 
            }

            float step_cost = (cx == nx || cy == ny) ? 1.0f : 1.4142f;
            float penalty = 0.0f;

            if (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                // DESPERATION MODE: The gap is tight. We apply a massive penalty 
                // so the robot avoids 253 unless there is absolutely no other choice.
                penalty = 25.0f * center_weight_; 
            } else {
                // NORMAL MODE: Elastic squeeze towards the center
                penalty = (static_cast<float>(cost) / 252.0f) * center_weight_;
            }

            // =====================================================================
            // TURN PENALTY LOGIC
            // Penalize paths that change direction to favor straight lines
            // =====================================================================
            float turn_cost = 0.0f;
            if (came_from[current.index] != std::numeric_limits<unsigned int>::max()) {
                unsigned int px, py;
                costmap_->indexToCells(came_from[current.index], px, py);

                int dx1 = static_cast<int>(cx) - static_cast<int>(px);
                int dy1 = static_cast<int>(cy) - static_cast<int>(py);
                int dx2 = static_cast<int>(nx) - static_cast<int>(cx);
                int dy2 = static_cast<int>(ny) - static_cast<int>(cy);

                if (dx1 != dx2 || dy1 != dy2) {
                    turn_cost = turn_penalty_;
                }
            }

            float tentative_g = g_score[current.index] + step_cost + penalty + turn_cost;

            if (tentative_g < g_score[neighbor_index]) {
                came_from[neighbor_index] = current.index;
                g_score[neighbor_index] = tentative_g;
                float f_score = tentative_g + getHeuristic(neighbor_index, goal_index);
                open_set.push({neighbor_index, f_score});
            }
        }
    }

    if (!found_goal) {
        RCLCPP_WARN(logger_, "Failed to find a path! Ensure the tip of your red goal arrow is NOT inside a wall.");
        return path;
    }

    std::vector<unsigned int> reversed_path;
    unsigned int curr = goal_index;
    while (curr != std::numeric_limits<unsigned int>::max()) {
        reversed_path.push_back(curr);
        curr = came_from[curr];
    }
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

    if (!path.poses.empty()) {
        path.poses.front().pose.position.x = start.pose.position.x;
        path.poses.front().pose.position.y = start.pose.position.y;
        path.poses.back().pose.position.x = goal.pose.position.x;
        path.poses.back().pose.position.y = goal.pose.position.y;
    }

    smoothPath(path.poses);
    calculateOrientations(path.poses, goal);

    return path;
}

std::vector<unsigned int> SmartCenterPlanner::getNeighbors(unsigned int current_index)
{
    std::vector<unsigned int> neighbors;
    unsigned int mx, my;
    costmap_->indexToCells(current_index, mx, my);

    int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (int i = 0; i < 8; i++) {
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

    double dx = std::abs(static_cast<double>(cx) - static_cast<double>(gx));
    double dy = std::abs(static_cast<double>(cy) - static_cast<double>(gy));
    
    return std::max(dx, dy) + 0.4142 * std::min(dx, dy);
}

void SmartCenterPlanner::smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path)
{
    if (path.size() < 3 || smooth_iterations_ <= 0) return;
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
    for (size_t i = 0; i < path.size() - 1; i++) {
        double dx = path[i+1].pose.position.x - path[i].pose.position.x;
        double dy = path[i+1].pose.position.y - path[i].pose.position.y;
        double yaw = std::atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        path[i].pose.orientation = tf2::toMsg(q);
    }
    path.back().pose.orientation = goal.pose.orientation;
}

bool SmartCenterPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
    return costmap_->worldToMap(wx, wy, mx, my);
}

void SmartCenterPlanner::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy)
{
    costmap_->mapToWorld(mx, my, wx, wy);
}

}  // namespace nav2_smart_center_planner

PLUGINLIB_EXPORT_CLASS(nav2_smart_center_planner::SmartCenterPlanner, nav2_core::GlobalPlanner)


















//code with 4 manhattan type movement and no diagonal 
// #include "studica_control/smart_center_planner.hpp"
// #include <cmath>
// #include <queue>
// #include <algorithm>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include "nav2_util/node_utils.hpp"
// #include "pluginlib/class_list_macros.hpp"

// namespace nav2_smart_center_planner
// {

// void SmartCenterPlanner::configure(
//     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//     std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
//     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
// {
//     name_ = name;
//     costmap_ros_ = costmap_ros;
//     costmap_ = costmap_ros_->getCostmap();

//     auto node = parent.lock();

//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".center_weight", rclcpp::ParameterValue(2.5));
//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".smooth_iterations", rclcpp::ParameterValue(3));
//     nav2_util::declare_parameter_if_not_declared(node, name_ + ".turn_penalty", rclcpp::ParameterValue(0.1));

//     node->get_parameter(name_ + ".center_weight", center_weight_);
//     node->get_parameter(name_ + ".smooth_iterations", smooth_iterations_);
//     node->get_parameter(name_ + ".turn_penalty", turn_penalty_);

//     RCLCPP_INFO(logger_, "Configured Smart Center Planner: weight=%.2f, turn_penalty=%.2f", center_weight_, turn_penalty_);
// }

// void SmartCenterPlanner::cleanup() { RCLCPP_INFO(logger_, "Cleaning up..."); }
// void SmartCenterPlanner::activate() { RCLCPP_INFO(logger_, "Activating..."); }
// void SmartCenterPlanner::deactivate() { RCLCPP_INFO(logger_, "Deactivating..."); }

// nav_msgs::msg::Path SmartCenterPlanner::createPlan(
//     const geometry_msgs::msg::PoseStamped & start,
//     const geometry_msgs::msg::PoseStamped & goal)
// {
//     nav_msgs::msg::Path path;
//     path.header.stamp = rclcpp::Clock().now();
//     path.header.frame_id = costmap_ros_->getGlobalFrameID();

//     width_ = costmap_->getSizeInCellsX();
//     height_ = costmap_->getSizeInCellsY();

//     unsigned int start_x, start_y, goal_x, goal_y;
//     if (!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
//         !worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
//         RCLCPP_WARN(logger_, "Start or Goal is out of bounds!");
//         return path;
//     }

//     unsigned int start_index = costmap_->getIndex(start_x, start_y);
//     unsigned int goal_index = costmap_->getIndex(goal_x, goal_y);

//     if (start_index == goal_index) {
//         path.poses.push_back(start);
//         geometry_msgs::msg::PoseStamped precise_goal = goal;
//         precise_goal.header.stamp = path.header.stamp;
//         precise_goal.header.frame_id = costmap_ros_->getGlobalFrameID();
//         precise_goal.pose.position.z = 0.0;
//         path.poses.push_back(precise_goal);
//         return path;
//     }

//     unsigned int grid_size = width_ * height_;
//     std::vector<float> g_score(grid_size, std::numeric_limits<float>::infinity());
//     std::vector<unsigned int> came_from(grid_size, std::numeric_limits<unsigned int>::max());
//     std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

//     g_score[start_index] = 0.0;
//     open_set.push({start_index, static_cast<float>(getHeuristic(start_index, goal_index))});

//     bool found_goal = false;

//     while (!open_set.empty()) {
//         AStarNode current = open_set.top();
//         open_set.pop();

//         if (current.index == goal_index) {
//             found_goal = true;
//             break;
//         }

//         unsigned int cx, cy;
//         costmap_->indexToCells(current.index, cx, cy);

//         for (unsigned int neighbor_index : getNeighbors(current.index)) {
//             unsigned int nx, ny;
//             costmap_->indexToCells(neighbor_index, nx, ny);

//             unsigned char cost = costmap_->getCost(nx, ny);

//             if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
//                 continue; 
//             }

//             float step_cost = 1.0f;
//             float penalty = 0.0f;

//             if (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
//                 penalty = 25.0f * center_weight_; 
//             } else {
//                 penalty = (static_cast<float>(cost) / 252.0f) * center_weight_;
//             }

//             // ==========================================================
//             // TURN PENALTY IMPLEMENTATION (Anti-Staircase Logic)
//             // ==========================================================
//             float turn_cost = 0.0f;
            
//             // Ensure this isn't the very first step from the start node
//             if (came_from[current.index] != std::numeric_limits<unsigned int>::max()) {
//                 unsigned int px, py;
//                 costmap_->indexToCells(came_from[current.index], px, py);
                
//                 // Direction we were traveling to get to the 'current' cell
//                 int dx_current = (int)cx - (int)px;
//                 int dy_current = (int)cy - (int)py;
                
//                 // Direction we want to travel to get to the 'neighbor' cell
//                 int dx_next = (int)nx - (int)cx;
//                 int dy_next = (int)ny - (int)cy;

//                 // If the directions don't match, we are making a 90-degree turn
//                 if (dx_current != dx_next || dy_current != dy_next) {
//                     turn_cost = turn_penalty_;
//                 }
//             }

//             // Add the turn cost to the G score calculation
//             float tentative_g = g_score[current.index] + step_cost + penalty + turn_cost;

//             if (tentative_g < g_score[neighbor_index]) {
//                 came_from[neighbor_index] = current.index;
//                 g_score[neighbor_index] = tentative_g;
//                 float f_score = tentative_g + getHeuristic(neighbor_index, goal_index);
//                 open_set.push({neighbor_index, f_score});
//             }
//         }
//     }

//     if (!found_goal) {
//         RCLCPP_WARN(logger_, "Failed to find a path! Ensure the tip of your red goal arrow is NOT inside a wall.");
//         return path;
//     }

//     std::vector<unsigned int> reversed_path;
//     unsigned int curr = goal_index;
//     while (curr != std::numeric_limits<unsigned int>::max()) {
//         reversed_path.push_back(curr);
//         curr = came_from[curr];
//     }
//     std::reverse(reversed_path.begin(), reversed_path.end());

//     for (unsigned int idx : reversed_path) {
//         unsigned int mx, my;
//         costmap_->indexToCells(idx, mx, my);
//         double wx, wy;
//         mapToWorld(mx, my, wx, wy);

//         geometry_msgs::msg::PoseStamped pose;
//         pose.header.frame_id = costmap_ros_->getGlobalFrameID();
//         pose.header.stamp = path.header.stamp;
//         pose.pose.position.x = wx;
//         pose.pose.position.y = wy;
//         pose.pose.position.z = 0.0;
//         path.poses.push_back(pose);
//     }

//     if (!path.poses.empty()) {
//         path.poses.front().pose.position.x = start.pose.position.x;
//         path.poses.front().pose.position.y = start.pose.position.y;
//         path.poses.back().pose.position.x = goal.pose.position.x;
//         path.poses.back().pose.position.y = goal.pose.position.y;
//     }

//     smoothPath(path.poses);
//     calculateOrientations(path.poses, goal);

//     return path;
// }

// std::vector<unsigned int> SmartCenterPlanner::getNeighbors(unsigned int current_index)
// {
//     std::vector<unsigned int> neighbors;
//     unsigned int mx, my;
//     costmap_->indexToCells(current_index, mx, my);

//     // STRICT 4-WAY MOVEMENT ONLY (North, South, East, West)
//     int dx[4] = {-1, 1, 0, 0};
//     int dy[4] = {0, 0, -1, 1};

//     for (int i = 0; i < 4; i++) {
//         int nx = mx + dx[i];
//         int ny = my + dy[i];

//         if (nx >= 0 && nx < (int)width_ && ny >= 0 && ny < (int)height_) {
//             neighbors.push_back(costmap_->getIndex(nx, ny));
//         }
//     }
//     return neighbors;
// }

// double SmartCenterPlanner::getHeuristic(unsigned int current_index, unsigned int goal_index)
// {
//     unsigned int cx, cy, gx, gy;
//     costmap_->indexToCells(current_index, cx, cy);
//     costmap_->indexToCells(goal_index, gx, gy);

//     double dx = std::abs(static_cast<double>(cx) - static_cast<double>(gx));
//     double dy = std::abs(static_cast<double>(cy) - static_cast<double>(gy));
    
//     // MANHATTAN DISTANCE HEURISTIC
//     return dx + dy;
// }

// void SmartCenterPlanner::smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path)
// {
//     if (path.size() < 3 || smooth_iterations_ <= 0) return;
//     std::vector<geometry_msgs::msg::PoseStamped> smoothed = path;
//     for (int iter = 0; iter < smooth_iterations_; iter++) {
//         for (size_t i = 1; i < path.size() - 1; i++) {
//             smoothed[i].pose.position.x = 0.5 * path[i].pose.position.x + 0.25 * path[i-1].pose.position.x + 0.25 * path[i+1].pose.position.x;
//             smoothed[i].pose.position.y = 0.5 * path[i].pose.position.y + 0.25 * path[i-1].pose.position.y + 0.25 * path[i+1].pose.position.y;
//         }
//         path = smoothed;
//     }
// }

// void SmartCenterPlanner::calculateOrientations(std::vector<geometry_msgs::msg::PoseStamped>& path, const geometry_msgs::msg::PoseStamped& goal)
// {
//     if (path.empty()) return;
//     for (size_t i = 0; i < path.size() - 1; i++) {
//         double dx = path[i+1].pose.position.x - path[i].pose.position.x;
//         double dy = path[i+1].pose.position.y - path[i].pose.position.y;
//         double yaw = std::atan2(dy, dx);
//         tf2::Quaternion q;
//         q.setRPY(0, 0, yaw);
//         path[i].pose.orientation = tf2::toMsg(q);
//     }
//     path.back().pose.orientation = goal.pose.orientation;
// }

// bool SmartCenterPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
// {
//     return costmap_->worldToMap(wx, wy, mx, my);
// }

// void SmartCenterPlanner::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy)
// {
//     costmap_->mapToWorld(mx, my, wx, wy);
// }

// }  // namespace nav2_smart_center_planner

// PLUGINLIB_EXPORT_CLASS(nav2_smart_center_planner::SmartCenterPlanner, nav2_core::GlobalPlanner)