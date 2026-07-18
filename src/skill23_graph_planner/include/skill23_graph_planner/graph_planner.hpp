#ifndef SKILL23_GRAPH_PLANNER__GRAPH_PLANNER_HPP_
#define SKILL23_GRAPH_PLANNER__GRAPH_PLANNER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace skill23_graph_planner
{

struct GraphNode
{
  int id{0};
  double x{0.0};
  double y{0.0};
};

struct GraphEdge
{
  int to{0};
  double cost{0.0};
};

/**
 * @brief Nav2 GlobalPlanner: snap poses to a GeoJSON corridor graph and
 *        Dijkstra-search a densified path for FollowPath / RPP.
 *
 * Graph format (laptop/route_dashboard export):
 *   Point features: id, frame=map, coordinates [x,y]
 *   MultiLineString edges: startid, endid (directed; usually both ways)
 */
class GraphPlanner : public nav2_core::GlobalPlanner
{
public:
  GraphPlanner() = default;
  ~GraphPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  bool loadGraph(const std::string & filepath);
  int nearestNode(double x, double y, double * dist_out) const;
  std::vector<int> dijkstra(int start_id, int goal_id) const;
  nav_msgs::msg::Path densify(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<int> & node_ids) const;
  static void setYaw(
    geometry_msgs::msg::PoseStamped & pose, double yaw);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("GraphPlanner")};
  std::string name_;
  std::string global_frame_;

  std::string graph_filepath_;
  double path_density_{0.05};
  double max_snap_distance_{1.5};
  bool allow_same_node_{true};

  std::unordered_map<int, GraphNode> nodes_;
  std::unordered_map<int, std::vector<GraphEdge>> adj_;
  bool graph_loaded_{false};
};

}  // namespace skill23_graph_planner

#endif  // SKILL23_GRAPH_PLANNER__GRAPH_PLANNER_HPP_
