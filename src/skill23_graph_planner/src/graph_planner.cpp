#include "skill23_graph_planner/graph_planner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "nlohmann/json.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace skill23_graph_planner
{
namespace
{

double hypot2(double dx, double dy)
{
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

void GraphPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = rclcpp::get_logger(name_);

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("GraphPlanner: failed to lock lifecycle node");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".graph_filepath", rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_density", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_snap_distance", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_same_node", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".graph_filepath", graph_filepath_);
  node->get_parameter(name_ + ".path_density", path_density_);
  node->get_parameter(name_ + ".max_snap_distance", max_snap_distance_);
  node->get_parameter(name_ + ".allow_same_node", allow_same_node_);

  if (path_density_ < 0.01) {
    path_density_ = 0.01;
  }

  if (graph_filepath_.empty()) {
    RCLCPP_ERROR(
      logger_,
      "GraphPlanner: graph_filepath is empty — set planner_server.GridBased.graph_filepath");
    graph_loaded_ = false;
    return;
  }

  graph_loaded_ = loadGraph(graph_filepath_);
  if (graph_loaded_) {
    RCLCPP_INFO(
      logger_,
      "GraphPlanner ready: %zu nodes, graph=%s density=%.3f max_snap=%.2f",
      nodes_.size(), graph_filepath_.c_str(), path_density_, max_snap_distance_);
  }
}

void GraphPlanner::cleanup()
{
  nodes_.clear();
  adj_.clear();
  graph_loaded_ = false;
}

void GraphPlanner::activate() {}

void GraphPlanner::deactivate() {}

bool GraphPlanner::loadGraph(const std::string & filepath)
{
  nodes_.clear();
  adj_.clear();

  std::ifstream in(filepath);
  if (!in) {
    RCLCPP_ERROR(logger_, "GraphPlanner: cannot open graph file: %s", filepath.c_str());
    return false;
  }

  nlohmann::json root;
  try {
    in >> root;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "GraphPlanner: JSON parse failed: %s", ex.what());
    return false;
  }

  if (!root.contains("features") || !root["features"].is_array()) {
    RCLCPP_ERROR(logger_, "GraphPlanner: missing FeatureCollection.features");
    return false;
  }

  for (const auto & feat : root["features"]) {
    if (!feat.contains("geometry") || !feat.contains("properties")) {
      continue;
    }
    const auto & geom = feat["geometry"];
    const auto & props = feat["properties"];
    const std::string type = geom.value("type", "");

    if (type == "Point") {
      if (!props.contains("id") || !geom.contains("coordinates")) {
        continue;
      }
      GraphNode node;
      node.id = props["id"].get<int>();
      const auto & coords = geom["coordinates"];
      if (!coords.is_array() || coords.size() < 2) {
        continue;
      }
      node.x = coords[0].get<double>();
      node.y = coords[1].get<double>();
      nodes_[node.id] = node;
    }
  }

  std::size_t edge_count = 0;
  for (const auto & feat : root["features"]) {
    if (!feat.contains("geometry") || !feat.contains("properties")) {
      continue;
    }
    const auto & geom = feat["geometry"];
    const auto & props = feat["properties"];
    const std::string type = geom.value("type", "");
    if (type != "MultiLineString" && type != "LineString") {
      continue;
    }
    if (!props.contains("startid") || !props.contains("endid")) {
      continue;
    }
    const int start_id = props["startid"].get<int>();
    const int end_id = props["endid"].get<int>();
    if (!nodes_.count(start_id) || !nodes_.count(end_id)) {
      continue;
    }
    const auto & a = nodes_.at(start_id);
    const auto & b = nodes_.at(end_id);
    GraphEdge edge;
    edge.to = end_id;
    edge.cost = hypot2(b.x - a.x, b.y - a.y);
    if (edge.cost < 1e-6) {
      edge.cost = 1e-6;
    }
    adj_[start_id].push_back(edge);
    ++edge_count;
  }

  if (nodes_.empty() || edge_count == 0) {
    RCLCPP_ERROR(
      logger_,
      "GraphPlanner: graph has %zu nodes / %zu edges — invalid",
      nodes_.size(), edge_count);
    nodes_.clear();
    adj_.clear();
    return false;
  }

  RCLCPP_INFO(
    logger_, "Loaded GeoJSON graph: %zu nodes, %zu directed edges",
    nodes_.size(), edge_count);
  return true;
}

int GraphPlanner::nearestNode(double x, double y, double * dist_out) const
{
  int best = -1;
  double best_d = std::numeric_limits<double>::infinity();
  for (const auto & kv : nodes_) {
    const double d = hypot2(kv.second.x - x, kv.second.y - y);
    if (d < best_d) {
      best_d = d;
      best = kv.first;
    }
  }
  if (dist_out) {
    *dist_out = best_d;
  }
  return best;
}

std::vector<int> GraphPlanner::dijkstra(int start_id, int goal_id) const
{
  std::unordered_map<int, double> dist;
  std::unordered_map<int, int> prev;
  using QItem = std::pair<double, int>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;

  for (const auto & kv : nodes_) {
    dist[kv.first] = std::numeric_limits<double>::infinity();
  }
  dist[start_id] = 0.0;
  open.push({0.0, start_id});

  while (!open.empty()) {
    const auto [cost_u, u] = open.top();
    open.pop();
    if (cost_u > dist.at(u)) {
      continue;
    }
    if (u == goal_id) {
      break;
    }
    const auto it = adj_.find(u);
    if (it == adj_.end()) {
      continue;
    }
    for (const auto & edge : it->second) {
      const double cand = cost_u + edge.cost;
      if (cand < dist[edge.to]) {
        dist[edge.to] = cand;
        prev[edge.to] = u;
        open.push({cand, edge.to});
      }
    }
  }

  std::vector<int> path;
  if (start_id != goal_id && !prev.count(goal_id)) {
    return path;
  }
  if (start_id == goal_id) {
    path.push_back(start_id);
    return path;
  }

  for (int cur = goal_id; ; ) {
    path.push_back(cur);
    if (cur == start_id) {
      break;
    }
    cur = prev.at(cur);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

void GraphPlanner::setYaw(geometry_msgs::msg::PoseStamped & pose, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
}

nav_msgs::msg::Path GraphPlanner::densify(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<int> & node_ids) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = start.header.stamp;

  auto append_xy = [&](double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = 0.0;
    setYaw(p, yaw);
    path.poses.push_back(p);
  };

  // Start at robot pose, then walk graph nodes, then finish at goal pose.
  std::vector<std::pair<double, double>> waypoints;
  waypoints.emplace_back(start.pose.position.x, start.pose.position.y);
  for (int id : node_ids) {
    const auto & n = nodes_.at(id);
    // Skip duplicate if robot already almost on first node
    if (!waypoints.empty()) {
      const auto & last = waypoints.back();
      if (hypot2(n.x - last.first, n.y - last.second) < 0.02) {
        continue;
      }
    }
    waypoints.emplace_back(n.x, n.y);
  }
  {
    const double gx = goal.pose.position.x;
    const double gy = goal.pose.position.y;
    if (hypot2(gx - waypoints.back().first, gy - waypoints.back().second) > 0.02) {
      waypoints.emplace_back(gx, gy);
    }
  }

  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const double x0 = waypoints[i].first;
    const double y0 = waypoints[i].second;
    const double x1 = waypoints[i + 1].first;
    const double y1 = waypoints[i + 1].second;
    const double seg = hypot2(x1 - x0, y1 - y0);
    const double yaw = std::atan2(y1 - y0, x1 - x0);
    const int steps = std::max(1, static_cast<int>(std::ceil(seg / path_density_)));
    for (int s = 0; s < steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(steps);
      append_xy(x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw);
    }
  }
  // Final goal pose (keep requested yaw for approach)
  geometry_msgs::msg::PoseStamped g = goal;
  g.header = path.header;
  path.poses.push_back(g);

  return path;
}

nav_msgs::msg::Path GraphPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty;
  empty.header.frame_id = global_frame_;
  empty.header.stamp = start.header.stamp;

  if (!graph_loaded_) {
    RCLCPP_ERROR(logger_, "GraphPlanner: no graph loaded — cannot plan");
    return empty;
  }

  const double sx = start.pose.position.x;
  const double sy = start.pose.position.y;
  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  double d_start = 0.0;
  double d_goal = 0.0;
  const int start_id = nearestNode(sx, sy, &d_start);
  const int goal_id = nearestNode(gx, gy, &d_goal);

  if (start_id < 0 || goal_id < 0) {
    RCLCPP_ERROR(logger_, "GraphPlanner: empty node set");
    return empty;
  }

  if (d_start > max_snap_distance_) {
    RCLCPP_WARN(
      logger_,
      "Start (%.2f,%.2f) is %.2fm from nearest node %d (max_snap=%.2f)",
      sx, sy, d_start, start_id, max_snap_distance_);
    return empty;
  }
  if (d_goal > max_snap_distance_) {
    RCLCPP_WARN(
      logger_,
      "Goal (%.2f,%.2f) is %.2fm from nearest node %d (max_snap=%.2f)",
      gx, gy, d_goal, goal_id, max_snap_distance_);
    return empty;
  }

  if (start_id == goal_id && !allow_same_node_) {
    RCLCPP_WARN(logger_, "Start and goal snap to same node %d", start_id);
    return empty;
  }

  const auto node_path = dijkstra(start_id, goal_id);
  if (node_path.empty()) {
    RCLCPP_WARN(
      logger_,
      "No graph route from node %d (%.2f,%.2f) to node %d (%.2f,%.2f)",
      start_id, sx, sy, goal_id, gx, gy);
    return empty;
  }

  auto path = densify(start, goal, node_path);
  RCLCPP_INFO(
    logger_,
    "Plan OK: start_node=%d (d=%.2f) goal_node=%d (d=%.2f) via %zu graph nodes → %zu poses",
    start_id, d_start, goal_id, d_goal, node_path.size(), path.poses.size());
  return path;
}

}  // namespace skill23_graph_planner

PLUGINLIB_EXPORT_CLASS(skill23_graph_planner::GraphPlanner, nav2_core::GlobalPlanner)
