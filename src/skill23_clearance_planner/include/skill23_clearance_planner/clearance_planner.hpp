#ifndef SKILL23_CLEARANCE_PLANNER__CLEARANCE_PLANNER_HPP_
#define SKILL23_CLEARANCE_PLANNER__CLEARANCE_PLANNER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace skill23_clearance_planner
{

class ClearancePlanner : public nav2_core::GlobalPlanner
{
public:
  ClearancePlanner() = default;
  ~ClearancePlanner() override = default;

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
  struct GridPoint
  {
    int x;
    int y;
  };

  bool runAStar(
    const GridPoint & start, const GridPoint & goal, bool allow_narrow,
    std::vector<GridPoint> & raw_path);

  void buildLethalDistanceField();
  uint16_t lethalDist(int x, int y) const;

  struct AxisMetrics
  {
    unsigned char peak{0};
    double sum{0.0};
    int cells{0};
    double mean() const {return cells > 0 ? sum / static_cast<double>(cells) : 0.0;}
  };

  bool isAllowedCost(unsigned char cost, bool allow_narrow) const;
  bool inBounds(int x, int y) const;
  unsigned char cellCost(int x, int y) const;
  double traversalPenalty(unsigned char cost, bool allow_narrow, int x, int y) const;
  double heuristic(const GridPoint & a, const GridPoint & b) const;
  bool snapToAllowed(int & mx, int & my, bool allow_narrow, int max_r) const;
  // If goal is on inscribed (253), move to nearest free (0–252) cell.
  // Prefer closer, then axis-aligned (parallel), then lower cost.
  bool snapGoalOffInscribed(int & mx, int & my, int max_r) const;
  bool axisLineFree(int x0, int y0, int x1, int y1, bool allow_narrow) const;
  AxisMetrics axisMetrics(int x0, int y0, int x1, int y1) const;
  AxisMetrics pathMetrics(
    const std::vector<GridPoint> & path, size_t i, size_t j) const;
  bool metricsNotWorse(const AxisMetrics & link, const AxisMetrics & orig) const;
  bool lShapeFree(
    int x0, int y0, int x1, int y1, bool allow_narrow, GridPoint & corner) const;
  bool lShapeFreeMetrics(
    int x0, int y0, int x1, int y1, bool allow_narrow,
    GridPoint & corner, AxisMetrics & metrics) const;
  void simplifyCollinear(std::vector<GridPoint> & path) const;
  void collapseUnitStairs(std::vector<GridPoint> & path, bool allow_narrow) const;
  std::vector<GridPoint> orthogonalSmooth(
    const std::vector<GridPoint> & path, bool allow_narrow) const;
  void densifyAxisAligned(std::vector<GridPoint> & path, bool allow_narrow) const;
  void assignSmoothOrientations(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  void assignTangentOrientations(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  geometry_msgs::msg::Pose makePoseFromCell(int mx, int my) const;

  struct WorldPoint
  {
    double x{0.0};
    double y{0.0};
  };

  std::vector<WorldPoint> gridPathToWorld(const std::vector<GridPoint> & path) const;
  std::vector<WorldPoint> densifyWorldPath(
    const std::vector<WorldPoint> & path, double spacing) const;
  double pointToPolylineDistance(
    double x, double y, const std::vector<WorldPoint> & poly) const;
  bool worldPointAllowed(double wx, double wy, bool allow_narrow) const;
  std::vector<WorldPoint> constrainedSmoothDiagonals(
    const std::vector<WorldPoint> & path, bool allow_narrow) const;

  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("clearance_planner")};
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;

  bool allow_unknown_{false};
  bool allow_narrow_{false};
  bool shift_goal_off_inscribed_{false};
  bool straight_line_only_{true};
  bool smooth_path_{true};
  int smooth_iterations_{10};
  double max_deviation_{0.05};
  bool use_final_approach_orientation_{true};
  double w_distance_{1.0};
  double w_cell_cost_{15.0};
  double w_inflation_{2.0};
  double narrow_wall_penalty_{50.0};
  double edt_narrow_weight_{40.0};
  double turn_penalty_{16.0};

  std::vector<uint16_t> lethal_dist_;
  int lethal_dist_width_{0};
  int lethal_dist_height_{0};
};

}  // namespace skill23_clearance_planner

#endif  // SKILL23_CLEARANCE_PLANNER__CLEARANCE_PLANNER_HPP_
