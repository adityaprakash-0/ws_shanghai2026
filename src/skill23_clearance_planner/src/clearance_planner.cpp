#include "skill23_clearance_planner/clearance_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace skill23_clearance_planner
{

namespace
{
constexpr unsigned char kLethal = nav2_costmap_2d::LETHAL_OBSTACLE;       // 254
constexpr unsigned char kInscribed = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;  // 253
constexpr unsigned char kUnknown = nav2_costmap_2d::NO_INFORMATION;       // 255
constexpr uint16_t kDistInf = 65535;

struct QueueNode
{
  int idx;
  double f;
  bool operator<(const QueueNode & other) const {return f > other.f;}
};
}  // namespace

void ClearancePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  auto node = parent_.lock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_narrow", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".straight_line_only", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_final_approach_orientation", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_distance", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_cell_cost", rclcpp::ParameterValue(15.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_inflation", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".narrow_wall_penalty", rclcpp::ParameterValue(50.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".edt_narrow_weight", rclcpp::ParameterValue(40.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".turn_penalty", rclcpp::ParameterValue(16.0));

  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".allow_narrow", allow_narrow_);
  node->get_parameter(name_ + ".straight_line_only", straight_line_only_);
  node->get_parameter(name_ + ".use_final_approach_orientation", use_final_approach_orientation_);
  node->get_parameter(name_ + ".w_distance", w_distance_);
  node->get_parameter(name_ + ".w_cell_cost", w_cell_cost_);
  node->get_parameter(name_ + ".w_inflation", w_inflation_);
  node->get_parameter(name_ + ".narrow_wall_penalty", narrow_wall_penalty_);
  node->get_parameter(name_ + ".edt_narrow_weight", edt_narrow_weight_);
  node->get_parameter(name_ + ".turn_penalty", turn_penalty_);

  RCLCPP_INFO(
    logger_,
    "[%s] configured: allow_unknown=%s allow_narrow=%s straight_line_only=%s "
    "w_distance=%.2f w_cell_cost=%.2f w_inflation=%.2f turn_penalty=%.2f edt_narrow_weight=%.2f",
    name_.c_str(), allow_unknown_ ? "true" : "false", allow_narrow_ ? "true" : "false",
    straight_line_only_ ? "true" : "false", w_distance_, w_cell_cost_, w_inflation_,
    turn_penalty_, edt_narrow_weight_);
}

void ClearancePlanner::cleanup()
{
  lethal_dist_.clear();
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
}

void ClearancePlanner::activate() {}
void ClearancePlanner::deactivate() {}

bool ClearancePlanner::inBounds(int x, int y) const
{
  return x >= 0 && y >= 0 &&
         x < static_cast<int>(costmap_->getSizeInCellsX()) &&
         y < static_cast<int>(costmap_->getSizeInCellsY());
}

unsigned char ClearancePlanner::cellCost(int x, int y) const
{
  return costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
}

void ClearancePlanner::buildLethalDistanceField()
{
  const int width = static_cast<int>(costmap_->getSizeInCellsX());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());
  const int total = width * height;
  lethal_dist_.assign(static_cast<size_t>(total), kDistInf);
  lethal_dist_width_ = width;
  lethal_dist_height_ = height;

  std::queue<int> q;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (cellCost(x, y) == kLethal) {
        const int idx = y * width + x;
        lethal_dist_[static_cast<size_t>(idx)] = 0;
        q.push(idx);
      }
    }
  }

  static const int kDx[4] = {1, -1, 0, 0};
  static const int kDy[4] = {0, 0, 1, -1};
  while (!q.empty()) {
    const int idx = q.front();
    q.pop();
    const int x = idx % width;
    const int y = idx / width;
    const uint16_t nd = static_cast<uint16_t>(lethal_dist_[static_cast<size_t>(idx)] + 1);
    for (int i = 0; i < 4; ++i) {
      const int nx = x + kDx[i];
      const int ny = y + kDy[i];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      const int nidx = ny * width + nx;
      if (nd < lethal_dist_[static_cast<size_t>(nidx)]) {
        lethal_dist_[static_cast<size_t>(nidx)] = nd;
        q.push(nidx);
      }
    }
  }
}

uint16_t ClearancePlanner::lethalDist(int x, int y) const
{
  if (lethal_dist_.empty() ||
    x < 0 || y < 0 ||
    x >= lethal_dist_width_ || y >= lethal_dist_height_)
  {
    return kDistInf;
  }
  return lethal_dist_[static_cast<size_t>(y * lethal_dist_width_ + x)];
}

bool ClearancePlanner::isAllowedCost(unsigned char cost, bool allow_narrow) const
{
  if (cost >= kLethal && cost != kUnknown) {
    return false;
  }
  if (cost == kUnknown && !allow_unknown_) {
    return false;
  }
  if (cost == kInscribed && !allow_narrow) {
    return false;
  }
  return true;
}

double ClearancePlanner::heuristic(const GridPoint & a, const GridPoint & b) const
{
  const int dx = std::abs(a.x - b.x);
  const int dy = std::abs(a.y - b.y);
  if (straight_line_only_) {
    return w_distance_ * static_cast<double>(dx + dy);
  }
  return w_distance_ * std::hypot(static_cast<double>(dx), static_cast<double>(dy));
}

double ClearancePlanner::traversalPenalty(unsigned char cost, bool allow_narrow, int x, int y) const
{
  if (cost == kUnknown) {
    return w_inflation_;
  }
  if (cost == kInscribed && allow_narrow) {
    // Distance-to-254: closer to lethal => higher cost (prefer outer 253).
    const uint16_t d = lethalDist(x, y);
    const double dist = (d == kDistInf) ? 20.0 : static_cast<double>(d);
    return w_cell_cost_ * static_cast<double>(kInscribed)
      + w_inflation_ + narrow_wall_penalty_ + edt_narrow_weight_ / (dist + 0.5);
  }
  // Prefer lowest occupancy cost (0 best, 252 worst among free cells).
  return w_cell_cost_ * static_cast<double>(cost)
    + w_inflation_ * (static_cast<double>(cost) / 252.0);
}

bool ClearancePlanner::snapToAllowed(int & mx, int & my, bool allow_narrow, int max_r) const
{
  if (inBounds(mx, my) && isAllowedCost(cellCost(mx, my), allow_narrow)) {
    return true;
  }
  int best_d = max_r * max_r + 1;
  uint16_t best_clear = 0;
  int bx = mx;
  int by = my;
  bool found = false;
  for (int dy = -max_r; dy <= max_r; ++dy) {
    for (int dx = -max_r; dx <= max_r; ++dx) {
      const int nx = mx + dx;
      const int ny = my + dy;
      if (!inBounds(nx, ny) || !isAllowedCost(cellCost(nx, ny), allow_narrow)) {
        continue;
      }
      const int d = dx * dx + dy * dy;
      const uint16_t clear = allow_narrow ? lethalDist(nx, ny) : kDistInf;
      // Prefer closer snap; when narrow, break ties toward farther from 254.
      if (!found || d < best_d || (d == best_d && clear > best_clear)) {
        best_d = d;
        best_clear = clear;
        bx = nx;
        by = ny;
        found = true;
      }
    }
  }
  if (!found) {
    return false;
  }
  mx = bx;
  my = by;
  return true;
}

bool ClearancePlanner::axisLineFree(int x0, int y0, int x1, int y1, bool allow_narrow) const
{
  if (x0 != x1 && y0 != y1) {
    return false;
  }
  const int n = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
  if (n == 0) {
    return inBounds(x0, y0) && isAllowedCost(cellCost(x0, y0), allow_narrow);
  }
  const int dx = (x1 - x0) / n;
  const int dy = (y1 - y0) / n;
  for (int i = 0; i <= n; ++i) {
    const int x = x0 + i * dx;
    const int y = y0 + i * dy;
    if (!inBounds(x, y) || !isAllowedCost(cellCost(x, y), allow_narrow)) {
      return false;
    }
  }
  return true;
}

bool ClearancePlanner::lShapeFree(
  int x0, int y0, int x1, int y1, bool allow_narrow, GridPoint & corner) const
{
  const bool xy = axisLineFree(x0, y0, x1, y0, allow_narrow) &&
    axisLineFree(x1, y0, x1, y1, allow_narrow);
  const bool yx = axisLineFree(x0, y0, x0, y1, allow_narrow) &&
    axisLineFree(x0, y1, x1, y1, allow_narrow);
  if (xy && yx) {
    auto peak = [this](int ax, int ay, int bx, int by) {
        const int n = std::max(std::abs(bx - ax), std::abs(by - ay));
        unsigned char p = 0;
        const int dx = n ? (bx - ax) / n : 0;
        const int dy = n ? (by - ay) / n : 0;
        for (int i = 0; i <= n; ++i) {
          p = std::max(p, cellCost(ax + i * dx, ay + i * dy));
        }
        return p;
      };
    const unsigned char xy_peak = std::max(peak(x0, y0, x1, y0), peak(x1, y0, x1, y1));
    const unsigned char yx_peak = std::max(peak(x0, y0, x0, y1), peak(x0, y1, x1, y1));
    corner = (xy_peak <= yx_peak) ? GridPoint{x1, y0} : GridPoint{x0, y1};
    return true;
  }
  if (xy) {
    corner = {x1, y0};
    return true;
  }
  if (yx) {
    corner = {x0, y1};
    return true;
  }
  return false;
}

bool ClearancePlanner::runAStar(
  const GridPoint & start, const GridPoint & goal, bool allow_narrow,
  std::vector<GridPoint> & raw_path)
{
  const int width = static_cast<int>(costmap_->getSizeInCellsX());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());
  const int total = width * height;

  auto toIndex = [width](int x, int y) {return y * width + x;};
  auto toPoint = [width](int idx) {
      return GridPoint{idx % width, idx / width};
    };

  std::vector<double> g(total, std::numeric_limits<double>::infinity());
  std::vector<int> parent(total, -1);
  std::vector<unsigned char> state(total, 0);  // 0 unseen, 1 open, 2 closed

  const int sidx = toIndex(start.x, start.y);
  const int gidx = toIndex(goal.x, goal.y);
  g[sidx] = 0.0;

  std::priority_queue<QueueNode> pq;
  pq.push({sidx, heuristic(start, goal)});
  state[sidx] = 1;

  std::vector<GridPoint> moves;
  if (straight_line_only_) {
    moves = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  } else {
    moves = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  }

  while (!pq.empty()) {
    const QueueNode current = pq.top();
    pq.pop();
    if (state[current.idx] == 2) {
      continue;
    }
    state[current.idx] = 2;

    if (current.idx == gidx) {
      std::vector<GridPoint> reverse_path;
      int it = gidx;
      while (it >= 0) {
        reverse_path.push_back(toPoint(it));
        if (it == sidx) {
          break;
        }
        it = parent[it];
      }
      if (reverse_path.empty() ||
        reverse_path.back().x != start.x || reverse_path.back().y != start.y)
      {
        return false;
      }
      raw_path.assign(reverse_path.rbegin(), reverse_path.rend());
      return true;
    }

    const GridPoint cp = toPoint(current.idx);
    int in_dx = 0;
    int in_dy = 0;
    if (parent[current.idx] >= 0) {
      const GridPoint pp = toPoint(parent[current.idx]);
      in_dx = cp.x - pp.x;
      in_dy = cp.y - pp.y;
    }

    for (const auto & m : moves) {
      const int nx = cp.x + m.x;
      const int ny = cp.y + m.y;
      if (!inBounds(nx, ny)) {
        continue;
      }
      const unsigned char ccost = cellCost(nx, ny);
      if (!isAllowedCost(ccost, allow_narrow)) {
        continue;
      }
      const int nidx = toIndex(nx, ny);
      if (state[nidx] == 2) {
        continue;
      }

      const double step = (m.x != 0 && m.y != 0) ? std::sqrt(2.0) : 1.0;
      double turn = 0.0;
      if (parent[current.idx] >= 0 && (m.x != in_dx || m.y != in_dy)) {
        turn = turn_penalty_;
      }
      const double tentative =
        g[current.idx] + w_distance_ * step + turn + traversalPenalty(ccost, allow_narrow, nx, ny);

      if (tentative < g[nidx]) {
        g[nidx] = tentative;
        parent[nidx] = current.idx;
        const double f = tentative + heuristic({nx, ny}, goal);
        pq.push({nidx, f});
        state[nidx] = 1;
      }
    }
  }

  return false;
}

void ClearancePlanner::simplifyCollinear(std::vector<GridPoint> & path) const
{
  if (path.size() < 3) {
    return;
  }
  std::vector<GridPoint> out;
  out.reserve(path.size());
  out.push_back(path.front());
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    const auto & prev = out.back();
    const auto & cur = path[i];
    const auto & next = path[i + 1];
    const int d1x = cur.x - prev.x;
    const int d1y = cur.y - prev.y;
    const int d2x = next.x - cur.x;
    const int d2y = next.y - cur.y;
    const int cross = d1x * d2y - d1y * d2x;
    const int dot = d1x * d2x + d1y * d2y;
    if (cross == 0 && dot > 0) {
      continue;
    }
    out.push_back(cur);
  }
  out.push_back(path.back());
  path.swap(out);
}

void ClearancePlanner::collapseUnitStairs(
  std::vector<GridPoint> & path, bool allow_narrow) const
{
  // Replace short axis zig-zags (staircases) with a single L when free.
  if (path.size() < 4) {
    return;
  }
  bool changed = true;
  int guard = 0;
  while (changed && guard++ < 8) {
    changed = false;
    simplifyCollinear(path);
    std::vector<GridPoint> out;
    out.reserve(path.size());
    size_t i = 0;
    while (i < path.size()) {
      if (i + 3 < path.size()) {
        const GridPoint & a = path[i];
        const GridPoint & d = path[i + 3];
        // Skip three intermediate corners if an L from a→d is free.
        GridPoint corner;
        if ((a.x != d.x && a.y != d.y) && lShapeFree(a.x, a.y, d.x, d.y, allow_narrow, corner)) {
          out.push_back(a);
          if ((corner.x != a.x || corner.y != a.y) &&
            (corner.x != d.x || corner.y != d.y))
          {
            out.push_back(corner);
          }
          i += 3;
          changed = true;
          continue;
        }
      }
      out.push_back(path[i]);
      ++i;
    }
    path.swap(out);
  }
  simplifyCollinear(path);
}

std::vector<ClearancePlanner::GridPoint> ClearancePlanner::orthogonalSmooth(
  const std::vector<GridPoint> & path, bool allow_narrow) const
{
  if (path.size() < 3) {
    return path;
  }
  std::vector<GridPoint> out;
  out.reserve(path.size());
  size_t i = 0;
  while (i + 1 < path.size()) {
    out.push_back(path[i]);
    size_t best = i + 1;
    GridPoint corner{path[i].x, path[i].y};
    bool has_corner = false;
    for (size_t j = path.size() - 1; j > i; --j) {
      const auto & a = path[i];
      const auto & b = path[j];
      if (a.x == b.x || a.y == b.y) {
        if (axisLineFree(a.x, a.y, b.x, b.y, allow_narrow)) {
          best = j;
          has_corner = false;
          break;
        }
      } else {
        GridPoint c;
        if (lShapeFree(a.x, a.y, b.x, b.y, allow_narrow, c)) {
          best = j;
          corner = c;
          has_corner = true;
          break;
        }
      }
    }
    if (has_corner &&
      (corner.x != path[i].x || corner.y != path[i].y) &&
      (corner.x != path[best].x || corner.y != path[best].y))
    {
      out.push_back(corner);
    }
    i = best;
  }
  out.push_back(path.back());
  return out;
}

void ClearancePlanner::densifyAxisAligned(
  std::vector<GridPoint> & path, bool allow_narrow) const
{
  if (path.size() < 2) {
    return;
  }
  std::vector<GridPoint> out;
  out.reserve(path.size() * 2);
  out.push_back(path.front());

  auto appendAxis = [&](const GridPoint & a, const GridPoint & b) {
      const int n = std::max(std::abs(b.x - a.x), std::abs(b.y - a.y));
      if (n <= 0) {
        return;
      }
      const int dx = (b.x - a.x) / n;
      const int dy = (b.y - a.y) / n;
      for (int k = 1; k <= n; ++k) {
        out.push_back({a.x + k * dx, a.y + k * dy});
      }
    };

  for (size_t i = 1; i < path.size(); ++i) {
    GridPoint a = out.back();
    const GridPoint b = path[i];
    if (a.x != b.x && a.y != b.y) {
      GridPoint c{b.x, a.y};
      if (!lShapeFree(a.x, a.y, b.x, b.y, allow_narrow, c)) {
        c = {b.x, a.y};
      }
      if (c.x != a.x || c.y != a.y) {
        appendAxis(a, c);
        a = out.back();
      }
    }
    appendAxis(a, b);
  }
  path.swap(out);
}

geometry_msgs::msg::Pose ClearancePlanner::makePoseFromCell(int mx, int my) const
{
  double wx = 0.0;
  double wy = 0.0;
  costmap_->mapToWorld(
    static_cast<unsigned int>(mx),
    static_cast<unsigned int>(my),
    wx, wy);
  geometry_msgs::msg::Pose p;
  p.position.x = wx;
  p.position.y = wy;
  p.position.z = 0.0;
  p.orientation.w = 1.0;
  return p;
}

void ClearancePlanner::assignSmoothOrientations(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (path.poses.empty()) {
    return;
  }

  // Constant yaw along each straight axis segment so RPP does not micro-rotate.
  auto seg_dir = [](double dx, double dy) {
      if (std::abs(dx) >= std::abs(dy)) {
        return dx >= 0.0 ? 1 : 3;  // +x / -x
      }
      return dy >= 0.0 ? 2 : 4;  // +y / -y
    };

  size_t i = 0;
  while (i + 1 < path.poses.size()) {
    const double dx0 =
      path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    const double dy0 =
      path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    const int dir0 = seg_dir(dx0, dy0);
    size_t j = i + 1;
    while (j + 1 < path.poses.size()) {
      const double dx =
        path.poses[j + 1].pose.position.x - path.poses[j].pose.position.x;
      const double dy =
        path.poses[j + 1].pose.position.y - path.poses[j].pose.position.y;
      if (seg_dir(dx, dy) != dir0) {
        break;
      }
      ++j;
    }
    const double yaw = std::atan2(
      path.poses[j].pose.position.y - path.poses[i].pose.position.y,
      path.poses[j].pose.position.x - path.poses[i].pose.position.x);
    const auto q = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    for (size_t k = i; k < j; ++k) {
      path.poses[k].pose.orientation = q;
    }
    // Corner pose gets the outgoing segment yaw on the next loop.
    i = j;
  }

  if (path.poses.size() == 1) {
    path.poses.front().pose.orientation = start.pose.orientation;
  }

  if (use_final_approach_orientation_) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  } else if (path.poses.size() >= 2) {
    path.poses.back().pose.orientation =
      path.poses[path.poses.size() - 2].pose.orientation;
  }
}

nav_msgs::msg::Path ClearancePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = global_frame_;

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      logger_, "[%s] start/goal frames must be '%s' but got start='%s' goal='%s'",
      name_.c_str(), global_frame_.c_str(),
      start.header.frame_id.c_str(), goal.header.frame_id.c_str());
    return path;
  }

  unsigned int usx, usy, ugx, ugy;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, usx, usy)) {
    RCLCPP_ERROR(logger_, "[%s] Start pose is outside costmap", name_.c_str());
    return path;
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, ugx, ugy)) {
    RCLCPP_ERROR(logger_, "[%s] Goal pose is outside costmap", name_.c_str());
    return path;
  }

  int sx = static_cast<int>(usx);
  int sy = static_cast<int>(usy);
  int gx = static_cast<int>(ugx);
  int gy = static_cast<int>(ugy);
  constexpr int kSnapR = 12;
  const int orig_gx = gx;
  const int orig_gy = gy;

  // Build EDT before narrow snap / search so outer-253 preference works.
  if (allow_narrow_) {
    buildLethalDistanceField();
  } else {
    lethal_dist_.clear();
    lethal_dist_width_ = 0;
    lethal_dist_height_ = 0;
  }

  if (!snapToAllowed(sx, sy, false, kSnapR) &&
    !(allow_narrow_ && snapToAllowed(sx, sy, true, kSnapR)))
  {
    RCLCPP_ERROR(
      logger_, "[%s] Start pose cell cost=%u is blocked and no free cell nearby",
      name_.c_str(), static_cast<unsigned int>(cellCost(static_cast<int>(usx), static_cast<int>(usy))));
    return path;
  }
  if (!snapToAllowed(gx, gy, false, kSnapR) &&
    !(allow_narrow_ && snapToAllowed(gx, gy, true, kSnapR)))
  {
    RCLCPP_ERROR(
      logger_,
      "[%s] Goal pose (%.2f, %.2f) cell cost=%u is blocked and no free cell nearby",
      name_.c_str(), goal.pose.position.x, goal.pose.position.y,
      static_cast<unsigned int>(cellCost(static_cast<int>(ugx), static_cast<int>(ugy))));
    return path;
  }
  if (gx != orig_gx || gy != orig_gy) {
    RCLCPP_WARN(
      logger_,
      "[%s] Goal snapped from cell (%d,%d) to (%d,%d) to reach (%.2f, %.2f)",
      name_.c_str(), orig_gx, orig_gy, gx, gy,
      goal.pose.position.x, goal.pose.position.y);
  }

  std::vector<GridPoint> raw;
  const GridPoint s{sx, sy};
  const GridPoint g{gx, gy};

  // Always try free (0–252) first. Narrow (253 + EDT) only if allow_narrow_.
  bool found = runAStar(s, g, false, raw);
  bool used_narrow_fallback = false;
  if (!found && allow_narrow_) {
    found = runAStar(s, g, true, raw);
    used_narrow_fallback = found;
  }

  if (!found) {
    RCLCPP_ERROR(
      logger_,
      "[%s] Could not generate path between start=(%.2f, %.2f) and goal=(%.2f, %.2f)",
      name_.c_str(),
      start.pose.position.x, start.pose.position.y,
      goal.pose.position.x, goal.pose.position.y);
    return path;
  }

  const bool used_narrow = used_narrow_fallback;
  simplifyCollinear(raw);
  raw = orthogonalSmooth(raw, used_narrow);
  simplifyCollinear(raw);
  collapseUnitStairs(raw, used_narrow);
  raw = orthogonalSmooth(raw, used_narrow);
  simplifyCollinear(raw);
  densifyAxisAligned(raw, used_narrow);

  if (used_narrow_fallback) {
    RCLCPP_WARN(
      logger_, "[%s] Planned using 253 fallback cells with distance-to-254 EDT",
      name_.c_str());
  }

  path.poses.reserve(raw.size());
  for (const auto & cell : raw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose = makePoseFromCell(cell.x, cell.y);
    path.poses.push_back(pose);
  }

  assignSmoothOrientations(path, start, goal);

  if (!path.poses.empty()) {
    if (gx == orig_gx && gy == orig_gy) {
      path.poses.back().pose.position = goal.pose.position;
    }
    if (use_final_approach_orientation_) {
      path.poses.back().pose.orientation = goal.pose.orientation;
    }
  }

  return path;
}

}  // namespace skill23_clearance_planner

PLUGINLIB_EXPORT_CLASS(skill23_clearance_planner::ClearancePlanner, nav2_core::GlobalPlanner)
