#include "skill23_custom_costmap/custom_dynamic_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace skill23_custom_costmap
{

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

void CustomDynamicLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("CustomDynamicLayer: failed to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("scan_topic", rclcpp::ParameterValue(std::string("/scan")));
  declareParameter("map_topic", rclcpp::ParameterValue(std::string("/map")));
  declareParameter("obstacle_max_range", rclcpp::ParameterValue(2.5));
  declareParameter("obstacle_min_range", rclcpp::ParameterValue(0.12));
  declareParameter("raytrace_max_range", rclcpp::ParameterValue(3.0));
  declareParameter("obstacle_confirmation_count", rclcpp::ParameterValue(2));
  declareParameter("clear_confirmation_count", rclcpp::ParameterValue(3));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(1.0));
  declareParameter("occupied_thresh", rclcpp::ParameterValue(50));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "scan_topic", scan_topic_);
  node->get_parameter(name_ + "." + "map_topic", map_topic_);
  node->get_parameter(name_ + "." + "obstacle_max_range", obstacle_max_range_);
  node->get_parameter(name_ + "." + "obstacle_min_range", obstacle_min_range_);
  node->get_parameter(name_ + "." + "raytrace_max_range", raytrace_max_range_);
  node->get_parameter(name_ + "." + "obstacle_confirmation_count", obstacle_confirmation_count_);
  node->get_parameter(name_ + "." + "clear_confirmation_count", clear_confirmation_count_);
  node->get_parameter(name_ + "." + "transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + "." + "occupied_thresh", occupied_thresh_);

  obstacle_confirmation_count_ = std::max(1, obstacle_confirmation_count_);
  clear_confirmation_count_ = std::max(1, clear_confirmation_count_);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();
  matchSize();

  // Match ObstacleLayer QoS used elsewhere in Studio (sensor data).
  auto sensor_qos = rclcpp::SensorDataQoS();
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, sensor_qos,
    std::bind(&CustomDynamicLayer::laserCallback, this, std::placeholders::_1));

  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_qos.reliable();
  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&CustomDynamicLayer::mapCallback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(
    logger_,
    "[%s] CustomDynamicLayer enabled topic=%s confirm_obs=%d confirm_clear=%d "
    "obs_range=%.2f clear_range=%.2f",
    name_.c_str(), scan_topic_.c_str(),
    obstacle_confirmation_count_, clear_confirmation_count_,
    obstacle_max_range_, raytrace_max_range_);
}

void CustomDynamicLayer::activate()
{
  current_ = true;
}

void CustomDynamicLayer::deactivate()
{
  // Keep subscriptions; layer may be toggled via enabled_.
}

void CustomDynamicLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  std::fill(state_.begin(), state_.end(), CellState::NO_OVERRIDE);
  std::fill(obstacle_hits_.begin(), obstacle_hits_.end(), 0);
  std::fill(clear_hits_.begin(), clear_hits_.end(), 0);
  need_reprocess_ = false;
  latest_scan_.reset();
  has_touch_ = false;
  current_ = true;
}

void CustomDynamicLayer::matchSize()
{
  auto * master = layered_costmap_->getCostmap();
  ensureStorage(master->getSizeInCellsX(), master->getSizeInCellsY());
}

void CustomDynamicLayer::ensureStorage(unsigned int size_x, unsigned int size_y)
{
  if (size_x == size_x_ && size_y == size_y_ && !state_.empty()) {
    return;
  }
  size_x_ = size_x;
  size_y_ = size_y;
  const size_t n = static_cast<size_t>(size_x_) * static_cast<size_t>(size_y_);
  state_.assign(n, CellState::NO_OVERRIDE);
  obstacle_hits_.assign(n, 0);
  clear_hits_.assign(n, 0);
  // Keep static_lethal_ if same size; else reset and wait for map.
  if (static_lethal_.size() != n) {
    static_lethal_.assign(n, 0);
  }
}

void CustomDynamicLayer::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
  need_reprocess_ = true;
}

void CustomDynamicLayer::mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  const unsigned int mx = msg->info.width;
  const unsigned int my = msg->info.height;
  if (mx == 0 || my == 0) {
    return;
  }
  // Align to layered costmap size when available.
  auto * master = layered_costmap_->getCostmap();
  if (master->getSizeInCellsX() == mx && master->getSizeInCellsY() == my) {
    ensureStorage(mx, my);
    static_lethal_.assign(static_cast<size_t>(mx) * my, 0);
    for (size_t i = 0; i < msg->data.size() && i < static_lethal_.size(); ++i) {
      const int8_t v = msg->data[i];
      static_lethal_[i] = (v >= occupied_thresh_) ? 1 : 0;
    }
  } else {
    // Map not yet matching costmap — store by world lookup later if needed.
    // For Studio, map_server and global costmap share size after resize.
    ensureStorage(master->getSizeInCellsX(), master->getSizeInCellsY());
    const double res = msg->info.resolution;
    const double ox = msg->info.origin.position.x;
    const double oy = msg->info.origin.position.y;
    for (unsigned int j = 0; j < size_y_; ++j) {
      for (unsigned int i = 0; i < size_x_; ++i) {
        double wx, wy;
        master->mapToWorld(i, j, wx, wy);
        const int mi = static_cast<int>(std::floor((wx - ox) / res));
        const int mj = static_cast<int>(std::floor((wy - oy) / res));
        const size_t idx = static_cast<size_t>(j) * size_x_ + i;
        if (mi < 0 || mj < 0 ||
          mi >= static_cast<int>(mx) || mj >= static_cast<int>(my))
        {
          static_lethal_[idx] = 0;
          continue;
        }
        const size_t midx = static_cast<size_t>(mj) * mx + static_cast<size_t>(mi);
        const int8_t v = msg->data[midx];
        static_lethal_[idx] = (v >= occupied_thresh_) ? 1 : 0;
      }
    }
  }
}

bool CustomDynamicLayer::isStaticLethal(unsigned int index) const
{
  if (index >= static_lethal_.size()) {
    return false;
  }
  return static_lethal_[index] != 0;
}

bool CustomDynamicLayer::worldToMapChecked(
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  auto * master = layered_costmap_->getCostmap();
  return master->worldToMap(wx, wy, mx, my);
}

void CustomDynamicLayer::touch(double wx, double wy)
{
  if (!has_touch_) {
    min_x_ = max_x_ = wx;
    min_y_ = max_y_ = wy;
    has_touch_ = true;
    return;
  }
  min_x_ = std::min(min_x_, wx);
  min_y_ = std::min(min_y_, wy);
  max_x_ = std::max(max_x_, wx);
  max_y_ = std::max(max_y_, wy);
}

void CustomDynamicLayer::markObstacle(unsigned int mx, unsigned int my)
{
  const unsigned int index = my * size_x_ + mx;
  if (index >= state_.size()) {
    return;
  }

  clear_hits_[index] = 0;
  if (obstacle_hits_[index] < 255) {
    obstacle_hits_[index]++;
  }

  if (obstacle_hits_[index] >= static_cast<uint8_t>(obstacle_confirmation_count_)) {
    state_[index] = CellState::DYNAMIC_OBSTACLE;
  }

  double wx, wy;
  layered_costmap_->getCostmap()->mapToWorld(mx, my, wx, wy);
  touch(wx, wy);
}

void CustomDynamicLayer::markFreeObservable(unsigned int mx, unsigned int my)
{
  const unsigned int index = my * size_x_ + mx;
  if (index >= state_.size()) {
    return;
  }

  // Only accumulate free evidence on cells we can observe through a clear ray.
  obstacle_hits_[index] = 0;
  if (clear_hits_[index] < 255) {
    clear_hits_[index]++;
  }

  if (clear_hits_[index] < static_cast<uint8_t>(clear_confirmation_count_)) {
    double wx, wy;
    layered_costmap_->getCostmap()->mapToWorld(mx, my, wx, wy);
    touch(wx, wy);
    return;
  }

  const bool static_occ = isStaticLethal(index);
  const CellState prev = state_[index];

  if (static_occ) {
    // Mapped wall verified gone — runtime override only.
    state_[index] = CellState::VERIFIED_FREE;
  } else if (prev == CellState::DYNAMIC_OBSTACLE) {
    // Temporary obstacle removed; static was free → stop overriding.
    state_[index] = CellState::NO_OVERRIDE;
  }
  // NO_OVERRIDE + static free: stay NO_OVERRIDE

  double wx, wy;
  layered_costmap_->getCostmap()->mapToWorld(mx, my, wx, wy);
  touch(wx, wy);
}

void CustomDynamicLayer::raytraceFree(
  unsigned int x0, unsigned int y0,
  unsigned int x1, unsigned int y1)
{
  // Bresenham from (x0,y0) toward (x1,y1), marking free on cells BEFORE the end.
  int dx = static_cast<int>(x1) - static_cast<int>(x0);
  int dy = static_cast<int>(y1) - static_cast<int>(y0);
  const int abs_dx = std::abs(dx);
  const int abs_dy = std::abs(dy);
  const int sx = dx > 0 ? 1 : -1;
  const int sy = dy > 0 ? 1 : -1;

  int err = abs_dx - abs_dy;
  int x = static_cast<int>(x0);
  int y = static_cast<int>(y0);

  const int steps = std::max(abs_dx, abs_dy);
  for (int i = 0; i < steps; ++i) {
    // Do not mark the endpoint as free — that is the obstacle cell (or max range).
    if (!(static_cast<unsigned int>(x) == x1 && static_cast<unsigned int>(y) == y1)) {
      if (x >= 0 && y >= 0 &&
        static_cast<unsigned int>(x) < size_x_ &&
        static_cast<unsigned int>(y) < size_y_)
      {
        markFreeObservable(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
      }
    }

    const int e2 = 2 * err;
    if (e2 > -abs_dy) {
      err -= abs_dy;
      x += sx;
    }
    if (e2 < abs_dx) {
      err += abs_dx;
      y += sy;
    }
  }
}

void CustomDynamicLayer::processScan(const sensor_msgs::msg::LaserScan & scan)
{
  geometry_msgs::msg::TransformStamped laser_to_global;
  try {
    laser_to_global = tf_->lookupTransform(
      global_frame_, scan.header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "[%s] TF %s -> %s failed: %s",
      name_.c_str(), scan.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    current_ = false;
    return;
  }

  const double lx = laser_to_global.transform.translation.x;
  const double ly = laser_to_global.transform.translation.y;
  const double yaw = tf2::getYaw(laser_to_global.transform.rotation);

  unsigned int ox, oy;
  if (!worldToMapChecked(lx, ly, ox, oy)) {
    current_ = false;
    return;
  }

  touch(lx, ly);
  current_ = true;

  const double angle_min = scan.angle_min;
  const double angle_inc = scan.angle_increment;
  const size_t n = scan.ranges.size();

  for (size_t i = 0; i < n; ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range)) {
      // No return: not evidence of free — optionally clear along max raytrace
      // only when range is +inf (clear to max). Nav2 treats inf as free to max.
      if (std::isinf(range) && range > 0.0f) {
        const double a = yaw + angle_min + static_cast<double>(i) * angle_inc;
        const double clear_r = std::min(raytrace_max_range_, static_cast<double>(scan.range_max));
        const double ex = lx + clear_r * std::cos(a);
        const double ey = ly + clear_r * std::sin(a);
        unsigned int mx, my;
        if (worldToMapChecked(ex, ey, mx, my)) {
          raytraceFree(ox, oy, mx, my);
          // Endpoint at max range is free observable (no hit).
          markFreeObservable(mx, my);
        } else {
          // Clip to map edge along the ray in coarse steps.
          for (double r = 0.05; r <= clear_r; r += layered_costmap_->getCostmap()->getResolution()) {
            const double wx = lx + r * std::cos(a);
            const double wy = ly + r * std::sin(a);
            unsigned int cx, cy;
            if (worldToMapChecked(wx, wy, cx, cy)) {
              markFreeObservable(cx, cy);
            } else {
              break;
            }
          }
        }
      }
      continue;
    }

    if (range < scan.range_min) {
      continue;
    }

    const double a = yaw + angle_min + static_cast<double>(i) * angle_inc;
    const bool hit = range <= scan.range_max &&
      range >= obstacle_min_range_ &&
      range <= obstacle_max_range_;

    // Clear along the ray up to the hit (or raytrace_max_range), not beyond.
    const double clear_r = std::min(
      static_cast<double>(range),
      raytrace_max_range_);
    if (clear_r > obstacle_min_range_) {
      // Endpoint for free ray: just before the obstacle / at clear_r.
      const double free_r = hit ?
        std::max(0.0, clear_r - layered_costmap_->getCostmap()->getResolution() * 0.5) :
        clear_r;
      const double fx = lx + free_r * std::cos(a);
      const double fy = ly + free_r * std::sin(a);
      unsigned int fx_m, fy_m;
      if (worldToMapChecked(fx, fy, fx_m, fy_m)) {
        raytraceFree(ox, oy, fx_m, fy_m);
        markFreeObservable(fx_m, fy_m);
      }
    }

    if (hit) {
      const double ex = lx + static_cast<double>(range) * std::cos(a);
      const double ey = ly + static_cast<double>(range) * std::sin(a);
      unsigned int mx, my;
      if (worldToMapChecked(ex, ey, mx, my)) {
        markObstacle(mx, my);
      }
    }
  }
}

void CustomDynamicLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  matchSize();

  sensor_msgs::msg::LaserScan::SharedPtr scan;
  bool reprocess = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (need_reprocess_ && latest_scan_) {
      scan = latest_scan_;
      need_reprocess_ = false;
      reprocess = true;
    }
  }

  if (reprocess && scan) {
    has_touch_ = false;
    std::lock_guard<std::mutex> lock(data_mutex_);
    processScan(*scan);
  }

  if (has_touch_) {
    *min_x = std::min(*min_x, min_x_);
    *min_y = std::min(*min_y, min_y_);
    *max_x = std::max(*max_x, max_x_);
    *max_y = std::max(*max_y, max_y_);
  }
}

void CustomDynamicLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  if (state_.empty() || size_x_ == 0) {
    return;
  }

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x_), max_i);
  max_j = std::min(static_cast<int>(size_y_), max_j);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      const unsigned int index =
        static_cast<unsigned int>(j) * size_x_ + static_cast<unsigned int>(i);
      if (index >= state_.size()) {
        continue;
      }
      switch (state_[index]) {
        case CellState::NO_OVERRIDE:
          // Leave master as Static Layer wrote it.
          break;
        case CellState::DYNAMIC_OBSTACLE:
          master_grid.setCost(i, j, LETHAL_OBSTACLE);
          break;
        case CellState::VERIFIED_FREE:
          master_grid.setCost(i, j, FREE_SPACE);
          break;
      }
    }
  }
}

}  // namespace skill23_custom_costmap

PLUGINLIB_EXPORT_CLASS(skill23_custom_costmap::CustomDynamicLayer, nav2_costmap_2d::Layer)
