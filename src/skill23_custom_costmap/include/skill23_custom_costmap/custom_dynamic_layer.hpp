#ifndef SKILL23_CUSTOM_COSTMAP__CUSTOM_DYNAMIC_LAYER_HPP_
#define SKILL23_CUSTOM_COSTMAP__CUSTOM_DYNAMIC_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace skill23_custom_costmap
{

enum class CellState : uint8_t
{
  NO_OVERRIDE = 0,
  DYNAMIC_OBSTACLE = 1,
  VERIFIED_FREE = 2,
};

/**
 * @brief Global costmap layer: dynamic obstacles + verified-free static overrides.
 *
 * Explicitly writes LETHAL / FREE into the master grid (not MAX combination).
 * NO_OVERRIDE leaves the static-populated master cell unchanged.
 */
class CustomDynamicLayer : public nav2_costmap_2d::Layer
{
public:
  CustomDynamicLayer() = default;
  ~CustomDynamicLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  void matchSize() override;
  bool isClearable() override {return true;}
  void activate() override;
  void deactivate() override;

private:
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void ensureStorage(unsigned int size_x, unsigned int size_y);
  void touch(double wx, double wy);
  void processScan(const sensor_msgs::msg::LaserScan & scan);
  void raytraceFree(
    unsigned int x0, unsigned int y0,
    unsigned int x1, unsigned int y1);
  void markObstacle(unsigned int mx, unsigned int my);
  void markFreeObservable(unsigned int mx, unsigned int my);
  bool worldToMapChecked(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  bool isStaticLethal(unsigned int index) const;

  std::string global_frame_;
  std::string scan_topic_;
  std::string map_topic_;
  double transform_tolerance_{1.0};
  double obstacle_max_range_{2.5};
  double obstacle_min_range_{0.12};
  double raytrace_max_range_{3.0};
  int obstacle_confirmation_count_{2};
  int clear_confirmation_count_{3};
  int occupied_thresh_{50};

  unsigned int size_x_{0};
  unsigned int size_y_{0};
  std::vector<CellState> state_;
  std::vector<uint8_t> obstacle_hits_;
  std::vector<uint8_t> clear_hits_;
  std::vector<uint8_t> static_lethal_;  // 1 = static map occupied

  bool need_reprocess_{false};
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  bool rolling_window_{false};
  bool has_touch_{false};
  double min_x_{0.0};
  double min_y_{0.0};
  double max_x_{0.0};
  double max_y_{0.0};

  std::mutex data_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

}  // namespace skill23_custom_costmap

#endif  // SKILL23_CUSTOM_COSTMAP__CUSTOM_DYNAMIC_LAYER_HPP_
