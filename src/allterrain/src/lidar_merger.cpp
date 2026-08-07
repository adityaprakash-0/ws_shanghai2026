// #include <chrono>
// #include <cmath>
// #include <memory>
// #include <vector>
// #include <limits>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "message_filters/subscriber.h"
// #include "message_filters/sync_policies/approximate_time.h"
// #include "message_filters/synchronizer.h"

// using namespace std::chrono_literals;

// struct Extrusion {
//     double min_deg;
//     double max_deg;
// };

// class LidarMergerNode : public rclcpp::Node {
// public:
//     LidarMergerNode() : Node("lidar_merger_node") {
//         // Physical Offsets
//         front_offset_x_ = 0.195;
//         back_offset_x_  = -0.195;

//         // Extrusions (Blind Spots)
//         // front_extrusions_ = {{110.0, 130.0}, {145.0, 160.0}, {175.0, 193.0}, {200.0, 225.0}, {230.0, 260.0}};
//         // back_extrusions_  = {{110.0, 130.0}, {145.0, 165.0}, {175.0, 190.0}, {200.0, 225.0}, {225.0, 255.0}};

//         // front_extrusions_ = {{110.0, 130.0}, {145.0, 160.0}, {175.0, 193.0}, {200.0, 225.0}, {230.0, 260.0}};
//            front_extrusions_ = {};
//            back_extrusions_  = {};
       

//         auto sensor_qos = rclcpp::SensorDataQoS();

//         pub_merged_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", sensor_qos);
//         pub_front_filtered_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan/front_filtered", sensor_qos);
//         pub_back_filtered_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan/back_filtered", sensor_qos);

//         sub_front_.subscribe(this, "/scan/front", sensor_qos.get_rmw_qos_profile());
//         sub_back_.subscribe(this, "/scan/back", sensor_qos.get_rmw_qos_profile());

//         sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
//             SyncPolicy(10), sub_front_, sub_back_);
//         sync_->registerCallback(std::bind(&LidarMergerNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

//         RCLCPP_INFO(this->get_logger(), "C++ Dual LiDAR Merger Active (720-Bin High Resolution).");
//     }

// private:
//     double front_offset_x_, back_offset_x_;
//     std::vector<Extrusion> front_extrusions_, back_extrusions_;

//     rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_merged_;
//     rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_front_filtered_;
//     rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_back_filtered_;

//     message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_front_;
//     message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_back_;

//     typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> SyncPolicy;
//     std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

//     sensor_msgs::msg::LaserScan apply_extrusion_filter(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg, const std::vector<Extrusion>& extrusions) {
//         sensor_msgs::msg::LaserScan filtered = *scan_msg;
        
//         for (size_t i = 0; i < filtered.ranges.size(); ++i) {
//             double angle_rad = filtered.angle_min + i * filtered.angle_increment;
//             double angle_deg = std::fmod(angle_rad * (180.0 / M_PI), 360.0);
//             if (angle_deg < 0) angle_deg += 360.0;

//             bool blocked = false;
//             for (const auto& ext : extrusions) {
//                 if (ext.min_deg < ext.max_deg) {
//                     if (angle_deg >= ext.min_deg && angle_deg <= ext.max_deg) blocked = true;
//                 } else {
//                     if (angle_deg >= ext.min_deg || angle_deg <= ext.max_deg) blocked = true;
//                 }
//             }

//             if (blocked) {
//                 filtered.ranges[i] = std::numeric_limits<float>::infinity();
//             }
//         }
//         return filtered;
//     }

//     void sync_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& front_msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr& back_msg) {
        
//         // 1. Apply chassis filters
//         sensor_msgs::msg::LaserScan front_filtered = apply_extrusion_filter(front_msg, front_extrusions_);
//         sensor_msgs::msg::LaserScan back_filtered = apply_extrusion_filter(back_msg, back_extrusions_);

//         pub_front_filtered_->publish(front_filtered);
//         pub_back_filtered_->publish(back_filtered);

//         // 2. Setup High-Res Merged Scan (720 bins = 0.5 degrees per laser point)
//         int num_bins = 720;
//         sensor_msgs::msg::LaserScan merged_msg;
//         // merged_msg.header.stamp = this->now(); // Fresh stamp to prevent TF timeouts
//         // DELETE THIS LINE:
// // merged_msg.header.stamp = this->now(); 

// // REPLACE WITH THIS LINE:
//         merged_msg.header.stamp = front_msg->header.stamp;
//         merged_msg.header.frame_id = "base_footprint";
//         merged_msg.angle_min = -M_PI;
//         merged_msg.angle_max = M_PI;
//         merged_msg.angle_increment = (2.0 * M_PI) / num_bins;
//         merged_msg.time_increment = front_msg->time_increment;
//         merged_msg.scan_time = front_msg->scan_time;
//         merged_msg.range_min = 0.1;
//         merged_msg.range_max = front_msg->range_max;

//         std::vector<float> merged_ranges(num_bins, std::numeric_limits<float>::infinity());

//         // Lambda function to map points to the shared base_link grid
//         auto process_to_grid = [&](const sensor_msgs::msg::LaserScan& scan, double offset_x, bool is_back) {
//             for (size_t i = 0; i < scan.ranges.size(); ++i) {
//                 float r = scan.ranges[i];
//                 if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;

//                 double angle = scan.angle_min + i * scan.angle_increment;
//                 if (is_back) angle += M_PI; // Mathematically flip back lidar

//                 // Convert to Cartesian, apply physical chassis offset
//                 double x = r * std::cos(angle) + offset_x;
//                 double y = r * std::sin(angle);

//                 // Convert back to Polar from the exact center of base_link
//                 double new_r = std::sqrt(x * x + y * y);
//                 double new_theta = std::atan2(y, x);

//                 // Slot into the new high-res array
//                 int bin = std::round((new_theta - merged_msg.angle_min) / merged_msg.angle_increment);
//                 if (bin >= 0 && bin < num_bins) {
//                     if (new_r < merged_ranges[bin]) {
//                         merged_ranges[bin] = new_r; // Keep closest wall
//                     }
//                 }
//             }
//         };

//         process_to_grid(front_filtered, front_offset_x_, false);
//         process_to_grid(back_filtered, back_offset_x_, true);

//         merged_msg.ranges = merged_ranges;
//         pub_merged_->publish(merged_msg);
//     }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<LidarMergerNode>());
//     rclcpp::shutdown();
//     return 0;
// }

//wo4king perfect just one line update in next

// #include <chrono>
// #include <cmath>
// #include <memory>
// #include <vector>
// #include <limits>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "message_filters/subscriber.h"
// #include "message_filters/sync_policies/approximate_time.h"
// #include "message_filters/synchronizer.h"

// using namespace std::chrono_literals;

// class LidarMergerNode : public rclcpp::Node {
// public:
//     LidarMergerNode() : Node("lidar_merger_node") {
//         // Physical Offsets
//         front_offset_x_ = 0.195;
//         back_offset_x_  = -0.195;

//         // Robot Footprint Dimensions (Meters)
//         footprint_length_ = 0.45; // 43 cm long (X-axis)
//         footprint_width_  = 0.45; // 40 cm wide (Y-axis)

//         auto sensor_qos = rclcpp::SensorDataQoS();

//         pub_merged_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", sensor_qos);

//         sub_front_.subscribe(this, "/scan/front", sensor_qos.get_rmw_qos_profile());
//         sub_back_.subscribe(this, "/scan/back", sensor_qos.get_rmw_qos_profile());

//         sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
//             SyncPolicy(10), sub_front_, sub_back_);
//         sync_->registerCallback(std::bind(&LidarMergerNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

//         RCLCPP_INFO(this->get_logger(), "Lightweight Dual LiDAR Merger Active (Footprint Filter Enabled).");
//     }

// private:
//     double front_offset_x_, back_offset_x_;
//     double footprint_length_, footprint_width_;

//     rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_merged_;

//     message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_front_;
//     message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_back_;

//     typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> SyncPolicy;
//     std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

//     void sync_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& front_msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr& back_msg) {
        
//         // Setup High-Res Merged Scan (720 bins = 0.5 degrees per laser point)
//         int num_bins = 720;
//         sensor_msgs::msg::LaserScan merged_msg;
//         merged_msg.header.stamp = front_msg->header.stamp; // Keep synchronized stamp
//         merged_msg.header.frame_id = "base_footprint";
//         merged_msg.angle_min = -M_PI;
//         merged_msg.angle_max = M_PI;
//         merged_msg.angle_increment = (2.0 * M_PI) / num_bins;
//         merged_msg.time_increment = front_msg->time_increment;
//         merged_msg.scan_time = front_msg->scan_time;
//         merged_msg.range_min = 0.1;
//         merged_msg.range_max = std::max(front_msg->range_max, back_msg->range_max);

//         std::vector<float> merged_ranges(num_bins, std::numeric_limits<float>::infinity());

//         // Lambda function to map points to the shared base_link grid
//         auto process_to_grid = [&](const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan, double offset_x, bool is_back) {
//             for (size_t i = 0; i < scan->ranges.size(); ++i) {
//                 float r = scan->ranges[i];
//                 if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;

//                 double angle = scan->angle_min + i * scan->angle_increment;
//                 if (is_back) angle += M_PI; // Mathematically flip back lidar

//                 // Convert to Cartesian relative to center of robot (base_footprint)
//                 double x = r * std::cos(angle) + offset_x;
//                 double y = r * std::sin(angle);

//                 // --- THE FOOTPRINT FILTER ---
//                 // If the point lands inside the 43x40cm box, it hit the robot body! 
//                 // We add a tiny 1.5cm safety buffer (0.015) so we don't catch the exact edge of the metal.
//                 if (std::abs(x) <= (footprint_length_ / 2.0) + 0.015 && 
//                     std::abs(y) <= (footprint_width_ / 2.0) + 0.015) {
//                     continue; // Skip this point entirely
//                 }

//                 // Convert back to Polar from the exact center of base_footprint
//                 double new_r = std::sqrt(x * x + y * y);
//                 double new_theta = std::atan2(y, x);

//                 // Slot into the new high-res array
//                 int bin = std::round((new_theta - merged_msg.angle_min) / merged_msg.angle_increment);
                
//                 // Ensure wrapping correctly (e.g. 720 becomes 0)
//                 if (bin == num_bins) bin = 0;

//                 if (bin >= 0 && bin < num_bins) {
//                     if (new_r < merged_ranges[bin]) {
//                         merged_ranges[bin] = new_r; // Keep closest wall
//                     }
//                 }
//             }
//         };

//         // Process both raw scans directly into the final merged array
//         process_to_grid(front_msg, front_offset_x_, false);
//         process_to_grid(back_msg, back_offset_x_, true);

//         merged_msg.ranges = merged_ranges;
//         pub_merged_->publish(merged_msg);
//     }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<LidarMergerNode>());
//     rclcpp::shutdown();
//     return 0;
// }

// ============================================================================
// PREVIOUS IMPLEMENTATION (kept intact for reference; intentionally disabled)
// ============================================================================
// #if 0
// #include <chrono>
// #include <cmath>
// #include <memory>
// #include <vector>
// #include <limits>
// #include <algorithm>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "message_filters/subscriber.h"
// #include "message_filters/sync_policies/approximate_time.h"
// #include "message_filters/synchronizer.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"

// using namespace std::chrono_literals;

// class LidarMergerNode : public rclcpp::Node {
// public:
//     LidarMergerNode() : Node("lidar_merger_node") {
//         // Robot footprint ≈ URDF collision with FOOTPRINT_MARGIN buffer
//         footprint_length_ = 0.44;
//         footprint_width_  = 0.40;

//         merged_ranges_.assign(NUM_BINS, std::numeric_limits<float>::infinity());
//         merged_intensities_.assign(NUM_BINS, 0.0F);

//         // Read the sensor poses from robot_state_publisher instead of duplicating
//         // the URDF offsets and rear-lidar rotation in this node.
//         tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         auto sensor_qos = rclcpp::SensorDataQoS();

//         pub_merged_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", sensor_qos);

//         sub_front_.subscribe(this, "/scan/front", sensor_qos.get_rmw_qos_profile());
//         sub_back_.subscribe(this, "/scan/back", sensor_qos.get_rmw_qos_profile());

//         SyncPolicy sync_policy(10);
//         // At 6 Hz, independently rotating lidars can naturally differ by up to
//         // about half a scan period. Never pair across a dropped/old scan.
//         sync_policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(MAX_SYNC_SKEW_SEC));
//         sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
//             static_cast<const SyncPolicy&>(sync_policy));
//         sync_->connectInput(sub_front_, sub_back_);
//         sync_->registerCallback(std::bind(&LidarMergerNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

//         RCLCPP_INFO(
//             this->get_logger(),
//             "Dual LiDAR Merger Active (motion compensation, 0.25 deg bins, intensity + footprint filter).");
//     }

// private:
//     static constexpr int NUM_BINS = 1440;                // 0.25° resolution
//     static constexpr double FOOTPRINT_MARGIN = 0.02;     // m, beyond chassis box
//     static constexpr double MAX_SYNC_SKEW_SEC = 0.10;    // reject stale scan pairs
//     static constexpr double TF_TIMEOUT_SEC = 0.20;

//     double footprint_length_, footprint_width_;
//     std::vector<float> merged_ranges_;
//     std::vector<float> merged_intensities_;

//     rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_merged_;

//     message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_front_;
//     message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_back_;
//     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//     typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> SyncPolicy;
//     std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

//     void sync_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& front_msg,
//                        const sensor_msgs::msg::LaserScan::ConstSharedPtr& back_msg) {
//         const rclcpp::Time front_stamp(front_msg->header.stamp);
//         const rclcpp::Time back_stamp(back_msg->header.stamp);
//         const rclcpp::Time target_stamp =
//             front_stamp >= back_stamp ? front_stamp : back_stamp;
//         const double scan_skew = std::abs((front_stamp - back_stamp).seconds());
//         if (scan_skew > MAX_SYNC_SKEW_SEC) {
//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(), *this->get_clock(), 2000,
//                 "Dropping LiDAR pair with %.1f ms timestamp skew",
//                 scan_skew * 1000.0);
//             return;
//         }

//         geometry_msgs::msg::TransformStamped front_tf;
//         geometry_msgs::msg::TransformStamped back_tf;
//         try {
//             // Transform each scan from its own acquisition pose into the robot
//             // pose at the newer scan's timestamp. The odom frame is locally
//             // continuous, so this removes inter-LiDAR motion displacement.
//             front_tf = tf_buffer_->lookupTransform(
//                 "base_footprint", target_stamp,
//                 front_msg->header.frame_id, front_stamp,
//                 "odom", rclcpp::Duration::from_seconds(TF_TIMEOUT_SEC));
//             back_tf = tf_buffer_->lookupTransform(
//                 "base_footprint", target_stamp,
//                 back_msg->header.frame_id, back_stamp,
//                 "odom", rclcpp::Duration::from_seconds(TF_TIMEOUT_SEC));
//         } catch (const tf2::TransformException& ex) {
//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(), *this->get_clock(), 2000,
//                 "Waiting for time-aligned LiDAR transforms: %s", ex.what());
//             return;
//         }

//         sensor_msgs::msg::LaserScan merged_msg;
//         merged_msg.header.stamp =
//             front_stamp >= back_stamp ? front_msg->header.stamp : back_msg->header.stamp;
//         merged_msg.header.frame_id = "base_footprint";
//         merged_msg.angle_min = -M_PI;
//         merged_msg.angle_increment = (2.0 * M_PI) / static_cast<double>(NUM_BINS);
//         // LaserScan angle_max is inclusive; make its metadata agree with 1440 ranges.
//         merged_msg.angle_max =
//             merged_msg.angle_min + (NUM_BINS - 1) * merged_msg.angle_increment;
//         // Output bins combine two scans and are no longer in acquisition-time order.
//         merged_msg.time_increment = 0.0;
//         merged_msg.scan_time = std::max(front_msg->scan_time, back_msg->scan_time);
//         merged_msg.range_min = std::min(front_msg->range_min, back_msg->range_min);
//         merged_msg.range_max = std::max(front_msg->range_max, back_msg->range_max);

//         std::fill(merged_ranges_.begin(), merged_ranges_.end(),
//                   std::numeric_limits<float>::infinity());
//         std::fill(merged_intensities_.begin(), merged_intensities_.end(), 0.0F);

//         auto process_to_grid = [&](const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
//                                    const geometry_msgs::msg::TransformStamped& sensor_tf) {
//             const auto& q = sensor_tf.transform.rotation;
//             const double yaw = std::atan2(
//                 2.0 * (q.w * q.z + q.x * q.y),
//                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
//             const double cos_yaw = std::cos(yaw);
//             const double sin_yaw = std::sin(yaw);
//             const double offset_x = sensor_tf.transform.translation.x;
//             const double offset_y = sensor_tf.transform.translation.y;

//             for (size_t i = 0; i < scan->ranges.size(); ++i) {
//                 float r = scan->ranges[i];
//                 if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;

//                 double angle = scan->angle_min + i * scan->angle_increment;
//                 const double sensor_x = r * std::cos(angle);
//                 const double sensor_y = r * std::sin(angle);
//                 const double x = offset_x + cos_yaw * sensor_x - sin_yaw * sensor_y;
//                 const double y = offset_y + sin_yaw * sensor_x + cos_yaw * sensor_y;

//                 if (std::abs(x) <= (footprint_length_ / 2.0) + FOOTPRINT_MARGIN &&
//                     std::abs(y) <= (footprint_width_ / 2.0) + FOOTPRINT_MARGIN) {
//                     continue;
//                 }

//                 double new_r = std::hypot(x, y);
//                 double new_theta = std::atan2(y, x);

//                 int bin = static_cast<int>(std::round(
//                     (new_theta - merged_msg.angle_min) / merged_msg.angle_increment));
//                 if (bin >= NUM_BINS) bin = 0;
//                 if (bin < 0) bin = NUM_BINS - 1;

//                 if (new_r < merged_ranges_[bin]) {
//                     merged_ranges_[bin] = static_cast<float>(new_r);
//                     merged_intensities_[bin] =
//                         i < scan->intensities.size() ? scan->intensities[i] : 0.0F;
//                 }
//             }
//         };

//         process_to_grid(front_msg, front_tf);
//         process_to_grid(back_msg, back_tf);

//         merged_msg.ranges = merged_ranges_;
//         merged_msg.intensities = merged_intensities_;
//         pub_merged_->publish(merged_msg);
//     }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<LidarMergerNode>());
//     rclcpp::shutdown();
//     return 0;
// }
// #endif

// ============================================================================
// SYNCHRONIZED, TF-AWARE, MOTION-COMPENSATED DUAL-LIDAR MERGER
//
// Each lidar contributes only its forward-facing sector (default +/-130 deg).
// TF supplies each physical mounting pose, including the rear lidar's pi yaw.
// Points are transformed into base_footprint at one common output timestamp.
// Temporal buckets compensate robot motion during and between rotating scans
// without doing a costly TF lookup for every individual beam.
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class SynchronizedLidarMerger : public rclcpp::Node
{
public:
    using LaserScan = sensor_msgs::msg::LaserScan;
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<LaserScan, LaserScan>;

    SynchronizedLidarMerger()
    : Node("lidar_merger_node")
    {
        target_frame_ = declare_parameter<std::string>(
            "target_frame", "base_footprint");
        fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
        front_half_fov_rad_ = degrees_to_radians(
            declare_parameter<double>("front_half_fov_deg", 130.0));
        back_half_fov_rad_ = degrees_to_radians(
            declare_parameter<double>("back_half_fov_deg", 130.0));
        max_sync_skew_sec_ =
            declare_parameter<double>("max_sync_skew_sec", 0.10);
        tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.15);
        // 720 bins (0.5 deg) matches the useful angular density at 6 Hz.
        // Higher bin counts create artificial gaps rather than more detail.
        output_bins_ = declare_parameter<int>("output_bins", 720);
        motion_buckets_ = declare_parameter<int>("motion_buckets", 24);
        footprint_length_ =
            declare_parameter<double>("footprint_length", 0.44);
        footprint_width_ =
            declare_parameter<double>("footprint_width", 0.40);
        footprint_margin_ =
            declare_parameter<double>("footprint_margin", 0.01);

        front_half_fov_rad_ =
            std::clamp(front_half_fov_rad_, 0.0, M_PI);
        back_half_fov_rad_ =
            std::clamp(back_half_fov_rad_, 0.0, M_PI);
        output_bins_ = std::max(output_bins_, 360);
        motion_buckets_ = std::clamp(motion_buckets_, 1, 64);
        max_sync_skew_sec_ = std::max(max_sync_skew_sec_, 0.0);

        merged_ranges_.resize(
            static_cast<std::size_t>(output_bins_),
            std::numeric_limits<float>::infinity());
        merged_intensities_.resize(
            static_cast<std::size_t>(output_bins_), 0.0F);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        const auto sensor_qos = rclcpp::SensorDataQoS();
        publisher_ = create_publisher<LaserScan>("/scan", sensor_qos);
        front_sub_.subscribe(
            this, "/scan/front", sensor_qos.get_rmw_qos_profile());
        back_sub_.subscribe(
            this, "/scan/back", sensor_qos.get_rmw_qos_profile());

        SyncPolicy policy(20);
        policy.setMaxIntervalDuration(
            rclcpp::Duration::from_seconds(max_sync_skew_sec_));
        synchronizer_ =
            std::make_shared<message_filters::Synchronizer<SyncPolicy>>(policy);
        synchronizer_->connectInput(front_sub_, back_sub_);
        synchronizer_->registerCallback(
            std::bind(
                &SynchronizedLidarMerger::merge_callback, this,
                std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(
            get_logger(),
            "Dual LiDAR merger: front/back +/-%.1f/%.1f deg, skew <= %.0f ms, "
            "%d bins, %d motion buckets, TF %s -> %s",
            radians_to_degrees(front_half_fov_rad_),
            radians_to_degrees(back_half_fov_rad_),
            max_sync_skew_sec_ * 1000.0, output_bins_, motion_buckets_,
            fixed_frame_.c_str(), target_frame_.c_str());
    }

private:
    struct PlanarTransform
    {
        double x{0.0};
        double y{0.0};
        double cos_yaw{1.0};
        double sin_yaw{0.0};
        bool valid{false};
    };

    static double degrees_to_radians(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    static double radians_to_degrees(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    static double normalize_angle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    static rclcpp::Duration beam_offset(
        const LaserScan & scan, std::size_t index)
    {
        double increment = static_cast<double>(scan.time_increment);
        if (increment <= 0.0 && scan.scan_time > 0.0F &&
            scan.ranges.size() > 1)
        {
            // Some drivers leave time_increment at zero. scan_time still gives
            // enough information for useful intra-scan motion compensation.
            increment =
                static_cast<double>(scan.scan_time) /
                static_cast<double>(scan.ranges.size() - 1);
        }
        if (increment <= 0.0) {
            return rclcpp::Duration(0, 0);
        }
        return rclcpp::Duration::from_seconds(
            increment * static_cast<double>(index));
    }

    static rclcpp::Time scan_end_stamp(const LaserScan & scan)
    {
        const rclcpp::Time start(scan.header.stamp);
        if (scan.ranges.empty()) {
            return start;
        }
        return start + beam_offset(scan, scan.ranges.size() - 1);
    }

    static rclcpp::Time scan_mid_stamp(const LaserScan & scan)
    {
        const rclcpp::Time start(scan.header.stamp);
        if (scan.ranges.empty()) {
            return start;
        }
        return start + beam_offset(scan, (scan.ranges.size() - 1) / 2);
    }

    PlanarTransform lookup_planar_transform(
        const std::string & source_frame,
        const rclcpp::Time & source_stamp,
        const rclcpp::Time & target_stamp)
    {
        PlanarTransform result;
        try {
            const auto transform = tf_buffer_->lookupTransform(
                target_frame_, target_stamp,
                source_frame, source_stamp,
                fixed_frame_,
                rclcpp::Duration::from_seconds(tf_timeout_sec_));

            const auto & q = transform.transform.rotation;
            const double yaw = std::atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

            result.x = transform.transform.translation.x;
            result.y = transform.transform.translation.y;
            result.cos_yaw = std::cos(yaw);
            result.sin_yaw = std::sin(yaw);
            result.valid = true;
        } catch (const tf2::TransformException & exception) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for motion-compensation TF (%s at beam time -> %s): %s",
                source_frame.c_str(), target_frame_.c_str(), exception.what());
        }
        return result;
    }

    std::vector<PlanarTransform> build_motion_transforms(
        const LaserScan & scan, const rclcpp::Time & target_stamp)
    {
        std::vector<PlanarTransform> transforms(
            static_cast<std::size_t>(motion_buckets_));
        if (scan.ranges.empty() || scan.header.frame_id.empty()) {
            return transforms;
        }

        const std::size_t count = scan.ranges.size();
        for (int bucket = 0; bucket < motion_buckets_; ++bucket) {
            const double fraction =
                (static_cast<double>(bucket) + 0.5) /
                static_cast<double>(motion_buckets_);
            const std::size_t index = std::min(
                static_cast<std::size_t>(
                    fraction * static_cast<double>(count)),
                count - 1);
            const rclcpp::Time beam_stamp(scan.header.stamp);
            transforms[static_cast<std::size_t>(bucket)] =
                lookup_planar_transform(
                    scan.header.frame_id,
                    beam_stamp + beam_offset(scan, index),
                    target_stamp);
        }
        return transforms;
    }

    void add_scan(
        const LaserScan::ConstSharedPtr & scan,
        double half_fov_rad,
        const std::vector<PlanarTransform> & transforms,
        const LaserScan & output_template)
    {
        const std::size_t count = scan->ranges.size();
        if (count == 0) {
            return;
        }

        for (std::size_t i = 0; i < count; ++i) {
            const double sensor_angle =
                static_cast<double>(scan->angle_min) +
                static_cast<double>(i) *
                static_cast<double>(scan->angle_increment);

            // Keep only the forward-facing sector in each lidar's own frame.
            if (std::abs(normalize_angle(sensor_angle)) > half_fov_rad) {
                continue;
            }

            const float range = scan->ranges[i];
            if (!std::isfinite(range) ||
                range < scan->range_min || range > scan->range_max)
            {
                continue;
            }

            const std::size_t bucket = std::min(
                i * static_cast<std::size_t>(motion_buckets_) / count,
                static_cast<std::size_t>(motion_buckets_ - 1));
            const auto & transform = transforms[bucket];
            if (!transform.valid) {
                continue;
            }

            const double sensor_x =
                static_cast<double>(range) * std::cos(sensor_angle);
            const double sensor_y =
                static_cast<double>(range) * std::sin(sensor_angle);
            const double base_x =
                transform.x + transform.cos_yaw * sensor_x -
                transform.sin_yaw * sensor_y;
            const double base_y =
                transform.y + transform.sin_yaw * sensor_x +
                transform.cos_yaw * sensor_y;

            // Remove returns from the chassis and mounting structure.
            if (std::abs(base_x) <= footprint_length_ / 2.0 + footprint_margin_ &&
                std::abs(base_y) <= footprint_width_ / 2.0 + footprint_margin_)
            {
                continue;
            }

            const double merged_range = std::hypot(base_x, base_y);
            const double merged_angle = std::atan2(base_y, base_x);
            int bin = static_cast<int>(std::llround(
                (merged_angle - output_template.angle_min) /
                output_template.angle_increment));

            // +pi and -pi represent the same output direction.
            bin %= output_bins_;
            if (bin < 0) {
                bin += output_bins_;
            }

            const std::size_t output_index = static_cast<std::size_t>(bin);
            if (merged_range < merged_ranges_[output_index]) {
                merged_ranges_[output_index] =
                    static_cast<float>(merged_range);
                merged_intensities_[output_index] =
                    i < scan->intensities.size() ?
                    scan->intensities[i] : 0.0F;
            }
        }
    }

    void merge_callback(
        const LaserScan::ConstSharedPtr & front,
        const LaserScan::ConstSharedPtr & back)
    {
        if (front->header.frame_id.empty() || back->header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping scans with empty frame_id");
            return;
        }

        // Compare scan mid-times, not only packet/header arrival times.
        const rclcpp::Time front_mid = scan_mid_stamp(*front);
        const rclcpp::Time back_mid = scan_mid_stamp(*back);
        const double midpoint_skew =
            std::abs((front_mid - back_mid).seconds());
        if (midpoint_skew > max_sync_skew_sec_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping LiDAR pair: midpoint skew %.1f ms exceeds %.1f ms",
                midpoint_skew * 1000.0, max_sync_skew_sec_ * 1000.0);
            return;
        }

        // Synthetic output represents every point at this common robot pose.
        const rclcpp::Time front_end = scan_end_stamp(*front);
        const rclcpp::Time back_end = scan_end_stamp(*back);
        const rclcpp::Time target_stamp =
            front_end >= back_end ? front_end : back_end;

        const auto front_transforms =
            build_motion_transforms(*front, target_stamp);
        const auto back_transforms =
            build_motion_transforms(*back, target_stamp);

        LaserScan merged;
        merged.header.stamp = target_stamp;
        merged.header.frame_id = target_frame_;
        merged.angle_min = static_cast<float>(-M_PI);
        merged.angle_increment = static_cast<float>(
            2.0 * M_PI / static_cast<double>(output_bins_));
        merged.angle_max =
            merged.angle_min +
            static_cast<float>(output_bins_ - 1) * merged.angle_increment;
        merged.time_increment = 0.0F;
        merged.scan_time = std::max(front->scan_time, back->scan_time);
        merged.range_min = std::min(front->range_min, back->range_min);
        // Base origin is offset from each lidar, so its valid polar range can
        // be slightly greater than the sensors' advertised maximum.
        merged.range_max =
            std::max(front->range_max, back->range_max) + 0.5F;

        std::fill(
            merged_ranges_.begin(), merged_ranges_.end(),
            std::numeric_limits<float>::infinity());
        std::fill(
            merged_intensities_.begin(), merged_intensities_.end(), 0.0F);

        add_scan(
            front, front_half_fov_rad_, front_transforms, merged);
        add_scan(
            back, back_half_fov_rad_, back_transforms, merged);

        merged.ranges = merged_ranges_;
        merged.intensities = merged_intensities_;
        publisher_->publish(merged);
    }

    std::string target_frame_;
    std::string fixed_frame_;
    double front_half_fov_rad_{0.0};
    double back_half_fov_rad_{0.0};
    double max_sync_skew_sec_{0.10};
    double tf_timeout_sec_{0.15};
    int output_bins_{720};
    int motion_buckets_{24};
    double footprint_length_{0.44};
    double footprint_width_{0.40};
    double footprint_margin_{0.02};

    std::vector<float> merged_ranges_;
    std::vector<float> merged_intensities_;
    rclcpp::Publisher<LaserScan>::SharedPtr publisher_;
    message_filters::Subscriber<LaserScan> front_sub_;
    message_filters::Subscriber<LaserScan> back_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SynchronizedLidarMerger>());
    rclcpp::shutdown();
    return 0;
}