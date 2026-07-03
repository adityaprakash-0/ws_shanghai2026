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


#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

using namespace std::chrono_literals;

class LidarMergerNode : public rclcpp::Node {
public:
    LidarMergerNode() : Node("lidar_merger_node") {
        // Physical Offsets
        front_offset_x_ = 0.195;
        back_offset_x_  = -0.195;

        // Robot Footprint Dimensions (Meters)
        footprint_length_ = 0.45; // 43 cm long (X-axis) + buffer
        footprint_width_  = 0.45; // 40 cm wide (Y-axis) + buffer

        auto sensor_qos = rclcpp::SensorDataQoS();

        pub_merged_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", sensor_qos);

        sub_front_.subscribe(this, "/scan/front", sensor_qos.get_rmw_qos_profile());
        sub_back_.subscribe(this, "/scan/back", sensor_qos.get_rmw_qos_profile());

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_front_, sub_back_);
        sync_->registerCallback(std::bind(&LidarMergerNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Lightweight Dual LiDAR Merger Active (Footprint Filter Enabled).");
    }

private:
    double front_offset_x_, back_offset_x_;
    double footprint_length_, footprint_width_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_merged_;

    message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_front_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_back_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    void sync_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& front_msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr& back_msg) {
        
        // Setup High-Res Merged Scan (720 bins = 0.5 degrees per laser point)
        int num_bins = 720;
        sensor_msgs::msg::LaserScan merged_msg;
        merged_msg.header.stamp = front_msg->header.stamp; // Keep synchronized stamp
        merged_msg.header.frame_id = "base_footprint";
        merged_msg.angle_min = -M_PI;
        merged_msg.angle_max = M_PI;
        merged_msg.angle_increment = (2.0 * M_PI) / num_bins;
        merged_msg.time_increment = front_msg->time_increment;
        merged_msg.scan_time = front_msg->scan_time;
        merged_msg.range_min = 0.1;
        merged_msg.range_max = std::max(front_msg->range_max, back_msg->range_max);

        std::vector<float> merged_ranges(num_bins, std::numeric_limits<float>::infinity());

        // Lambda function to map points to the shared base_footprint grid
        auto process_to_grid = [&](const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan, double offset_x, bool is_back) {
            for (size_t i = 0; i < scan->ranges.size(); ++i) {
                float r = scan->ranges[i];
                if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;

                double angle = scan->angle_min + i * scan->angle_increment;
                if (is_back) angle += M_PI; // Mathematically flip back lidar

                // Convert to Cartesian relative to center of robot (base_footprint)
                double x = r * std::cos(angle) + offset_x;
                double y = r * std::sin(angle);

                // --- THE FOOTPRINT FILTER ---
                if (std::abs(x) <= (footprint_length_ / 2.0) + 0.015 && 
                    std::abs(y) <= (footprint_width_ / 2.0) + 0.015) {
                    continue; // Skip points hitting the robot body
                }

                // Convert back to Polar from the exact center of base_footprint
                double new_r = std::hypot(x, y);
                double new_theta = std::atan2(y, x);

                // Slot into the new high-res array
                int bin = std::round((new_theta - merged_msg.angle_min) / merged_msg.angle_increment);
                
                // Wrap around safely
                if (bin >= num_bins) bin = 0;
                if (bin < 0) bin = num_bins - 1;

                if (new_r < merged_ranges[bin]) {
                    merged_ranges[bin] = new_r; // Keep closest wall
                }
            }
        };

        // Process both raw scans directly into the final merged array
        process_to_grid(front_msg, front_offset_x_, false);
        process_to_grid(back_msg, back_offset_x_, true);

        merged_msg.ranges = merged_ranges;
        pub_merged_->publish(merged_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMergerNode>());
    rclcpp::shutdown();
    return 0;
}