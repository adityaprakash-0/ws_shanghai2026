// Dual-LiDAR merger: /scan/front + /scan/back -> one 360 deg /scan in
// base_footprint, consumed by AMCL and the obstacle detector.
//
// Deliberately minimal. Earlier revisions did per-beam motion compensation
// through timed TF lookups; on this robot the EKF odom transform always lags
// the scan end time, so every cycle blocked in TF, failed, and fell back to
// an uncompensated scan anyway — pure latency into AMCL. At competition
// speeds (< 0.5 m/s, 7 Hz scans) the skew error is below the 3 cm map
// resolution, so this node uses only the fixed sensor mount transforms from
// the URDF (looked up once, then cached) and stamps the merged scan at the
// sweep midpoint, a time the EKF can always serve.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
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
        front_half_fov_rad_ = degrees_to_radians(
            declare_parameter<double>("front_half_fov_deg", 132.0));
        back_half_fov_rad_ = degrees_to_radians(
            declare_parameter<double>("back_half_fov_deg", 132.0));
        max_sync_skew_sec_ =
            declare_parameter<double>("max_sync_skew_sec", 0.10);
        // 720 bins (0.5 deg) matches the useful angular density at 7 Hz.
        // Higher bin counts create artificial gaps rather than more detail.
        output_bins_ = declare_parameter<int>("output_bins", 720);
        footprint_length_ =
            declare_parameter<double>("footprint_length", 0.44);
        footprint_width_ =
            declare_parameter<double>("footprint_width", 0.40);
        footprint_margin_ =
            declare_parameter<double>("footprint_margin", 0.01);

        front_half_fov_rad_ = std::clamp(front_half_fov_rad_, 0.0, M_PI);
        back_half_fov_rad_ = std::clamp(back_half_fov_rad_, 0.0, M_PI);
        output_bins_ = std::max(output_bins_, 360);
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
            "%d bins, static mount transforms -> %s",
            radians_to_degrees(front_half_fov_rad_),
            radians_to_degrees(back_half_fov_rad_),
            max_sync_skew_sec_ * 1000.0, output_bins_, target_frame_.c_str());
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

    static rclcpp::Time scan_mid_stamp(const LaserScan & scan)
    {
        const rclcpp::Time start(scan.header.stamp);
        if (scan.ranges.size() < 2) {
            return start;
        }
        double increment = static_cast<double>(scan.time_increment);
        if (increment <= 0.0 && scan.scan_time > 0.0F) {
            // Some drivers leave time_increment at zero; scan_time still
            // tells us how long the sweep took.
            increment =
                static_cast<double>(scan.scan_time) /
                static_cast<double>(scan.ranges.size() - 1);
        }
        if (increment <= 0.0) {
            return start;
        }
        return start + rclcpp::Duration::from_seconds(
            increment * static_cast<double>(scan.ranges.size() - 1) / 2.0);
    }

    /// Fixed lidar mount pose in the target frame, from the URDF via
    /// robot_state_publisher. Static, so one successful lookup is cached for
    /// the lifetime of the node — no TF traffic afterwards.
    const PlanarTransform & mount_transform(const std::string & source_frame)
    {
        auto & cached = mounts_[source_frame];
        if (cached.valid) {
            return cached;
        }
        try {
            const auto transform = tf_buffer_->lookupTransform(
                target_frame_, source_frame, tf2::TimePointZero);
            const auto & q = transform.transform.rotation;
            const double yaw = std::atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            cached.x = transform.transform.translation.x;
            cached.y = transform.transform.translation.y;
            cached.cos_yaw = std::cos(yaw);
            cached.sin_yaw = std::sin(yaw);
            cached.valid = true;
        } catch (const tf2::TransformException & exception) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for %s -> %s mount transform: %s",
                source_frame.c_str(), target_frame_.c_str(),
                exception.what());
        }
        return cached;
    }

    void add_scan(
        const LaserScan::ConstSharedPtr & scan,
        double half_fov_rad,
        const PlanarTransform & transform,
        const LaserScan & output_template)
    {
        const std::size_t count = scan->ranges.size();
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

        const auto & front_mount = mount_transform(front->header.frame_id);
        const auto & back_mount = mount_transform(back->header.frame_id);
        if (!front_mount.valid || !back_mount.valid) {
            // Better no scan than a wrongly-placed one; the mount transforms
            // arrive as soon as robot_state_publisher is up.
            return;
        }

        // Stamp at the pair midpoint: the pose consumers (AMCL, EKF lookup)
        // always have data at this slightly-past time, unlike the sweep end.
        const rclcpp::Time target_stamp =
            front_mid < back_mid
            ? front_mid + (back_mid - front_mid) * 0.5
            : back_mid + (front_mid - back_mid) * 0.5;

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

        add_scan(front, front_half_fov_rad_, front_mount, merged);
        add_scan(back, back_half_fov_rad_, back_mount, merged);

        const bool empty = std::none_of(
            merged_ranges_.begin(), merged_ranges_.end(),
            [](float r) {return std::isfinite(r);});
        if (empty) {
            // AMCL silently drifts on an all-infinity scan, so make it loud.
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Merged scan has no finite returns — check %s/%s data",
                front->header.frame_id.c_str(), back->header.frame_id.c_str());
        }

        merged.ranges = merged_ranges_;
        merged.intensities = merged_intensities_;
        publisher_->publish(merged);
    }

    std::string target_frame_;
    double front_half_fov_rad_{0.0};
    double back_half_fov_rad_{0.0};
    double max_sync_skew_sec_{0.10};
    int output_bins_{720};
    double footprint_length_{0.44};
    double footprint_width_{0.40};
    double footprint_margin_{0.01};

    std::unordered_map<std::string, PlanarTransform> mounts_;
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
