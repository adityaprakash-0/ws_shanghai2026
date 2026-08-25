// #include <chrono>
// #include <cmath>
// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float64.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2/LinearMath/Matrix3x3.h"
// #include "tf2_ros/transform_broadcaster.h"
// #include "std_msgs/msg/float32.hpp"
// #include "std_msgs/msg/float32_multi_array.hpp"

// using namespace std::chrono_literals;

// class AllTerrainNode : public rclcpp::Node {
// public:
//     AllTerrainNode() : Node("allterrain_node") {
//         this->declare_parameter<double>("ticks_per_rev", 1464.0);
//         this->declare_parameter<double>("wheel_radius", 0.06);
        
//         ticks_per_rev_ = this->get_parameter("ticks_per_rev").as_double();
//         wheel_radius_ = this->get_parameter("wheel_radius").as_double();
//         meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / ticks_per_rev_;

//         pub_fl_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_0/cmd", 10);
//         pub_fr_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_1/cmd", 10);
//         pub_rl_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_2/cmd", 10);
//         pub_rr_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_3/cmd", 10);

//         odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
//         wheel_dist_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_distances", 10);
//         robot_dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("/robot_distance", 10);

//         tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

//         cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "/cmd_vel", 10, std::bind(&AllTerrainNode::cmd_vel_callback, this, std::placeholders::_1));
        
//         imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
//             "/imu", 10, std::bind(&AllTerrainNode::imu_callback, this, std::placeholders::_1));

//         sub_fl_enc_ = this->create_subscription<std_msgs::msg::Float64>(
//             "/titan0/m_0/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_fl_ = msg->data; });
//         sub_fr_enc_ = this->create_subscription<std_msgs::msg::Float64>(
//             "/titan0/m_1/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_fr_ = msg->data; });
//         sub_rl_enc_ = this->create_subscription<std_msgs::msg::Float64>(
//             "/titan0/m_2/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_rl_ = msg->data; });
//         sub_rr_enc_ = this->create_subscription<std_msgs::msg::Float64>(
//             "/titan0/m_3/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_rr_ = msg->data; });

//         last_time_ = this->now();
//         odom_timer_ = this->create_wall_timer(20ms, std::bind(&AllTerrainNode::publish_odometry, this));

//         RCLCPP_INFO(this->get_logger(), "AllTerrain node started successfully.");
//     }

// private:
//     double x_ = 0.0, y_ = 0.0, th_ = 0.0;
//     double current_yaw_rad_ = 0.0, prev_yaw_rad_ = 0.0;
//     double prev_robot_dist_ = 0.0;
//     rclcpp::Time last_time_;

//     double ticks_per_rev_, wheel_radius_, meters_per_tick_;
//     double ticks_fl_ = 0.0, ticks_fr_ = 0.0, ticks_rl_ = 0.0, ticks_rr_ = 0.0;

//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_cmd_, pub_fr_cmd_, pub_rl_cmd_, pub_rr_cmd_;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_dist_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr robot_dist_pub_;
//     std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
//     rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_fl_enc_, sub_fr_enc_, sub_rl_enc_, sub_rr_enc_;
//     rclcpp::TimerBase::SharedPtr odom_timer_;

//     void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//         double linear = msg->linear.x;
//         double angular = msg->angular.z;

//         double left_speed = linear - angular;
//         double right_speed = linear + angular;

//         std_msgs::msg::Float64 fl_cmd, fr_cmd, rl_cmd, rr_cmd;
//         fl_cmd.data = left_speed;
//         rl_cmd.data = left_speed;
//         fr_cmd.data = right_speed;
//         rr_cmd.data = right_speed;

//         pub_fl_cmd_->publish(fl_cmd);
//         pub_rl_cmd_->publish(rl_cmd);
//         pub_fr_cmd_->publish(fr_cmd);
//         pub_rr_cmd_->publish(rr_cmd);
//     }

//     void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
//         tf2::Quaternion q(
//             msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
//         tf2::Matrix3x3 m(q);
//         double roll, pitch, yaw;
//         m.getRPY(roll, pitch, yaw);
//         current_yaw_rad_ = yaw;
//     }

//     void publish_odometry() {
//         rclcpp::Time current_time = this->now();
//         double dt = (current_time - last_time_).seconds();
//         if (dt <= 0.0) return;

//         double dist_fl = ticks_fl_ * meters_per_tick_;
//         double dist_fr = ticks_fr_ * meters_per_tick_;
//         double dist_rl = ticks_rl_ * meters_per_tick_;
//         double dist_rr = ticks_rr_ * meters_per_tick_;

//         double current_robot_dist = (dist_fl + dist_fr + dist_rl + dist_rr) / 4.0;

//         // 1. Publish exact hardware distances directly
//         std_msgs::msg::Float32MultiArray wheel_msg;
//         wheel_msg.data.resize(4);
//         wheel_msg.data[0] = dist_fl; wheel_msg.data[1] = dist_fr;
//         wheel_msg.data[2] = dist_rl; wheel_msg.data[3] = dist_rr;
//         wheel_dist_pub_->publish(wheel_msg);

//         std_msgs::msg::Float32 robot_msg;
//         robot_msg.data = current_robot_dist;
//         robot_dist_pub_->publish(robot_msg);

//         // 2. Odometry Math Shield
//         double delta_distance = current_robot_dist - prev_robot_dist_;
        
//         // If encoders reset to zero, distance drops massively. Shield it.
//         if (delta_distance < -0.1) {
//             delta_distance = 0.0;
//         }

//         double delta_yaw = current_yaw_rad_ - prev_yaw_rad_;
//         while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
//         while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

//         // If yaw instantly jumps more than ~28 degrees in 20ms, an IMU reset happened. Shield it.
//         if (std::abs(delta_yaw) > 0.5) {
//             delta_yaw = 0.0;
//         }

//         double v_x = delta_distance / dt;
//         double v_th = delta_yaw / dt;

//         x_ += delta_distance * cos(th_);
//         y_ += delta_distance * sin(th_);
//         th_ += delta_yaw;

//         tf2::Quaternion odom_quat;
//         odom_quat.setRPY(0, 0, th_);

//         nav_msgs::msg::Odometry odom;
//         odom.header.stamp = current_time;
//         odom.header.frame_id = "odom";
//         odom.child_frame_id = "base_footprint";

//         odom.pose.pose.position.x = x_;
//         odom.pose.pose.position.y = y_;
//         odom.pose.pose.orientation.x = odom_quat.x();
//         odom.pose.pose.orientation.y = odom_quat.y();
//         odom.pose.pose.orientation.z = odom_quat.z();
//         odom.pose.pose.orientation.w = odom_quat.w();
//         odom.twist.twist.linear.x = v_x;
//         odom.twist.twist.angular.z = v_th;

//         odom_pub_->publish(odom);

//         geometry_msgs::msg::TransformStamped t;
//         t.header.stamp = current_time;
//         t.header.frame_id = "odom";
//         t.child_frame_id = "base_footprint";
//         t.transform.translation.x = x_;
//         t.transform.translation.y = y_;
//         t.transform.rotation.x = odom_quat.x();
//         t.transform.rotation.y = odom_quat.y();
//         t.transform.rotation.z = odom_quat.z();
//         t.transform.rotation.w = odom_quat.w();

//         tf_broadcaster_->sendTransform(t);

//         prev_robot_dist_ = current_robot_dist;
//         prev_yaw_rad_ = current_yaw_rad_;
//         last_time_ = current_time;
//     }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<AllTerrainNode>());
//     rclcpp::shutdown();
//     return 0;
// }


#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

// REMOVED TF BROADCASTER INCLUDE - The EKF handles this now!

using namespace std::chrono_literals;

class AllTerrainNode : public rclcpp::Node {
public:
    AllTerrainNode() : Node("allterrain_node") {
        this->declare_parameter<double>("ticks_per_rev", 1221.0);
        this->declare_parameter<double>("wheel_radius", 0.0625);
        
        ticks_per_rev_ = this->get_parameter("ticks_per_rev").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / ticks_per_rev_;

        pub_fl_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_2/cmd", 10);
        pub_fr_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_0/cmd", 10);
        pub_rl_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_3/cmd", 10);
        pub_rr_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_1/cmd", 10);

        // --- UPDATED: Publish to /wheel/odom for the EKF to consume ---
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
        
        wheel_dist_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_distances", 10);
        robot_dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("/robot_distance", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&AllTerrainNode::cmd_vel_callback, this, std::placeholders::_1));

        sim_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/adev/simulation", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                simulation_mode_ = msg->data;
            });
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&AllTerrainNode::imu_callback, this, std::placeholders::_1));

        sub_fl_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_2/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_fl_ = msg->data; });
        sub_fr_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_0/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_fr_ = msg->data; });
        sub_rl_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_3/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_rl_ = msg->data; });
        sub_rr_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_1/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_rr_ = msg->data; });

        last_time_ = this->now();
        // ~20 Hz — matches encoder update rate so vx is not zero between tick updates
        odom_timer_ = this->create_wall_timer(50ms, std::bind(&AllTerrainNode::publish_odometry, this));

        RCLCPP_INFO(this->get_logger(), "AllTerrain node started. Diff-drive EKF mode (r=%.4f, ticks=%.0f).",
                    wheel_radius_, ticks_per_rev_);
    }

private:
    double x_ = 0.0, y_ = 0.0, th_ = 0.0;
    double current_yaw_rad_ = 0.0, prev_yaw_rad_ = 0.0;
    double prev_robot_dist_ = 0.0;
    rclcpp::Time last_time_;

    double ticks_per_rev_, wheel_radius_, meters_per_tick_;
    double ticks_fl_ = 0.0, ticks_fr_ = 0.0, ticks_rl_ = 0.0, ticks_rr_ = 0.0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_cmd_, pub_fr_cmd_, pub_rl_cmd_, pub_rr_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr robot_dist_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sim_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_fl_enc_, sub_fr_enc_, sub_rl_enc_, sub_rr_enc_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    bool simulation_mode_ = false;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (simulation_mode_) {
            return;
        }
        // Nav2 cmd_vel.linear.x / angular.z are Titan duty cycles (−1.0…1.0), not m/s.
        // Skid-steer mix: left = v − ω, right = v + ω (see titan_component /cmd topics).
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        double left_speed = linear - angular;
        double right_speed = linear + angular;

        std_msgs::msg::Float64 fl_cmd, fr_cmd, rl_cmd, rr_cmd;
        fl_cmd.data = left_speed; rl_cmd.data = left_speed;
        fr_cmd.data = right_speed; rr_cmd.data = right_speed;

        pub_fl_cmd_->publish(fl_cmd); pub_rl_cmd_->publish(rl_cmd);
        pub_fr_cmd_->publish(fr_cmd); pub_rr_cmd_->publish(rr_cmd);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_rad_ = yaw;
    }

    void publish_odometry() {
        if (simulation_mode_) {
            return;
        }
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) return;

        double dist_fl = ticks_fl_ * meters_per_tick_;
        double dist_fr = ticks_fr_ * meters_per_tick_;
        double dist_rl = ticks_rl_ * meters_per_tick_;
        double dist_rr = ticks_rr_ * meters_per_tick_;

        double current_robot_dist = (dist_fl + dist_fr + dist_rl + dist_rr) / 4.0;

        std_msgs::msg::Float32MultiArray wheel_msg;
        wheel_msg.data.resize(4);
        wheel_msg.data[0] = dist_fl; wheel_msg.data[1] = dist_fr;
        wheel_msg.data[2] = dist_rl; wheel_msg.data[3] = dist_rr;
        wheel_dist_pub_->publish(wheel_msg);

        std_msgs::msg::Float32 robot_msg;
        robot_msg.data = current_robot_dist;
        robot_dist_pub_->publish(robot_msg);

        double delta_distance = current_robot_dist - prev_robot_dist_;
        if (delta_distance < -0.1) delta_distance = 0.0;

        double delta_yaw = current_yaw_rad_ - prev_yaw_rad_;
        while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
        while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

        if (std::abs(delta_yaw) > 0.5) delta_yaw = 0.0;

        double v_x = delta_distance / dt;
        double v_th = delta_yaw / dt;

        x_ += delta_distance * cos(th_);
        y_ += delta_distance * sin(th_);
        th_ += delta_yaw;

        tf2::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, th_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        // NOTE: EKF requires frame_ids to match exactly what you put in the yaml!
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.x = odom_quat.x();
        odom.pose.pose.orientation.y = odom_quat.y();
        odom.pose.pose.orientation.z = odom_quat.z();
        odom.pose.pose.orientation.w = odom_quat.w();
        odom.twist.twist.linear.x = v_x;
        odom.twist.twist.angular.z = v_th;

        // Covariances — ~±2 cm distance accuracy → σ≈0.02 m → var≈4e-4 (same as grip)
        for (size_t i = 0; i < 36; ++i) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        odom.pose.covariance[0] = 4e-4;   // x
        odom.pose.covariance[7] = 4e-4;   // y
        odom.pose.covariance[14] = 1e6;   // z unused
        odom.pose.covariance[21] = 1e6;   // roll unused
        odom.pose.covariance[28] = 1e6;   // pitch unused
        odom.pose.covariance[35] = 0.02;  // yaw (not fused; IMU owns heading)

        odom.twist.covariance[0] = 0.0025;  // vx  (σ≈0.05 m/s)
        odom.twist.covariance[7] = 1e6;     // vy unused (diff-drive)
        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;
        odom.twist.covariance[35] = 0.001;  // vyaw (not fused)

        odom_pub_->publish(odom);

        // --- REMOVED TF BROADCASTER CODE ENTIRELY ---
        // The EKF is now exclusively responsible for broadcasting odom -> base_footprint.

        prev_robot_dist_ = current_robot_dist;
        prev_yaw_rad_ = current_yaw_rad_;
        last_time_ = current_time;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AllTerrainNode>());
    rclcpp::shutdown();
    return 0;
}