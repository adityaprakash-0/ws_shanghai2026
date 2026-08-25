#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

// ROS REP-103 body frame (base_footprint):
//   +X forward, +Y left, +Z up, +yaw = CCW about Z
//
// 4-wheel X-drive (omni wheels at ~45°). Wheel mix signs match ROS axes;
// √2 accounts for the 45° projection between wheel roll and body travel.
class XDriveNode : public rclcpp::Node {
public:
    XDriveNode() : Node("xdrive_node") {
        this->declare_parameter<double>("ticks_per_rev", 1221);
        this->declare_parameter<double>("wheel_radius", 0.05);
        this->declare_parameter<double>("geom_scale", std::sqrt(2.0));

        ticks_per_rev_ = this->get_parameter("ticks_per_rev").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        geom_scale_ = this->get_parameter("geom_scale").as_double();
        meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / ticks_per_rev_;

        pub_fl_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_2/cmd", 10);
        pub_fr_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_0/cmd", 10);
        pub_rl_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_3/cmd", 10);
        pub_rr_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_1/cmd", 10);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
        wheel_dist_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_distances", 10);
        robot_dist_x_pub_ = this->create_publisher<std_msgs::msg::Float32>("/robot_distance_x", 10);
        robot_dist_y_pub_ = this->create_publisher<std_msgs::msg::Float32>("/robot_distance_y", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&XDriveNode::cmd_vel_callback, this, std::placeholders::_1));

        sim_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/adev/simulation", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                simulation_mode_ = msg->data;
            });

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&XDriveNode::imu_callback, this, std::placeholders::_1));

        sub_fl_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_2/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_fl_ = msg->data; });
        sub_fr_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_0/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_fr_ = msg->data; });
        sub_rl_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_3/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_rl_ = msg->data; });
        sub_rr_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_1/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_rr_ = msg->data; });

        last_time_ = this->now();
        // ~20 Hz — matches encoder update rate so vx/vy is not zero between tick updates
        odom_timer_ = this->create_wall_timer(50ms, std::bind(&XDriveNode::publish_odometry, this));

        RCLCPP_INFO(
            this->get_logger(),
            "X-Drive node EKF mode (ROS axes: +X fwd, +Y left, +yaw CCW), geom_scale=%.4f",
            geom_scale_);
    }

private:
    double x_ = 0.0, y_ = 0.0, th_ = 0.0;
    double current_yaw_rad_ = 0.0, prev_yaw_rad_ = 0.0;

    double prev_dist_fl_ = 0.0, prev_dist_fr_ = 0.0, prev_dist_rl_ = 0.0, prev_dist_rr_ = 0.0;
    rclcpp::Time last_time_;

    double ticks_per_rev_, wheel_radius_, meters_per_tick_;
    double geom_scale_;
    double ticks_fl_ = 0.0, ticks_fr_ = 0.0, ticks_rl_ = 0.0, ticks_rr_ = 0.0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_cmd_, pub_fr_cmd_, pub_rl_cmd_, pub_rr_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr robot_dist_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr robot_dist_y_pub_;

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
        // Twist in ROS body frame: +vx forward, +vy left, +wz CCW
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double wz = msg->angular.z;

        // Duty-cycle mix uses ROS X-drive / mecanum signs WITHOUT 1/√2.
        // Putting 1/√2 on motor cmds makes slow forward sit in the motor
        // deadband unevenly → one diagonal wins → looks like a strafe.
        // √2 stays in odometry only (body_from_wheels / geom_scale).
        constexpr double k_lat = 0.02;  // treat tiny stick noise as 0
        if (std::abs(vy) < k_lat) {
            vy = 0.0;
        }
        if (std::abs(wz) < k_lat) {
            wz = 0.0;
        }

        double front_left = 0.0;
        double front_right = 0.0;
        double rear_left = 0.0;
        double rear_right = 0.0;

        // Near-pure forward/back: force equal wheel cmds so low-speed
        // friction can't create a diagonal (strafe) couple.
        if (std::abs(vy) < 1e-6 && std::abs(wz) < 1e-6) {
            front_left = front_right = rear_left = rear_right = vx;
        } else {
            front_left = vx - vy - wz;
            front_right = vx + vy + wz;
            rear_left = vx + vy - wz;
            rear_right = vx - vy + wz;
        }

        // Keep direction if any channel would saturate past ±1
        const double peak = std::max({
            std::abs(front_left), std::abs(front_right),
            std::abs(rear_left), std::abs(rear_right), 1.0});
        if (peak > 1.0) {
            front_left /= peak;
            front_right /= peak;
            rear_left /= peak;
            rear_right /= peak;
        }

        std_msgs::msg::Float64 fl_cmd, fr_cmd, rl_cmd, rr_cmd;
        fl_cmd.data = front_left;
        fr_cmd.data = front_right;
        rl_cmd.data = rear_left;
        rr_cmd.data = rear_right;

        pub_fl_cmd_->publish(fl_cmd);
        pub_fr_cmd_->publish(fr_cmd);
        pub_rl_cmd_->publish(rl_cmd);
        pub_rr_cmd_->publish(rr_cmd);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_rad_ = yaw;
    }

    // Inverse of cmd_vel mix (ROS body frame):
    //   vx = geom_scale/4 * ( FL + FR + RL + RR)
    //   vy = geom_scale/4 * (-FL + FR + RL - RR)   // +vy = left
    void body_from_wheels(
        double fl, double fr, double rl, double rr,
        double & body_x, double & body_y) const
    {
        body_x = geom_scale_ * (fl + fr + rl + rr) / 4.0;
        body_y = geom_scale_ * (-fl + fr + rl - rr) / 4.0;
    }

    void publish_odometry() {
        if (simulation_mode_) {
            return;
        }
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) return;

        const double dist_fl = ticks_fl_ * meters_per_tick_;
        const double dist_fr = ticks_fr_ * meters_per_tick_;
        const double dist_rl = ticks_rl_ * meters_per_tick_;
        const double dist_rr = ticks_rr_ * meters_per_tick_;

        std_msgs::msg::Float32MultiArray wheel_msg;
        wheel_msg.data.resize(4);
        wheel_msg.data[0] = dist_fl;
        wheel_msg.data[1] = dist_fr;
        wheel_msg.data[2] = dist_rl;
        wheel_msg.data[3] = dist_rr;
        wheel_dist_pub_->publish(wheel_msg);

        double current_dist_x = 0.0, current_dist_y = 0.0;
        body_from_wheels(dist_fl, dist_fr, dist_rl, dist_rr, current_dist_x, current_dist_y);

        std_msgs::msg::Float32 msg_x, msg_y;
        msg_x.data = static_cast<float>(current_dist_x);
        msg_y.data = static_cast<float>(current_dist_y);
        robot_dist_x_pub_->publish(msg_x);
        robot_dist_y_pub_->publish(msg_y);

        const double delta_fl = dist_fl - prev_dist_fl_;
        const double delta_fr = dist_fr - prev_dist_fr_;
        const double delta_rl = dist_rl - prev_dist_rl_;
        const double delta_rr = dist_rr - prev_dist_rr_;

        double delta_x_local = 0.0, delta_y_local = 0.0;
        body_from_wheels(delta_fl, delta_fr, delta_rl, delta_rr, delta_x_local, delta_y_local);

        if (std::abs(delta_x_local) > 0.5 || std::abs(delta_y_local) > 0.5) {
            delta_x_local = 0.0;
            delta_y_local = 0.0;
        }

        double delta_yaw = current_yaw_rad_ - prev_yaw_rad_;
        while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
        while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;
        if (std::abs(delta_yaw) > 0.5) delta_yaw = 0.0;

        const double v_x = delta_x_local / dt;
        const double v_y = delta_y_local / dt;
        const double v_th = delta_yaw / dt;

        // Integrate in odom frame using body-frame increments (ROS yaw CCW)
        x_ += delta_x_local * cos(th_) - delta_y_local * sin(th_);
        y_ += delta_x_local * sin(th_) + delta_y_local * cos(th_);
        th_ += delta_yaw;

        tf2::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, th_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.x = odom_quat.x();
        odom.pose.pose.orientation.y = odom_quat.y();
        odom.pose.pose.orientation.z = odom_quat.z();
        odom.pose.pose.orientation.w = odom_quat.w();
        odom.twist.twist.linear.x = v_x;
        odom.twist.twist.linear.y = v_y;
        odom.twist.twist.angular.z = v_th;

        for (int i = 0; i < 36; ++i) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        // Calibrated 2026-08-23 (skill23_test odom_cal): fwd scale 0.998, strafe L/R 1.021/1.006
        // ~±1–2 % distance error → σ≈0.019 m → var≈3.5e-4
        odom.pose.covariance[0] = 3.5e-4;   // x
        odom.pose.covariance[7] = 3.5e-4;   // y
        odom.pose.covariance[14] = 1e6;   // z unused
        odom.pose.covariance[21] = 1e6;   // roll unused
        odom.pose.covariance[28] = 1e6;   // pitch unused
        odom.pose.covariance[35] = 0.02;  // yaw (not fused; IMU owns heading)

        odom.twist.covariance[0] = 0.0025;  // vx  (σ≈0.05 m/s)
        odom.twist.covariance[7] = 0.0036;  // vy  strafe left ~2 % low → trust slightly less
        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;
        odom.twist.covariance[35] = 0.001;  // vyaw from IMU Δyaw (not fused)

        odom_pub_->publish(odom);
        prev_dist_fl_ = dist_fl;
        prev_dist_fr_ = dist_fr;
        prev_dist_rl_ = dist_rl;
        prev_dist_rr_ = dist_rr;
        prev_yaw_rad_ = current_yaw_rad_;
        last_time_ = current_time;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XDriveNode>());
    rclcpp::shutdown();
    return 0;
}
