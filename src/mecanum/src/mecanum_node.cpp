#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class MecanumNode : public rclcpp::Node {
public:
    MecanumNode() : Node("mecanum_node") {
        this->declare_parameter<double>("ticks_per_rev", 1221);
        this->declare_parameter<double>("wheel_radius", 0.05);

        ticks_per_rev_ = this->get_parameter("ticks_per_rev").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
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
            "/cmd_vel", 10, std::bind(&MecanumNode::cmd_vel_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&MecanumNode::imu_callback, this, std::placeholders::_1));

        sub_fl_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_2/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_fl_ = msg->data; });
        sub_fr_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_0/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_fr_ = msg->data; });
        sub_rl_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_3/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_rl_ = msg->data; });
        sub_rr_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_1/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_rr_ = msg->data; });

        last_time_ = this->now();
        odom_timer_ = this->create_wall_timer(20ms, std::bind(&MecanumNode::publish_odometry, this));

        RCLCPP_INFO(this->get_logger(), "Mecanum node started. EKF Mode & X/Y PID separation active.");
    }

private:
    double x_ = 0.0, y_ = 0.0, th_ = 0.0;
    double current_yaw_rad_ = 0.0, prev_yaw_rad_ = 0.0;

    double prev_dist_fl_ = 0.0, prev_dist_fr_ = 0.0, prev_dist_rl_ = 0.0, prev_dist_rr_ = 0.0;
    rclcpp::Time last_time_;

    double ticks_per_rev_, wheel_radius_, meters_per_tick_;
    double ticks_fl_ = 0.0, ticks_fr_ = 0.0, ticks_rl_ = 0.0, ticks_rr_ = 0.0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_cmd_, pub_fr_cmd_, pub_rl_cmd_, pub_rr_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr robot_dist_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr robot_dist_y_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_fl_enc_, sub_fr_enc_, sub_rl_enc_, sub_rr_enc_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double x_vel = msg->linear.x;
        double y_vel = msg->linear.y;
        double angular = msg->angular.z;

        // Mecanum kinematics — duty cycle −1.0…1.0 on /cmd_vel and /titan*/cmd
        double front_left = x_vel - y_vel - angular;
        double front_right = x_vel + y_vel + angular;
        double rear_left = x_vel + y_vel - angular;
        double rear_right = x_vel - y_vel + angular;

        std_msgs::msg::Float64 fl_cmd, fr_cmd, rl_cmd, rr_cmd;
        fl_cmd.data = front_left;
        rl_cmd.data = rear_left;
        fr_cmd.data = front_right;
        rr_cmd.data = rear_right;

        pub_fl_cmd_->publish(fl_cmd);
        pub_rl_cmd_->publish(rl_cmd);
        pub_fr_cmd_->publish(fr_cmd);
        pub_rr_cmd_->publish(rr_cmd);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_rad_ = yaw;
    }

    void publish_odometry() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) return;

        double dist_fl = ticks_fl_ * meters_per_tick_;
        double dist_fr = ticks_fr_ * meters_per_tick_;
        double dist_rl = ticks_rl_ * meters_per_tick_;
        double dist_rr = ticks_rr_ * meters_per_tick_;

        std_msgs::msg::Float32MultiArray wheel_msg;
        wheel_msg.data.resize(4);
        wheel_msg.data[0] = dist_fl;
        wheel_msg.data[1] = dist_fr;
        wheel_msg.data[2] = dist_rl;
        wheel_msg.data[3] = dist_rr;
        wheel_dist_pub_->publish(wheel_msg);

        double current_dist_x = (dist_fl + dist_fr + dist_rl + dist_rr) / 4.0;
        double current_dist_y = (-dist_fl + dist_fr + dist_rl - dist_rr) / 4.0;

        std_msgs::msg::Float32 msg_x, msg_y;
        msg_x.data = current_dist_x;
        msg_y.data = current_dist_y;
        robot_dist_x_pub_->publish(msg_x);
        robot_dist_y_pub_->publish(msg_y);

        double delta_fl = dist_fl - prev_dist_fl_;
        double delta_fr = dist_fr - prev_dist_fr_;
        double delta_rl = dist_rl - prev_dist_rl_;
        double delta_rr = dist_rr - prev_dist_rr_;

        double delta_x_local = (delta_fl + delta_fr + delta_rl + delta_rr) / 4.0;
        double delta_y_local = (-delta_fl + delta_fr + delta_rl - delta_rr) / 4.0;

        if (std::abs(delta_x_local) > 0.5 || std::abs(delta_y_local) > 0.5) {
            delta_x_local = 0.0;
            delta_y_local = 0.0;
        }

        double delta_yaw = current_yaw_rad_ - prev_yaw_rad_;
        while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
        while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;
        if (std::abs(delta_yaw) > 0.5) delta_yaw = 0.0;

        double v_x = delta_x_local / dt;
        double v_y = delta_y_local / dt;
        double v_th = delta_yaw / dt;

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

        // pose cov 6x6 row-major: x,y,z,roll,pitch,yaw
        // twist cov: vx,vy,vz,vroll,vpitch,vyaw
        // Unused axes → 1e6 so EKF / consumers ignore them.
        for (int i = 0; i < 36; ++i) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        odom.pose.covariance[0]  = 0.05;   // x  (m^2) — wheel slip
        odom.pose.covariance[7]  = 0.05;   // y  (m^2) — strafe slip
        odom.pose.covariance[14] = 1e6;    // z
        odom.pose.covariance[21] = 1e6;    // roll
        odom.pose.covariance[28] = 1e6;    // pitch
        odom.pose.covariance[35] = 0.02;   // yaw (rad^2) — from IMU delta

        odom.twist.covariance[0]  = 0.1;   // vx
        odom.twist.covariance[7]  = 0.1;   // vy
        odom.twist.covariance[14] = 1e6;   // vz
        odom.twist.covariance[21] = 1e6;   // vroll
        odom.twist.covariance[28] = 1e6;   // vpitch
        odom.twist.covariance[35] = 0.001; // vyaw — IMU Δyaw/dt, matches gyro noise

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
    rclcpp::spin(std::make_shared<MecanumNode>());
    rclcpp::shutdown();
    return 0;
}
