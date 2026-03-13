#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class XDriveNode : public rclcpp::Node {
public:
    XDriveNode() : Node("xdrive_node") {
        this->declare_parameter<double>("ticks_per_rev", 1464.0);
        this->declare_parameter<double>("wheel_radius", 0.06);
        
        ticks_per_rev_ = this->get_parameter("ticks_per_rev").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / ticks_per_rev_;

        pub_fl_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_0/cmd", 10);
        pub_fr_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_1/cmd", 10);
        pub_rl_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_2/cmd", 10);
        pub_rr_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_3/cmd", 10);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
        
        wheel_dist_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_distances", 10);
        robot_dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("/robot_distance", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&XDriveNode::cmd_vel_callback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&XDriveNode::imu_callback, this, std::placeholders::_1));

        sub_fl_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_0/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_fl_ = msg->data; });
        sub_fr_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_1/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_fr_ = msg->data; });
        sub_rl_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_2/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_rl_ = msg->data; });
        sub_rr_enc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan0/m_3/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { ticks_rr_ = msg->data; });

        last_time_ = this->now();
        odom_timer_ = this->create_wall_timer(20ms, std::bind(&XDriveNode::publish_odometry, this));

        RCLCPP_INFO(this->get_logger(), "X-Drive node started. EKF Mode Active.");
    }

private:
    double x_ = 0.0, y_ = 0.0, th_ = 0.0;
    double current_yaw_rad_ = 0.0, prev_yaw_rad_ = 0.0;
    
    // Track previous individual wheel distances
    double prev_dist_fl_ = 0.0, prev_dist_fr_ = 0.0, prev_dist_rl_ = 0.0, prev_dist_rr_ = 0.0;
    rclcpp::Time last_time_;

    double ticks_per_rev_, wheel_radius_, meters_per_tick_;
    double ticks_fl_ = 0.0, ticks_fr_ = 0.0, ticks_rl_ = 0.0, ticks_rr_ = 0.0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_cmd_, pub_fr_cmd_, pub_rl_cmd_, pub_rr_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr robot_dist_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_fl_enc_, sub_fr_enc_, sub_rl_enc_, sub_rr_enc_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double x_vel = msg->linear.x;
        double y_vel = msg->linear.y; // X-Drive uses Y for strafing
        double angular = msg->angular.z;

        // Old X-Drive Kinematics
        double front_left = (x_vel - y_vel) - angular;
        double front_right = (x_vel + y_vel) + angular;
        double rear_left = (x_vel + y_vel) - angular;
        double rear_right = (x_vel - y_vel) + angular;

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
        tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
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

        // Publish raw wheel distances
        std_msgs::msg::Float32MultiArray wheel_msg;
        wheel_msg.data.resize(4);
        wheel_msg.data[0] = dist_fl; wheel_msg.data[1] = dist_fr;
        wheel_msg.data[2] = dist_rl; wheel_msg.data[3] = dist_rr;
        wheel_dist_pub_->publish(wheel_msg);

        // Calculate deltas since last loop
        double delta_fl = dist_fl - prev_dist_fl_;
        double delta_fr = dist_fr - prev_dist_fr_;
        double delta_rl = dist_rl - prev_dist_rl_;
        double delta_rr = dist_rr - prev_dist_rr_;

        // Inverse Kinematics for X-Drive (Mathematically identical to Mecanum in local space)
        double delta_x_local = (delta_fl + delta_fr + delta_rl + delta_rr) / 4.0;
        double delta_y_local = (-delta_fl + delta_fr + delta_rl - delta_rr) / 4.0;

        // Odometry Math Shield (If encoder reset causes massive spike, ignore it)
        if (std::abs(delta_x_local) > 0.5 || std::abs(delta_y_local) > 0.5) {
            delta_x_local = 0.0;
            delta_y_local = 0.0;
        }

        // Aggregate distance metric for robot_dist_pub
        std_msgs::msg::Float32 robot_msg;
        robot_msg.data = (dist_fl + dist_fr + dist_rl + dist_rr) / 4.0; 
        robot_dist_pub_->publish(robot_msg);

        // Yaw calculation from IMU
        double delta_yaw = current_yaw_rad_ - prev_yaw_rad_;
        while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
        while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

        if (std::abs(delta_yaw) > 0.5) delta_yaw = 0.0;

        // Calculate velocities
        double v_x = delta_x_local / dt;
        double v_y = delta_y_local / dt;
        double v_th = delta_yaw / dt;

        // Rotate local translation into global frame
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
        odom.twist.twist.linear.y = v_y; // IMPORTANT FOR X-DRIVE EKF
        odom.twist.twist.angular.z = v_th;

        // Covariances
        odom.pose.covariance[0] = 0.01;  // X
        odom.pose.covariance[7] = 0.01;  // Y
        odom.pose.covariance[35] = 0.01; // Yaw
        odom.twist.covariance[0] = 0.01; // Vx
        odom.twist.covariance[7] = 0.01; // Vy
        odom.twist.covariance[35] = 0.01;// V_yaw

        odom_pub_->publish(odom);

        // Update previous values for next iteration
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