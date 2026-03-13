#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

class PidMove : public rclcpp::Node {
public:
    PidMove() : Node("pid_move") {
        Kp_dist_ = 1.0; Kd_dist_ = 0.2; dist_tolerance_ = 0.02; min_move_speed_ = 0.13;
        Kp_angle_ = 0.08; Ki_angle_ = 0.0; Kd_angle_ = 0.0;

        sub_dist_ = this->create_subscription<std_msgs::msg::Float32>(
            "/robot_distance", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { current_dist_ = msg->data; });
        sub_gyro_ = this->create_subscription<std_msgs::msg::Float32>(
            "/gyro_z", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { current_angle_ = msg->data; });
        sub_goal_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/pid_move/goal", 10, std::bind(&PidMove::goal_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pid_move/done", 10);

        imu_client_ = this->create_client<studica_control::srv::SetData>("/imu/get_imu_data");
        titan_client_ = this->create_client<studica_control::srv::SetData>("/titan0/titan_cmd");

        timer_ = this->create_wall_timer(50ms, std::bind(&PidMove::control_loop, this));
        RCLCPP_INFO(this->get_logger(), "C++ PID Move Node Started (Pure Hardware Resets).");
    }

private:
    double Kp_dist_, Kd_dist_, dist_tolerance_, min_move_speed_;
    double Kp_angle_, Ki_angle_, Kd_angle_;
    
    bool active_ = false;
    double target_dist_ = 0.0, target_angle_ = 0.0;
    double current_dist_ = 0.0, current_angle_ = 0.0;
    double prev_dist_error_ = 0.0, prev_angle_error_ = 0.0, angle_integral_ = 0.0;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_dist_, sub_gyro_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr imu_client_, titan_client_;
    rclcpp::TimerBase::SharedPtr timer_, delay_timer_;

    void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Move Goal: Dist=%.2f m, Angle=%.2f deg. Resetting Hardware...", msg->x, msg->y);
        target_dist_ = msg->x;
        target_angle_ = msg->y;
        active_ = false;

        // Reset IMU
        if (imu_client_->service_is_ready()) {
            auto req_imu = std::make_shared<studica_control::srv::SetData::Request>();
            req_imu->params = "reset";
            imu_client_->async_send_request(req_imu);
        }

        // Reset all 4 Encoders physically
        if (titan_client_->service_is_ready()) {
            for (int i = 0; i < 4; i++) {
                auto req_enc = std::make_shared<studica_control::srv::SetData::Request>();
                req_enc->params = "reset_encoder";
                req_enc->initparams.n_encoder = i;
                titan_client_->async_send_request(req_enc);
            }
        }

        // Wait 500ms for hardware to drop to 0.0 before we start doing math
        if (delay_timer_) delay_timer_->cancel();
        delay_timer_ = this->create_wall_timer(500ms, [this]() {
            delay_timer_->cancel();
            prev_dist_error_ = 0.0;
            prev_angle_error_ = 0.0;
            angle_integral_ = 0.0;
            active_ = true;
        });
    }

    void control_loop() {
        if (!active_) return;

        // Drive purely based on the raw distance published by allterrain_node
        double dist_error = target_dist_ - current_dist_;
        double angle_error = target_angle_ - current_angle_;

        if (std::abs(dist_error) < dist_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Move Complete!");
            geometry_msgs::msg::Twist stop_msg;
            cmd_pub_->publish(stop_msg);
            active_ = false;
            
            std_msgs::msg::Bool done_msg;
            done_msg.data = true;
            done_pub_->publish(done_msg);
            return;
        }

        double dist_deriv = dist_error - prev_dist_error_;
        double linear_v = (Kp_dist_ * dist_error) + (Kd_dist_ * dist_deriv);
        
        if (linear_v > 0) linear_v = std::max(linear_v, min_move_speed_);
        else if (linear_v < 0) linear_v = std::min(linear_v, -min_move_speed_);
        linear_v = std::clamp(linear_v, -0.5, 0.5);

        angle_integral_ += angle_error;
        angle_integral_ = std::clamp(angle_integral_, -15.0, 15.0);
        double angle_deriv = angle_error - prev_angle_error_;
        double angular_v = (Kp_angle_ * angle_error) + (Ki_angle_ * angle_integral_) + (Kd_angle_ * angle_deriv);
        angular_v = std::clamp(angular_v, -0.2, 0.2);

        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_v;
        twist.angular.z = angular_v;
        cmd_pub_->publish(twist);

        prev_dist_error_ = dist_error;
        prev_angle_error_ = angle_error;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidMove>());
    rclcpp::shutdown();
    return 0;
}