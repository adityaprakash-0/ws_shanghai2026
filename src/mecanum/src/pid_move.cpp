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
    PidMove() : Node("mecanum_pid_move") {
        // --- Mecanum Tuning ---
        Kp_dist_ = 1.0; Kd_dist_ = 0.2; dist_tolerance_ = 0.02; min_move_speed_ = 0.13;
        Kp_angle_ = 0.08; Ki_angle_ = 0.0; Kd_angle_ = 0.0;
        max_move_speed_ = 0.20;

        active_ = false;

        // Subscribing to X and Y distances calculated by mecanum_node
        sub_dist_x_ = this->create_subscription<std_msgs::msg::Float32>(
            "/robot_distance_x", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { current_dist_x_ = msg->data; });
        sub_dist_y_ = this->create_subscription<std_msgs::msg::Float32>(
            "/robot_distance_y", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { current_dist_y_ = msg->data; });
        
        sub_gyro_ = this->create_subscription<std_msgs::msg::Float32>(
            "/gyro_z", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { current_angle_ = msg->data; });
        
        sub_goal_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/pid_move/goal", 10, std::bind(&PidMove::goal_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pid_move/done", 10);

        // Hardware reset clients (Updated to match your actual namespaces)
        imu_client_ = this->create_client<studica_control::srv::SetData>("/imu/get_imu_data");
        titan_client_ = this->create_client<studica_control::srv::SetData>("/titan0/titan_cmd");
        light_client_ = this->create_client<studica_control::srv::SetData>("/light_tower/set");

        timer_ = this->create_wall_timer(50ms, std::bind(&PidMove::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Mecanum C++ PID Move Node Started (Pure Hardware Resets).");
    }

private:
    double Kp_dist_, Kd_dist_, dist_tolerance_, min_move_speed_, max_move_speed_;
    double Kp_angle_, Ki_angle_, Kd_angle_;
    
    bool active_;
    double target_dist_ = 0.0, target_angle_ = 0.0;
    int move_mode_ = 0; // 0 = X (Forward/Back), 1 = Y (Strafe)
    
    double current_dist_x_ = 0.0, current_dist_y_ = 0.0, current_angle_ = 0.0;
    double prev_error_dist_ = 0.0, prev_angle_error_ = 0.0, angle_integral_ = 0.0;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_dist_x_, sub_dist_y_, sub_gyro_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
    
    rclcpp::Client<studica_control::srv::SetData>::SharedPtr imu_client_, titan_client_, light_client_;
    rclcpp::TimerBase::SharedPtr timer_, delay_timer_;

    void set_light(const std::string & color)
    {
        if (!light_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "light_tower service not ready");
            return;
        }
        auto req = std::make_shared<studica_control::srv::SetData::Request>();
        req->params = color;
        light_client_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "Light tower → %s", color.c_str());
    }

    void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        std::string axis_str = (msg->y == 0.0) ? "X (Forward/Back)" : "Y (Strafing)";
        RCLCPP_INFO(this->get_logger(), "Move Goal: Dist=%.2f m, Axis=%s, Angle=%.2f deg. Resetting Hardware...", msg->x, axis_str.c_str(), msg->z);

        target_dist_ = msg->x;
        move_mode_ = static_cast<int>(msg->y);
        target_angle_ = msg->z;
        active_ = false;

        // 1. Reset IMU
        if (imu_client_->service_is_ready()) {
            auto req_imu = std::make_shared<studica_control::srv::SetData::Request>();
            req_imu->params = "reset";
            imu_client_->async_send_request(req_imu);
            RCLCPP_INFO(this->get_logger(), "IMU reset requested");
        } else {
            RCLCPP_WARN(this->get_logger(), "IMU Reset service unavailable!");
        }

        // 2. Reset all 4 Encoders physically (Using exact "reset_encoder" command)
        if (titan_client_->service_is_ready()) {
            for (int i = 0; i < 4; i++) {
                auto req_enc = std::make_shared<studica_control::srv::SetData::Request>();
                req_enc->params = "reset_encoder";
                req_enc->initparams.n_encoder = i;
                titan_client_->async_send_request(req_enc);
            }
            RCLCPP_INFO(this->get_logger(), "Encoder reset requested on m_0..m_3");
        } else {
            RCLCPP_WARN(this->get_logger(), "Titan Encoder Reset service unavailable!");
        }

        // 3. Wait 500ms for hardware to drop to 0.0 before we start doing math
        if (delay_timer_) {
            delay_timer_->cancel();
            delay_timer_.reset();
        }
        delay_timer_ = this->create_wall_timer(500ms, [this]() {
            if (delay_timer_) {
                delay_timer_->cancel();
                delay_timer_.reset();
            }
            prev_error_dist_ = 0.0;
            prev_angle_error_ = 0.0;
            angle_integral_ = 0.0;
            active_ = true;
            set_light("green");
            RCLCPP_INFO(
                this->get_logger(),
                "PID ACTIVE — target=%.2f m  dist_x=%.3f  dist_y=%.3f  gyro_z=%.1f",
                target_dist_, current_dist_x_, current_dist_y_, current_angle_);
        });
    }

    void control_loop() {
        if (!active_) return;

        // Select the correct distance axis to track
        double error_dist = (move_mode_ == 0) ? (target_dist_ - current_dist_x_) : (target_dist_ - current_dist_y_);
        double angle_error = target_angle_ - current_angle_;

        if (std::abs(error_dist) < dist_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Move Complete! final dist_x=%.3f dist_y=%.3f",
                        current_dist_x_, current_dist_y_);
            geometry_msgs::msg::Twist stop_msg;
            cmd_pub_->publish(stop_msg);
            active_ = false;
            set_light("red");

            std_msgs::msg::Bool done_msg;
            done_msg.data = true;
            done_pub_->publish(done_msg);
            return;
        }

        // Translation PID
        double dist_deriv = error_dist - prev_error_dist_;
        double linear_v = (Kp_dist_ * error_dist) + (Kd_dist_ * dist_deriv);

        if (linear_v > 0) {
            linear_v = std::clamp(linear_v, min_move_speed_, max_move_speed_);
        } else if (linear_v < 0) {
            linear_v = std::clamp(linear_v, -max_move_speed_, -min_move_speed_);
        }

        // Route velocity to the correct axis
        double linear_v_x = (move_mode_ == 0) ? linear_v : 0.0;
        double linear_v_y = (move_mode_ == 1) ? linear_v : 0.0;

        // Rotation PID (Heading Lock)
        angle_integral_ += angle_error;
        angle_integral_ = std::clamp(angle_integral_, -15.0, 15.0);
        double angle_deriv = angle_error - prev_angle_error_;

        double angular_v = (Kp_angle_ * angle_error) + (Ki_angle_ * angle_integral_) + (Kd_angle_ * angle_deriv);
        angular_v = std::clamp(angular_v, -0.2, 0.2);

        // Send Command
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_v_x;
        twist.linear.y = linear_v_y; // Applies strafing if in mode 1
        twist.angular.z = angular_v;
        cmd_pub_->publish(twist);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "driving vx=%.2f vy=%.2f wz=%.2f | err=%.3f dist_x=%.3f dist_y=%.3f",
            linear_v_x, linear_v_y, angular_v, error_dist, current_dist_x_, current_dist_y_);

        prev_error_dist_ = error_dist;
        prev_angle_error_ = angle_error;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidMove>());
    rclcpp::shutdown();
    return 0;
}