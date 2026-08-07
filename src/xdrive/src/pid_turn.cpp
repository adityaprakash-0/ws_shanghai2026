#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

class PidTurn : public rclcpp::Node {
public:
    PidTurn() : Node("xdrive_pid_turn") {
        Kp_ = 0.04;
        Kd_ = 0.15;
        tolerance_ = 1.0;
        min_speed_ = 0.13;
        max_speed_ = 0.20;

        active_ = false;

        sub_gyro_ = this->create_subscription<std_msgs::msg::Float32>(
            "/gyro_z", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { current_angle_ = msg->data; });

        sub_goal_ = this->create_subscription<std_msgs::msg::Float32>(
            "/pid_turn/goal", 10, std::bind(&PidTurn::goal_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pid_turn/done", 10);

        imu_client_ = this->create_client<studica_control::srv::SetData>("/imu/get_imu_data");

        timer_ = this->create_wall_timer(50ms, std::bind(&PidTurn::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "X-Drive C++ PID Turn Node Started.");
    }

private:
    double Kp_, Kd_, tolerance_, min_speed_, max_speed_;
    bool active_;
    double target_angle_ = 0.0, current_angle_ = 0.0, prev_error_ = 0.0;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_gyro_, sub_goal_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;

    rclcpp::Client<studica_control::srv::SetData>::SharedPtr imu_client_;
    rclcpp::TimerBase::SharedPtr timer_, delay_timer_;

    double normalize_angle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    void goal_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Turn Goal: %.2f degrees. Resetting IMU...", msg->data);
        target_angle_ = normalize_angle(msg->data);
        active_ = false;

        if (imu_client_->service_is_ready()) {
            auto req = std::make_shared<studica_control::srv::SetData::Request>();
            req->params = "reset";
            imu_client_->async_send_request(req);
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "IMU Reset service unavailable! Check if hardware driver node is running.");
        }

        if (delay_timer_) delay_timer_->cancel();
        delay_timer_ = this->create_wall_timer(500ms, [this]() {
            delay_timer_->cancel();
            prev_error_ = 0.0;
            active_ = true;
        });
    }

    void control_loop() {
        if (!active_) return;

        double error = normalize_angle(target_angle_ - current_angle_);

        if (std::abs(error) < tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Turn Complete!");
            geometry_msgs::msg::Twist stop_msg;
            cmd_pub_->publish(stop_msg);
            active_ = false;

            std_msgs::msg::Bool done_msg;
            done_msg.data = true;
            done_pub_->publish(done_msg);
            return;
        }

        double derivative = normalize_angle(error - prev_error_);
        double output = (Kp_ * error) + (Kd_ * derivative);

        if (output > 0) {
            output = std::clamp(output, min_speed_, max_speed_);
        } else if (output < 0) {
            output = std::clamp(output, -max_speed_, -min_speed_);
        }

        geometry_msgs::msg::Twist twist;
        twist.angular.z = output;
        cmd_pub_->publish(twist);

        prev_error_ = error;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidTurn>());
    rclcpp::shutdown();
    return 0;
}
