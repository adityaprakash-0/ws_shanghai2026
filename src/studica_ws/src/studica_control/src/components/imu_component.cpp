// /*
//  * imu_component.cpp
//  *
//  * ros2 component for the onboard imu (inertial measurement unit).
//  * reads orientation, rotation rate, and linear acceleration from the
//  * navx sensor built into the vmx-pi board.
//  *
//  * topic:   <topic> (sensor_msgs/Imu) — published at 20hz
//  *            orientation as a quaternion (x, y, z, w)
//  *            angular velocity in radians per second (x, y, z)
//  *            linear acceleration in meters per second squared (x, y, z)
//  *
//  * service: get_imu_data (studica_control/SetData)
//  *   returns the current pitch, yaw, and roll as a string
//  *   (no specific params field needed — any call returns the values)
//  */

#include "studica_control/imu_component.hpp"
#include <cmath>
#include "std_msgs/msg/float32.hpp"

namespace studica_control {

// Reads IMU parameters from params.yaml and creates the IMU node
std::shared_ptr<rclcpp::Node> Imu::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    control->declare_parameter<std::string>("imu.name");
    control->declare_parameter<std::string>("imu.topic");
    control->declare_parameter<std::string>("imu.frame_id", "imu_link");

    std::string name     = control->get_parameter("imu.name").as_string();
    std::string topic    = control->get_parameter("imu.topic").as_string();
    std::string frame_id = control->get_parameter("imu.frame_id").as_string();

    return std::make_shared<Imu>(vmx, name, topic, frame_id);
}

// Composable node constructor
Imu::Imu(const rclcpp::NodeOptions &options) : Node("imu", options) {}

// Main constructor
Imu::Imu(std::shared_ptr<VMXPi> vmx, const std::string &name, const std::string &topic, const std::string &frame_id)
    : rclcpp::Node(name), vmx_(vmx), frame_id_(frame_id) {

    imu_ = std::make_shared<studica_driver::Imu>(vmx_);
    imu_->ZeroYaw();

    // Service for reading pitch, yaw, and roll on demand
    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/get_imu_data",
        std::bind(&Imu::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Publishes full IMU data at 20Hz
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
    
    // Gyro Publishers
    pub_gyro_x_ = this->create_publisher<std_msgs::msg::Float32>("/gyro_x", 10);
    pub_gyro_y_ = this->create_publisher<std_msgs::msg::Float32>("/gyro_y", 10);
    pub_gyro_z_ = this->create_publisher<std_msgs::msg::Float32>("/gyro_z", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Imu::publish_data, this));

    RCLCPP_INFO(this->get_logger(), "IMU component ready.");
}

Imu::~Imu() {}

void Imu::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                       std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (!imu_) {
        RCLCPP_WARN(this->get_logger(), "IMU is not available.");
        response->success = false;
        response->message = "IMU not available.";
        return;
    }

    try {
        if (request->params == "zero_yaw" || request->params == "reset") {
            imu_->ZeroYaw();
            response->success = true;
            response->message = "IMU Yaw Reset to Zero.";
            RCLCPP_INFO(this->get_logger(), "IMU Yaw Reset Command Executed.");
        } else {
            // Calculate Euler Angles for the service response
            float ros_roll  = -imu_->GetPitch();      
            float ros_pitch = imu_->GetRoll();        
            float ros_yaw   = -imu_->GetYaw(); // Converts NavX CW+ to ROS CCW+

            response->success = true;
            response->message = "Pitch: " + std::to_string(ros_pitch) + ", "
                              + "Yaw: " + std::to_string(ros_yaw) + ", "
                              + "Roll: " + std::to_string(ros_roll) + ".";
        }
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "Failed to get IMU data: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Failed to get IMU data: %s", e.what());
    }
}

void Imu::publish_data() {
    rclcpp::Time current_time = this->get_clock()->now();

    // ---------------------------------------------------------
    // 1. CALCULATE FIXED EULER ANGLES (Degrees)
    // ---------------------------------------------------------
    float imu_pitch = imu_->GetPitch();
    float imu_yaw   = imu_->GetYaw();
    float imu_roll  = imu_->GetRoll();

    // Apply mounting offset (ROS X = NavX -Y, ROS Y = NavX +X)
    float ros_roll  = -imu_pitch;      
    float ros_pitch = imu_roll;        
    
    // Invert Yaw (NavX natively reports Clockwise-Positive. ROS requires Counter-Clockwise-Positive)
    float ros_yaw   = -imu_yaw; 

    // Publish to Float32 topics for your PID controllers
    std_msgs::msg::Float32 msg_x, msg_y, msg_z;
    msg_x.data = ros_roll;
    msg_y.data = ros_pitch;
    msg_z.data = ros_yaw;

    pub_gyro_x_->publish(msg_x);
    pub_gyro_y_->publish(msg_y);
    pub_gyro_z_->publish(msg_z);

    // -----------------------------------f----------------------
    // 2. CONVERT FIXED EULER ANGLES TO QUATERNION (Radians)
    // ---------------------------------------------------------
    // This perfectly synchronizes your /gyro_z topic with the /imu topic
    double roll_rad  = ros_roll  * (M_PI / 180.0);
    double pitch_rad = ros_pitch * (M_PI / 180.0);
    double yaw_rad   = ros_yaw   * (M_PI / 180.0);

    double cr = std::cos(roll_rad * 0.5);
    double sr = std::sin(roll_rad * 0.5);
    double cp = std::cos(pitch_rad * 0.5);
    double sp = std::sin(pitch_rad * 0.5);
    double cy = std::cos(yaw_rad * 0.5);
    double sy = std::sin(yaw_rad * 0.5);

    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = current_time;
    msg.header.frame_id = frame_id_;

    msg.orientation.w = cr * cp * cy + sr * sp * sy;
    msg.orientation.x = sr * cp * cy - cr * sp * sy;
    msg.orientation.y = cr * sp * cy + sr * cp * sy;
    msg.orientation.z = cr * cp * sy - sr * sp * cy;

    // ---------------------------------------------------------
    // 3. GYRO & ACCEL VECTORS (Mapped to match Euler orientation)
    // ---------------------------------------------------------
    msg.angular_velocity.x = -imu_->GetRawGyroY() * (M_PI / 180.0);
    msg.angular_velocity.y =  imu_->GetRawGyroX() * (M_PI / 180.0);
    msg.angular_velocity.z =  imu_->GetRawGyroZ() * (M_PI / 180.0); // Flipped for CCW+

    msg.linear_acceleration.x = -imu_->GetRawAccelY() * 9.80665;
    msg.linear_acceleration.y =  imu_->GetRawAccelX() * 9.80665;
    msg.linear_acceleration.z =  imu_->GetRawAccelZ() * 9.80665; // Gravity remains down (Z+ up)

    msg.orientation_covariance = {1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 0.0005};
    msg.angular_velocity_covariance = {1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 0.0005};
    msg.linear_acceleration_covariance = {0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5};

    publisher_->publish(msg);
}

void Imu::DisplayVMXError(VMXErrorCode vmxerr) {
    printf("VMX Error %d: %s\n", vmxerr, GetVMXErrorString(vmxerr));
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Imu)