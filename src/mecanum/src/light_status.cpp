/**
 * Temporary mecanum light-tower status:
 *   moving  → solid green
 *   stopped → solid red
 *
 * Watches /cmd_vel and also re-asserts color on a timer (helps when the
 * light_tower service appears late under sudo/DDS).
 */

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "studica_control/srv/set_data.hpp"

using namespace std::chrono_literals;

class LightStatus : public rclcpp::Node
{
public:
  LightStatus()
  : Node("mecanum_light_status")
  {
    declare_parameter<double>("vel_eps", 0.02);
    declare_parameter<std::string>("service", "/light_tower/set");
    vel_eps_ = get_parameter("vel_eps").as_double();
    service_name_ = get_parameter("service").as_string();

    client_ = create_client<studica_control::srv::SetData>(service_name_);

    create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&LightStatus::on_cmd, this, std::placeholders::_1));

    // Discover service + keep color asserted (sudo/DDS can be late).
    create_wall_timer(500ms, std::bind(&LightStatus::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "mecanum_light_status: green=moving, red=stopped  service=%s",
      service_name_.c_str());
  }

private:
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = now();
    const bool moving =
      std::abs(msg->linear.x) > vel_eps_ ||
      std::abs(msg->linear.y) > vel_eps_ ||
      std::abs(msg->angular.z) > vel_eps_;

    if (moving == moving_) {
      return;
    }
    moving_ = moving;
    desired_ = moving_ ? "green" : "red";
    dirty_ = true;
    RCLCPP_INFO(
      get_logger(), "Light status: %s → %s",
      moving_ ? "moving" : "stopped", desired_.c_str());
  }

  void tick()
  {
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for %s … (is control_server/light_tower up?)",
        service_name_.c_str());
      return;
    }

    if (!started_) {
      started_ = true;
      desired_ = "red";
      dirty_ = true;
      RCLCPP_INFO(get_logger(), "Connected to %s — setting red (stopped)",
        service_name_.c_str());
    }

    // Re-assert every ~2 s in case a prior call was dropped.
    const double t = now().seconds();
    if (dirty_ || (t - last_sent_sec_) >= 2.0) {
      set_tower(desired_);
      dirty_ = false;
      last_sent_sec_ = t;
    }
  }

  void set_tower(const std::string & cmd)
  {
    if (!client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<studica_control::srv::SetData::Request>();
    req->params = cmd;
    auto future = client_->async_send_request(
      req,
      [this, cmd](rclcpp::Client<studica_control::srv::SetData>::SharedFuture fut) {
        try {
          auto resp = fut.get();
          if (!resp->success) {
            RCLCPP_WARN(get_logger(), "light_tower set '%s' failed: %s",
              cmd.c_str(), resp->message.c_str());
          } else {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "light_tower OK: %s", resp->message.c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "light_tower set '%s' exception: %s",
            cmd.c_str(), e.what());
        }
      });
    (void)future;
  }

  double vel_eps_{0.02};
  std::string service_name_{"/light_tower/set"};
  bool moving_{false};
  bool started_{false};
  bool dirty_{true};
  std::string desired_{"red"};
  double last_sent_sec_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Client<studica_control::srv::SetData>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LightStatus>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
