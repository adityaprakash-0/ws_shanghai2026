/*
 * light_tower_component.hpp
 *
 * ROS2 component for a 5-output LED light tower (red/green/yellow/buzzer + continuous).
 * Enforces mutual exclusion — activating a new colour/buzzer automatically clears
 * the current one.
 *
 * service: <name>/set  (studica_control/SetData)
 *   params:
 *     "off"              — everything off
 *     "<color>"          — solid on  (colors: red, green, yellow, buzzer)
 *     "<color>:blink"    — software blink at default_blink_hz (from params.yaml)
 *     "<color>:blink_hw" — hardware blink (continuous pin LOW, hardware drives rate)
 *     "<color>:<hz>"     — software blink at a specific hz (e.g. "red:2.5")
 *
 * topic (publishes): <name>/state (std_msgs/String)
 *   current state string, e.g. "off", "red:solid", "green:blink:1.0", "yellow:blink_hw"
 *   published on every state change and at 1 hz as a heartbeat.
 *
 * Optional cmd_vel drive indicator (params under light_tower.*):
 *   follow_cmd_vel (bool, default true)
 *     true  → solid green while |cmd_vel| > threshold, solid red when stopped
 *     false → disable this behaviour (SetData / wait_light patterns still work)
 *   To remove later: set follow_cmd_vel: false in hw_params, or change
 *   moving_color / stopped_color (e.g. yellow / off).
 */

#ifndef LIGHT_TOWER_COMPONENT_HPP
#define LIGHT_TOWER_COMPONENT_HPP

#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "light_tower.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class LightTower : public rclcpp::Node {
public:
    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    explicit LightTower(const rclcpp::NodeOptions &options);

    LightTower(std::shared_ptr<VMXPi> vmx,
               const std::string &name,
               int pin_continuous,
               int pin_red,
               int pin_green,
               int pin_yellow,
               int pin_buzzer,
               double default_blink_hz,
               bool follow_cmd_vel = true,
               const std::string &cmd_vel_topic = "/cmd_vel",
               double cmd_vel_eps = 0.02,
               double override_hold_sec = 2.0,
               const std::string &moving_color = "green",
               const std::string &stopped_color = "red");

    ~LightTower();

private:
    std::shared_ptr<studica_driver::LightTower> tower_;

    std::string active_color_{"none"};
    double      blink_hz_{0.0};
    bool        blink_state_{false};
    double      default_blink_hz_{1.0};

    // --- cmd_vel → green/red (disable via follow_cmd_vel:=false) ---
    bool        follow_cmd_vel_{true};
    double      cmd_vel_eps_{0.02};
    double      override_hold_sec_{2.0};
    std::string moving_color_{"green"};
    std::string stopped_color_{"red"};
    bool        last_moving_{false};
    bool        have_motion_state_{false};
    rclcpp::Time override_until_{0, 0, RCL_ROS_TIME};

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       state_pub_;
    rclcpp::TimerBase::SharedPtr                              heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr                              blink_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request>  request,
                            std::shared_ptr<studica_control::srv::SetData::Response> response);

    void        set_output(const std::string &color, double blink_hz);
    void        set_pin(const std::string &color, bool value);
    void        clear_active();
    void        cancel_blink();
    std::string state_string() const;
    void        publish_state();
    void        on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
    bool        override_active() const;
};

} // namespace studica_control

#endif // LIGHT_TOWER_COMPONENT_HPP
