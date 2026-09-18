/*
 * light_tower_component.cpp
 *
 * ROS2 component for a 5-pin LED tower (red / green / yellow / buzzer + continuous).
 *
 * Blink modes:
 *   solid    — continuous=HIGH, color=HIGH
 *   blink_hw — continuous=LOW,  color=HIGH  (hardware drives the flash rate)
 *   blink    — continuous=HIGH, driver toggles color pin at default_blink_hz
 *   <hz>     — continuous=HIGH, driver toggles color pin at the given hz
 *
 * Mutual exclusion is enforced in software: activating any output first clears
 * the currently active one.
 *
 * Drive indicator (optional — see follow_cmd_vel):
 *   |cmd_vel| above eps → solid green (moving_color)
 *   otherwise           → solid red   (stopped_color)
 * Explicit SetData (wait_light patterns, dashboard) pauses auto mode for
 * override_hold_sec so flashes are not overwritten mid-pattern.
 *
 * REMOVE / CHANGE LATER:
 *   - hw_params: light_tower.follow_cmd_vel: false   → turn off entirely
 *   - light_tower.moving_color / stopped_color        → pick other colours
 *   - Or delete the cmd_vel subscription block below
 */

#include "studica_control/light_tower_component.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace studica_control {


std::shared_ptr<rclcpp::Node> LightTower::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    control->declare_parameter<std::string>("light_tower.name",          "light_tower");
    control->declare_parameter<int>        ("light_tower.pin_continuous", -1);
    control->declare_parameter<int>        ("light_tower.pin_red",        -1);
    control->declare_parameter<int>        ("light_tower.pin_green",      -1);
    control->declare_parameter<int>        ("light_tower.pin_yellow",     -1);
    control->declare_parameter<int>        ("light_tower.pin_buzzer",     -1);
    control->declare_parameter<double>     ("light_tower.default_blink_hz", 1.0);
    // Drive indicator — set follow_cmd_vel: false in hw_params to disable.
    control->declare_parameter<bool>       ("light_tower.follow_cmd_vel", true);
    control->declare_parameter<std::string>("light_tower.cmd_vel_topic", "/cmd_vel");
    control->declare_parameter<double>     ("light_tower.cmd_vel_eps", 0.02);
    control->declare_parameter<double>     ("light_tower.override_hold_sec", 2.0);
    control->declare_parameter<std::string>("light_tower.moving_color", "green");
    control->declare_parameter<std::string>("light_tower.stopped_color", "red");

    std::string name         = control->get_parameter("light_tower.name").as_string();
    int pin_continuous       = control->get_parameter("light_tower.pin_continuous").as_int();
    int pin_red              = control->get_parameter("light_tower.pin_red").as_int();
    int pin_green            = control->get_parameter("light_tower.pin_green").as_int();
    int pin_yellow           = control->get_parameter("light_tower.pin_yellow").as_int();
    int pin_buzzer           = control->get_parameter("light_tower.pin_buzzer").as_int();
    double default_blink_hz  = control->get_parameter("light_tower.default_blink_hz").as_double();
    bool follow_cmd_vel      = control->get_parameter("light_tower.follow_cmd_vel").as_bool();
    std::string cmd_vel_topic = control->get_parameter("light_tower.cmd_vel_topic").as_string();
    double cmd_vel_eps       = control->get_parameter("light_tower.cmd_vel_eps").as_double();
    double override_hold_sec = control->get_parameter("light_tower.override_hold_sec").as_double();
    std::string moving_color = control->get_parameter("light_tower.moving_color").as_string();
    std::string stopped_color = control->get_parameter("light_tower.stopped_color").as_string();

    return std::make_shared<LightTower>(vmx, name,
                                        pin_continuous, pin_red, pin_green,
                                        pin_yellow, pin_buzzer, default_blink_hz,
                                        follow_cmd_vel, cmd_vel_topic, cmd_vel_eps,
                                        override_hold_sec, moving_color, stopped_color);
}


LightTower::LightTower(const rclcpp::NodeOptions &options)
    : rclcpp::Node("light_tower", options) {}


LightTower::LightTower(std::shared_ptr<VMXPi> vmx,
                       const std::string &name,
                       int pin_continuous,
                       int pin_red,
                       int pin_green,
                       int pin_yellow,
                       int pin_buzzer,
                       double default_blink_hz,
                       bool follow_cmd_vel,
                       const std::string &cmd_vel_topic,
                       double cmd_vel_eps,
                       double override_hold_sec,
                       const std::string &moving_color,
                       const std::string &stopped_color)
    : rclcpp::Node(name)
    , default_blink_hz_(default_blink_hz)
    , follow_cmd_vel_(follow_cmd_vel)
    , cmd_vel_eps_(std::max(0.0, cmd_vel_eps))
    , override_hold_sec_(std::max(0.0, override_hold_sec))
    , moving_color_(moving_color.empty() ? "green" : moving_color)
    , stopped_color_(stopped_color.empty() ? "red" : stopped_color)
{
    tower_ = std::make_shared<studica_driver::LightTower>(
        static_cast<VMXChannelIndex>(pin_continuous),
        static_cast<VMXChannelIndex>(pin_red),
        static_cast<VMXChannelIndex>(pin_green),
        static_cast<VMXChannelIndex>(pin_yellow),
        static_cast<VMXChannelIndex>(pin_buzzer),
        vmx);

    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/set",
        std::bind(&LightTower::cmd_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    state_pub_ = this->create_publisher<std_msgs::msg::String>(name + "/state", 10);

    // 1 hz heartbeat so late subscribers always see the current state
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&LightTower::publish_state, this));

    if (follow_cmd_vel_) {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 10,
            std::bind(&LightTower::on_cmd_vel, this, std::placeholders::_1));
        // Start in "stopped" (red) so operators see the tower is live.
        set_output(stopped_color_, 0.0);
        RCLCPP_INFO(this->get_logger(),
                    "cmd_vel drive indicator ON topic=%s eps=%.3f "
                    "moving=%s stopped=%s (set follow_cmd_vel:=false to disable)",
                    cmd_vel_topic.c_str(), cmd_vel_eps_,
                    moving_color_.c_str(), stopped_color_.c_str());
    }

    RCLCPP_INFO(this->get_logger(),
                "light_tower ready. pins: continuous=%d  red=%d  green=%d  yellow=%d  buzzer=%d  "
                "default_blink_hz=%.1f",
                pin_continuous, pin_red, pin_green, pin_yellow, pin_buzzer, default_blink_hz);
}

LightTower::~LightTower()
{
    cancel_blink();
    tower_->AllOff();
}


void LightTower::cmd_callback(
    const std::shared_ptr<studica_control::srv::SetData::Request>  request,
          std::shared_ptr<studica_control::srv::SetData::Response> response)
{
    const std::string &params = request->params;

    // split "color" or "color:mode_or_hz"
    std::string color;
    std::string mode_str;
    auto sep = params.find(':');
    if (sep == std::string::npos) {
        color    = params;
        mode_str = "";
    } else {
        color    = params.substr(0, sep);
        mode_str = params.substr(sep + 1);
    }

    // validate color
    if (color != "off" && color != "red" && color != "green" &&
        color != "yellow" && color != "buzzer") {
        response->success = false;
        response->message = "unknown color '" + color + "' — use: red, green, yellow, buzzer, off";
        return;
    }

    // resolve blink_hz: >0 = sw blink, 0 = solid, <0 = hw blink
    double blink_hz = 0.0;
    if (!mode_str.empty()) {
        if (mode_str == "blink") {
            blink_hz = default_blink_hz_;
        } else if (mode_str == "blink_hw") {
            blink_hz = -1.0;
        } else {
            try {
                blink_hz = std::stod(mode_str);
            } catch (const std::exception &) {
                response->success = false;
                response->message = "invalid mode '" + mode_str
                                  + "' — use: blink, blink_hw, or a numeric hz";
                return;
            }
        }
    }

    // Pause cmd_vel auto colours so wait_light / dashboard flashes win.
    if (follow_cmd_vel_) {
        override_until_ = this->now() + rclcpp::Duration::from_seconds(override_hold_sec_);
        have_motion_state_ = false;  // force re-apply after hold expires
    }

    set_output(color, blink_hz);

    response->success = true;
    response->message = state_string();
    RCLCPP_INFO(this->get_logger(), "%s", state_string().c_str());
}


bool LightTower::override_active() const
{
    return this->now() < override_until_;
}


void LightTower::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!follow_cmd_vel_ || override_active()) {
        return;
    }
    const double mag =
        std::fabs(msg->linear.x) + std::fabs(msg->linear.y) +
        std::fabs(msg->linear.z) + std::fabs(msg->angular.x) +
        std::fabs(msg->angular.y) + std::fabs(msg->angular.z);
    const bool moving = mag > cmd_vel_eps_;
    if (have_motion_state_ && moving == last_moving_) {
        return;
    }
    last_moving_ = moving;
    have_motion_state_ = true;
    // blink_hz = 0 → solid
    set_output(moving ? moving_color_ : stopped_color_, 0.0);
}


void LightTower::set_output(const std::string &color, double blink_hz)
{
    clear_active();
    cancel_blink();

    if (color == "off") {
        active_color_ = "none";
        blink_hz_     = 0.0;
        tower_->AllOff();
        publish_state();
        return;
    }

    active_color_ = color;
    blink_hz_     = blink_hz;

    if (blink_hz > 0.0) {
        // software blink — continuous HIGH, driver toggles the colour pin
        tower_->SetContinuous(true);
        blink_state_ = true;
        set_pin(color, true);
        auto period_ns = static_cast<int64_t>(1e9 / blink_hz);
        blink_timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(period_ns),
            [this, color]() {
                blink_state_ = !blink_state_;
                set_pin(color, blink_state_);
            });
    } else if (blink_hz < 0.0) {
        // hardware blink — continuous LOW, hardware drives the flash rate
        tower_->SetContinuous(false);
        set_pin(color, true);
    } else {
        // solid — continuous HIGH
        tower_->SetContinuous(true);
        set_pin(color, true);
    }

    publish_state();
}


void LightTower::set_pin(const std::string &color, bool value)
{
    if      (color == "red")    tower_->SetRed(value);
    else if (color == "green")  tower_->SetGreen(value);
    else if (color == "yellow") tower_->SetYellow(value);
    else if (color == "buzzer") tower_->SetBuzzer(value);
}


void LightTower::clear_active()
{
    if (active_color_ != "none") {
        set_pin(active_color_, false);
        tower_->SetContinuous(false);
    }
}


void LightTower::cancel_blink()
{
    if (blink_timer_) {
        blink_timer_->cancel();
        blink_timer_.reset();
    }
}


std::string LightTower::state_string() const
{
    if (active_color_ == "none") return "off";
    if (blink_hz_ > 0.0)        return active_color_ + ":blink:" + std::to_string(blink_hz_);
    if (blink_hz_ < 0.0)        return active_color_ + ":blink_hw";
    return active_color_ + ":solid";
}


void LightTower::publish_state()
{
    std_msgs::msg::String msg;
    msg.data = state_string();
    state_pub_->publish(msg);
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::LightTower)
