#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace skill23_nav_guard
{

struct PulseState
{
  double active_until{0.0};
  double cooldown_until{0.0};
  std::string trigger_sensor;
  double trigger_range{0.0};
};

class WheelSafetyGuardNode : public rclcpp::Node
{
public:
  WheelSafetyGuardNode()
  : Node("wheel_safety_guard")
  {
    auto in_topic = declare_parameter<std::string>("input_topic", "/cmd_vel_nav_smoothed");
    auto out_topic = declare_parameter<std::string>("output_topic", "/cmd_vel");
    if (!in_topic.empty() && in_topic.front() != '/') {
      in_topic = "/" + in_topic;
    }
    if (!out_topic.empty() && out_topic.front() != '/') {
      out_topic = "/" + out_topic;
    }
    in_topic_ = in_topic;
    out_topic_ = out_topic;

    const auto us_fl_topic = declare_parameter<std::string>("ultrasonic_fl_topic", "/ultrasonic_fl/range");
    const auto us_rl_topic = declare_parameter<std::string>("ultrasonic_rl_topic", "/ultrasonic_rl/range");
    const auto ir_fr_topic = declare_parameter<std::string>("ir_fr_topic", "/ir_fr/range");
    const auto ir_rr_topic = declare_parameter<std::string>("ir_rr_topic", "/ir_rr/range");

    danger_m_ = declare_parameter<double>("danger_distance_m", 0.15);
    safe_m_ = declare_parameter<double>("safe_distance_m", 0.18);
    push_y_ = declare_parameter<double>("push_linear_y", 0.08);
    pulse_s_ = declare_parameter<double>("pulse_duration_s", 0.12);
    cooldown_s_ = declare_parameter<double>("cooldown_s", 0.40);
    const double rate_hz = std::max(5.0, declare_parameter<double>("control_rate_hz", 10.0));

    rclcpp::QoS cmd_qos(10);
    cmd_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      in_topic_, cmd_qos,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        nav_cmd_ = *msg;
        nav_cmd_seen_ = true;
      });

    bind_range_sub(us_fl_topic, ranges_.fl, "ultrasonic_fl", false);
    bind_range_sub(us_rl_topic, ranges_.rl, "ultrasonic_rl", false);
    bind_range_sub(ir_fr_topic, ranges_.fr, "ir_fr", true);
    bind_range_sub(ir_rr_topic, ranges_.rr, "ir_rr", true);

    rclcpp::QoS pub_qos(10);
    pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    pub_ = create_publisher<geometry_msgs::msg::Twist>(out_topic_, pub_qos);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&WheelSafetyGuardNode::tick, this));

    diag_timer_ = create_wall_timer(
      std::chrono::seconds(8),
      std::bind(&WheelSafetyGuardNode::log_diagnostics, this));

    RCLCPP_INFO(
      get_logger(),
      "Wheel safety guard: SUB %s -> PUB %s | danger=%.2fm push_y=%.2f pulse=%.2fs",
      in_topic_.c_str(), out_topic_.c_str(), danger_m_, push_y_, pulse_s_);
    RCLCPP_INFO(
      get_logger(),
      "Sensors: %s %s %s %s",
      us_fl_topic.c_str(), ir_fr_topic.c_str(), us_rl_topic.c_str(), ir_rr_topic.c_str());
  }

private:
  struct RangeSlot
  {
    double value{std::numeric_limits<double>::infinity()};
    bool seen{false};
    bool ir_blind{false};  // IR reported inf — wall closer than sensor min range
  };

  void bind_range_sub(const std::string & topic, RangeSlot & slot, const char * label, bool is_ir)
  {
    auto cb = [this, &slot, label, is_ir](const sensor_msgs::msg::Range::SharedPtr msg) {
      slot.seen = true;
      slot.ir_blind = false;
      slot.value = valid_range(*msg, is_ir, slot.ir_blind);
      (void)label;
    };
    // Match studica_control publishers (RELIABLE default QoS).
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    range_subs_.push_back(create_subscription<sensor_msgs::msg::Range>(topic, qos, cb));
  }

  // Ultrasonic: inf = no echo. IR: inf or below min = wall too close (GP2Y blind zone).
  static double valid_range(const sensor_msgs::msg::Range & msg, bool is_ir, bool & ir_blind)
  {
    ir_blind = false;
    if (!std::isfinite(msg.range)) {
      if (is_ir) {
        ir_blind = true;
        return 0.0;
      }
      return std::numeric_limits<double>::infinity();
    }
    if (msg.range > msg.max_range) {
      return std::numeric_limits<double>::infinity();
    }
    if (msg.range < msg.min_range) {
      if (is_ir) {
        ir_blind = true;
      }
      return 0.0;
    }
    return static_cast<double>(msg.range);
  }

  void maybe_start_pulse(
    const char * sensor, double range, bool left_side, PulseState & pulse, double now)
  {
    if (now < pulse.cooldown_until || now < pulse.active_until) {
      return;
    }
    if (range >= danger_m_) {
      return;
    }

    pulse.active_until = now + pulse_s_;
    pulse.cooldown_until = now + pulse_s_ + cooldown_s_;
    pulse.trigger_sensor = sensor;
    pulse.trigger_range = range;

    const char * push_dir = left_side ? "RIGHT (-linear.y)" : "LEFT (+linear.y)";
    if (range < 1e-6 && (std::string(sensor).rfind("ir_", 0) == 0)) {
      RCLCPP_WARN(
        get_logger(),
        "DANGER sensor=%s distance=inf (IR blind — wall too close) — pushing %s",
        sensor, push_dir);
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "DANGER sensor=%s distance=%.3fm < threshold=%.3fm — pushing %s",
      sensor, range, danger_m_, push_dir);
  }

  void tick()
  {
    const double now = now_monotonic();

    if (ranges_.fl.value > safe_m_ && ranges_.rl.value > safe_m_) {
      left_pulse_.active_until = 0.0;
    }
    if (ranges_.fr.value > safe_m_ && ranges_.rr.value > safe_m_) {
      right_pulse_.active_until = 0.0;
    }

    maybe_start_pulse("ultrasonic_fl", ranges_.fl.value, true, left_pulse_, now);
    maybe_start_pulse("ultrasonic_rl", ranges_.rl.value, true, left_pulse_, now);
    maybe_start_pulse("ir_fr", ranges_.fr.value, false, right_pulse_, now);
    maybe_start_pulse("ir_rr", ranges_.rr.value, false, right_pulse_, now);

    geometry_msgs::msg::Twist out = nav_cmd_;
    double push_y = 0.0;
    const bool left_active = now < left_pulse_.active_until;
    const bool right_active = now < right_pulse_.active_until;

    if (left_active) {
      push_y -= push_y_;
    }
    if (right_active) {
      push_y += push_y_;
    }

    const bool pushing = std::abs(push_y) > 1e-6;
    if (pushing) {
      const double cap = std::max(0.12, std::abs(push_y_));
      out.linear.y = std::clamp(out.linear.y + push_y, -cap, cap);

      if (!was_pushing_) {
        RCLCPP_INFO(
          get_logger(),
          "Pushed successfully -> %s linear.y=%.3f linear.x=%.3f (trigger=%s was %.3fm)",
          out_topic_.c_str(), out.linear.y, out.linear.x,
          left_active ? left_pulse_.trigger_sensor.c_str() : right_pulse_.trigger_sensor.c_str(),
          left_active ? left_pulse_.trigger_range : right_pulse_.trigger_range);
      }
    }
    was_pushing_ = pushing;

    // Always relay Nav2 velocity; mecanum_node listens on /cmd_vel.
    pub_->publish(out);
  }

  void log_diagnostics()
  {
    auto fmt = [](const RangeSlot & s) {
      if (!s.seen) {
        return std::string("NO_DATA");
      }
      if (s.ir_blind) {
        return std::string("TOO_CLOSE(inf)");
      }
      if (!std::isfinite(s.value)) {
        return std::string("clear");
      }
      char buf[16];
      std::snprintf(buf, sizeof(buf), "%.3fm", s.value);
      return std::string(buf);
    };

    RCLCPP_INFO(
      get_logger(),
      "Status nav_in=%s | FL=%s FR=%s RL=%s RR=%s | pub=%s",
      nav_cmd_seen_ ? "ok" : "waiting",
      fmt(ranges_.fl).c_str(), fmt(ranges_.fr).c_str(),
      fmt(ranges_.rl).c_str(), fmt(ranges_.rr).c_str(),
      out_topic_.c_str());

    if (!ranges_.fl.seen && !ranges_.fr.seen && !ranges_.rl.seen && !ranges_.rr.seen) {
      RCLCPP_ERROR(
        get_logger(),
        "No wheel sensor data received — check topics "
        "/ultrasonic_fl/range /ir_fr/range /ultrasonic_rl/range /ir_rr/range");
    }
  }

  static double now_monotonic()
  {
    const auto t = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration<double>(t).count();
  }

  struct RangeState
  {
    RangeSlot fl;
    RangeSlot fr;
    RangeSlot rl;
    RangeSlot rr;
  };

  std::string in_topic_;
  std::string out_topic_;
  geometry_msgs::msg::Twist nav_cmd_;
  RangeState ranges_;
  PulseState left_pulse_;
  PulseState right_pulse_;
  bool nav_cmd_seen_{false};
  bool was_pushing_{false};

  double danger_m_{0.15};
  double safe_m_{0.18};
  double push_y_{0.08};
  double pulse_s_{0.12};
  double cooldown_s_{0.40};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> range_subs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace skill23_nav_guard

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<skill23_nav_guard::WheelSafetyGuardNode>());
  rclcpp::shutdown();
  return 0;
}
