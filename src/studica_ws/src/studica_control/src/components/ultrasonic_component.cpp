/*
 * ultrasonic_component.cpp
 *
 * ros2 component for an hc-sr04-style ultrasonic range sensor.
 * measures distance by timing how long a sound pulse takes to bounce back.
 * effective range is roughly 2cm to 4m. readings outside this range are
 * reported as infinity. supports multiple sensors by adding entries to the
 * sensors list in params.yaml.
 *
 * topic:   <topic> (sensor_msgs/Range)
 *            distance in metres, published at 20hz.
 *            radiation_type: ultrasound
 *            valid range: 0.02m – 4.0m
 *
 * service: ultrasonic_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     get_distance             — returns distance in millimetres as a string
 *     get_distance_inches      — returns distance in inches as a string
 *     get_distance_millimeters — returns distance in millimetres as a string
 */

#include "studica_control/ultrasonic_component.hpp"

#include <algorithm>
#include <thread>

namespace studica_control {

namespace {
int period_ms_from_hz(double hz)
{
    const double clamped = std::max(1.0, std::min(hz, 20.0));
    return static_cast<int>(1000.0 / clamped);
}
}  // namespace


// reads ultrasonic parameters from params.yaml and creates one node per entry
// in the sensors list
std::vector<std::shared_ptr<rclcpp::Node>> Ultrasonic::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> ultrasonic_nodes;

    control->declare_parameter<std::vector<std::string>>("ultrasonic.sensors", std::vector<std::string>{});
    control->declare_parameter<double>("ultrasonic.publish_rate_hz", 20.0);
    // Space pings so multiple HC-SR04s do not hear each other (crosstalk → fake 1–2 m).
    control->declare_parameter<int>("ultrasonic.stagger_ms", 40);
    const double publish_rate_hz = control->get_parameter("ultrasonic.publish_rate_hz").as_double();
    const int stagger_ms = control->get_parameter("ultrasonic.stagger_ms").as_int();
    std::vector<std::string> sensor_ids = control->get_parameter("ultrasonic.sensors").as_string_array();

    int index = 0;
    for (const auto &sensor : sensor_ids) {
        std::string ping_param     = "ultrasonic." + sensor + ".ping";
        std::string echo_param     = "ultrasonic." + sensor + ".echo";
        std::string frame_id_param = "ultrasonic." + sensor + ".frame_id";

        control->declare_parameter<int>(ping_param, -1);
        control->declare_parameter<int>(echo_param, -1);
        control->declare_parameter<std::string>(frame_id_param, sensor);

        int ping             = control->get_parameter(ping_param).as_int();
        int echo             = control->get_parameter(echo_param).as_int();
        std::string frame_id = control->get_parameter(frame_id_param).as_string();
        const int phase_ms = std::max(0, stagger_ms) * index;

        RCLCPP_INFO(control->get_logger(),
                    "%s -> ping: %d, echo: %d, frame_id: %s, phase_offset_ms: %d",
                    sensor.c_str(), ping, echo, frame_id.c_str(), phase_ms);

        auto ultrasonic = std::make_shared<Ultrasonic>(
            vmx, sensor, ping, echo, frame_id, publish_rate_hz, phase_ms);
        ultrasonic_nodes.push_back(ultrasonic);
        ++index;
    }

    return ultrasonic_nodes;
}


// composable node constructor — used when loading as a plugin
Ultrasonic::Ultrasonic(const rclcpp::NodeOptions &options) : Node("ultrasonic", options) {}


// main constructor — connects to the sensor and sets up the publisher,
// service, and periodic timer
Ultrasonic::Ultrasonic(std::shared_ptr<VMXPi> vmx, const std::string &name,
                       VMXChannelIndex ping, VMXChannelIndex echo, const std::string &frame_id,
                       double publish_rate_hz, int phase_offset_ms)
    : Node(name), vmx_(vmx), ping_(ping), echo_(echo), frame_id_(frame_id),
      publish_rate_hz_(publish_rate_hz) {

    ultrasonic_ = std::make_shared<studica_driver::Ultrasonic>(ping_, echo_, vmx_);

    // service for reading distance on demand
    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/ultrasonic_cmd",
        std::bind(&Ultrasonic::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    publisher_ = this->create_publisher<sensor_msgs::msg::Range>(name + "/range", 10);

    if (phase_offset_ms > 0) {
        start_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(phase_offset_ms),
            [this]() {
                if (start_timer_) {
                    start_timer_->cancel();
                    start_timer_.reset();
                }
                start_periodic_timer();
            });
    } else {
        start_periodic_timer();
    }

    RCLCPP_INFO(this->get_logger(),
                "%s ultrasonic publish_rate=%.1f Hz phase_offset_ms=%d",
                name.c_str(), publish_rate_hz_, phase_offset_ms);
}

void Ultrasonic::start_periodic_timer() {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms_from_hz(publish_rate_hz_)),
        std::bind(&Ultrasonic::publish_range, this));
}

Ultrasonic::~Ultrasonic() {}


void Ultrasonic::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                              std::shared_ptr<studica_control::srv::SetData::Response> response) {
    cmd(request->params, response);
}


// dispatches incoming service commands. each command triggers a ping first
// to get a fresh reading before returning the result.
void Ultrasonic::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_distance") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceMM());

    } else if (params == "get_distance_inches") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceIN());

    } else if (params == "get_distance_millimeters") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceMM());

    } else {
        response->success = false;
        response->message = "unknown command '" + params + "'";
    }
}


// triggers a ping, converts the result to metres, and publishes a standard
// range message. readings outside the sensor's valid range are set to infinity.
void Ultrasonic::publish_range() {
    constexpr double min_range     = 0.02;  // metres — hc-sr04 minimum
    constexpr double max_range     = 4.0;   // metres — hc-sr04 maximum
    constexpr double field_of_view = 0.26;  // radians — approximate cone width

    ultrasonic_->Ping();
    // Driver Ping() only waits 5 ms; HC-SR04 needs ~23 ms for a clean timeout at 4 m.
    // Extra wait avoids reading a stale capture count as a fake mid-range distance.
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    double distance_m = ultrasonic_->GetDistanceMM() / 1000.0;
    if (distance_m < min_range || distance_m > max_range) distance_m = INFINITY;

    sensor_msgs::msg::Range msg;
    msg.header.stamp    = this->get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.radiation_type  = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view   = field_of_view;
    msg.min_range       = min_range;
    msg.max_range       = max_range;
    msg.range           = distance_m;

    publisher_->publish(msg);
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Ultrasonic)
