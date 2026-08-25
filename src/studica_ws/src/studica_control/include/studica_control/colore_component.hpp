/*
 * colore_component.hpp
 *
 * ROS2 component for the Studica Colore color sensor.
 *   CAN: studica_driver::Colore
 *   USB: studica_driver::ColoreUsb  (CDC serial, typically /dev/ttyACM0)
 *
 * Topics (per sensor name):
 *   <name>/color        std_msgs/ColorRGBA   sRGB 0–1 derived from XYZ
 *   <name>/color_info   ColoreColorMsg       optional (publish_outputs: info)
 *   <name>/color_match  ColoreMatch          optional (publish_outputs: match)
 *
 * Service: <name>/colore_cmd (studica_control/SetData)
 *   get_config
 *   set_brightness          initparams.n_encoder = 0–255
 *   learn_color,<label>     snapshot current xy as a named reference
 */

#ifndef COLORE_COMPONENT_H
#define COLORE_COMPONENT_H

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "colore.hpp"
#include "colore_usb.hpp"
#include "studica_control/msg/colore_color_msg.hpp"
#include "studica_control/msg/colore_match.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

enum class ColoreTransport { Can, Usb };

struct ColorePublishOutputs {
    bool info{false};
    bool match{false};
};

struct ColoreReference {
    std::string label;
    float x{0.f};
    float y{0.f};
};

class Colore : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(
        rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    explicit Colore(const rclcpp::NodeOptions &options);

    Colore(std::shared_ptr<VMXPi> vmx, const std::string &name,
           ColoreTransport transport, uint8_t can_id, const std::string &serial_port,
           int publish_rate_hz,
           const std::vector<std::string> &publish_outputs,
           int brightness,
           const std::string &measmode, float measmode_z_mm,
           const std::vector<ColoreReference> &references, float match_threshold);

    ~Colore();

    static ColoreTransport parse_transport(const std::string &v);
    static ColorePublishOutputs parse_publish_outputs(const std::vector<std::string> &names);

private:
    ColoreTransport transport_{ColoreTransport::Can};
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Colore> colore_can_;
    std::shared_ptr<studica_driver::ColoreUsb> colore_usb_;
    int publish_rate_hz_{10};
    ColorePublishOutputs outputs_{};
    std::vector<ColoreReference> references_;
    float match_threshold_{0.05f};
    float last_x_{0.f};
    float last_y_{0.f};
    bool have_last_xy_{false};

    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr color_publisher_;
    rclcpp::Publisher<studica_control::msg::ColoreColorMsg>::SharedPtr info_publisher_;
    rclcpp::Publisher<studica_control::msg::ColoreMatch>::SharedPtr match_publisher_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(const std::string &params,
             const studica_control::srv::SetData::Request &request,
             std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_color();
    void publish_match(float x, float y);
    void apply_measmode(const std::string &mode, float z_mm);
    static void xyz_to_srgb(float X, float Y, float Z, float &r, float &g, float &b);
    static bool xyz_to_xy(float X, float Y, float Z, float &x, float &y);
};

}  // namespace studica_control

#endif  // COLORE_COMPONENT_H
