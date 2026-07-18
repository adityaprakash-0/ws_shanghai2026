#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanRelay : public rclcpp::Node
{
public:
  ScanRelay()
  : Node("scan_relay")
  {
    const auto in_topic = declare_parameter<std::string>("input_scan_topic", "/scan");
    const auto out_topic = declare_parameter<std::string>("output_scan_topic", "/scan_filtered");

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(out_topic, 10);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      in_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { pub_->publish(*msg); });

    RCLCPP_INFO(get_logger(), "Scan relay: %s -> %s", in_topic.c_str(), out_topic.c_str());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanRelay>());
  rclcpp::shutdown();
  return 0;
}
