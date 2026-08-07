// Copyright 2026 — skill23_rpp

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "skill23_rpp/rpp_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<skill23_rpp::RppNode>();
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
