cmake_minimum_required(VERSION 3.8)
project(skill23_nav_guard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(wheel_safety_guard src/wheel_safety_guard.cpp)
ament_target_dependencies(wheel_safety_guard rclcpp geometry_msgs sensor_msgs)

install(TARGETS wheel_safety_guard
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
