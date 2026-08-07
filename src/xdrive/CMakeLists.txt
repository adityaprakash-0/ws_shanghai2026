cmake_minimum_required(VERSION 3.8)
project(xdrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(studica_control REQUIRED)

add_executable(xdrive_node src/xdrive_node.cpp)
ament_target_dependencies(xdrive_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
)

add_executable(pid_move src/pid_move.cpp)
ament_target_dependencies(pid_move rclcpp std_msgs geometry_msgs studica_control)

add_executable(pid_turn src/pid_turn.cpp)
ament_target_dependencies(pid_turn rclcpp std_msgs geometry_msgs studica_control)

install(TARGETS
  xdrive_node
  pid_move
  pid_turn
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
