cmake_minimum_required(VERSION 3.8)
project(grip)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(studica_control REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(grip_node src/grip_node.cpp)
ament_target_dependencies(grip_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

add_executable(pid_move src/pid_move.cpp)
ament_target_dependencies(pid_move rclcpp std_msgs geometry_msgs studica_control)

add_executable(pid_turn src/pid_turn.cpp)
ament_target_dependencies(pid_turn rclcpp std_msgs geometry_msgs studica_control)

# Dual-speed rotate-to-heading RPP (grip nav only) — see dual_rotate_rpp_plugin.xml
add_library(grip_dual_rotate_rpp_controller SHARED
  src/dual_rotate_rpp_controller.cpp
)
target_include_directories(grip_dual_rotate_rpp_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(grip_dual_rotate_rpp_controller
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  nav2_core
  nav2_util
  nav2_costmap_2d
  nav2_regulated_pure_pursuit_controller
  pluginlib
)
target_compile_definitions(grip_dual_rotate_rpp_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS
  grip_node
  pid_move
  pid_turn
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS grip_dual_rotate_rpp_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES dual_rotate_rpp_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core dual_rotate_rpp_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(grip_dual_rotate_rpp_controller)
ament_export_dependencies(
  rclcpp
  nav2_core
  nav2_regulated_pure_pursuit_controller
  pluginlib
)

ament_package()
