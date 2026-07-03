cmake_minimum_required(VERSION 3.8)
project(mecanum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(studica_control REQUIRED)
find_package(message_filters REQUIRED) 

# Build the executable
add_executable(mecanum_node src/mecanum_node.cpp)
ament_target_dependencies(mecanum_node
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

# Install the executable so ROS 2 can find it using ros2 run
install(TARGETS
  mecanum_node
  pid_move
  pid_turn
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(DIRECTORY 
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()