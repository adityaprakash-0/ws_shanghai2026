cmake_minimum_required(VERSION 3.8)
project(allterrain)

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


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Executables ---
add_executable(allterrain_node src/allterrain_node.cpp)
ament_target_dependencies(allterrain_node
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

add_executable(lidar_merger_node src/lidar_merger.cpp)
ament_target_dependencies(lidar_merger_node 
  rclcpp 
  sensor_msgs 
  message_filters
)

# --- Install Executables ---
install(TARGETS
  pid_move
  pid_turn
  allterrain_node
  lidar_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- ADDED: Install non-code directories (Launch, Config, URDF) ---
install(DIRECTORY 
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()