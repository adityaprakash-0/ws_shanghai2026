cmake_minimum_required(VERSION 3.5)
project(skill23_rpp)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(include)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
)

add_library(rpp_core SHARED
  src/regulated_pure_pursuit_controller.cpp
  src/rpp_node.cpp
)
ament_target_dependencies(rpp_core ${dependencies})

add_executable(rpp src/main.cpp)
ament_target_dependencies(rpp ${dependencies})
target_link_libraries(rpp rpp_core)

install(TARGETS rpp_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS rpp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(rpp_core)
ament_export_dependencies(${dependencies})

ament_package()
