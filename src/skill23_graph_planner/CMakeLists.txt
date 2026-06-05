cmake_minimum_required(VERSION 3.8)
project(skill23_graph_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

set(COMMON_DEPS
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

include_directories(include)

# GraphPlanner — Nav2 global planner on a GeoJSON corridor graph
add_library(graph_planner_plugin SHARED
  src/graph_planner.cpp
)
target_include_directories(graph_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(graph_planner_plugin ${COMMON_DEPS})
target_link_libraries(graph_planner_plugin nlohmann_json::nlohmann_json)
pluginlib_export_plugin_description_file(nav2_core graph_planner_plugin.xml)

install(TARGETS graph_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES graph_planner_plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_include_directories(include)
ament_export_libraries(graph_planner_plugin)
ament_export_dependencies(${COMMON_DEPS})

ament_package()
