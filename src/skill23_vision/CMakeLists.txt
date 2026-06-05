cmake_minimum_required(VERSION 3.8)
project(skill23_vision)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FindColor.srv"
  "srv/SetGate.srv"
  DEPENDENCIES std_msgs
)

# Node is a standalone script (rosidl already owns the Python package name).
install(PROGRAMS
  scripts/vision_gate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
