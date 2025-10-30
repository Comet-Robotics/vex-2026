cmake_minimum_required(VERSION 3.8)
project(msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# -------------------------------------------------------------
# ROS message generation
# -------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# -------------------------------------------------------------
# Install include directory and export it
# -------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
