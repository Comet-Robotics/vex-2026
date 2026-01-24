cmake_minimum_required(VERSION 3.8)
project(msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorMsg.msg"
  "msg/Brain.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Export the runtime dependency so nodes using these msgs can link properly
ament_export_dependencies(rosidl_default_runtime)

ament_package()