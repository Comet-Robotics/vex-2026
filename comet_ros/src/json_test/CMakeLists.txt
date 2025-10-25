cmake_minimum_required(VERSION 3.8)
project(json_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(utils REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(json_test src/json_test.cpp)
ament_target_dependencies(json_test rclcpp std_msgs utils nlohmann_json)

install(TARGETS json_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
