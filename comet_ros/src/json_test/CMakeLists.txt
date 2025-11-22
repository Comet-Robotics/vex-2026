cmake_minimum_required(VERSION 3.8)
project(json_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(utils REQUIRED)
find_package(msgs REQUIRED)

add_executable(json_test src/json_test.cpp)
ament_target_dependencies(json_test
  rclcpp
  std_msgs
  nlohmann_json
  utils
  msgs
)

# Explicitly add msgs include directory
target_include_directories(json_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../msgs/include>  # <- add this line
)

install(TARGETS json_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
