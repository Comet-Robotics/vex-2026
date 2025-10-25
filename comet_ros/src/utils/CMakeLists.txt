cmake_minimum_required(VERSION 3.8)
project(utils)

find_package(ament_cmake REQUIRED)

# Export include directory so downstream packages can include headers
ament_export_include_directories(include)

# Install the include folder
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
