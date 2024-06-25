cmake_minimum_required(VERSION 3.8)
project(zed_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
