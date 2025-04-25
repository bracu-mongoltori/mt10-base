cmake_minimum_required(VERSION 3.5)
project(finalassemdoneright)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)

install(
  DIRECTORY launch urdf worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
