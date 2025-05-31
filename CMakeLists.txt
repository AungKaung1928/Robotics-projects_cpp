cmake_minimum_required(VERSION 3.8)
project(simple_navigator)

# C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executables
add_executable(obstacle_detector src/obstacle_detector.cpp)
add_executable(simple_navigator src/simple_navigator.cpp)

# Link dependencies
ament_target_dependencies(obstacle_detector
  rclcpp std_msgs sensor_msgs geometry_msgs)

ament_target_dependencies(simple_navigator
  rclcpp std_msgs sensor_msgs geometry_msgs)

# Install executables
install(TARGETS
  obstacle_detector
  simple_navigator
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
