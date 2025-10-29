cmake_minimum_required(VERSION 3.5)
project(my_first_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

add_executable(robot_commander src/robot_commander.cpp)
ament_target_dependencies(robot_commander rclcpp trajectory_msgs)

install(TARGETS
  robot_commander
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()