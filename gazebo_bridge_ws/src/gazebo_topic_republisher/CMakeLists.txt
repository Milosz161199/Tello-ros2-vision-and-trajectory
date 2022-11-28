cmake_minimum_required(VERSION 3.5)
project(gazebo_topic_republisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(gazebo_bridge_interfaces_msgs REQUIRED)

include_directories(include)

add_executable(bridge_node src/gazebo_to_ros_node.cpp)

ament_target_dependencies(bridge_node
  rclcpp
  gazebo_ros
  gazebo_dev
  gazebo_bridge_interfaces_msgs
)

install(TARGETS
  bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()