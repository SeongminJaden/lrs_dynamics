cmake_minimum_required(VERSION 3.8)
project(hpop_gazebo)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hpop_msgs REQUIRED)
find_package(hpop_core REQUIRED)
find_package(hpop_perturbations REQUIRED)
find_package(gazebo_msgs REQUIRED)

# Try to find Gazebo
find_package(gazebo QUIET)
find_package(gazebo_ros QUIET)

# Include directories
include_directories(
  include
  ${hpop_core_INCLUDE_DIRS}
  ${hpop_perturbations_INCLUDE_DIRS}
)

# Gazebo HPOP Bridge Node (publishes state, subscribes to gravity)
add_executable(gazebo_hpop_bridge_node src/gazebo_hpop_bridge_node.cpp)
target_include_directories(gazebo_hpop_bridge_node PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../hpop_core/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../hpop_perturbations/include
)
ament_target_dependencies(gazebo_hpop_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  hpop_msgs
  gazebo_msgs
)

# Install targets
install(TARGETS
  gazebo_hpop_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
