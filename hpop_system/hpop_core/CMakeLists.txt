cmake_minimum_required(VERSION 3.8)
project(hpop_core)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR NOT CMAKE_BUILD_TYPE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(hpop_msgs REQUIRED)

# Header-only library
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Test executable - basic propagator test
add_executable(test_propagator test/test_propagator.cpp)
target_include_directories(test_propagator PRIVATE include)
target_link_libraries(test_propagator)

# Test executable - full propagation with all perturbations
add_executable(test_full_propagation test/test_full_propagation.cpp)
target_include_directories(test_full_propagation PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../hpop_perturbations/include
)

# ROS2 Node: Orbit Propagator
add_executable(orbit_propagator_node src/nodes/orbit_propagator_node.cpp)
target_include_directories(orbit_propagator_node PRIVATE include)
ament_target_dependencies(orbit_propagator_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  hpop_msgs
)

# ROS2 Node: Orbit Demo (for visualization testing)
add_executable(orbit_demo_node src/nodes/orbit_demo_node.cpp)
target_include_directories(orbit_demo_node PRIVATE include)
ament_target_dependencies(orbit_demo_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  visualization_msgs
  gazebo_msgs
  hpop_msgs
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS test_propagator test_full_propagation orbit_propagator_node orbit_demo_node
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

# Export targets
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  hpop_msgs
)

ament_package()
