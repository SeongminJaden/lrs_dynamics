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
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hpop_msgs REQUIRED)
find_package(hpop_core REQUIRED)
find_package(hpop_perturbations REQUIRED)
find_package(gazebo_msgs REQUIRED)

# Find Gazebo
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

# Include directories
include_directories(
  include
  ${hpop_core_INCLUDE_DIRS}
  ${hpop_perturbations_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

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

# Magnetic Docking Gazebo Plugin
add_library(magnetic_docking_plugin SHARED
  src/magnetic_docking_plugin.cpp
)
target_include_directories(magnetic_docking_plugin PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(magnetic_docking_plugin
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(magnetic_docking_plugin
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  gazebo_ros
)

# Arm Capture Controller Node
find_package(sensor_msgs REQUIRED)
add_executable(arm_capture_controller src/arm_capture_controller.cpp)
ament_target_dependencies(arm_capture_controller
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
)

# Install targets
install(TARGETS
  gazebo_hpop_bridge_node
  arm_capture_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  magnetic_docking_plugin
  LIBRARY DESTINATION lib
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

# Install mesh files
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
