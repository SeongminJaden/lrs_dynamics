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

# Rendezvous Controller Node
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(hpop_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# MoveIt2 for robot arm control
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(rendezvous_controller src/rendezvous_controller.cpp)

# Base dependencies
set(RENDEZVOUS_DEPS
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  gazebo_msgs
  hpop_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# Add MoveIt2 if available
if(moveit_ros_planning_interface_FOUND AND moveit_msgs_FOUND)
  message(STATUS "MoveIt2 found - enabling MoveGroup interface")
  target_compile_definitions(rendezvous_controller PRIVATE MOVEIT2_AVAILABLE)
  list(APPEND RENDEZVOUS_DEPS moveit_ros_planning_interface moveit_msgs)
else()
  message(STATUS "MoveIt2 not found - using trajectory controller only")
endif()

ament_target_dependencies(rendezvous_controller ${RENDEZVOUS_DEPS})

# ==================== RViz2 Mission Control Panel ====================
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(ament_index_cpp REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# MOC the header file
qt5_wrap_cpp(MOC_FILES include/hpop_gazebo/mission_control_panel.hpp)

add_library(mission_control_panel SHARED
  src/rviz_panel/mission_control_panel.cpp
  ${MOC_FILES}
)
target_include_directories(mission_control_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mission_control_panel
  Qt5::Widgets
)
ament_target_dependencies(mission_control_panel
  rclcpp
  rviz_common
  rviz_rendering
  pluginlib
  std_msgs
  std_srvs
  ament_index_cpp
)

pluginlib_export_plugin_description_file(rviz_common plugins.xml)

# Install targets
install(TARGETS
  gazebo_hpop_bridge_node
  arm_capture_controller
  rendezvous_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  magnetic_docking_plugin
  mission_control_panel
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
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
