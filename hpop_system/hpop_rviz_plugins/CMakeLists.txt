cmake_minimum_required(VERSION 3.8)
project(hpop_rviz_plugins)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Qt5 MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hpop_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Qt5 include directories
set(QT_INCLUDE_DIRS
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Include directories
include_directories(
  include
  ${QT_INCLUDE_DIRS}
)

# HPOP Panel plugin library
# Note: Header files with Q_OBJECT must be listed for AUTOMOC
add_library(hpop_panel SHARED
  src/hpop_panel.cpp
  src/satellite_table_widget.cpp
  src/propagation_control_widget.cpp
  include/hpop_rviz_plugins/hpop_panel.hpp
)

target_include_directories(hpop_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hpop_panel
  Qt5::Core
  Qt5::Widgets
)

ament_target_dependencies(hpop_panel
  rclcpp
  rviz_common
  rviz_rendering
  pluginlib
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  hpop_msgs
)

# Orbit Display plugin library
# Note: Header files with Q_OBJECT must be listed for AUTOMOC
add_library(hpop_displays SHARED
  src/orbit_display.cpp
  src/earth_display.cpp
  include/hpop_rviz_plugins/orbit_display.hpp
  include/hpop_rviz_plugins/earth_display.hpp
)

target_include_directories(hpop_displays PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hpop_displays
  Qt5::Core
  Qt5::Widgets
)

ament_target_dependencies(hpop_displays
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
  std_msgs
  geometry_msgs
  visualization_msgs
  hpop_msgs
)

# Prevent pluginlib from using boost
target_compile_definitions(hpop_panel PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(hpop_displays PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Install plugin description
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install targets
install(TARGETS hpop_panel hpop_displays
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install resources
install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  pluginlib
  Qt5
)

ament_package()
