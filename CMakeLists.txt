cmake_minimum_required(VERSION 3.8)
project(rsds)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR NOT CMAKE_BUILD_TYPE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -ffast-math")
endif()

# Warning flags
add_compile_options(-Wall -Wextra -Wpedantic)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(OpenMP REQUIRED)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# Source files
set(SOURCES
  src/ggm_model.cpp
  src/legendre.cpp
  src/leo_gravity_world_plugin.cc
)

# Compile shared library for Gazebo plugin
add_library(leo_gravity_world_plugin SHARED ${SOURCES})
target_link_libraries(leo_gravity_world_plugin
  ${GAZEBO_LIBRARIES}
  OpenMP::OpenMP_CXX
)
ament_target_dependencies(leo_gravity_world_plugin
  rclcpp
  ignition-math6
)

# Test executable
add_executable(test_ggm test/test_ggm.cpp src/ggm_model.cpp)
target_link_libraries(test_ggm
  ${GAZEBO_LIBRARIES}
  OpenMP::OpenMP_CXX
)
ament_target_dependencies(test_ggm ignition-math6)

# GGM05C sample data extractor
add_executable(extract_ggm_sample test/extract_ggm_sample.cpp src/ggm_model.cpp)
target_link_libraries(extract_ggm_sample
  ${GAZEBO_LIBRARIES}
  OpenMP::OpenMP_CXX
)
ament_target_dependencies(extract_ggm_sample ignition-math6)

# Install plugin library
install(TARGETS leo_gravity_world_plugin test_ggm extract_ggm_sample
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install data (GGM05C)
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
ament_package()
