cmake_minimum_required(VERSION 3.8)
project(hpop_tle)

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
find_package(hpop_msgs REQUIRED)
find_package(hpop_core REQUIRED)
find_package(CURL REQUIRED)

# Header-only library
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE hpop_core::hpop_core)

# TLE Service Node
add_executable(tle_service_node src/nodes/tle_service_node.cpp)
target_include_directories(tle_service_node PRIVATE
  include
  ${CURL_INCLUDE_DIRS}
)
target_link_libraries(tle_service_node ${CURL_LIBRARIES})
ament_target_dependencies(tle_service_node
  rclcpp
  std_msgs
  hpop_msgs
  hpop_core
)

# Test executable
add_executable(test_tle test/test_tle.cpp)
target_include_directories(test_tle PRIVATE
  include
  ${CURL_INCLUDE_DIRS}
)
target_link_libraries(test_tle ${CURL_LIBRARIES})
ament_target_dependencies(test_tle hpop_core)

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

install(TARGETS tle_service_node test_tle
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

# Export targets
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hpop_core CURL)

ament_package()
