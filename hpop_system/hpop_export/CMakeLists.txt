cmake_minimum_required(VERSION 3.8)
project(hpop_export)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hpop_msgs REQUIRED)

# Header-only library
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export service node
add_executable(export_service_node src/export_service_node.cpp)
target_include_directories(export_service_node PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../hpop_core/include
)
ament_target_dependencies(export_service_node
  rclcpp
  std_msgs
  hpop_msgs
)

# Test executable
add_executable(test_export test/test_export.cpp)
target_include_directories(test_export PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../hpop_core/include
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

install(TARGETS export_service_node test_export
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
