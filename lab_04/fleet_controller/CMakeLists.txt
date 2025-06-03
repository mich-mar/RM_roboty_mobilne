cmake_minimum_required(VERSION 3.8)
project(fleet_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

# Add the executable
add_executable(fleet_controller_node
  src/main.cpp
  src/controller.cpp
)

target_include_directories(fleet_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(fleet_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# Install executables
install(TARGETS
  fleet_controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
