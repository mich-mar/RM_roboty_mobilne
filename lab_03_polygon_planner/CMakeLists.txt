cmake_minimum_required(VERSION 3.8)
project(lab_03_polygon_planner)

# Domyślne ustawienia kompilacji
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Znajdź zależności
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Dodaj plik wykonywalny
add_executable(polygon_path_planner src/polygon_path_planner.cpp)
ament_target_dependencies(polygon_path_planner rclcpp geometry_msgs nav_msgs)

# Instalacja
install(TARGETS
  polygon_path_planner
  DESTINATION lib/${PROJECT_NAME})

ament_package()
