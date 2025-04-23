# Wymagana minimalna wersja CMake
cmake_minimum_required(VERSION 3.5)

# Nazwa projektu
project(lab_02_planowanie_ruchu)

# --------------------------
# Znajdowanie potrzebnych paczek ROS2
# --------------------------
find_package(ament_cmake REQUIRED)       # System budowania ROS2
find_package(rclcpp REQUIRED)            # C++ API ROS2
find_package(geometry_msgs REQUIRED)     # Wiadomości geometryczne (np. Twist, Pose)
find_package(sensor_msgs REQUIRED)       # Wiadomości z sensorów

# --------------------------
# Ustawienia kompilatora
# --------------------------
set(CMAKE_CXX_STANDARD 17)               # Wymuszamy standard C++17
add_compile_options(-Wall -Wextra -Wpedantic)  # Ostrzeżenia kompilatora, dobre praktyki

# --------------------------
# Tworzenie programów wykonywalnych z plików źródłowych
# --------------------------
add_executable(square_trajectory src/square_trajectory.cpp)   # Ruch po kwadracie
add_executable(circle_trajectory src/circle_trajectory.cpp)   # Ruch po okręgu
add_executable(spiral_trajectory src/spiral_trajectory.cpp)   # Ruch spiralny

# --------------------------
# Dodanie zależności do każdego programu
# --------------------------
foreach(target square_trajectory circle_trajectory spiral_trajectory)
  ament_target_dependencies(${target}
    rclcpp                # Umożliwia komunikację ROS2
    geometry_msgs         # Obsługuje wiadomości typu Twist itp.
    sensor_msgs           # Wiadomości czujników
  )
endforeach()

# --------------------------
# Sekcja instalacyjna
# --------------------------

# Zainstaluj pliki wykonywalne do odpowiedniego katalogu lib/
install(TARGETS
  square_trajectory
  circle_trajectory
  spiral_trajectory
  DESTINATION lib/${PROJECT_NAME})

# Zainstaluj plik nagłówkowy (bazowa klasa trajektorii)
install(FILES
  src/trajectory_base.hpp
  DESTINATION include/${PROJECT_NAME})

# Oznaczenie pakietu jako kompletny pakiet ROS2
ament_package()
