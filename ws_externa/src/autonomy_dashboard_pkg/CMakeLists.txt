cmake_minimum_required(VERSION 3.8)
project(autonomy_dashboard_pkg)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL Concurrent)
find_package(PCL REQUIRED COMPONENTS common features kdtree search)
find_package(TIFF REQUIRED)
find_package(urdf REQUIRED)
find_package(assimp REQUIRED)
find_package(OpenMP)
find_library(GEOTIFF_LIBRARY NAMES geotiff REQUIRED)

# ---------------------------------------------------------------------------
# Dashboard en C++ (Qt5 + OpenGL)
# ---------------------------------------------------------------------------
add_executable(autonomy_dashboard
  src/main.cpp
  src/ros_bridge.cpp
  src/exporters.cpp
  src/mapio.cpp
  src/robot_model.cpp
  src/map2d_view.cpp
  src/cloud3d_view.cpp
  src/main_window.cpp
  src/ros_bridge.hpp
  src/mapio.hpp
  src/robot_model.hpp
  src/map2d_view.hpp
  src/cloud3d_view.hpp
  src/main_window.hpp
)
target_include_directories(autonomy_dashboard PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${TIFF_INCLUDE_DIRS}
  /usr/include/geotiff
)
target_compile_definitions(autonomy_dashboard PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(autonomy_dashboard
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs tf2 tf2_ros urdf)
target_link_libraries(autonomy_dashboard
  Qt5::Widgets Qt5::OpenGL Qt5::Concurrent
  ${PCL_LIBRARIES}
  ${TIFF_LIBRARIES}
  ${GEOTIFF_LIBRARY}
  assimp::assimp
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(autonomy_dashboard OpenMP::OpenMP_CXX)
endif()

install(TARGETS autonomy_dashboard DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Versión anterior en Python (se conserva por si se necesita)
# ---------------------------------------------------------------------------
ament_python_install_package(autonomy_dashboard)
install(PROGRAMS
  autonomy_dashboard/autonomy_dashboard_node.py
  autonomy_dashboard/autonomy_ui.py
  autonomy_dashboard/test_ui.py
  autonomy_dashboard/ply_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
