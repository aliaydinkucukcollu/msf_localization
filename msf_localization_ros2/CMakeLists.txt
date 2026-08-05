cmake_minimum_required(VERSION 3.8)
project(msf_localization_ros2)

# ==============================================================================
# Compiler Options
# ==============================================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
    )
endif()

# ==============================================================================
# C++ Standard
# ==============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
# Dependencies
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)

# GeographicLib
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)

include_directories(include)

# ==============================================================================
# Core Library
# ==============================================================================
add_subdirectory(
    ../msf_localization_core
    ${CMAKE_BINARY_DIR}/msf_localization_core
)

# ==============================================================================
# Executable
# ==============================================================================
add_executable(${PROJECT_NAME}
    src/msf_localization_ros2.cpp
    src/msf_localization_ros2_node.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME}
        msf_localization_core
        Eigen3::Eigen
        ${GeographicLib_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

# ==============================================================================
# Install
# ==============================================================================
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
