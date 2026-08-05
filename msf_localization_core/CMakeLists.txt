cmake_minimum_required(VERSION 3.8)
project(msf_localization_core)

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
# Library
# ==============================================================================

add_library(${PROJECT_NAME}
    src/filters/lkf.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

# dependencies

find_package(Eigen3 REQUIRED)

target_link_libraries(${PROJECT_NAME}
    PUBLIC Eigen3::Eigen
)

# ==============================================================================
# Apps
# ==============================================================================

add_executable(lkf_demo
    apps/lkf_demo.cpp
)

target_link_libraries(lkf_demo
    PRIVATE ${PROJECT_NAME}
)

# ==============================================================================
# Installation
# ==============================================================================

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)