cmake_minimum_required(VERSION 3.5)  # ROS 2 uses CMake 3.5+
project(sclerp_motion_planner)

## Compile as C++17 (Recommended for ROS 2)
add_compile_options(-std=c++17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ASSIMP REQUIRED assimp)

find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem) 
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(kinlib REQUIRED)

# Ensure Eigen3 include directory is set
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# Add library
add_library(${PROJECT_NAME} SHARED
  src/sclerp_interface.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Link against kinlib using imported target
target_link_libraries(${PROJECT_NAME} kinlib::kinlib)

# Dependencies without kinlib
set(SCLERP_DEPENDENCIES
  rclcpp
  std_msgs
  urdf
  kdl_parser
  trajectory_msgs
  rosbag2_cpp
  Boost
)

ament_target_dependencies(${PROJECT_NAME} ${SCLERP_DEPENDENCIES})

# PYBIND11 MODULE
file(GLOB BINDING_SRCS
  bindings/*.cpp
)

pybind11_add_module(sclerp_py ${BINDING_SRCS})

target_include_directories(sclerp_py PRIVATE include ${kinlib_INCLUDE_DIRS})

add_library(sclerp_py_target STATIC EXCLUDE_FROM_ALL pybind.cpp)
ament_target_dependencies(sclerp_py_target ${SCLERP_DEPENDENCIES})
target_link_libraries(sclerp_py PRIVATE ${PROJECT_NAME} sclerp_py_target kinlib::kinlib)

# Install rules
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS sclerp_py
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(${SCLERP_DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()