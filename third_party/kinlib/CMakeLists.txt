cmake_minimum_required(VERSION 3.5)
project(kinlib)

add_compile_options(-std=c++17)

# System and ROS dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(ASSIMP REQUIRED assimp)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(fcl REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

pkg_check_modules(CCD REQUIRED ccd)

# Include paths
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${CCD_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
)

link_directories(${CCD_LIBRARY_DIRS})
add_definitions(${CCD_CFLAGS_OTHER})

# Main C++ library
add_library(${PROJECT_NAME} SHARED
  src/DualQuat.cpp
  src/manipulator.cpp
  src/kinlib_kinematics.cpp
  src/lemke.cpp
  src/collision_utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} ${ASSIMP_LIBRARIES} ${FCL_LIBRARIES})

set(DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  urdf
  kdl_parser
  fcl
  Boost
  CCD
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# === PYBIND11 MODULE ===
file(GLOB BINDING_SRCS
  bindings/*.cpp
)

pybind11_add_module(kinlib_py ${BINDING_SRCS})

target_include_directories(kinlib_py PRIVATE include)

add_library(kinlib_py_target STATIC EXCLUDE_FROM_ALL pybind.cpp)
ament_target_dependencies(kinlib_py_target ${DEPENDENCIES})
target_link_libraries(kinlib_py PRIVATE ${PROJECT_NAME} kinlib_py_target)

# === INSTALLS ===
install(TARGETS ${PROJECT_NAME}
  EXPORT export_kinlib_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT export_kinlib_targets
  NAMESPACE kinlib::
  DESTINATION lib/cmake/kinlib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS kinlib_py
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# === EXPORTS ===
ament_export_targets(export_kinlib_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})
ament_package()