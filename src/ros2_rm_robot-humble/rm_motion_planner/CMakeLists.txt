# =============================================================================
# 项目名称: rm_motion_planner
# 开发日期: 2025-08-01 至今
# 🧪 版本信息: V1.0.0，调试版本
# 功能说明: 
#   - 提供核心功能模块1
#   - 实现关键功能2
#   - 支持扩展功能3
# 开发者: likang
# 联系方式: email@example.com
# =============================================================================

cmake_minimum_required(VERSION 3.5)
project(rm_motion_planner)

# Option to skip building original node executables (useful for local testing of examples)
option(BUILD_OLD_NODES "Build existing rm_sclerp_planner node executables" OFF)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  # examples use C++17 features (std::optional, std::filesystem etc.)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rm_ros_interfaces REQUIRED)
find_package(kinlib REQUIRED)
find_package(sclerp_motion_planner REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(sorting_robot_interfaces REQUIRED)
find_package(rm_driver REQUIRED)
find_package(ament_cmake_gtest QUIET)

# Ensure Eigen3 include directory is set
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# 临时硬编码
# include_directories(${CMAKE_SOURCE_DIR}/../src/ros2_rm_robot-humble/rm_driver/include)
# message("======================${CMAKE_SOURCE_DIR}/../src/ros2_rm_robot-humble/rm_driver/include") 

if(BUILD_OLD_NODES)
  # 添加单臂规划器节点
  add_executable(rm_sclerp_planner_node_rightArm 
                 src/rm_sclerp_planner_node_rightArm.cpp
                 src/robot_interface.cpp
                 )
  target_include_directories(rm_sclerp_planner_node_rightArm PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${rm_driver_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(rm_sclerp_planner_node_rightArm 
                        kinlib::kinlib 
                        ${rm_driver_LIBRARIES})
  ament_target_dependencies(rm_sclerp_planner_node_rightArm
    rclcpp 
    rclcpp_action
    std_msgs
    sensor_msgs
    geometry_msgs
    builtin_interfaces
    trajectory_msgs
    control_msgs
    rm_ros_interfaces
    sclerp_motion_planner
    sorting_robot_interfaces
    urdf
    Boost
  )

  # ✨ 添加单臂规划器节点
  add_executable(rm_sclerp_planner_node_leftArm 
                 src/rm_sclerp_planner_node_leftArm.cpp
                 src/robot_interface.cpp
                 )
  target_include_directories(rm_sclerp_planner_node_leftArm PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${rm_driver_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(rm_sclerp_planner_node_leftArm 
                        kinlib::kinlib 
                        ${rm_driver_LIBRARIES})
  ament_target_dependencies(rm_sclerp_planner_node_leftArm
    rclcpp 
    rclcpp_action
    std_msgs
    sensor_msgs
    geometry_msgs
    builtin_interfaces
    trajectory_msgs
    control_msgs
    rm_ros_interfaces
    sclerp_motion_planner
    sorting_robot_interfaces
    urdf
    Boost
  )
endif()

# Example: common library for new modular components (examples only)
add_library(rm_motion_planner_common STATIC
  src/gripper_client.cpp
  src/trajectory_executor.cpp
  src/utils.cpp
  src/grasp_place_handlers.cpp
  src/mock_robot_interface_impl.cpp
)
target_include_directories(rm_motion_planner_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rm_motion_planner_common
  kinlib::kinlib
  ${rm_driver_LIBRARIES}
)
ament_target_dependencies(rm_motion_planner_common
  rclcpp
  trajectory_msgs
  rm_ros_interfaces
  Eigen3
)
target_include_directories(rm_motion_planner_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(rm_motion_planner_common
  rclcpp_action
  sorting_robot_interfaces
)

# Refactor node (lightweight) that uses the new modular components
add_executable(rm_sclerp_planner_node_leftArm_refactor
  src/rm_sclerp_planner_node_leftArm_refactor.cpp
)
target_link_libraries(rm_sclerp_planner_node_leftArm_refactor
  rm_motion_planner_common
)
ament_target_dependencies(rm_sclerp_planner_node_leftArm_refactor
  rclcpp
  rclcpp_action
  sorting_robot_interfaces
  rm_ros_interfaces
)
target_include_directories(rm_sclerp_planner_node_leftArm_refactor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Add a simple test executable (gtest) if testing enabled
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_utils
    test/test_utils.cpp
  )
  if(TARGET test_utils)
    target_include_directories(test_utils PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  # link using plain signature to avoid conflict with ament_add_gtest internals
  target_link_libraries(test_utils rm_motion_planner_common)
  ament_target_dependencies(test_utils rclcpp Eigen3)
  endif()
endif()





# ✨ 添加 jodell 电吸盘客户端节点
add_executable(jodell_evs08_client
               src/jodell_evs08_client.cpp)
target_include_directories(jodell_evs08_client PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${rm_driver_INCLUDE_DIRS}
)
target_link_libraries(jodell_evs08_client
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${rm_driver_LIBRARIES}
)
ament_target_dependencies(jodell_evs08_client
  rclcpp
  std_msgs
  rm_ros_interfaces
  Boost
)



if(BUILD_OLD_NODES)
  install(TARGETS 
    rm_sclerp_planner_node_rightArm
    rm_sclerp_planner_node_leftArm
    jodell_evs08_client
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  # Install only the client when old nodes are disabled (keep behavior minimal)
  install(TARGETS 
    jodell_evs08_client
    DESTINATION lib/${PROJECT_NAME}
  )
endif()


install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# 添加配置文件安装指令
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()