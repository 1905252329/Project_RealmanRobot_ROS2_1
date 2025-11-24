cmake_minimum_required(VERSION 3.8)
project(rm_jodell_evs08_control_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 声明srv文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InitializeGripper.srv"
  "srv/ActivateGripper.srv"
  "srv/DeactivateGripper.srv"
  "srv/CloseModbus.srv"
  DEPENDENCIES std_msgs
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)

# 安装Python模块
ament_python_install_package(${PROJECT_NAME})

# 安装可执行文件
install(PROGRAMS
  rm_jodell_evs08_control_py/gripper_control_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()