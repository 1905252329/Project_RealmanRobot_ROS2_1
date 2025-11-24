cmake_minimum_required(VERSION 3.5)
project(rm_ros_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # msg
  "msg/Armcurrentstatus.msg"
  "msg/Armoriginalstate.msg"
  "msg/Armsoftversion.msg"
  "msg/Armstate.msg"
  "msg/Cartepos.msg"
  "msg/Carteposcustom.msg"
  "msg/Forcepositionmovejoint.msg"
  "msg/Forcepositionmovepose.msg"
  "msg/Getallframe.msg"
  "msg/Gripperpick.msg"
  "msg/Gripperset.msg"
  "msg/Handangle.msg"
  "msg/Handforce.msg"
  "msg/Handposture.msg"
  "msg/Handseq.msg"
  "msg/Handspeed.msg"
  "msg/Handstatus.msg"
  "msg/Jointcurrent.msg"
  "msg/Jointenflag.msg"
  "msg/Jointerrclear.msg"
  "msg/Jointerrorcode.msg"
  "msg/Jointpos.msg"
  "msg/Jointposcustom.msg"
  "msg/Jointposeeuler.msg"
  "msg/Jointspeed.msg"
  "msg/Jointteach.msg"
  "msg/Jointtemperature.msg"
  "msg/Jointvoltage.msg"
  "msg/Liftheight.msg"
  "msg/Liftspeed.msg"
  "msg/Liftstate.msg"
  "msg/Movec.msg"
  "msg/Movej.msg"
  "msg/Movejp.msg"
  "msg/Movel.msg"
  "msg/Ortteach.msg"
  "msg/Posteach.msg"
  "msg/Rmerr.msg"
  "msg/Rmplusbase.msg"
  "msg/Rmplusstate.msg"
  "msg/Setforceposition.msg"
  "msg/Setrealtimepush.msg"
  "msg/Sixforce.msg"

  # srv
  srv/GetDH.srv
  srv/ModbusControl.srv
  srv/jodell_evs08/ActivateGripper.srv
  srv/jodell_evs08/CloseModbus.srv
  srv/jodell_evs08/DeactivateGripper.srv
  srv/jodell_evs08/InitializeGripper.srv
  srv/jodell_evs08/GetParasGripper.srv

  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()