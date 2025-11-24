'''
Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
Date: 2025-08-27 12:16:09
LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
LastEditTime: 2025-09-14 20:20:43
FilePath: /ros2_rm_ws/src/ros2_rm_robot-humble/rm_jodell_evs08_control_py/launch/gripper_control.launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
from launch import LaunchDescription
from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='rm_jodell_evs08_control_py',
#             executable='gripper_control_server',
#             name='jodell_evs08_control_server',
#             output='screen',
#             parameters=[
#                 {'robot_ip': '169.254.247.18'},
#                 {'robot_port': 8080}
#             ]
#         )
#     ])


def generate_launch_description():
    # 左臂夹爪控制服务节点
    left_gripper_node = Node(
        package='rm_jodell_evs08_control_py',
        executable='gripper_control_server',
        name='jodell_evs08_control_server',
        namespace='left_arm',
        output='screen',
        parameters=[
            # {'robot_ip': '169.254.247.18'}, # Lab429,realman IP
            {'robot_ip': '192.168.1.19'}, # Lab433,realman IP
            {'robot_port': 8080},
            {'register_address': 1000},
            {'comm_port': 1},
            {'device_address': 9}
        ]
    )
    
    # 右臂夹爪控制服务节点
    right_gripper_node = Node(
        package='rm_jodell_evs08_control_py',
        executable='gripper_control_server',
        name='jodell_evs08_control_server',
        namespace='right_arm',
        output='screen',
        parameters=[
            # {'robot_ip': '169.254.247.19'},
            {'robot_ip': '192.168.1.18'},
            {'robot_port': 8080},
            {'register_address': 1000},
            {'comm_port': 1},
            {'device_address': 9}
        ]
    )

    return LaunchDescription([
        left_gripper_node,
        right_gripper_node
    ])