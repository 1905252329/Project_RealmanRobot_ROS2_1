'''
Author: hawrkchen
Date: 2025-09-17 09:28:54
LastEditTime: 2025-09-28 16:29:50
Description: 
FilePath: /ros2_rm_ws/src/ros2_rm_robot-humble/rm_driver/launch/rm_75_right_driver.launch.py
'''

# 在双臂系统中为右臂
import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    arm_config = os.path.join(
        get_package_share_directory('rm_driver'),
        'config',
        'rm_75_right_config.yaml'
        )

    with open(arm_config,'r') as f:
        params = yaml.safe_load(f)["rm_driver"]["ros__parameters"]

    return LaunchDescription([

        Node(
            package= "rm_driver",       # 功能包。
            executable= "rm_driver",    # 节点。
            parameters=[params],        # 加载的参数配置
            namespace="right_arm",      # 明确指定命名空间
            output='screen'
            )

    ])

# 通过ROS2日志系统实现结构化日志输出
# def generate_launch_description():
#    #参数加载
#     arm_config = os.path.join(
#         get_package_share_directory('rm_driver'),
#         'config',
#         'rm_75_right_config.yaml'
#         )

#     with open(arm_config,'r') as f:
#         params = yaml.safe_load(f)["rm_driver"]["ros__parameters"]

#     # 创建带日志前缀的节点
#     driver_node = Node(
#         package="rm_driver",
#         executable="rm_driver",
#         parameters=[params],
#         namespace="right_arm",
#         output='screen',
#         # 通过环境变量设置日志格式
#         emulate_tty=True,  # 确保颜色输出
#         prefix=['sh', '-c', 'echo -e "\033[35m[DRIVER_NODE]\033[0m $0 "$@"'],
#         arguments=['__log_level:=info']  # 设置默认日志级别
#     )

#     return LaunchDescription([
#         # 启动前日志（通过ROS2日志系统）
#         ExecuteProcess(
#             cmd=['ros2', 'topic', 'pub', '/right_arm/log', 'std_msgs/String', 
#                  '{"data": "\033[36m[SYSTEM]\033[0m 🚀初始化右臂驱动系统"}'],
#             output='screen'
#         ),
#         driver_node
#     ])