'''
Author: hawrkchen
Date: 2025-09-17 09:28:54
LastEditTime: 2025-09-28 14:28:34
Description: 
FilePath: /ros2_rm_ws/src/ros2_rm_robot-humble/rm_motion_planner/launch/rm_sclerp_planner_node_rightArm.launch.py
'''


import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (  # 新增导入ExecuteProcess
    DeclareLaunchArgument, 
    GroupAction, 
    IncludeLaunchDescription,
    ExecuteProcess 
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, FindExecutable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    # 声明是否启动驱动节点的参数
    launch_driver_arg = DeclareLaunchArgument(
        'launch_driver',
        default_value='true',
        description='Whether to launch the driver node'
    )

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rm_driver'), 
                         'launch', 
                         'rm_75_right_driver.launch.py')
        ]),
        condition=IfCondition(LaunchConfiguration('launch_driver'))
    )
    """

    # 获取参数
    start_joint_positions_arg = DeclareLaunchArgument(
        'start_joint_positions',
        default_value='',  # 空默认值
        description='起始关节角度，用逗号分隔'
    )
    
    target_joint_positions_arg = DeclareLaunchArgument(
        'target_joint_positions',
        default_value='',  # 空默认值
        description='目标关节角度，用逗号分隔'
    )
    
    # 使用ROS 2的方式查找rm_description包路径
    default_urdf_path = os.path.join(
        get_package_share_directory('rm_description'),
        'urdf',
        'rm_75_6fb.urdf'
    )
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf_path,
        description='URDF文件路径'
    )

    # 获取参数值
    start_joint_positions = LaunchConfiguration('start_joint_positions')
    target_joint_positions = LaunchConfiguration('target_joint_positions')
    urdf_path = LaunchConfiguration('urdf_path')

    # 参数文件路径
    params_file = os.path.join(
        get_package_share_directory('rm_motion_planner'),
        'config',
        'planner_params_rightArm.yaml'
    )

    # 打印参数文件路径
    print("参数文件路径:", params_file)

    # 读取参数文件
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['rm_sclerp_planner_node_rightArm']['ros__parameters']

    # 创建节点组
    planner_node_group = GroupAction([Node(
            package='rm_motion_planner',
            executable='rm_sclerp_planner_node_rightArm',
            name='rm_sclerp_planner_node_rightArm',
            output='screen',
            parameters=[params], 
            namespace='right_arm'   
        )
    ])

    # 返回launch描述
    return LaunchDescription([
        # launch_driver_arg,
        # driver_launch,
        start_joint_positions_arg,
        target_joint_positions_arg,
        urdf_path_arg,
        planner_node_group,
        ExecuteProcess(
            cmd=['bash', '-c', 'echo "\033[32m[INFO]\033[0m 机械臂规控节点已启动"'],
            output='screen'
        )
    ])