'''
Author: Likang25
Date: 2025-09-17 09:28:54
LastEditTime: 2025-09-17 16:45:57
Description: 

'''

import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    rm_75_left_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_driver')),'launch', 'rm_75_left_driver.launch.py'))
    )

    rm_75_right_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_driver')),'launch', 'rm_75_right_driver.launch.py'))
    )

    rm_sclerp_planner_node_leftArm = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_motion_planner')),'launch', 'rm_sclerp_planner_node_leftArm.launch.py'))
    )

    rm_sclerp_planner_node_rightArm = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_motion_planner')),'launch', 'rm_sclerp_planner_node_rightArm.launch.py'))
    )

    jodell_gripper_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_jodell_evs08_control_py')),'launch', 'gripper_control.launch.py'))
    )

    return LaunchDescription([
    rm_75_left_driver,
    rm_75_right_driver,
    rm_sclerp_planner_node_leftArm,
    rm_sclerp_planner_node_rightArm,
    jodell_gripper_control
    ])