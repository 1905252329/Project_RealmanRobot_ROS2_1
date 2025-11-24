import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    
    # 方案1：使用参数文件启动双臂规划器
    '''
    ## 声明参数
    left_arm_namespace = DeclareLaunchArgument(
        'left_arm_namespace',
        default_value='left_arm',
        description='左臂命名空间'
    )  
    right_arm_namespace = DeclareLaunchArgument(
        'right_arm_namespace',
        default_value='right_arm',
        description='右臂命名空间'
    )

    ## 获取参数值
    left_ns = LaunchConfiguration('left_arm_namespace')
    right_ns = LaunchConfiguration('right_arm_namespace')  
    ## 左臂驱动组（注释掉的备选方案）
    left_arm_group = GroupAction([
        PushRosNamespace(left_ns),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rm_driver'), 'launch', 'rm_75_driver.launch.py')
            ),
            # launch_arguments={
            #     'arm_ip': '169.254.247.18'
            # }.items()
        )
    ])
    # 右臂驱动组（注释掉的备选方案）
    right_arm_group = GroupAction([
        PushRosNamespace(right_ns),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rm_driver'), 'launch', 'rm_75_driver.launch.py')
            ),
            # launch_arguments={
            #     'arm_ip': '169.254.247.19'
            # }.items()
        )
    ])

    '''

    # 方案2：使用参数启动双臂驱动
    ## 左臂驱动
    left_arm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rm_driver'), 'launch', 'rm_dual_arm_driver.launch.py')
        )
    )
    ## 右臂驱动
    right_arm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rm_driver'), 'launch', 'rm_dual_arm_driver.launch.py')
        )
    )

    # 参数文件路径
    params_file = os.path.join(get_package_share_directory('rm_motion_planner'),'config', 'rm_dual_arm_sclerp_planner_params.yaml')
  
    # 双臂规划器节点
    dual_arm_planner = Node(
        package='rm_motion_planner',
        executable='rm_dual_arm_sclerp_planner_node',
        name='rm_dual_arm_sclerp_planner_node',
        output='screen',
        parameters=[params_file]
    )

    # 返回launch描述
    return LaunchDescription([
        # 方案一：使用命名空间，注释掉的备选方案
        # left_arm_namespace,
        # right_arm_namespace,
        # 方案二：使用参数启动双臂驱动
        left_arm_driver,
        right_arm_driver,
        dual_arm_planner
    ])