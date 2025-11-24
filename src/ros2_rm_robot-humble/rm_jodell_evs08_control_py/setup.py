'''
Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
Date: 2025-08-27 12:14:34
LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
LastEditTime: 2025-08-30 20:51:38
FilePath: /ros2_rm_ws/src/ros2_rm_robot-humble/rm_jodell_evs08_control_py/setup.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''

from setuptools import setup
from glob import glob
import os

package_name = 'rm_jodell_evs08_control_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ],
    install_requires=['setuptools', 'Robotic_Arm'],
    zip_safe=False,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Python control package for Jodell EVS08 gripper',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'gripper_control_server = rm_jodell_evs08_control_py.gripper_control_server:main',
        ],
    },

)