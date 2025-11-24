# 运动规划器

## 开发思路
1. 创建一个运动规划器节点，该节点接收目标点位信息，并调用规划算法进行路径规划。
2. 创建一个规划算法，该算法接收目标点位信息，并返回规划路径。
3. 创建一个路径平滑算法，该算法接收规划路径，并返回平滑后的路径。
4. 创建一个路径发布器，该发布器将平滑后的路径发布到指定话题
5. 创建一个路径可视化工具，该工具将平滑后的路径可视化显示在RViz中。
6. 创建一个路径执行器，该执行器接收平滑后的路径，并执行路径规划。
7. 创建一个路径评估器，该评估器接收执行后的路径，并评估其是否满足要求。
8. 创建一个路径优化工具，该工具将路径优化后的结果发布到指定话题。
9. 创建一个路径优化可视化工具，该工具将路径优化后的结果可视化显示在RViz中。
10. 创建一个路径规划工具，该工具将路径规划后的结果发布到指定话题。

## 功能模块：
1. rm_sclerp_planner 规划阶段：负责高层次的运动规划
   
   （1）用 sclerp 进行路径规划(用 rrt 进行路径优化),生成轨迹点序列

   （2）通过/rm_group_controller/follow_joint_trajectory动作发送给rm_control

2. rm_control 处理阶段:负责轨迹插值和细分成低级命令
   
   （1）接收轨迹点并进行插值处理

   （2）使用定时器以20ms周期将轨迹点逐一发布到 /rm_driver/movej_canfd_cmd 话题

   （3）每个轨迹点是一个Jointpos消息，包含6或7个关节角度值

3. rm_driver 控制阶段:负责与硬件通信，将命令发送给机械臂
   
   （1）订阅/rm_driver/movej_canfd_cmd话题

   （2）接收每个轨迹点并转换为角度值（弧度转角度）

   （3）调用底层API rm_movej_canfd将命令发送给机械臂

   （4）机械臂执行相应的关节运动



rm_motion_planner 功能包类似 rm_moveit2_config 功能包，用于规划运动轨迹。包的组成初步如下
(1) rm_75_config/config/motion_controller.yaml : 包含 /rm_group_controller/follow_joint_trajectory 

(2) include:?
    ?: 是否要将kinlib 和 sclerp_motion_plannner 拷贝进来

(3) src/rm_sclerp_planner_test.cpp

* 运动学求解器
* 规划器

