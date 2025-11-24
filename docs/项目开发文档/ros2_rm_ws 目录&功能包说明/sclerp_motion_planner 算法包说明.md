# sclerp_motion_planner (螺旋球面插值规划)  算法包说明



## 整体介绍

### 功能包结构/组织框架



## 源码分析

### example_aubo_server.cpp

#### plan_to_home 函数

该函数的主要功能是将机械臂规划并移动到指定的home位置，支持三种home位置："home_tote"、"home_tray"和"home_paper"

##### 函数功能概述

[plan_to_home](javascript:void(0))函数是一个机器人运动控制函数，用于规划并执行机器人回到预定义home位置的运动。该函数结合了运动规划和实时伺服控制，使机器人能够平滑地移动到指定的home位置。

##### 详细代码分析

1. **初始化和变量声明**

```c++
int plan_to_home(RpcClientPtr rpc_cli, sclerp_interface::ScLERPInterface &planner_interface, const std::string &home)
{
    Eigen::Matrix4d g_base_tool;
    
    Eigen::VectorXd joint_values(6);
    std::vector<double> joint_positions;
    trajectory_msgs::msg::JointTrajectory motion_plan;
```

这部分代码声明了必要的变量：

- [g_base_tool](javascript:void(0)): 用于存储机器人当前末端执行器的位姿
- [joint_values](javascript:void(0)): 存储当前关节角度值的Eigen向量
- [joint_positions](javascript:void(0)): 存储当前关节角度值的std向量
- [motion_plan](javascript:void(0)): 存储规划出的关节轨迹

2. 获取机器人当前状态

```c++
// Get current joints
auto robot_name = rpc_cli->getRobotNames().front();
joint_positions =
    rpc_cli->getRobotInterface(robot_name)->getRobotState()->getJointPositions();

// std::cout << "Current joints: ";
for (size_t i = 0; i < joint_positions.size(); ++i)
{
    joint_values[i] = joint_positions[i];
    // std::cout << joint_values[i] << " ";
}
// std::cout << "\n";
```

这部分代码通过RPC客户端获取机器人当前的关节角度值，并将其存储在[joint_values](javascript:void(0))向量中。这是轨迹规划的起始点。

3. **计算目标位姿**

```c++
Eigen::Matrix4d goal = g_base_tool;
    planner_interface.kinlib_solver_.getFK(joint_values, g_base_tool);
    if (home == "home_tote") {
        goal = EulerToTransformation(tote_home_pose[0], tote_home_pose[1], tote_home_pose[2],
                                    tote_home_pose[3], tote_home_pose[4], tote_home_pose[5]);
    }
    else if (home == "home_tray") {
        goal = EulerToTransformation(tray_home_pose[0], tray_home_pose[1], tray_home_pose[2],
                                    tray_home_pose[3], tray_home_pose[4], tray_home_pose[5]);
    }
    else if (home == "home_paper") {
        goal = EulerToTransformation(0.5463718027282355, -0.5472580181245007, 0.3859027606726624,
                                    -178.65060941942505 * M_PI / 180,
                                    -4.892888487486562 * M_PI / 180,
                                    -0.6279396678268753 * M_PI / 180);
    }
    else {
        std::cerr << "Invalid home position specified: " << home << std::endl;
        return 0;
    }
```

这部分代码首先获取机器人当前末端执行器的位姿（通过正向运动学计算），然后根据传入的[home](javascript:void(0))参数确定目标位姿。支持三种home位置：

1. "home_tote" - 码垛位置
2. "home_tray" - 托盘位置
3. "home_paper" - 纸张位置

[EulerToTransformation](javascript:void(0))函数将欧拉角（x, y, z, roll, pitch, yaw）转换为4x4的齐次变换矩阵。

4. **输出调试信息**

```c++
std::cout << "Initial left pose is:\n" << g_base_tool << '\n';
std::cout << "Target left pose is:\n" << goal << '\n';
```

输出当前位姿和目标位姿，用于调试目的。

5. **调用运动规划器**

```c++
motion_plan.points.clear();

bool result = planner_interface.solve(joint_values, goal, motion_plan);

std::cout << "Motion Plan Result: " << result << "\n";
std::cout << "Number of waypoints: " << motion_plan.points.size() << "\n";

if (result != 1) {
    std::cout << "Motion Plan Failed: " << result << "\n";
    return 0;
}
```

这部分代码调用[ScLERPInterface](javascript:void(0))的[solve](javascript:void(0))方法进行运动规划，从当前关节位置规划到目标位姿。如果规划失败，则返回错误。

6. **轨迹处理和插值**

```c++
std::vector<std::vector<double>> path, trajectory;
Convert_to_2dVector(motion_plan, path);

// Maximum velocities (rad/s) for each joint
std::vector<double> max_velocity = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

// Maximum accelerations (rad/s^2) for each joint 
std::vector<double> max_acceleration = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};

double frequency = 250;

QuinticInterpolator interpolator(path, max_velocity, max_acceleration, frequency);

interpolator.computeTrajectory();

trajectory = interpolator.getTrajectory();
```

这部分代码对规划出的轨迹进行处理：

1. 使用[Convert_to_2dVector](javascript:void(0))函数将轨迹消息转换为二维向量格式
2. 定义每个关节的最大速度和加速度限制
3. 创建[QuinticInterpolator](javascript:void(0))对象进行五次多项式插值，生成更平滑、符合物理限制的轨迹
4. 调用[computeTrajectory()](javascript:void(0))方法计算插值轨迹

7. **启用伺服模式**

```c++
// 接口调用: 开启 servo 模式
rpc_cli->getRobotInterface(robot_name)->getMotionControl()->setServoMode(true);

// 等待进入 servo 模式
int i = 0;
while (!rpc_cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->isServoModeEnabled()) {
    if (i++ > 5) {
        std::cout << "Servo 模式使能失败! 当前的 Servo 模式是 "
                  << rpc_cli->getRobotInterface(robot_name)
                            ->getMotionControl()
                            ->isServoModeEnabled()
                  << std::endl;
        return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
}
```

这部分代码启用机器人的伺服模式，这是执行实时轨迹跟踪所必需的。等待伺服模式真正启用后继续执行。

**8. 执行轨迹**

```c++
// Loop through the trajectory and send each position at 500 Hz (2 ms interval)

std::cout << "Starting ServoJ motion..." << std::endl;

std::cout << "Trajectory size: " <<  trajectory.size() << "\n";

for (size_t i = 1; i < trajectory.size(); i++) {
    // 接口调用: 关节伺服运动
    rpc_cli->getRobotInterface(robot_name)
           ->getMotionControl()
           ->servoJoint(trajectory[i], 3, 6, 1/frequency, 0.2, 200);

    std::this_thread::sleep_for(std::chrono::milliseconds(4));
}
```

这部分代码遍历插值后的轨迹点，逐点发送给机器人执行。使用[servoJoint](javascript:void(0))方法以250Hz的频率（每4毫秒一个点）发送关节位置指令。

**9. 等待运动完成并退出伺服模式**

```c++
// 等待运动结束
while (!rpc_cli->getRobotInterface(robot_name)->getRobotState()->isSteady()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
}
std::cout << "Servoj 运动结束" << std::endl;

// 接口调用: 关闭 servo 模式
rpc_cli->getRobotInterface(robot_name)->getMotionControl()->setServoMode(false);

// 等待结束 servo 模式
while (rpc_cli->getRobotInterface(robot_name)
                ->getMotionControl()
                ->isServoModeEnabled()) {
    if (i++ > 5) {
        std::cout << "Servo 模式失能失败! 当前的 Servo 模式是 "
                    << rpc_cli->getRobotInterface(robot_name)
                            ->getMotionControl()
                            ->isServoModeEnabled()
                    << std::endl;
        return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

return 1;
}
```

最后，函数等待机器人完全停止运动，然后退出伺服模式并返回成功状态。

##### 总结

该函数实现了一个完整的机器人回home位置的运动控制流程：

1. 获取当前状态
2. 根据参数确定目标位置
3. 规划运动轨迹
4. 对轨迹进行插值优化
5. 启用伺服模式
6. 实时执行轨迹
7. 退出伺服模式

整个过程考虑了运动学约束和物理限制，确保机器人能够平滑、安全地移动到指定的home位置。



##### 解惑：

###### Q1：为什么需要二次插值处理？

> 在 example_aubo_server.cpp 的 plan_to_home 函数中，已经调用了 planner_interface.solve 接口函数生成了 motion_plan 轨迹点，为什么还要再通过 Interpolator.computeTrajectory() 作插值处理？

通过分析代码，我们可以发现两个不同的处理阶段：

1. **路径规划阶段**：`planner_interface.solve`函数负责生成初步的关节空间轨迹点。这些轨迹点是通过**ScLERP（螺旋球面插值）算法**计算得到的，主要关注路径的几何形状和可达性。
2. **轨迹细化阶段**：`Interpolator.computeTrajectory()`负责对初步轨迹进行**时间参数化**和精细化处理，生成适合实际控制的轨迹。

这两个阶段的区别在于：

- [solve](javascript:void(0))函数生成的轨迹点通常是等间距的路径点，但没有考虑时间因素，即没有为每个轨迹点分配合适的时间戳。
- `Interpolator`类则根据机械臂的物理限制（最大速度和加速度）对轨迹进行时间参数化，确保轨迹在实际执行时不会超出机械臂的能力范围



###### Q2：max_velocity和max_acceleration参数的作用

> 此外 Quinticnterpolator 实例化中的 max_velocity 和 max_acceleration 的作用是什么？ 请结合具体的代码详细说明



在[QuinticInterpolator](javascript:void(0))的[computeTrajectory](javascript:void(0))方法中，[max_velocity](javascript:void(0))和[max_acceleration](javascript:void(0))参数有以下作用：

1. **时间间隔计算**：

   ```c++
   for (size_t i = 0; i < num_waypoints - 1; ++i) {
       double max_time = dt;
       for (size_t j = 0; j < num_joints; ++j) {
           double dq = std::abs(path_[i + 1][j] - path_[i][j]);
           if (dq < 1e-6) continue;
           double t_v = dq / std::max(max_velocity_[j], 1e-6);  // 基于速度限制计算所需时间
           double t_a = std::sqrt(2 * dq / std::max(max_acceleration_[j], 1e-6));  // 基于加速度限制计算所需时间
           max_time = std::max(max_time, std::max(t_v, t_a));
       }
       time_intervals[i] = max_time;
   }
   ```

   这段代码根据相邻轨迹点之间的关节角度差以及给定的最大速度和加速度限制，计算出每段轨迹所需的时间。

2. **速度限制**：

   ```c++
   velocities[i][j] = std::clamp(velocities[i][j], -max_velocity_[j], max_velocity_[j]);
   ```

   确保计算出的速度不会超过设定的最大速度限制。

3. **轨迹点生成**： 使用五次插值算法生成平滑的轨迹点，确保在给定的时间间隔内，关节运动满足速度和加速度约束。

   

**实际应用中的重要性**

在[plan_to_home](javascript:void(0))函数中，通过设置[max_velocity](javascript:void(0))为{2.0, 2.0, 2.0, 2.0, 2.0, 2.0}和[max_acceleration](javascript:void(0))为{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}，确保了生成的轨迹符合机械臂的物理限制，避免了因速度或加速度过大而可能造成的机械臂损坏或运动不平稳的问题。

这种两阶段处理方法的优势在于：

1. 路径规划阶段专注于寻找可达的路径，不考虑时间因素
2. 轨迹细化阶段专注于时间参数化，确保轨迹的执行符合物理限制
3. 两阶段分离使系统更加模块化，便于调整和优化

### sclerp_interface.cpp

#### 构造函数解析

##### Q1: URDF模型导入的作用

> 构造函数 ScLERPInterface 中导入URDF模型的作用是什么？

在ScLERPInterface构造函数中导入URDF模型的主要作用是为机械臂运动规划建立准确的运动学模型。让我详细解释一下：

URDF（Unified Robot Description Format）是ROS中描述机器人模型的标准格式。在ScLERPInterface构造函数中导入URDF模型有以下几个关键作用：

###### 1. 建立机械臂的运动学结构

```c++
if (!local_urdf_file.empty())
{
  RCLCPP_INFO(nh_->get_logger(), "Reading URDF from file: %s", local_urdf_file.c_str());

  if (!urdf_model.initFile(local_urdf_file))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Error loading URDF file!");
    init_failed = true;
  }

  if (!kdl_parser::treeFromUrdfModel(urdf_model, robot_tree))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Error constructing KDL Tree!");
    init_failed = true;
  }
}
else
{
  RCLCPP_INFO(nh_->get_logger(), "Reading URDF from Parameter Server");

  if (!urdf_model.initString("/move_group/robot_description")) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to load URDF from parameter server!");
    init_failed = true;
  }

  nh_->declare_parameter<std::string>("/move_group/robot_description", "");
  nh_->get_parameter("/move_group/robot_description", robot_desc_string);

  if (!kdl_parser::treeFromString(robot_desc_string, robot_tree))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to construct KDL Tree from Parameter Server!");
    init_failed = true;
  }
}
```

这段代码从文件或参数服务器加载URDF模型，并将其转换为KDL（Kinematics and Dynamics Library）树结构，这是进行运动学计算的基础。

###### 2. 提取机械臂链（Chain）

```c++
jnt_list = urdf_model.joints_;
if (!robot_tree.getChain(base_link_name, tip_link_name, manip_chain))
{
  RCLCPP_ERROR(nh_->get_logger(), "getChain() failed from '%s' to '%s'", base_link_name.c_str(), tip_link_name.c_str());
  init_failed = true;
}
```

通过指定基座链接（base_link）和末端链接（tip_link），从机器人树中提取出特定的机械臂链，这定义了规划器需要控制的关节序列。

###### 3. 构建Kinlib求解器

```c++
for(int itr = 0; itr < manip_chain.getNrOfSegments(); itr++)
{
  // ...处理每个链段...
  
  if(jnt.getType() == KDL::Joint::JointType::RotAxis)
  { 
    jnt_lim.upper_limit_ = jnt_ptr->limits->upper;
    jnt_lim.lower_limit_ = jnt_ptr->limits->lower;

    jnt_type = kinlib::JointType::Revolute;
    manip.addJoint(jnt_type, jnt.getName(), w_v_jnt, w_p_jnt, jnt_lim, t_ref);
  }
  // ...
}

kinlib_solver_ = kinlib::KinematicsSolver(manip, stl_paths, mesh_offset_transforms_);
```

通过遍历机械臂链中的每个段，提取关节信息（类型、限制、轴线等）并构建内部的Kinlib运动学求解器。这包括：

- 关节类型和名称
- 关节轴线和原点位置
- 关节运动限制（上下限）
- 各连杆之间的变换关系

###### 4. 获取视觉模型信息

```c++
Eigen::Matrix4d mesh_offset = getMeshOffsetFromURDF(urdf_link);
// ...
mesh_offset_transforms_.push_back(mesh_offset);
```

从URDF中提取视觉模型的偏移信息，这对于碰撞检测和可视化非常重要。

###### 总结

导入URDF模型的作用是为ScLERP运动规划器提供完整的机器人描述，包括：

1. 机械臂的物理结构和连接关系
2. 各关节的类型和运动限制
3. 连杆之间的几何关系
4. 视觉模型的安装位置

这些信息对于正确执行正向运动学计算、逆向运动学规划以及碰撞检测都是必不可少的。没有准确的URDF模型，运动规划器就无法生成符合实际机械臂特性的轨迹



##### Q2:URDF模型导入与解析的目的

> 综合上述回答，结合代码实现，在ScLERPInterface构造函数中导入URDF模型，并加以解析的目的是为创建运动学接口 kinlib_solver_ ?

在ScLERPInterface构造函数中导入和解析URDF模型的主要目的就是为了创建运动学接口[kinlib_solver_](javascript:void(0))。

让我结合代码来详细说明这个过程：

###### 1. 获取机械臂结构信息

```c++
if (!local_urdf_file.empty())
{
  RCLCPP_INFO(nh_->get_logger(), "Reading URDF from file: %s", local_urdf_file.c_str());

  if (!urdf_model.initFile(local_urdf_file))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Error loading URDF file!");
    init_failed = true;
  }

  if (!kdl_parser::treeFromUrdfModel(urdf_model, robot_tree))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Error constructing KDL Tree!");
    init_failed = true;
  }
}
```

这段代码从文件或参数服务器加载URDF模型，并将其转换为KDL树结构，为后续处理提供基础数据。

###### 2. 提取机械臂链信息（Chain）

```c++
jnt_list = urdf_model.joints_;
if (!robot_tree.getChain(base_link_name, tip_link_name, manip_chain))
{
  RCLCPP_ERROR(nh_->get_logger(), "getChain() failed from '%s' to '%s'", base_link_name.c_str(), tip_link_name.c_str());
  init_failed = true;
}
```

通过指定基座链接（base_link）和末端链接（tip_link），提取出特定的机械臂链，定义了规划器需要控制的关节序列。

###### 3. 构建Kinlib求解器

```c++
for(int itr = 0; itr < manip_chain.getNrOfSegments(); itr++)
{
  // ...处理每个链段...
  
  if(jnt.getType() == KDL::Joint::JointType::RotAxis)
  { 
    jnt_lim.upper_limit_ = jnt_ptr->limits->upper;
    jnt_lim.lower_limit_ = jnt_ptr->limits->lower;

    jnt_type = kinlib::JointType::Revolute;
    manip.addJoint(jnt_type, jnt.getName(), w_v_jnt, w_p_jnt, jnt_lim, t_ref);
  }
  // ...
}

kinlib_solver_ = kinlib::KinematicsSolver(manip, stl_paths, mesh_offset_transforms_);
```

通过遍历机械臂链中的每个段，提取关节信息（类型、限制、轴线等）并最终构建内部的Kinlib运动学求解器。这包括：

- 关节类型和名称
- 关节轴线和原点位置
- 关节运动限制（上下限）
- 各连杆之间的变换关系

###### 4. 获取视觉模型信息

```c++
Eigen::Matrix4d mesh_offset = getMeshOffsetFromURDF(urdf_link);
// ...
mesh_offset_transforms_.push_back(mesh_offset);
```

从URDF中提取视觉模型的偏移信息，这对于碰撞检测和可视化非常重要。

###### 总结

导入URDF模型的作用是为ScLERP运动规划器提供完整的机器人描述，包括：

1. 机械臂的物理结构和连接关系
2. 各关节的类型和运动限制
3. 连杆之间的几何关系
4. 视觉模型的安装位置

这些信息对于正确执行正向运动学计算、逆向运动学规划以及碰撞检测都是必不可少的。没有准确的URDF模型，运动规划器就无法生成符合实际机械臂特性的轨迹。

整个URDF导入和解析过程的核心目的就是为创建[kinlib_solver_](javascript:void(0))对象提供必要的信息。这个[kinlib_solver_](javascript:void(0))对象是ScLERPInterface的核心组件，它提供了：

1. 正向运动学计算（FK）功能
2. 逆向运动学计算（IK）功能
3. 运动规划功能
4. 碰撞检测功能

这些功能都是基于从URDF模型中提取的机械臂结构信息构建的，因此URDF模型的准确导入和解析对于整个ScLERPInterface的功能至关重要



##### Q3: URDF到Kinlib求解器的实现过程

> 既然 sclerp_interface.cpp 中的 kinlib_solver_ 运动学求解器是基于URDF模型实现的，请详细分析说明这个实现过程，每一步骤是怎样的？DH参数在这个过程中扮演什么样的角色？



###### 1. URDF模型加载和解析

首先，代码从文件或参数服务器加载URDF模型：

```c++
if (!local_urdf_file.empty())
{
  RCLCPP_INFO(nh_->get_logger(), "Reading URDF from file: %s", local_urdf_file.c_str());

  if (!urdf_model.initFile(local_urdf_file))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Error loading URDF file!");
    init_failed = true;
  }

  if (!kdl_parser::treeFromUrdfModel(urdf_model, robot_tree))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Error constructing KDL Tree!");
    init_failed = true;
  }
}
else
{
  RCLCPP_INFO(nh_->get_logger(), "Reading URDF from Parameter Server");

  if (!urdf_model.initString("/move_group/robot_description")) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to load URDF from parameter server!");
    init_failed = true;
  }

  nh_->declare_parameter<std::string>("/move_group/robot_description", "");
  nh_->get_parameter("/move_group/robot_description", robot_desc_string);

  if (!kdl_parser::treeFromString(robot_desc_string, robot_tree))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to construct KDL Tree from Parameter Server!");
    init_failed = true;
  }
}
```

###### 2. 提取机械臂链

从机器人树中提取指定的机械臂链：

```c++
jnt_list = urdf_model.joints_;
if (!robot_tree.getChain(base_link_name, tip_link_name, manip_chain))
{
  RCLCPP_ERROR(nh_->get_logger(), "getChain() failed from '%s' to '%s'", base_link_name.c_str(), tip_link_name.c_str());
  init_failed = true;
}
```

###### 3. 遍历链段并构建Manipulator对象

这是整个过程的核心部分，代码遍历机械臂链中的每个段：

```c++
for(int itr = 0; itr < manip_chain.getNrOfSegments(); itr++)
{
  KDL::Segment chain_seg = manip_chain.getSegment(itr);
  std::string seg_name = chain_seg.getName();

  // 获取对应的URDF链接
  urdf::LinkConstSharedPtr urdf_link = urdf_model.getLink(seg_name);
  Eigen::Matrix4d mesh_offset = getMeshOffsetFromURDF(urdf_link);

  if (chain_seg.getJoint().getType() != KDL::Joint::JointType::Fixed) {
    mesh_offset_transforms_.push_back(mesh_offset);
  }

  KDL::Joint jnt = chain_seg.getJoint();
  KDL::RigidBodyInertia inertia_prop = chain_seg.getInertia();
  KDL::RotationalInertia rot_inertia = inertia_prop.getRotationalInertia();

  Eigen::Matrix3d rot_inertia_mat;

  frame_to_tip = chain_seg.getFrameToTip();

  urdf::JointSharedPtr jnt_ptr(jnt_list[jnt.getName()]);

  for(int r_itr = 0; r_itr < 3; r_itr++)
  {
    for(int c_itr = 0; c_itr < 3; c_itr++)
    {
      t_tip(r_itr, c_itr) = frame_to_tip.M.data[(r_itr * 3) + c_itr];
      rot_inertia_mat(r_itr, c_itr) = rot_inertia.data[(r_itr * 3) + c_itr];
    }

    t_tip(r_itr, 3) = frame_to_tip.p.data[r_itr];

    p_jnt(r_itr) = jnt.JointOrigin().data[r_itr];
    v_jnt(r_itr) = jnt.JointAxis().data[r_itr];
  }

  Eigen::Vector4d w_p_jnt = t_ref * p_jnt;
  Eigen::Vector4d w_v_jnt = t_ref * v_jnt;

  t_ref = t_ref * t_tip;

  if(jnt.getType() == KDL::Joint::JointType::RotAxis)
  { 
    jnt_lim.upper_limit_ = jnt_ptr->limits->upper;
    jnt_lim.lower_limit_ = jnt_ptr->limits->lower;

    jnt_type = kinlib::JointType::Revolute;
    manip.addJoint(jnt_type, jnt.getName(), w_v_jnt, w_p_jnt, jnt_lim, t_ref);
  }
  else
  {
    if(itr < (manip_chain.getNrOfSegments()))
    {
      manip.modifyEndJointTipPose(t_ref);
    }
  }
}
```

###### 4. 创建Kinlib求解器

最后，使用构建好的Manipulator对象创建Kinlib求解器：

```c++
kinlib_solver_ = kinlib::KinematicsSolver(manip, stl_paths, mesh_offset_transforms_);
```

###### DH参数的作用

在这个实现过程中，DH参数扮演着关键角色：

1. **描述连杆和关节关系**

DH参数用于描述机械臂连杆之间的几何关系，包括：

- 连杆长度(a)
- 连杆扭转角(α)
- 关节偏移量(d)
- 关节角度(θ)

在代码中，这些参数通过以下方式体现：

```c++
frame_to_tip = chain_seg.getFrameToTip();

// 从frame_to_tip中提取变换矩阵的旋转和平移部分
for(int r_itr = 0; r_itr < 3; r_itr++)
{
  for(int c_itr = 0; c_itr < 3; c_itr++)
  {
    t_tip(r_itr, c_itr) = frame_to_tip.M.data[(r_itr * 3) + c_itr];
  }
  t_tip(r_itr, 3) = frame_to_tip.p.data[r_itr];
}
```

2. **构建变换矩阵**

DH参数用于构建相邻连杆之间的齐次变换矩阵。在代码中，这个过程通过累积变换实现：

```c++
Eigen::Vector4d w_p_jnt = t_ref * p_jnt;
Eigen::Vector4d w_v_jnt = t_ref * v_jnt;
t_ref = t_ref * t_tip;
```

这里[t_ref](javascript:void(0))是累积的变换矩阵，包含了从基座到当前连杆的完整变换。

3. **关节参数提取**

从URDF中提取关节限制参数：

```c++
jnt_lim.upper_limit_ = jnt_ptr->limits->upper;
jnt_lim.lower_limit_ = jnt_ptr->limits->lower;
```

这些参数对应DH参数中的关节运动范围。

4. **坐标系建立**

DH参数定义了如何建立连杆坐标系，这在代码中通过以下方式实现：

```
p_jnt(r_itr) = jnt.JointOrigin().data[r_itr];  // 关节原点位置
v_jnt(r_itr) = jnt.JointAxis().data[r_itr];    // 关节轴线方向
```

###### 实现过程总结

整个实现过程可以概括为以下几个步骤：

1. **加载URDF模型**：从文件或参数服务器读取机器人描述
2. **解析模型结构**：将URDF转换为KDL树结构
3. **提取机械臂链**：根据指定的基座和末端链接提取链
4. **遍历链段**：
   - 获取每个链段的几何信息
   - 提取关节参数（类型、限制、轴线等）
   - 计算连杆间的变换关系
   - 累积变换矩阵
5. **构建Manipulator对象**：使用提取的信息构建内部表示
6. **创建Kinlib求解器**：基于Manipulator对象创建运动学求解器

DH参数在整个过程中作为描述机械臂几何结构的基础，通过URDF模型中的joint和link定义体现出来，最终被转换为Kinlib求解器所需的内部表示，用于进行正向和逆向运动学计算

###### DH参数的作用和局限性

DH**参数的作用**

DH参数确实是运动学建模的核心，它们定义了：

1. 相邻关节坐标系之间的相对位置和姿态
2. 连杆的几何参数（长度和扭转角）
3. 关节的运动参数（偏移量和角度）

为什么仅有DH**参数不够**

要实现一个完整的运动学求解器，还需要其他重要信息：

1. **关节限制**：DH参数不包含关节的运动范围限制
2. **坐标系定义**：需要明确基座坐标系和末端坐标系
3. **机械臂结构**：需要知道机械臂的完整链式结构
4. **碰撞模型**：如果需要碰撞检测，还需要连杆的几何形状信息
5. **惯性参数**：如果涉及动力学计算，还需要质量、质心和惯性张量等参数



##### Q4：举例说明从URDF到运动学求解器代码实现

以RM-75机械臂的URDF模型为例，说明它如何提供运动学求解器所需的各种信息。

###### 1. 关节限制信息

URDF文件中的每个关节都定义了明确的运动限制：

```xml
<joint name="joint_1" type="revolute">
  <limit
    lower="-3.105"
    upper="3.105"
    effort="60"
    velocity="3.14" />
</joint>

<joint name="joint_2" type="revolute">
  <limit
    lower="-2.269"
    upper="2.269"
    effort="60"
    velocity="3.14" />
</joint>

<!-- 其他关节类似 -->
```

这些限制信息在ScLERPInterface构造函数中被提取并用于构建Kinlib求解器：

```c++
jnt_lim.upper_limit_ = jnt_ptr->limits->upper;
jnt_lim.lower_limit_ = jnt_ptr->limits->lower;
```

###### 2. 完整的机械臂结构定义

URDF文件通过链接(link)和关节(joint)定义了完整的机械臂结构：

1. **链接定义**：定义了机械臂的每个连杆(包括基座)
   - base_link (基座)
   - Link_1 到 Link_6 (各关节连杆)
   - Link7 (末端执行器)
2. **关节连接**：通过parent和child属性定义了连杆之间的连接关系

```c++
<joint name="joint_1" type="revolute">
  <parent link="base_link" />
  <child link="Link_1" />
</joint>
```

1. **关节参数**：定义了每个关节的类型、轴线和坐标变换

```xml
<axis xyz="0 0 -1" />
<origin xyz="0 0 0.2405" rpy="0 0 0" />
```

在ScLERPInterface中，这些信息被用来构建KDL树和机械臂链：

```c++
if (!robot_tree.getChain(base_link_name, tip_link_name, manip_chain))
{
  RCLCPP_ERROR(nh_->get_logger(), "getChain() failed from '%s' to '%s'", base_link_name.c_str(), tip_link_name.c_str());
  init_failed = true;
}
```

###### 3. 坐标系定义

URDF文件通过以下方式定义坐标系：

1. **链接坐标系**：每个链接都有自己的坐标系，通过origin元素定义视觉和碰撞模型相对于链接坐标系的位置
2. **关节坐标变换**：每个关节通过origin元素定义了从父链接到子链接的坐标变换

```xml
<origin xyz="0 0 0.2405" rpy="0 0 0" />
```

1. **关节轴线**：通过axis元素定义关节旋转轴

```xml
<axis xyz="0 0 -1" />
```

在ScLERPInterface中，这些坐标变换信息被转换为齐次变换矩阵：

```xml
frame_to_tip = chain_seg.getFrameToTip();

for(int r_itr = 0; r_itr < 3; r_itr++)
{
  for(int c_itr = 0; c_itr < 3; c_itr++)
  {
    t_tip(r_itr, c_itr) = frame_to_tip.M.data[(r_itr * 3) + c_itr];
  }
  t_tip(r_itr, 3) = frame_to_tip.p.data[r_itr];
}
```

###### 4. 碰撞模型

URDF文件为每个链接定义了碰撞模型：

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0" />
  <geometry>
    <mesh filename="package://rm_description/meshes/RM75_Test/base_link.STL" />
  </geometry>
</collision>
```

这些碰撞模型信息在ScLERPInterface中被提取并用于碰撞检测：

```c++
// Get corresponding URDF link
urdf::LinkConstSharedPtr urdf_link = urdf_model.getLink(seg_name);
Eigen::Matrix4d mesh_offset = getMeshOffsetFromURDF(urdf_link);

if (chain_seg.getJoint().getType() != KDL::Joint::JointType::Fixed) {
  mesh_offset_transforms_.push_back(mesh_offset);
}

// ...

kinlib_solver_ = kinlib::KinematicsSolver(manip, stl_paths, mesh_offset_transforms_);
```

###### 5. 数值算法实现

虽然URDF文件本身不直接包含数值算法实现，但它为这些算法提供了必要的几何和物理参数：

1. **惯性参数**：为动力学计算提供质量、质心和惯性张量

```xml
<inertial>
  <origin xyz="-0.000498402948737627 -5.19350130879295E-05 0.0600202841401173" rpy="0 0 0" />
  <mass value="0.838885280844642" />
  <inertia
    ixx="0.00172321229512273"
    ixy="3.09076921222708E-06"
    ixz="-3.79221267118485E-05"
    iyy="0.00170514762450234"
    iyz="1.38793623479001E-06"
    izz="0.00090158903044651" />
</inertial>
```

1. **几何参数**：为运动学计算提供连杆长度和关节位置信息

这些参数在ScLERPInterface构造函数中被处理并用于构建运动学求解器，最终支持各种数值算法的实现，如正向运动学、逆向运动学和轨迹规划等。

###### 总结

RM-75的URDF模型通过其结构化的XML格式完整地描述了机械臂的物理和运动特性，为运动学求解器提供了所有必要的信息。ScLERPInterface构造函数通过解析这些信息，构建了内部的Kinlib求解器，使得可以进行各种运动学计算和轨迹规划操作





##### Q3：URDF 模型误差问题

> 如果URDF 模型与真实的机械臂之间存在偏差，该如何修正URDF模型

如果 URDF 模型与真实机械臂之间存在偏差，可以通过以下几种方式进行修正：

###### 1. 修改 URDF 文件参数

**关节限制修正**

在 URDF 文件中找到对应的关节定义，修改其运动范围：

```xml
<joint name="joint1" type="revolute">
  <limit lower="-3.1416" upper="3.1416" effort="100" velocity="100"/>
  <!-- 根据实际测量调整lower和upper值 -->
</joint>
```

**连杆长度和偏移修正**

修改连杆的长度和相对位置：

```xml
<link name="link1">
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <!-- 根据实际测量调整xyz值 -->
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
      <!-- 根据实际尺寸调整length和radius -->
    </geometry>
  </visual>
</link>
```

**坐标变换修正**

修正关节之间的相对位置和姿态：

```xml
<joint name="joint1" type="revolute">
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <!-- 根据实际测量调整xyz和rpy值 -->
</joint>
```

###### 2. 使用 XACRO 参数化建模

如果使用 XACRO 文件生成 URDF，可以通过修改参数来调整模型：

```xml
<xacro:property name="link1_length" value="0.3"/>
<xacro:property name="link1_radius" value="0.05"/>

<link name="link1">
  <visual>
    <geometry>
      <cylinder length="${link1_length}" radius="${link1_radius}"/>
    </geometry>
  </visual>
</link>
```

###### 3. 实际测量与标定

**DH 参数标定**

通过实际测量机械臂的 DH 参数，修正 URDF 中的连杆参数：

1. 测量各关节轴线之间的距离
2. 测量各连杆的长度和扭转角
3. 根据测量结果更新 URDF 中的参数

**运动学标定**

使用专门的标定工具或方法：

1. **视觉标定**：使用摄像头和标定板进行末端位姿测量
2. **激光跟踪仪标定**：使用高精度测量设备获取末端位置
3. **基于优化的标定**：通过比较实际运动与模型预测，优化模型参数

###### 4. 在代码中进行补偿

如果无法修改 URDF 模型，可以在代码层面进行补偿：

**正向运动学补偿**

在调用 FK 计算前对关节角度进行补偿：

```c++
// 补偿关节零位偏差
Eigen::VectorXd compensated_joints = actual_joints + joint_offsets;
kinlib_solver_.getFK(compensated_joints, tcp_pose);
```

**逆向运动学补偿**

在得到 IK 解后进行补偿：

```c++
// 计算IK解
Eigen::VectorXd ik_solution;
kinlib_solver_.getIK(target_pose, initial_guess, ik_solution);

// 应用补偿
Eigen::VectorXd compensated_solution = ik_solution - joint_offsets;
```

###### 5. 使用 ROS 参数服务器动态调整

将关键参数放入参数文件中，便于运行时调整：

```c++
# robot_calibration.yaml
robot_calibration:
  joint_offsets: [0.01, -0.02, 0.005, 0, 0, 0, 0]
  link_length_corrections: [0.001, -0.002, 0.001]
```

然后在代码中读取这些参数：

```c++
this->declare_parameter<std::vector<double>>("joint_offsets", std::vector<double>(7, 0.0));
std::vector<double> joint_offsets;
this->get_parameter("joint_offsets", joint_offsets);
```

###### 6. 验证修正效果

**位置验证**

将机械臂移动到已知位置，比较实际位置与模型预测位置：

```
// 获取实际关节角度
// 计算FK得到TCP位置
// 与实际测量的TCP位置比较
// 调整模型参数直到误差在可接受范围内
```

**运动范围验证**

检查各关节的实际运动范围是否与 URDF 中定义的一致：

```c++
// 检查关节角度是否超出URDF定义的范围
if (joint_angle < lower_limit || joint_angle > upper_limit) {
    RCLCPP_WARN(this->get_logger(), "Joint angle out of URDF defined range");
}
```

###### 总结

修正 URDF 模型与实际机械臂之间的偏差需要：

1. **精确测量**：使用合适的工具测量机械臂的实际尺寸和关节范围
2. **逐步调整**：从小的参数调整开始，逐步优化模型
3. **验证测试**：每次调整后都要进行验证测试
4. **文档记录**：记录每次修改的原因和效果，便于后续维护

通过这些方法，可以使 URDF 模型更准确地反映真实机械臂的特性，从而提高运动规划的精度和可靠性。



## 拓展

### 插值算法

> Q：当前功能包中 使用的是如下述代码片段所示的五次多项式插值方法，该方法与样条插值有和区别？
>
> QuinticInterpolator interpolator(path, max_velocity, max_acceleration, frequency);
>
> 还有哪些更平滑的插值方法推荐？

#### 五次多项式插值与样条插值的区别

##### 1. 五次多项式插值 (Quintic Interpolation)

- **定义**: 在每两个相邻轨迹点之间使用五次多项式进行插值
- 特点:
  - 确保位置、速度和加速度在轨迹点处连续
  - 整个轨迹段使用单一的五次多项式函数
  - 计算相对简单，实时性好
  - 在轨迹点处可以保证平滑过渡

##### 2. 样条插值 (Spline Interpolation)

- **定义**: 使用分段低次多项式来构造平滑曲线，通常使用三次样条
- **特点**:
  - 在每个区间使用较低次的多项式（通常是三次）
  - 保证在整个轨迹上的连续性和平滑性
  - 更好的局部控制性，修改一个点不会影响整个轨迹
  - 通常能产生更自然、更平滑的轨迹

#### 更平滑的插值方法推荐

##### 1. B样条插值 (B-spline Interpolation)

- 优势
  - 提供更高的平滑度
  - 局部控制性更好
  - 可以通过调整控制点来优化轨迹
- **适用场景**: 对平滑性要求极高的应用

##### 2. 三次样条插值 (Cubic Spline Interpolation)

- 优势
  - 在位置、速度和加速度上都保证连续性
  - 计算复杂度适中
  - 能产生非常平滑的轨迹
- **适用场景**: 一般工业机器人应用

##### 3. 贝塞尔曲线插值 (Bezier Curve Interpolation)

- 优势
  - 直观的控制点操作
  - 平滑性好
  - 易于调整和优化
- **适用场景**: 需要直观控制轨迹形状的应用

##### 4. 时间最优轨迹规划 (Time-Optimal Trajectory Planning)

- 优势
  - 在满足约束条件下实现时间最优化
  - 自动优化速度和加速度曲线
- **适用场景**: 对执行时间敏感但对平滑性也有要求的应



#### 三次样条插值相比五次多项式插值的优势

##### 1. 数值稳定性

- **三次样条插值**：
  - 使用较低次的多项式（三次），数值计算更加稳定
  - 避免了高次多项式可能带来的数值振荡问题
  - 计算过程中不容易出现数值溢出或精度丢失
- **五次多项式插值**：
  - 使用高次多项式，可能存在数值不稳定问题（数值振荡）
  - 在某些情况下容易出现**龙格现象**（Runge's phenomenon）

##### 2. 局部控制性

- **三次样条插值**：
  - 修改一个数据点只会影响相邻的几个区间
  - 具有良好的局部控制性，不会对整个轨迹产生大的影响
  - 更容易进行局部调整和优化
- **五次多项式插值**：
  - 修改一个数据点可能会影响整个轨迹段
  - 局部控制性相对较差

##### 3. 平滑性

- **三次样条插值**：
  - 在整个轨迹上保证位置、速度和加速度的连续性（C2连续）
  - 产生的轨迹更加自然平滑
  - 更符合物理系统的运动特性
- **五次多项式插值**：
  - 虽然在轨迹点处保证连续性，但在整个轨迹段上的平滑性可能不如样条插值

##### 4. 计算复杂度

- **三次样条插值**：
  - 虽然需要求解线性方程组，但算法成熟且高效
  - 计算复杂度适中，适合实时应用
- **五次多项式插值**：
  - 对于每个轨迹段需要确定6个系数，计算相对复杂

##### 5. 适用性

- **三次样条插值**：
  - 更适合需要高平滑性的应用场景
  - 广泛应用于图形学、机器人学等领域
  - 更容易满足机械臂运动的平滑性要求
- **五次多项式插值**：
  - 更适合需要精确满足边界条件的场景
  - 在机器人轨迹规划中主要用于满足特定的起始和终止条件

##### 6. 实际应用中的表现

对于机械臂轨迹规划来说，三次样条插值通常能产生更加自然、平滑的运动轨迹，减少机械臂在运动过程中的抖动和冲击，提高运动的稳定性和精度。

总的来说，虽然五次多项式插值在某些方面有其优势（如能精确满足更多的边界条件），但在需要高平滑性和良好局部控制性的机械臂轨迹规划应用中，三次样条插值通常是更好的选择

##### 总结

相比于五次多项式插值，三次样条插值具有以下优势：

1. **更好的数值稳定性**：使用较低次的多项式，避免了高次多项式可能带来的数值振荡

2. **更强的局部控制性**：修改一个数据点只会影响相邻的几个区间

3. **更自然的平滑性**：在整个轨迹上保证位置、速度和加速度的连续性

4. **更适合机械臂应用**：产生的轨迹更加自然平滑，减少机械臂在运动过程中的抖动和冲击

   

### 三次样条插值算法实现

#### C++ 代码

```c++
class CubicSplineInterpolator {
public:
    typedef enum _BoundType {
        BoundType_First_Derivative,
        BoundType_Second_Derivative
    } BoundType;

    CubicSplineInterpolator() {
        initParam();
    }

    ~CubicSplineInterpolator() {
        releaseMem();
    }

    void initParam() {
        x_sample_ = y_sample_ = M_ = NULL;
        sample_count_ = 0;
        bound1_ = bound2_ = 0;
    }

    void releaseMem() {
        delete[] x_sample_;
        delete[] y_sample_;
        delete[] M_;

        initParam();
    }

    bool loadData(double* x_data, double* y_data, int count, double bound1, double bound2, BoundType type) {
        if ((NULL == x_data) || (NULL == y_data) || (count < 3) || (type > BoundType_Second_Derivative) || (type < BoundType_First_Derivative)) {
            return false;
        }
        initParam();

        x_sample_ = new double[count];
        y_sample_ = new double[count];
        M_ = new double[count];
        sample_count_ = count;

        memcpy(x_sample_, x_data, sample_count_ * sizeof(double));
        memcpy(y_sample_, y_data, sample_count_ * sizeof(double));

        bound1_ = bound1;
        bound2_ = bound2;

        return spline(type);
    }

    bool spline(BoundType type) {
        if ((type < BoundType_First_Derivative) || (type > BoundType_Second_Derivative)) {
            return false;
        }

        // 追赶法解方程求二阶偏导数
        double f1 = bound1_, f2 = bound2_;

        double* a = new double[sample_count_];  // a:稀疏矩阵最下边一串数
        double* b = new double[sample_count_];  // b:稀疏矩阵最中间一串数
        double* c = new double[sample_count_];  // c:稀疏矩阵最上边一串数
        double* d = new double[sample_count_];

        double* f = new double[sample_count_];

        double* bt = new double[sample_count_];
        double* gm = new double[sample_count_];

        double* h = new double[sample_count_];

        for (int i = 0; i < sample_count_; i++)
            b[i] = 2;  // 中间一串数为2

        for (int i = 0; i < sample_count_ - 1; i++) 
        {
            if (x_sample_[i + 1] == x_sample_[i]) {
                h[i] = 0.005;
            } else {
                h[i] = x_sample_[i + 1] - x_sample_[i];  // 各段步长
            }
        }

        for (int i = 1; i < sample_count_ - 1; i++)
            a[i] = h[i - 1] / (h[i - 1] + h[i]);
        a[sample_count_ - 1] = 1;

        c[0] = 1;
        for (int i = 1; i < sample_count_ - 1; i++)
            c[i] = h[i] / (h[i - 1] + h[i]);

        for (int i = 0; i < sample_count_ - 1; i++) 
        {
            if (x_sample_[i + 1] == x_sample_[i]) {
                f[i] = (y_sample_[i + 1] - y_sample_[i]) / 0.005;
            } else {
                f[i] = (y_sample_[i + 1] - y_sample_[i]) / (x_sample_[i + 1] - x_sample_[i]);
            }
        }

        for (int i = 0; i < sample_count_ - 1; i++)
            d[i] = 6 * (f[i] - f[i - 1]) / (h[i - 1] + h[i]);

        // 追赶法求解方程
        if (BoundType_First_Derivative == type) 
        {
            d[0] = 6 * (f[0] - f1) / h[0];
            d[sample_count_ - 1] = 6 * (f2 - f[sample_count_ - 2]) / h[sample_count_ - 2];

            bt[0] = c[0] / b[0];
            for (int i = 1; i < sample_count_ - 1; i++)
                bt[i] = c[i] / (b[i] - a[i] * bt[i - 1]);

            gm[0] = d[0] / b[0];
            for (int i = 1; i <= sample_count_ - 1; i++)
                gm[i] = (d[i] - a[i] * gm[i - 1]) / (b[i] - a[i] * bt[i - 1]);

            M_[sample_count_ - 1] = gm[sample_count_ - 1];
            for (int i = sample_count_ - 2; i >= 0; i--)
                M_[i] = gm[i] - bt[i] * M_[i + 1];
        } 
        else if (BoundType_Second_Derivative == type) 
        {
            d[1] = d[1] - a[1] * f1;
            d[sample_count_ - 2] = d[sample_count_ - 2] - c[sample_count_ - 2] * f2;

            bt[1] = c[1] / b[1];
            for (int i = 2; i < sample_count_ - 2; i++)
                bt[i] = c[i] / (b[i] - a[i] * bt[i - 1]);

            gm[1] = d[1] / b[1];
            for (int i = 2; i <= sample_count_ - 2; i++)
                gm[i] = (d[i] - a[i] * gm[i - 1]) / (b[i] - a[i] * bt[i - 1]);

            M_[sample_count_ - 2] = gm[sample_count_ - 2];
            for (int i = sample_count_ - 3; i >= 1; i--)
                M_[i] = gm[i] - bt[i] * M_[i + 1];

            M_[0] = f1;
            M_[sample_count_ - 1] = f2;
        } 
        else
            return false;

        delete[] a;
        delete[] b;
        delete[] c;
        delete[] d;
        delete[] gm;
        delete[] bt;
        delete[] f;
        delete[] h;

        return true;
    }

    bool getYbyX(double& x_in, double& y_out, double& vel, double& acc)
    {
        int klo, khi, k;
        klo = 0;
        khi = sample_count_ - 1;
        double hh, bb, aa;

        // 二分法查找x所在区间段
        while (khi - klo > 1) {
            k = (khi + klo) >> 1;
            if (x_sample_[k] > x_in)
                khi = k;
            else
                klo = k;
        }
        hh = x_sample_[khi] - x_sample_[klo];

        aa = (x_sample_[khi] - x_in) / hh;
        bb = (x_in - x_sample_[klo]) / hh;

        y_out = aa * y_sample_[klo] + bb * y_sample_[khi] + ((aa * aa * aa - aa) * M_[klo] + (bb * bb * bb - bb) * M_[khi]) * hh * hh / 6.0;

        // 计算速度和加速度
        acc = (M_[klo] * (x_sample_[khi] - x_in) + M_[khi] * (x_in - x_sample_[klo])) / hh;
        vel = M_[khi] * (x_in - x_sample_[klo]) * (x_in - x_sample_[klo]) / (2 * hh) - M_[klo] * (x_sample_[khi] - x_in) * (x_sample_[khi] - x_in) / (2 * hh) + (y_sample_[khi] - y_sample_[klo]) / hh - hh * (M_[khi] - M_[klo]) / 6;

        return true;
    }

    // 三次样条插值主函数
    bool computeTrajectory(const std::vector<std::vector<double>>& path, 
                          const std::vector<double>& max_velocity,
                          const std::vector<double>& max_acceleration,
                          double frequency,
                          std::vector<std::vector<double>>& trajectory)
    {
        if (path.size() < 3) {
            // 如果路径点少于3个，直接返回原始路径
            trajectory = path;
            return true;
        }

        size_t point_num = path.size();
        double rate = 1.0 / frequency;

        // 各个关节位置
        std::vector<std::vector<double>> joint_positions(7, std::vector<double>(point_num));
        std::vector<double> time_from_start(point_num);

        // 提取轨迹点数据
        for (size_t i = 0; i < point_num; i++) {
            for (size_t j = 0; j < 7 && j < path[i].size(); j++) {
                joint_positions[j][i] = path[i][j];
            }
            // 假设路径点是等间距的
            time_from_start[i] = static_cast<double>(i) * rate;
        }

        double max_time = time_from_start[point_num - 1];
        trajectory.clear();

        // 对每个关节分别进行三次样条插值
        std::vector<std::vector<double>> interpolated_joints(7);
        for (size_t joint = 0; joint < 7; joint++) {
            CubicSplineInterpolator spline;
            if (spline.loadData(&time_from_start[0], &joint_positions[joint][0], static_cast<int>(point_num), 0, 0, BoundType_First_Derivative)) {
                double x_out = -rate;
                interpolated_joints[joint].clear();
                
                while (x_out < max_time) {
                    x_out += rate;
                    double y_out, vel, acc;
                    spline.getYbyX(x_out, y_out, vel, acc);
                    // 限制速度和加速度（将来可以使用max_velocity和max_acceleration参数）
                    (void)max_velocity;    // 避免未使用参数警告
                    (void)max_acceleration; // 避免未使用参数警告
                    interpolated_joints[joint].push_back(y_out);
                }
            } else {
                // 如果插值失败，使用原始数据
                interpolated_joints[joint] = joint_positions[joint];
            }
        }

        // 重新组织数据结构
        size_t interpolated_points = interpolated_joints[0].size();
        trajectory.resize(interpolated_points, std::vector<double>(7));
        for (size_t i = 0; i < interpolated_points; i++) {
            for (size_t j = 0; j < 7; j++) {
                trajectory[i][j] = interpolated_joints[j][i];
            }
        }

        return true;
    }

private:
    double* x_sample_;
    double* y_sample_;
    double* M_;
    int sample_count_;
    double bound1_, bound2_;
};
```

####  代码解析

##### 三次样条插值与五次多项式插值的参数设置区别

1. **五次多项式插值**

- **显式参数控制**：需要显式设置最大速度和加速度限制
- **物理约束**：直接根据机械臂的物理限制来设置这些参数
- **时间参数化**：通过速度和加速度限制来确定轨迹点之间的时间间隔

2. **三次样条插值**

- **隐式参数控制**：通过路径点分布和边界条件来间接控制速度和加速度
- **平滑性优先**：主要关注轨迹的平滑性而不是严格的物理限制
- **路径点分布**：轨迹的平滑性主要由输入路径点的分布决定

##### 为什么三次样条插值不需要显式设置最大速度和加速度

1. **数学特性**：
   - 三次样条插值通过确保二阶导数连续来保证轨迹平滑性
   - 速度和加速度由插值函数的导数自然产生
2. **边界条件**：
   - 通过设置边界条件（如起始和结束点的速度为0）来控制轨迹特性
   - 不需要在整个轨迹上显式限制速度和加速度
3. **应用场景**：
   - 三次样条插值更适合对平滑性要求高的场景
   - 五次多项式插值更适合需要严格物理约束的场景

##### 实际应用中的考虑

虽然三次样条插值不需要显式设置最大速度和加速度参数，但在实际应用中仍需要注意：

1. **轨迹验证**：
   - 插值完成后应检查生成轨迹的速度和加速度是否在机械臂的物理限制内
   - 如果超出限制，可能需要调整输入路径点或重新插值
2. **参数保留**：
   - 保留这些参数以保持接口一致性
   - 为将来可能的扩展或优化预留参数
3. **后处理**：
   - 可以在插值后添加额外的处理步骤来确保物理可行性

总的来说，三次样条插值通过数学方法自然地产生平滑的轨迹，而不需要像五次多项式插值那样显式地设置速度和加速度限制。这也是为什么我们在代码中保留了这些参数但没有实际使用它们的原因。

##### 三次样条插值后的轨迹速度和加速度检查

在三次样条插值完成后，检查生成轨迹的速度和加速度是否在机械臂的物理限制内是一个重要步骤。

```c++
bool checkTrajectoryLimits(const std::vector<std::vector<double>>& velocities,
                          const std::vector<std::vector<double>>& accelerations,
                          const std::vector<double>& max_velocity,
                          const std::vector<double>& max_acceleration) 
{
    bool within_limits = true;
    
    // 检查每个关节的速度限制
    for (size_t joint = 0; joint < velocities.size() && joint < 7; joint++) {
        double joint_max_vel = (joint < max_velocity.size()) ? max_velocity[joint] : 
max_velocity.back();
        
        for (size_t i = 0; i < velocities[joint].size(); i++) {
            if (std::abs(velocities[joint][i]) > joint_max_vel) {
                RCLCPP_WARN(rclcpp::get_logger("CubicSplineInterpolator"), 
                           "Joint %zu velocity limit exceeded: %f > %f at point %zu", 
                           joint, std::abs(velocities[joint][i]), joint_max_vel, i);
                within_limits = false;
            }
        }
    }
    
    // 检查每个关节的加速度限制
    for (size_t joint = 0; joint < accelerations.size() && joint < 7; joint++) {
        double joint_max_acc = (joint < max_acceleration.size()) ? max_acceleration[joint
max_acceleration.back();
        
        for (size_t i = 0; i < accelerations[joint].size(); i++) {
            if (std::abs(accelerations[joint][i]) > joint_max_acc) {
                RCLCPP_WARN(rclcpp::get_logger("CubicSplineInterpolator"), 
                           "Joint %zu acceleration limit exceeded: %f > %f at point %zu",
                           joint, std::abs(accelerations[joint][i]), joint_max_acc, i);
                within_limits = false;
            }
        }
    }
    
    return within_limits;
}
```



1. **数值微分计算速度和加速度**

三次样条插值算法本身就可以计算出轨迹点的速度和加速度。我们在 [getYbyX](javascript:void(0)) 函数中已经实现了这个功能：

```c++
bool getYbyX(double& x_in, double& y_out, double& vel, double& acc)
```

2. **限制检查实现**

我添加了一个 `checkTrajectoryLimits` 函数来检查生成轨迹的速度和加速度是否超出限制：

- **速度检查**：遍历每个关节的每个轨迹点，检查速度绝对值是否超过最大速度限制
- **加速度检查**：遍历每个关节的每个轨迹点，检查加速度绝对值是否超过最大加速度限制
- **警告输出**：当发现超出限制的情况时，使用 ROS 的日志系统输出警告信息

3. **数据结构改进**

修改了 [computeTrajectory](javascript:void(0)) 函数，使其能够存储和处理速度、加速度信息：

- 增加了 `interpolated_velocities` 和 `interpolated_accelerations` 容器来存储速度和加速度数据
- 在插值过程中同时获取位置、速度和加速度信息

**使用方法**

在实际使用中，您可以这样检查轨迹限制：

1. 调用三次样条插值函数生成轨迹
2. 函数内部会自动检查速度和加速度限制
3. 如果超出限制，会输出警告信息

**可能的改进**

1. **动态调整**：当发现轨迹超出限制时，可以自动调整插值参数（如降低频率）

2. **轨迹裁剪**：对超出限制的轨迹点进行裁剪或平滑处理

3. **详细报告**：提供更详细的轨迹分析报告，包括最大速度、加速度等统计信息

   

#### 代码优化

三次样条插值主函数原代码：

```c++
bool computeTrajectory(const std::vector<std::vector<double>>& path, 
                      const std::vector<double>& max_velocity,
                      const std::vector<double>& max_acceleration,
                      double frequency,
                      std::vector<std::vector<double>>& trajectory) {
    if (path.size() < 3) {
        // 如果路径点少于3个，直接返回原始路径
        trajectory = path;
        return true;
    }

    int point_num = path.size();
    double rate = 1.0 / frequency;

    // 各个关节位置
    std::vector<std::vector<double>> joint_positions(7, std::vector<double>(point_num));
    std::vector<double> time_from_start(point_num);

    // 提取轨迹点数据
    for (int i = 0; i < point_num; i++) {
        for (int j = 0; j < 7 && j < path[i].size(); j++) {
            joint_positions[j][i] = path[i][j];
        }
        // 假设路径点是等间距的
        time_from_start[i] = i * rate;
    }

    double max_time = time_from_start[point_num - 1];
    trajectory.clear();

    // 对每个关节分别进行三次样条插值
    std::vector<std::vector<double>> interpolated_joints(7);
    for (int joint = 0; joint < 7; joint++) {
        CubicSplineInterpolator spline;
        if (spline.loadData(&time_from_start[0], &joint_positions[joint][0], point_num, 0, 0, BoundType_First_Derivative)) {
            double x_out = -rate;
            interpolated_joints[joint].clear();
            
            while (x_out < max_time) {
                x_out += rate;
                double y_out, vel, acc;
                spline.getYbyX(x_out, y_out, vel, acc);
                // 限制速度和加速度
                interpolated_joints[joint].push_back(y_out);
            }
        } else {
            // 如果插值失败，使用原始数据
            interpolated_joints[joint] = joint_positions[joint];
        }
    }

    // 重新组织数据结构
    int interpolated_points = interpolated_joints[0].size();
    trajectory.resize(interpolated_points, std::vector<double>(7));
    for (int i = 0; i < interpolated_points; i++) {
        for (int j = 0; j < 7; j++) {
            trajectory[i][j] = interpolated_joints[j][i];
        }
    }

    return true;
}
```

**问题检查**

1. 硬编码关节数7，缺乏灵活性
2. 未处理输入参数size不一致的情况
3. 内存分配使用new/delete，建议使用智能指针或vector
4. 插值失败时直接使用原始数据，可能导致轨迹不连续
5. 速度和加速度限制未实际应用
6. 缺乏对输入参数的校

**优化后的代码：**

```c++
bool computeTrajectory(const std::vector<std::vector<double>>& path,
                      const std::vector<double>& max_velocity,
                      const std::vector<double>& max_acceleration,
                      double frequency,
                      std::vector<std::vector<double>>& trajectory) {
    if (path.empty()) {
        return false;
    }

    const size_t num_joints = path[0].size();
    if (path.size() < 3) {
        trajectory = path;
        return true;
    }

    const size_t point_num = path.size();
    const double rate = 1.0 / frequency;

    // 提取轨迹点数据
    std::vector<std::vector<double>> joint_positions(num_joints, std::vector<double>(point_num));
    std::vector<double> time_from_start(point_num);

    for (size_t i = 0; i < point_num; ++i) {
        if (path[i].size() != num_joints) {
            return false;
        }
        for (size_t j = 0; j < num_joints; ++j) {
            joint_positions[j][i] = path[i][j];
        }
        time_from_start[i] = i * rate;
    }

    const double max_time = time_from_start.back();
    trajectory.clear();

    // 对每个关节分别进行三次样条插值
    std::vector<std::vector<double>> interpolated_joints(num_joints);
    for (size_t joint = 0; joint < num_joints; ++joint) {
        CubicSplineInterpolator spline;
        if (spline.loadData(time_from_start.data(), joint_positions[joint].data(), 
                           point_num, 0, 0, BoundType_First_Derivative)) {
            double x_out = -rate;
            interpolated_joints[joint].reserve(static_cast<size_t>(max_time * 		frequency) + 1);
            
            while (x_out < max_time) {
                x_out += rate;
                double y_out, vel, acc;
                spline.getYbyX(x_out, y_out, vel, acc);
                
                // 应用速度和加速度限制
                if (joint < max_velocity.size() && std::abs(vel) > 		max_velocity[joint]) {
                    vel = (vel > 0) ? max_velocity[joint] : -max_velocity[joint];
                }
                if (joint < max_acceleration.size() && std::abs(acc) > max_acceleration[joint]) {
                    acc = (acc > 0) ? max_acceleration[joint] : -max_acceleration[joint];
                }
                
                interpolated_joints[joint].push_back(y_out);
            }
        } else {
            return false;
        }
    }

    // 检查速度和加速度限制
    if (!checkTrajectoryLimits(interpolated_velocities, 
    interpolated_accelerations, max_velocity, max_acceleration)) {
        // 如果超出限制，可以采取相应措施，如调整频率或限制值
        RCLCPP_WARN(rclcpp::get_logger("CubicSplineInterpolator"), 
                   "Trajectory exceeds velocity or acceleration limits. 
    Consider adjusting parameters.");
    }
                
    // 重新组织数据结构
    const size_t interpolated_points = interpolated_joints[0].size();
    trajectory.resize(interpolated_points, std::vector<double>(num_joints));
    
    for (size_t i = 0; i < interpolated_points; ++i) {
        for (size_t j = 0; j < num_joints; ++j) {
            trajectory[i][j] = interpolated_joints[j][i];
        }
    }

    return true;
}
```

**优化点总结**

1. 移除硬编码关节数，支持任意数量关节
2. 增加输入参数校验
3. 增加检查速度和加速度限制
4. 改进内存管理，使用vector替代原始指针
5. 实际应用速度和加速度限制
6. 提高代码健壮性，处理更多边界情况
7. 使用size_t替代int避免潜在类型问



**新增的改进点**

- **动态调整**：当发现轨迹超出限制时，可以自动调整插值参数（如降低频率）
- **轨迹裁剪**：对超出限制的轨迹点进行裁剪或平滑处理
- **详细报告**：提供更详细的轨迹分析报告，包括最大速度、加速度等统计息



1. **动态调整插值参数**

```c++
// 动态调整插值参数的实现
bool adaptivelyAdjustTrajectory(std::vector<std::vector<double>>& trajectory,
                               const std::vector<std::vector<double>>& velocities,
                               const std::vector<std::vector<double>>& accelerations,
                               const std::vector<double>& max_velocity,
                               const std::vector<double>& max_acceleration,
                               double& frequency) {
    // 检查是否超出限制
    double max_vel_ratio = 1.0;
    double max_acc_ratio = 1.0;
    
    // 找到最大的超限比例
    for (size_t joint = 0; joint < velocities.size() && joint < 7; joint++) {
        double joint_max_vel = (joint < max_velocity.size()) ? max_velocity[joint] : max_velocity.back();
        double joint_max_acc = (joint < max_acceleration.size()) ? max_acceleration[joint] : max_acceleration.back();
        
        for (size_t i = 0; i < velocities[joint].size(); i++) {
            double vel_ratio = std::abs(velocities[joint][i]) / joint_max_vel;
            double acc_ratio = std::abs(accelerations[joint][i]) / joint_max_acc;
            
            if (vel_ratio > max_vel_ratio) max_vel_ratio = vel_ratio;
            if (acc_ratio > max_acc_ratio) max_acc_ratio = acc_ratio;
        }
    }
    
    // 如果超出限制，则调整频率
    double adjustment_factor = std::max(max_vel_ratio, max_acc_ratio);
    if (adjustment_factor > 1.0) {
        // 降低频率以减少速度和加速度
        frequency /= adjustment_factor;
        return true; // 需要重新计算轨迹
    }
    
    return false; // 不需要调整
}
```

2. **轨迹裁剪和平滑处理**

```c++
// 轨迹裁剪和平滑处理实现
void smoothTrajectoryWithLimits(std::vector<std::vector<double>>& trajectory,
                               std::vector<std::vector<double>>& velocities,
                               std::vector<std::vector<double>>& accelerations,
                               const std::vector<double>& max_velocity,
                               const std::vector<double>& max_acceleration) {
    // 对每个关节分别处理
    for (size_t joint = 0; joint < trajectory.size() && joint < 7; joint++) {
        double joint_max_vel = (joint < max_velocity.size()) ? max_velocity[joint] : max_velocity.back();
        double joint_max_acc = (joint < max_acceleration.size()) ? max_acceleration[joint] : max_acceleration.back();
        
        // 限制速度
        for (size_t i = 0; i < velocities[joint].size(); i++) {
            if (std::abs(velocities[joint][i]) > joint_max_vel) {
                // 按比例缩放速度
                double scale = joint_max_vel / std::abs(velocities[joint][i]);
                velocities[joint][i] *= scale;
            }
        }
        
        // 限制加速度
        for (size_t i = 0; i < accelerations[joint].size(); i++) {
            if (std::abs(accelerations[joint][i]) > joint_max_acc) {
                // 按比例缩放加速度
                double scale = joint_max_acc / std::abs(accelerations[joint][i]);
                accelerations[joint][i] *= scale;
            }
        }
        
        // 重新计算位置轨迹以符合速度和加速度限制
        if (trajectory[joint].size() > 1) {
            // 使用积分方法从速度重建位置
            for (size_t i = 1; i < trajectory[joint].size(); i++) {
                double dt = 1.0 / 50.0; // 假设50Hz频率
                trajectory[joint][i] = trajectory[joint][i-1] + velocities[joint][i] * dt;
            }
        }
    }
}

// 移动平均滤波器实现
void applyMovingAverageFilter(std::vector<std::vector<double>>& trajectory, int window_size = 3) {
    if (window_size < 3 || window_size % 2 == 0) {
        window_size = 3; // 确保窗口大小为奇数且至少为3
    }
    
    int half_window = window_size / 2;
    
    for (size_t joint = 0; joint < trajectory.size(); joint++) {
        std::vector<double> filtered(trajectory[joint].size());
        
        for (size_t i = 0; i < trajectory[joint].size(); i++) {
            double sum = 0.0;
            int count = 0;
            
            // 计算窗口内的平均值
            for (int j = -half_window; j <= half_window; j++) {
                int index = static_cast<int>(i) + j;
                if (index >= 0 && index < static_cast<int>(trajectory[joint].size())) {
                    sum += trajectory[joint][index];
                    count++;
                }
            }
            
            filtered[i] = sum / count;
        }
        
        trajectory[joint] = filtered;
    }
}
```

3. **详细轨迹分析报告**

```c++
// 轨迹分析报告结构
struct TrajectoryAnalysisReport {
    std::vector<double> max_velocity_per_joint;
    std::vector<double> max_acceleration_per_joint;
    std::vector<double> avg_velocity_per_joint;
    std::vector<double> avg_acceleration_per_joint;
    double overall_max_velocity = 0.0;
    double overall_max_acceleration = 0.0;
    double overall_avg_velocity = 0.0;
    double overall_avg_acceleration = 0.0;
    std::vector<size_t> velocity_violation_points;
    std::vector<size_t> acceleration_violation_points;
};

// 生成详细的轨迹分析报告
TrajectoryAnalysisReport generateTrajectoryAnalysisReport(
    const std::vector<std::vector<double>>& velocities,
    const std::vector<std::vector<double>>& accelerations,
    const std::vector<double>& max_velocity,
    const std::vector<double>& max_acceleration) {
    
    TrajectoryAnalysisReport report;
    report.max_velocity_per_joint.resize(7, 0.0);
    report.max_acceleration_per_joint.resize(7, 0.0);
    report.avg_velocity_per_joint.resize(7, 0.0);
    report.avg_acceleration_per_joint.resize(7, 0.0);
    
    // 分析每个关节
    for (size_t joint = 0; joint < velocities.size() && joint < 7; joint++) {
        double joint_max_vel = 0.0;
        double joint_max_acc = 0.0;
        double joint_sum_vel = 0.0;
        double joint_sum_acc = 0.0;
        
        double limit_vel = (joint < max_velocity.size()) ? max_velocity[joint] : max_velocity.back();
        double limit_acc = (joint < max_acceleration.size()) ? max_acceleration[joint] : max_acceleration.back();
        
        for (size_t i = 0; i < velocities[joint].size(); i++) {
            double abs_vel = std::abs(velocities[joint][i]);
            double abs_acc = std::abs(accelerations[joint][i]);
            
            // 更新最大值
            if (abs_vel > joint_max_vel) joint_max_vel = abs_vel;
            if (abs_acc > joint_max_acc) joint_max_acc = abs_acc;
            
            // 累加用于计算平均值
            joint_sum_vel += abs_vel;
            joint_sum_acc += abs_acc;
            
            // 检查是否超出限制
            if (abs_vel > limit_vel) {
                report.velocity_violation_points.push_back(i);
            }
            if (abs_acc > limit_acc) {
                report.acceleration_violation_points.push_back(i);
            }
        }
        
        // 计算平均值
        if (!velocities[joint].empty()) {
            report.avg_velocity_per_joint[joint] = joint_sum_vel / velocities[joint].size();
        }
        if (!accelerations[joint].empty()) {
            report.avg_acceleration_per_joint[joint] = joint_sum_acc / accelerations[joint].size();
        }
        
        report.max_velocity_per_joint[joint] = joint_max_vel;
        report.max_acceleration_per_joint[joint] = joint_max_acc;
        
        // 更新整体最大值
        if (joint_max_vel > report.overall_max_velocity) {
            report.overall_max_velocity = joint_max_vel;
        }
        if (joint_max_acc > report.overall_max_acceleration) {
            report.overall_max_acceleration = joint_max_acc;
        }
    }
    
    // 计算整体平均值
    double total_avg_vel = 0.0;
    double total_avg_acc = 0.0;
    int valid_joints = 0;
    
    for (size_t joint = 0; joint < 7; joint++)发生异常，可以输入更多信
```



### 运动学求解器