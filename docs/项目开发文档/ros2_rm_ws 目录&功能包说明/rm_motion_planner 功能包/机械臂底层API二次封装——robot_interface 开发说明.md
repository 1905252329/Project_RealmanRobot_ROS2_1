# RM Motion Planner 开发日志

## 项目概述

ros2_rm_ws 是一个基于 ROS2 Humble 的机器人软件包工作空间，主要用于 Realman RM 系列机械臂的控制、仿真、驱动和应用开发。

### 核心功能

- 机械臂运动控制（rm_control）
- 机械臂状态获取（get_arm_state）
- 力控与位置控制（force_position_control）
- Gazebo 仿真支持（rm_gazebo）
- 驱动接口与配置（rm_driver）
- MoveIt2 运动规划配置（rm_moveit2_config）
- 示例程序（rm_example）

## 开发环境配置

### 技术栈

- ROS2 版本: Humble
- 编程语言: C++, Python
- 构建系统: CMake
- 依赖管理: ament_cmake
- 仿真平台: Gazebo
- 运动规划框架: MoveIt2

### 必需工具

- ROS2 Humble 安装环境
- CMake >= 3.5
- GCC 或 Clang 编译器
- Python 3.x
- Git

### 构建和运行

```bash
# 构建命令
colcon build

# 运行命令
source install/setup.bash
ros2 run rm_motion_planner test_fk

```

## 开发规范与实现步骤

### 1. ROS2测试目录创建

#### 1.1 标准目录结构

在ROS2项目中，测试目录是按照标准ROS2包结构创建的。`/home/byd/Documents/Projects/DualArmProjects/realmanRobot/ros2_rm_ws/src/ros2_rm_robot-humble/rm_motion_planner/test` 目录的创建遵循以下规范：

```
rm_motion_planner/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── rm_motion_planner/
├── src/
├── test/                    # 测试目录
│   ├── test_fk.cpp         # 测试源文件
│   └── developLog.md       # 开发日志文件
├── launch/
└── config/
```

#### 1.2 创建步骤

1. **手动创建目录**:

   ```bash
   mkdir -p /home/byd/Documents/Projects/DualArmProjects/realmanRobot/ros2_rm_ws/src/ros2_rm_robot-humble/rm_motion_planner/test
   ```
2. **添加测试文件**:

   - 创建测试源文件，如 `test_fk.cpp`
   - 添加测试相关的文档或配置文件
3. **更新CMakeLists.txt配置**:
   在包的 CMakeLists.txt 中添加测试可执行文件的配置

### 2. ROS2测试代码标准实践规范

#### 2.1 测试文件命名规范

- 测试文件应以 `test_` 为前缀命名
- 使用描述性名称表明测试内容，如 `test_forward_kinematics.cpp`
- 使用 `.cpp` 扩展名表示C++源文件

#### 2.2 测试类设计规范

```cpp
class FKTestNode : public rclcpp::Node {
    public:
        FKTestNode() : Node("fk_test_node") {
            // 初始化和测试代码
       }

    private:
        // 成员变量和方法
};
```
#### 2.3 参数管理规范

1. **参数声明**:

   ```cpp
   this->declare_parameter("offline_mode", false);
   this->declare_parameter("robot_arm_model", "RM_MODEL_RM_65_E");
   ```
2. **参数获取**:

   ```cpp
   this->get_parameter("offline_mode", offline_mode_);
   this->get_parameter("robot_arm_model", robot_arm_model_);
   ```
3. **参数使用**:

   ```bash
   ros2 run rm_motion_planner test_fk --ros-args -p offline_mode:=true
   ```

#### 2.4 错误处理规范

1. **详细日志记录**:

   ```cpp
   RCLCPP_INFO(this->get_logger(), "初始化RM API...");
   RCLCPP_ERROR(this->get_logger(), "创建机器人句柄失败: 返回空指针");
   ```
2. **错误码处理**:

   ```cpp
   if (result != 0) {
       RCLCPP_ERROR(this->get_logger(), "获取机器人信息失败，错误码: %d", result);
   }
   ```

### 3. 具体开发步骤

#### 3.1 创建测试文件

1. **创建测试节点类**:

   ```cpp
   class FKTestNode : public rclcpp::Node {
   public:
       FKTestNode() : Node("fk_test_node") {
           // 初始化和测试代码
       }
   
   private:
       // 成员变量和方法
   };
   ```
2. **实现main函数**:

   ```cpp
   int main(int argc, char** argv) {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<FKTestNode>());
       rclcpp::shutdown();
       return 0;
   }
   ```

#### 3.2 实现测试功能

1. **参数管理**:

   ```cpp
   // 声明参数
   this->declare_parameter("robot_ip", "169.254.247.19");
   this->declare_parameter("robot_port", 8080);
   this->declare_parameter("offline_mode", false);
   
   // 获取参数
   this->get_parameter("robot_ip", robot_ip_);
   this->get_parameter("robot_port", robot_port_);
   this->get_parameter("offline_mode", offline_mode_);
   ```
2. **初始化RM API**:

   ```cpp
   void initialize_rm_api() {
       RCLCPP_INFO(this->get_logger(), "初始化RM API...");
   
       // 初始化算法数据
       rm_algo_init_sys_data(RM_MODEL_RM_65_E, RM_MODEL_RM_B_E);
       RCLCPP_INFO(this->get_logger(), "算法数据初始化完成");
   
       if (offline_mode_) {
           RCLCPP_INFO(this->get_logger(), "离线模式: 跳过机器人连接");
           return;
       }
   
       // 创建机器人连接
       robot_handle_ = rm_create_robot_arm(robot_ip_.c_str(), robot_port_);
   }
   ```
3. **执行测试**:

   ```cpp
   void test_forward_kinematics() {
       // 测试关节角度
       float joint_angles_deg[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
   
       // 执行正向运动学计算
       rm_pose_t pose;
       if (offline_mode_) {
           pose = rm_algo_forward_kinematics(NULL, joint_angles_deg);
       } else {
           pose = rm_algo_forward_kinematics(robot_handle_, joint_angles_deg);
       }
   
       // 输出结果
       RCLCPP_INFO(this->get_logger(), "位置: x=%f, y=%f, z=%f", 
                  pose.position.x, pose.position.y, pose.position.z);
   }
   ```

#### 3.3 CMakeLists.txt配置

1. **添加可执行文件**:

   ```cmake
   add_executable(test_fk test/test_fk.cpp)
   ```
2. **配置包含目录**:

   ```cmake
   target_include_directories(test_fk PUBLIC
     ${EIGEN3_INCLUDE_DIRS}
     ${rm_driver_INCLUDE_DIRS}
   )
   ```
3. **链接库**:

   ```cmake
   target_link_libraries(test_fk ${rm_driver_LIBRARIES})
   ```
4. **声明依赖**:

   ```cmake
   ament_target_dependencies(test_fk 
     rclcpp 
     std_msgs
     # ... 其他依赖
   )
   ```
5. **安装目标**:

   ```cmake
   install(TARGETS 
     test_fk
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

#### 3.4 编译和运行

1. **编译测试**:

   ```bash
   colcon build --packages-select rm_motion_planner
   ```
2. **更新环境**:

   ```bash
   source install/setup.bash
   ```
3. **运行测试**:

   ```bash
   ros2 run rm_motion_planner test_fk
   ```
4. **参数化运行**:

   ```bash
   ros2 run rm_motion_planner test_fk --ros-args -p offline_mode:=true
   ```

### 4. 最佳实践

#### 4.1 模块化设计

将测试功能分解为独立的模块：

- 初始化模块
- 参数处理模块
- 测试执行模块
- 结果分析模块

#### 4.2 可重用性

设计可重用的测试组件：

- 通用的初始化函数
- 标准化的参数处理
- 可配置的测试场景

#### 4.3 可维护性

保持代码的可维护性：

- 清晰的代码结构和注释
- 一致的命名规范
- 模块化的功能实现

#### 4.4 调试支持

实现调试和问题解决机制：

- 详细的日志记录
- 错误码处理
- 网络连接检查
- 参数验证



### 5. 机械臂接口调用方案比对

RM API提供了两种不同的接口调用模式，分别适用于不同的使用场景：

#### 1. 面向对象的 C++ API 封装模式（Demo源文件方式）

这种模式使用RM_Service类对API函数进行封装，适用于算法演示和离线计算：

```c++
RM_Service robotic_arm;
rm_robot_handle handle;
```

特点：

- 使用面向对象的方式访问RM API功能
- 不需要连接真实机械臂
- 通过rm_algo_init_sys_data初始化算法数据
- 适用于离线算法测试和验证

#### 2. 直接 C 风格 API 调用模式（节点源文件方式）

这种模式直接调用RM API的C函数，适用于实际控制真实机械臂：

```c++
rm_robot_handle* robot_handle_ = nullptr;
robot_handle_ = rm_create_robot_arm("127.0.0.1", 8080);
```

特点：

- 直接调用RM API的C函数
- 需要通过 rm_create_robot_arm 创建与真实机械臂的网络连接
- 可以同时使用机械臂实时数据和算法库
- 适用于实际控制场景

#### 3. 主要区别对比


| 特性        | 面向对象封装模式 | 直接C风格调用模式 |
| ----------- | ---------------- | ----------------- |
| API使用方式 | RM_Service类封装 | 直接调用C函数     |
| 连接方式    | 不连接真实机械臂 | 连接真实机械臂    |
| 内存管理    | 栈上分配局部变量 | 堆上分配指针      |
| 应用场景    | 离线算法测试     | 实际机械臂控制    |

#### 4. 选择建议

在我们的项目中，使用直接C风格API调用模式（rm_create_robot_arm）连接真实机械臂是正确的选择，因为我们需要：

- 与实际的机械臂通信
- 获取实时数据
- 进行实际控制操作



## 机械臂连接配置问题记录

### 1. 初始问题说明

运行测试节点时出现以下错误：

```
[INFO] [1755593438.806651238] [fk_test_node]: 初始化RM API... 
[INFO] [1755593438.806963217] [fk_test_node]: 初始化算法系统数据... 
[INFO] [1755593438.807356960] [fk_test_node]: 算法数据初始化完成 
[rm_get_arm_software_info] get_arm_software_info revice error 
[rm_create_robot_arm] get robot info err! 
[INFO] [1755593439.350954464] [fk_test_node]: 机器人句柄ID: -1 
[ERROR] [1755593439.351121702] [fk_test_node]: 无效的机器人句柄ID 
[ERROR] [1755593439.351160712] [fk_test_node]: 无效的机器人句柄
```
### 2. 问题分析与尝试方向

#### 2.1 网络连接测试

首先确认机械臂网络连接状态：

```bash
telnet 169.254.247.19 8080
```
输出结果：

```
Trying 169.254.247.19...
Connected to 169.254.247.19.
Escape character is '^]'.
```
这表明网络连接正常，防火墙未阻止连接，且机械臂已开启并处于待连接状态。

#### 2.2 离线模式支持

为支持在无物理设备情况下测试算法，添加离线模式支持：

##### 代码实现

```cpp
// 添加参数支持
this->declare_parameter("offline_mode", false);
this->get_parameter("offline_mode", offline_mode_);

// 在离线模式下跳过机器人连接
if (offline_mode_) {
    RCLCPP_INFO(this->get_logger(), "离线模式: 跳过机器人连接");
    return;
}
```
##### 运行指令

运行离线模式：

```bash
ros2 run rm_motion_planner test_fk --ros-args -p offline_mode:=true
```


##### 离线模式工作原理

当在离线模式下将NULL作为句柄传递给`rm_algo_forward_kinematics`函数时，它的计算原理如下：

###### 1. 初始化依赖数据

```cpp
rm_algo_init_sys_data(RM_MODEL_RM_65_E, RM_MODEL_RM_B_E);
```

该函数会设置以下关键参数：

- 机械臂型号参数（影响DH参数）
- 末端力传感器型号
- 默认的坐标系设置
- 关节限位和其他物理参数

###### 2. 正向运动学计算过程

当传递NULL句柄时，`rm_algo_forward_kinematics`函数会：

1. 使用之前通过`rm_algo_init_sys_data`设置的机械臂型号对应的DH参数
2. 应用标准的正向运动学算法（通常是基于DH参数的变换矩阵连乘）
3. 使用默认的工具坐标系和工作坐标系（除非通过其他API函数另行设置）
4. 根据输入的关节角度计算末端执行器的位姿

##### 与在线模式的区别

- **在线模式**：使用实际连接的机械臂的实时参数和配置
- **离线模式**：使用通过`rm_algo_init_sys_data`设置的参数，以及默认的坐标系配置

#### 2.3 机械臂型号配置

添加机械臂型号参数支持，以适应不同型号的机械臂：

```cpp
// 添加机械臂型号参数
this->declare_parameter("robot_arm_model", "RM_MODEL_RM_75_E");
this->declare_parameter("force_sensor_model", "RM_MODEL_RM_ISF_E");
this->get_parameter("force_sensor_model", force_sensor_model_);
this->get_parameter("connection_timeout", connection_timeout_);

// 根据参数选择机械臂型号
rm_robot_arm_model_e Model; // = RM_MODEL_RM_65_E;
rm_force_type_e Type; // = RM_MODEL_RM_B_E;
      
if (robot_arm_model_ == "RM_MODEL_RM_65_E") {
    Model = RM_MODEL_RM_65_E;
    RCLCPP_INFO(this->get_logger(), "使用机械臂型号: RM_MODEL_RM_65_E");
} else if (robot_arm_model_ == "RM_MODEL_RM_75_E") {
    Model = RM_MODEL_RM_75_E;
    RCLCPP_INFO(this->get_logger(), "使用机械臂型号: RM_MODEL_RM_75_E");
} else {
    RCLCPP_INFO(this->get_logger(), "使用机械臂型号: RM_MODEL_RM_75_E");
}
      
if (force_sensor_model_ == "RM_MODEL_RM_B_E") {
    Type = RM_MODEL_RM_B_E;
    RCLCPP_INFO(this->get_logger(), "使用传感器型号: RM_MODEL_RM_B_E");
} else {
    RCLCPP_INFO(this->get_logger(), "使用传感器型号: RM_MODEL_RM_ISF_E");
}
```
运行不同型号：

```bash
ros2 run rm_motion_planner test_fk --ros-args -p offline_mode:=true -p robot_model:=RM_MODEL_RM_75_E
```
#### 2.4 坐标系设置

为提高计算准确性，添加工作坐标系和工具坐标系设置：

```cpp
// 设置工作坐标系和工具坐标系
RCLCPP_INFO(this->get_logger(), "设置工作坐标系和工具坐标系...");
// Set the work frame
rm_frame_t coord_work;
coord_work.pose.position.x = 0.0f;
coord_work.pose.position.y = 0.0f;
coord_work.pose.position.z = 0.0f;
coord_work.pose.quaternion.w = 1.0f;
coord_work.pose.quaternion.x = 0.0f;
coord_work.pose.quaternion.y = 0.0f;
coord_work.pose.quaternion.z = 0.0f;
coord_work.pose.euler.rx = 0.0f;
coord_work.pose.euler.ry = 0.0f;
coord_work.pose.euler.rz = 0.0f;
coord_work.payload = 0.0f;
rm_algo_set_workframe(&coord_work);
// Set the tool frame
rm_frame_t coord_tool;
coord_tool.pose.position.x = 0.0f;
coord_tool.pose.position.y = 0.0f;
coord_tool.pose.position.z = 0.0f;
coord_tool.pose.quaternion.w = 1.0f;
coord_tool.pose.quaternion.x = 0.0f;
coord_tool.pose.quaternion.y = 0.0f;
coord_tool.pose.quaternion.z = 0.0f;
coord_tool.pose.euler.rx = 0.0f;
coord_tool.pose.euler.ry = 0.0f;
coord_tool.pose.euler.rz = 0.0f;
coord_tool.payload = 0.0f;
rm_algo_set_toolframe(&coord_tool);
      
RCLCPP_INFO(this->get_logger(), "坐标系设置完成");
```
#### 2.5 欧拉角支持

添加四元数转欧拉角功能，提供更直观的姿态表示：

```cpp
// 四元数转欧拉角函数
void quaternionToRPY(float qx, float qy, float qz, float qw, float& roll, float& pitch, float& yaw) {
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}
```
### 3. CMake配置修复

#### 3.1 链接错误

编译时出现链接错误：

```
/usr/bin/ld: CMakeFiles/test_fk.dir/test/test_fk.cpp.o: in function `FKTestNode::initialize_rm_api()':
test_fk.cpp:(.text._ZN10FKTestNode17initialize_rm_apiEv[_ZN10FKTestNode17initialize_rm_apiEv]+0x2a8): undefined reference to `rm_algo_init_sys_data'
...
```
#### 3.2 解决方案

修改CMakeLists.txt，为test_fk可执行文件添加rm_driver库链接：

```cmake
# 添加RM API 正运动学测试节点
add_executable(test_fk test/test_fk.cpp)
target_include_directories(test_fk PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${rm_driver_INCLUDE_DIRS}
)
target_link_libraries(test_fk ${rm_driver_LIBRARIES})
ament_target_dependencies(test_fk 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  builtin_interfaces
  trajectory_msgs
  control_msgs
  rm_ros_interfaces
  sclerp_motion_planner
  urdf
  Boost
)
```


### 4. 实践方案总结

#### ROS2参数管理

- 为节点添加`offline_mode`参数以支持离线测试
- 为网络连接相关的节点添加可配置的IP地址(`robot_ip`)和端口(`robot_port`)参数
- 使用ROS2标准参数传递方式（`--ros-args`）
- 支持动态参数配置（如`-p offline_mode:=true`）

#### 依赖管理

- 在CMakeLists.txt中使用`ament_target_dependencies`声明依赖关系
- 使用`target_link_libraries`为每个可执行文件添加必要的库链接
- 每次编译后必须执行`source install/setup.bash`更新环境

#### 错误处理

- 添加详细的日志输出以帮助诊断初始化过程
- 对关键初始化结果进行验证检查（如位置数据是否为零）
- 在调用RM API时添加详细的错误检查机制

#### 网络连接调试

- 使用telnet测试端口连通性：`telnet <机械臂IP> <端口号>`
- 检查Linux防火墙状态：`sudo ufw status`
- 如果网络连接存在问题，可以使用离线模式验证算法部分是否正常工作



### 5. 问题根因分析与最终解决方法

#### 问题描述

在运行test_fk节点时，出现以下错误信息：

```
[rm_get_arm_software_info] get_arm_software_info revice error
[rm_create_robot_arm] get robot info err!
[INFO] [1755651651.137487248] [fk_test_node]: 机器人句柄ID: -1
[ERROR] [1755651651.137679445] [fk_test_node]: 无效的机器人句柄ID
[INFO] [1755651651.137730519] [fk_test_node]: 尝试获取更多错误信息...
[ERROR] [1755651651.137766840] [fk_test_node]: 获取机器人信息失败，错误码: -2
[INFO] [1755651651.137797625] [fk_test_node]: 错误码说明:
[INFO] [1755651651.137849370] [fk_test_node]:   -1: 未找到对应句柄，句柄为空或已被删除
[INFO] [1755651651.137878939] [fk_test_node]:   -2: 获取到的机械臂基本信息非法，检查句柄是否已被删除
[ERROR] [1755651651.137907868] [fk_test_node]: 无效的机器人句柄
```



![运动学正解在线模式(连接真实机械臂)创建机器人控制句柄值为 -1 问题(2025-08-20_11-27)](Md_Pictures/运动学正解在线模式(连接真实机械臂)创建机器人控制句柄值为 -1 问题(2025-08-20_11-27).png)

#### 问题分析

根据API文档，句柄返回值为-1的问题可能有以下几个原因：

1. **网络连接问题**：机械臂IP地址或端口不正确
2. **机械臂未准备好**：机械臂未开启或未处于待连接状态
3. **防火墙问题**：网络防火墙阻止了连接
4. **达到最大连接数**：API文档提到最大连接数为5，可能已达到上限
5. **机械臂固件版本不兼容**：API版本与机械臂固件版本不匹配
6. **线程模式未正确初始化**：RM API需要特定的线程模式来处理网络通信和数据接收

#### 解决方案

通过在initialize_rm_api函数中添加线程模式初始化代码解决了该问题：

```cpp
// 初始化线程模式
int init_result = rm_init(RM_TRIPLE_MODE_E);
if (init_result != 0) {
    RCLCPP_ERROR(this->get_logger(), "初始化线程模式失败，错误码: %d", init_result);
    return;
}
RCLCPP_INFO(this->get_logger(), "线程模式初始化成功");
```

#### 原因解释

##### RM API线程模式的作用

在Realman机器人API中，线程模式决定了API如何处理网络通信和数据接收。根据 [API文档](https://develop.realman-robotics.com/robot/apic/classes/roboticArm/#%E5%88%9D%E5%A7%8B%E5%8C%96%E7%BA%BF%E7%A8%8B%E6%A8%A1%E5%BC%8Frm-init)，有三种线程模式：

1. **RM_SINGLE_MODE_E**: 单线程模式，单线程非阻塞等待数据返回

2. **RM_DUAL_MODE_E**: 双线程模式，增加接收线程监测队列中的数据

3. **RM_TRIPLE_MODE_E**: 三线程模式，在双线程模式基础上增加线程监测UDP接口数据

   

![Realman机械臂连接配置——初始化线程模式 rm_init (2025-08-20_12-15)](Md_Pictures/Realman机械臂连接配置——初始化线程模式 rm_init (2025-08-20_12-15).png)

##### 为什么添加线程模式初始化能解决问题

1. **网络通信机制**
   RM API需要特定的线程来处理与机械臂的网络通信。如果没有正确初始化线程模式：
   - API可能无法创建必要的接收线程
   - 网络数据包可能无法被正确接收和处理
   - 连接请求可能发送成功，但无法接收响应，导致连接超时
2. **数据接收处理**
   在未初始化线程模式的情况下：

   - API可能使用默认的单线程模式，这在某些网络环境下可能不够稳定
   - 三线程模式提供了更强大的数据接收和处理能力
   - 增加了UDP接口监测线程，可以更好地处理机械臂的实时状态信息
3. **连接建立过程**
   机械臂连接建立过程包括：

   - 发送连接请求到机械臂
   - 等待并接收机械臂的响应
   - 验证连接并创建句柄
     如果没有正确的线程支持，第2步可能失败，导致连接无法建立，句柄ID返回-1。
4. **错误处理机制**
   添加线程模式初始化后，确保了：

   - 线程模式正确初始化
   - 如果初始化失败，能及时发现并报告错误
   - 为后续的网络通信提供了必要的线程支持

#### 实际效果

通过添加线程模式初始化：

1. **确保了API的完整初始化**：API不仅需要算法数据初始化，还需要线程系统初始化
2. **提供了稳定的网络通信环境**：三线程模式提供了更可靠的网络数据处理能力
3. **增强了错误检测能力**：可以及时发现线程初始化失败的问题

![运动学正解在线模式(连接真实机械臂)解算成功(2025-08-20_13-40)](Md_Pictures/运动学正解在线模式(连接真实机械臂)解算成功(2025-08-20_13-40).png)



#### 最佳实践建议

在使用RM API时，应该按照以下顺序进行初始化：

1. 初始化线程模式（rm_init）
2. 设置日志回调（rm_set_log_call_back，可选但推荐）
3. 设置超时时间（rm_set_timeout）
4. 初始化算法数据（rm_algo_init_sys_data）
5. 创建机器人连接（rm_create_robot_arm）

这样可以确保API的所有组件都正确初始化，避免因缺少必要组件而导致的连接失败问题。

添加线程模式初始化代码解决了句柄返回-1的问题，是因为它为API提供了必要的线程支持，确保了网络通信和数据处理能够正常进行，从而使机械臂连接能够成功建立。





## 正运动学解算误差问题

### 1. 误差现象

基于RM-75机械臂离线模式计算的正运动学位置与示教器显示位置存在误差：

计算结果：

```
x=0.029906, y=0.406032, z=0.428469
```
示教器显示：

```
x=0.022655, y=0.420926, z=0.423825
```
### 2. 可能原因

#### 2.1 DH参数差异

机械臂的实际DH参数与RM API库中使用的默认参数可能存在差异。每台机械臂在生产过程中可能会有细微的制造公差，导致实际的连杆长度、关节偏移等参数与理论值不完全一致。

#### 2.2 坐标系设置问题

代码中设置了默认的工作坐标系和工具坐标系，但实际机械臂可能使用了不同的坐标系设置。

#### 2.3 机械臂型号配置

使用的是RM-75机械臂，但可能有不同版本或配置，使用的DH参数可能不同。

#### 2.4 关节角度输入误差

输入的关节角度与示教器上显示的角度不完全一致。

### 3. 解决方案建议

#### 3.1 校准DH参数

如果可能，获取这台机械臂的具体DH参数，并在计算时使用这些参数。

#### 3.2 检查坐标系设置

确保离线模式下使用的坐标系与机械臂实际使用的坐标系一致。

#### 3.3 使用在线模式验证

尝试使用在线模式连接真实的机械臂进行计算，看看结果是否更接近：

```
ros2 run rm_motion_planner test_fk
```

或

```bash
ros2 run rm_motion_planner test_fk --ros-args -p offline_mode:=false
```
#### 3.4 检查机械臂固件版本

确认机械臂的固件版本与RM API库版本是否兼容。



### 4. 未来改进方向

#### 1. 增强坐标系配置

- 提供更灵活的坐标系配置选项
- 支持从配置文件加载坐标系参数

#### 2. 误差补偿机制

- 引入误差补偿机制以提高计算精度
- 支持用户自定义DH参数

#### 3. 更多测试用例

- 添加更多关节角度组合的测试用例
- 支持批量测试和结果对比

#### 4. 可视化工具

- 添加简单的可视化工具以直观显示计算结果
- 支持结果导出功能





## 附录

### 日志信息说明

#### ROS 2日志前缀的组成部分

在运行ROS 2节点时，输出的日志信息前缀包含多个部分，用于标识日志的来源和上下文。

##### 1. [rm_sclerp_planner_node_1-1] [INFO] [1755669550.077916589] [rm_sclerp_planner_node_1] 格式

这个前缀包含四个部分：

1. **[rm_sclerp_planner_node_1-1]** - 进程名称和实例编号

   - [rm_sclerp_planner_node_1](file:///home/byd/Documents/Projects/DualArmProjects/realmanRobot/ros2_rm_ws/src/ros2_rm_robot-humble/rm_motion_planner/launch/rm_sclerp_planner_node_1.launch.py#L0-L0) 是节点的可执行文件名称
   - `-1` 表示这是该可执行文件的第一个实例（进程ID的一部分）
2. **[INFO]** - 日志级别

   - 表示这是一条INFO级别的日志消息
   - 其他可能的级别包括DEBUG、WARN、ERROR等
3. **[1755669550.077916589]** - 时间戳

   - Unix时间戳，表示自1970年1月1日以来的秒数和纳秒数
   - 格式为`秒.纳秒`
4. **[rm_sclerp_planner_node_1]** - 节点名称

   - 这是实际的ROS 2节点名称，通过代码中`Node("rm_sclerp_planner_node_1")`设置

##### 2. [INFO] [1755667669.531083124] [fk_test_node] 格式

这个前缀包含三个部分：

1. **[INFO]** - 日志级别
2. **[1755667669.531083124]** - 时间戳
3. **[fk_test_node]** - 节点名称

#### 区别解释

##### 为什么第一个格式多了一个进程标识？

主要原因是**启动方式不同**：

1. **rm_sclerp_planner_node_1** 通过launch文件启动

   - 使用了`ros2 launch`命令
   - 启动系统会为每个进程分配进程标识符
   - 因此日志包含了进程信息`[rm_sclerp_planner_node_1-1]`
2. **test_fk** 直接通过ros2 run命令启动

   - 使用了`ros2 run rm_motion_planner test_fk`命令
   - 直接运行可执行文件，不经过launch系统
   - 因此日志不包含进程标识信息

#### 详细分析

##### Launch文件启动的特点：

```bash
# 通过launch文件启动
ros2 launch some_package some_launch.py
```
- Launch系统管理多个节点和进程
- 每个进程需要唯一标识符来区分
- 日志系统会显示完整的进程信息

##### 直接运行的特点：

```bash
# 直接运行节点
ros2 run package_name executable_name
```
- 直接执行单个可执行文件
- 没有复杂的进程管理
- 日志系统简化了前缀显示

#### 实际影响

这两种日志格式在功能上没有区别，只是显示方式不同：

1. **内容相同**：都包含了日志级别、时间戳和节点名称
2. **功能一致**：都可以用于调试和问题追踪
3. **来源不同**：区别仅在于启动方式和日志系统的显示策略

#### 示例对比

```
通过launch启动的节点日志：
[my_node-1] [INFO] [1632456789.123456789] [my_node]: 这是一条日志消息

直接运行的节点日志：
[INFO] [1632456789.123456789] [my_node]: 这是一条日志消息
```
无论哪种格式，对于开发和调试来说都提供了相同的信息，只是在复杂的多节点系统中，带有进程标识的日志更容易追踪特定节点的行为。



### 代码功能解释

#### 1. 参数获取

```cpp
this->get_parameter("robot_model", robot_model_);
```

这行代码调用ROS 2节点的`get_parameter`方法，用于从参数服务器获取名为"robot_model"的参数值，并将其存储在`robot_model_`成员变量中。

#### 2. 数值后缀 f

在C++中，数字后缀 `f` 表示该数值是 `float` 类型（单精度浮点数），而不是默认的 `double` 类型（双精度浮点数）。

示例：

```cpp
float joint_angles_test[7] = {5.73f, 11.46f, 17.19f, 22.92f, 28.65f, 34.38f, 40.11f}; // 0.1-0.7弧度转换为度
```

#### 3. 弧度转角度说明

注释中"0.1-0.7弧度转换为度"的含义：

- 原始值是 0.1 到 0.7 弧度的等差数列（步长为 0.1）
- 通过公式：角度(度) = 弧度 × 180° / π 转换而来

具体转换过程：

```
0.1 弧度 = 0.1 × 180° / π ≈ 5.73°
0.2 弧度 = 0.2 × 180° / π ≈ 11.46°
0.3 弧度 = 0.3 × 180° / π ≈ 17.19°
0.4 弧度 = 0.4 × 180° / π ≈ 22.92°
0.5 弧度 = 0.5 × 180° / π ≈ 28.65°
0.6 弧度 = 0.6 × 180° / π ≈ 34.38°
0.7 弧度 = 0.7 × 180° / π ≈ 40.11°
```

