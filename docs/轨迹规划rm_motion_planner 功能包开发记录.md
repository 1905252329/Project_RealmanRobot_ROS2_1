# 联调阶段 rm_motion_planner 包开发记录

## 🎯  右臂节点接入感知后的功能逻辑重构

### 开发日期

20250926

### 背景与需求

原有的轨迹发布机制为定时器机制，即通过定时器回调函数（trajectory_timer_callback）发布轨迹点到 /rm_driver/movej_canfd_cmd话题上的，具体实现是先在节点的构造函数中定义了一个周期为 timer_period 的定时器 trajectory_timer，通过回调机制，以固定周期调用 trajectory_timer_callback 函数发布轨迹点到指定话题上。具体代码入下：

```c++
int execution_rate;
this->get_parameter("trajectory_execution_rate", execution_rate);
auto timer_period = std::chrono::microseconds(1000000 / execution_rate);

trajectory_timer_ = this->create_wall_timer(timer_period,
                    std::bind(&RmSclerpPlannerNode1::trajectory_timer_callback, this));
```

我已经将节点源文件中的 timer_callback 函数体中的轨迹规划代码放入了 box_centerPoint_callback 函数中，现在想把通过定时器回调机制（trajectory_timer_callback）发布轨迹点到 /rm_driver/movej_canfd_cmd话题上的代码实现，即 trajectory_timer_callback 函数体的代码移植到box_centerPoint_callback 函数体的轨迹规划部分代码之后，该如何实现

### 方案与实现

#### 方案说明

根据代码分析，你需要将轨迹执行相关的代码从[trajectory_timer_callback](javascript:void(0))函数移植到[box_centerPoint_callback](javascript:void(0))函数中。这将使系统在接收到盒子中心点位姿后直接执行轨迹，而不是通过定时器机制。

为了在[box_centerPoint_callback](javascript:void(0))函数中保持固定周期发布轨迹点，我们需要：

- 在[box_centerPoint_callback](javascript:void(0)) 函数中直接实现周期性发布逻辑
- 使用与原定时器相同的周期参数来控制发布频率
- 逐个发布轨迹点，而不是一次性发布所有轨迹点
- 在发布完所有轨迹点后执行后续操作（激活吸盘等）

具体需要移植的代码包括：

- 轨迹发布逻辑（使用[publishJointTrajectory](javascript:void(0))函数）
- 吸盘激活逻辑
- 移动到交接位置的逻辑
- 延时等待逻辑
- 吸盘关闭逻辑

#### 代码实现

下面是将[trajectory_timer_callback](javascript:void(0))中的代码移植到[box_centerPoint_callback](javascript:void(0))函数后的实现：

```cpp
/**
 * @brief 盒子中心点位姿回调函数
 * 
 * 处理从 /right_posedemo 话题接收到的盒子中心点位姿消息。
 * 
 * @param msg 接收到的 PoseStamped 消息
 */
void box_centerPoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if(!first_received_){
        first_received_ = true; 
    } else {
        return;
    }

    // 提取位置信息
    Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // 提取四元数并归一化
    Eigen::Quaterniond orientation(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
    orientation.normalize();
    
    // 打印接收到的位姿信息
    RCLCPP_INFO(this->get_logger(), "收到盒子中心点位姿 - 位置: [%f, %f, %f], 方向: [%f, %f, %f, %f]",
                position.x(), position.y(), position.z(),
                orientation.x(), orientation.y(), orientation.z(), orientation.w());
    
    // 进一步处理逻辑：
    // - 触发轨迹规划
    if (!trajectory_planned_) 
    {
        RCLCPP_INFO(this->get_logger(), "开始规划轨迹");

        Eigen::Quaterniond quaternion;
        Eigen::Vector3d tcpPos;
        quaternion = Eigen::Quaterniond(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
        
        tcpPos = Eigen::Vector3d(
            msg->pose.position.x + 0.075 + 0.1318,
            msg->pose.position.y,
            msg->pose.position.z);            

        // 获取参数
        std::string start_joints_str;       
        this->get_parameter("start_joint_positions", start_joints_str);
        // 打印参数
        RCLCPP_INFO(this->get_logger(), "原始起始关节角度字符串: %s", start_joints_str.c_str());
        // 解析起始关节位置参数
        Eigen::VectorXd jointAngles_start(7);
        parse_joint_positions(start_joints_str, jointAngles_start);       

        // 计算正向运动学
        Eigen::Matrix4d matrix_base2tcp_target;
        try {
            matrix_base2tcp_target = tcpPose2HomogenMatrix(quaternion, tcpPos);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "计算正向运动学时出错: %s", e.what());
            return;
        }
        
        // 调用重构后的sclerp_planner函数进行轨迹规划
        trajectory_msgs::msg::JointTrajectory joint_trajectory;
        sclerp_planner(jointAngles_start, matrix_base2tcp_target, joint_trajectory);
        
        // 标记轨迹已规划
        trajectory_planned_ = true;
        
        // 现在直接以固定周期执行轨迹发布（原trajectory_timer_callback中的逻辑）
        if (trajectory_planned_ && !joint_trajectory.points.empty()) 
        {
            // 获取轨迹执行频率参数
            int execution_rate;
            this->get_parameter("trajectory_execution_rate", execution_rate);
            
            RCLCPP_INFO(this->get_logger(), "开始发布轨迹，包含 %ld 个轨迹点，发布频率: %d Hz", 
                       joint_trajectory.points.size(), execution_rate);
            
            // 逐个发布轨迹点，保持固定周期
            for (const auto& point : joint_trajectory.points) {
                // 检查关节位置数据是否有效
                if (point.positions.size() < 7) {
                    RCLCPP_WARN(this->get_logger(), "轨迹点关节数据不足7个，跳过该点");
                    continue;
                }
                
                // 创建关节位置消息
                auto joint_msg = rm_ros_interfaces::msg::Jointpos();
                joint_msg.dof = 7;
                joint_msg.follow = true;
                joint_msg.expand = 0.0;
                
                // 填充关节位置数据
                joint_msg.joint.assign(point.positions.begin(), point.positions.begin() + 7);
                
                // 发布消息到机械臂驱动节点
                joint_pos_publisher_->publish(joint_msg);
                
                RCLCPP_DEBUG(this->get_logger(), "发布轨迹点: [%f, %f, %f, %f, %f, %f, %f]",
                            joint_msg.joint[0], joint_msg.joint[1], joint_msg.joint[2],
                            joint_msg.joint[3], joint_msg.joint[4], joint_msg.joint[5],
                            joint_msg.joint[6]);
                
                // 添加延迟以控制发布频率
                std::this_thread::sleep_for(std::chrono::microseconds(1000000 / execution_rate));
            }
            
            RCLCPP_INFO(this->get_logger(), "轨迹发布完成");
            
            // 激活吸盘
            std::string jodell_activate_cmd = "ros2 service call /right_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \""
                                "{}\"";
            call_service_sync(jodell_activate_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
            // 运动到交接位置
            std::vector<double> target_joint_position = {-86.658, 116.426, 12.293, -52.829, -12.231, 116.485, -95.181};
            rmRobotAPI_->rm_movej(target_joint_position, 50);  

            // 延时一段时间等待右臂到达交付点位
            std::this_thread::sleep_for(std::chrono::milliseconds(8000)); 

            std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \""
                                "{}\"";
            call_service_sync(jodell_deactivate_cmd);

            std::string jodell_CloseModbus_cmd = "ros2 service call /right_arm/close_modbus rm_ros_interfaces/srv/CloseModbus \""
                                "{}\"";
            call_service_sync(jodell_CloseModbus_cmd);
        }
    
    }

    first_received_ = false;
}
```

#### 改动总结

1. 将原本在[trajectory_timer_callback](javascript:void(0))中的轨迹执行逻辑移到[box_centerPoint_callback](javascript:void(0))函数中，使其在轨迹规划完成后立即执行。
2. 使用规划生成的[joint_trajectory](javascript:void(0))变量代替原来的[planned_trajectory_](javascript:void(0))成员变量。
3. 移除了对[trajectory_executing_](javascript:void(0))标志的检查，因为现在直接在回调函数中执行。
4. 移除了对[trajectory_timer_->cancel()](javascript:void(0))的调用，因为我们不再使用定时器。

这样修改后，当系统接收到盒子中心点位姿时，会立即进行轨迹规划并执行，而不需要通过定时器机制

#### 改动优化

将已存在的轨迹发布功能独立的函数 publishJointTrajectory 新增执行频率参数，变更后入下

```c++
    void publishJointTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory, int execution_rate)
    {
        // 检查轨迹数据是否有效
        if (trajectory.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "尝试发布空轨迹数据");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "开始发布轨迹，包含 %ld 个轨迹点，发布频率: %d Hz", 
                trajectory.points.size(), execution_rate);
        
        // 逐个发布轨迹点
        for (const auto& point : trajectory.points) {
            // 检查关节位置数据是否有效
            if (point.positions.size() < 7) {
                RCLCPP_WARN(this->get_logger(), "轨迹点关节数据不足7个，跳过该点");
                continue;
            }
            
            // 创建关节位置消息
            auto joint_msg = rm_ros_interfaces::msg::Jointpos();
            joint_msg.dof = 7;
            joint_msg.follow = true;
            joint_msg.expand = 0.0;
            
            // 填充关节位置数据
            joint_msg.joint.assign(point.positions.begin(), point.positions.begin() + 7);
            
            // 发布消息到机械臂驱动节点
            joint_pos_publisher_->publish(joint_msg);
            
            RCLCPP_DEBUG(this->get_logger(), "发布轨迹点: [%f, %f, %f, %f, %f, %f, %f]",
                        joint_msg.joint[0], joint_msg.joint[1], joint_msg.joint[2],
                        joint_msg.joint[3], joint_msg.joint[4], joint_msg.joint[5],
                        joint_msg.joint[6]);
            
            // 添加小延迟以控制发布频率
            // std::this_thread::sleep_for(std::chrono::milliseconds(20));
            std::this_thread::sleep_for(std::chrono::microseconds(1000000 / execution_rate));
        }
        
        RCLCPP_INFO(this->get_logger(), "轨迹发布完成");
    }
```



在话题订阅回调函数中添加规划器初始化，并调用 publishJointTrajectory 成员函数

```c++
void box_centerPoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if(!first_received_){
        first_received_ = true; 
    } else {
        return;
    }

    initialize_planner();
    // 检查规划器是否已初始化
    if (!planner_interface_ ) {
        RCLCPP_WARN(this->get_logger(), "规划器尚未初始化完成，无法执行轨迹规划");
        first_received_ = false;  // 允许下次再尝试
        return;
    }        

    // 位置信息提取(与存储)
    {
    Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // 提取四元数并归一化
    Eigen::Quaterniond orientation(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
    orientation.normalize();

    // 打印接收到的位姿信息
    RCLCPP_INFO(this->get_logger(), "收到盒子中心点位姿 - 位置: [%f, %f, %f], 方向: [%f, %f, %f, %f]",
                position.x(), position.y(), position.z(),
                orientation.x(), orientation.y(), orientation.z(), orientation.w());

    // 存储最新位姿
    // {
    //     std::lock_guard<std::mutex> lock(pose_mutex_);
    //     latest_box_pose_ = *msg;
    //     new_pose_received_ = true;

    // }
    }


    // 进一步处理逻辑
    {
    // - 触发轨迹规划与发布执行
    if (!trajectory_planned_) 
    {
        RCLCPP_INFO(this->get_logger(), "开始规划轨迹");

        Eigen::Quaterniond quaternion;
        Eigen::Vector3d tcpPos;
        quaternion = Eigen::Quaterniond(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);

        tcpPos = Eigen::Vector3d(
            msg->pose.position.x + 0.075 + 0.1318,
            msg->pose.position.y,
            msg->pose.position.z);            

        // 获取参数
        std::string start_joints_str;       
        this->get_parameter("start_joint_positions", start_joints_str);
        // 打印参数
        RCLCPP_INFO(this->get_logger(), "原始起始关节角度字符串: %s", start_joints_str.c_str());
        // 解析起始关节位置参数
        Eigen::VectorXd jointAngles_start(7);
        parse_joint_positions(start_joints_str, jointAngles_start);       

        // 计算正向运动学
        Eigen::Matrix4d matrix_base2tcp_target;
        try {
            matrix_base2tcp_target = tcpPose2HomogenMatrix(quaternion, tcpPos);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "计算正向运动学时出错: %s", e.what());
            return;
        }

        // 调用重构后的sclerp_planner函数进行轨迹规划
        trajectory_msgs::msg::JointTrajectory joint_trajectory;
        sclerp_planner(jointAngles_start, matrix_base2tcp_target, joint_trajectory);

        // 标记轨迹已规划
        trajectory_planned_ = true;

        // 以固定周期执行轨迹发布（原trajectory_timer_callback中的逻辑）
        if(trajectory_planned_ ){
            int execution_rate;
            this->get_parameter("trajectory_execution_rate", execution_rate);
            publishJointTrajectory(joint_trajectory, execution_rate);    

            // 后处理动作
            {
            // 激活吸盘
            std::string jodell_activate_cmd = "ros2 service call /right_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \""
                                "{}\"";
            call_service_sync(jodell_activate_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
            // 运动到交接位置
            std::vector<double> target_joint_position = {-86.658, 116.426, 12.293, -52.829, -12.231, 116.485, -95.181};
            rmRobotAPI_->rm_movej(target_joint_position, 50);  

            // 延时一段时间等待右臂到达交付点位
            std::this_thread::sleep_for(std::chrono::milliseconds(8000)); 

            std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \""
                                "{}\"";
            call_service_sync(jodell_deactivate_cmd);

            std::string jodell_CloseModbus_cmd = "ros2 service call /right_arm/close_modbus rm_ros_interfaces/srv/CloseModbus \""
                                "{}\"";
            call_service_sync(jodell_CloseModbus_cmd);
            }  

        } 

    }

    first_received_ = false;
    }

}
```



## 🎯  左右臂之间联动机制

### 开发日期

2025/09/27

### 背景与需求

已知信息

1) 项目整体功能概述

   - 当前ROS2 工作空间下的项目实现的任务是双臂协同进行纸箱的抓取，扫码与分类放置。

   - 右臂（节点rm_sclerp_planner_node_rightArm）负责从感知模块获取目标纸箱的6D 位姿信息，然后输入到轨迹规划器中获得轨迹点并以话题的形式发布到驱动节点控制机械臂执行轨迹运动，抓取物体再运动到指定位置等待左臂接收物体

   - 左臂（节点rm_sclerp_planner_node_leftArm）在右臂到达物体交付位置后，运到到交付点吸取物体，然后运到到扫码位置，完成扫码后再运动到指定位置放置物体。

2. 双臂当前节点设计模式

   - 当前右臂节点的设计模式为基于话题回调机制，

   - 当前左臂节点 的设计模式为基于动作驱动的机制。

3. 双臂节点重构目标与设计约束
   - 右臂：期望将其重构为基于动作驱动的机制，即将当前节点重构为一个动作服务端节点，具体实现类似当前左臂节点中的 grasp_box_server_ 动作服务，根据外部的动作客户端（中控系统下的一个动作客户端节点）发布的请求来触发动作服务（执行抓取物体和交付物体的任务）。
   - 左臂：期望左臂任务的触发信号由右臂节点产生

问题/任务：

- 右臂：在重构为动作服务端的前提下，如何实现发送信号给左臂执行任务？以话题的形式，还是以动作或是服务的形式实现？评估三种方案的可行性，并给出可行方案的具体实施步骤，与代码实现
- 左臂：为了能够实现接收右臂的触发信号的功能目标，左臂现有的动作服务端的设计机制是否要重构？如何根据右臂采用的可行方案（话题，或动作，或服务）作相应的调整？

要求：

​	只作方案说明与代码展示，不应用到当前项目中



### 方案说明与实现

根据对代码的分析，我现在可以提供关于双臂协同系统的重构方案建议。

基于对现有代码的分析，我将为右臂重构为动作服务端以及左右臂间通信提供三种方案的评估和推荐。

#### 三种通信方案的评估

##### 1. 基于话题的通信方案

话题通信是一种发布/订阅模式的异步通信方式。

**优点：**

- 实现简单，易于调试
- 松耦合，发送方和接收方不需要同时在线
- 支持多个订阅者同时接收同一消息

**缺点：**

- 无法确认消息是否被成功接收和处理
- 不适合需要精确控制和反馈的场景
- 难以处理复杂的交互逻辑

##### 2. 基于服务的通信方案

服务通信是请求/响应模式的同步通信方式。

**优点：**

- 提供请求/响应机制，可以得到明确的处理结果
- 实现相对简单，适合一次性的任务触发

**缺点：**

- 同步阻塞特性可能导致系统响应性下降
- 不适合长时间运行的任务
- 无法提供过程中的反馈信息

##### 3. 基于动作的通信方案

动作通信是一种**增强型**的**异步通信**方式，支持**目标**、**反馈**和**结果**。

**优点：**

- 提供完整的**请求-反馈-结果**机制
- 支持任务取消
- 适合长时间运行的任务
- 可以提供实时进度反馈

**缺点：**

- 实现相对复杂
- 系统开销略大

#### 推荐方案及实现

根据需求分析，右臂需要重构为动作服务端，左臂需要接收右臂的触发信号。

考虑到系统需要提供反馈、可能的任务取消以及长时间运行的特性，推荐使用**基于动作的通信方案**。

##### 1. 在左臂节点中添加动作服务端

（注意：下述代码中使用了命名空间加以区分，前提是没有设置节点级命名空间）

```cpp
// 左臂节点 - 添加动作服务端以接收右臂触发信号
#include "rclcpp_action/rclcpp_action.hpp"
#include "sorting_robot_interfaces/action/trigger_arm.hpp"

class RmSclerpPlannerNode1 : public rclcpp::Node
{
public:
    explicit RmSclerpPlannerNode1(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("rm_sclerp_planner_node_leftArm", options)
    {
        // 初始化其他组件...
        
        // 创建接收右臂触发信号的动作服务端
        trigger_left_arm_server_ = rclcpp_action::create_server<TriggerArm>(
            this,
            "left_arm/trigger_arm_action",  // 使用命名空间区分
            std::bind(&RmSclerpPlannerNode1:handle_trigger_left_arm_goal, this, _1, _2),
            std::bind(&RmSclerpPlannerNode1::handle_trigger_left_arm_cancel, this, _1),
            std::bind(&RmSclerpPlannerNode1::handle_trigger_left_arm_accepted, this, _1)
        );
        
        // 保留原有的抓取动作服务
        grasp_box_server_ = rclcpp_action::create_server<GraspBox>(
            this,
            "left_arm/grasp_action",
            std::bind(&RmSclerpPlannerNode1::handle_grasp_box_goal, this, _1, _2),
            std::bind(&RmSclerpPlannerNode1::handle_grasp_box_cancel, this, _1),
            std::bind(&RmSclerpPlannerNode1::handle_grasp_box_accepted, this, _1)
        );
    }

private:
    
    // 原有动作服务端
    rclcpp_action::Server<GraspBox>::SharedPtr grasp_box_server_;
    
    // 动作服务端 - 接收右臂触发信号
    rclcpp_action::Server<TriggerArm>::SharedPtr trigger_left_arm_server_;

    // 处理触发信号的目标回调
    rclcpp_action::GoalResponse handle_trigger_left_arm_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const TriggerArm::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "收到右臂触发信号: %s", goal->task_description.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 处理触发信号的取消回调
    rclcpp_action::CancelResponse handle_trigger_left_arm_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<TriggerArm>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "收到取消触发信号请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 处理触发信号的接受回调
    void handle_trigger_left_arm_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<TriggerArm>> goal_handle)
    {
        // 使用线程执行实际任务，避免阻塞
        std::thread{std::bind(&RmSclerpPlannerNode1::execute_trigger_left_arm, this, _1), goal_handle}.detach();
    }

    // 实际执行函数 - 接收到右臂触发信号后执行左臂任务
    void execute_trigger_left_arm(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<TriggerArm>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<TriggerArm::Feedback>();
        auto result = std::make_shared<TriggerArm::Result>();
        
        RCLCPP_INFO(this->get_logger(), "开始执行左臂任务: %s", goal->task_description.c_str());
        
        // 发布反馈 - 开始执行
        feedback->status = "开始执行左臂抓取任务";
        goal_handle->publish_feedback(feedback);
        
        // 执行左臂任务（吸取物体、扫码、放置等）
        bool success = execute_left_arm_task();
        
        if (success) {
            // 发布反馈 - 任务完成
            feedback->status = "左臂任务完成";
            goal_handle->publish_feedback(feedback);
            
            // 设置结果
            result->success = true;
            result->message = "左臂任务成功完成";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "左臂任务成功完成");
        } else {
            result->success = false;
            result->message = "左臂任务执行失败";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "左臂任务执行失败");
        }
    }
    
    // 左臂任务执行函数
    bool execute_left_arm_task() {
        // 实现左臂的吸取物体、扫码、放置等任务
        // 这里包含原有的左臂任务逻辑
        RCLCPP_INFO(this->get_logger(), "执行左臂抓取、扫码、放置任务");
        return true; // 简化示例
    }
};
```

##### 2. 在右臂节点中添加动作客户端

（注意：下述代码中使用了命名空间加以区分，前提是没有设置节点级命名空间）

```cpp
// 右臂节点 - 添加动作客户端以触发左臂任务
#include "rclcpp_action/rclcpp_action.hpp"
#include "sorting_robot_interfaces/action/trigger_arm.hpp"

class RmSclerpPlannerNodeRightArm : public rclcpp::Node
{
public:
    explicit RmSclerpPlannerNodeRightArm(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("rm_sclerp_planner_node_rightArm", options)
    {
        // 初始化其他组件...
        
        // 创建动作客户端以触发左臂任务
        trigger_left_arm_client_ = rclcpp_action::create_client<TriggerArm>(
            this,
            "left_arm/trigger_action"  // 连接到左臂的动作服务
        );
    }

private:
    // 动作客户端 - 用于触发左臂任务
    rclcpp_action::Client<TriggerArm>::SharedPtr trigger_left_arm_client_;

    // 触发左臂任务的函数
    void trigger_left_arm_task(const std::string& task_description) {
        if (!trigger_left_arm_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "左臂触发动作服务不可用");
            return;
        }
        
        // 创建目标
        auto goal_msg = TriggerArm::Goal();
        goal_msg.task_description = task_description;
        
        // 设置回调函数
        auto send_goal_options = rclcpp_action::Client<TriggerArm>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&RmSclerpPlannerNodeRightArm::trigger_goal_response_callback, this, _1);
        send_goal_options.feedback_callback = 
            std::bind(&RmSclerpPlannerNodeRightArm::trigger_feedback_callback, this, _1, _2);
        send_goal_options.result_callback = 
            std::bind(&RmSclerpPlannerNodeRightArm::trigger_result_callback, this, _1);
            
        // 发送目标
        trigger_left_arm_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "已发送触发左臂任务请求: %s", task_description.c_str());
    }
    
    // 目标响应回调
    void trigger_goal_response_callback(
        std::shared_future<rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "触发左臂任务目标被拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "触发左臂任务目标已接受");
        }
    }
    
    // 反馈回调
    void trigger_feedback_callback(
        rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr goal_handle,
        const std::shared_ptr<const TriggerArm::Feedback> feedback) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "左臂任务反馈: %s", feedback->status.c_str());
    }
    
    // 结果回调
    void trigger_result_callback(
        const rclcpp_action::ClientGoalHandle<TriggerArm>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "左臂任务成功完成: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "左臂任务被中止: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "左臂任务被取消: %s", result.result->message.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "左臂任务未知结果");
                break;
        }
    }
    
    // 在右臂完成交付任务后调用此函数触发左臂
    void on_right_arm_delivery_complete() {
        // 右臂完成抓取和交付任务后，触发左臂执行任务
        trigger_left_arm_task("开始执行抓取、扫码、放置任务");
    }
};
```

##### 3. 动作接口定义

需要在`sorting_robot_interfaces/action/`目录下创建`TriggerArm.action`文件：

```idl
# TriggerArm.action
string task_description
---
bool success
string message
---
string status
```

根据 ROS 2 项目的标准结构和最佳实践，新的动作接口定义应该存放在 [sorting_robot_interfaces](javascript:void(0)) 包中。这个包专门用于定义机器人系统中使用的各种接口，包括动作、服务和消息。

对于您提到的用于右臂触发左臂的动作接口，应该存放在以下目录：

```
src/ros2_rm_robot-humble/sorting_robot_interfaces/action/
```

这个目录中已经包含了其他动作接口定义文件，如：

- [GraspBox.action](javascript:void(0))
- [PlaceBox.action](javascript:void(0))
- [DetectMaterial.action](javascript:void(0))
- [ScanQR.action](javascript:void(0))

将新的动作接口定义文件放在这个目录下有以下优势：

1. **符合ROS 2标准**：ROS 2推荐将接口定义（包括动作、服务、消息）放在专门的接口包中
2. **保持一致性**：与现有的其他动作接口定义文件保持在同一位置
3. **便于管理**：所有机器人系统使用的接口都集中在一个包中
4. **易于构建**：CMakeLists.txt已经配置好处理该目录下的动作定义文件
5. **方便引用**：其他包可以方便地依赖和使用这些接口

新动作接口文件可以命名为`TriggerArm.action`或类似的具有描述性的名称，并放置在`src/ros2_rm_robot-humble/sorting_robot_interfaces/action/`目录下

##### 4. Launch文件配置

确保左右臂节点的launch文件正确配置命名空间：

```python
# 左臂launch文件
left_arm_node = Node(
    package='rm_motion_planner',
    executable='rm_sclerp_planner_node_leftArm',
    name='left_arm_node',
    namespace='left_arm',  # 左臂命名空间
    parameters=[/* 参数配置 */]
)

# 右臂launch文件
right_arm_node = Node(
    package='rm_motion_planner',
    executable='rm_sclerp_planner_node_rightArm',
    name='right_arm_node',
    namespace='right_arm',  # 右臂命名空间
    parameters=[/* 参数配置 */]
)
```

##### 总结

这个方案实现了正确的架构：

1. **左臂节点**包含动作服务端，监听来自右臂的触发信号
2. **右臂节点**包含动作客户端，在完成交付任务后触发左臂任务
3. 通过命名空间(`left_arm/trigger_action`)确保通信的准确性和隔离性
4. 保留了左臂原有的抓取动作服务，不影响现有功能

这样的设计符合双臂协同工作的实际需求，右臂完成抓取和交付后通知左臂开始执行自己的任务

##### 补充说明：命名空间应用

> 已经在左右臂的启动文件中添加了命名空间，在左右臂节点的动作服务端或是动作客户端的定义中还需要添加命名空间吗？比如在左臂节点中的动作服务端：
>
> ```c++
> trigger_left_arm_server_ = rclcpp_action::create_server<TriggerArm>(
>     this,
>     "left_arm/trigger_action",  // 使用命名空间区分
> ```

根据您提供的launch文件信息，您已经在launch文件中为左右臂节点添加了命名空间：

```python
# 右臂launch文件中
namespace='right_arm'

# 左臂launch文件中  
namespace='left_arm'
```

在这种情况下，**不需要**在节点代码中再次添加命名空间前缀。让我解释原因：

###### 1. 节点级命名空间的工作原理

当您在launch文件中设置了`namespace='left_arm'`，该节点下的所有通信接口（话题、服务、动作）都会自动加上这个前缀：

- 节点内创建的动作服务名称: `"trigger_action"`
- 实际注册的动作服务名称: `"/left_arm/trigger_action"`

###### 2. 正确的实现方式

```cpp
// 左臂节点中的动作服务端定义
trigger_left_arm_server_ = rclcpp_action::create_server<TriggerArm>(
    this,
    "trigger_action",  // 不需要再添加"left_arm/"前缀
    std::bind(&RmSclerpPlannerNode1::handle_trigger_goal, this, _1, _2),
    std::bind(&RmSclerpPlannerNode1::handle_trigger_cancel, this, _1),
    std::bind(&RmSclerpPlannerNode1::handle_trigger_accepted, this, _1)
);

// 右臂节点中的动作客户端定义
trigger_left_arm_client_ = rclcpp_action::create_client<TriggerArm>(
    this,
    "/left_arm/trigger_action"  // 这里需要使用绝对路径指定左臂的命名空间
);
```

###### 3. 详细说明

1. **左臂动作服务端**: 由于左臂节点已经有`left_arm`命名空间，所以直接使用相对名称`"trigger_action"`即可，系统会自动将其解析为`"/left_arm/trigger_action"`
2. **右臂动作客户端**: 由于右臂节点的命名空间是`right_arm`，要连接到左臂的服务，需要使用绝对路径`"/left_arm/trigger_action"`来明确指定目标

###### 4. 完整示例

```cpp
// 左臂节点
class RmSclerpPlannerNode1 : public rclcpp::Node
{
public:
    RmSclerpPlannerNode1() : Node("rm_sclerp_planner_node_leftArm") 
    {
        // 由于launch文件中设置了namespace="left_arm"
        // 这里使用相对名称即可
        trigger_server_ = rclcpp_action::create_server<TriggerArm>(
            this,
            "trigger_action",  // 实际名称会是 /left_arm/trigger_action
            // 回调函数...
        );
    }
};

// 右臂节点
class RmSclerpPlannerNodeRightArm : public rclcpp::Node
{
public:
    RmSclerpPlannerNodeRightArm() : Node("rm_sclerp_planner_node_rightArm")
    {
        // 由于launch文件中设置了namespace="right_arm"
        // 要连接左臂服务需要使用绝对路径
        trigger_client_ = rclcpp_action::create_client<TriggerArm>(
            this,
            "/left_arm/trigger_action",  // 明确指定左臂的命名空间
        );
    }
};
```

###### 总结

1. **不需要在节点内重复添加命名空间**：launch文件中的节点级命名空间会自动应用到所有通信接口
2. **客户端需要明确指定目标命名空间**：右臂客户端连接左臂服务时使用绝对路径`"/left_arm/trigger_action"`
3. **服务端使用相对路径**：左臂服务端直接使用`"trigger_action"`，由系统自动加上节点命名空间

这样既保持了代码的简洁性，又确保了跨节点通信的准确性。



### 开发总结

推荐使用基于动作的通信方案，将右臂重构为动作服务端，通过动作机制触发左臂任务。这种方案能够提供完整的请求-反馈-结果机制，支持任务取消，适合工业应用场景，并且与现有的左臂动作机制保持一致，便于维护和扩展。方案优势

- **一致性**：左右臂都使用动作机制，保持系统架构的一致性
- **可扩展性**：动作机制支持复杂的交互和反馈，便于未来功能扩展
- **可靠性**：动作机制提供完整的状态管理和错误处理
- **实时性**：支持任务取消和实时反馈

### QNA：右臂节点中动作客户端的启动和目标发送过程

#### 1. 动作客户端的启动

动作客户端在右臂节点的构造函数中创建，不需要显式的"启动"操作：

```cpp
// 右臂节点构造函数中创建动作客户端
class RmSclerpPlannerNodeRightArm : public rclcpp::Node
{
public:
    explicit RmSclerpPlannerNodeRightArm(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("rm_sclerp_planner_node_rightArm", options)
    {
        // 初始化其他组件...
        
        // 创建动作客户端以触发左臂任务
        // 客户端在创建后自动连接到对应的服务端
        trigger_left_arm_client_ = rclcpp_action::create_client<TriggerArm>(
            this,
            "/left_arm/trigger_action"  // 连接到左臂的动作服务
        );
        
        RCLCPP_INFO(this->get_logger(), "右臂动作客户端已创建");
    }
    
private:
    // 动作客户端 - 用于触发左臂任务
    rclcpp_action::Client<TriggerArm>::SharedPtr trigger_left_arm_client_;
};
```

#### 2. 发送动作目标给左臂服务端

发送目标是一个异步过程，分为几个步骤：

##### 步骤1：检查服务端是否可用

```cpp
// 检查动作服务是否可用
bool is_server_available = trigger_left_arm_client_->wait_for_action_server(std::chrono::seconds(5));
if (!is_server_available) {
    RCLCPP_ERROR(this->get_logger(), "左臂触发动作服务不可用");
    return;
}
```

##### 步骤2：创建目标消息

```cpp
// 创建目标消息
auto goal_msg = TriggerArm::Goal();
goal_msg.task_description = "开始执行抓取、扫码、放置任务";
```

##### 步骤3：设置回调选项

```cpp
// 设置回调选项
auto send_goal_options = rclcpp_action::Client<TriggerArm>::SendGoalOptions();

// 目标响应回调 - 当服务端接受或拒绝目标时调用
send_goal_options.goal_response_callback = 
    std::bind(&RmSclerpPlannerNodeRightArm::trigger_goal_response_callback, this, std::placeholders::_1);

// 反馈回调 - 当服务端发送反馈时调用
send_goal_options.feedback_callback = 
    std::bind(&RmSclerpPlannerNodeRightArm::trigger_feedback_callback, this, 
              std::placeholders::_1, std::placeholders::_2);

// 结果回调 - 当服务端返回最终结果时调用
send_goal_options.result_callback = 
    std::bind(&RmSclerpPlannerNodeRightArm::trigger_result_callback, this, std::placeholders::_1);
```

##### 步骤4：异步发送目标

```cpp
// 异步发送目标
auto goal_handle_future = trigger_left_arm_client_->async_send_goal(goal_msg, send_goal_options);
```

#### 3. 完整的触发函数实现

```cpp
// 触发左臂任务的完整实现
// 触发函数
void RmSclerpPlannerNodeRightArm::trigger_left_arm_task(const std::string& task_description) 
{
    // 1. 检查动作服务是否可用
    if (!trigger_left_arm_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "左臂触发动作服务不可用");
        return;
    }
    
    // 2. 创建目标消息
    auto goal_msg = TriggerArm::Goal();
    goal_msg.task_description = task_description;
    
    // 3. 设置回调选项
    auto send_goal_options = rclcpp_action::Client<TriggerArm>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
        std::bind(&RmSclerpPlannerNodeRightArm::trigger_goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = 
        std::bind(&RmSclerpPlannerNodeRightArm::trigger_feedback_callback, this, 
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = 
        std::bind(&RmSclerpPlannerNodeRightArm::trigger_result_callback, this, std::placeholders::_1);
        
    // 4. 异步发送目标
    auto goal_handle_future = trigger_left_arm_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "已发送触发左臂任务请求: %s", task_description.c_str());
}

// 目标响应回调函数
void RmSclerpPlannerNodeRightArm::trigger_goal_response_callback(
    std::shared_future<rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "触发左臂任务目标被拒绝");
    } else {
        RCLCPP_INFO(this->get_logger(), "触发左臂任务目标已接受");
    }
}

// 反馈回调函数
void RmSclerpPlannerNodeRightArm::trigger_feedback_callback(
    rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr goal_handle,
    const std::shared_ptr<const TriggerArm::Feedback> feedback) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "左臂任务反馈: %s", feedback->status.c_str());
}

// 结果回调函数
void RmSclerpPlannerNodeRightArm::trigger_result_callback(
    const rclcpp_action::ClientGoalHandle<TriggerArm>::WrappedResult & result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "左臂任务成功完成: %s", result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "左臂任务被中止: %s", result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "左臂任务被取消: %s", result.result->message.c_str());
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "左臂任务未知结果");
            break;
    }
    
}
```

#### 4. 实际触发时机

在右臂完成交付任务后触发左臂任务：

```cpp
// 在右臂节点中，当完成交付任务后调用此函数
void RmSclerpPlannerNodeRightArm::on_right_arm_delivery_complete() {
    // 右臂完成抓取和交付任务后，触发左臂执行任务
    trigger_left_arm_task("开始执行抓取、扫码、放置任务");
    
    // 或者在特定的回调函数中触发，例如：
    // 在box_centerPoint_callback中完成交付后调用
}
```

#### 5. 通信流程总结

1. **客户端创建**：在节点构造函数中创建动作客户端
2. **服务检测**：发送目标前检查服务端是否可用
3. **目标构建**：创建包含任务描述的目标消息
4. **回调设置**：设置目标响应、反馈和结果回调函数
5. **异步发送**：使用`async_send_goal`发送目标
6. **回调处理**：服务端处理过程中通过回调函数接收响应、反馈和结果

整个过程是异步的，不会阻塞右臂节点的其他操作，符合ROS 2动作通信的设计理念。



## 🎯  双臂使用相同动作的区分

### 开发日期

2025/09/27

### 背景与需求

根据上述功能开发中的方案说明与实现，是不是可以理解为 ROS2 的节点中支持动作的服务端与动作客户端的同时存在？前提是服务端的动作名称和客户端的动作名称不同。

如果左右两个臂都有抓取与放置的任务，且左右臂共用位于 src/ros2_rm_robot-humble/sorting_robot_interfaces/action 目录下的动作定义（GraspBox.action, PlaceBox.action）, 且左右臂执行不同任务的触发由一个中控节点负责，该如何定义左右臂节点中的动作服务，实现区分？

### 方案说明与实现

是的，您的理解是正确的。在ROS 2中，一个节点可以同时作为动作服务端和动作客户端，并且可以通过不同的动作名称来区分不同的动作服务。

对于左右臂的情况，我们可以使用命名空间来区分左右臂的动作服务。以下是如何实现的详细说明：

#### 1. 动作服务端与客户端共存

在ROS 2中，一个节点完全可以同时包含动作服务端和动作客户端。它们通过不同的动作名称进行区分。

#### 2. 左右臂动作服务的命名空间区分

对于左右臂使用**相同动作定义**但需要区分的情况，可以通过以下方式实现：

##### 步骤一：使用命名空间区分动作服务

```cpp
// 左臂节点中的动作服务定义
class LeftArmNode : public rclcpp::Node
{
public:
    LeftArmNode() : Node("left_arm_node")
    {
        // 左臂抓取动作服务，使用命名空间区分
        grasp_box_server_left_ = rclcpp_action::create_server<GraspBox>(
            this,
            "left_arm/grasp_action",  // 使用命名空间
            std::bind(&LeftArmNode::handle_grasp_goal, this, _1, _2),
            std::bind(&LeftArmNode::handle_grasp_cancel, this, _1),
            std::bind(&LeftArmNode::handle_grasp_accepted, this, _1));
            
        // 左臂放置动作服务
        place_box_server_left_ = rclcpp_action::create_server<PlaceBox>(
            this,
            "left_arm/place_action",  // 使用命名空间
            std::bind(&LeftArmNode::handle_place_goal, this, _1, _2),
            std::bind(&LeftArmNode::handle_place_cancel, this, _1),
            std::bind(&LeftArmNode::handle_place_accepted, this, _1));
    }
    
private:
    rclcpp_action::Server<GraspBox>::SharedPtr grasp_box_server_left_;
    rclcpp_action::Server<PlaceBox>::SharedPtr place_box_server_left_;
    // 左臂具体实现...
};

// 右臂节点中的动作服务定义
class RightArmNode : public rclcpp::Node
{
public:
    RightArmNode() : Node("right_arm_node")
    {
        // 右臂抓取动作服务，使用命名空间区分
        grasp_box_server_right_ = rclcpp_action::create_server<GraspBox>(
            this,
            "right_arm/grasp_action",  // 使用命名空间
            std::bind(&RightArmNode::handle_grasp_goal, this, _1, _2),
            std::bind(&RightArmNode::handle_grasp_cancel, this, _1),
            std::bind(&RightArmNode::handle_grasp_accepted, this, _1));
            
        // 右臂放置动作服务
        place_box_server_right_ = rclcpp_action::create_server<PlaceBox>(
            this,
            "right_arm/place_action",  // 使用命名空间
            std::bind(&RightArmNode::handle_place_goal, this, _1, _2),
            std::bind(&RightArmNode::handle_place_cancel, this, _1),
            std::bind(&RightArmNode::handle_place_accepted, this, _1));
    }
    
private:
    rclcpp_action::Server<GraspBox>::SharedPtr grasp_box_server_right_;
    rclcpp_action::Server<PlaceBox>::SharedPtr place_box_server_right_;
    // 右臂具体实现...
};
```

##### 步骤二：中控节点作为动作客户端

```cpp
// 中控节点作为动作客户端，控制左右臂
class CentralControlNode : public rclcpp::Node
{
public:
    CentralControlNode() : Node("central_control_node")
    {
        // 创建左臂动作客户端
        left_grasp_client_ = rclcpp_action::create_client<GraspBox>(
            this,
            "left_arm/grasp_action");
            
        left_place_client_ = rclcpp_action::create_client<PlaceBox>(
            this,
            "left_arm/place_action");
            
        // 创建右臂动作客户端
        right_grasp_client_ = rclcpp_action::create_client<GraspBox>(
            this,
            "right_arm/grasp_action");
            
        right_place_client_ = rclcpp_action::create_client<PlaceBox>(
            this,
            "right_arm/place_action");
    }
    
    // 控制左臂抓取
    void control_left_arm_grasp()
    {
        if (!left_grasp_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "左臂抓取动作服务不可用");
            return;
        }
        
        // 创建目标并发送
        auto goal_msg = GraspBox::Goal();
        goal_msg.target = "box1";
        
        auto send_goal_options = rclcpp_action::Client<GraspBox>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&CentralControlNode::left_grasp_goal_response_callback, this, _1);
        send_goal_options.feedback_callback = 
            std::bind(&CentralControlNode::left_grasp_feedback_callback, this, _1, _2);
        send_goal_options.result_callback = 
            std::bind(&CentralControlNode::left_grasp_result_callback, this, _1);
            
        left_grasp_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    // 控制右臂抓取
    void control_right_arm_grasp()
    {
        if (!right_grasp_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "右臂抓取动作服务不可用");
            return;
        }
        
        // 创建目标并发送
        auto goal_msg = GraspBox::Goal();
        goal_msg.target = "box2";
        
        auto send_goal_options = rclcpp_action::Client<GraspBox>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&CentralControlNode::right_grasp_goal_response_callback, this, _1);
        send_goal_options.feedback_callback = 
            std::bind(&CentralControlNode::right_grasp_feedback_callback, this, _1, _2);
        send_goal_options.result_callback = 
            std::bind(&CentralControlNode::right_grasp_result_callback, this, _1);
            
        right_grasp_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
private:
    // 左臂客户端
    rclcpp_action::Client<GraspBox>::SharedPtr left_grasp_client_;
    rclcpp_action::Client<PlaceBox>::SharedPtr left_place_client_;
    
    // 右臂客户端
    rclcpp_action::Client<GraspBox>::SharedPtr right_grasp_client_;
    rclcpp_action::Client<PlaceBox>::SharedPtr right_place_client_;
    
    // 回调函数实现...
    void left_grasp_goal_response_callback(
        std::shared_future<rclcpp_action::ClientGoalHandle<GraspBox>::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "左臂抓取目标被拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "左臂抓取目标已接受");
        }
    }
    
    void right_grasp_goal_response_callback(
        std::shared_future<rclcpp_action::ClientGoalHandle<GraspBox>::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "右臂抓取目标被拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "右臂抓取目标已接受");
        }
    }
    
    // 其他回调函数...
};
```

这两个步骤是**互补的，需要同时应用**，而不是二选一的关系。让我详细解释：

- **步骤一**：在左右臂节点中定义动作服务时使用命名空间，这样就创建了带命名空间的动作服务端点
- **步骤二**：在中控节点中创建动作客户端时使用相同的命名空间，这样才能正确连接到对应的动作服务

两者必须配合使用才能实现完整的通信机制：

```cpp
// 左臂节点 - 提供服务（步骤一）
grasp_box_server_left_ = rclcpp_action::create_server<GraspBox>(
    this,
    "left_arm/grasp_action",  // 服务端点名称
    // ... 其他参数
);

// 中控节点 - 请求服务（步骤二）
left_grasp_client_ = rclcpp_action::create_client<GraspBox>(
    this,
    "left_arm/grasp_action");  // 连接到相同名称的服务端点
```



#### 3. launch文件中的配置变动

在launch文件中，也需要相应地配置节点参数：

```python
# launch文件示例
def generate_launch_description():
    left_arm_node = Node(
        package='rm_motion_planner',
        executable='rm_sclerp_planner_node_leftArm',
        name='left_arm_node',
        parameters=[{
            'arm_side': 'left'
        }],
        # 其他参数...
    )
    
    right_arm_node = Node(
        package='rm_motion_planner',
        executable='rm_sclerp_planner_node_rightArm',
        name='right_arm_node',
        parameters=[{
            'arm_side': 'right'
        }],
        # 其他参数...
    )
    
    central_control_node = Node(
        package='central_control',
        executable='central_control_node',
        name='central_control_node'
    )
    
    return LaunchDescription([
        left_arm_node,
        right_arm_node,
        central_control_node
    ])
```

#### 总结

通过使用命名空间（如"left_arm/grasp_action"和"right_arm/grasp_action"）来区分不同臂的动作服务，可以实现：

1. 同一节点中动作服务端和客户端的共存
2. 通过不同名称区分左右臂的相同类型动作
3. 中控节点通过客户端精确控制指定臂的指定动作
4. 保持动作接口定义的复用性，无需为每个臂创建重复的动作定义

这种方案具有良好的可扩展性和维护性，符合ROS 2的最佳实践。

### QNA：动作命名空间的实现与解疑

#### 动作命名空间的实现

命名空间的定义**不是**在launch文件中通过parameters参数实现的，而是在代码中直接指定动作服务/客户端的名称。

正确的做法是在创建动作服务和客户端时直接在名称中包含命名空间：

```cpp
// 正确方式 - 在代码中直接指定命名空间
// 动作服务端（在左臂节点中）
auto server = rclcpp_action::create_server<GraspBox>(
    this,
    "left_arm/grasp_action",  // 直接在名称中包含命名空间
    // ... 回调函数
);

// 动作客户端（在中控节点中）
auto client = rclcpp_action::create_client<GraspBox>(
    this,
    "left_arm/grasp_action");  // 使用相同的命名空间名称连接
```

launch文件中的parameters参数主要用于传递节点参数，而不是定义动作服务的命名空间：

```python
# launch文件中的参数主要用于配置节点行为
left_arm_node = Node(
    package='rm_motion_planner',
    executable='rm_sclerp_planner_node_leftArm',
    name='left_arm_node',
    parameters=[{
        'initial_position': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'max_velocity': 1.0,
        # 这些是节点参数，不是动作命名空间
    }],
)
```

完整示例

以下是一个完整的示例，展示了如何在实际代码中实现：

```cpp
// 左臂节点实现
class LeftArmNode : public rclcpp::Node
{
public:
    LeftArmNode() : Node("left_arm_node")
    {
        // 创建带命名空间的动作服务
        grasp_server_ = rclcpp_action::create_server<GraspBox>(
            this,
            "left_arm/grasp_action",  // 命名空间在名称中定义
            std::bind(&LeftArmNode::handle_grasp_goal, this, _1, _2),
            std::bind(&LeftArmNode::handle_grasp_cancel, this, _1),
            std::bind(&LeftArmNode::handle_grasp_accepted, this, _1));
    }
    // ... 其他实现
};

// 中控节点实现
class CentralControlNode : public rclcpp::Node
{
public:
    CentralControlNode() : Node("central_control_node")
    {
        // 创建带相同命名空间的动作客户端
        left_grasp_client_ = rclcpp_action::create_client<GraspBox>(
            this,
            "left_arm/grasp_action");  // 与服务端相同的命名空间名称
            
        right_grasp_client_ = rclcpp_action::create_client<GraspBox>(
            this,
            "right_arm/grasp_action"); // 右臂使用不同的命名空间
    }
    
    void send_grasp_command_to_left_arm() {
        // 通过左臂客户端发送命令
        auto goal = GraspBox::Goal();
        goal.target = "box1";
        
        left_grasp_client_->async_send_goal(goal);
    }
    // ... 其他实现
};
```

总结

1. **步骤一和步骤二需要同时应用**：服务端和客户端必须使用相同的命名空间名称才能正确通信
2. **命名空间在代码中定义**：不是通过launch文件parameters参数实现，而是在创建动作服务/客户端时直接指定名称
3. **命名空间的作用**：通过不同的名称区分相同类型的动作服务，实现左右臂的独立控制

#### 解疑：ROS2 中的命名空间其他应用

> - 已知左右臂驱动节点(rm_driver)采用命名空间进行区分的方式是在 rm_75_left_driver.launch.py 文件中添加 参数项`namespace="left_arm"`。
> - 前述内容解释动作命名空间的定义不是在launch文件中通过parameters参数实现的，而是在代码中直接指定动作服务/客户端的名称。这是ROS2 的标准规范吗？
> - 话题或服务的命名空间定义方式也和动作一样吗？



是的，这确实是ROS 2的标准规范。让我详细解释一下：

##### 1. ROS 2命名空间机制

在ROS 2中，命名空间有两种不同的应用层面：

###### 节点级命名空间（Node-level Namespace）

这是通过launch文件中的`namespace`参数或命令行参数设置的，它会影响节点下所有话题、服务、动作等的名称。

```python
# 在launch文件中设置节点级命名空间
Node(
    package="rm_driver",
    executable="rm_driver",
    namespace="left_arm",  # 节点级命名空间
    # ...
)
```

这会导致该节点下所有通信接口都自动加上前缀：

- 话题 `/joint_states` 变成 `/left_arm/joint_states`
- 服务 `/get_params` 变成 `/left_arm/get_params`

###### 接口级名称（Interface-level Names）

这是在代码中创建具体通信接口时指定的名称，可以是相对路径或绝对路径：

```cpp
// 相对名称 - 会受节点命名空间影响
auto publisher = this->create_publisher<std_msgs::msg::String>("status", 10);
// 如果节点命名空间是"left_arm"，实际名称会是"/left_arm/status"

// 绝对名称 - 不受节点命名空间影响
auto publisher = this->create_publisher<std_msgs::msg::String>("/system/status", 10);
// 实际名称始终是"/system/status"

// 带命名空间的相对名称
auto publisher = this->create_publisher<std_msgs::msg::String>("left_arm/status", 10);
// 如果节点没有命名空间，实际名称是"/left_arm/status"
// 如果节点有命名空间"robot1"，实际名称是"/robot1/left_arm/status"
```

##### 2. 动作、话题、服务的命名空间处理方式

是的，动作、话题、服务的命名空间定义方式是一样的，都遵循ROS 2的统一命名规则：

```cpp
// 话题（Topic）
auto publisher = this->create_publisher<StdMsgs::msg::String>("left_arm/status", 10);
auto subscriber = this->create_subscription<StdMsgs::msg::String>("left_arm/status", 10, callback);

// 服务（Service）
auto server = this->create_service<SomeService>("left_arm/get_data", callback);
auto client = this->create_client<SomeService>("left_arm/get_data");

// 动作（Action）
auto action_server = rclcpp_action::create_server<SomeAction>(
    this, "left_arm/grasp_action", /* callbacks */);
auto action_client = rclcpp_action::create_client<SomeAction>(
    this, "left_arm/grasp_action");
```

##### 3. 实际应用中的最佳实践

在您的项目中，可以结合两种方式使用：

```python
# launch文件中设置节点级命名空间
Node(
    package="rm_motion_planner",
    executable="rm_sclerp_planner_node_leftArm",
    namespace="left_arm",  # 节点级命名空间
    name="motion_planner"
)
```



```cpp
// 代码中使用相对名称
class MotionPlannerNode : public rclcpp::Node {
public:
    MotionPlannerNode() : Node("motion_planner") {
        // 由于节点有"left_arm"命名空间，实际服务名会是"/left_arm/grasp_action"
        grasp_action_server_ = rclcpp_action::create_server<GraspBox>(
            this, "grasp_action", /* callbacks */);
    }
};
```



或者直接在代码中指定完整名称：

```cpp
// 代码中使用带命名空间的绝对名称
class MotionPlannerNode : public rclcpp::Node {
public:
    MotionPlannerNode() : Node("left_arm_motion_planner") {
        // 明确指定命名空间，不依赖节点级命名空间设置
        grasp_action_server_ = rclcpp_action::create_server<GraspBox>(
            this, "left_arm/grasp_action", /* callbacks */);
    }
};
```

##### 总结

1. **是的，这是ROS 2的标准规范**：动作、话题、服务的命名空间处理方式一致
2. **两种命名空间机制**：节点级命名空间（通过launch文件设置）和接口级名称（在代码中指定）
3. **灵活应用**：可以根据需要选择使用节点级命名空间自动处理，或在代码中明确指定完整名称
4. **一致性原则**：服务端和客户端必须使用完全相同的名称（包括命名空间）才能正确通信

您在任务04中的理解是正确的，命名空间是在创建通信接口时直接指定的，而不是通过parameters参数传递的。parameters主要用于传递配置参数，而不是定义接口名称。



### QNA：launch 文件配置变动说明

> 在launch文件中，配置的节点参数 `parameters=[{ 'arm_side': 'left'}]` 作用是什

launch文件中的`parameters=[{'arm_side': 'left'}]`这样的参数配置不是用于定义动作命名空间的，而是用于传递节点配置参数的。

让我解释一下这类参数的作用：

#### 1. ROS 2节点参数的作用

在ROS 2中，launch文件中的parameters参数用于向节点传递配置信息，这些配置信息可以在节点代码中通过`get_parameter()`方法获取。这些参数通常包括：

- 初始化配置值
- 运行时可调参数
- 设备特定的设置
- 算法参数等

#### 2. 参数的具体用途示例

```python
# launch文件中的参数配置示例
left_arm_node = Node(
    package='rm_motion_planner',
    executable='rm_sclerp_planner_node_leftArm',
    name='left_arm_node',
    parameters=[{
        'arm_side': 'left',                    # 标识机械臂类型
        'initial_joint_positions': [-1.0, -1.5, -0.5, -2.0, -1.5, 2.0, -0.3],
        'max_velocity': 1.0,                   # 最大速度限制
        'urdf_path': '/path/to/left_arm.urdf'  # URDF文件路径
    }],
)
```

在节点代码中，可以通过以下方式获取这些参数：

```cpp
class RmSclerpPlannerNode : public rclcpp::Node
{
public:
    RmSclerpPlannerNode() : Node("rm_sclerp_planner_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("arm_side", "unknown");
        this->declare_parameter<std::vector<double>>("initial_joint_positions");
        this->declare_parameter<double>("max_velocity", 0.5);
        this->declare_parameter<std::string>("urdf_path", "");
        
        // 获取参数值
        std::string arm_side;
        this->get_parameter("arm_side", arm_side);
        
        if (arm_side == "left") {
            RCLCPP_INFO(this->get_logger(), "初始化左臂节点");
            // 左臂特定的初始化逻辑
        } else if (arm_side == "right") {
            RCLCPP_INFO(this->get_logger(), "初始化右臂节点");
            // 右臂特定的初始化逻辑
        }
        
        // 使用其他参数...
        std::vector<double> initial_positions;
        this->get_parameter("initial_joint_positions", initial_positions);
        
        double max_velocity;
        this->get_parameter("max_velocity", max_velocity);
    }
};
```

#### 3. 与动作命名空间的区别

正如我们在任务05中讨论的：

- **Parameters参数**：用于传递节点配置信息，不参与接口命名
- **命名空间**：用于区分通信接口（动作、话题、服务）的名称

```python
# 正确的使用方式
left_arm_node = Node(
    package='rm_motion_planner',
    executable='rm_sclerp_planner_node_leftArm',
    namespace='left_arm',  # 节点级命名空间
    name='motion_planner',
    parameters=[{          # 节点配置参数
        'arm_side': 'left',
        'max_velocity': 1.0
    }],
)
```

#### 4. 实际项目中的应用

在您的项目中，`arm_side`参数可能用于：

1. **区分左右臂的特定逻辑**：

```cpp
std::string arm_side;
this->get_parameter("arm_side", arm_side);

if (arm_side == "left") {
    // 左臂特定的轨迹规划参数
    target_position_ = Eigen::Vector3d(-0.3, 0.5, 0.2);
} else {
    // 右臂特定的轨迹规划参数
    target_position_ = Eigen::Vector3d(-0.3, -0.5, 0.2);
}
```

1. **配置文件路径选择**：

```cpp
std::string config_path;
if (arm_side == "left") {
    config_path = "/config/left_arm_calibration.yaml";
} else {
    config_path = "/config/right_arm_calibration.yaml";
}
```

#### 总结

launch文件中的 parameters 参数用于向节点传递配置信息，而不是用于定义动作服务的命名空间。命名空间应该在创建动作服务/客户端时直接在名称中指定，或者通过节点级的 namespace 参数来设置。parameters 参数主要用于传递节点运行时需要的配置值。



#### 拓展：左右臂节点复用

> - 是否可以理解为通过 launch 文件的参数配置，传递到节点源文件中的节点类中，根据不同的参数而实例化不同的节点？即左右臂的节点实现为同一个源文件/同一个节点类，区分左右臂的方式是 launch 文件中的节点配置（节点名称和参数等）？
> - 若是上述推论成立，launch 文件中左右臂节点的 parameters 一项是否还可以用加载参数配置文件 YAML 的方式实现

##### 节点复用实现步骤

确实可以使用同一个源文件/节点类来实现左右臂功能，通过launch文件中的参数配置来区分：

###### 共用的节点源文件（节点类）实现

```cpp
// 单个节点类实现左右臂功能
class RmArmNode : public rclcpp::Node
{
public:
    RmArmNode() : Node("rm_arm_node")
    {
        // 声明并获取参数
        this->declare_parameter<std::string>("arm_side", "unknown");
        this->declare_parameter<std::vector<double>>("home_position");
        this->declare_parameter<std::string>("gripper_service_name");
        
        this->get_parameter("arm_side", arm_side_);
        
        // 根据参数区分左右臂行为
        if (arm_side_ == "left") {
            // 左臂特定配置
            target_pose_ = Eigen::Vector3d(-0.3, 0.5, 0.2);
            gripper_service_name_ = "/left_arm/activate_gripper";
        } else if (arm_side_ == "right") {
            // 右臂特定配置
            target_pose_ = Eigen::Vector3d(-0.3, -0.5, 0.2);
            gripper_service_name_ = "/right_arm/activate_gripper";
        }
        
        // 创建带命名空间的通信接口
        std::string action_name = arm_side_ + "_arm/grasp_action";
        action_server_ = rclcpp_action::create_server<GraspBox>(
            this, action_name, /* callbacks */);
    }

private:
    std::string arm_side_;
    std::string gripper_service_name_;
    Eigen::Vector3d target_pose_;
    rclcpp_action::Server<GraspBox>::SharedPtr action_server_;
};
```

###### 对应的共用 launch文件配置：

```python
# 左臂节点配置
left_arm_node = Node(
    package='rm_motion_planner',
    executable='rm_arm_node',  # 同一个可执行文件
    name='left_arm_node',
    parameters=[{
        'arm_side': 'left',
        'home_position': [-1.0, -1.5, -0.5, -2.0, -1.5, 2.0, -0.3],
        'gripper_service_name': '/left_arm/activate_gripper'
    }],
    namespace='left_arm'
)

# 右臂节点配置
right_arm_node = Node(
    package='rm_motion_planner',
    executable='rm_arm_node',  # 同一个可执行文件
    name='right_arm_node',
    parameters=[{
        'arm_side': 'right',
        'home_position': [-1.0, 1.5, -0.5, -2.0, -1.5, 2.0, -0.3],
        'gripper_service_name': '/right_arm/activate_gripper'
    }],
    namespace='right_arm'
)
```

###### 参数配置还可以通过 YAML 配置文件来实现

是的，完全可以通过YAML 配置文件来实现参数配置，这是ROS 2中的标准做法：

```yaml
# arm_config.yaml
left_arm_node:
  ros__parameters:
    arm_side: "left"
    home_position: [-1.0, -1.5, -0.5, -2.0, -1.5, 2.0, -0.3]
    max_velocity: 1.0
    gripper_service_name: "/left_arm/activate_gripper"
    urdf_path: "/path/to/left_arm.urdf"

right_arm_node:
  ros__parameters:
    arm_side: "right"
    home_position: [-1.0, 1.5, -0.5, -2.0, -1.5, 2.0, -0.3]
    max_velocity: 1.0
    gripper_service_name: "/right_arm/activate_gripper"
    urdf_path: "/path/to/right_arm.urdf"
```

然后在launch文件中加载：

```python
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取配置文件路径
    config = os.path.join(
        get_package_share_directory('rm_motion_planner'),
        'config',
        'arm_config.yaml'
    )
    
    # 加载YAML配置
    with open(config, 'r') as f:
        params = yaml.safe_load(f)
    
    left_arm_node = Node(
        package='rm_motion_planner',
        executable='rm_arm_node',
        name='left_arm_node',
        parameters=[params['left_arm_node']['ros__parameters']],
        namespace='left_arm'
    )
    
    right_arm_node = Node(
        package='rm_motion_planner',
        executable='rm_arm_node',
        name='right_arm_node',
        parameters=[params['right_arm_node']['ros__parameters']],
        namespace='right_arm'
    )
    
    return LaunchDescription([
        left_arm_node,
        right_arm_node
    ])
```

##### 优势分析

使用单一节点类配合参数配置的方式有以下优势：

1. **代码复用性高**：避免重复实现相似功能
2. **维护成本低**：只需维护一份代码
3. **配置灵活**：通过参数轻松调整行为
4. **扩展性强**：添加新臂或修改配置只需调整参数

##### 实际应用示例

```cpp
// 节点类中根据参数执行不同逻辑
void RmArmNode::execute_grasp_action() {
    if (arm_side_ == "left") {
        // 左臂特定的抓取轨迹
        move_to_pose(left_grasp_pose_);
    } else {
        // 右臂特定的抓取轨迹
        move_to_pose(right_grasp_pose_);
    }
    
    // 调用对应的夹爪服务
    call_service(gripper_service_name_);
}
```

##### 总结

1. **是的**，可以使用同一个节点类通过参数配置实现左右臂功能
2. **是的**，可以通过YAML配置文件方式管理参数，这是推荐做法
3. 这种方式符合ROS 2的最佳实践，提高了代码的复用性和可维护性
4. 命名空间和参数配置可以结合使用，实现更灵活的系统架构




## 🎯  基于动作的中控程序触发双臂动作执行与交互

### 开发日期

2025/10/16

### 现状分析

#### 双臂作业流程

感知节点发布6D 位姿话题 -> 右臂规控节点订阅 6D 位姿话题 -> 右臂执行纸箱吸取动作 -> 右臂请求左臂接收纸箱动作 -> 左臂响应并执行接收纸箱动作

#### 现有方案说明

基于章节《右臂节点接入感知后的功能逻辑重构》与《左右臂联动机制设计与实现》双臂规控节点的实现方案是：

1）右臂

- 话题订阅者：订阅感知箱体6D 位姿信息
- 话题发布者：发布关节位置信息给驱动节点 m_driver
- 动作客户端：请求左臂执行接受纸箱与放置动作
- 动作服务端：为中控程序预留，未调用

2）左臂

- 话题发布者：发布关节位置信息给驱动节点 rm_drvier
- 动作服务端：响应右臂请求左臂执行接受纸箱与放置动作
- 动作服务端：为中控程序预留，未调用

#### 代码实现说明

采用延时机制

```c++
 void execute_grasp_box(const std::shared_ptr<GraspBoxHandle> goal_handle)
 {
 	// ... 其他代码...
 	
    bool triggerArm = true;
    if(triggerArm)
    {
    	// ... 其他代码 ...
        // 触发左臂动作
        RCLCPP_INFO(this->get_logger(), "%s 触发左臂动作服务", REQUEST_ICON.c_str());  
        trigger_left_arm_task("开始执行抓取、扫码、放置任务"); 
        
    }
    
        // 延时用于等待左臂吸取到纸箱
        std::this_thread::sleep_for(std::chrono::milliseconds(4000)); 
        // 关闭吸盘（TODO：改为动作返回结果触发吸盘关闭）
        std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \""
                            "{}\"";
        call_service_sync(jodell_deactivate_cmd);

        // 机械臂复位到初始位置
        std::vector<double> initial_joint_position = {-44.811, 69.029, -48.806, 71.437, -40.008, 83.439, -62.740};
        rmRobotAPI_->rm_movej(initial_joint_position, 50);

        // 延时用于等待左臂完成任务
        std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 

        // 输出最终结果
        if(rclcpp::ok()){
            result->success = true;
            result->message = "Grasp box action completed successfully";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Grasp box action succeeded");
        }    
 	
 }
```



**存在的问题**

- 当前 `trigger_left_arm_task()` 使用 `async_send_goal()` 异步发送目标，立即返回，不等待结果。
- 右臂继续执行后续代码（如关闭吸盘、复位），无法感知左臂任务是否成功。



### 开发目标

##### 一句话描述

由中控程序(节点)统一负责感知信息的接收，发布与双臂作业流程的任务决策与调度。

##### 工程实现策略

- 双臂规控节点与中控节点以动作机制交互通信
- 双臂规孔节点之间以动作机制交互通信

##### 代码模块实现

- 话题发布者：发布关节位置信息给驱动节点 rm_driver
- 动作客户端：请求左臂执行接受纸箱与放置动作
- 动作服务端：接收中控程序动作请求

##### 改进方向量化

已知

- 在 rm_sclerp_planner_node_rightArm.cpp 的 execute_grasp_box 函数中通过 trigger_left_arm_task 函数调用 trigger_left_arm_client_ 发送动作执行请求。
- 在rm_sclerp_planner_node_leftArm.cpp 中 trigger_left_arm_client_ 响应上述动作请求， 具体是通过 execute_trigger_left_arm 函数调用 execute_left_arm_task 函数，进而调用 tempForShow 函数，实现左臂执行期望的动作任务。

目标：

希望 rm_sclerp_planner_node_rightArm.cpp 中的函数 execute_grasp_box 结尾输出最终结果代码前添加获取 rm_sclerp_planner_node_leftArm.cpp 中 execute_trigger_left_arm 函数的最终结果， 即将左臂的 trigger_left_arm_server_ 动作的执行结果传递给右臂,进而右臂的 grasp_box_server_  输出整个任务的执行结果。

总结：**右臂发送动作目标后，非阻塞，根据左臂吸取目标纸箱成败的返回结果，决定执行后续动作**

### 可行性方案



#### 方案一：**同步阻塞模式**

要实现右臂在 `execute_grasp_box` 中等待并获取左臂 `trigger_left_arm_server_` 动作执行结果，需将当前异步调用改为同步等待模式。核心思路是：**右臂发送动作目标后，阻塞等待其完成，并通过客户端的 `async_send_goal` 返回的 `Future` 获取最终结果**。

适用于任务流明确、需严格顺序执行的场景。



##### ✅ 功能方案说明

1. **现状分析**
   - 当前 `trigger_left_arm_task()` 使用 `async_send_goal()` 异步发送目标，立即返回，不等待结果。
   - 右臂继续执行后续代码（如关闭吸盘、复位），无法感知左臂任务是否成功。
2. **改进方案**
   - 修改 `trigger_left_arm_task()` 为同步函数，返回 `TriggerArm::ResultCode` 和结果消息。
   - 使用 `async_send_goal()` 获取 `std::shared_future<GoalHandle>`，再通过 `get_result()` 阻塞等待结果。
   - 在右臂 `execute_grasp_box` 中根据左臂返回结果决定自身任务成败。
3. **关键点**
   - 使用 `rclcpp_action::Client<TriggerArm>::SendGoalOptions` 设置回调非必需（因同步等待）。
   - 增加超时机制防止无限等待。
   - 线程安全：`execute_grasp_box` 运行在独立线程中，阻塞不会影响节点其他功能。

**实施步骤**

- 修改 `trigger_left_arm_task()` 为同步函数，返回 `TriggerArm::ResultCode` 和结果消息。
- 使用 `async_send_goal()` 获取 `std::shared_future<GoalHandle>`，再通过 `get_result()` 阻塞等待结果。
- 在右臂 `execute_grasp_box` 中根据左臂返回结果决定自身任务成败。
- 使用 `rclcpp_action::Client<TriggerArm>::SendGoalOptions` 设置回调非必需（因同步等待）。
- 增加超时机制防止无限等待,超时时间（如 `30s`）应根据实际任务耗时合理设置。
- 线程安全：`execute_grasp_box` 运行在独立线程中，阻塞不会影响节点其他功能
- 需确保 `trigger_left_arm_server_` 在左臂节点中正确返回 `succeed/abort/cancel` 结果。

##### 代码实现

🧩 改进代码展示（仅展示需修改部分）

1. 修改 `trigger_left_arm_task` 函数（返回同步结果）

```cpp
// 新增同步版本的触发函数，返回结果
bool RmSclerpPlannerNode1::trigger_left_arm_task_sync(
    const std::string& task_description,
    std::string& out_result_message)
{
    if (!trigger_left_arm_client_->wait_for_action_server(std::chrono::seconds(5))) 
    {
        RCLCPP_ERROR(this->get_logger(), "左臂触发动作服务不可用");
        out_result_message = "Left arm action server not available";
        return false;
    }

    auto goal_msg = TriggerArm::Goal();
    goal_msg.task_description = task_description;

    // 同步发送目标并等待结果
    auto future_goal_handle = trigger_left_arm_client_->async_send_goal(goal_msg);
    if (future_goal_handle.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "等待左臂动作目标接受超时");
        out_result_message = "Timeout waiting for goal acceptance";
        return false;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "左臂动作目标被拒绝");
        out_result_message = "Goal rejected by left arm";
        return false;
    }

    // 等待结果
    auto future_result = goal_handle->async_result();
    if (future_result.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "等待左臂动作结果超时");
        out_result_message = "Timeout waiting for left arm result";
        return false;
    }

    auto wrapped_result = future_result.get();
    switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            out_result_message = "Left arm task succeeded: " + wrapped_result.result->message;
            RCLCPP_INFO(this->get_logger(), "%s", out_result_message.c_str());
            return true;
        case rclcpp_action::ResultCode::ABORTED:
            out_result_message = "Left arm task aborted: " + wrapped_result.result->message;
            RCLCPP_ERROR(this->get_logger(), "%s", out_result_message.c_str());
            return false;
        case rclcpp_action::ResultCode::CANCELED:
            out_result_message = "Left arm task canceled: " + wrapped_result.result->message;
            RCLCPP_WARN(this->get_logger(), "%s", out_result_message.c_str());
            return false;
        default:
            out_result_message = "Unknown result from left arm";
            return false;
    }
}
```

2. 在 `execute_grasp_box` 中调用同步函数并处理结果

```c++
void execute_grasp_box(const std::shared_ptr<GraspBoxHandle> goal_handle)
{
    // ... 其他逻辑保持不变 ...

    // 触发左臂任务并等待结果
    std::string left_arm_result_msg;
    bool left_arm_success = trigger_left_arm_task_sync("开始执行抓取、扫码、放置任务", left_arm_result_msg);

    // 根据左臂结果决定本任务成败
    if (!left_arm_success) {
        result->success = false;
        result->message = "Dependent left arm task failed: " + left_arm_result_msg;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Aborting grasp box due to left arm failure: %s", left_arm_result_msg.c_str());
        return;
    }

    // 左臂成功，继续后续操作（如延时、关吸盘等）
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

    std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
    call_service_sync(jodell_deactivate_cmd);

    // 机械臂复位
    std::vector<double> initial_joint_position = {-44.811, 69.029, -48.806, 71.437, -40.008, 83.439, -62.740};
    rmRobotAPI_->rm_movej(initial_joint_position, 50);

    // 输出最终结果
    if(rclcpp::ok()){
        result->success = true;
        result->message = "Grasp box and left arm task both succeeded";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Grasp box action succeeded");
    }
}
```



#### 方案二：异步非阻塞监听模式

> 希望非阻塞但仍能响应结果，即保留异步回调机制，并通过状态标志或事件队列通知主流程。

非阻塞但仍能响应结果，应保留异步回调机制，并通过状态标志或事件队列通知主流程

要实现 **非阻塞异步等待左臂动作结果**，同时让右臂的 `execute_grasp_box` 能响应最终结果，应采用 **异步回调 + 状态标志 + 条件变量（Condition Variable）** 的机制。

##### ✅ 功能方案说明（非阻塞、响应式）

🎯 目标

在不阻塞主线程的前提下：

- 右臂发送 `trigger_left_arm_task` 后继续执行其他操作（如激活吸盘、避障移动）
- 当左臂任务完成时，通过回调函数接收结果
- 在右臂 `execute_grasp_box` 中能“感知”到左臂的结果，并据此决定最终任务成败

🔧 核心设计

1. **共享状态结构体**：用于存储左臂任务的执行结果和完成状态。
2. **条件变量（`std::condition_variable`）**：右臂线程可等待该变量被通知。
3. **互斥锁（`std::mutex`）**：保护共享状态的线程安全。
4. **异步回调接收结果**：通过 `result_callback` 更新共享状态并通知等待线程。
5. **限时等待（`wait_for`）**：避免无限期挂起，保证系统健壮性。

------

##### 🧩 改进代码展示（仅关键修改部分）

1. 在类私有成员中添加共享状态与同步机制

```cpp
private:
    // ... existing code ...

    // 用于同步左臂任务结果的结构体
    struct LeftArmResult {
        std::atomic<bool> completed{false};
        bool success{false};
        std::string message;
    };

    LeftArmResult left_arm_result_;
    std::mutex result_mutex_;
    std::condition_variable result_cv_;  // 用于通知结果到达
```

------

2. 修改 `trigger_left_arm_task` 为异步监听模式

```cpp
void RmSclerpPlannerNode1::trigger_left_arm_task(const std::string& task_description) 
{
    if (!trigger_left_arm_client_->wait_for_action_server(std::chrono::seconds(5))) 
    {
        RCLCPP_ERROR(this->get_logger(), "左臂触发动作服务不可用");
        return;
    }

    auto goal_msg = TriggerArm::Goal();
    goal_msg.task_description = task_description;

    auto send_goal_options = rclcpp_action::Client<TriggerArm>::SendGoalOptions();
    
    // ❗关键：设置结果回调，用于更新共享状态
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<TriggerArm>::WrappedResult& result) {
        std::lock_guard<std::mutex> lock(result_mutex_);

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                left_arm_result_.success = true;
                left_arm_result_.message = "Left arm succeeded: " + result.result->message;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                left_arm_result_.success = false;
                left_arm_result_.message = "Left arm aborted: " + result.result->message;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                left_arm_result_.success = false;
                left_arm_result_.message = "Left arm canceled: " + result.result->message;
                break;
            default:
                left_arm_result_.success = false;
                left_arm_result_.message = "Left arm unknown failure";
                break;
        }

        left_arm_result_.completed = true;
        result_cv_.notify_one();  // 通知等待线程
        RCLCPP_INFO(this->get_logger(), "左臂任务结果已接收: %s", left_arm_result_.message.c_str());
    };

    trigger_left_arm_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "已发送触发左臂任务请求: %s", task_description.c_str());
}
```

------

3. 在 `execute_grasp_box` 中等待左臂结果（非阻塞式等待）

```cpp
void execute_grasp_box(const std::shared_ptr<GraspBoxHandle> goal_handle)
{
    // ... 其他初始化、轨迹执行、吸盘激活等逻辑 ...

    // ✅ 触发左臂任务（异步，立即返回）
    trigger_left_arm_task("开始执行抓取、扫码、放置任务");

    // ✅ 可在此期间执行其他非依赖操作（如避障移动）
    rm_current_arm_state_t rm_robot_current_state;
    rm_get_arm_state(rm_robot_current_state);
    std::vector<float> gripper_tcp_pose = {
        rm_robot_current_state.pose.position.x + 0.15f,
        rm_robot_current_state.pose.position.y,
        rm_robot_current_state.pose.position.z,
        rm_robot_current_state.pose.euler.rx,
        rm_robot_current_state.pose.euler.ry,
        rm_robot_current_state.pose.euler.rz
    };
    rm_movel_cmd(gripper_tcp_pose, 40);

    // ✅ 等待左臂任务完成（带超时的非忙等待）
    bool left_arm_done = false;
    {
        std::unique_lock<std::mutex> lock(result_mutex_);
        auto wait_ret = result_cv_.wait_for(lock, std::chrono::seconds(30), [this] {
            return left_arm_result_.completed.load();
        });

        if (wait_ret) {
            left_arm_done = true;
        } else {
            left_arm_result_.success = false;
            left_arm_result_.message = "等待左臂任务超时 (30s)";
            left_arm_result_.completed = true;
        }
    }

    // ✅ 根据左臂结果决定本任务成败
    if (!left_arm_result_.success) {
        result->success = false;
        result->message = "协同任务失败: " + left_arm_result_.message;
        goal_handle->abort(result);
        RCLCPP_WARN(this->get_logger(), "右臂任务因左臂失败而终止: %s", left_arm_result_.message.c_str());
        return;
    }

    // ✅ 左臂成功，继续收尾
    std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
    call_service_sync(jodell_deactivate_cmd);

    std::vector<double> initial_joint_position = {-44.811, 69.029, -48.806, 71.437, -40.008, 83.439, -62.740};
    rmRobotAPI_->rm_movej(initial_joint_position, 50);

    // ✅ 最终成功
    result->success = true;
    result->message = "双臂协同任务成功完成";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "右臂任务成功，左臂也已完成");
}
```

------

##### 🔄 流程图（文字描述）

```
右臂 execute_grasp_box()
    ↓
触发左臂任务 → 异步发送目标（不等待）
    ↓
执行本地操作（如吸盘、避障）
    ↓
等待 result_cv_（最多30秒）
    ↙               ↘
左臂完成 ← 回调触发 → 更新 left_arm_result_ 并 notify
    ↓
收到结果 or 超时
    ↓
根据 success 字段决定 succeed/abort
```

------

##### ⚠️ 注意事项

- **线程安全**：所有对 `left_arm_result_` 的读写都必须加锁。
- **超时机制**：防止死锁，建议设置合理超时（如30秒）。
- **状态重置**：若支持重复调用，应在每次调用前重置 `left_arm_result_` 状态。
- **避免竞态**：确保 `result_callback` 不会在对象销毁后被调用（可通过 `weak_from_this` 优化）。
- **日志清晰**：建议记录时间戳，便于调试异步流程。

------

##### ✅ 优势总结

| 特性       | 说明                           |
| :--------- | :----------------------------- |
| **非阻塞** | 发送任务后立即继续执行本地操作 |
| **响应式** | 通过回调实时感知结果           |
| **可控性** | 支持超时、取消、错误传播       |
| **可扩展** | 易于扩展为多阶段任务编排       |

此方案完全满足你“保留异步机制并通过事件通知主流程”的需求，是ROS 2中推荐的异步协作模式。

### 方案深入解读

#### QNA#01 编程范式

针对上述的『可行性方案一分析』和『可行性方案二分析』解释如下几点

> - “ 在类私有成员中添加共享状态与同步机制”：什么是“共享状态”，“同步机制”
>
> - “修改 trigger_left_arm_task 为异步监听模式”： 什么是“异步监听模式”？ 为何采用“异步”而不是“同步”
> - ROS2 开发中常用的阻塞模式有哪几种

##### 共享状态与同步机制

**共享状态 (Shared State)**

共享状态是指在多个执行线程或异步操作之间需要共享和访问的数据。在我们的场景中，共享状态是用于存储左臂动作执行结果的数据结构：

```cpp
struct LeftArmResult {
    std::atomic<bool> completed{false};  // 标记任务是否完成
    bool success{false};                 // 标记任务是否成功
    std::string message;                 // 任务结果消息
};
```

这个结构体被多个线程访问：

- 一个线程是右臂的`execute_grasp_box`函数，它需要读取左臂任务的结果
- 另一个线程是ROS 2的回调系统，当左臂任务完成时，会调用结果回调函数更新这个状态

**同步机制 (Synchronization Mechanism)**

同步机制是用来协调多个线程对共享资源的访问，防止出现竞态条件（race condition）的技术。在我们的实现中，使用了以下几种同步机制：

1. **互斥锁 (Mutex)**: `std::mutex`用于保护对共享状态的访问，确保同一时间只有一个线程能修改或读取状态
2. **条件变量 (Condition Variable)**: `std::condition_variable`用于线程间通信，允许一个线程等待某个条件满足（如左臂任务完成）
3. **原子变量 (Atomic Variable)**: `std::atomic<bool>`提供了一种无需锁的线程安全布尔操作方式

```cpp
LeftArmResult left_arm_result_;       // 共享状态
std::mutex result_mutex_;             // 互斥锁
std::condition_variable result_cv_;   // 条件变量
```

##### 异步监听模式

**什么是异步监听模式**

异步监听模式是一种编程范式，其中程序发起一个操作后不会阻塞等待结果，而是继续执行其他任务。当操作完成时，系统会通过回调函数或其他机制通知程序结果已经准备好。

在我们的实现中：

1. 右臂节点通过`trigger_left_arm_client_->async_send_goal()`发送目标给左臂，立即返回
2. 右臂继续执行其他操作（如移动机械臂、激活吸盘等）
3. 当左臂完成任务时，ROS 2系统会自动调用注册的结果回调函数
4. 回调函数更新共享状态并通知等待的线程

**为何采用异步而非同步**

1. **提高资源利用率**: 异步模式允许右臂在等待左臂完成任务的同时执行其他操作，而不是空闲等待
2. **更好的用户体验**: 系统响应更快，不会因为等待某个操作而完全冻结
3. **避免死锁**: 在复杂的多线程环境中，同步等待可能导致死锁问题
4. **符合ROS 2设计理念**: ROS 2是基于事件驱动的框架，异步模式更符合其设计原则

##### ROS 2开发中常用的阻塞模式

在ROS 2开发中，有几种常见的阻塞模式：

3.1 同步服务调用

使用`send_request()`发送服务请求并等待响应：

```cpp
auto future = client->async_send_request(request);
auto response = future.get(); // 阻塞等待结果
```

**3.2 同步动作目标发送**

使用动作客户端发送目标并等待结果：

```cpp
auto goal_handle = client->async_send_goal(goal).get(); // 等待目标被接受
auto result = client->async_get_result(goal_handle).get(); // 等待结果
```

**3.3 定时器阻塞**

使用`rclcpp::Rate`或`std::this_thread::sleep_for`进行周期性阻塞：

```cpp
rclcpp::Rate rate(10); // 10Hz
while (rclcpp::ok()) {
    // 执行任务
    rate.sleep(); // 阻塞直到下一个周期
}
```

3.4 条件变量等待

使用条件变量等待特定条件满足：

```cpp
std::unique_lock<std::mutex> lock(mutex);
condition_variable.wait(lock, []{ return condition_met; });
```

**3.5 等待动作服务器**

等待动作服务器变为可用状态：

```cpp
if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    // 超时处理
}
```

这些阻塞模式在不同场景下有各自的应用价值，但在高并发、实时性要求高的机器人系统中，通常更推荐使用异步模式以提高系统整体效率和响应性

#### QNA#02 技术方案可行性评估

##### 问题描述

> 根据上述说明内容：
>
> - 在类私有成员中添加共享状态与同步机制
> - 这个结构体被多个线程访问：
>   - 一个线程是右臂的 execute_grasp_box 函数，它需要读取左臂任务的结果
>   - 另一个线程是 ROS 2 的回调系统，当左臂任务完成时，会调用结果回调函数更新这个状态
>
> 是否可以不采用“在类私有成员中添加共享状态与同步机制“的方案；而是利用ROS 2的回调系统，当左臂任务完成时，自动更新并返回结果给右臂，直接在右臂的 execute_grasp_box 函数，读取左臂任务的结果。具体变更点为：
>
> - 在右臂的 execute_grasp_box 函数添加获取左臂任务（吸取目标物体）的结果代码
> - 在左臂的 execute_trigger_left_arm 函数中添加发布任务执行（吸取目标物体）的结果代码

##### 问题分析

1. **ROS 2 动作机制的限制**：
   - ROS 2 的动作通信是异步的，右臂通过 `trigger_left_arm_client_` 发送目标给左臂后，不能直接在 `execute_grasp_box` 函数中"读取"左臂的结果
   - 动作的结果只能通过回调函数接收，无法直接同步读取
2. **线程模型问题**：
   - `execute_grasp_box` 函数在一个独立线程中运行
   - 动作结果回调在 ROS 2 的执行线程中运行
   - 这两个线程是独立的，无法直接共享局部变量
3. **执行流程问题**：
   - `execute_grasp_box` 需要在某个点等待左臂任务完成才能决定最终结果
   - 如果不使用某种同步机制，右臂会直接完成任务，无法知道左臂是否成功

##### 解决方案

我们可以改进你的想法，通过在右臂中使用类成员变量存储左臂任务结果，并使用条件变量进行同步。这样既利用了 ROS 2 的回调机制，又能满足同步需求。

**『右臂代码实现』**

1. 在类私有成员中添加结果存储和同步机制

```cpp
// rm_sclerp_planner_node_leftArm.cpp

// ... existing code ...

private:
    // 💌 添加左臂任务结果存储（新增）
    struct LeftArmTaskResult {
        bool completed{false};
        bool success{false};
        std::string message{""};
    };
    
    LeftArmTaskResult left_arm_task_result_;
    std::mutex left_arm_result_mutex_;
    std::condition_variable left_arm_result_cv_;

// ... existing code ...
```

2. 修改 `trigger_left_arm_task` 函数，添加结果回调

```cpp

// 触发左臂任务的函数
void trigger_left_arm_task(const std::string& task_description) 
{
    if (!trigger_left_arm_client_->wait_for_action_server(std::chrono::seconds(5))) 
    {
        RCLCPP_ERROR(this->get_logger(), "左臂触发动作服务不可用");
        
        // 更新结果状态
        {
            std::lock_guard<std::mutex> lock(left_arm_result_mutex_);
            left_arm_task_result_.completed = true;
            left_arm_task_result_.success = false;
            left_arm_task_result_.message = "Left arm action server not available";
        }
        left_arm_result_cv_.notify_one();
        return;
    }

    // 创建目标
    auto goal_msg = TriggerArm::Goal();
    goal_msg.task_description = task_description;
    
    // 设置回调函数
    auto send_goal_options = rclcpp_action::Client<TriggerArm>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
        std::bind(&RmSclerpPlannerNode1::trigger_goal_response_callback, this, _1);
    send_goal_options.feedback_callback = 
        std::bind(&RmSclerpPlannerNode1::trigger_feedback_callback, this, _1, _2);
    
    // 💌 添加结果回调（新增部分）
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<TriggerArm>::WrappedResult & result) {
        std::lock_guard<std::mutex> lock(left_arm_result_mutex_);
        left_arm_task_result_.completed = true;
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                left_arm_task_result_.success = true;
                left_arm_task_result_.message = "Left arm succeeded: " + result.result->message;
                RCLCPP_INFO(this->get_logger(), "左臂任务成功: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                left_arm_task_result_.success = false;
                left_arm_task_result_.message = "Left arm aborted: " + result.result->message;
                RCLCPP_ERROR(this->get_logger(), "左臂任务中止: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                left_arm_task_result_.success = false;
                left_arm_task_result_.message = "Left arm canceled: " + result.result->message;
                RCLCPP_WARN(this->get_logger(), "左臂任务取消: %s", result.result->message.c_str());
                break;
            default:
                left_arm_task_result_.success = false;
                left_arm_task_result_.message = "Left arm unknown result";
                RCLCPP_ERROR(this->get_logger(), "左臂任务未知结果");
                break;
        }
        left_arm_result_cv_.notify_one();
    };
        
    // 发送目标
    trigger_left_arm_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "已发送触发左臂任务请求: %s", task_description.c_str());
}
```

3. 修改 `execute_grasp_box` 函数，等待并处理左臂任务结果

```cpp
// Grasp box 实际执行函数
void execute_grasp_box(const std::shared_ptr<GraspBoxHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing grasp box action");
    rclcpp::Rate loop_rate(1);    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GraspBox::Feedback>();
    auto result = std::make_shared<GraspBox::Result>();

    // 重置左臂任务结果状态
    {
        std::lock_guard<std::mutex> lock(left_arm_result_mutex_);
        left_arm_task_result_.completed = false;
        left_arm_task_result_.success = false;
        left_arm_task_result_.message = "";
    }

    // ... existing initialization and trajectory planning code ...

    // 激活吸盘
    RCLCPP_INFO(this->get_logger(), "%s 激活吸盘服务", LAUNCH_ICON.c_str());
    std::string jodell_activate_cmd = "ros2 service call /right_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \""
                        "{}\"";
    call_service_sync(jodell_activate_cmd);   
    // 延时
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

    // 用于避障
    rm_current_arm_state_t rm_robot_current_state;
    rm_get_arm_state(rm_robot_current_state);
    std::vector<float> gripper_tcp_pose(6, 0.0f);
    gripper_tcp_pose = {
        rm_robot_current_state.pose.position.x + 0.15f,
        rm_robot_current_state.pose.position.y,
        rm_robot_current_state.pose.position.z,
        rm_robot_current_state.pose.euler.rx,
        rm_robot_current_state.pose.euler.ry,
        rm_robot_current_state.pose.euler.rz
    };
    rm_movel_cmd(gripper_tcp_pose, 40);
    
    // 换手场景
    bool triggerArm = true;
    if(triggerArm)
    {
        // 运动到交接位置
        std::vector<double> target_joint_position = {-84.578, 97.793, 19.109, -47.483, -24.050, 127.871, -99.886};
        rmRobotAPI_->rm_movej(target_joint_position, 50);              
        rm_get_arm_state(rm_robot_current_state);            
        gripper_tcp_pose = {rm_robot_current_state.pose.position.x ,
                            rm_robot_current_state.pose.position.y,
                            rm_robot_current_state.pose.position.z - 0.115f,
                            rm_robot_current_state.pose.euler.rx,
                            rm_robot_current_state.pose.euler.ry,
                            rm_robot_current_state.pose.euler.rz
                    };
        rm_movel_cmd(gripper_tcp_pose, 50); 
        RCLCPP_INFO(this->get_logger(), "%s 右臂到达交付点", COMPLETE_ICON.c_str());            
        // 触发左臂动作
        RCLCPP_INFO(this->get_logger(), "%s 触发左臂动作服务", REQUEST_ICON.c_str());  
        trigger_left_arm_task("开始执行抓取、扫码、放置任务");   
    }

    // 💌 等待左臂任务完成并获取结果（新增）
    bool left_arm_success = false;
    std::string left_arm_message = "";
    {
        std::unique_lock<std::mutex> lock(left_arm_result_mutex_);
        if (!left_arm_result_cv_.wait_for(lock, std::chrono::seconds(30), [this] { 
                return left_arm_task_result_.completed; 
            })) {
            // 超时处理
            RCLCPP_ERROR(this->get_logger(), "等待左臂任务结果超时");
            left_arm_success = false;
            left_arm_message = "Timeout waiting for left arm task result";
        } else {
            left_arm_success = left_arm_task_result_.success;
            left_arm_message = left_arm_task_result_.message;
        }
    }

    // 根据左臂任务结果决定后续操作
    if (!left_arm_success) {
        // 左臂任务失败，关闭右臂吸盘并报告失败
        std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
        call_service_sync(jodell_deactivate_cmd);
        
        result->success = false;
        result->message = "Left arm task failed: " + left_arm_message;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "因左臂任务失败，终止抓取任务: %s", left_arm_message.c_str());
        return;
    }

    // 左臂任务成功，继续执行
    // ⚠ 🚥 延时用于等待左臂吸取到纸箱（可注释或删除）
    // std::this_thread::sleep_for(std::chrono::milliseconds(4000)); 
    // 关闭吸盘
    // std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
   // call_service_sync(jodell_deactivate_cmd);

    // 机械臂复位到初始位置
    std::vector<double> initial_joint_position = {-44.811, 69.029, -48.806, 71.437, -40.008, 83.439, -62.740};
    rmRobotAPI_->rm_movej(initial_joint_position, 50);

    // 输出最终结果
    if(rclcpp::ok()){
        result->success = true;
        result->message = "Grasp box action and left arm task both completed successfully";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "抓取任务成功完成");
    }
}
```

##### 方案优势

1. **充分利用ROS 2回调机制**：通过[result_callback](javascript:void(0))处理左臂任务结果
2. **非阻塞等待**：右臂在执行其他操作的同时等待左臂结果
3. **超时处理**：避免无限期等待
4. **线程安全**：使用互斥锁和条件变量确保线程安全

##### 总结

虽然不能直接"读取"左臂任务结果，但可以通过回调机制更新共享状态，并在需要时等待该状态变化。这种方式既满足了异步通信的要求，又实现了任务间的同步协调。



#### QNA#03：区分ROS2 动作中的状态反馈与结果返回

##### 需求

> 方案二给出了右臂节点的修改方案与代码实现，如何修改左臂实现动作执行的结果的返回，即在左臂节点的 execute_trigger_left_arm 函数中的吸盘延迟语句后返回执行结果？
>
> 期望的修改后的局部代码实现如下：
>
> ```
> // 延时一段时间让吸盘工作
> std::this_thread::sleep_for(std::chrono::milliseconds
> (2000));
> 
> // 发布吸盘吸取结果
> // ... 结果发布代码 ...
> // 用于避障
> // ...其他代码...
> ```



##### 代码实现

旧版本的左臂吸取物体执行函数

```c++
// 实际执行函数 - 接收到右臂触发信号后执行左臂任务
void execute_trigger_left_arm(
    const std::shared_ptr<TriggerArmHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TriggerArm::Feedback>();
    auto result = std::make_shared<TriggerArm::Result>();
    
    RCLCPP_INFO(this->get_logger(), "开始执行左臂任务: %s", goal->task_description.c_str());
    
    // 发布反馈 - 开始执行
    feedback->status = "开始执行左臂抓取任务";
    goal_handle->publish_feedback(feedback);
    
    // 执行左臂任务（吸取物体、扫码、放置等）
    bool success = execute_left_arm_task();
    
    if (success) {
        // 发布反馈 - 任务完成
        feedback->status = "左臂任务完成";
        goal_handle->publish_feedback(feedback);
        
        // 设置结果
        result->success = true;
        result->message = "左臂任务成功完成";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "左臂任务成功完成");
    } else {
        result->success = false;
        result->message = "左臂任务执行失败";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "左臂任务执行失败");
    }
}
```

在`execute_left_arm_task`函数中，我们需要修改吸盘工作后的代码：

```c++
// 左臂任务执行函数 
bool execute_left_arm_task() {
    // 实现左臂的吸取物体、扫码、放置等任务
    tempForShow();
    RCLCPP_INFO(this->get_logger(), "执行左臂抓取、扫码、放置任务");
    
    // 这里应该返回实际的任务执行结果
    // 目前简化为总是返回true
    return true; // 简化示例，实际应根据任务执行情况返回
}
```

###### 

在ROS 2动作框架中，结果是通过`goal_handle`来返回的。左臂作为动作服务端，需要通过`goal_handle->succeed(result)`、`goal_handle->abort(result)`或`goal_handle->canceled(result)`来返回执行结果。

1. **动作结果返回**：在`execute_trigger_left_arm`函数中，通过`goal_handle->succeed(result)`或`goal_handle->abort(result)`返回执行结果。
2. **结果内容**：结果包含`success`布尔值和`message`字符串，符合ROS 2动作接口规范。

##### 存在问题

上述第一版代码实现中，反馈的是动作执行的结果，而不是执行过程的状态。

> 想在左臂的`execute_trigger_left_arm`函数中，在吸盘工作延时后添加反馈信息，以便右臂可以通过回调函数获取左臂任务的执行状态。要发布的反馈信息应该是与吸盘工作状态相关的，而不是"规划完成"。

期望新增的功能是左臂到达物体接收点位后开启吸盘吸附物体（在 tempForShow 函数中实现），根据吸盘状态，返回吸取结果，其吸盘的返回结果属于左臂动作执行过程中的状态信息，而不是任务完成的最终结果。

> 在左臂的`execute_trigger_left_arm`函数中，在吸盘工作延时后添加反馈信息，以便右臂可以通过回调函数获取左臂任务的执行状态，进而作决策。

左臂执行吸取和放置纸箱动作任务过程中，增加**中间状态反馈**，右臂根据左臂的状态反馈决策自身动作。

在ROS 2动作框架中，结果不是在执行过程中"发布"的，而是在任务完成时通过 goal handle 返回的。

如果是想在 `execute_trigger_left_arm` 函数中添加结果返回，那么应该是在整个任务完成时通过 goal handle返回结果，而不是在**中间过程**中发布。在**动作模式**下，结果只能通过 `goal handle` 的`succeed()`、`abort()`或`canceled()`方法返回。

##### 代码改进

如果您的意思是在 [tempForShow](javascript:void(0)) 函数中添加状态信息输出，那么下面的代码是合适的。

```c++
// 延时一段时间让吸盘工作
std::this_thread::sleep_for(std::chrono::milliseconds(2000));

// 发布吸盘吸取结果
RCLCPP_INFO(this->get_logger(), "✅ 吸盘吸取任务完成");
// 可以在这里添加检查吸盘状态的代码，例如：
// bool suction_ok = checkSuctionStatus();
// if (!suction_ok) {
//     RCLCPP_WARN(this->get_logger(), "⚠️ 吸盘未能成功吸取物体");
// }

// 用于避障
```

如果需要在动作执行过程中提供反馈，应该使用`goal_handle->publish_feedback(feedback)`方法，而不是发布结果。

**结果只能在任务完全结束时返回一次。**

根据您的需求，正确的代码应该如下：

```cpp
// 延时一段时间让吸盘工作
std::this_thread::sleep_for(std::chrono::milliseconds(2000));

// 发布吸盘吸取结果反馈
auto feedback = std::make_shared<TriggerArm::Feedback>();
feedback->status = "吸盘吸取物体成功"; // 或者使用数字状态码，如 2 表示吸取成功
goal_handle->publish_feedback(feedback);
RCLCPP_INFO(this->get_logger(), "✅ 吸取物体成功");

// 用于避障
```

这样修改后，右臂节点通过`trigger_feedback_callback`函数就能接收到左臂发送的反馈信息，可以更好地了解左臂任务的执行状态。

不过需要注意的是，这种反馈只是通知右臂任务进行到了哪一步，并不是最终的结果。最终的结果还是要通过`goal_handle->succeed(result)`等方式在任务完全结束时返回。

##### 其他变更点

方案二中实现的“在类私有成员中添加结果存储和同步机制” 部分的代码应该修改为表示任务执行过程的状态

右臂的 trigger_left_arm_task 函数和 execute_grasp_box 也要进行相应的修改

修改说明

1. **修改共享状态结构**：将原来的简单结果存储改为支持过程状态的结构
2. **修改左臂节点**：在适当位置添加过程状态反馈
3. **修改右臂节点**：更新回调函数以处理过程状态，并在主流程中使用这些状态

代码实现

###### 1. 修改右臂节点类私有成员中的共享状态结构

```cpp
// 替换原有的 LeftArmTaskResult 结构
struct LeftArmTaskState {
    bool completed{false};           // 任务是否完成
    bool success{false};             // 最终结果
    std::string message{""};         // 最终消息
    std::string current_status{""};  // 当前执行状态
    std::mutex mutex;                // 保护结构的互斥锁
};

LeftArmTaskState left_arm_task_state_;
std::condition_variable left_arm_state_cv_;
```

###### 2. 修改右臂的 trigger_left_arm_task 函数

```cpp
// 触发左臂任务的函数
void trigger_left_arm_task(const std::string& task_description) 
{
    if (!trigger_left_arm_client_->wait_for_action_server(std::chrono::seconds(5))) 
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 左臂触发动作服务不可用");
        
        // 更新状态
        {
            std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
            left_arm_task_state_.completed = true;
            left_arm_task_state_.success = false;
            left_arm_task_state_.message = "Left arm action server not available";
            left_arm_task_state_.current_status = "Failed to connect";
        }
        left_arm_state_cv_.notify_one();
        return;
    }

    // 创建目标
    auto goal_msg = TriggerArm::Goal();
    goal_msg.task_description = task_description;
    
    // 设置回调函数
    auto send_goal_options = rclcpp_action::Client<TriggerArm>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
        std::bind(&RmSclerpPlannerNode1::trigger_goal_response_callback, this, _1);
    
    // 添加反馈回调
    send_goal_options.feedback_callback = [this](
        typename rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr goal_handle,
        const std::shared_ptr<const TriggerArm::Feedback> feedback) {
        
        // 更新过程状态
        std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
        left_arm_task_state_.current_status = feedback->status;
        RCLCPP_INFO(this->get_logger(), "🔄 收到左臂反馈: %s", feedback->status.c_str());
    };
    
    // 添加结果回调
    send_goal_options.result_callback = [this](
        const rclcpp_action::ClientGoalHandle<TriggerArm>::WrappedResult & result) {
        
        std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
        left_arm_task_state_.completed = true;
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                left_arm_task_state_.success = true;
                left_arm_task_state_.message = "Left arm succeeded: " + result.result->message;
                RCLCPP_INFO(this->get_logger(), "✅ 左臂任务成功: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                left_arm_task_state_.success = false;
                left_arm_task_state_.message = "Left arm aborted: " + result.result->message;
                RCLCPP_ERROR(this->get_logger(), "❌ 左臂任务中止: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                left_arm_task_state_.success = false;
                left_arm_task_state_.message = "Left arm canceled: " + result.result->message;
                RCLCPP_WARN(this->get_logger(), "⚠️ 左臂任务取消: %s", result.result->message.c_str());
                break;
            default:
                left_arm_task_state_.success = false;
                left_arm_task_state_.message = "Left arm unknown result";
                RCLCPP_ERROR(this->get_logger(), "❌ 左臂任务未知结果");
                break;
        }
        left_arm_state_cv_.notify_one();
    };
        
    // 发送目标
    trigger_left_arm_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "📨 已发送触发左臂任务请求: %s", task_description.c_str());
}
```

『代码说明』

为什么状态更新代码放在服务可用性判断中？

这是为了处理连接失败的情况。在这种情况下：

1. **错误处理的完整性**：即使无法连接到左臂动作服务器，我们也需要确保右臂主流程能够继续执行，而不是无限等待一个永远不会到来的结果。
2. **状态一致性**：通过将连接失败的状态写入共享状态结构，右臂主流程可以通过统一的等待机制（使用条件变量）来处理这种错误情况。
3. **避免阻塞**：通过设置`completed = true`并调用`notify_one()`，可以立即唤醒正在等待结果的右臂主流程，让它知道任务已经"完成"（虽然是以失败的方式）。
4. **统一的错误处理流程**：无论任务是成功、失败还是连接失败，右臂主流程都可以通过相同的机制来处理结果。

###### 3. 修改右臂的 execute_grasp_box 函数

『代码实现』

```cpp
// Grasp box 实际执行函数
void execute_grasp_box(const std::shared_ptr<GraspBoxHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "🔄 执行抓取盒子动作");
    rclcpp::Rate loop_rate(1);    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GraspBox::Feedback>();
    auto result = std::make_shared<GraspBox::Result>();

    // 重置左臂任务状态
    {
        std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
        left_arm_task_state_.completed = false;
        left_arm_task_state_.success = false;
        left_arm_task_state_.message = "";
        left_arm_task_state_.current_status = "Initializing";
    }

    // ... existing initialization and trajectory planning code ...

    // 激活吸盘
    RCLCPP_INFO(this->get_logger(), "✅ 激活吸盘服务");
    std::string jodell_activate_cmd = "ros2 service call /right_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \"{}\"";
    call_service_sync(jodell_activate_cmd);   
    // 延时
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

    // 用于避障
    rm_current_arm_state_t rm_robot_current_state;
    rm_get_arm_state(rm_robot_current_state);
    std::vector<float> gripper_tcp_pose(6, 0.0f);
    gripper_tcp_pose = {
        rm_robot_current_state.pose.position.x + 0.15f,
        rm_robot_current_state.pose.position.y,
        rm_robot_current_state.pose.position.z,
        rm_robot_current_state.pose.euler.rx,
        rm_robot_current_state.pose.euler.ry,
        rm_robot_current_state.pose.euler.rz
    };
    rm_movel_cmd(gripper_tcp_pose, 40);
    
    // 换手场景
    bool triggerArm = true;
    if(triggerArm)
    {
        // 运动到交接位置
        std::vector<double> target_joint_position = {-84.578, 97.793, 19.109, -47.483, -24.050, 127.871, -99.886};
        rmRobotAPI_->rm_movej(target_joint_position, 50);              
        rm_get_arm_state(rm_robot_current_state);            
        gripper_tcp_pose = {rm_robot_current_state.pose.position.x ,
                            rm_robot_current_state.pose.position.y,
                            rm_robot_current_state.pose.position.z - 0.115f,
                            rm_robot_current_state.pose.euler.rx,
                            rm_robot_current_state.pose.euler.ry,
                            rm_robot_current_state.pose.euler.rz
                    };
        rm_movel_cmd(gripper_tcp_pose, 50); 
        RCLCPP_INFO(this->get_logger(), "🏁 右臂到达交付点");            
        // 触发左臂动作
        RCLCPP_INFO(this->get_logger(), "📨 触发左臂动作服务");  
        trigger_left_arm_task("开始执行抓取、扫码、放置任务");   
    }

    // 等待并处理左臂任务过程状态和最终结果
    bool left_arm_success = false;
    std::string left_arm_message = "";
    
    // 先等待过程状态更新（例如吸盘吸取成功）
    {
        std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
        if (!left_arm_state_cv_.wait_for(lock, std::chrono::seconds(15), [this] { 
                return left_arm_task_state_.current_status == "吸盘吸取物体成功" || 
                       left_arm_task_state_.completed; // 如果任务已完成也退出等待
            })) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 等待左臂过程状态超时");
        } else {
            RCLCPP_INFO(this->get_logger(), "🔄 左臂当前状态: %s", left_arm_task_state_.current_status.c_str());
            // 可以根据过程状态执行一些操作
            if (left_arm_task_state_.current_status == "吸盘吸取物体成功") {
                RCLCPP_INFO(this->get_logger(), "✅ 检测到左臂吸盘吸取成功");
                // 在这里可以执行一些响应操作
            }
        }
    }
    
    // 继续等待最终结果
    {
        std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
        if (!left_arm_state_cv_.wait_for(lock, std::chrono::seconds(30), [this] { 
                return left_arm_task_state_.completed; 
            })) {
            // 超时处理
            RCLCPP_ERROR(this->get_logger(), "❌ 等待左臂任务结果超时");
            left_arm_success = false;
            left_arm_message = "Timeout waiting for left arm task result";
        } else {
            left_arm_success = left_arm_task_state_.success;
            left_arm_message = left_arm_task_state_.message;
        }
    }

    // 根据左臂任务结果决定后续操作
    if (!left_arm_success) {
        // 左臂任务失败，关闭右臂吸盘并报告失败
        std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
        call_service_sync(jodell_deactivate_cmd);
        
        result->success = false;
        result->message = "Left arm task failed: " + left_arm_message;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "❌ 因左臂任务失败，终止抓取任务: %s", left_arm_message.c_str());
        return;
    }

    // 左臂任务成功，继续执行
    // 延时用于等待左臂吸取到纸箱
    std::this_thread::sleep_for(std::chrono::milliseconds(4000)); 
    // 关闭吸盘
    std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
    call_service_sync(jodell_deactivate_cmd);

    // 机械臂复位到初始位置
    std::vector<double> initial_joint_position = {-44.811, 69.029, -48.806, 71.437, -40.008, 83.439, -62.740};
    rmRobotAPI_->rm_movej(initial_joint_position, 50);

    // 输出最终结果
    if(rclcpp::ok()){
        result->success = true;
        result->message = "Grasp box action and left arm task both completed successfully";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "🏁 抓取任务成功完成");
    }
}
```

**『代码说明』**

"先等待过程状态更新"部分：

```cpp
// 先等待过程状态更新（例如吸盘吸取成功）
{
    std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
    if (!left_arm_state_cv_.wait_for(lock, std::chrono::seconds(15), [this] { 
            return left_arm_task_state_.current_status == "吸盘吸取物体成功" || 
                   left_arm_task_state_.completed; // 如果任务已完成也退出等待
        })) {
        RCLCPP_WARN(this->get_logger(), "⚠️ 等待左臂过程状态超时");
    } else {
        RCLCPP_INFO(this->get_logger(), "🔄 左臂当前状态: %s", left_arm_task_state_.current_status.c_str());
        // 可以根据过程状态执行一些操作
        if (left_arm_task_state_.current_status == "吸盘吸取物体成功") {
            RCLCPP_INFO(this->get_logger(), "✅ 检测到左臂吸盘吸取成功");
            // 在这里可以执行一些响应操作
        }
    }
}
```

**功能解释：**

- 这部分代码专门等待左臂发送的中间过程状态，例如"吸盘吸取物体成功"
- 它允许右臂在左臂完成特定步骤后立即做出反应，而不需要等待整个任务完成
- 使用15秒超时，避免无限等待
- 等待条件是`current_status == "吸盘吸取物体成功"`或任务已完成（成功或失败）

**语法解释：**

- `std::unique_lock<std::mutex>`：比`lock_guard`更灵活的锁，支持解锁和重新加锁
- `left_arm_state_cv_.wait_for()`：带超时的条件等待，避免无限阻塞
- Lambda表达式`[this] { ... }`：等待条件谓词，返回true时停止等待
- `return left_arm_task_state_.current_status == "吸盘吸取物体成功" || left_arm_task_state_.completed`：等待条件，满足任一条件即停止等待

"继续等待最终结果"部分：

```cpp
// 继续等待最终结果
{
    std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
    if (!left_arm_state_cv_.wait_for(lock, std::chrono::seconds(30), [this] { 
            return left_arm_task_state_.completed; 
        })) {
        // 超时处理
        RCLCPP_ERROR(this->get_logger(), "❌ 等待左臂任务结果超时");
        left_arm_success = false;
        left_arm_message = "Timeout waiting for left arm task result";
    } else {
        left_arm_success = left_arm_task_state_.success;
        left_arm_message = left_arm_task_state_.message;
    }
}
```

**功能解释：**

- 这部分代码等待左臂任务的最终结果（成功或失败）
- 使用更长的超时时间（30秒），因为完整任务可能需要更长时间
- 一旦任务完成（completed=true），就获取最终的成功状态和消息

**语法解释：**

- 与过程状态等待类似，但等待条件更简单：只检查`completed`标志
- 超时时间更长（30秒），因为这是等待整个任务完成
- 超时或正常完成后，将结果存储在局部变量中供后续使用

总结

这种两阶段等待的设计允许右臂：

- 在关键中间步骤完成后立即做出反应（如确认左臂已成功吸取物体）
- 等待并处理最终任务结果
- 通过合理的超时机制避免无限等待
- 通过统一的状态管理机制处理各种情况（成功、失败、超时等）

###### 4. 在左臂的 execute_trigger_left_arm 函数中添加过程状态反馈

```cpp
// 实际执行函数 - 接收到右臂触发信号后执行左臂任务
void execute_trigger_left_arm(
    const std::shared_ptr<TriggerArmHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TriggerArm::Feedback>();
    auto result = std::make_shared<TriggerArm::Result>();
    
    RCLCPP_INFO(this->get_logger(), "🔄 开始执行左臂任务: %s", goal->task_description.c_str());
    
    // 发布反馈 - 开始执行
    feedback->status = "开始执行左臂抓取任务";
    goal_handle->publish_feedback(feedback);
    
    // 执行左臂任务（吸取物体、扫码、放置等）
    // 发布反馈 - 正在执行
    feedback->status = "正在执行左臂任务";
    goal_handle->publish_feedback(feedback);
    
    bool success = execute_left_arm_task();
    
    if (success) {
        // 发布反馈 - 任务完成
        feedback->status = "左臂任务完成";
        goal_handle->publish_feedback(feedback);
        
        // 设置结果
        result->success = true;
        result->message = "左臂任务成功完成";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "✅ 左臂任务成功完成");
    } else {
        result->success = false;
        result->message = "左臂任务执行失败";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "❌ 左臂任务执行失败");
    }
}
```

###### 5. 在左臂的 tempForShow 函数中添加过程反馈

```cpp
void RmSclerpPlannerNode1::tempForShow() 
{
    // ... existing code ...

    // 启动吸盘
    auto suck_start_time = std::chrono::high_resolution_clock::now();
    std::string jodell_activate_cmd = "ros2 service call /left_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \""
                        "{}\"";
    call_service_sync(jodell_activate_cmd);  
    auto suck_end_time = std::chrono::high_resolution_clock::now();
    auto suck_duration = std::chrono::duration<double>(suck_end_time - suck_start_time);
    RCLCPP_INFO(this->get_logger(), "⏱️ 吸盘启动耗时: %.6f 秒", suck_duration.count());

    // 延时一段时间让吸盘工作
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 发布吸盘吸取结果反馈
    // 注意：这里需要一个指向goal_handle的引用才能发布反馈
    // 在实际实现中，您可能需要将goal_handle传递给tempForShow函数
    // 或者在execute_left_arm_task中处理反馈发布
    
    // 用于避障
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x + 0.05f,
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z,
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
        };
    rm_movel_cmd(gripper_tcp_pose, 40);
    
    // ... existing code ...
}
```

###### 问题与解决

上述代码实现中左臂节点中的 execute_trigger_left_arm 函数体实现，采用的是多层嵌套调用的形式（execute_trigger_left_arm -> execute_left_arm_task -> tempForShow)，还需要 execute_left_arm_task 和 tempForShow 函数的参数列表，添加参数`const std::shared_ptr<TriggerArmHandle> goal_handle`, 这样才能在最内层的 [tempForShow](javascript:void(0)) 函数中使用`goal_handle->publish_feedback(feedback)`来发布过程状态反馈。

1. 修改 execute_left_arm_task 函数

```cpp
// 左臂任务执行函数 - 修改参数列表，添加 goal_handle 参数
bool execute_left_arm_task(const std::shared_ptr<TriggerArmHandle> goal_handle) {
    // 实现左臂的吸取物体、扫码、放置等任务
    tempForShow(goal_handle);  // 同时传递 goal_handle 给 tempForShow
    RCLCPP_INFO(this->get_logger(), "执行左臂抓取、扫码、放置任务");
    return true; // 简化示例
}
```

2. 修改 tempForShow 函数

```cpp
// 修改 tempForShow 函数签名，添加 goal_handle 参数
void tempForShow(const std::shared_ptr<TriggerArmHandle> goal_handle) 
{
    // 使用系统命令调用服务替代直接客户端调用
    // RCLCPP_INFO(this->get_logger(), "%s 开始初始化吸盘机械臂",LAUNCH_ICON.c_str());
    // std::string jodell_initialize_cmd = "ros2 service call /left_arm/initialize_gripper rm_ros_interfaces/srv/InitializeGripper \""
    //     "{ip: '192.168.1.18', port: 8080, register_address: 1000, comm_port: 1, device_address: 9}\"";
    // call_service_sync(jodell_initialize_cmd); 
    // RCLCPP_INFO(this->get_logger(), "%s 吸盘初始化完成",COMPLETE_ICON.c_str());

    std::vector<double> target_joint_position = {-67.902, -54.464, -56.450, -49.018, -138.739, 89.207, 73.986};
    rmRobotAPI_->rm_movej(target_joint_position, 50);

    //新的释放点
    // target_joint_position = {-73.217, -64.311, -34.648, -53.269, -149.306, 70.793, 75.306};
    // rmRobotAPI_->rm_movej(target_joint_position, 50);

    rm_current_arm_state_t rm_robot_current_state;
    rm_get_arm_state(rm_robot_current_state);
    std::vector<float> gripper_tcp_pose(6, 0.0f);
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x ,
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z - 0.115f,
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
                };
    RCLCPP_INFO(this->get_logger(), "旋转前位姿欧拉角: [%f, %f, %f]",
                gripper_tcp_pose[3], gripper_tcp_pose[4], gripper_tcp_pose[5]);
    rm_movel_cmd(gripper_tcp_pose, 50);

    // 启动吸盘
    auto suck_start_time = std::chrono::high_resolution_clock::now();
    std::string jodell_activate_cmd = "ros2 service call /left_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \""
                        "{}\"";
    call_service_sync(jodell_activate_cmd);  
    auto suck_end_time = std::chrono::high_resolution_clock::now();
    auto suck_duration = std::chrono::duration<double>(suck_end_time - suck_start_time);
    RCLCPP_INFO(this->get_logger(), "%s 吸盘启动耗时: %.6f 秒", TIMER_ICON.c_str(), suck_duration.count());

    // 延时一段时间让吸盘工作
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 发布吸盘吸取结果反馈
    if (goal_handle) {
        auto feedback = std::make_shared<TriggerArm::Feedback>();
        feedback->status = "吸盘吸取物体成功";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "✅ 吸取物体成功");
    }

    // 用于避障
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x + 0.05f,
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z,
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
        };
    rm_movel_cmd(gripper_tcp_pose, 40);
    // 运动到释放点
    std::vector<double> release_joint_position = {-113.882, -52.867, -33.962, -29.249, -55.023, 126.469, 105.515};
    rmRobotAPI_->rm_movej(release_joint_position, 20);        
    // 关闭吸盘
    std::string jodell_deactivate_cmd = "ros2 service call /left_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \""
                        "{}\"";
    call_service_sync(jodell_deactivate_cmd);
    // 关闭 Modbus（可注释）
    // std::string jodell_CloseModbus_cmd = "ros2 service call /left_arm/close_modbus rm_ros_interfaces/srv/CloseModbus \""
    //                     "{}\"";
    // call_service_sync(jodell_CloseModbus_cmd);        
}
```

3. 修改 execute_trigger_left_arm 函数

```cpp
// 实际执行函数 - 接收到右臂触发信号后执行左臂任务
void execute_trigger_left_arm(
    const std::shared_ptr<TriggerArmHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TriggerArm::Feedback>();
    auto result = std::make_shared<TriggerArm::Result>();
    
    RCLCPP_INFO(this->get_logger(), "开始执行左臂任务: %s", goal->task_description.c_str());
    
    // 发布反馈 - 开始执行
    feedback->status = "开始执行左臂抓取任务";
    goal_handle->publish_feedback(feedback);
    
    // 执行左臂任务（吸取物体、扫码、放置等）
    bool success = execute_left_arm_task(goal_handle);  // 传递 goal_handle 参数
    
    if (success) {
        // 发布反馈 - 任务完成
        feedback->status = "左臂任务完成";
        goal_handle->publish_feedback(feedback);
        
        // 设置结果
        result->success = true;
        result->message = "左臂任务成功完成";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "左臂任务成功完成");
    } else {
        result->success = false;
        result->message = "左臂任务执行失败";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "左臂任务执行失败");
    }
```

为了避免修改[execute_left_arm_task](javascript:void(0))和[tempForShow](javascript:void(0))函数的参数列表，还可以修改左臂节点中的 execute_trigger_left_arm 函数体实现，注释掉 execute_left_arm_task 调用与结果返回代码片段； 同时将 execute_left_arm_task 中的 tempForShow函数代码提取出

```c++
void execute_trigger_left_arm(
    const std::shared_ptr<TriggerArmHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TriggerArm::Feedback>();
    auto result = std::make_shared<TriggerArm::Result>();
    
    RCLCPP_INFO(this->get_logger(), "🔄 开始执行左臂任务: %s", goal-
    
    // 发布反馈 - 开始执行
    feedback->status = "开始执行左臂抓取任务";
    goal_handle->publish_feedback(feedback);
    // 异步非阻塞模式启动吸盘
    call_jodell_service_async(jodell_activate_cmd);
    
    // 方案1
    {    
    // 执行左臂任务（吸取物体、扫码、放置等）
    // bool success = execute_left_arm_task();
    // if (success) {
        // 发布反馈 - 任务完成
    //     feedback->status = "左臂任务完成";
    //     goal_handle->publish_feedback(feedback);
        
        // 设置结果
    //     result->success = true;
    //     result->message = "左臂任务成功完成";
    //     goal_handle->succeed(result);
    //     RCLCPP_INFO(this->get_logger(), "左臂任务成功完成");
    // } else {
    //     result->success = false;
    //     result->message = "左臂任务执行失败";
    //     goal_handle->abort(result);
    //     RCLCPP_ERROR(this->get_logger(), "左臂任务执行失败");
    // }
    }
    
    // 方案2
    {
    // 接收纸箱点位
    // 靠近纸箱
    std::vector<double> target_joint_position = {-67.902, -54.464, -
    rmRobotAPI_->rm_movej(target_joint_position, 50);
    // 接触纸箱，工具坐标系Z 轴方向进给运动
    rm_current_arm_state_t rm_robot_current_state;
    rm_get_arm_state(rm_robot_current_state);
    std::vector<float> gripper_tcp_pose(6, 0.0f);
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x ,
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z - 0.1
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
                };
    RCLCPP_INFO(this->get_logger(), "旋转前位姿欧拉角: [%f, %f, %f]"
                gripper_tcp_pose[3], gripper_tcp_pose[4], gripper_tc
    rm_movel_cmd(gripper_tcp_pose, 50);
    
    feedback->status = "左臂成功接触到纸箱"; // 或者使用数字状态码，
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "✅ 左臂成功接触到纸箱");
    // 延时一段时间让吸盘工作以吸紧物体
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    // TODO：吸盘状态检查，判断吸取是否成功
    // 发布吸盘吸取结果反馈
    feedback->status = "吸盘吸取物体成功"; // 或者使用数字状态码，如
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "✅ 吸盘吸取物体成功");
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // 上升用于避障
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x + 0.1
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z,
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
        };
    rm_movel_cmd(gripper_tcp_pose, 40);
    // 关闭吸盘（异步非阻塞模式）
    call_jodell_service_async(jodell_deactivate_cmd);
    // 运动到释放点
    std::vector<double> release_joint_position = {-113.882, -52.867,
    rmRobotAPI_->rm_movej(release_joint_position, 40);        
    // 关闭 Modbus（可注释🗒）
    // call_jodell_service_sync(jodell_CloseModbus_cmd);  
    // 发布反馈 - 任务完成
    feedback->status = "左臂任务完成";
    goal_handle->publish_feedback(feedback);
    
    // 设置结果
    result->success = true;
    result->message = "左臂任务成功完成";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "✅ 左臂任务成功完成");
    }
}
// 左臂接收纸箱及后续任务动作执行函数 
bool execute_left_arm_task() {
    // 实现左臂的吸取物体、扫码、放置等任务
    // 这里包含原有的左臂任务逻辑
    tempForShow();
    RCLCPP_INFO(this->get_logger(), "执行左臂抓取、扫码、放置任务");
    return true; // 简化示例
}
```



###### 变更总结

通过这些修改，我们实现了：

1. **过程状态跟踪**：不仅跟踪最终结果，还能跟踪任务执行过程中的状态变化
2. **实时反馈**：右臂可以通过反馈回调实时了解左臂任务的执行状态
3. **更精细的控制**：右臂可以根据左臂的过程状态做出相应的操作
4. **更好的错误处理**：通过过程状态和最终结果的结合，可以实现更精确的错误处理

这种设计使双臂协同更加智能和可靠，能够更好地处理复杂的任务流程。



**其他改进建议**

在 [tempForShow](javascript:void(0)) 函数中，吸盘工作延时后可以添加一些状态检查：

- **状态检查**：在 [tempForShow](javascript:void(0)) 函数的吸盘延时后，可以添加实际的吸盘状态检查逻辑，以确定是否成功吸取物体。
- **错误处理**：如果吸盘未能成功吸取物体，应该返回失败结果，这样右臂可以根据结果决定后续操作

```c++
void tempForShow() 
{
    // 使用系统命令调用服务替代直接客户端调用
    // RCLCPP_INFO(this->get_logger(), "%s 开始初始化吸盘机械臂",LAUNCH_ICON.c_str());
    // std::string jodell_initialize_cmd = "ros2 service call /left_arm/initialize_gripper rm_ros_interfaces/srv/InitializeGripper \""
    //     "{ip: '192.168.1.18', port: 8080, register_address: 1000, comm_port: 1, device_address: 9}\"";
    // call_service_sync(jodell_initialize_cmd); 
    // RCLCPP_INFO(this->get_logger(), "%s 吸盘初始化完成",COMPLETE_ICON.c_str());

    std::vector<double> target_joint_position = {-67.902, -54.464, -56.450, -49.018, -138.739, 89.207, 73.986};
    rmRobotAPI_->rm_movej(target_joint_position, 50);

    //新的释放点
    // target_joint_position = {-73.217, -64.311, -34.648, -53.269, -149.306, 70.793, 75.306};
    // rmRobotAPI_->rm_movej(target_joint_position, 50);

    rm_current_arm_state_t rm_robot_current_state;
    rm_get_arm_state(rm_robot_current_state);
    std::vector<float> gripper_tcp_pose(6, 0.0f);
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x ,
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z - 0.115f,
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
                };
    RCLCPP_INFO(this->get_logger(), "旋转前位姿欧拉角: [%f, %f, %f]",
                gripper_tcp_pose[3], gripper_tcp_pose[4], gripper_tcp_pose[5]);
    rm_movel_cmd(gripper_tcp_pose, 50);

    // 启动吸盘
    auto suck_start_time = std::chrono::high_resolution_clock::now();
    std::string jodell_activate_cmd = "ros2 service call /left_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \""
                        "{}\"";
    call_service_sync(jodell_activate_cmd);  
    auto suck_end_time = std::chrono::high_resolution_clock::now();
    auto suck_duration = std::chrono::duration<double>(suck_end_time - suck_start_time);
    RCLCPP_INFO(this->get_logger(), "%s 吸盘启动耗时: %.6f 秒", TIMER_ICON.c_str(), suck_duration.count());

    // 延时一段时间让吸盘工作
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 可以在这里添加吸盘状态检查，以确定是否成功吸取物体
    // 例如检查吸盘压力传感器读数等
    // 如果有相关接口，可以调用API检查吸盘状态
    // bool suction_success = checkSuctionStatus();
    
    // 用于避障
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x + 0.05f,
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z,
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
        };
    rm_movel_cmd(gripper_tcp_pose, 40);
    // 运动到释放点
    std::vector<double> release_joint_position = {-113.882, -52.867, -33.962, -29.249, -55.023, 126.469, 105.515};
    rmRobotAPI_->rm_movej(release_joint_position, 20);        
    // 关闭吸盘
    std::string jodell_deactivate_cmd = "ros2 service call /left_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \""
                        "{}\"";
    call_service_sync(jodell_deactivate_cmd);
    // 关闭 Modbus（可注释）
    // std::string jodell_CloseModbus_cmd = "ros2 service call /left_arm/close_modbus rm_ros_interfaces/srv/CloseModbus \""
    //                     "{}\"";
    // call_service_sync(jodell_CloseModbus_cmd);        
}
```

###### 

#### QNA#04：动作过程状态传递机制

> 分析左臂节点中的 execute_trigger_left_arm 函数中的状态反馈`feedback->status = "吸盘吸取物体成功"`, 是如何传递到右臂节点的 left_arm_task_state_.current_status 中的,详细说明背后的机制和传递流程
>
> ```
> left_arm_task_state_.current_st
> ```

##### 传递机制与流程分析

1. 左臂节点发布反馈

在左臂节点的[tempForShow](javascript:void(0))函数中：

```cpp
// 发布吸盘吸取结果反馈
if (goal_handle) {
    auto feedback = std::make_shared<TriggerArm::Feedback>();
    feedback->status = "吸盘吸取物体成功";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "✅ 吸取物体成功");
}
```

这行代码通过`goal_handle->publish_feedback(feedback)`将反馈信息发布出去。

2. ROS 2动作通信机制

ROS 2动作通信基于以下机制：

1. **分布式通信**：左臂作为动作服务端，右臂作为动作客户端，通过ROS 2的分布式通信机制进行数据交换
2. **话题传输**：动作反馈通过专门的话题进行传输，格式为`/action_name/_action/feedback`
3. **序列化/反序列化**：反馈数据在传输过程中会被自动序列化和反序列化

3. 右臂节点接收反馈

在右臂节点的[trigger_left_arm_task](javascript:void(0))函数中注册了反馈回调：

```cpp
// 添加反馈回调
send_goal_options.feedback_callback = [this](
    typename rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr goal_handle,
    const std::shared_ptr<const TriggerArm::Feedback> feedback) {
    
    // 更新过程状态
    std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
    left_arm_task_state_.current_status = feedback->status;
    RCLCPP_INFO(this->get_logger(), "🔄 收到左臂反馈: %s", feedback->status.c_str());
};
```

4. 完整的传递流程

```
1. 左臂节点: 
   tempForShow() 
   -> feedback->status = "吸盘吸取物体成功"
   -> goal_handle->publish_feedback(feedback)
   
2. ROS 2中间件:
   -> 将反馈数据序列化
   -> 通过网络传输到右臂节点
   -> 在/trigger_arm_action/_action/feedback话题上发布
   
3. 右臂节点:
   -> 接收到反馈消息
   -> 自动反序列化为TriggerArm::Feedback对象
   -> 调用注册的feedback_callback函数
   -> 在回调函数中更新left_arm_task_state_.current_status = feedback->status
```

5. 线程同步机制

为了确保线程安全，使用了互斥锁：

```cpp
std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
left_arm_task_state_.current_status = feedback->status;
```

这确保了在多线程环境下对共享状态的安全访问。

6. 条件变量通知

在结果回调中还会使用条件变量通知等待的线程：

```cpp
// 结果回调中
send_goal_options.result_callback = [this](
    const rclcpp_action::ClientGoalHandle<TriggerArm>::WrappedResult & result) {
    
    std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
    left_arm_task_state_.completed = true;
    // ... 其他状态更新
    left_arm_state_cv_.notify_one();  // 通知等待线程
};
```

##### 关键技术点

1. **异步通信**：整个过程是异步的，左臂发布反馈时不会阻塞，右臂接收反馈也是异步处理
2. **类型安全**：使用了强类型的消息定义，确保发送和接收的数据格式一致
3. **线程安全**：通过互斥锁保护共享状态，通过条件变量实现线程间通信
4. **自动序列化**：ROS 2自动处理数据的序列化和反序列化，开发者无需关心底层细节

#### QNA#05 回调函数重复定义问题

> 在右臂的 trigger_left_arm_task 函数中反馈回调和结果回调函数貌似定义了两次?

右臂的`trigger_left_arm_task`函数中的回调函数定义代码实现中，确实存在重复定义回调函数的问题。

```cpp
// 触发左臂任务的函数
void trigger_left_arm_task(const std::string& task_description) 
{
    if (!trigger_left_arm_client_->wait_for_action_server(std::chrono::seconds(5))) 
    {
        // ... 错误处理代码
        return;
    }

    // 创建目标
    auto goal_msg = TriggerArm::Goal();
    goal_msg.task_description = task_description;
    
    // 设置回调函数
    auto send_goal_options = rclcpp_action::Client<TriggerArm>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
        std::bind(&RmSclerpPlannerNode1::trigger_goal_response_callback, this, _1);
    
    // 添加反馈回调
    send_goal_options.feedback_callback = [this](
        typename rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr goal_handle,
        const std::shared_ptr<const TriggerArm::Feedback> feedback) {
        
        // 更新过程状态
        std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
        left_arm_task_state_.current_status = feedback->status;
        RCLCPP_INFO(this->get_logger(), "🔄 收到左臂反馈: %s", feedback->status.c_str());
    };
    
    // 添加结果回调
    send_goal_options.result_callback = [this](
        const rclcpp_action::ClientGoalHandle<TriggerArm>::WrappedResult & result) {
        
        std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
        left_arm_task_state_.completed = true;
        
        // ... 结果处理代码
        
        left_arm_state_cv_.notify_one();
    };
        
    // 发送目标
    trigger_left_arm_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "📨 已发送触发左臂任务请求: %s", task_description.c_str());
}
```

同时，代码中还保留了之前的回调函数定义：

```cpp
// #region TriggerArm 动作触发函数     
    
void trigger_goal_response_callback(
    typename rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr goal_handle) 
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "触发左臂任务目标被拒绝");
    } else {
        RCLCPP_INFO(this->get_logger(), "触发左臂任务目标已接受");
    }
}

// 反馈回调
void trigger_feedback_callback(
    typename rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr goal_handle,
    const std::shared_ptr<const TriggerArm::Feedback> feedback) 
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "左臂任务反馈: %s", feedback->status.c_str());
}

// 结果回调
void trigger_result_callback(
    const typename rclcpp_action::ClientGoalHandle<TriggerArm>::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "左臂任务成功完成: %s", result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "左臂任务被中止: %s", result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "左臂任务被取消: %s", result.result->message.c_str());
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "左臂任务未知结果");
            break;
    }
}
```

###### 问题分析

确实存在重复定义的问题：

1. **类成员函数定义的回调**：`trigger_goal_response_callback`、`trigger_feedback_callback`、`trigger_result_callback`
2. **Lambda表达式定义的回调**：在`trigger_left_arm_task`函数中使用lambda表达式重新定义了反馈回调和结果回调

###### 解决方案

应该选择其中一种方式，推荐使用Lambda表达式，因为它可以更方便地捕获和更新共享状态。需要删除类成员函数定义的回调，并在`trigger_left_arm_task`中正确设置所有回调：

```cpp
// 触发左臂任务的函数
void trigger_left_arm_task(const std::string& task_description) 
{
    if (!trigger_left_arm_client_->wait_for_action_server(std::chrono::seconds(5))) 
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 左臂触发动作服务不可用");
        
        // 更新状态
        {
            std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
            left_arm_task_state_.completed = true;
            left_arm_task_state_.success = false;
            left_arm_task_state_.message = "Left arm action server not available";
            left_arm_task_state_.current_status = "Failed to connect";
        }
        left_arm_state_cv_.notify_one();
        return;
    }

    // 创建目标
    auto goal_msg = TriggerArm::Goal();
    goal_msg.task_description = task_description;
    
    // 设置回调函数
    auto send_goal_options = rclcpp_action::Client<TriggerArm>::SendGoalOptions();
    
    // 目标响应回调
    send_goal_options.goal_response_callback = [this](
        typename rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "❌ 触发左臂任务目标被拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ 触发左臂任务目标已接受");
        }
    };
    
    // 反馈回调
    send_goal_options.feedback_callback = [this](
        typename rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr goal_handle,
        const std::shared_ptr<const TriggerArm::Feedback> feedback) {
        
        // 更新过程状态
        std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
        left_arm_task_state_.current_status = feedback->status;
        RCLCPP_INFO(this->get_logger(), "🔄 收到左臂反馈: %s", feedback->status.c_str());
    };
    
    // 结果回调
    send_goal_options.result_callback = [this](
        const rclcpp_action::ClientGoalHandle<TriggerArm>::WrappedResult & result) {
        
        std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
        left_arm_task_state_.completed = true;
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                left_arm_task_state_.success = true;
                left_arm_task_state_.message = "Left arm succeeded: " + result.result->message;
                RCLCPP_INFO(this->get_logger(), "✅ 左臂任务成功: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                left_arm_task_state_.success = false;
                left_arm_task_state_.message = "Left arm aborted: " + result.result->message;
                RCLCPP_ERROR(this->get_logger(), "❌ 左臂任务中止: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                left_arm_task_state_.success = false;
                left_arm_task_state_.message = "Left arm canceled: " + result.result->message;
                RCLCPP_WARN(this->get_logger(), "⚠️ 左臂任务取消: %s", result.result->message.c_str());
                break;
            default:
                left_arm_task_state_.success = false;
                left_arm_task_state_.message = "Left arm unknown result";
                RCLCPP_ERROR(this->get_logger(), "❌ 左臂任务未知结果");
                break;
        }
        left_arm_state_cv_.notify_one();
    };
        
    // 发送目标
    trigger_left_arm_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "📨 已发送触发左臂任务请求: %s", task_description.c_str());
}
```

同时，应该删除原来定义的三个回调函数成员函数：

- `trigger_goal_response_callback`
- `trigger_feedback_callback`
- `trigger_result_callback`



### 方案二应用与调试

参用上述方案二，代码实现为 『方案深入解读/QNA#03』中最终实现的代码，存在如下问题

#### Debug01

##### 问题说明

运行双臂节点, 右臂输出为:

```
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760606993.998572017] [right_arm.rm_sclerp_planner_node_rightArm]: 🏁 右臂到达交付点 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760606993.998633584] [right_arm.rm_sclerp_planner_node_rightArm]: 📨 触发左臂动作服务 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760606993.999119726] [right_arm.rm_sclerp_planner_node_rightArm]: 已发送触发左臂任务请求: 开始执行抓取、扫码、放置任务 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760606993.999753258] [right_arm.rm_sclerp_planner_node_rightArm]: 触发左臂任务目标已接受 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760606994.005386826] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 开始执行左臂抓取任务 [rm_sclerp_planner_node_rightArm-1] [WARN] [1760606999.007213125] [right_arm.rm_sclerp_planner_node_rightArm]: ⚠️ 等待左臂过程状态超时 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607000.333876558] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 吸盘吸取物体成功 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607006.121219421] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 左臂任务完成 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607006.121395196] [right_arm.rm_sclerp_planner_node_rightArm]: ✅ 左臂任务成功: 左臂任务成功完成

左臂的输出为: [rm_sclerp_planner_node_leftArm-1] [INFO] [1760607000.333311569] [left_arm.rm_sclerp_planner_node_leftArm]: ✅ 吸取物体成功
```

从上述两个节点的输出来看,左臂节点执行完成了 execute_trigger_left_arm 中的"发布吸盘吸取结果反馈"语句,为何在右臂execute_grasp_box 函数中执行到了 `return left_arm_task_state_.current_status == "吸盘吸取物体成功" `语句?

根据您提供的日志输出，我发现了问题所在。这是一个典型的**竞态条件（race condition）**问题。

##### 问题分析

从日志时间戳可以看出：

1. 右臂在 `1760606999.007213125`（约999秒）时超时，显示"⚠️ 等待左臂过程状态超时"
2. 左臂在 `1760607000.333311569`（约1000秒）时才输出"✅ 吸取物体成功"
3. 右臂在 `1760607000.333876558`（约1000秒）时才收到"🔄 收到左臂反馈: 吸盘吸取物体成功"

这说明右臂的等待超时时间（15秒）不足以等待左臂完成吸盘操作。当右臂的等待超时后，条件变量的等待已经结束，即使之后收到了反馈信息，也不会重新触发等待逻辑。

##### 问题原因

1. **超时时间设置过短**：15秒可能不足以完成吸盘操作和反馈传输
2. **竞态条件**：右臂的等待已经超时退出，即使左臂之后发送了反馈，也不会被处理

##### 解决方案

###### 方案一：增加超时时间

```cpp
// 增加等待超时时间
{
    std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
    if (!left_arm_state_cv_.wait_for(lock, std::chrono::seconds(30), [this] { 
            return left_arm_task_state_.current_status == "吸盘吸取物体成功" || 
                   left_arm_task_state_.completed;
        })) {
        RCLCPP_WARN(this->get_logger(), "⚠️ 等待左臂过程状态超时");
    } else {
        RCLCPP_INFO(this->get_logger(), "🔄 左臂当前状态: %s", left_arm_task_state_.current_status.c_str());
        if (left_arm_task_state_.current_status == "吸盘吸取物体成功") {
            RCLCPP_INFO(this->get_logger(), "✅ 检测到左臂吸盘吸取成功");
        }
    }
}
```

###### 方案二：改进等待逻辑

```cpp
// 改进等待逻辑，即使超时也检查当前状态
{
    std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
    bool wait_result = left_arm_state_cv_.wait_for(lock, std::chrono::seconds(20), [this] { 
            return left_arm_task_state_.current_status == "吸盘吸取物体成功" || 
                   left_arm_task_state_.completed;
        });
    
    if (!wait_result) {
        RCLCPP_WARN(this->get_logger(), "⚠️ 等待左臂过程状态超时，当前状态: %s", 
                   left_arm_task_state_.current_status.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "🔄 左臂当前状态: %s", left_arm_task_state_.current_status.c_str());
        if (left_arm_task_state_.current_status == "吸盘吸取物体成功") {
            RCLCPP_INFO(this->get_logger(), "✅ 检测到左臂吸盘吸取成功");
        }
    }
}
```

###### 方案三：使用循环检查（推荐）

```cpp
// 使用循环检查状态，确保不会错过反馈
{
    std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
    auto wait_until = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    bool status_received = false;
    
    while (std::chrono::steady_clock::now() < wait_until) {
        // 检查当前状态
        if (left_arm_task_state_.current_status == "吸盘吸取物体成功") {
            status_received = true;
            RCLCPP_INFO(this->get_logger(), "✅ 检测到左臂吸盘吸取成功");
            break;
        }
        
        // 检查任务是否已完成
        if (left_arm_task_state_.completed) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 左臂任务已完成，但未收到吸盘成功状态");
            break;
        }
        
        // 等待一小段时间或直到有通知
        left_arm_state_cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    
    if (!status_received && std::chrono::steady_clock::now() >= wait_until) {
        RCLCPP_WARN(this->get_logger(), "⚠️ 等待左臂过程状态超时，当前状态: %s", 
                   left_arm_task_state_.current_status.c_str());
    }
}
```

###### 建议

1. **增加超时时间**：将15秒增加到20-30秒，确保有足够时间完成吸盘操作
2. **优化左臂反馈时机**：确保在吸盘操作完成后立即发送反馈
3. **改进右臂等待逻辑**：即使超时也要检查当前状态，避免错过反馈

这样可以确保右臂能够正确接收到左臂发送的"吸盘吸取物体成功"状态反馈。

#### Debug02

##### 问题描述

增加超时时间,修改代码后,运行双臂节点, 右臂输出为:

```
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760607711.343926365] [right_arm.rm_sclerp_planner_node_rightArm]: 🏁 右臂到达交付点 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607711.343972605] [right_arm.rm_sclerp_planner_node_rightArm]: 📨 触发左臂动作服务 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607711.344579897] [right_arm.rm_sclerp_planner_node_rightArm]: 已发送触发左臂任务请求: 开始执行抓取、扫码、放置任务 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607711.345276789] [right_arm.rm_sclerp_planner_node_rightArm]: 触发左臂任务目标已接受 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607711.345925586] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 开始执行左臂抓取任务 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607717.869018733] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 吸盘吸取物体成功 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607723.514468763] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 左臂任务完成 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607723.514899673] [right_arm.rm_sclerp_planner_node_rightArm]: ✅ 左臂任务成功: 左臂任务成功完成 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607723.515355478] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 左臂当前状态: 左臂任务完成 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760607723.515644725] [right_arm.rm_sclerp_planner_node_rightArm]: Grasp box action succeeded
```

左臂的输出:

```
[rm_sclerp_planner_node_leftArm-1] [INFO] [1760607717.868797710] [left_arm.rm_sclerp_planner_node_leftArm]: ✅ 吸取物体成功
```

基于上述输出信息, 分析为何右臂没有进入"先等待过程状态更新"代码的 else 语句

##### 问题分析

从日志时间戳可以看出：

1. 右臂在 `1760607711.345925586` 收到"开始执行左臂抓取任务"反馈
2. 右臂在 `1760607717.869018733` 收到"吸盘吸取物体成功"反馈
3. 但是在等待过程状态更新的代码中，没有输出"检测到左臂吸盘吸取成功"的信息
4. 最后在等待结束后才输出"左臂当前状态: 左臂任务完成"

##### 原因分析

这个现象说明右臂的等待逻辑存在问题。根据日志分析，最可能的原因是：

1. **等待谓词条件不满足**：当"吸盘吸取物体成功"反馈到达时，右臂可能已经超出了等待过程状态的阶段，进入了等待最终结果的阶段。
2. **条件变量通知时机**：虽然左臂发送了"吸盘吸取物体成功"反馈，但在右臂的等待逻辑中，可能因为条件检查的时机问题，没有正确捕获到这个状态。
3. **状态更新与等待不同步**：共享状态的更新和条件变量的通知可能存在时序问题。

##### 详细解释

从代码逻辑来看：

```cpp
// 先等待过程状态更新（例如吸盘吸取成功）
{
    std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
    if (!left_arm_state_cv_.wait_for(lock, std::chrono::seconds(30), [this] { 
            return left_arm_task_state_.current_status == "吸盘吸取物体成功" || 
                   left_arm_task_state_.completed; // 如果任务已完成也退出等待
        })) {
        RCLCPP_WARN(this->get_logger(), "⚠️ 等待左臂过程状态超时");
    } else {
        RCLCPP_INFO(this->get_logger(), "🔄 左臂当前状态: %s", left_arm_task_state_.current_status.c_str());
        // 可以根据过程状态执行一些操作
        if (left_arm_task_state_.current_status == "吸盘吸取物体成功") {
            RCLCPP_INFO(this->get_logger(), "✅ 检测到左臂吸盘吸取成功");
            // 在这里可以执行一些响应操作
        }
    }
}
```

从日志可以看出，右臂确实收到了"吸盘吸取物体成功"的反馈，但是没有输出"检测到左臂吸盘吸取成功"的信息。这说明：

1. 可能当条件变量被通知时，右臂已经退出了等待过程状态更新的代码块
2. 或者在检查条件时，`left_arm_task_state_.current_status` 还没有被更新为"吸盘吸取物体成功"
3. 或者在检查条件时，`left_arm_task_state_.completed` 已经为 true，导致直接退出等待

##### 解决方案

为了确保能正确捕获过程状态，可以采用更健壮的等待机制：

```cpp
// 改进的等待过程状态更新代码
{
    std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
    auto wait_start = std::chrono::steady_clock::now();
    auto wait_timeout = std::chrono::seconds(30);
    bool status_received = false;
    
    while (std::chrono::steady_clock::now() - wait_start < wait_timeout) {
        // 检查是否收到了期望的过程状态
        if (left_arm_task_state_.current_status == "吸盘吸取物体成功") {
            RCLCPP_INFO(this->get_logger(), "✅ 检测到左臂吸盘吸取成功");
            status_received = true;
            break;
        }
        
        // 如果任务已完成，也退出等待
        if (left_arm_task_state_.completed) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 左臂任务已完成，但未收到吸盘成功状态");
            break;
        }
        
        // 等待一小段时间或直到有通知
        left_arm_state_cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    
    if (!status_received) {
        if (std::chrono::steady_clock::now() - wait_start >= wait_timeout) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 等待左臂过程状态超时，当前状态: %s", 
                       left_arm_task_state_.current_status.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "🔄 左臂当前状态: %s", left_arm_task_state_.current_status.c_str());
        }
    }
}
```

这种实现方式通过主动轮询状态，可以更可靠地捕获过程状态更新，避免因条件变量通知时机问题导致的状态丢失。

#### Debug03

##### 问题描述

按照上述回答修改代码后,运行双臂节点, 右臂输出为:

```
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760609367.257579620] [right_arm.rm_sclerp_planner_node_rightArm]: 🏁 右臂到达交付点 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760609367.257640449] [right_arm.rm_sclerp_planner_node_rightArm]: 📨 触发左臂动作服务 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760609367.258326814] [right_arm.rm_sclerp_planner_node_rightArm]: 已发送触发左臂任务请求: 开始执行抓取、扫码、放置任务 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760609367.259519680] [right_arm.rm_sclerp_planner_node_rightArm]: 触发左臂任务目标已接受 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760609367.259770324] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 开始执行左臂抓取任务 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760609376.690971664] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 吸盘吸取物体成功 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760609382.345509972] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 左臂任务完成 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760609382.345774599] [right_arm.rm_sclerp_planner_node_rightArm]: ✅ 左臂任务成功: 左臂任务成功完成 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760609382.346607068] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 左臂当前状态: 左臂任务完成 [rm_sclerp_planner_node_rightArm-1] [INFO] [1760609382.346958506] [right_arm.rm_sclerp_planner_node_rightArm]: Grasp box action succeeded
```

左臂输出为:

```
[rm_sclerp_planner_node_leftArm-1] [INFO] [1760609376.689736271] [left_arm.rm_sclerp_planner_node_leftArm]: ✅ 吸盘吸取物体成功
```

从输出和机械臂实际作业结果来看, 右臂还是没有进入 if (left_arm_task_state_.current_status == "吸盘吸取物体成功 ") 语句中执行"关闭吸盘" 和 "机械臂复位" 语句,分析原因并解决

根据您提供的日志和之前的分析，问题出在右臂等待过程状态的逻辑上。从日志可以看出，右臂在收到"吸盘吸取物体成功"反馈后，没有进入相应的if语句执行相关操作。让我分析原因并提供解决方案。

##### 问题分析

从日志时间戳可以看出：

1. 右臂在 `1760609376.690971664` 收到"吸盘吸取物体成功"反馈
2. 但在等待过程状态更新的代码中，没有输出相应的处理信息
3. 最终在等待结束后输出的是"左臂当前状态: 左臂任务完成"

这说明右臂虽然接收到了反馈，但在等待逻辑中没有正确处理

##### 原因分析

主要问题在于等待逻辑的设计：

1. 使用了 `wait_for` 配合谓词检查，但谓词检查可能错过了状态更新的时机
2. 当"吸盘吸取物体成功"状态到达时，可能已经超出了等待阶段
3. 存在竞态条件，状态更新和等待检查不同步

##### 解决方案

我们需要重构等待逻辑，采用主动轮询的方式确保不会错过状态更新：

```cpp
// 替换原有的"先等待过程状态更新"和"继续等待最终结果"代码块
{
    std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
    auto wait_start = std::chrono::steady_clock::now();
    auto total_timeout = std::chrono::seconds(35); // 总超时时间
    bool suction_success_handled = false;
    
    RCLCPP_INFO(this->get_logger(), "⏳ 开始等待左臂任务状态更新");
    
    while (std::chrono::steady_clock::now() - wait_start < total_timeout) {
        // 检查是否收到吸盘吸取成功的状态且尚未处理
        if (left_arm_task_state_.current_status == "吸盘吸取物体成功" && !suction_success_handled) {
            RCLCPP_INFO(this->get_logger(), "✅ 检测到左臂吸盘吸取成功，执行相应操作");
            suction_success_handled = true;
            
            // 执行吸盘成功后的操作（关闭右臂吸盘）
            std::this_thread::sleep_for(std::chrono::milliseconds(4000)); // 等待左臂吸取纸箱
            std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
            call_service_sync(jodell_deactivate_cmd);
            RCLCPP_INFO(this->get_logger(), "🔧 右臂吸盘已关闭");
        }
        
        // 检查任务是否完成
        if (left_arm_task_state_.completed) {
            RCLCPP_INFO(this->get_logger(), "🏁 左臂任务已完成，最终状态: %s", left_arm_task_state_.current_status.c_str());
            left_arm_success = left_arm_task_state_.success;
            left_arm_message = left_arm_task_state_.message;
            break;
        }
        
        // 等待通知或短暂超时，避免忙等待
        left_arm_state_cv_.wait_for(lock, std::chrono::milliseconds(50));
    }
    
    // 检查是否超时
    if (!left_arm_task_state_.completed) {
        RCLCPP_WARN(this->get_logger(), "⚠️ 等待左臂任务状态超时");
        left_arm_success = false;
        left_arm_message = "Timeout waiting for left arm task result";
    } else if (!suction_success_handled) {
        RCLCPP_WARN(this->get_logger(), "⚠️ 左臂任务完成但未检测到吸盘成功状态，当前状态: %s", 
                   left_arm_task_state_.current_status.c_str());
        // 即使未明确收到吸盘成功状态，也执行清理操作
        std::this_thread::sleep_for(std::chrono::milliseconds(4000));
        std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
        call_service_sync(jodell_deactivate_cmd);
        RCLCPP_INFO(this->get_logger(), "🔧 右臂吸盘已关闭（默认操作）");
    }
}
```

或者，我们可以采用更简单的方案，将等待逻辑分为两个部分：

```cpp
// 等待并处理左臂任务过程状态和最终结果
bool left_arm_success = false;
std::string left_arm_message = "";
bool 吸盘吸取成功 = false;

// 先等待过程状态更新（例如吸盘吸取成功）
{
    std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
    auto wait_start = std::chrono::steady_clock::now();
    auto wait_timeout = std::chrono::seconds(25);
    
    while (std::chrono::steady_clock::now() - wait_start < wait_timeout) {
        // 检查是否收到吸盘吸取成功的状态
        if (left_arm_task_state_.current_status == "吸盘吸取物体成功") {
            RCLCPP_INFO(this->get_logger(), "✅ 检测到左臂吸盘吸取成功");
            吸盘吸取成功 = true;
            
            // 执行吸盘成功后的操作
            std::this_thread::sleep_for(std::chrono::milliseconds(4000)); // 等待左臂吸取纸箱
            std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
            call_service_sync(jodell_deactivate_cmd);
            break;
        }
        
        // 如果任务已完成，也退出等待
        if (left_arm_task_state_.completed) {
            break;
        }
        
        // 等待通知或超时
        left_arm_state_cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    
    if (!吸盘吸取成功) {
        RCLCPP_WARN(this->get_logger(), "⚠️ 未检测到左臂吸盘吸取成功状态，当前状态: %s", 
                   left_arm_task_state_.current_status.c_str());
    }
}

// 继续等待最终结果
{
    std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
    if (!left_arm_state_cv_.wait_for(lock, std::chrono::seconds(30), [this] { 
            return left_arm_task_state_.completed; 
        })) {
        RCLCPP_ERROR(this->get_logger(), "❌ 等待左臂任务结果超时");
        left_arm_success = false;
        left_arm_message = "Timeout waiting for left arm task result";
    } else {
        left_arm_success = left_arm_task_state_.success;
        left_arm_message = left_arm_task_state_.message;
    }
}
```



这个解决方案的关键改进点：

1. **主动轮询**：使用循环主动检查状态，而不是依赖条件变量的谓词检查
2. **状态标记**：使用`suction_success_handled`标记确保操作只执行一次
3. **超时控制**：设置合理的总超时时间，避免无限等待
4. **兜底处理**：即使未明确收到状态更新，也确保执行必要的清理操作
5. **日志增强**：增加更多日志信息帮助调试

这样修改后，无论"吸盘吸取物体成功"状态何时到达，都能被正确捕获和处理，确保右臂能够执行相应的操作

##### 最终方案

#### Debug04

##### 问题描述

根据任务16 的回答只修改右臂的 execute_grasp_box 函数后（而左臂的动作执行过程的反馈在 execute_trigger_left_arm 函数中，而非tempForShow函数中，请查看最新的左臂节点源文件，而非上述任务回答中给出的代码实现），执行两个节点，输出如下：

右臂：

```
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615598.000553026] [right_arm.rm_sclerp_planner_node_rightArm]: 🏁 右臂到达交付点
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615598.000607649] [right_arm.rm_sclerp_planner_node_rightArm]: 📨 触发左臂动作服务
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615598.001186772] [right_arm.rm_sclerp_planner_node_rightArm]: 已发送触发左臂任务请求: 开始执行抓取、扫码、放置任务
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615598.001222515] [right_arm.rm_sclerp_planner_node_rightArm]: ⏳ 开始等待左臂任务状态更新
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615598.001906819] [right_arm.rm_sclerp_planner_node_rightArm]: ✅ 触发左臂任务目标已接受
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615598.010219111] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 开始执行左臂抓取任务
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615606.167570984] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 吸盘吸取物体成功
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615606.172252702] [right_arm.rm_sclerp_planner_node_rightArm]: ✅ 检测到左臂吸盘吸取成功，执行相应操作
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615606.172386971] [right_arm.rm_sclerp_planner_node_rightArm]: 执行命令: ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper "{}"
[rm_sclerp_planner_node_rightArm-1] waiting for service to become available...
[rm_sclerp_planner_node_rightArm-1] requester: making request: rm_ros_interfaces.srv.DeactivateGripper_Request()
[rm_sclerp_planner_node_rightArm-1] 
[rm_sclerp_planner_node_rightArm-1] response:
[rm_sclerp_planner_node_rightArm-1] rm_ros_interfaces.srv.DeactivateGripper_Response(success=True, message='Gripper deactivated (vacuum released)')
[rm_sclerp_planner_node_rightArm-1] 
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615608.743956946] [right_arm.rm_sclerp_planner_node_rightArm]: 服务调用成功
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615608.744084240] [right_arm.rm_sclerp_planner_node_rightArm]: 🔧 右臂吸盘已关闭
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615608.744106383] [right_arm.rm_sclerp_planner_node_rightArm]: ⏱️ 从触发左臂动作到关闭右臂吸盘耗时: 10.742883 秒
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615610.439257641] [RmRobotAPI]: rm_movej 调用成功
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615611.506645653] [right_arm.rm_sclerp_planner_node_rightArm]: 🔄 收到左臂反馈: 左臂任务完成
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615611.506986829] [right_arm.rm_sclerp_planner_node_rightArm]: ✅ 左臂任务成功: 左臂任务成功完成
[rm_sclerp_planner_node_rightArm-1] [INFO] [1760615611.507050348] [right_arm.rm_sclerp_planner_node_rightArm]: 🏁 左臂任务已完成，最终状态: 左臂任务完成
```

左臂：

```
[rm_sclerp_planner_node_leftArm-1] [INFO] [1760615598.001348880] [left_arm.rm_sclerp_planner_node_leftArm]: 收到右臂触发信号: 开始执行抓取、扫码、放置任务
[rm_sclerp_planner_node_leftArm-1] [INFO] [1760615598.008440847] [left_arm.rm_sclerp_planner_node_leftArm]: 🔄 开始执行左臂任务: 开始执行抓取、扫码、放置任务
[rm_sclerp_planner_node_leftArm-1] [INFO] [1760615599.381584507] [RmRobotAPI]: rm_movej 调用成功
[rm_sclerp_planner_node_leftArm-1] [INFO] [1760615599.388869653] [left_arm.rm_sclerp_planner_node_leftArm]: 旋转前位姿欧拉角: [3.096000, 0.031000, 3.122000]
[rm_sclerp_planner_node_leftArm-1] [INFO] [1760615600.461764932] [RmRobotAPI]: rm_movel调用成功
[rm_sclerp_planner_node_leftArm-1] [INFO] [1760615600.461845475] [left_arm.rm_sclerp_planner_node_leftArm]: 执行命令: ros2 service call /left_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper "{}"
[rm_sclerp_planner_node_leftArm-1] waiting for service to become available...
[rm_sclerp_planner_node_leftArm-1] requester: making request: rm_ros_interfaces.srv.ActivateGripper_Request()
[rm_sclerp_planner_node_leftArm-1] 
[rm_sclerp_planner_node_leftArm-1] response:
[rm_sclerp_planner_node_leftArm-1] rm_ros_interfaces.srv.ActivateGripper_Response(success=True, message='Gripper activated (vacuum enabled)')
[rm_sclerp_planner_node_leftArm-1] 
[rm_sclerp_planner_node_leftArm-1] [INFO] [1760615604.167089413] [left_arm.rm_sclerp_planner_node_leftArm]: 服务调用成功
[rm_sclerp_planner_node_leftArm-1] [INFO] [1760615604.167210178] [left_arm.rm_sclerp_planner_node_leftArm]: ⏱️ 吸盘启动耗时: 3.705365 秒
[rm_sclerp_planner_node_leftArm-1] [INFO] [1760615606.167481642]
```

从双臂执行结果来看，存在的问题是：左臂执行函数 execute_trigger_left_arm 的 `RCLCPP_INFO(this->get_logger(), "✅ 吸盘吸取物体成功");` 语句完成之后，执行“避障”语句时，右臂的 execute_grasp_box 函数还未执行到主动轮询部分的 while 循环语句中的吸盘关闭部分代码。

结合左右臂函数体实现与终端日志信息，分析原因，并给出解决方

##### 问题分析

从日志时间戳可以看出：

**左臂执行时间线**：

1. `1760615606.167481642` - 左臂完成吸盘吸取并发布反馈"吸盘吸取物体成功"
2. 然后立即执行避障操作（rm_movel调用）

**右臂执行时间线**：

1. `1760615606.167570984` - 右臂收到"吸盘吸取物体成功"反馈
2. `1760615606.172252702` - 右臂开始执行吸盘关闭操作
3. `1760615608.744084240` - 右臂完成吸盘关闭
4. `1760615610.439257641` - 右臂执行机械臂复位（rm_movej调用）

**问题**： 右臂在收到"吸盘吸取物体成功"反馈后，需要约2.5秒才能完成吸盘关闭操作，而在这期间左臂已经开始了避障动作。这导致了双臂动作的时序不协调。

##### 原因分析

1. **同步等待时机不当**：右臂在收到反馈后立即执行耗时操作（关闭吸盘需要约2.5秒），而没有提前做好准备
2. **缺乏协调机制**：左臂发送反馈后立即继续执行，没有等待右臂完成相应操作
3. **时序依赖问题**：右臂的吸盘关闭应该在左臂开始避障动作之前完成

##### 解决方案

我们需要调整双臂的协调机制，确保右臂在左臂执行避障动作之前完成吸盘关闭操作。以下是几种解决方案：

###### 方案一：左臂等待右臂完成操作后再继续执行

修改左臂的`execute_trigger_left_arm`函数：

```cpp
// 在左臂的 execute_trigger_left_arm 函数中调整执行顺序
void execute_trigger_left_arm(const std::shared_ptr<TriggerArmHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TriggerArm::Feedback>();
    auto result = std::make_shared<TriggerArm::Result>();
    
    RCLCPP_INFO(this->get_logger(), "🔄 开始执行左臂任务: %s", goal->task_description.c_str());
    
    // 发布反馈 - 开始执行
    feedback->status = "开始执行左臂抓取任务";
    goal_handle->publish_feedback(feedback);
    
    // 执行左臂任务
    tempForShow(goal_handle);  // 传入goal_handle以便发布反馈
    
    // 发布反馈 - 任务完成
    feedback->status = "左臂任务完成";
    goal_handle->publish_feedback(feedback);
    
    // 设置结果
    result->success = true;
    result->message = "左臂任务成功完成";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "✅ 左臂任务成功完成");
}
```

修改左臂的[tempForShow](javascript:void(0))函数，将吸盘成功反馈和避障操作分开：

```cpp
void tempForShow(const std::shared_ptr<TriggerArmHandle> goal_handle) 
{
    // ... 原有代码直到吸盘启动 ...
    
    // 启动吸盘
    auto suck_start_time = std::chrono::high_resolution_clock::now();
    std::string jodell_activate_cmd = "ros2 service call /left_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \"{}\"";
    call_service_sync(jodell_activate_cmd);  
    auto suck_end_time = std::chrono::high_resolution_clock::now();
    auto suck_duration = std::chrono::duration<double>(suck_end_time - suck_start_time);
    RCLCPP_INFO(this->get_logger(), "⏱️ 吸盘启动耗时: %.6f 秒", suck_duration.count());

    // 延时一段时间让吸盘工作
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 发布吸盘吸取结果反馈
    if (goal_handle) {
        auto feedback = std::make_shared<TriggerArm::Feedback>();
        feedback->status = "吸盘吸取物体成功";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "✅ 吸盘吸取物体成功");
    }
    
    // 注意：这里不再执行避障操作，让右臂完成相应操作后再继续
    
    // 等待一小段时间确保右臂完成操作
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    // 现在执行避障操作
    // 用于避障
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x + 0.05f,
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z,
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
        };
    rm_movel_cmd(gripper_tcp_pose, 40);
    
    // ... 后续代码 ...
}
```

###### 方案二：右臂优化等待和执行逻辑

修改右臂的 execute_grasp_box 中的等待逻辑，使其更加高效：

```cpp
// 在右臂 execute_grasp_box 函数中优化等待逻辑
{
    std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
    auto wait_start = std::chrono::steady_clock::now();
    auto total_timeout = std::chrono::seconds(35);
    bool suction_success_handled = false;
    auto suction_success_time = std::chrono::steady_clock::time_point();
    
    RCLCPP_INFO(this->get_logger(), "⏳ 开始等待左臂任务状态更新");
    
    while (std::chrono::steady_clock::now() - wait_start < total_timeout) {
        // 检查是否收到吸盘吸取成功的状态且尚未处理
        if (left_arm_task_state_.current_status == "吸盘吸取物体成功" && !suction_success_handled) {
            RCLCPP_INFO(this->get_logger(), "✅ 检测到左臂吸盘吸取成功，执行相应操作");
            suction_success_handled = true;
            suction_success_time = std::chrono::steady_clock::now();
            
            // 记录开始时间
            auto close_start_time = std::chrono::high_resolution_clock::now();
            
            // 执行吸盘成功后的操作（关闭右臂吸盘）
            std::this_thread::sleep_for(std::chrono::milliseconds(4000)); // 等待左臂吸取纸箱
            std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
            call_service_sync(jodell_deactivate_cmd);
            
            auto close_end_time = std::chrono::high_resolution_clock::now();
            auto close_duration = std::chrono::duration<double>(close_end_time - close_start_time);
            RCLCPP_INFO(this->get_logger(), "🔧 右臂吸盘已关闭");
            RCLCPP_INFO(this->get_logger(), "⏱️ 从触发左臂动作到关闭右臂吸盘耗时: %.6f 秒", close_duration.count());
        }
        
        // 检查任务是否完成
        if (left_arm_task_state_.completed) {
            // 如果已经处理了吸盘成功但距离成功时间很短，再等一小会儿
            if (suction_success_handled) {
                auto elapsed_since_success = std::chrono::steady_clock::now() - suction_success_time;
                if (elapsed_since_success < std::chrono::seconds(5)) {
                    // 等待足够时间确保右臂操作完成
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "🏁 左臂任务已完成，最终状态: %s", left_arm_task_state_.current_status.c_str());
            left_arm_success = left_arm_task_state_.success;
            left_arm_message = left_arm_task_state_.message;
            break;
        }
        
        // 等待通知或短暂超时，避免忙等待
        left_arm_state_cv_.wait_for(lock, std::chrono::milliseconds(50));
    }
    
    // 检查是否超时
    if (!left_arm_task_state_.completed) {
        RCLCPP_WARN(this->get_logger(), "⚠️ 等待左臂任务状态超时");
        left_arm_success = false;
        left_arm_message = "Timeout waiting for left arm task result";
    } else if (!suction_success_handled) {
        RCLCPP_WARN(this->get_logger(), "⚠️ 左臂任务完成但未检测到吸盘成功状态，当前状态: %s", 
                   left_arm_task_state_.current_status.c_str());
        // 即使未明确收到吸盘成功状态，也执行清理操作
        std::this_thread::sleep_for(std::chrono::milliseconds(4000));
        std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
        call_service_sync(jodell_deactivate_cmd);
        RCLCPP_INFO(this->get_logger(), "🔧 右臂吸盘已关闭（默认操作）");
    }
}
```

###### 方案三：增加中间状态反馈（推荐）

最优解决方案是在左臂中增加更多的状态反馈，让右臂能够更精确地协调动作：

```cpp
// 在左臂 tempForShow 函数中增加更多状态反馈
void tempForShow(const std::shared_ptr<TriggerArmHandle> goal_handle) 
{
    // ... 原有代码 ...
    
    // 启动吸盘
    auto suck_start_time = std::chrono::high_resolution_clock::now();
    std::string jodell_activate_cmd = "ros2 service call /left_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \"{}\"";
    call_service_sync(jodell_activate_cmd);  
    auto suck_end_time = std::chrono::high_resolution_clock::now();
    auto suck_duration = std::chrono::duration<double>(suck_end_time - suck_start_time);
    RCLCPP_INFO(this->get_logger(), "⏱️ 吸盘启动耗时: %.6f 秒", suck_duration.count());

    // 延时一段时间让吸盘工作
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 发布吸盘吸取结果反馈
    if (goal_handle) {
        auto feedback = std::make_shared<TriggerArm::Feedback>();
        feedback->status = "吸盘吸取物体成功";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "✅ 吸盘吸取物体成功");
    }
    
    // 等待右臂完成操作的通知（通过额外的状态反馈）
    std::this_thread::sleep_for(std::chrono::milliseconds(3500)); // 等待右臂完成吸盘关闭
    
    // 发布避障开始反馈
    if (goal_handle) {
        auto feedback = std::make_shared<TriggerArm::Feedback>();
        feedback->status = "开始避障动作";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "🔄 开始避障动作");
    }
    
    // 用于避障
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x + 0.05f,
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z,
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
        };
    rm_movel_cmd(gripper_tcp_pose, 40);
    
    // ... 后续代码 ...
}
```

##### 推荐方案

我推荐使用**方案三**，因为它提供了最清晰的双臂协调机制。通过增加中间状态反馈，右臂可以更精确地知道何时执行相应操作，而左臂也知道何时继续执行后续动作。

这样可以确保：

1. 左臂在吸盘吸取成功后通知右臂
2. 右臂收到通知后执行吸盘关闭操作
3. 左臂等待右臂完成操作后再继续执行避障动作
4. 整个双臂协同过程更加有序和可

#### 最终应用

右臂 执行函数

```c++
void execute_grasp_box(const std::shared_ptr<GraspBoxHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "🔄 执行抓取盒子动作 Grasp box");
    rclcpp::Rate loop_rate(1);    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GraspBox::Feedback>();
    auto result = std::make_shared<GraspBox::Result>();

    // 重置左臂接收盒子任务状态
    {
        std::lock_guard<std::mutex> lock(left_arm_task_state_.mutex);
        left_arm_task_state_.completed = false;
        left_arm_task_state_.success = false;
        left_arm_task_state_.message = "";
        left_arm_task_state_.current_status = "Initializing";
    }        


    // 初始化
    try {
        // 机械臂复位
        std::vector<double> initial_joint_position = {-44.811, 69.029, -48.806, 71.437, -40.008, 83.439, -62.740};
        rmRobotAPI_->rm_movej(initial_joint_position, 50);


        // 使用类成员变量跟踪是否已初始化规划器
        if (!planner_initialized_) {
            initialize_planner();
            // 检查规划器是否已初始化
            if (!planner_interface_) {
                RCLCPP_WARN(this->get_logger(), "%s 规划器尚未初始化完成，无法执行轨迹规划", WARN_ICON.c_str());
                return;
            }
            // 标记规划器已初始化
            planner_initialized_ = true;
        }           
        // 发布反馈
        auto feedback = std::make_shared<GraspBox::Feedback>();
        feedback->status = 1; // 初始化完成
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "机械臂已就位，规划器初始化完成");

    }catch (const std::exception& e) {
        result->success = false;
        result->message = "Hardware initialization failed: " + std::string(e.what());
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Hardware initialization failed: %s", e.what());
        return;
    }

    // 检查是否需要取消
    if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Action canceled during initialization";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Action canceled during initialization");
        return;
    }

    // 启动吸盘(异步非阻塞模式)
    RCLCPP_INFO(this->get_logger(), "✅ 启动吸盘服务");
    call_jodell_service_async(jodell_activate_cmd); 

    // 轨迹规划与执行
    // 打印接收到的位姿信息
    RCLCPP_INFO(this->get_logger(), "%s 收到盒子中心点位姿 - 位置: [%f, %f, %f], 方向: [%f, %f, %f, %f]", 
            DATA_ICON.c_str(),
            goal->pose.position.x, 
            goal->pose.position.y, 
            goal->pose.position.z,
            goal->pose.orientation.x, 
            goal->pose.orientation.y, 
            goal->pose.orientation.z, 
            goal->pose.orientation.w
        );

    if (!trajectory_planned_) 
    {
        RCLCPP_INFO(this->get_logger(), "%s 开始规划轨迹", WORKING_ICON.c_str());

        // 获取参数
        std::string start_joints_str;       
        this->get_parameter("start_joint_positions", start_joints_str);
        // 打印参数
        RCLCPP_INFO(this->get_logger(), "%s 原始起始关节角度字符串: %s", DATA_ICON.c_str(), start_joints_str.c_str());
        // 解析起始关节位置参数
        Eigen::VectorXd jointAngles_start(7);
        parse_joint_positions(start_joints_str, jointAngles_start);       

        // 计算正向运动学
        //#region 目标姿态为感知获取的盒子姿态
        /*
        Eigen::Vector3d tcpPos;
        Eigen::Quaterniond quaternion;

        tcpPos = Eigen::Vector3d(
            goal->pose.position.x + 0.080 + 0.1318,
            goal->pose.position.y,
            goal->pose.position.z);            
        quaternion = Eigen::Quaterniond(
            goal->pose.orientation.w,
            goal->pose.orientation.x,
            goal->pose.orientation.y,
            goal->pose.orientation.z);
        Eigen::Matrix4d matrix_base2tcp_target;
        try {
            matrix_base2tcp_target = tcpPose2HomogenMatrix(quaternion, tcpPos);
            // 打印目标矩阵
            RCLCPP_INFO(this->get_logger(), "%s 目标齐次矩阵 matrix_base2tcp_target:",DATA_ICON.c_str());
            for (int i = 0; i < 4; i++) {
                RCLCPP_INFO(this->get_logger(), "  [%f, %f, %f, %f]", 
                            matrix_base2tcp_target(i,0), matrix_base2tcp_target(i,1), 
                            matrix_base2tcp_target(i,2), matrix_base2tcp_target(i,3));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "%s 计算正向运动学时出错: %s", ERROR_ICON.c_str(), e.what());
            return;
        }
        */
        //#endregion

        // #region 目标姿态沿用初始姿态
        rm_current_arm_state_t rm_robot_current_state_1;
        rm_get_arm_state(rm_robot_current_state_1);
        Eigen::Matrix<double, 6, 1> gripper_tcp_pose_1;
        gripper_tcp_pose_1 << goal->pose.position.x + 0.060 + 0.1318,
                                goal->pose.position.y,
                                goal->pose.position.z,
                                rm_robot_current_state_1.pose.euler.rx,
                                rm_robot_current_state_1.pose.euler.ry,
                                rm_robot_current_state_1.pose.euler.rz;
        Eigen::Matrix4d matrix_base2tcp_target;  
        matrix_base2tcp_target = tcpPose2HomogenMatrix(gripper_tcp_pose_1); 
        // #endregion

        // 调用重构后的sclerp_planner函数进行轨迹规划
        trajectory_msgs::msg::JointTrajectory joint_trajectory;
        bool planner_result = sclerp_planner(jointAngles_start, matrix_base2tcp_target, joint_trajectory);
        if (!planner_result) {
            RCLCPP_ERROR(this->get_logger(), "%s 轨迹规划失败", ERROR_ICON.c_str());
            result->success = false;
            result->message = "Trajectory planning failed";
            goal_handle->abort(result);
            return;
        }

        // 标记轨迹已规划
        RCLCPP_INFO(this->get_logger(), "%s 轨迹规划成功", SUCCESS_ICON.c_str());

        // 启动吸盘(异步非阻塞模式)
        // RCLCPP_INFO(this->get_logger(), "✅ 启动吸盘服务");
        // call_jodell_service_async(jodell_activate_cmd); 

        // 以固定周期执行轨迹发布（原trajectory_timer_callback中的逻辑）
        if(trajectory_planned_ )
        {
            int execution_rate;
            this->get_parameter("trajectory_execution_rate", execution_rate);
            publishJointTrajectory(joint_trajectory, execution_rate);    

        } 

        trajectory_planned_ = false;

    }

    // 启动吸盘（同步阻塞模式）
    // RCLCPP_INFO(this->get_logger(), "%s 启动吸盘服务", LAUNCH_ICON.c_str());
    // RCLCPP_INFO(this->get_logger(), "✅ 启动吸盘服务");
    // call_jodell_service_sync(jodell_activate_cmd);   

    // 延时让右臂吸盘工作以吸紧物体
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); 

    // 右臂上升用于避障
    rm_current_arm_state_t rm_robot_current_state;
    rm_get_arm_state(rm_robot_current_state);
    std::vector<float> gripper_tcp_pose(6, 0.0f);
    gripper_tcp_pose = {
        rm_robot_current_state.pose.position.x + 0.10f,
        rm_robot_current_state.pose.position.y,
        rm_robot_current_state.pose.position.z,
        rm_robot_current_state.pose.euler.rx,
        rm_robot_current_state.pose.euler.ry,
        rm_robot_current_state.pose.euler.rz
    };
    rm_movel_cmd(gripper_tcp_pose, 40);


    // 换手场景
    bool triggerArm = true;
    if(triggerArm)
    {
        // 运动到交接位置
        std::vector<double> target_joint_position = {-84.578, 97.793, 19.109, -47.483, -24.050, 127.871, -99.886};
        rmRobotAPI_->rm_movej(target_joint_position, 50);              
        rm_get_arm_state(rm_robot_current_state);            
        gripper_tcp_pose = {rm_robot_current_state.pose.position.x ,
                            rm_robot_current_state.pose.position.y,
                            rm_robot_current_state.pose.position.z - 0.115f,
                            rm_robot_current_state.pose.euler.rx,
                            rm_robot_current_state.pose.euler.ry,
                            rm_robot_current_state.pose.euler.rz
                    };
        // RCLCPP_INFO(this->get_logger(), "旋转前位姿欧拉角: [%f, %f, %f]",
        //             gripper_tcp_pose[3], gripper_tcp_pose[4], gripper_tcp_pose[5]);
        rm_movel_cmd(gripper_tcp_pose, 50); 
        RCLCPP_INFO(this->get_logger(), "%s 右臂到达交付点", COMPLETE_ICON.c_str());            
        // 触发左臂动作
        RCLCPP_INFO(this->get_logger(), "%s 触发左臂动作服务", REQUEST_ICON.c_str());  
        trigger_left_arm_task("开始执行抓取、扫码、放置任务");   

        // 左臂就位后，关闭吸盘   

        // auto suck_start_time = std::chrono::high_resolution_clock::now();

// #region  🚥左臂动作反馈触发右臂吸盘关闭判断 🚨

        // 根据左臂动作执行状态（吸取结果）决定右臂吸盘关闭
        bool left_arm_success = false;
        std::string left_arm_message = "";


        // 等待左臂执行过程中间状态更新和最终结果
        {
            std::unique_lock<std::mutex> lock(left_arm_task_state_.mutex);
            auto wait_start = std::chrono::steady_clock::now();
            auto total_timeout = std::chrono::seconds(25); // 总超时时间
            bool suction_success_handled = false;
            bool arm_arrived_handled = false;

            RCLCPP_INFO(this->get_logger(), "⏳ 开始等待左臂任务状态更新");

            while (std::chrono::steady_clock::now() - wait_start < total_timeout) 
            {
                // 检查是否收到吸盘吸取成功的状态且尚未处理
                if (left_arm_task_state_.current_status == "左臂成功接触到纸箱" &&
                !arm_arrived_handled)  
                {   
                    RCLCPP_INFO(this->get_logger(), "✅ 左臂成功接触到纸箱");
                    arm_arrived_handled = true;
                    // 右臂吸盘关闭策略（1）：左臂接触到纸箱时
                    call_jodell_service_async(jodell_deactivate_cmd);
                }

                if (left_arm_task_state_.current_status == "吸盘吸取物体成功" && !suction_success_handled) 
                {
                    RCLCPP_INFO(this->get_logger(), "✅ 检测到左臂吸盘吸取成功，执行相应操作");
                    suction_success_handled = true;
                    
					// 右臂吸盘关闭策略（2）：左臂吸取成功时
                    // call_jodell_service_sync(jodell_deactivate_cmd);
                    // RCLCPP_INFO(this->get_logger(), "🔧 右臂吸盘已关闭");
                    // auto suck_end_time = std::chrono::high_resolution_clock::now();
                    // auto suck_duration = std::chrono::duration<double>(suck_end_time - suck_start_time);
                    // RCLCPP_INFO(this->get_logger(), "%s 从触发左臂动作到关闭右臂吸盘耗时: %.6f 秒", TIMER_ICON.c_str(), suck_duration.count());

                    // 机械臂复位到初始位置
                    std::vector<double> initial_joint_position = {-44.811, 69.029, -48.806, 71.437, -40.008, 83.439, -62.740};
                    rmRobotAPI_->rm_movej(initial_joint_position, 50);
                }

                // 检查任务是否完成
                if (left_arm_task_state_.completed) {
                    RCLCPP_INFO(this->get_logger(), "🏁 左臂任务已完成，最终状态: %s", left_arm_task_state_.current_status.c_str());
                    left_arm_success = left_arm_task_state_.success;
                    left_arm_message = left_arm_task_state_.message;
                    break;
                }

                // 等待通知或短暂超时，避免忙等待
                left_arm_state_cv_.wait_for(lock, std::chrono::milliseconds(50));
            }

            // 检查是否超时
            if (!left_arm_task_state_.completed) {
                RCLCPP_WARN(this->get_logger(), "⚠️ 等待左臂任务状态超时");
                left_arm_success = false;
                left_arm_message = "Timeout waiting for left arm task result";
            } else if (!suction_success_handled) 
            {
                RCLCPP_WARN(this->get_logger(), "⚠️ 左臂任务完成但未检测到吸盘成功状态，当前状态: %s", 
                        left_arm_task_state_.current_status.c_str());
                // 即使未明确收到吸盘成功状态，也执行清理操作
                call_jodell_service_sync(jodell_deactivate_cmd);
                RCLCPP_INFO(this->get_logger(), "🔧 右臂吸盘已关闭（默认操作）");
                // 机械臂复位到初始位置
                std::vector<double> initial_joint_position = {-44.811, 69.029, -48.806, 71.437, -40.008, 83.439, -62.740};
                rmRobotAPI_->rm_movej(initial_joint_position, 50);
            }
        }

        // 根据左臂任务结果决定后续操作
        if (!left_arm_success) {
            // 左臂任务失败，关闭右臂吸盘并报告失败
            call_jodell_service_sync(jodell_deactivate_cmd);

            result->success = false;
            result->message = "Left arm task failed: " + left_arm_message;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "❌ 因左臂任务失败，终止抓取任务: %s", left_arm_message.c_str());
            return;
        }
// #endregion
        
// #region ⌛固定时间延时触发式右臂吸盘关闭判断 🚨 
        {
        // 延时用于等待左臂吸取到纸箱
        // std::this_thread::sleep_for(std::chrono::milliseconds(4000)); 
        // 关闭吸盘（TODO：改为动作返回结果触发吸盘关闭）
        // call_jodell_service_sync(jodell_deactivate_cmd);

        // 机械臂复位到初始位置
        // std::vector<double> initial_joint_position = {-44.811, 69.029, -48.806, 71.437, -40.008, 83.439, -62.740};
        // rmRobotAPI_->rm_movej(initial_joint_position, 50);

        // 延时用于等待左臂完成任务
        // std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 
        }
 // #endregion

    }


    // 输出最终结果
    if(rclcpp::ok()){
        result->success = true;
        result->message = "Grasp box action completed successfully";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Grasp box action succeeded");
    }

}

```

左臂 execute_trigger_left_arm 函数

```c++
void execute_trigger_left_arm(
    const std::shared_ptr<TriggerArmHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TriggerArm::Feedback>();
    auto result = std::make_shared<TriggerArm::Result>();

    RCLCPP_INFO(this->get_logger(), "🔄 开始执行左臂任务: %s", goal->task_description.c_str());

    // 发布反馈 - 开始执行
    feedback->status = "开始执行左臂抓取任务";
    goal_handle->publish_feedback(feedback);

    // 异步非阻塞模式启动吸盘
    call_jodell_service_async(jodell_activate_cmd);

    // 方案1
    {    
    // 执行左臂任务（吸取物体、扫码、放置等）
    // bool success = execute_left_arm_task();
    // if (success) {
        // 发布反馈 - 任务完成
    //     feedback->status = "左臂任务完成";
    //     goal_handle->publish_feedback(feedback);

        // 设置结果
    //     result->success = true;
    //     result->message = "左臂任务成功完成";
    //     goal_handle->succeed(result);
    //     RCLCPP_INFO(this->get_logger(), "左臂任务成功完成");
    // } else {
    //     result->success = false;
    //     result->message = "左臂任务执行失败";
    //     goal_handle->abort(result);
    //     RCLCPP_ERROR(this->get_logger(), "左臂任务执行失败");
    // }
    }

    // 方案2
    {
    // 接收纸箱点位
    // 靠近纸箱
    std::vector<double> target_joint_position = {-67.902, -54.464, -56.450, -49.018, -138.739, 89.207, 73.986};
    rmRobotAPI_->rm_movej(target_joint_position, 50);
    // 接触纸箱，工具坐标系Z 轴方向进给运动
    rm_current_arm_state_t rm_robot_current_state;
    rm_get_arm_state(rm_robot_current_state);
    std::vector<float> gripper_tcp_pose(6, 0.0f);
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x ,
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z - 0.115f,
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
                };
    RCLCPP_INFO(this->get_logger(), "旋转前位姿欧拉角: [%f, %f, %f]",
                gripper_tcp_pose[3], gripper_tcp_pose[4], gripper_tcp_pose[5]);
    rm_movel_cmd(gripper_tcp_pose, 50);

    feedback->status = "左臂成功接触到纸箱"; // 或者使用数字状态码，如 2 表示吸取成功
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "✅ 左臂成功接触到纸箱");


    // 延时一段时间让吸盘工作以吸紧物体
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));

    // TODO：吸盘状态检查，判断吸取是否成功

    // 发布吸盘吸取结果反馈
    feedback->status = "吸盘吸取物体成功"; // 或者使用数字状态码，如 2 表示吸取成功
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "✅ 吸盘吸取物体成功");
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 上升用于避障
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x + 0.15f,
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z,
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
        };
    rm_movel_cmd(gripper_tcp_pose, 40);

    // 关闭吸盘（异步非阻塞模式）
    call_jodell_service_async(jodell_deactivate_cmd);

    // 运动到释放点
    std::vector<double> release_joint_position = {-113.882, -52.867, -33.962, -29.249, -55.023, 126.469, 105.515};
    rmRobotAPI_->rm_movej(release_joint_position, 40);        

    // 关闭 Modbus（可注释🗒）
    // call_jodell_service_sync(jodell_CloseModbus_cmd);  

    // 发布反馈 - 任务完成
    feedback->status = "左臂任务完成";
    goal_handle->publish_feedback(feedback);

    // 设置结果
    result->success = true;
    result->message = "左臂任务成功完成";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "✅ 左臂任务成功完成");

    }

}
```



## 🎯  电吸盘调用改进

### 现有电吸盘接口函数

> 分析吸盘调用函数call_service_sync 是同步阻塞模式还是异步非阻塞模式？

[call_service_sync](javascript:void(0))函数是**同步阻塞模式**。

从函数实现可以看出：

```cpp
void call_service_sync(const std::string& cmd) {
    RCLCPP_INFO(this->get_logger(), "执行命令: %s", cmd.c_str());
    int result = system(cmd.c_str());
    if (result == 0) {
        RCLCPP_INFO(this->get_logger(), "服务调用成功");
    } else {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
    }
}
```

该函数使用`system()`调用执行命令，这会阻塞当前线程直到命令执行完成。函数名中的"sync"也明确表示这是同步调用。当右臂调用此函数关闭吸盘时，会一直等待直到服务调用完成并返回结果，期间线程无法执行其他操作。

### 电吸盘接口函数改进

> 如何修改call_service_sync函数为异步非阻塞模式？

#### 可行方案

要将[call_service_sync](javascript:void(0))函数修改为异步非阻塞模式，可以使用以下几种方法：

##### 方法一：使用std::async创建异步任务

```cpp
// 添加必要的头文件
#include <future>

// 修改函数为异步非阻塞模式
std::future<int> call_service_async(const std::string& cmd) {
    RCLCPP_INFO(this->get_logger(), "执行命令: %s", cmd.c_str());
    
    // 使用std::async在后台线程中执行system调用
    return std::async(std::launch::async, [this, cmd]() {
        int result = system(cmd.c_str());
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "服务调用成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用失败");
        }
        return result;
    });
}
```

使用示例：

```cpp
// 调用异步函数
auto future = call_service_async(jodell_deactivate_cmd);

// 可以选择是否等待结果
// future.wait(); // 阻塞等待完成
// 或者检查是否完成
// if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
//     int result = future.get(); // 获取结果
// }
```

##### 方法二：使用ROS 2的异步服务客户端

```cpp
// 使用ROS 2内置的服务机制替代system调用
// 首先声明服务客户端
rclcpp::Client<rm_ros_interfaces::srv::DeactivateGripper>::SharedPtr deactivate_gripper_client_;

// 在构造函数中初始化客户端
deactivate_gripper_client_ = this->create_client<rm_ros_interfaces::srv::DeactivateGripper>(
    "/right_arm/deactivate_gripper");

// 异步调用服务的函数
void call_service_async(const std::string& service_name) {
    while (!deactivate_gripper_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "中断等待服务可用");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务可用...");
    }

    auto request = std::make_shared<rm_ros_interfaces::srv::DeactivateGripper::Request>();
    
    // 异步调用服务
    auto result_future = deactivate_gripper_client_->async_send_request(request,
        [this](rclcpp::Client<rm_ros_interfaces::srv::DeactivateGripper>::SharedFuture future) {
            auto result = future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "服务调用成功: %s", result->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "服务调用失败: %s", result->message.c_str());
            }
        });
    
    RCLCPP_INFO(this->get_logger(), "已发送异步服务请求");
}
```

##### 方法三：使用线程执行system调用

```cpp
#include <thread>

void call_service_async(const std::string& cmd) {
    RCLCPP_INFO(this->get_logger(), "执行命令: %s", cmd.c_str());
    
    // 在新线程中执行system调用，不阻塞当前线程
    std::thread([this, cmd]() {
        int result = system(cmd.c_str());
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "服务调用成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用失败");
        }
    }).detach(); // 分离线程，让它独立运行
}
```

推荐使用方法二（ROS 2异步服务客户端），因为：

1. 它是ROS 2原生支持的方式，与ROS 2生态系统更好地集成
2. 提供了更好的错误处理和类型安全性
3. 不需要使用system()调用，避免了潜在的安全风险
4. 可以获得服务调用的返回值和状态信息
5. 更符合ROS 2的编程范式

这些方法都可以将原来的同步阻塞调用改为异步非阻塞调用，让程序在等待服务调用完成的同时可以继续执行其他任务。

#### QNA

> 在左右臂程序中，吸盘的调用分为初始化，启动和关闭，若是按照上述回答中的方法二，岂不是要创建3个服务客户端与相应的异步调用服务的函数？是否可以统一成一个客户端和调用函数? 类似既有的 call_service_sync函数

是的，完全可以使用一个统一的服务客户端和调用函数来处理所有吸盘操作（初始化、启动和关闭）。我们可以通过模板或函数参数来实现通用的异步服务调用函数。

以下是实现方案：

##### 方案一：使用模板函数创建通用异步服务调用

```cpp
// 声明不同吸盘服务的客户端
rclcpp::Client<rm_ros_interfaces::srv::InitializeGripper>::SharedPtr init_gripper_client_;
rclcpp::Client<rm_ros_interfaces::srv::ActivateGripper>::SharedPtr activate_gripper_client_;
rclcpp::Client<rm_ros_interfaces::srv::DeactivateGripper>::SharedPtr deactivate_gripper_client_;

// 在构造函数中初始化所有客户端
init_gripper_client_ = this->create_client<rm_ros_interfaces::srv::InitializeGripper>("/right_arm/initialize_gripper");
activate_gripper_client_ = this->create_client<rm_ros_interfaces::srv::ActivateGripper>("/right_arm/activate_gripper");
deactivate_gripper_client_ = this->create_client<rm_ros_interfaces::srv::DeactivateGripper>("/right_arm/deactivate_gripper");

// 通用异步服务调用模板函数
template<typename ServiceT>
void call_service_async(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    const std::string& service_name,
    typename ServiceT::Request::SharedPtr request)
{
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "中断等待服务 %s 可用", service_name.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务 %s 可用...", service_name.c_str());
    }

    // 异步调用服务
    auto result_future = client->async_send_request(request,
        [this, service_name](typename rclcpp::Client<ServiceT>::SharedFuture future) {
            auto result = future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "服务 %s 调用成功: %s", service_name.c_str(), result->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "服务 %s 调用失败: %s", service_name.c_str(), result->message.c_str());
            }
        });
    
    RCLCPP_INFO(this->get_logger(), "已发送异步服务请求到 %s", service_name.c_str());
}

// 使用示例：
// 1. 初始化吸盘
auto init_request = std::make_shared<rm_ros_interfaces::srv::InitializeGripper::Request>();
init_request->ip = "192.168.1.19";
init_request->port = 8080;
init_request->register_address = 1000;
init_request->comm_port = 1;
init_request->device_address = 9;
call_service_async<rm_ros_interfaces::srv::InitializeGripper>(init_gripper_client_, "initialize_gripper", init_request);

// 2. 启动吸盘
auto activate_request = std::make_shared<rm_ros_interfaces::srv::ActivateGripper::Request>();
call_service_async<rm_ros_interfaces::srv::ActivateGripper>(activate_gripper_client_, "activate_gripper", activate_request);

// 3. 关闭吸盘
auto deactivate_request = std::make_shared<rm_ros_interfaces::srv::DeactivateGripper::Request>();
call_service_async<rm_ros_interfaces::srv::DeactivateGripper>(deactivate_gripper_client_, "deactivate_gripper", deactivate_request);
```

##### 方案二：使用std::function创建更通用的异步调用函数

```cpp
// 使用std::function实现通用异步服务调用
void call_service_async_generic(
    const std::string& service_name,
    std::function<void()> service_call)
{
    RCLCPP_INFO(this->get_logger(), "准备异步调用服务: %s", service_name.c_str());
    
    // 在新线程中执行服务调用，不阻塞当前线程
    std::thread([this, service_name, service_call]() {
        try {
            service_call();
            RCLCPP_INFO(this->get_logger(), "异步服务调用 %s 完成", service_name.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "异步服务调用 %s 失败: %s", service_name.c_str(), e.what());
        }
    }).detach();
}

// 使用示例：
// 1. 初始化吸盘
call_service_async_generic("initialize_gripper", [this]() {
    std::string cmd = "ros2 service call /right_arm/initialize_gripper rm_ros_interfaces/srv/InitializeGripper \""
                      "{ip: '192.168.1.19', port: 8080, register_address: 1000, comm_port: 1, device_address: 9}\"";
    system(cmd.c_str());
});

// 2. 启动吸盘
call_service_async_generic("activate_gripper", [this]() {
    std::string cmd = "ros2 service call /right_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \"{}\"";
    system(cmd.c_str());
});

// 3. 关闭吸盘
call_service_async_generic("deactivate_gripper", [this]() {
    std::string cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"";
    system(cmd.c_str());
});
```

##### 方案三：改进现有函数，添加回调参数（推荐）

```cpp
// 改进现有的call_service_sync函数，添加完成回调
void call_service_async(
    const std::string& cmd,
    std::function<void(bool success, const std::string& message)> callback = nullptr)
{
    RCLCPP_INFO(this->get_logger(), "执行命令: %s", cmd.c_str());
    
    // 在新线程中执行system调用，不阻塞当前线程
    std::thread([this, cmd, callback]() {
        int result = system(cmd.c_str());
        bool success = (result == 0);
        std::string message = success ? "服务调用成功" : "服务调用失败";
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        }
        
        // 如果提供了回调函数，则调用它
        if (callback) {
            callback(success, message);
        }
    }).detach();
}

// 使用示例：
// 1. 初始化吸盘
call_service_async(
    "ros2 service call /right_arm/initialize_gripper rm_ros_interfaces/srv/InitializeGripper \""
    "{ip: '192.168.1.19', port: 8080, register_address: 1000, comm_port: 1, device_address: 9}\"",
    [this](bool success, const std::string& message) {
        if (success) {
            RCLCPP_INFO(this->get_logger(), "吸盘初始化完成");
        } else {
            RCLCPP_ERROR(this->get_logger(), "吸盘初始化失败: %s", message.c_str());
        }
    });

// 2. 启动吸盘
call_service_async(
    "ros2 service call /right_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \"{}\"",
    [this](bool success, const std::string& message) {
        if (success) {
            RCLCPP_INFO(this->get_logger(), "吸盘启动成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "吸盘启动失败: %s", message.c_str());
        }
    });

// 3. 关闭吸盘
call_service_async(
    "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \"{}\"",
    [this](bool success, const std::string& message) {
        if (success) {
            RCLCPP_INFO(this->get_logger(), "吸盘关闭成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "吸盘关闭失败: %s", message.c_str());
        }
    });
```

推荐使用方案三，因为它对现有代码改动最小，保持了与现有[call_service_sync](javascript:void(0))函数的兼容性，同时添加了异步非阻塞特性。如果不需要回调处理，可以直接像以前一样调用；如果需要异步处理结果，可以提供回调函数。



## 🎯  双臂作业流程异常/错误处理（TODO）

### 轨迹规划异常/失败处理机制





### 吸盘吸附目标物体异常/失败处理机制



## AI 提示词

如何重构右臂的 函数，将当前的函数体代码分割成几个模块，分别封装成独立可复用的模块



## 遗留问题

### 创建规划器接口时出错: bad_weak_ptr

#### 问题描述

将规划器初始化函数 initialize_planner();放入了节点的构造函数中, 运行节点报错:

```
[rm_sclerp_planner_node_rightArm-1] [ERROR] [1758867661.618640457] [rm_sclerp_planner_node_rightArm]: 创建规划器接口时出错: bad_weak_ptr
```



#### 原因分析

错误信息"创建规划器接口时出错: bad_weak_ptr"表明在创建[sclerp_interface::ScLERPInterface](javascript:void(0))对象时发生了`bad_weak_ptr`异常。这个错误通常与ROS 2中的节点共享指针生命周期管理有关。

查看代码发现，[ScLERPInterface](javascript:void(0)) 构造函数需要一个`node_ptr`参数，即节点的共享指针。在[initialize_planner()](javascript:void(0))函数中，使用了`shared_from_this()`来获取当前节点的共享指针：

```
cppplanner_interface_ = std::make_unique<sclerp_interface::ScLERPInterface>(
    base_link, tip_link, shared_from_this(), urdf_path);
```

`bad_weak_ptr`异常通常发生在以下情况：

1. `shared_from_this()`在对象尚未完全构造或已经被销毁时被调用
2. 对象不是通过shared_ptr创建的
3. 在构造函数中调用`shared_from_this()`

在当前代码中，问题出现在节点构造函数中直接调用了[initialize_rmRobot()](javascript:void(0))和[initialize_planner()](javascript:void(0))，此时节点对象可能尚未完全构造完成，调用`shared_from_this()`就会导致`bad_weak_ptr`异常

#### 解决方案

将[initialize_planner()](javascript:void(0))和[initialize_rmRobot()](javascript:void(0))的调用从构造函数中移出，改为在节点完全构造后通过定时器回调或其他机制调用。

##### 方案一：使用初始化定时器（推荐）

保持现有的[init_timer_callback()](javascript:void(0))机制，在其中调用[initialize_rmRobot()](javascript:void(0))和[initialize_planner()](javascript:void(0))：

```cpp
class RmSclerpPlannerNode1 : public rclcpp::Node
{
  public:
    explicit RmSclerpPlannerNode1(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("rm_sclerp_planner_node_rightArm", options)
    {
        /************************************* 参数申明与获取 ***********************************/
        // 声明参数
        this->declare_parameter<std::string>("start_joint_positions",  "-0.924, 1.504, -0.371, 1.876, -0.962, 1.126, -0.046");
        this->declare_parameter<std::string>("target_joint_positions", "-1.473, 1.868, -0.651, 0.330, -0.871, 1.490, -1.036");
        this->declare_parameter<std::string>("urdf_path", "");
        this->declare_parameter<int>("trajectory_execution_rate", 50);  // 轨迹执行频率(Hz)（话题发布频率）
        this->declare_parameter<int>("trajectory_interpolation_rate", 100);  // 轨迹插值频率(Hz)

        // #region URDF路径参数检查（若无则强制指定）
        // 检查是否提供了URDF路径
        std::string urdf_path;
        this->get_parameter("urdf_path", urdf_path);
        if (urdf_path.empty()) {
            RCLCPP_WARN(this->get_logger(), "未提供URDF路径参数，将使用默认URDF路径");
            // 使用默认URDF路径
            urdf_path = "/home/byd/hrt2/ros2_rm_ws/src/ros2_rm_robot-humble/rm_description/urdf/rm_75_6fb.urdf";
            this->set_parameter(rclcpp::Parameter("urdf_path", urdf_path));
        }
        // 重新获取参数以确保获取到设置的值
        this->get_parameter("urdf_path", urdf_path);
        RCLCPP_INFO(this->get_logger(), "使用URDF路径: %s", urdf_path.c_str());
        // #endregion

        /********************************* 通信：话题，服务，动作 *******************************/        
        // 创建发布者，用于直接向rm_driver发送轨迹点
        joint_pos_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Jointpos>(
            "/rm_driver/movej_canfd_cmd", 20);

        // 创建订阅者，用于获取当前关节位置
        // 修改订阅话题为 /joint_states，这是rm_driver实际发布关节状态的话题
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 20,
            std::bind(&RmSclerpPlannerNode1::joint_state_callback, this, std::placeholders::_1));

        // 创建订阅者，用于接收盒子中心点位姿
        box_centerPoint_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/right_posedemo", 20,
            std::bind(&RmSclerpPlannerNode1::box_centerPoint_callback, this, std::placeholders::_1));

        // 创建MoveJ消息发布者
        movej_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movej>(
            "/rm_driver/movej_cmd", 10);

        /******************************************* 定时器 ****************************************/
        //『初始化定时器』
        // 用于初始化规划器和获取DH参数
        // 📝 在不用获取DH参数时，可注释不用
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RmSclerpPlannerNode1::init_timer_callback, this));

        // 『定时器』，
        // 用于周期性检查和执行任务
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 100ms检查一次
            std::bind(&RmSclerpPlannerNode1::timer_callback, this));

        // 『轨迹定时器』
        // 用于发布轨迹点，提高轨迹执行频率
        int execution_rate;
        this->get_parameter("trajectory_execution_rate", execution_rate);
        auto timer_period = std::chrono::microseconds(1000000 / execution_rate);
        trajectory_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&RmSclerpPlannerNode1::trajectory_timer_callback, this));

        /************************************** 变量初始化 **************************************/
        // 初始化轨迹点索引
        trajectory_point_index_ = 0;
        trajectory_executing_ = false;
        // 初始化 current_joint_positions_
        current_joint_positions_.fill(0.0);

        RCLCPP_INFO(this->get_logger(), "rm_sclerp_planner_node_1节点已经启动.");

        // 不再在构造函数中调用 initialize_rmRobot() 和 initialize_planner()
        // 这些将在 init_timer_callback 中调用，确保节点完全构造完成
    }
```

##### 方案二：使用ROS 2的on_activate机制

另一种方法是利用ROS 2的生命周期节点特性，将初始化代码放在`on_activate`回调中：

```cpp
class RmSclerpPlannerNode1 : public rclcpp::Node
{
public:
    explicit RmSclerpPlannerNode1(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("rm_sclerp_planner_node_rightArm", options)
    {
        // ... 其他初始化代码保持不变 ...
        
        // 将 initialize_rmRobot() 和 initialize_planner() 移到 on_activate 中
    }
    
    void on_activate()
    {
        Node::on_activate();
        // 初始化机械臂
        initialize_rmRobot();
        // 初始化规划器
        initialize_planner();
        
        // 初始化电吸盘
        std::string jodell_initialize_cmd = "ros2 service call /right_arm/initialize_gripper rm_ros_interfaces/srv/InitializeGripper \""
            "{ip: '192.168.1.19', port: 8080, register_address: 1000, comm_port: 1, device_address: 9}\"";
        call_service_sync(jodell_initialize_cmd); 
    }
    
    // ... 其他代码保持不变 ...
};
```

推荐使用方案一，因为它保持了现有代码结构的一致性，并且更容易理解和维护

##### QNA

> 若是保持现有的init_timer_callback()机制，在其中调用initialize_rmRobot()和initialize_planner(),该如何修改box_centerPoint_callback 函数确保首次订阅到话题消息以及规划器初始化成功才触发轨迹规划与发布执行?

为了确保在[box_centerPoint_callback](javascript:void(0))函数中只有在首次订阅到话题消息且规划器初始化成功后才触发轨迹规划与发布执行，需要进行以下修改：

###### 修改方案

需要在类中添加一个标志位来跟踪规划器是否已初始化，并在[box_centerPoint_callback](javascript:void(0))中检查该标志位：

1. 在类的成员变量中添加规划器初始化状态标志
2. 在[init_timer_callback()](javascript:void(0))中设置该标志
3. 在[box_centerPoint_callback](javascript:void(0))中检查该标志

###### 具体实现

```cpp
/**
 * @brief 盒子中心点位姿回调函数
 * 
 * 处理从 /right_posedemo 话题接收到的盒子中心点位姿消息。
 * 
 * @param msg 接收到的 PoseStamped 消息
 */
void box_centerPoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if(!first_received_){
        first_received_ = true; 
    } else {
        return;
    }

    // 检查规划器是否已初始化
    if (!planner_interface_) {
        RCLCPP_WARN(this->get_logger(), "规划器尚未初始化完成，无法执行轨迹规划");
        first_received_ = false;  // 允许下次再尝试
        return;
    }

    // 提取位置信息
    Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // 提取四元数并归一化
    Eigen::Quaterniond orientation(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
    orientation.normalize();
    
    // 打印接收到的位姿信息
    RCLCPP_INFO(this->get_logger(), "收到盒子中心点位姿 - 位置: [%f, %f, %f], 方向: [%f, %f, %f, %f]",
                position.x(), position.y(), position.z(),
                orientation.x(), orientation.y(), orientation.z(), orientation.w());
    
    // 进一步处理逻辑：
    // - 触发轨迹规划
    if (!trajectory_planned_) 
    {
        RCLCPP_INFO(this->get_logger(), "开始规划轨迹");

        Eigen::Quaterniond quaternion;
        Eigen::Vector3d tcpPos;
        quaternion = Eigen::Quaterniond(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
        
        tcpPos = Eigen::Vector3d(
            msg->pose.position.x + 0.075 + 0.1318,
            msg->pose.position.y,
            msg->pose.position.z);            

        // 获取参数
        std::string start_joints_str;       
        this->get_parameter("start_joint_positions", start_joints_str);
        // 打印参数
        RCLCPP_INFO(this->get_logger(), "原始起始关节角度字符串: %s", start_joints_str.c_str());
        // 解析起始关节位置参数
        Eigen::VectorXd jointAngles_start(7);
        parse_joint_positions(start_joints_str, jointAngles_start);       

        // 计算正向运动学
        Eigen::Matrix4d matrix_base2tcp_target;
        try {
            matrix_base2tcp_target = tcpPose2HomogenMatrix(quaternion, tcpPos);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "计算正向运动学时出错: %s", e.what());
            return;
        }
        
        // 调用重构后的sclerp_planner函数进行轨迹规划
        trajectory_msgs::msg::JointTrajectory joint_trajectory;
        sclerp_planner(jointAngles_start, matrix_base2tcp_target, joint_trajectory);
        
        // 标记轨迹已规划
        trajectory_planned_ = true;
        
        // 现在直接以固定周期执行轨迹发布（原trajectory_timer_callback中的逻辑）
        if (trajectory_planned_ && !joint_trajectory.points.empty()) 
        {
            // 获取轨迹执行频率参数
            int execution_rate;
            this->get_parameter("trajectory_execution_rate", execution_rate);
            
            RCLCPP_INFO(this->get_logger(), "开始发布轨迹，包含 %ld 个轨迹点，发布频率: %d Hz", 
                       joint_trajectory.points.size(), execution_rate);
            
            // 逐个发布轨迹点，保持固定周期
            for (const auto& point : joint_trajectory.points) {
                // 检查关节位置数据是否有效
                if (point.positions.size() < 7) {
                    RCLCPP_WARN(this->get_logger(), "轨迹点关节数据不足7个，跳过该点");
                    continue;
                }
                
                // 创建关节位置消息
                auto joint_msg = rm_ros_interfaces::msg::Jointpos();
                joint_msg.dof = 7;
                joint_msg.follow = true;
                joint_msg.expand = 0.0;
                
                // 填充关节位置数据
                joint_msg.joint.assign(point.positions.begin(), point.positions.begin() + 7);
                
                // 发布消息到机械臂驱动节点
                joint_pos_publisher_->publish(joint_msg);
                
                RCLCPP_DEBUG(this->get_logger(), "发布轨迹点: [%f, %f, %f, %f, %f, %f, %f]",
                            joint_msg.joint[0], joint_msg.joint[1], joint_msg.joint[2],
                            joint_msg.joint[3], joint_msg.joint[4], joint_msg.joint[5],
                            joint_msg.joint[6]);
                
                // 添加延迟以控制发布频率
                std::this_thread::sleep_for(std::chrono::microseconds(1000000 / execution_rate));
            }
            
            RCLCPP_INFO(this->get_logger(), "轨迹发布完成");
            
            // 激活吸盘
            std::string jodell_activate_cmd = "ros2 service call /right_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \""
                                "{}\"";
            call_service_sync(jodell_activate_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
            // 运动到交接位置
            std::vector<double> target_joint_position = {-86.658, 116.426, 12.293, -52.829, -12.231, 116.485, -95.181};
            rmRobotAPI_->rm_movej(target_joint_position, 50);  

            // 延时一段时间等待右臂到达交付点位
            std::this_thread::sleep_for(std::chrono::milliseconds(8000)); 

            std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \""
                                "{}\"";
            call_service_sync(jodell_deactivate_cmd);

            std::string jodell_CloseModbus_cmd = "ros2 service call /right_arm/close_modbus rm_ros_interfaces/srv/CloseModbus \""
                                "{}\"";
            call_service_sync(jodell_CloseModbus_cmd);
        }
    }

    first_received_ = false;
}
```

同时需要在类的私有成员变量中添加一个标志位来跟踪规划器初始化状态，并在[init_timer_callback()](javascript:void(0))中设置该标志：

```cpp
private:
    // 添加规划器初始化状态标志
    bool planner_initialized_ = false;
    
    // ... 其他成员变量
    
    // 在init_timer_callback中设置标志
    void init_timer_callback()
    {
        // 初始化机械臂
        initialize_rmRobot();

        // 初始化规划器
        initialize_planner();
        
        // 设置规划器初始化完成标志
        if (planner_interface_) {
            planner_initialized_ = true;
        }

        // 初始化电吸盘
        std::string jodell_initialize_cmd = "ros2 service call /right_arm/initialize_gripper rm_ros_interfaces/srv/InitializeGripper \""
            "{ip: '192.168.1.19', port: 8080, register_address: 1000, comm_port: 1, device_address: 9}\"";
        call_service_sync(jodell_initialize_cmd); 

        // 取消初始化定时器
        init_timer_->cancel();
    }
```

这样修改后，[box_centerPoint_callback](javascript:void(0))函数会检查规划器是否已初始化完成，只有在初始化完成后才会执行轨迹规划与发布。

> 从上述回答来看,在类的成员变量中添加规划器初始化状态标志 planner_initialized_ 只在init_timer_callback()中设置该标志, 在box_centerPoint_callback 中并没有检查该标志? 而是使用了已有的规划器指针 planner_interface_,具体入下:
>
> ```c++
>// 检查规划器是否已初始化
> if (!planner_interface_) {
>  RCLCPP_WARN(this->get_logger(), "规划器尚未初始化完成，无法执行轨迹规划");
>  first_received_ = false;  // 允许下次再尝试
>     return;
>    }
>    ```

您说得对，我的回答中确实存在不一致的地方。让我重新整理一下思路。

您指出的问题是正确的 - 我提到了添加`planner_initialized_`标志位，但在代码中却使用了`planner_interface_`指针来检查规划器状态。这里有两种方式可以实现：

**方案一：使用planner_interface_指针检查（推荐）**

这种方式直接检查规划器接口指针是否存在以及是否初始化完成，是比较直观的方法：

```cpp
/**
 * @brief 盒子中心点位姿回调函数
 * 
 * 处理从 /right_posedemo 话题接收到的盒子中心点位姿消息。
 * 
 * @param msg 接收到的 PoseStamped 消息
 */
void box_centerPoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if(!first_received_){
        first_received_ = true; 
    } else {
        return;
    }

    // 检查规划器是否已初始化
    if (!planner_interface_) {
        RCLCPP_WARN(this->get_logger(), "规划器尚未初始化完成，无法执行轨迹规划");
        first_received_ = false;  // 允许下次再尝试
        return;
    }

    // ... 其余代码保持不变 ...
}
```

**方案二：使用planner_initialized_标志位**

如果您更倾向于使用专门的标志位，那么需要在[init_timer_callback()](javascript:void(0))中设置该标志，并在[box_centerPoint_callback](javascript:void(0))中检查它：

```cpp
// 在类成员变量中添加
bool planner_initialized_ = false;

// 在init_timer_callback中设置标志
void init_timer_callback()
{
    // 初始化机械臂
    initialize_rmRobot();

    // 初始化规划器
    initialize_planner();
    
    // 设置规划器初始化完成标志
    if (planner_interface_) {
        planner_initialized_ = true;
    }

    // ... 其余代码 ...
}

// 在box_centerPoint_callback中检查标志
void box_centerPoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if(!first_received_){
        first_received_ = true; 
    } else {
        return;
    }

    // 检查规划器是否已初始化
    if (!planner_initialized_) {
        RCLCPP_WARN(this->get_logger(), "规划器尚未初始化完成，无法执行轨迹规划");
        first_received_ = false;  // 允许下次再尝试
        return;
    }

    // ... 其余代码保持不变 ...
}
```

实际上，第一种方法（使用`planner_interface_`指针）更加直接和可靠，因为它不仅检查了指针是否存在，还检查了规划器是否真正初始化完成，这种方式更符合实际需求

## 🎯 双臂节点重构

> 分析rm_sclerp_planner_node_leftArm.cpp 和 rm_sclerp_planner_node_rightArm.cpp 两个文件的结构模块和运行逻辑，
>
> 考虑到左右臂节点中涉及的话题，服务和动作的不同，
>
> 分析是否可以提取出相同的部分（比如类的定义，或是URDF文件加载与解析这类通用的功能模块）放入独立的头文件定义，和独立的源文件中实现？

当前左右臂节点的实现方式是将类定义和实现都放在了单个源文件中，而不是分离到头文件和源文件中。我们可以从这两个文件中提取出通用的部分，创建独立的头文件和源文件来提高代码复用性和可维护性。

### 分析左右臂节点的结构和运行逻辑

通过分析左右臂节点文件，我发现它们具有以下相似性：

1. **相同的功能模块**：
   - URDF文件加载与解析
   - 机械臂状态管理
   - 轨迹规划接口
   - 吸盘控制相关功能
   - 关节状态处理
   - 坐标变换函数
2. **不同的部分**：
   - 话题、服务、动作的命名空间（左臂 vs 右臂）
   - IP地址配置
   - 初始关节角度
   - 特定任务逻辑

### 提取通用功能的实现方案

我们可以创建以下通用模块：

#### 1. 通用机械臂控制器基类 (rm_arm_controller.h)

```cpp
#ifndef RM_ARM_CONTROLLER_H
#define RM_ARM_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <urdf/model.h>
#include <memory>
#include <mutex>
#include <array>

// 通用服务头文件
#include "rm_ros_interfaces/srv/initialize_gripper.hpp"
#include "rm_ros_interfaces/srv/activate_gripper.hpp"
#include "rm_ros_interfaces/srv/deactivate_gripper.hpp"
#include "rm_ros_interfaces/srv/close_modbus.hpp"
#include "rm_ros_interfaces/srv/get_paras_gripper.hpp"

namespace rm_motion_planner {

class RmArmController : public rclcpp::Node
{
protected:
    // 通用成员变量
    std::array<double, 7> current_joint_positions_;
    std::mutex joint_state_mutex_;
    
    // 吸盘服务客户端
    rclcpp::Client<rm_ros_interfaces::srv::InitializeGripper>::SharedPtr init_gripper_client_;
    rclcpp::Client<rm_ros_interfaces::srv::ActivateGripper>::SharedPtr activate_gripper_client_;
    rclcpp::Client<rm_ros_interfaces::srv::DeactivateGripper>::SharedPtr deactivate_gripper_client_;
    rclcpp::Client<rm_ros_interfaces::srv::CloseModbus>::SharedPtr closeModbus_gripper_client_;
    rclcpp::Client<rm_ros_interfaces::srv::GetParasGripper>::SharedPtr getParas_gripper_client_;

    // 通用函数
    bool getLinkNamesFromURDF(const std::string& urdf_path, std::string& base_link, std::string& tip_link);
    
    Eigen::Matrix4d tcpPose2HomogenMatrix(const Eigen::Quaterniond& quaternion, const Eigen::Vector3d& tcpPos);
    
    Eigen::Matrix4d tcpPose2HomogenMatrix(const Eigen::Matrix<double, 6, 1>& tcpPose);
    
    Eigen::Matrix3f eulerToRotationMatrix(const Eigen::Vector3f& euler);
    
    Eigen::Vector3f rotationMatrixToEuler(const Eigen::Matrix3f& R);
    
    void parse_joint_positions(const std::string& str, Eigen::VectorXd& joints);
    
    void call_jodell_service_sync(const std::string& cmd);
    
    void call_jodell_service_async(const std::string& cmd);

public:
    explicit RmArmController(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    virtual ~RmArmController() = default;
    
    // 关节状态回调（通用实现）
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

} // namespace rm_motion_planner

#endif // RM_ARM_CONTROLLER_H
```

#### 2. 通用机械臂控制器实现 (rm_arm_controller.cpp)

```cpp
#include "rm_arm_controller.h"
#include <fstream>
#include <thread>
#include <sstream>

namespace rm_motion_planner {

RmArmController::RmArmController(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options)
{
    // 初始化关节位置数组
    current_joint_positions_.fill(0.0);
}

void RmArmController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() >= 7) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (int i = 0; i < 7; i++) {
            current_joint_positions_[i] = msg->position[i];
        }
        RCLCPP_DEBUG(this->get_logger(), "Received joint state update");
    } else {
        RCLCPP_WARN(this->get_logger(), "Received joint state data with less than 7 joints");
    }
}

bool RmArmController::getLinkNamesFromURDF(const std::string& urdf_path, std::string& base_link, std::string& tip_link) {
    urdf::Model urdf_model;
    
    if (!urdf_model.initFile(urdf_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load URDF file: %s", urdf_path.c_str());
        return false;
    }
    
    base_link = urdf_model.getRoot()->name;
    
    std::string last_link = base_link;
    for (const auto& link : urdf_model.links_) {
        bool has_child = false;
        for (const auto& joint : urdf_model.joints_) {
            if (joint.second->parent_link_name == link.first) {
                has_child = true;
                break;
            }
        }
        if (!has_child) {
            last_link = link.first;
        }
    }
    tip_link = last_link;
    
    return true;
}

Eigen::Matrix4d RmArmController::tcpPose2HomogenMatrix(const Eigen::Quaterniond& quaternion, const Eigen::Vector3d& tcpPos)
{
    Eigen::Matrix4d homogenMatrix = Eigen::Matrix4d::Identity();
    
    homogenMatrix(0, 3) = tcpPos[0];
    homogenMatrix(1, 3) = tcpPos[1];
    homogenMatrix(2, 3) = tcpPos[2];
    
    Eigen::Matrix3d rotation = quaternion.normalized().toRotationMatrix();
    homogenMatrix.block<3, 3>(0, 0) = rotation;
    
    return homogenMatrix;
}

Eigen::Matrix4d RmArmController::tcpPose2HomogenMatrix(const Eigen::Matrix<double, 6, 1>& tcpPose)
{
    Eigen::Matrix4d homogenMatrix = Eigen::Matrix4d::Identity();
    
    homogenMatrix(0, 3) = tcpPose[0];
    homogenMatrix(1, 3) = tcpPose[1];
    homogenMatrix(2, 3) = tcpPose[2];
    
    double roll = tcpPose[3];
    double pitch = tcpPose[4];
    double yaw = tcpPose[5];
    
    Eigen::Matrix3d rotation;
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);
    
    rotation(0, 0) = cp * cy;
    rotation(0, 1) = -cr * sy + sr * sp * cy;
    rotation(0, 2) = sr * sy + cr * sp * cy;
    
    rotation(1, 0) = cp * sy;
    rotation(1, 1) = cr * cy + sr * sp * sy;
    rotation(1, 2) = -sr * cy + cr * sp * sy;
    
    rotation(2, 0) = -sp;
    rotation(2, 1) = sr * cp;
    rotation(2, 2) = cr * cp;
    
    homogenMatrix.block<3, 3>(0, 0) = rotation;
    
    return homogenMatrix;
}

Eigen::Matrix3f RmArmController::eulerToRotationMatrix(const Eigen::Vector3f& euler) {
    float roll  = euler[0];
    float pitch = euler[1];
    float yaw   = euler[2];
    
    Eigen::Matrix3f Rz, Ry, Rx;
    Rz = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    Ry = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());
    Rx = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
    
    return Rz * Ry * Rx;
}

Eigen::Vector3f RmArmController::rotationMatrixToEuler(const Eigen::Matrix3f& R) {
    float yaw, pitch, roll;
    
    pitch = atan2(-R(2,0), sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)));
    
    if (fabs(pitch - M_PI/2) < 1e-6) {
        yaw = 0;
        roll = yaw + atan2(R(0,1), R(1,1));
    } else if (fabs(pitch + M_PI/2) < 1e-6) {
        yaw = 0;
        roll = -yaw + atan2(-R(0,1), R(1,1));
    } else {
        yaw = atan2(R(1,0), R(0,0));
        roll = atan2(R(2,1), R(2,2));
    }
    
    return Eigen::Vector3f(roll, pitch, yaw);
}

void RmArmController::parse_joint_positions(const std::string& str, Eigen::VectorXd& joints)
{
    if (joints.size() != 7) {
        joints.resize(7);
    }
    
    std::stringstream ss(str);
    std::string item;
    int i = 0;
    while (std::getline(ss, item, ',') && i < joints.size()) {
        try {
            item.erase(0, item.find_first_not_of(' '));
            item.erase(item.find_last_not_of(' ') + 1);
            joints(i) = std::stod(item);
        } catch (const std::exception& e) {
            joints(i) = 0.0;
            RCLCPP_WARN(this->get_logger(), "Failed to parse joint value '%s', setting to 0.0", item.c_str());
        }
        i++;
    }
    
    while (i < joints.size()) {
        joints(i) = 0.0;
        i++;
    }
}

void RmArmController::call_jodell_service_sync(const std::string& cmd) {
    RCLCPP_INFO(this->get_logger(), "Executing command: %s", cmd.c_str());
    int result = system(cmd.c_str());
    if (result == 0) {
        RCLCPP_INFO(this->get_logger(), "Service call succeeded");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }
}

void RmArmController::call_jodell_service_async(const std::string& cmd) {
    RCLCPP_INFO(this->get_logger(), "Executing command: %s", cmd.c_str());
    
    std::thread([this, cmd]() {
        int result = system(cmd.c_str());
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "Service call succeeded");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }).detach();
}

} // namespace rm_motion_planner
```

#### 3. 吸盘控制器类 (gripper_controller.h)

```cpp
#ifndef GRIPPER_CONTROLLER_H
#define GRIPPER_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include "rm_ros_interfaces/srv/initialize_gripper.hpp"
#include "rm_ros_interfaces/srv/activate_gripper.hpp"
#include "rm_ros_interfaces/srv/deactivate_gripper.hpp"
#include "rm_ros_interfaces/srv/close_modbus.hpp"
#include "rm_ros_interfaces/srv/get_paras_gripper.hpp"

namespace rm_motion_planner {

enum class GripperCommand {
    Init,
    Open,
    Close,
    Disconnect,
    Paras
};

struct VacuumData {
    bool success;
    int channel1_vacuum;
    int channel2_vacuum;
    std::string message;
};

class GripperController {
private:
    rclcpp::Node::SharedPtr node_;
    
    rclcpp::Client<rm_ros_interfaces::srv::InitializeGripper>::SharedPtr init_gripper_client_;
    rclcpp::Client<rm_ros_interfaces::srv::ActivateGripper>::SharedPtr activate_gripper_client_;
    rclcpp::Client<rm_ros_interfaces::srv::DeactivateGripper>::SharedPtr deactivate_gripper_client_;
    rclcpp::Client<rm_ros_interfaces::srv::CloseModbus>::SharedPtr close_modbus_client_;
    rclcpp::Client<rm_ros_interfaces::srv::GetParasGripper>::SharedPtr getParas_gripper_client_;

    template<typename ServiceType>
    typename ServiceType::Response::SharedPtr call_gripper_service(
        typename rclcpp::Client<ServiceType>::SharedPtr client,
        const std::string& service_name,
        std::shared_ptr<typename ServiceType::Request> request = 
            std::make_shared<typename ServiceType::Request>());

public:
    explicit GripperController(rclcpp::Node::SharedPtr node);
    
    bool execute_gripper_command(GripperCommand command, 
                                const std::string& ip = "192.168.1.18",
                                int port = 8080,
                                int register_address = 1000,
                                int comm_port = 1,
                                int device_address = 9);
                                
    VacuumData execute_paras_command();
};

} // namespace rm_motion_planner

#endif // GRIPPER_CONTROLLER_H
```

#### 4. 左臂特定实现

 (rm_sclerp_planner_node_leftArm.cpp 简化版)

```cpp
#include "rm_arm_controller.h"
#include "gripper_controller.h"

class RmSclerpPlannerNodeLeft : public rm_motion_planner::RmArmController
{
public:
    explicit RmSclerpPlannerNodeLeft(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : RmArmController("rm_sclerp_planner_node_leftArm", options)
    {
        // 左臂特定初始化
        initialize_left_arm();
    }

private:
    std::unique_ptr<rm_motion_planner::GripperController> gripper_controller_;
    
    void initialize_left_arm() {
        // 左臂特定初始化逻辑
        gripper_controller_ = std::make_unique<rm_motion_planner::GripperController>(shared_from_this());
    }
    
    // 左臂特定功能函数
    // ...
};
```

### 优势分析

通过这种方式重构代码，我们可以获得以下优势：

1. **代码复用性**：通用功能只需实现一次，左右臂都可以使用
2. **维护性**：修改通用功能时只需修改一处
3. **可读性**：每个文件职责更清晰，代码更易理解
4. **扩展性**：添加新臂或新功能更容易
5. **测试性**：通用功能可以独立测试

### 实施建议

1. 首先创建通用模块（[rm_arm_controller.h](javascript:void(0))、[rm_arm_controller.cpp](javascript:void(0))、[gripper_controller.h](javascript:void(0))、gripper_controller.cpp）
2. 然后修改左右臂节点文件，继承通用基类
3. 将特定逻辑保留在各自的实现文件中

这种重构方式符合面向对象设计原则，提高了代码的模块化程度和可维护性。



## 🎯 Sclerp 规划器新增防碰撞与避障

### 原函数代码

规划器初始化成员函数

```c++
void initialize_planner() 
{
    // 功能逻辑 : 输入参数检查 -> 系统资源初始化 -> 规划器初始化

    // #region URDF 路径获取与检查
    // 获取urdf_path参数
    std::string urdf_path;
    this->get_parameter("urdf_path", urdf_path);
    RCLCPP_INFO(this->get_logger(), "尝试使用URDF路径: %s", urdf_path.c_str());

    // 检查是否提供了URDF路径
    if (urdf_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "未提供URDF路径参数。必须提供有效的URDF文件路径。");
        planner_interface_ = nullptr;
        return;
    }

    // 检查URDF文件是否存在
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.good()) {
        RCLCPP_ERROR(this->get_logger(), "指定的URDF文件不存在或无法访问: %s", urdf_path.c_str());
        planner_interface_ = nullptr;
        return;
    }
    urdf_file.close();
    RCLCPP_INFO(this->get_logger(), "URDF文件检查通过: %s", urdf_path.c_str());
    // #endregion


    // #region 初始化路径规划器（强制指定urdf_path)
    // 初始化链接名称
    std::string base_link = "base_link";
    std::string tip_link = "Link7";

    // 从URDF文件中获取实际的链接名称
    std::string actual_base_link, actual_tip_link;
    if (getLinkNamesFromURDF(urdf_path, actual_base_link, actual_tip_link)) {
        base_link = actual_base_link;
        tip_link = actual_tip_link;
        RCLCPP_INFO(this->get_logger(), "从 URDF 文件中获取链接名称: base='%s', tip='%s'", 
                    base_link.c_str(), tip_link.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "无法从 URDF 文件中获取链接名称，使用默认值: base='%s', tip='%s'", 
                    base_link.c_str(), tip_link.c_str());
    }

    // 创建ScLERP运动规划器实例
    try {
        RCLCPP_INFO(this->get_logger(), "使用指定的URDF文件: %s", urdf_path.c_str());

        planner_interface_ = std::make_unique<sclerp_interface::ScLERPInterface>(
            base_link, tip_link, shared_from_this(), urdf_path);
        // planner_interface_ = std::make_unique<sclerp_interface::ScLERPInterface>(
        //     base_link, tip_link, this->get_node_base_interface(), urdf_path);

        if (!planner_interface_) {
            RCLCPP_ERROR(this->get_logger(), "无法创建规划器接口实例");
        } else {
            RCLCPP_INFO(this->get_logger(), "成功创建规划器接口实例");
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "创建规划器接口时出错: %s", e.what());
        planner_interface_ = nullptr;
    }
    // #endregion
}
```

生成轨迹成员函数

```c++
bool sclerp_planner(const Eigen::VectorXd &jointAngles_start,
                    const Eigen::Matrix4d &matrix_base2tcp_target,
                    trajectory_msgs::msg::JointTrajectory &joint_trajectory )
{

    // 检查 planner_interface_ 是否已正确初始化
    if (!planner_interface_) {
        RCLCPP_ERROR(this->get_logger(), "%s sclerp_planner：规划器接口未初始化", WARN_ICON.c_str());
        return false;
    }

    // 使用solve方法生成轨迹数据
    bool result = false;     

    try {
        result = planner_interface_->solve(jointAngles_start, matrix_base2tcp_target, joint_trajectory);

        RCLCPP_INFO(this->get_logger(), "%s 规划器返回结果: %s", SIGNAL_ICON.c_str(), result ? "成功" : "失败");

        if(!result) {
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "%s %s 规划成功，轨迹点数量: %ld", SUCCESS_ICON.c_str(), DATA_ICON.c_str(), joint_trajectory.points.size());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "%s 调用规划器solve方法时出错: %s", ERROR_ICON.c_str(), e.what());
        return false;
    }

    if (result && !joint_trajectory.points.empty()) 
    {
        // RCLCPP_INFO(this->get_logger(), "规划成功，轨迹点数量: %ld", joint_trajectory.points.size());

        // 直接将规划器生成的轨迹点复制到 planned_trajectory_ 中
        planned_trajectory_.points.clear();
        planned_trajectory_.header = joint_trajectory.header;
        planned_trajectory_.joint_names = joint_trajectory.joint_names;

        // 复制轨迹点数据
        for (const auto& point : joint_trajectory.points) {
            trajectory_msgs::msg::JointTrajectoryPoint new_point;
            new_point.positions = point.positions;
            new_point.velocities = point.velocities;
            new_point.accelerations = point.accelerations;
            new_point.effort = point.effort;
            new_point.time_from_start = point.time_from_start;
            planned_trajectory_.points.push_back(new_point);
        }

        trajectory_point_index_ = 0;
        trajectory_executing_ = true;
        trajectory_planned_ = true;  // 标记轨迹已规划

    } else {
        RCLCPP_ERROR(this->get_logger(), "规划失败");

        // 打印规划失败时的详细信息
        RCLCPP_ERROR(this->get_logger(), "目标位姿矩阵 matrix_base2tcp_target:");
        for (int i = 0; i < 4; i++) {
            RCLCPP_ERROR(this->get_logger(), "  [%f, %f, %f, %f]", 
                        matrix_base2tcp_target(i,0), matrix_base2tcp_target(i,1), 
                        matrix_base2tcp_target(i,2), matrix_base2tcp_target(i,3));
        }
    }

}
```

### 需求与实现

> 分析若是将 src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_node_leftArm.cpp 中的 sclerp_planner 函数中的调用的函数接口 solve 
>
> ```c++
> result = planner_interface_->solve(jointAngles_start, matrix_base2tcp_target, joint_trajectory);
> ```
>
> 改为 third_party/sclerp_motion_planner/src/sclerp_interface.cpp 中如下成员函数solve：
>
> ```c++
> bool ScLERPInterface::solve(const Eigen::VectorXd &init_jnt_values,
>                        const Eigen::Matrix4d &g_f,
>                        const bool check_self_collision,
>                        const int num_links_ignore,
>                        const std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> &obstacles,
>                        const std::shared_ptr<CollisionUtils::ObstacleBase> &grasped_object,
>                        trajectory_msgs::msg::JointTrajectory &jnt_trajectory)
> ```
>
> - src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_node_leftArm.cpp 中的sclerp_planner 函数体该如何修改？
> - 除了修改 sclerp_planner函数体，节点源文件还需要进行哪些修改？
>
> （（只给出方案说明和具体的代码实现，不修改项目代码）

#### Update1：

如果要将 [rm_sclerp_planner_node_leftArm.cpp](javascript:void(0)) 中的 [sclerp_planner](javascript:void(0)) 函数中的函数接口 [solve](javascript:void(0)) 改为使用 third_party/sclerp_motion_planner/src/sclerp_interface.cpp 中带有碰撞检测参数的 [solve](javascript:void(0)) 函数，需要进行以下修改：

##### 1. 修改 [sclerp_planner](javascript:void(0)) 函数体

需要修改 [sclerp_planner](javascript:void(0)) 函数以适应新的 [solve](javascript:void(0)) 函数签名。新的函数签名包含额外的参数：是否检查自碰撞、忽略链接数、障碍物列表和被抓取物体。

```cpp
bool RmSclerpPlannerNode1::sclerp_planner(const Eigen::VectorXd &jointAngles_start,
                        const Eigen::Matrix4d &matrix_base2tcp_target,
                        trajectory_msgs::msg::JointTrajectory &joint_trajectory )
    {

        // 检查planner_interface_是否已正确初始化
        if (!planner_interface_) {
            RCLCPP_ERROR(this->get_logger(), "规划器接口未初始化");
            return false;
        }

        // 使用solve方法生成轨迹数据，新增碰撞检测相关参数
        bool result = false;
    
        try {
            // 新的solve函数参数包括碰撞检测相关参数
            bool check_self_collision = true;  // 启用自碰撞检测
            int num_links_ignore = 0;  // 不忽略任何链接
            std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> obstacles;  // 空障碍物列表
            std::shared_ptr<CollisionUtils::ObstacleBase> grasped_object = nullptr;  // 无被抓取物体
            
            result = planner_interface_->solve(
                			jointAngles_start, 
                			matrix_base2tcp_target, 
                            check_self_collision,
                            num_links_ignore,
                            obstacles, 
                			grasped_object,
                            joint_trajectory);
            
            RCLCPP_INFO(this->get_logger(), "规划器返回结果: %s", result ? "成功" : "失败");

            if(!result) {
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "规划成功，轨迹点数量: %ld", joint_trajectory.points.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "调用规划器solve方法时出错: %s", e.what());
            return false;
        }
        
        if (result && !joint_trajectory.points.empty()) 
        {
            // RCLCPP_INFO(this->get_logger(), "规划成功，轨迹点数量: %ld", joint_trajectory.points.size());
            
            // 直接将规划器生成的轨迹点复制到 planned_trajectory_ 中
            planned_trajectory_.points.clear();
            planned_trajectory_.header = joint_trajectory.header;
            planned_trajectory_.joint_names = joint_trajectory.joint_names;
            
            // 复制轨迹点数据
            for (const auto& point : joint_trajectory.points) {
                trajectory_msgs::msg::JointTrajectoryPoint new_point;
                new_point.positions = point.positions;
                new_point.velocities = point.velocities;
                new_point.accelerations = point.accelerations;
                new_point.effort = point.effort;
                new_point.time_from_start = point.time_from_start;
                planned_trajectory_.points.push_back(new_point);
            }
                       
            trajectory_point_index_ = 0;
            trajectory_executing_ = true;
            trajectory_planned_ = true;  // 标记轨迹已规划
            
        } else {
            RCLCPP_ERROR(this->get_logger(), "规划失败");
        
            // 打印规划失败时的详细信息
            RCLCPP_ERROR(this->get_logger(), "目标位姿矩阵 matrix_base2tcp_target:");
            for (int i = 0; i < 4; i++) {
                RCLCPP_ERROR(this->get_logger(), "  [%f, %f, %f, %f]", 
                            matrix_base2tcp_target(i,0), matrix_base2tcp_target(i,1), 
                            matrix_base2tcp_target(i,2), matrix_base2tcp_target(i,3));
            }
        }
        
        return result; // 添加返回值
    }
```

##### 2. 节点源文件中需要进行的其他修改

除了修改 [sclerp_planner](javascript:void(0)) 函数体，还需要在节点源文件中进行以下修改：

###### 2.1 添加必要的头文件包含

在文件开头添加碰撞检测相关的头文件：

```cpp
// 添加碰撞检测相关头文件
#include "kinlib/collision_utils.h"
```

###### 2.2 修改 ScLERPInterface 初始化代码

如果要使用碰撞检测功能，需要在初始化 [ScLERPInterface](javascript:void(0)) 时提供 STL 模型路径。需要修改 [initialize_planner](javascript:void(0)) 函数：

```cpp
void RmSclerpPlannerNode1::initialize_planner() 
{
    // 功能逻辑 : 输入参数检查 -> 系统资源初始化 -> 规划器初始化

    // #region URDF 路径获取与检查
    // 获取urdf_path参数
    std::string urdf_path;
    this->get_parameter("urdf_path", urdf_path);
    RCLCPP_INFO(this->get_logger(), "尝试使用URDF路径: %s", urdf_path.c_str());
    
    // 检查是否提供了URDF路径
    if (urdf_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "未提供URDF路径参数。必须提供有效的URDF文件路径。");
        planner_interface_ = nullptr;
        return;
    }
    
    // 检查URDF文件是否存在
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.good()) {
        RCLCPP_ERROR(this->get_logger(), "指定的URDF文件不存在或无法访问: %s", urdf_path.c_str());
        planner_interface_ = nullptr;
        return;
    }
    urdf_file.close();
    RCLCPP_INFO(this->get_logger(), "URDF文件检查通过: %s", urdf_path.c_str());
    // #endregion
    
    // #region 初始化STL模型路径（用于碰撞检测）
    // 定义STL模型路径向量
    std::vector<std::string> stl_paths;
    // 这里需要根据实际STL模型路径进行设置
    // 示例：
    // stl_paths.push_back("/path/to/link1.stl");
    // stl_paths.push_back("/path/to/link2.stl");
    // ...
    // #endregion
    
    
    // #region 初始化路径规划器（强制指定urdf_path)
    // 初始化链接名称
    std::string base_link = "base_link";
    std::string tip_link = "Link7";
    
    // 从URDF文件中获取实际的链接名称
    std::string actual_base_link, actual_tip_link;
    if (getLinkNamesFromURDF(urdf_path, actual_base_link, actual_tip_link)) {
        base_link = actual_base_link;
        tip_link = actual_tip_link;
        RCLCPP_INFO(this->get_logger(), "从 URDF 文件中获取链接名称: base='%s', tip='%s'", 
                    base_link.c_str(), tip_link.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "无法从 URDF 文件中获取链接名称，使用默认值: base='%s', tip='%s'", 
                    base_link.c_str(), tip_link.c_str());
    }

    // 创建ScLERP运动规划器实例（包含STL路径参数）
    try {
        RCLCPP_INFO(this->get_logger(), "使用指定的URDF文件: %s", urdf_path.c_str());

        planner_interface_ = std::make_unique<sclerp_interface::ScLERPInterface>(
            base_link, tip_link, shared_from_this(), urdf_path, stl_paths);
        
        if (!planner_interface_) {
            RCLCPP_ERROR(this->get_logger(), "无法创建规划器接口实例");
        } else {
            RCLCPP_INFO(this->get_logger(), "成功创建规划器接口实例");
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "创建规划器接口时出错: %s", e.what());
        planner_interface_ = nullptr;
    }
}
```

##### 总结

为了使用带有碰撞检测功能的新版 [solve](javascript:void(0)) 函数，需要进行以下关键修改：

1. 修改 [sclerp_planner](javascript:void(0)) 函数，添加新的参数并调用新的 [solve](javascript:void(0)) 函数
2. 添加必要的头文件包含，特别是碰撞检测相关的头文件
3. 修改 [ScLERPInterface](javascript:void(0)) 的初始化代码，提供 STL 模型路径以支持碰撞检测

这些修改将使系统能够使用具有碰撞检测功能的增强版路径规划器，提高机器人运动的安全性。



#### Update2

> 上述代码实现中的如下语句：
>
> ```
>     // #region 初始化STL模型路径（用于碰撞检测）
>     // 定义STL模型路径向量
>     std::vector<std::string> stl_paths;
>     // 这里需要根据实际STL模型路径进行设置
>     // 示例：
>     // stl_paths.push_back("/path/to/link1.stl");
>     // stl_paths.push_back("/path/to/link2.stl");
>     // ...
>     // #endregion
> ```
>
> 其中的STL 模型文件的后缀是 stl，是否应该修改为 STL,？因为实际的STL模型所处目录`/home/byd/Documents/Projects/DualArmProjects/realmanRobot/联调_20251028/ros2_rm_ws/src/ros2_rm_robot-humble/rm_description/meshes/rm_75_arm`下的文件后缀为 .STL

在大多数操作系统中，包括Linux（用户正在使用的系统），文件系统是区分大小写的。这意味着`.stl`和`.STL`被视为不同的文件扩展名。

在Linux系统中，文件系统是区分大小写的，这意味着`link1.stl`和[link1.STL](javascript:void(0))是两个不同的文件。如果代码中使用了错误的大小写，程序将无法找到这些文件。

所以，如果要引用实际的STL模型文件，应该使用正确的大小写：

```cpp
stl_paths.push_back("/path/to/link1.STL");
```

而不是：

```cpp
stl_paths.push_back("/path/to/link1.stl");
```

在实际使用中，路径应该匹配实际文件系统中的文件名，包括正确的大小写。对于用户系统中的文件，正确的路径应该是：

```cpp
stl_paths.push_back("/home/byd/Documents/Projects/DualArmProjects/realmanRobot/联调_20251028/ros2_rm_ws/src/ros2_rm_robot-humble/rm_description/meshes/rm_75_arm/base_link.STL");
stl_paths.push_back("/home/byd/Documents/Projects/DualArmProjects/realmanRobot/联调_20251028/ros2_rm_ws/src/ros2_rm_robot-humble/rm_description/meshes/rm_75_arm/link1.STL");
// ... 其他STL文件
```

总结：是的，应该使用`.STL`而不是`.stl`，因为实际文件的扩展名是大写的`.STL`，特别是在Linux系统中，文件系统区分大小写。

#### Update3

>  STL 模型所在的目录名称（/home/byd/Documents/Projects/DualArmProjects/realmanRobot/联调_20251028/ros2_rm_ws/src/ros2_rm_robot-humble/rm_description/meshes/rm_75_arm）过长，在Update1 中的的下述代码实现中需要引用多次目录，显得可读性差，如何解决该问题
>
> ```
> // #region 初始化STL模型路径（用于碰撞检测）
> // 定义STL模型路径向量
> std::vector<std::string> stl_paths;
> // 这里需要根据实际STL模型路径进行设置
> // 示例：
> // stl_paths.push_back("/path/to/link1.stl");
> // stl_paths.push_back("/path/to/link2.stl");
> // ...
> // #endregion
> ```
>
> 

针对STL模型目录路径过长导致代码可读性差的问题，有几种解决方案可以改善代码的可读性和维护性：

##### 方案1：使用相对路径和参数配置

可以将基本路径设置为ROS2参数，然后在代码中使用相对路径：

```cpp
// 在参数声明部分添加
this->declare_parameter<std::string>("meshes_base_path", 
    "/home/byd/Documents/Projects/DualArmProjects/realmanRobot/联调_20251028/ros2_rm_ws/src/ros2_rm_robot-humble/rm_description/meshes");

// 在initialize_planner函数中获取参数
std::string meshes_base_path;
this->get_parameter("meshes_base_path", meshes_base_path);
std::string arm_meshes_path = meshes_base_path + "/rm_75_arm";

// 然后使用相对路径添加STL文件
std::vector<std::string> stl_paths;
stl_paths.push_back(arm_meshes_path + "/base_link.STL");
stl_paths.push_back(arm_meshes_path + "/link1.STL");
stl_paths.push_back(arm_meshes_path + "/link2.STL");
// ... 其他链接
```

##### 方案2：使用常量或宏定义

定义一个常量来存储基本路径：

```cpp
// 在类定义中添加常量成员
const std::string MESHES_BASE_PATH = "/home/byd/Documents/Projects/DualArmProjects/realmanRobot/联调_20251028/ros2_rm_ws/src/ros2_rm_robot-humble/rm_description/meshes/rm_75_arm";

// 在initialize_planner函数中使用
stl_paths.push_back(MESHES_BASE_PATH + "/base_link.STL");
stl_paths.push_back(MESHES_BASE_PATH + "/link1.STL");
stl_paths.push_back(MESHES_BASE_PATH + "/link2.STL");
// ... 其他链接
```

##### 方案3：使用环境变量

通过环境变量来设置基本路径，提高可移植性：

```cpp
// 获取环境变量中的路径
const char* meshes_path_env = std::getenv("RM_ROBOT_MESHES_PATH");
std::string meshes_base_path = meshes_path_env ? 
    std::string(meshes_path_env) : 
    "/home/byd/Documents/Projects/DualArmProjects/realmanRobot/联调_20251028/ros2_rm_ws/src/ros2_rm_robot-humble/rm_description/meshes/rm_75_arm";

// 使用该路径
std::vector<std::string> stl_paths;
stl_paths.push_back(meshes_base_path + "/base_link.STL");
stl_paths.push_back(meshes_base_path + "/link1.STL");
// ... 其他链接
```

##### 方案4：使用ROS2包资源查找机制

利用ROS2的资源查找机制，通过包名获取路径：

```cpp
// 使用ament_index_cpp查找包路径
#include "ament_index_cpp/get_package_share_directory.hpp"

// 在initialize_planner函数中
std::string rm_description_path = ament_index_cpp::get_package_share_directory("rm_description");
std::string meshes_base_path = rm_description_path + "/meshes/rm_75_arm";

// 使用该路径
std::vector<std::string> stl_paths;
stl_paths.push_back(meshes_base_path + "/base_link.STL");
stl_paths.push_back(meshes_base_path + "/link1.STL");
// ... 其他链接
```

##### 推荐方案

我推荐使用**方案4**，因为它具有以下优点：

1. 不依赖硬编码的绝对路径，提高代码的可移植性
2. 遵循ROS2的最佳实践
3. 路径会自动适应不同的安装环境
4. 更容易维护和部署

这样修改后的代码会更加清晰和可维护：

```cpp
void RmSclerpPlannerNode1::initialize_planner() 
{
    // ... existing code ...

    // #region 初始化STL模型路径（用于碰撞检测）
    // 定义STL模型路径向量
    std::vector<std::string> stl_paths;
    
    // 使用ROS2包查找机制获取路径
    std::string rm_description_path = ament_index_cpp::get_package_share_directory("rm_description");
    std::string meshes_base_path = rm_description_path + "/meshes/rm_75_arm";
    
    // 添加STL模型文件
    stl_paths.push_back(meshes_base_path + "/base_link.STL");
    stl_paths.push_back(meshes_base_path + "/link1.STL");
    stl_paths.push_back(meshes_base_path + "/link2.STL");
    stl_paths.push_back(meshes_base_path + "/link3.STL");
    stl_paths.push_back(meshes_base_path + "/link4.STL");
    stl_paths.push_back(meshes_base_path + "/link5.STL");
    stl_paths.push_back(meshes_base_path + "/link6.STL");
    stl_paths.push_back(meshes_base_path + "/link7.STL");
    // #endregion
    
    // ... existing code ...
}
```

这种方案避免了硬编码长路径，提高了代码的可读性和可移植性。



#### QNA1：如何定义障碍物

> 如下代码中的 obstacles 和 grasped_object ，在存在障碍物和抓取物体时，该如何赋值
>
> ```
> std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> obstacles;  // 空障碍物列表
> std::shared_ptr<CollisionUtils::ObstacleBase> grasped_object = nullptr;  // 无被抓取物体
> ```



在问答01中提到的代码里，[obstacles](javascript:void(0))和[grasped_object](javascript:void(0))变量在存在障碍物和抓取物体时需要根据实际情况进行赋值。下面我来详细说明如何为这些变量赋值：

##### 1. 障碍物（obstacles）赋值

要添加障碍物到[obstacles](javascript:void(0))向量中，需要创建CollisionUtils::ObstacleBase的派生类实例。根据常见的障碍物类型，可能包括：

###### 球形障碍物

```cpp
// 创建球形障碍物
auto sphere_obstacle = std::make_shared<CollisionUtils::Sphere>(CollisionUtils::Sphere(
    Eigen::Vector3d(1.0, 0.5, 0.3),  // 球心位置 (x, y, z)
    0.2));                           // 球半径
obstacles.push_back(sphere_obstacle);
```

###### 立方体障碍物

```cpp
// 创建立方体障碍物
auto box_obstacle = std::make_shared<CollisionUtils::Box>(CollisionUtils::Box(
    Eigen::Vector3d(1.5, 1.0, 0.5),  // 中心位置 (x, y, z)
    Eigen::Vector3d(0.3, 0.4, 0.2))); // 尺寸 (length, width, height)
obstacles.push_back(box_obstacle);
```

###### 圆柱体障碍物

```cpp
// 创建圆柱体障碍物
auto cylinder_obstacle = std::make_shared<CollisionUtils::Cylinder>(CollisionUtils::Cylinder(
    Eigen::Vector3d(0.5, 1.5, 0.4),  // 中心位置 (x, y, z)
    0.15,                            // 半径
    0.5));                           // 高度
obstacles.push_back(cylinder_obstacle);
```

##### 2. 抓取物体（grasped_object）赋值

对于被抓取的物体，同样需要创建合适的障碍物对象并赋值给[grasped_object](javascript:void(0))：

```cpp
// 创建被抓取的物体（例如一个盒子）
grasped_object = std::make_shared<CollisionUtils::Box>(CollisionUtils::Box(
    Eigen::Vector3d(0.0, 0.0, 0.0),   // 相对于末端执行器的初始位置
    Eigen::Vector3d(0.1, 0.1, 0.2))); // 物体尺寸
```

##### 3. 在sclerp_planner函数中的完整应用示例

修改后的[sclerp_planner](javascript:void(0))函数可能如下所示：

```cpp
bool RmSclerpPlannerNode1::sclerp_planner(const Eigen::VectorXd &jointAngles_start,
                        const Eigen::Matrix4d &matrix_base2tcp_target,
                        trajectory_msgs::msg::JointTrajectory &joint_trajectory )
    {

        // 检查planner_interface_是否已正确初始化
        if (!planner_interface_) {
            RCLCPP_ERROR(this->get_logger(), "规划器接口未初始化");
            return false;
        }

        // 使用solve方法生成轨迹数据，新增碰撞检测相关参数
        bool result = false;
    
        try {
            // 碰撞检测相关参数
            bool check_self_collision = true;  // 启用自碰撞检测
            int num_links_ignore = 0;  // 不忽略任何链接
            
            // 定义障碍物列表
            std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> obstacles;
            
            // 添加环境中的障碍物（示例）
            // 添加一个球形障碍物
            auto sphere_obstacle = std::make_shared<CollisionUtils::Sphere>(CollisionUtils::Sphere(
                Eigen::Vector3d(0.5, 0.3, 0.4),  // 球心位置
                0.1));                           // 球半径
            obstacles.push_back(sphere_obstacle);
            
            // 添加一个立方体障碍物
            auto box_obstacle = std::make_shared<CollisionUtils::Box>(CollisionUtils::Box(
                Eigen::Vector3d(0.6, -0.2, 0.3),    // 中心位置
                Eigen::Vector3d(0.2, 0.3, 0.4)));   // 尺寸
            obstacles.push_back(box_obstacle);
            
            // 定义被抓取的物体（如果有的话）
            std::shared_ptr<CollisionUtils::ObstacleBase> grasped_object = nullptr;
            // 如果机器人正在抓取物体，则创建被抓取的物体
            // grasped_object = std::make_shared<CollisionUtils::Box>(CollisionUtils::Box(
            //     Eigen::Vector3d(0.0, 0.0, 0.0),     // 相对于末端执行器的位置
            //     Eigen::Vector3d(0.1, 0.1, 0.2)));   // 物体尺寸
            
            result = planner_interface_->solve(jointAngles_start, matrix_base2tcp_target, 
                                              check_self_collision, num_links_ignore,
                                              obstacles, grasped_object,
                                              joint_trajectory);
            
            RCLCPP_INFO(this->get_logger(), "规划器返回结果: %s", result ? "成功" : "失败");

            if(!result) {
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "规划成功，轨迹点数量: %ld", joint_trajectory.points.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "调用规划器solve方法时出错: %s", e.what());
            return false;
        }
        
        if (result && !joint_trajectory.points.empty()) 
        {
            // ... existing code ...
        } else {
            RCLCPP_ERROR(this->get_logger(), "规划失败");
        
            // ... existing code ...
        }
        
        return result;
    }
```

##### 4. 动态障碍物管理

在实际应用中，障碍物信息可能来自传感器数据或环境描述，可以设计专门的函数来管理障碍物：

```cpp
// 添加专门的函数来更新障碍物信息
void RmSclerpPlannerNode1::updateObstacles() {
    // 清空现有障碍物
    obstacles_.clear();
    
    // 根据传感器数据或环境描述添加障碍物
    // 这里可以从订阅的话题或其他传感器数据中获取障碍物信息
    
    // 示例：添加工作台作为静态障碍物
    auto table = std::make_shared<CollisionUtils::Box>(CollisionUtils::Box(
        Eigen::Vector3d(0.8, 0.0, 0.2),     // 工作台中心位置
        Eigen::Vector3d(0.6, 1.2, 0.4)));   // 工作台尺寸
    obstacles_.push_back(table);
    
    // 示例：添加另一个机械臂作为动态障碍物
    // 这需要从另一个机械臂的状态信息中获取当前关节角度，并计算各连杆的位置
}

// 在类定义中添加成员变量
std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> obstacles_;
std::shared_ptr<CollisionUtils::ObstacleBase> grasped_object_;
```

这样，在[sclerp_planner](javascript:void(0))函数中就可以直接使用这些成员变量：

```cpp
result = planner_interface_->solve(jointAngles_start, matrix_base2tcp_target, 
                                  check_self_collision, num_links_ignore,
                                  obstacles_,  // 使用成员变量
                                  grasped_object_,  // 使用成员变量
                                  joint_trajectory);
```

通过这些方法，可以根据实际环境中的障碍物和抓取情况为[obstacles](javascript:void(0))和[grasped_object](javascript:void(0))变量赋值，实现更安全的路径规划。

#### QNA2：动态障碍物轨迹预测和实时避障问题

> 针对问答04中实现的“动态障碍物管理”代码实现中的以另一条机械臂作为动态障碍物的情况，记时刻 t1 获取到作为动态障碍物的机械臂的位置信息，时刻 t2 计算出一条无碰撞的路径发给目标机械臂去执行。从时刻 t1 开始到目标机械臂执行基于 t1 时刻障碍物机械臂位置信息计算的无碰撞轨迹过程中，若是作为动态障碍物的机械臂的也处于运动中，那么时刻 t2 计算出一条无碰撞的路径存在滞后性，可能无效，即无法保证目标机械臂不与障碍物机械臂产生碰撞。
>
> - 检索网络资源，查找解决上述问题的成熟通用的方案？
> - 解决方案背后的原理机制是怎样的？
> - 有无开源的算法或是库代码？若是存在，请结合代码实现详细阐述方法原



##### 1. 主要解决方案

###### Temporal Probabilistic Roadmap (T-PRM)

T-PRM是一种基于时间扩展的经典PRM算法，专门用于动态环境中的路径规划。它的核心思想是将时间维度整合到传统的PRM中，创建一个时空图结构。

**原理机制**：

- 在传统的PRM基础上增加时间维度，形成时空路线图
- 动态障碍物被视为在时间维度上移动的障碍物
- 使用时间感知的A*算法在时空图中搜索路径
- 路径不仅考虑空间避障，还考虑时间避障

**优势**：

- 能够处理已知轨迹的动态障碍物
- 保持PRM的所有优点（如多查询能力、路径平滑等）
- 计算成本增加较小

###### Intent Prediction-Driven Model Predictive Control (Intent-MPC)

Intent-MPC是一种结合意图预测和模型预测控制的方法，用于处理动态障碍物。

**原理机制**：

- 基于历史轨迹数据预测动态障碍物的运动意图
- 定义多种可能的运动模式（如前进、转弯、停止）
- 估计每种意图的概率分布
- 使用模型预测控制(MPC)优化机器人轨迹，考虑预测的障碍物意图

**优势**：

- 能够处理多模态障碍物行为
- 实时性能好
- 可以处理不确定性

###### 其他方案

1. **Velocity Obstacles (VO) 和其扩展形式**：计算障碍物的未来可能位置，避免进入这些区域
2. **Dynamic Window Approach (DWA) 的扩展**：结合动态障碍物预测的局部规划方法
3. **基于学习的方法**：使用神经网络预测障碍物轨迹并规划路径

##### 2. 开源算法和库

###### T-PRM

T-PRM算法在GitHub上有开源实现：[VIS4ROB-lab/t_prm](https://github.com/VIS4ROB-lab/t_prm)

以下是使用T-PRM的基本示例代码：

```cpp
#include <tprm/obstacle_impl.h>
#include <tprm/temporal_prm.h>

// 设置机器人速度
tprm::HolonomicRobot::movement_speed = 0.1; // m/s

// 创建T-PRM对象
tprm::TemporalPRM tprm;

// 添加动态障碍物
// 障碍物在(1,1,1)位置，以(-0.1,-0.1,-0.1)速度移动，半径0.25m
tprm.addDynamicObstacle(
    std::make_shared<tprm::DynamicSphereObstacle>(
        tprm::Vector3d::Constant(1.), 
        tprm::Vector3d::Constant(-0.1), 
        0.25));

// 采样并构建路线图
tprm.placeSamples(150);
tprm.buildPRM(0.25);

// 查询路径
auto path = tprm.getShortestPath(
    tprm::Vector3d(0, 0, 0),  // 起点
    tprm::Vector3d(1., 1., 1.), // 终点
    0.5); // 起始时间

if (path.empty()) {
    std::cout << "No path found" << std::endl;
} else {
    for (auto& node : path)
        std::cout << "Node: " << node.position.transpose() 
                  << " at time " << node.time << std::endl;
}
```

###### MRPT (Mobile Robot Programming Toolkit)

MRPT是一个功能丰富的移动机器人编程工具包，包含运动规划、SLAM、计算机视觉等模块。

###### 其他相关库

1. **OMPL (Open Motion Planning Library)**：虽然主要用于静态环境，但有一些扩展可以处理动态障碍物
2. **ROS Navigation Stack**：包含一些处理动态障碍物的插件和扩展

##### 3. 方法原理详细阐述

###### T-PRM工作原理

1. **时空采样**：在配置空间和时间维度上进行采样
2. **碰撞检查**：检查采样点在不同时间点是否与动态障碍物冲突
3. **图构建**：连接不同时空位置的无冲突节点
4. **路径搜索**：使用时间感知的A*算法搜索时空路径

###### Intent-MPC工作原理

1. **意图识别**：基于历史轨迹数据，识别障碍物可能的运动意图
2. **概率估计**：计算每种意图的概率分布
3. **轨迹优化**：使用MPC优化机器人轨迹，考虑各种可能的障碍物意图
4. **实时更新**：随着新观测数据的到达，更新预测并重新优化

##### 4. 实际应用建议

针对您提到的双臂机器人场景，建议采用以下策略：

1. **实时感知**：通过传感器实时获取另一只机械臂的位置和速度
2. **短期预测**：使用简单的运动模型（如恒速模型）预测短期内的位置
3. **轨迹重规划**：以较高频率（如10Hz以上）重新规划轨迹
4. **安全边界**：在预测位置周围增加安全边界
5. **紧急停止**：设计紧急停止机制，当预测不确定性较大时启用

这些方法可以有效缓解由于感知延迟和执行滞后导致的碰撞风险，提高双臂协作的安全性。



## 🎯 节点代码简化：使用标准库函数替代自定义函数

### 开发日期

2025/10/31

### 问题背景

双臂节点的动作执行函数 execute_grasp_boxexecute_grasp_box 的代码实现过于繁多，复杂，臃肿，可读性差，维护性差，不够健壮。

### 改进点1：ROS2 参数获取片段

#### 问题分析



```c++
void execute_grasp_box(const std::shared_ptr<GraspBoxHandle> goal_handle)
{
    //... 其他代码 ...
    if (!trajectory_planned_) 
    {
        // ... 其他代码 ...
        
        RCLCPP_INFO(this->get_logger(), "%s 开始规划轨迹", WORKING_ICON.c_str());

        {
        // 获取参数
        std::string start_joints_str;       
        this->get_parameter("start_joint_positions", start_joints_str);
        // 打印参数
        RCLCPP_INFO(this->get_logger(), "%s 原始起始关节角度字符串: %s", DATA_ICON.c_str(), start_joints_str.c_str());
        // 解析起始关节位置参数
        Eigen::VectorXd jointAngles_start(7);
        parse_joint_positions(start_joints_str, jointAngles_start);       
        }
        
        // ... 其他代码 ...
    }
            
    // ... 其他代码 ...
}
```

- 代码语句数量多
- 需要调用额外定义的函数接口，导致整体代码量繁多

#### 解决方案

> 如下代码是从参数服务器中获取机械臂的初始关节位置数据的具体实现：
>
> ```c++
> {
> // 获取参数
> std::string start_joints_str;       
> this->get_parameter("start_joint_positions", start_joints_str);
> // 打印参数
> RCLCPP_INFO(this->get_logger(), "%s 原始起始关节角度字符串: %s", DATA_ICON.c_str(), start_joints_str.c_str());
> // 解析起始关节位置参数
> Eigen::VectorXd jointAngles_start(7);
> parse_joint_positions(start_joints_str, jointAngles_start);       
> }
> ```
>
> - 这是否是ROS2开发中的标准实践方式，即需要从参数服务器中获取数据时，需要解析字符串为其他类型数据（浮点数或是数组）
> - 是否存在根加简洁的方式实现上述功能目标？



在ROS 2中，从参数服务器获取参数并解析为特定数据类型是一种常见做法，但您的代码可以进行一些优化。

##### 1. 这是否是ROS2开发中的标准实践方式？

是的，您展示的代码确实是ROS 2中处理参数的标准方式之一。从参数服务器获取参数时，通常需要进行类型转换，因为参数在存储和传输过程中可能被序列化为字符串。ROS 2提供了几种参数类型，但对于数组或向量类型，通常需要以字符串形式存储并在代码中解析。

##### 2. 是否存在更加简洁的方式实现上述功能目标？

是的，有几种更简洁的方式可以实现相同的功能：

###### 方式一：使用double数组参数类型（推荐）

可以直接声明和获取double数组类型的参数，而不需要通过字符串解析：

```cpp
// 声明参数
this->declare_parameter("start_joint_positions", std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

// 获取参数
std::vector<double> joint_positions = this->get_parameter("start_joint_positions").as_double_array();
Eigen::Map<Eigen::VectorXd> jointAngles_start(joint_positions.data(), joint_positions.size());
```

###### 方式二：改进现有的字符串解析方法

如果您仍想使用字符串方式，可以优化[parse_joint_positions](javascript:void(0))函数：

```cpp
std::vector<double> parse_joint_positions(const std::string& str) {
    std::vector<double> joints;
    std::stringstream ss(str);
    std::string item;
    
    while (std::getline(ss, item, ',')) {
        // 清除空格
        item.erase(0, item.find_first_not_of(' '));
        item.erase(item.find_last_not_of(' ') + 1);
        
        try {
            joints.push_back(std::stod(item));
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("parser"), "无法解析关节值 '%s'，跳过", item.c_str());
        }
    }
    
    return joints;
}

// 使用方法：
std::string start_joints_str = this->get_parameter("start_joint_positions").as_string();
std::vector<double> joint_values = parse_joint_positions(start_joints_str);
Eigen::Map<Eigen::VectorXd> jointAngles_start(joint_values.data(), joint_values.size());
```

###### 方式三：使用更现代的C++方法

使用`std::regex`或`std::stringstream`进行更简洁的解析：

```cpp
std::vector<double> parse_string_to_vector(const std::string& str) {
    std::vector<double> result;
    std::regex reg(R"([-+]?\d*\.?\d+([eE][-+]?\d+)?)");  // 匹配浮点数
    std::sregex_iterator iter(str.begin(), str.end(), reg);
    std::sregex_iterator end;
    
    for (; iter != end; ++iter) {
        result.push_back(std::stod(iter->str()));
    }
    
    return result;
}
```

##### 建议

对于您的使用场景，我建议采用**方式一**，即直接使用`std::vector<double>`作为参数类型，这样可以避免字符串解析的复杂性，并且代码更清晰、更安全。

这种修改方式不仅简化了代码，还提高了代码的可读性和健壮性，避免了字符串解析可能带来的错误。

#### 方案改进

> 针对上述内容中的方式一代码实现：
>
> - 声明语句中的 `std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})` 是否可以修改为 `std::vector<double>(7, 0.0)`
> - 若是按照上述方式一的方式获取参数，参数文件 `src/ros2_rm_robot-humble/rm_motion_planner/config/planner_params_leftArm.yaml` 中的参数 `start_joint_positions` 的数值存储方式继续保持为字符串，还是是否需要修改？

##### 改进说明

**问题1：声明语句中的 std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) 是否可以修改为 std::vector<double>(7, 0.0)**

是的，完全可以修改为 `std::vector<double>(7, 0.0)`。这两种写法在功能上是等价的，都会创建一个包含7个元素且初始值都为0.0的vector。使用 `std::vector<double>(7, 0.0)` 的方式更加简洁，特别是当元素较多时，不需要重复写多个0.0。

**问题2：参数文件中的存储方式是否需要修改**

如果您要使用方式一（直接获取double数组类型参数），那么参数文件中的存储方式需要修改。

当前参数文件中使用的是字符串格式：

```yaml
start_joint_positions: "-1.734, -0.859, -0.484, -1.210, -1.458, 2.116, 1.104"
```

如果要直接使用double数组类型参数，应该修改为：

```yaml
start_joint_positions: [-1.734, -0.859, -0.484, -1.210, -1.458, 2.116, 1.104]
```

这样ROS 2就能正确地将其解析为double数组类型，而不是字符串类型。

##### 代码示例

让我展示一下这两种方式的代码示例：

```cpp
// 方式一：使用double数组类型参数（推荐）
// 声明参数
this->declare_parameter("start_joint_positions", std::vector<double>(7, 0.0));

// 获取参数
std::vector<double> joint_positions = this->get_parameter("start_joint_positions").as_double_array();
Eigen::Map<Eigen::VectorXd> jointAngles_start(joint_positions.data(), joint_positions.size());
```



```cpp
// 方式二：保持现有字符串方式
// 声明参数
this->declare_parameter<std::string>("start_joint_positions", "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0");

// 获取参数并解析
std::string start_joints_str = this->get_parameter("start_joint_positions").as_string();
// 然后使用解析函数处理字符串
```

因此，如果您希望使用更简洁的方式一，需要同时修改参数文件的格式

##### 代码解析

解释方式一代码示例中的如下语句

```c++
Eigen::Map<Eigen::VectorXd> jointAngles_start(joint_positions.data(), joint_positions.size());
```

1. **Eigen::Map**：
   - 这是 Eigen 库中的一个模板类，用于将现有的内存缓冲区映射为 Eigen 矩阵或向量
   - 它不会创建新的内存，而是直接使用传入的内存地址，避免了数据拷贝
   - 这是一种零开销的方式，将原始数据转换为Eigen的矩阵/向量格式
2. **Eigen::VectorXd**：
   - 这是Eigen库中动态大小的双精度浮点向量类型
   - "X" 表示动态大小（运行时确定），"d" 表示 double 类型
   - Vector 表示这是一个向量而不是矩阵
3. **构造参数**：
   - `joint_positions.data()`：获取 std::vector 的底层原始数据指针
   - `joint_positions.size()`：指定向量的大小
4. **整体效果**：
   - 这行代码创建了一个名为 jointAngles_start 的 Eigen 向量
   - 它直接使用 joint_positions 的内存，不进行数据拷贝
   - 修改 jointAngles_start 会直接影响 joint_positions 中的数据
5. **使用场景**：
   - 当需要在 Eigen 矩阵操作和原始数据之间转换时特别有用
   - 常用于与其他 API（如ROS）的数据结构交互
   - 可以避免不必要的数据拷贝，提高效率
6. **注意事项**：
   - 必须确保原始数据（joint_positions）在 Map 对象使用期间保持有效
   - 原始数据的生命周期必须长于 Map 对象
   - 修改Map会直接影响原始数据，这可能是优点也可能是风险

这种用法在机器人编程中很常见，特别是在需要与 ROS 消息或其他数据结构交互时，可以高效地在不同数据格式之间转换。

##### QNA1：

> 针对上述方式一的代码示例中的如下语句, 参数的默认数值如果为具体的数值，是写成[1.0, 2.0, 3.0 ,4.0, 5.0, 6.0, 7.0 ] 还是{1.0, 2.0, 3.0 ,4.0, 5.0, 6.0, 7.0 } ?
>
> ```
> this->declare_parameter("start_joint_positions", std::vector<double>(7, 0.0));
> ```

对于ROS 2参数声明中的默认值，应该使用**花括号**的形式：`{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}`，而不是方括号。

正确的写法应该是：

```cpp
this->declare_parameter("start_joint_positions", std::vector<double>({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}));
```

或者使用您之前提到的方式：

```cpp
this->declare_parameter("start_joint_positions", std::vector<double>(7, 0.0));  // 初始化7个元素，值都为0.0
```

原因如下：

1. `std::vector<double>({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0})` 使用了花括号初始化列表，这是C++11标准引入的统一初始化语法，适用于各种容器和数组。
2. `[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]` 这种方括号语法在C++中不是有效的初始化方式，方括号主要用于数组索引访问。
3. 在ROS 2参数系统中，当声明一个vector类型的参数时，我们需要提供一个std::vector对象作为默认值，而构造std::vector对象的正确方式是使用花括号初始化列表或重复值构造函数。

所以，如果您想为关节位置参数提供具体的默认值，应该使用花括号形式：

```cpp
this->declare_parameter("start_joint_positions", std::vector<double>({-1.734, -0.859, -0.484, -1.210, -1.458, 2.116, 1.104}));
```

##### QNA2：

> 结合上述代码实现, 说明为何方式一的声明参数语句中参数初始化赋值，使用的是std::vector<double>类型的数据，而问答02中的参数文件中的参数数据以double类型的数组形式存放？



让我来解释为什么在代码中使用`std::vector<double>`类型声明参数，而在YAML参数文件中以数组形式存放。

###### 1. 代码中的参数声明

在C++代码中：

```cpp
this->declare_parameter("start_joint_positions", std::vector<double>({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}));
```

这里使用`std::vector<double>`是因为：

- ROS 2的C++客户端库(rclcpp)需要明确的C++数据类型来处理参数
- `std::vector<double>`是C++中表示double数组的标准容器
- ROS 2参数系统支持多种类型，包括基本类型(bool, int, double, string)和复合类型(数组、结构体等)
- 对于数值数组，ROS 2使用`std::vector<T>`作为内部表示

###### 2. YAML参数文件中的数组表示

在YAML文件中：

```yaml
start_joint_positions: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
# 或者
start_joint_positions: 
  - 1.0
  - 2.0
  - 3.0
  - 4.0
  - 5.0
  - 6.0
  - 7.0
```

这里使用YAML数组语法是因为：

- YAML是一种人类可读的数据序列化标准

- ###### YAML原生支持数组/列表数据结构，使用方括号`[]`或破折号`-`**列表形式**

- ROS 2的参数系统在加载YAML文件时，会自动将YAML数组转换为对应的C++类型

- 对于double数组，YAML数组会被自动转换为`std::vector<double>`

###### 3. 两者之间的关系

这种设计体现了良好的分层架构：

1. **YAML层面**：使用YAML标准语法描述数据结构，与编程语言无关，便于配置管理
2. **ROS 2参数系统层面**：负责解析YAML并将其转换为对应语言的内部数据结构
3. **C++代码层面**：使用C++标准库类型`std::vector<double>`来接收和处理参数

4. 转换流程

```yaml
数组 [1.0, 2.0, 3.0] 
    ↓ (ROS 2参数系统解析)
std::vector<double>({1.0, 2.0, 3.0})
    ↓ (C++代码中使用)
Eigen::Map<Eigen::VectorXd> jointAngles_start(vector.data(), vector.size())
```

###### 5. 优势

这种设计的优势包括：

- **语言无关性**：YAML配置可以被不同语言的ROS 2客户端使用
- **类型安全**：C++代码中使用明确的类型，编译时检查
- **标准化**：遵循YAML和C++各自的标准语法
- **可读性**：YAML配置直观易懂，代码中类型明确

所以，代码中使用`std::vector<double>`而YAML中使用数组表示法是完全一致且合理的设计，它们在不同的抽象层次上工作，共同实现了参数的声明、配置和使用

##### QNA3：

>  结合上述问答，分析下述代码实现是否正确
>
> ```c++
> this->declare_parameter<std::vector<double>>("start_joint_positions",  {-1.734, -0.859, -0.484, -1.210, -1.458, 2.116, 1.104});
> this->declare_parameter<std::vector<double>>("target_joint_positions", {-0.973, -1.125, -0.899, -0.978, -2.299, 1.304, 1.449});
> ```

是的，这段代码实现是正确的。

让我分析一下：

###### 1. 语法正确性

```cpp
this->declare_parameter<std::vector<double>>("start_joint_positions",  {-1.734, -0.859, -0.484, -1.210, -1.458, 2.116, 1.104});
this->declare_parameter<std::vector<double>>("target_joint_positions", {-0.973, -1.125, -0.899, -0.978, -2.299, 1.304, 1.449});
```

这段代码在语法上完全正确：

- 使用了正确的模板参数 `std::vector<double>`
- 使用了花括号初始化列表 `{...}`，这是C++11标准引入的统一初始化语法
- 初始化列表中的数值都是合法的double类型

###### 2. 符合项目规范

根据项目规范记忆中的内容：

1. **ROS2参数处理与数组参数使用规范**：
   - 明确指出应该使用`std::vector<double>`类型参数
   - 通过`as_double_array()`获取参数
   - 避免字符串解析方式
   - 在声明具体默认值时应使用花括号初始化列表
2. **参数管理规范**：
   - 避免硬编码配置参数
   - 所有配置参数应通过参数服务器加载

###### 3. 与YAML配置文件的对应关系

这种方式与YAML配置文件中的数组形式完全对应：

```yaml
start_joint_positions: [-1.734, -0.859, -0.484, -1.210, -1.458, 2.116, 1.104]
target_joint_positions: [-0.973, -1.125, -0.899, -0.978, -2.299, 1.304, 1.449]
```

当YAML文件被加载时，这些数组会被自动转换为`std::vector<double>`类型，与代码中的声明一致。

###### 4. 使用方式

获取这些参数的正确方式应该是：

```cpp
std::vector<double> start_joints = this->get_parameter("start_joint_positions").as_double_array();
std::vector<double> target_joints = this->get_parameter("target_joint_positions").as_double_array();
```

###### 5. 优势

这种实现方式的优势包括：

- 类型安全：编译时就能检查类型正确性
- 简洁性：避免了字符串解析的复杂性
- 可读性：直接显示数值，易于理解
- 效率：避免了运行时的字符串解析开销

因此，这段代码实现完全符合ROS 2的最佳实践和项目的编码规范。



##### QNA4:

> 结合上述回答，有下述三种声明参数的方式
>
> ```c++
> this->declare_parameter<std::vector<double>>("start_joint_positions",  {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0})});
> 
> this->declare_parameter("start_joint_positions", std::vector<double>({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}));
> 
> this->declare_parameter<std::string>("start_joint_positions", std::vector<double>({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}));
> ```
>
> - 三种方式是否正确？
> - declare_parameter 是函数吗？
> - 为何 declare_parameter 后可以同时存在变量类型（如 [std::string](javascript:void(0)) 或 <std::vector<double>>）和参数列表？
> - 为何 declare_parameter 后的变量类型可有可无

###### 1. 三种方式是否正确？

让我分析每一种方式：

```cpp
this->declare_parameter<std::vector<double>>("start_joint_positions", {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0});
```

✅ **正确** - 这是标准的模板函数调用语法，显式指定参数类型

```cpp
this->declare_parameter("start_joint_positions", std::vector<double>({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}));
```

✅ **正确** - 利用C++模板类型推导，编译器会自动推导出参数类型

```cpp
this->declare_parameter<std::string>("start_joint_positions", std::vector<double>({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}));
```

❌ **错误** - 类型不匹配，模板参数指定为`std::string`，但实际传入的是`std::vector<double>`

###### 2. declare_parameter 是函数吗？

是的，[declare_parameter](javascript:void(0)) 是一个模板函数。在 ROS 2 的 rclcpp 库中，它是一个类成员函数，定义在 rclcpp::Node 类中。

###### 3. 为何 declare_parameter 后可以同时存在变量类型和参数列表？

这是C++中模板函数的标准语法：

```cpp
// 模板参数列表     函数参数列表
declare_parameter <Type>       (name, default_value)
```

- `<>` 中的是**模板参数**，用于指定编译时的类型
- `()` 中的是**函数参数**，用于传递运行时的值
- 这两者服务于不同的目的，可以同时存在

###### 4. 为何 declare_parameter 后的变量类型可有可无？

这得益于C++的**模板类型推导**特性：

```cpp
// 显式指定模板参数类型（无需类型推导）
this->declare_parameter<std::vector<double>>("start_joint_positions", {1.0, 2.0, 3.0});

// 依靠类型推导（编译器根据函数参数推导模板类型）
this->declare_parameter("start_joint_positions", std::vector<double>({1.0, 2.0, 3.0}));
```

两种方式在功能上是等价的，但有细微差别：

1. **显式指定类型**：
   - 更明确，可读性更好
   - 编译器不需要推导，编译可能稍快
   - 可以在某些复杂情况下避免推导错误
2. **依赖类型推导**：
   - 代码更简洁
   - 当构造函数参数与模板类型不完全一致时可能更灵活

在实际项目中，推荐使用显式指定类型的方式，因为它更清晰明确，符合项目规范中强调的类型安全要求。



### 改进点2：位姿变换计算

#### 问题分析



```c++
void execute_grasp_box(const std::shared_ptr<GraspBoxHandle> goal_handle)
{
    //... 其他代码 ...
    if (!trajectory_planned_) 
    {
        // ... 其他代码 ...
        
        RCLCPP_INFO(this->get_logger(), "%s 开始规划轨迹", WORKING_ICON.c_str());

        {
        // 获取参数
        std::string start_joints_str;       
        this->get_parameter("start_joint_positions", start_joints_str);
        // 打印参数
        RCLCPP_INFO(this->get_logger(), "%s 原始起始关节角度字符串: %s", DATA_ICON.c_str(), start_joints_str.c_str());
        // 解析起始关节位置参数
        Eigen::VectorXd jointAngles_start(7);
        parse_joint_positions(start_joints_str, jointAngles_start);       
        }
        
        // ... 其他代码 ...
    }
            
    // ... 其他代码 ...
}
```

- 代码语句数量多
- 需要调用额外定义的函数接口，导致整体代码量繁多
- 

> - 节点的 `execute_grasp_box` 函数中的 `Eigen::Matrix<double, 6, 1> gripper_tcp_pose_1` 变量类型是否可以替换成 Eigen::Isometry3d 更符合机器人位姿变换的功能需求？
> - 上述猜测若是可行的话，函数中的如下语句如何修改？tcpPose2HomogenMatrix 函数的参数类型与函数体实现该如何修改？
>
> ```c++
> Eigen::Matrix<double, 6, 1> gripper_tcp_pose_1;
> // TODO: 位置数据待修正，此处未修改URDF模型加入吸盘link，后续加入后应去
> gripper_tcp_pose_1 << goal->pose.position.x + 0.075 + 0.1318,
>                       goal->pose.position.y,
>                       goal->pose.position.z - 0.040,
>                       rm_robot_current_state_1.pose.euler.rx,
>                       rm_robot_current_state_1.pose.euler.ry,
>                       rm_robot_current_state_1.pose.euler.rz;
>  Eigen::Matrix4d matrix_base2tcp_target;  
>  matrix_base2tcp_target = tcpPose2HomogenMatrix(gripper_tcp_pose_1); 
> ```

#### 解决方案

##### 关于 Eigen::Isometry3d 

`Eigen::Isometry3d` 是 Eigen 库中表示三维刚体变换（等距变换）的类，包含：

- **旋转**：3×3 旋转矩阵
- **平移**：3×1 平移向量
- **齐次坐标**：4×4 变换矩阵形式

比原始矩阵更适合表示位姿变换，能保证变换的数学性质。

将 [gripper_tcp_pose_1](javascript:void(0)) 类型替换为 `Eigen::Isometry3d` 更符合机器人位姿变换的功能需求，因为：

- `Eigen::Isometry3d` 是专门设计用于表示3D空间中的刚体变换
- 它明确区分了旋转和平移部分，避免混淆
- 它提供了更丰富的变换操作方法
- 它是机器人学中的标准表示方式



##### 1. 修改变量定义和赋值：

```cpp
Eigen::Isometry3d gripper_tcp_pose_1 = Eigen::Isometry3d::Identity();
// 设置平移部分
gripper_tcp_pose_1.translation() = Eigen::Vector3d(
    goal->pose.position.x + 0.075 + 0.1318,
    goal->pose.position.y,
    goal->pose.position.z - 0.040
);

// 从欧拉角创建旋转矩阵并设置旋转部分
Eigen::AngleAxisd rollAngle(rm_robot_current_state_1.pose.euler.rx, Eigen::Vector3d::UnitX());
Eigen::AngleAxisd pitchAngle(rm_robot_current_state_1.pose.euler.ry, Eigen::Vector3d::UnitY());
Eigen::AngleAxisd yawAngle(rm_robot_current_state_1.pose.euler.rz, Eigen::Vector3d::UnitZ());
// RPY 顺序欧拉角（外旋 XYZ，内旋 ZYX）
Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
gripper_tcp_pose_1.linear() = quaternion.toRotationMatrix();
```

##### 2. 修改 tcpPose2HomogenMatrix 函数：

由于直接使用 `Eigen::Isometry3d`，不再需要 [tcpPose2HomogenMatrix](javascript:void(0)) 函数，可以直接使用 `gripper_tcp_pose_1.matrix()` 获取4x4齐次矩阵：

```cpp
Eigen::Matrix4d matrix_base2tcp_target = gripper_tcp_pose_1.matrix();
```

或者，如果仍想保留**转换函数**，可以将其修改为接受 `Eigen::Isometry3d` 参数：

```cpp
Eigen::Matrix4d transform2HomogenMatrix(const Eigen::Isometry3d& transform) {
    return transform.matrix();
}
```

这样修改后，代码将更加清晰地表达机器人的位姿变换概念，也更符合机器人学的标准实践

#### 拓展：Eigen::Isometry3d 用法



##### 定义和初始化：

```cpp
// 默认构造函数（单位变换）
Eigen::Isometry3d transform;

// 从旋转矩阵和平移向量构造
Eigen::Matrix3d rotation;
Eigen::Vector3d translation;
Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
transform.linear() = rotation;
transform.translation() = translation;

// 从4x4齐次矩阵构造
Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
Eigen::Isometry3d transform(matrix);
```

##### 赋值操作：

```cpp
// 设置平移部分
transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

// 设置旋转部分（通过旋转矩阵）
transform.linear() = Eigen::Matrix3d::Identity();

// 通过旋转角设置旋转部分
transform.linear() = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
```

##### 调用和使用：

```cpp
// 获取4x4齐次变换矩阵
Eigen::Matrix4d matrix = transform.matrix();

// 获取平移部分
Eigen::Vector3d translation = transform.translation();

// 获取旋转矩阵
Eigen::Matrix3d rotation = transform.linear();

// 变换点
Eigen::Vector3d point(1, 2, 3);
Eigen::Vector3d transformed_point = transform * point;
```

# 附录：知识拓展

## 日志输出展示优化

扩展建议/统一Emoji风格： 

```c++
// 建议在类定义顶部添加常量（可选）
 class RmSclerpPlannerNode1 : public rclcpp::Node 
 {
private:
    const std::string INFO_ICON = "ℹ️"; 
    const std::string SUCCESS_ICON = "✅";   
    const std::string ERROR_ICON = "❌";    
    const std::string WARN_ICON = "⚠️"; 
    const std::string FILE_ICON = "📂";

};
```

具体在代码中的应用是怎样的？直接将上述的 INFO_ICON 放入要输出的日志消息中吗？展示应用示例 上述的常量添加是位于类的 private 下，是否可以放在类外，采用  #rm_sclerp_planner_node_rightArm.cpp ？直接将上述的 INFO_ICON 放入要输出的日志消息中吗？展示应用示例 上述的常量添加是位于类的 private 下，是否可以放在类外，采用 #define 的形式？ 此外，上述回答中关于“Error Lens（错误提示增强）”的ROS2节点开发示例：

```c++
 // 在C++代码中添加带Emoji的日志 
#define LOG_ERROR(msg) RCLCPP_ERROR(get_logger(), "🔴 %s", msg) 
```

展示宏定义 LOG_ERROR(msg)  具体开发应用示例

### 统一 Emoji 风格的 ROS2 节点日志优化方案

根据您提供的代码和需求，我将展示三种实现方式：类内常量、全局宏定义和文件外常量定义。

#### 方案1：类内常量（推荐）

```cpp
class RmSclerpPlannerNode1 : public rclcpp::Node {
private:
    // Emoji常量定义
    const std::string INFO_ICON = "ℹ️";
    const std::string SUCCESS_ICON = "✅";
    const std::string DEBUG_ICON = "🔧";
    const std::string WARN_ICON = "⚠️";
    const std::string ERROR_ICON = "❌";

    const std::string FILE_ICON = "📝";
    const std::string PATH_ICON = "📂";
    const std::string LINK_ICON = "🔗"; 
    const std::string MODULE_ICON = "🧩";
    const std::string FUNCTION_ICON = "🔥";
    
    const std::string LAUNCH_ICON = "🚀";
    const std::string WAIT_ICON = "⏳"; 
    const std::string INITIALIZE_ICON = "⚙️";
    const std::string WORKING_ICON = "🔄";
    const std::string STOP_ICON = "🛑";
    const std::string TIMER_ICON = "⏱️";  
    const std::string COMPLETE_ICON = "🏁";  
    const std::string SIGNAL_ICON = "🚥";     
    const std::string TARGET_ICO = "🌲🎯 ";
    const std::string COMPLETE_ICO = "🎉";     
    
    const std::string MSG_ICON = "💬";
    const std::string DATA_ICON = "🔢";
    const std::string TOPIC_ICON = "📡"; 
    const std::string ACTION_ICON = "🤖";
    const std::string SERVICE_ICON = "🛠️";
    const std::string REQUEST_ICON = "📨";
    const std::string FEEDBACK_ICON = "🤝";
    const std::string IMPORTANT_ICON = "💡";   
        
    
public:
    explicit RmSclerpPlannerNode1(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("rm_sclerp_planner_node_rightArm", options) {
        // 使用示例
        RCLCPP_INFO(this->get_logger(), "%s 初始化右臂规划节点", INFO_ICON.c_str());
        
        try {
            initialize_hardware_and_planner();
            RCLCPP_INFO(this->get_logger(), "%s 硬件初始化完成", SUCCESS_ICON.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "%s 初始化失败: %s", ERROR_ICON.c_str(), e.what());
        }
    }
};
```

#### 方案2：全局宏定义（简单场景）

```cpp
// 在头文件或源文件顶部定义
#define LOG_INFO(msg) RCLCPP_INFO(get_logger(), "ℹ️ %s", msg)
#define LOG_SUCCESS(msg) RCLCPP_INFO(get_logger(), "✅ %s", msg)
#define LOG_ERROR(msg) RCLCPP_ERROR(get_logger(), "❌ %s", msg)
#define LOG_WARN(msg) RCLCPP_WARN(get_logger(), "⚠️ %s", msg)

// 使用示例（需在类方法中）
void RmSclerpPlannerNode1::some_method() {
    LOG_INFO("开始执行规划");
    if (success) {
        LOG_SUCCESS("规划完成");
    } else {
        LOG_ERROR("规划失败");
    }
}
```

#### 方案3：文件外常量定义（推荐跨文件使用）

1. 在 `rm_sclerp_planner_node_rightArm.hpp` 中添加：

```cpp
// 日志Emoji常量
namespace rm_logging {
    constexpr const char* INFO_ICON = "ℹ️";
    constexpr const char* SUCCESS_ICON = "✅";
    constexpr const char* ERROR_ICON = "❌";
    constexpr const char* WARN_ICON = "⚠️";
}
```

2. 在 [rm_sclerp_planner_node_rightArm.cpp](src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_node_rightArm.cpp) 中使用：

```cpp
#include "rm_sclerp_planner_node_rightArm.hpp"

void RmSclerpPlannerNode1::initialize_planner() {
    try {
        RCLCPP_INFO(this->get_logger(), "%s 初始化规划器...", rm_logging::INFO_ICON);
        // ...初始化代码...
        RCLCPP_INFO(this->get_logger(), "%s 规划器初始化成功", rm_logging::SUCCESS_ICON);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "%s 初始化错误: %s", 
                    rm_logging::ERROR_ICON, e.what());
    }
}
```

### 完整应用示例

以下是结合您代码的完整修改示例：

```cpp
// 在类定义顶部添加（方案1）
class RmSclerpPlannerNode1 : public rclcpp::Node {
private:
    const std::string INFO_ICON = "ℹ️";
    const std::string SUCCESS_ICON = "✅";
    const std::string ERROR_ICON = "❌";
    
    // 修改后的initialize_planner方法
    void initialize_planner() {
        // 获取urdf_path参数
        std::string urdf_path;
        this->get_parameter("urdf_path", urdf_path);
        RCLCPP_INFO(this->get_logger(), "%s 尝试使用URDF路径: %s", 
                   INFO_ICON.c_str(), urdf_path.c_str());
        
        if (urdf_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "%s 未提供URDF路径参数", 
                        ERROR_ICON.c_str());
            planner_interface_ = nullptr;
            return;
        }

        // 检查URDF文件
        std::ifstream urdf_file(urdf_path);
        if (!urdf_file.good()) {
            RCLCPP_ERROR(this->get_logger(), "%s URDF文件不存在: %s", 
                        ERROR_ICON.c_str(), urdf_path.c_str());
            planner_interface_ = nullptr;
            return;
        }
        urdf_file.close();
        
        try {
            planner_interface_ = std::make_unique<sclerp_interface::ScLERPInterface>(
                "base_link", "Link7", shared_from_this(), urdf_path);
            
            if (planner_interface_) {
                RCLCPP_INFO(this->get_logger(), "%s 规划器初始化成功", 
                           SUCCESS_ICON.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "%s 规划器初始化失败", 
                            ERROR_ICON.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "%s 创建规划器错误: %s", 
                        ERROR_ICON.c_str(), e.what());
            planner_interface_ = nullptr;
        }
    }
};
```

### 最佳实践建议

1. **统一风格**：
   - 推荐使用方案3（命名空间常量）实现跨文件统一
   - 对于简单项目，方案1（类内常量）更便捷

2. **终端兼容性**：
   ```cpp
   // 在main函数中添加终端检测
   int main(int argc, char **argv) {
       rclcpp::init(argc, argv);
       
       // 检测终端是否支持Emoji（简化示例）
       bool supports_emoji = true; // 实际可通过环境变量检测
       if (!supports_emoji) {
           RCLCPP_WARN(rclcpp::get_logger("rm_planner"), 
                      "终端不支持Emoji，将使用文本替代");
       }
       
       // ...其余代码...
   }
   ```

3. **回退方案**：
   ```cpp
   // 在配置文件中添加
   const std::string FALLBACK_INFO = "[INFO]";
   
   // 使用时根据条件选择
   const std::string& prefix = (supports_emoji) ? INFO_ICON : FALLBACK_INFO;
   RCLCPP_INFO(this->get_logger(), "%s 消息内容", prefix.c_str());
   ```

4. **VS Code配置**：
   ```json
   // settings.json中添加
   "terminal.integrated.fontFamily": "Fira Code, 'Noto Color Emoji'",
   "rclcpp.logging_format": "[{severity}] {message}"
   ```

选择最适合您项目规模的方案，建议从类内常量开始，随着项目扩大逐步迁移到命名空间常量方案。

### 补充说明: ROS2 日志中 Emoji 常量位置的详细解答

> 上述回答方案1中的代码示例中 Emoji 常量为何放在日志输出字符串之后，比如：
>
> ```
>   RCLCPP_INFO(this->get_logger(), "%s 初始化右臂规划节点", INFO_ICON.c_str()); 
> ```
>
> 可以写成下述模式吗 
>
> ```
> RCLCPP_INFO(this->get_logger(), INFO_ICON.c_str(),  "%s 初始化右臂规划节点" );
> ```
>
> s

您观察到的代码写法差异确实存在，这涉及到 ROS2 日志系统的格式化机制。以下是详细解释和最佳实践建议：

---

#### 一、为什么原始示例采用 `%s` 后置的写法？

ROS2 的 `RCLCPP_INFO` 等日志宏使用 `printf` 风格的格式化，其工作原理是：

1. **格式字符串在前**：第一个参数是格式字符串（包含 `%s` 等占位符）
2. **参数列表在后**：后续参数按顺序替换占位符

```cpp
// 正确写法（ROS2 标准格式）
RCLCPP_INFO(this->get_logger(), "%s 初始化右臂规划节点", INFO_ICON.c_str());
// 执行顺序：
// 1. 先处理格式字符串 "%s ..."
// 2. 用 INFO_ICON.c_str() 替换第一个 %s
```

---

#### 二、您提出的写法为何不可行？

```cpp
// 错误写法（会导致格式混乱）
RCLCPP_INFO(this->get_logger(), INFO_ICON.c_str(), "%s 初始化右臂规划节点");
// 问题：
// 1. 格式字符串被拆分成两个参数
// 2. 实际输出会是：`ℹ️ %s 初始化右臂规划节点`（%s 未被替换）
```

**根本原因**：
- ROS2 日志宏内部使用 `vsnprintf` 进行格式化
- 第一个非日志对象参数必须且只能是格式字符串
- 后续参数按顺序对应格式字符串中的占位符

---

#### 三、推荐解决方案（3种标准写法）

##### 方案1：标准格式化（推荐）
```cpp
const std::string INFO_ICON = "ℹ️";
RCLCPP_INFO(this->get_logger(), "%s 初始化右臂规划节点", INFO_ICON.c_str());
// 输出：ℹ️ 初始化右臂规划节点
```

##### 方案2：C++20 字符串拼接（需支持C++20）
```cpp
#include <format>
RCLCPP_INFO(this->get_logger(), std::format("{} 初始化右臂规划节点", INFO_ICON).c_str());
// 注意：需要ROS2 Humble+且编译器支持C++20
```

##### 方案3：辅助函数（最佳可维护性）
```cpp
// 在头文件中添加
void log_info(const rclcpp::Logger& logger, const std::string& icon, const std::string& msg) {
    RCLCPP_INFO(logger, "%s %s", icon.c_str(), msg.c_str());
}

// 使用示例
log_info(this->get_logger(), INFO_ICON, "初始化右臂规划节点");
```

---

#### 四、实际代码改造示例

以您代码中的 `initialize_planner()` 方法为例，改造前后对比：

**改造前（问题写法）**：

```cpp
void initialize_planner() {
    RCLCPP_INFO(this->get_logger(), INFO_ICON.c_str(), "初始化URDF路径检查...");  // 错误
    // ...
}
```

**改造后（正确写法）**：
```cpp
void initialize_planner() {
    // 方案1：标准格式化
    RCLCPP_INFO(this->get_logger(), "%s 初始化URDF路径检查...", INFO_ICON.c_str());
    
    // 或者方案3：辅助函数
    log_info(this->get_logger(), INFO_ICON, "开始加载URDF文件");
    
    std::string urdf_path;
    this->get_parameter("urdf_path", urdf_path);
    
    if (urdf_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "%s URDF路径未设置", ERROR_ICON.c_str());
        return;
    }
    // ...
}
```

---

#### 五、特殊场景处理

##### 1. 需要多个Emoji的情况
```cpp
RCLCPP_INFO(this->get_logger(), "%s %s 状态: %s", 
           INFO_ICON.c_str(), 
           "系统检查", 
           (is_ok ? SUCCESS_ICON.c_str() : ERROR_ICON.c_str()));
```

##### 2. 需要颜色控制的情况
```cpp
// 结合ANSI颜色代码（需终端支持）
RCLCPP_INFO(this->get_logger(), 
           "\033[32m%s\033[0m \033[34m%s\033[0m", 
           SUCCESS_ICON.c_str(), 
           "操作成功");
```

---

#### 六、最佳实践建议表

| 场景     | 推荐方案   | 代码示例                                               |
| -------- | ---------- | ------------------------------------------------------ |
| 简单日志 | 标准格式化 | `RCLCPP_INFO(..., "%s %s", icon.c_str(), msg.c_str())` |
| 复杂日志 | 辅助函数   | `log_info(logger, icon, "详细消息")`                   |
| 多Emoji  | 嵌套格式化 | `RCLCPP_INFO(..., "%s %s %s", icon1, icon2, msg)`      |
| 颜色控制 | ANSI+Emoji | `"\033[32m%s\033[0m 消息"`                             |

---

#### 七、为什么ROS2设计成这种格式？

这种设计出于以下考虑：
1. **安全性**：防止格式字符串注入攻击
2. **性能**：编译时确定格式字符串更高效
3. **兼容性**：与C语言printf系列函数保持一致
4. **可读性**：强制将格式字符串放在首位

建议坚持使用ROS2的标准日志格式，这能确保：
- 与ROS2工具链（如rqt_console）兼容
- 日志级别过滤正常工作
- 多线程环境下安全输出

如果需要更灵活的日志格式，可以考虑在应用层封装日志辅助函数，而不是试图改变ROS2底层日志机制。

## ROS2 下同时且快速编译多个功能包的指令

在 ROS2 中，您可以使用 `colcon build` 命令来同时编译多个功能包。以下是几种优化编译速度的方法：

### 1. 基本编译命令

```bash
colcon build --packages-select package1 package2 package3
```

这个命令会同时编译指定的多个功能包（`package1`, `package2`, `package3`）。

### 2. 加速编译的常用选项

#### 并行编译
```bash
colcon build --parallel-workers N
```
其中 `N` 是并行任务数，通常设置为 CPU 核心数的 1.5-2 倍。例如，对于 8 核 CPU：
```bash
colcon build --parallel-workers 12
```

#### 仅编译修改过的包
```bash
colcon build --symlink-install --packages-up-to
```
或针对特定包：
```bash
colcon build --symlink-install --packages-select package1 package2
```

#### 使用缓存（如果支持）
```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### 3. 完整优化编译命令示例

```bash
colcon build \
  --packages-select package1 package2 package3 \
  --parallel-workers $(nproc) \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 4. 其他有用选项

| 选项                                      | 说明                           |
| ----------------------------------------- | ------------------------------ |
| `--symlink-install`                       | 使用符号链接安装，加快迭代速度 |
| `--event-handlers console_direct+`        | 显示详细编译进度               |
| `--cmake-args -DCMAKE_BUILD_TYPE=Release` | 使用 Release 模式编译          |
| `--merge-install`                         | 合并安装到单个目录             |

### 5. 编译后操作

编译完成后，记得：
```bash
source install/setup.bash
```

### 注意事项

1. 首次编译建议不使用 `--parallel-workers` 或使用较小数值
2. 如果遇到编译错误，可以尝试先清理再编译：
   ```bash
   colcon build --clean
   ```
3. 对于大型工作空间，考虑使用 `ccache` 来加速重复编译

您可以通过 `colcon build --help` 查看所有可用选项。



## ROS2 功能包编译速度优化指南

在 ROS2 开发中，优化编译速度可以显著提高开发效率。以下是几种有效的优化方法：

### 1. 并行编译优化

#### 使用多核并行编译
```bash
colcon build --parallel-workers $(nproc)
```
或指定具体线程数（如12线程）：
```bash
colcon build --parallel-workers 12
```

#### 针对特定包的并行编译
```bash
colcon build --packages-select package1 package2 --parallel-workers 8
```

### 2. 增量编译策略

#### 只编译修改过的包
```bash
colcon build --packages-up-to
```

#### 使用符号链接安装（推荐开发时使用）

```bash
colcon build --symlink-install
```

### 3. 构建类型优化

#### 设置 Release 构建类型（比 Debug 快3-5倍）
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 针对特定包的构建类型设置

```bash
colcon build --packages-select package1 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 4. 缓存机制应用

#### 使用 ccache 加速重复编译
1. 安装 ccache：
   ```bash
   sudo apt install ccache
   ```
2. 设置环境变量：
   ```bash
   export CC="ccache gcc"
   export CXX="ccache g++"
   ```
3. 编译时添加参数：
   ```bash
   colcon build --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache
   ```

### 5. 编译选项优化

#### 禁用测试（如果不需要运行测试）
```bash
colcon build --cmake-args -DBUILD_TESTING=OFF
```

#### 禁用文档生成（如果不需要）
```bash
colcon build --cmake-args -DINSTALL_DOCUMENTATION=OFF
```

### 6. 工作空间结构优化

#### 合理组织功能包
- 将不常修改的基础包放在独立工作空间
- 经常修改的包放在单独工作空间

#### 使用混合安装策略
```bash
colcon build --merge-install
```

### 7. 完整优化命令示例

```bash
colcon build \
  --packages-select package1 package2 \
  --parallel-workers $(nproc) \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache
```

### 8. 其他实用技巧

1. **使用 `colcon graph` 分析依赖关系**：
   ```bash
   colcon graph --packages-above package1
   ```

2. **清理构建缓存**（当遇到奇怪错误时）：
   ```bash
   colcon build --clean
   ```

3. **监控编译时间**：
   ```bash
   time colcon build --parallel-workers 8
   ```

通过组合使用这些方法，通常可以将 ROS2 功能包的编译时间减少 50%-70%，特别是在大型工作空间中效果更为明显。



## ROS2 动作应用讲解

### 动作相关函数具体解析

#### 代码示例

##### action

`GraspBox.action`

```xml
# 目标定义
geometry_msgs/Pose pose
---
# 结果定义
bool success
string message
---
# 反馈定义
int32 status
```



##### node

`rm_sclerp_planner_node_leftArm.cpp`

```c++
//... 其余代码 ...
#include "sorting_robot_interfaces/action/grasp_box.hpp"
//... 其余代码 ...

using GraspBox = sorting_robot_interfaces::action::GraspBox;
using GraspBoxHandle = rclcpp_action::ServerGoalHandle<GraspBox>;
//... 其余代码 ...

class RmSclerpPlannerNode1 : public rclcpp::Node
{
 public:
    explicit RmSclerpPlannerNode1(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("rm_sclerp_planner_node_leftArm", options)
    {
    	//... 其余代码 ...
    	
        // 『动作服务端』
        // 抓取动作服务端
        grasp_box_server_ = rclcpp_action::create_server<GraspBox>(
            this,
            "grasp_action",
            std::bind(&RmSclerpPlannerNode1::handle_grasp_box_goal, this, _1, _2),
            std::bind(&RmSclerpPlannerNode1::handle_grasp_box_cancel, this, _1),
            std::bind(&RmSclerpPlannerNode1::handle_grasp_box_accepted, this, _1));
            
  private:
  
      	// 『动作服务端』
   		rclcpp_action::Server<GraspBox>::SharedPtr grasp_box_server_;
   		
   		//... 其余代码 ...
   		
   		    // Grasp Box 目标处理回调函数
    rclcpp_action::GoalResponse handle_grasp_box_goal(
        const rclcpp_action::GoalUUID & uuid, 
        std::shared_ptr<const GraspBox::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received grasp box goal: position(%.3f, %.3f, %.3f), orientation(%.3f, %.3f, %.3f, %.3f)", 
                    goal->pose.position.x, goal->pose.position.y, goal->pose.position.z,
                    goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->pose.orientation.w);
        (void)uuid;
        (void)goal;
        // 可以在这里添加目标验证逻辑
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Grasp Box  取消处理回调函数
    rclcpp_action::CancelResponse handle_grasp_box_cancel(const std::shared_ptr<GraspBoxHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel grasp box goal");
        (void)goal_handle;
        // 可以在这里添加取消处理逻辑
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Grasp Box  接受目标回调函数
    void handle_grasp_box_accepted( const std::shared_ptr<GraspBoxHandle> goal_handle)
    {
        // 使用线程执行实际任务，避免阻塞
        std::thread{std::bind(&RmSclerpPlannerNode1::execute_grasp_box, this, _1), goal_handle}.detach();
    }

    // Grasp box 实际执行函数
    void execute_grasp_box(const std::shared_ptr<GraspBoxHandle> goal_handle);
    
    //... 其余代码 ...
    
}
```

#### 代码讲解

##### 目标处理回调函数

```c++
rclcpp_action::GoalResponse handle_grasp_box_goal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const GraspBox::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received grasp box goal: position(%.3f, %.3f, %.3f), orientation(%.3f, %.3f, %.3f, %.3f)", 
                goal->pose.position.x, goal->pose.position.y, goal->pose.position.z,
                goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->pose.orientation.w);
    (void)uuid;
    (void)goal;
    // 可以在这里添加目标验证逻辑
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```

###### Info

这段代码是一个用于处理ROS 2（Robot Operating System 2）中动作（Action）目标的回调函数。它使用了`rclcpp_action`库，这是ROS 2中用于处理动作的C++库。动作是ROS 2中一种复杂的通信模式，它允许请求者（client）向执行者（server）发送一个目标（goal），执行者可以返回结果、反馈或者状态更新。

下面是对这段代码的逐行解释：

1. `rclcpp_action::GoalResponse handle_grasp_box_goal(`：定义了一个名为`handle_grasp_box_goal`的函数，它返回一个`rclcpp_action::GoalResponse`类型的值。这个函数是用来处理接收到的动作目标的。

2. `const rclcpp_action::GoalUUID & uuid,`：函数的第一个参数是一个常量引用，类型为`rclcpp_action::GoalUUID`。`GoalUUID`是一个唯一标识符，用于区分不同的动作目标。这里虽然接收了`uuid`参数，但在函数体内并没有使用它（通过`(void)uuid;`显式地标记为未使用，以避免编译器警告）。

3. `std::shared_ptr<const GraspBox::Goal> goal`：函数的第二个参数是一个指向`GraspBox::Goal`类型常量的共享指针。`GraspBox::Goal`很可能是一个自定义的消息类型，用于描述抓取箱子的目标，包括位置和方向等信息。使用共享指针可以高效且安全地管理内存，尤其是在多个对象可能需要访问同一个目标的情况下。

4. `RCLCPP_INFO(...)`：这行代码使用ROS 2的日志系统输出一条信息级别的日志。它打印出接收到的抓取箱子目标的位置（x, y, z坐标）和方向（四元数的x, y, z, w分量）。这对于调试和监控动作目标的接收非常有用。

5. `(void)uuid;` 和 `(void)goal;`：这两行是为了告诉编译器，这两个参数虽然被传入函数，但在当前函数体内并未被使用。这是一种避免编译器发出未使用变量警告的常见做法。

6. `// 可以在这里添加目标验证逻辑`：这是一个注释，提示开发者可以在这个位置添加对接收到的目标的验证逻辑。例如，检查目标是否在机器人的可达范围内，或者是否符合其他执行条件。

7. `return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;`：函数的最后一行返回一个`GoalResponse`枚举值，表示接受并执行接收到的目标。这是对动作服务器接收到目标后的一种标准响应，意味着服务器将尝试执行该目标。

总的来说，这段代码是一个动作服务器处理接收到的抓取箱子目标的示例。它打印出目标信息，并表明将接受并执行这个目标。在实际应用中，可能还需要添加更多的逻辑来处理和验证目标。

###### QNA

> handle_grasp_box_goal 函数传入的参数 goal 是否是包含 src/ros2_rm_robot-humble/sorting_robot_interfaces/action/GraspBox.action 中定义的变量的结构体 ?



『**回答**』

是的，`handle_grasp_box_goal` 函数传入的参数 `goal` 是包含 `src/ros2_rm_robot-humble/sorting_robot_interfaces/action/GraspBox.action` 中定义的变量的结构体。

1. **代码中的 `GraspBox` 类型定义**：
   
   ```cpp
   using GraspBox = sorting_robot_interfaces::action::GraspBox;
   ```
   这行代码表明 `GraspBox` 是 `sorting_robot_interfaces::action::GraspBox` 的别名。
   
2. **动作服务端的创建**：
   ```cpp
   grasp_box_server_ = rclcpp_action::create_server<GraspBox>(
       this,
       "grasp_action",
       std::bind(&RmSclerpPlannerNode1::handle_grasp_box_goal, this, _1, _2),
       std::bind(&RmSclerpPlannerNode1::handle_grasp_box_cancel, this, _1),
       std::bind(&RmSclerpPlannerNode1::handle_grasp_box_accepted, this, _1));
   ```
   这里创建了一个动作服务端，使用的动作类型是 `GraspBox`。当客户端发送目标时，`handle_grasp_box_goal` 函数会被调用，并且传入的 `goal` 参数类型是 `GraspBox::Goal`。

3. **`GraspBox.action` 文件的作用**：
   - 在 ROS 2 中，`.action` 文件定义了动作的三个部分：目标（goal）、结果（result）和反馈（feedback）。
   - 编译时，ROS 2 工具会根据 `.action` 文件自动生成对应的 C++ 类型，包括 `GraspBox::Goal`、`GraspBox::Result` 和 `GraspBox::Feedback`。
   - 因此，`GraspBox::Goal` 结构体包含了 `GraspBox.action` 文件中定义的目标变量。

4. **函数签名中的 `goal` 参数**：
   ```cpp
   rclcpp_action::GoalResponse handle_grasp_box_goal(
       const rclcpp_action::GoalUUID & uuid, 
       std::shared_ptr<const GraspBox::Goal> goal)
   ```
   - `goal` 参数的类型是 `std::shared_ptr<const GraspBox::Goal>`，它指向一个常量 `GraspBox::Goal` 对象。
   - 这个对象包含了 `GraspBox.action` 文件中定义的所有目标变量。

『**结论**』

`handle_grasp_box_goal` 函数中的 `goal` 参数确实是一个结构体，它包含了 `src/ros2_rm_robot-humble/sorting_robot_interfaces/action/GraspBox.action` 文件中定义的目标变量。这些变量是通过 ROS 2 的代码生成工具从 `.action` 文件自动生成的。

##### 取消处理回调函数

###### Info

```c++
// Grasp Box  取消处理回调函数
rclcpp_action::CancelResponse handle_grasp_box_cancel(const std::shared_ptr<GraspBoxHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel grasp box goal");
    (void)goal_handle;
    // 可以在这里添加取消处理逻辑
    return rclcpp_action::CancelResponse::ACCEPT;
}
```

这段代码是用C++编写的，并且是ROS 2（Robot Operating System 2）框架中的一个函数实现，特别是使用了`rclcpp_action`库来处理动作（Action）的取消请求。ROS 2是一个用于机器人软件开发的灵活和模块化的框架。在ROS 2中，动作（Action）是一种通信类型，用于实现需要长时间运行或者需要反馈和能够被取消的目标导向任务。

函数`handle_grasp_box_cancel`的目的是处理一个名为`grasp_box`的动作的取消请求。下面是该函数的详细解释：

1. **函数签名**:
   
    ```cpp
    rclcpp_action::CancelResponse handle_grasp_box_cancel(const std::shared_ptr<GraspBoxHandle> goal_handle)
    ```
    - 返回类型是`rclcpp_action::CancelResponse`，表示取消请求的响应。
    - 参数是一个`std::shared_ptr<GraspBoxHandle>`类型的共享指针`goal_handle`，指向与这个取消请求相关的目标（goal）的句柄。`GraspBoxHandle`可能是一个自定义类，用于管理`grasp_box`动作的状态和信息。
    
2. **函数体**:
    ```cpp
    RCLCPP_INFO(this->get_logger(), "Received request to cancel grasp box goal");
    ```
    - 使用`RCLCPP_INFO`宏打印一条信息日志，表明已经接收到了一个取消`grasp_box`目标的请求。`this->get_logger()`获取当前节点的日志器，用于记录日志信息。

    ```cpp
    (void)goal_handle;
    ```
    - 这行代码将`goal_handle`参数强制转换为`void`类型。这通常是为了避免编译器发出未使用参数的警告，因为在这个函数中，`goal_handle`参数实际上并没有被使用。在实际应用中，如果需要基于目标句柄执行某些操作，则会使用这个参数。

3. **取消处理逻辑**:
    - 注释指出可以在这里添加取消处理逻辑。这意味着，根据实际需求，开发者可以在这里编写代码来响应取消请求，例如清理资源、更新状态等。

4. **返回值**:
    ```cpp
    return rclcpp_action::CancelResponse::ACCEPT;
    ```
    - 函数返回`rclcpp_action::CancelResponse::ACCEPT`，表示接受取消请求。在ROS 2的动作接口中，取消请求可以被接受（ACCEPT）或拒绝（REJECT）。接受取消请求通常意味着动作将被停止，并且任何相关的资源将被清理。

总之，这个函数是用于处理ROS 2中一个特定动作（`grasp_box`）的取消请求的。它记录了取消请求的接收，并返回一个表示接受取消请求的响应。根据实际需求，可以在函数中添加更复杂的取消处理逻辑。

##### 接受目标回调函数

###### Info

```c++
// Grasp Box  接受目标回调函数
void handle_grasp_box_accepted( const 
std::shared_ptr<GraspBoxHandle> goal_handle)
{
    // 使用线程执行实际任务，避免阻塞
    std::thread{std::bind(&RmSclerpPlannerNode1::execute_grasp_box,
this, _1), goal_handle}.detach();
}
```

这段C++代码定义了一个名为`handle_grasp_box_accepted`的函数，该函数接受一个`std::shared_ptr<GraspBoxHandle>`类型的参数`goal_handle`。这个函数的主要目的是在非阻塞的方式下处理一个抓取盒子的请求。为了实现这一点，它使用了一个新线程来执行实际的任务。下面是对代码的详细解释：

1. **函数签名**:
   
    ```cpp
    void handle_grasp_box_accepted(const std::shared_ptr<GraspBoxHandle> goal_handle)
    ```
    - `void`：函数没有返回值。
    - `handle_grasp_box_accepted`：函数名。
    - `const std::shared_ptr<GraspBoxHandle> goal_handle`：函数参数，一个指向`GraspBoxHandle`对象的共享指针。`GraspBoxHandle`可能是一个类，用于表示或处理与抓取盒子相关的信息或状态。使用`const`意味着在函数内部不会修改`goal_handle`指向的对象。
    
2. **创建和启动线程**:
    ```cpp
    std::thread{std::bind(&RmSclerpPlannerNode1::execute_grasp_box, this, _1), goal_handle}.detach();
    ```
    这部分代码较为复杂，它做了几件事情：
    
    - **`std::bind`**: 用于将`RmSclerpPlannerNode1`类的成员函数`execute_grasp_box`与当前对象实例（`this`）以及一个占位符`_1`绑定。`std::bind`返回一个可调用对象，这个对象在被调用时会使用提供的参数调用`execute_grasp_box`函数。在这里，`_1`是一个占位符，表示`execute_grasp_box`函数的第一个参数将在稍后提供。
    
    - **传递参数**: 在创建`std::thread`对象时，`goal_handle`作为参数传递给了`std::bind`返回的可调用对象。实际上，这里的写法有些误导，因为`goal_handle`应该直接作为`std::bind`调用时的参数来替换`_1`，而不是在`std::thread`的构造函数中作为第二个参数。正确的用法应该是将`goal_handle`作为`std::bind`的参数，替换掉`_1`。一个更正的例子可能是：`std::bind(&RmSclerpPlannerNode1::execute_grasp_box, this, goal_handle)`。
    
    - **创建线程**: 使用`std::thread`的构造函数，以`std::bind`返回的可调用对象作为参数创建一个新线程。这意味着`execute_grasp_box`函数将在新线程中执行。
    
    - **分离线程**: `.detach()`用于将新创建的线程与`std::thread`对象分离，使得线程可以在后台独立运行。分离后的线程将在完成后自动释放资源。这意味着主线程（或调用`handle_grasp_box_accepted`的线程）不会等待新线程完成，从而实现了非阻塞的行为。

**注意**：代码中的`std::thread`构造方式是不正确的，因为它试图将两个参数（一个可调用对象和一个额外的参数）传递给`std::thread`的构造函数，而实际上应该将所有需要的参数封装在`std::bind`中。正确的实现应该确保`std::bind`正确地绑定了所有需要的参数，然后将结果传递给`std::thread`的构造函数。正确的代码可能类似于：
```cpp
void handle_grasp_box_accepted(const std::shared_ptr<GraspBoxHandle> goal_handle) {
    auto task = std::bind(&RmSclerpPlannerNode1::execute_grasp_box, this, goal_handle);
    std::thread(task).detach();
}
```
或者更简洁地，直接在`std::thread`的构造函数中使用`std::bind`：
```cpp
void handle_grasp_box_accepted(const std::shared_ptr<GraspBoxHandle> goal_handle) {
    std::thread(std::bind(&RmSclerpPlannerNode1::execute_grasp_box, this, goal_handle)).detach();
}
```



动作执行的结果反馈

按照ROS 2动作机制的最佳实践，应该使用动作的结果（Result）来传递执行结果



问答2：

（1）已知条件：当前右臂节点程序的动作执行函数 execute_grasp_box 中存在触发左臂动作的函数 trigger_left_arm_task，在此之后是关闭吸盘。

（2）目标任务：

- 现在想将（1）中的功能逻辑更改为，左臂节点中的动作执行函数 execute_trigger_left_arm 中执行到交付点位（此功能代码尚未在函数体中实现，是接下来的任务之一）后，返回一个结果（暂定为布尔量 isLeftArmArrived）给右臂节点中的动作客户端 trigger_left_arm_client_，
- 然后在右臂节点的动作执行函数 execute_grasp_box 中的 trigger_left_arm_task 函数调用之后添加获取左臂节点反馈的结果（上述提及的布尔量 isLeftArmArrived）然后关闭吸盘。

（3）疑问：

当前的左右臂节点程序设计实现为：在 grasp_box 动作服务端的执行函数 execute_grasp_box 中通过触发函数trigger_left_arm_task/trigger_right_arm_task  进而启动另一条臂节点中的动作服务端（trigger_left_arm_server_ 或  trigger_right_arm_server_），这种方案是否可行？

上述（2）目标任务的第一点功能，针对左臂节点中的动作执行函数 execute_trigger_left_arm 中执行完成左臂到达纸箱吸取点代码后反馈一个结果给右臂。所提出的添加布尔量的方式是否可行？按照ROS2 动作机制的最佳实践方式是怎样的？

上述（2）目标任务的第二点功能，针对在右臂节点的动作执行函数 execute_grasp_box 中的 trigger_left_arm_task 函数调用之后，根据获取的左臂节点反馈的结果决定是否关闭吸盘。所提出的添加布尔量的方式是否可行？按照ROS2 动作机制的最佳实践方式是怎样的？



给出方案说明与代码实现（不应用到当前项目中）





任务04：针对任务02的目标，即在右臂 execute_grasp_box 中，根据左臂的 trigger_left_arm_server_ 的返回结果，决定自身任务成败



## C++ 开发学习

### Eigen 库基础知识与工程应用

####  Eigen::AngleAxisd 概念与应用

`Eigen::AngleAxisd` 是 Eigen 库中表示旋转的一种方式，它使用旋转轴和旋转角度来描述三维空间中的旋转。让我详细解释：

##### **基本概念**

简介

- `AngleAxisd` 表示一个绕任意轴旋转指定角度的旋转
- "d" 后缀表示使用 double 精度（对应 float 精度的是 `AngleAxisf`）
- 它是旋转的轴角表示法，比欧拉角更稳定，没有万向节死锁问题
- 这种表示方式在机器人学和计算机图形学中特别常用，因为它既直观又稳定，是处理3D旋转的优秀选择。

**优势**：

- 避免万向节死锁
- 旋转插值更平滑
- 计算效率高
- 数值稳定性好

**注意事项**：

- 旋转角度使用弧度制
- 旋转轴必须是单位向量
- 旋转顺序很重要（旋转不满足交换律）

**应用场景**：

- 3D图形中的旋转变换
- 机器人学中的关节旋转
- 姿态表示和转换
- 导航系统中的方向表示

##### **构造方式**：

```cpp
// 构造一个绕z轴旋转45度的旋转
Eigen::AngleAxisd rotation(M_PI/4, Eigen::Vector3d::UnitZ());

// 也可以先创建空对象，再设置值
Eigen::AngleAxisd rotation;
rotation.angle() = M_PI/4;
rotation.axis() = Eigen::Vector3d::UnitZ();
```

3. **与其他旋转表示的转换**：
```cpp
// 转换为旋转矩阵
Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();

// 转换为四元数
Eigen::Quaterniond quaternion(rotation);

// 从旋转矩阵创建
Eigen::AngleAxisd rotation_from_matrix(rotation_matrix);

// 从四元数创建
Eigen::AngleAxisd rotation_from_quat(quaternion);
```

##### 应用案例

4. **实际使用示例**：

```cpp
// 创建绕x轴旋转的旋转（roll）
Eigen::AngleAxisd roll_rotation(roll_angle, Eigen::Vector3d::UnitX());

// 创建绕y轴旋转的旋转（pitch）
Eigen::AngleAxisd pitch_rotation(pitch_angle, Eigen::Vector3d::UnitY());

// 创建绕z轴旋转的旋转（yaw）
Eigen::AngleAxisd yaw_rotation(yaw_angle, Eigen::Vector3d::UnitZ());

// 组合旋转
Eigen::Quaterniond total_rotation = yaw_rotation * pitch_rotation * roll_rotation;

// 应用旋转到向量
Eigen::Vector3d point(1, 0, 0);
Eigen::Vector3d rotated_point = roll_rotation * point;
```

6. 计算齐次矩阵

```c++
Eigen::Matrix4d tcpPose2HomogenMatrix(const Eigen::Matrix<double, 6, 1>& tcpPose)
{
    Eigen::Matrix4d homogenMatrix = Eigen::Matrix4d::Identity();

    // 提取位置部分
    homogenMatrix(0, 3) = tcpPose[0];  // x
    homogenMatrix(1, 3) = tcpPose[1];  // y
    homogenMatrix(2, 3) = tcpPose[2];  // z

    // 提取欧拉角（RPY顺序：Rx, Ry, Rz 对应 Roll, Pitch, Yaw）
    double roll  = tcpPose[3];   // Rx
    double pitch = tcpPose[4];  // Ry
    double yaw   = tcpPose[5];    // Rz

    // 计算旋转矩阵（XYZ顺序，即RPY顺序）
    // 外旋 XYZ顺序，内旋 ZYX顺序
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)    

    // #region使用Eigen的AngleAxis和Quaternion
    {
    Eigen::AngleAxisd roll_axis(roll, Eigen::Vector3d::UnitX());    // Roll
    Eigen::AngleAxisd pitch_axis(pitch, Eigen::Vector3d::UnitY()); // Pitch
    Eigen::AngleAxisd yaw_axis(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quaternion = yaw_axis * pitch_axis * roll_axis ;
    homogenMatrix.block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
    }

    return homogenMatrix;
}
```

1. 解释 Eigen::Quaterniond 创建的变量 quaternion 是什么？ quaternion.toRotationMatrix() 是什么意思？

`Eigen::Quaterniond` 是Eigen库中表示四元数的类型，用于描述3D空间中的旋转。

```cpp
Eigen::Quaterniond quaternion = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
```

这段代码创建了一个四元数，表示按顺序进行的三次旋转：

- 绕X轴旋转roll角度
- 绕Y轴旋转pitch角度
- 绕Z轴旋转yaw角度

`quaternion.toRotationMatrix()` 是将四元数转换为3x3的旋转矩阵。旋转矩阵是另一种表示3D旋转的方式，可以直接用于坐标变换。

3. `quaternion.toRotationMatrix()` 和 `quaternion.normalized().toRotationMatrix()` 的区别

主要区别在于数值稳定性：

- `quaternion.toRotationMatrix()`：直接将四元数转换为旋转矩阵
- `quaternion.normalized().toRotationMatrix()`：先对四元数进行归一化，然后再转换为旋转矩阵

由于数值计算误差，四元数可能会略微偏离单位长度。归一化可以确保四元数是单位四元数，从而保证生成的旋转矩阵是正交的。在大多数情况下，这种差异非常小，但如果对数值精度要求很高，使用归一化版本更安全。

4. 解释 homogenMatrix.block<3, 3>(0, 0)

`homogenMatrix.block<3, 3>(0, 0)` 是Eigen库中用于操作矩阵块的语法：

- `block<3, 3>`：表示要操作一个3行3列的子矩阵块
- `(0, 0)`：表示这个子矩阵块在原矩阵中的起始位置（行索引0，列索引0）

具体来说：

```cpp
homogenMatrix.block<3, 3>(0, 0) = rotation;
```

这行代码将计算得到的3x3旋转矩阵[rotation](javascript:void(0))赋值给齐次变换矩阵[homogenMatrix](javascript:void(0))的左上角3x3部分，即旋转部分。

齐次变换矩阵的结构如下：

```
[ r11  r12  r13  tx ]
[ r21  r22  r23  ty ]
[ r31  r32  r33  tz ]
[  0    0    0   1 ]
```

其中`block<3, 3>(0, 0)`操作的就是包含r11到r33的这部分



#### Eigen::Isometry3d 概念与应用

> - Eigen::Isometry3d 类型的变量如何定义，初始化，赋值，调用

Eigen::Isometry3d 是用于表示3D空间中的等距变换（包括旋转和平移）的类型，非常适合表示机器人位姿。

##### 基本用法

###### 定义和初始化：

```cpp
// 默认构造函数（单位变换）
Eigen::Isometry3d transform;

// 从旋转矩阵和平移向量构造
Eigen::Matrix3d rotation;
Eigen::Vector3d translation;
Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
transform.linear() = rotation;
transform.translation() = translation;

// 从4x4齐次矩阵构造
Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
Eigen::Isometry3d transform(matrix);
```

###### 赋值操作：

```cpp
// 设置平移部分
transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

// 设置旋转部分（通过旋转矩阵）
transform.linear() = Eigen::Matrix3d::Identity();

// 通过旋转角设置旋转部分
transform.linear() = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
```

###### 调用和使用：

```cpp
// 获取4x4齐次变换矩阵
Eigen::Matrix4d matrix = transform.matrix();

// 获取平移部分
Eigen::Vector3d translation = transform.translation();

// 获取旋转矩阵
Eigen::Matrix3d rotation = transform.linear();

// 变换点
Eigen::Vector3d point(1, 2, 3);
Eigen::Vector3d transformed_point = transform * point;
```



#### Eigen::Map 概念与应用

##### 基本概念

- 这是Eigen库中的一个模板类，用于将现有的内存缓冲区映射为Eigen矩阵或向量
- 它不会创建新的内存，而是直接使用传入的内存地址，避免了数据拷贝
- 这是一种零开销的方式，将原始数据转换为Eigen的矩阵/向量格式

##### Eigen::Map与普通Eigen对象区别

Eigen::Map与普通Eigen对象的主要区别在于内存管理和数据存储方式：

1. **内存管理**：
   - 普通Eigen对象拥有自己的内存空间，会自动分配和管理内存
   - Eigen::Map不拥有内存，它只是映射到现有的内存区域，不会分配新内存

2. **数据存储**：
   - 普通Eigen对象存储在Eigen的内部数据结构中
   - Eigen::Map直接操作外部提供的内存缓冲区，可以与其他库(如OpenCV、STL容器)共享数据

3. **使用场景**：
   - 普通Eigen对象适用于需要独立存储数据的场景
   - Eigen::Map适用于需要与其他库交互或避免数据拷贝的场景

4. **性能考虑**：
   - Eigen::Map避免了数据拷贝，可以提高性能
   - 普通Eigen对象可能涉及内存分配和数据拷贝操作

示例代码：
```cpp
// 普通Eigen对象
Eigen::MatrixXd eigenMatrix(3, 3);  // 分配自己的内存

// Eigen::Map使用外部内存
double* externalData = new double[9];
Eigen::Map<Eigen::MatrixXd> mappedMatrix(externalData, 3, 3);  // 映射到外部内存
```



##### ROS2 中应用案例



```c++
// 声明参数
this->declare_parameter("start_joint_positions", std::vector<double>(7, 0.0));

// 获取参数   
std::vector<double> joint_positions = this->get_parameter("start_joint_positions").as_double_array();
Eigen::Map<Eigen::VectorXd> jointAngles_start(joint_positions.data(), joint_positions.size());  
```

1. **Eigen::Map**：
   - 这是Eigen库中的一个模板类，用于将现有的内存缓冲区映射为Eigen矩阵或向量
   - 它不会创建新的内存，而是直接使用传入的内存地址，避免了数据拷贝
   - 这是一种零开销的方式，将原始数据转换为Eigen的矩阵/向量格式
2. **Eigen::VectorXd**：
   - 这是Eigen库中动态大小的双精度浮点向量类型
   - "X"表示动态大小（运行时确定），"d"表示double类型
   - Vector表示这是一个向量而不是矩阵
3. **构造参数**：
   - `joint_positions.data()`：获取std::vector的底层原始数据指针
   - `joint_positions.size()`：指定向量的大小
4. **整体效果**：
   - 这行代码创建了一个名为jointAngles_start的Eigen向量
   - 它直接使用joint_positions的内存，不进行数据拷贝
   - 修改jointAngles_start会直接影响joint_positions中的数据
5. **使用场景**：
   - 当需要在Eigen矩阵操作和原始数据之间转换时特别有用
   - 常用于与其他API（如ROS）的数据结构交互
   - 可以避免不必要的数据拷贝，提高效率
6. **注意事项**：
   - 必须确保原始数据（joint_positions）在Map对象使用期间保持有效
   - 原始数据的生命周期必须长于Map对象
   - 修改Map会直接影响原始数据，这可能是优点也可能是风险

这种用法在机器人编程中很常见，特别是在需要与ROS消息或其他数据结构交互时，可以高效地在不同数据格式之间转换
