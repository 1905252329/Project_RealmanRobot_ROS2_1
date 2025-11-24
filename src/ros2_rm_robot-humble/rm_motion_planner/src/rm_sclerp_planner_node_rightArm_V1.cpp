/**
 * @brief realman robot 单臂控制节点
 * @note 开发调试版本
 * @note *（右）机械臂IP：169.254.247.19
 * @note 功能测试版本V1, 定时器设计模式
 * @paramaters: 基于示教采集的数据硬编码
 */
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rm_ros_interfaces/msg/jointpos.hpp"
#include "std_msgs/msg/bool.hpp"
// 修复头文件包含路径
#include "kinlib/collision_utils.h"
#include "sclerp_motion_planner/utils.h"
#include "sclerp_motion_planner/interpolator.h"
#include "sclerp_motion_planner/sclerp_interface.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "rm_ros_interfaces/srv/get_dh.hpp"
#include "rm_ros_interfaces/msg/movej.hpp"

// 添加 URDF 相关的头文件
// 添加 Eigen 头文件
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// 添加 geometry_msgs 头文件
#include "geometry_msgs/msg/pose_stamped.hpp"

// 添加 URDF 相关的头文件
#include "urdf/model.h"

// 添加 RM API 头文件
#include "rm_driver/rm_interface.h"
#include "rm_driver/rm_service.h"
#include "rm_driver/rm_define.h"
#include "rm_motion_planner/robot_interface.h"


// Jodell Srv 头文件
#include "rm_ros_interfaces/srv/initialize_gripper.hpp"
#include "rm_ros_interfaces/srv/activate_gripper.hpp"
#include "rm_ros_interfaces/srv/deactivate_gripper.hpp"
#include "rm_ros_interfaces/srv/close_modbus.hpp"



using namespace std::chrono_literals;
using std::placeholders::_1;

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

        {
        // // 初始化机械臂
        // initialize_rmRobot();

        // // 初始化规划器
        // initialize_planner();

        // // 初始化电吸盘
        // std::string jodell_initialize_cmd = "ros2 service call /right_arm/initialize_gripper rm_ros_interfaces/srv/InitializeGripper \""
        //     "{ip: '192.168.1.19', port: 8080, register_address: 1000, comm_port: 1, device_address: 9}\"";
        // call_service_sync(jodell_initialize_cmd); 
        }
        
      /********************************* 通信：话题，服务，动作 *******************************/        
        // 创建发布者，用于直接向rm_driver发送轨迹点
        joint_pos_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Jointpos>(
            "/right_arm/rm_driver/movej_canfd_cmd", 20);
            
        // 创建订阅者，用于获取当前关节位置
        // 修改订阅话题为 /joint_states，这是rm_driver实际发布关节状态的话题
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/right_arm/joint_states", 20,
            std::bind(&RmSclerpPlannerNode1::joint_state_callback, this, std::placeholders::_1));

        // 创建订阅者，用于接收盒子中心点位姿
        box_centerPoint_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/right_posedemo", 20,
            std::bind(&RmSclerpPlannerNode1::box_centerPoint_callback, this, std::placeholders::_1));
            
        // 创建MoveJ消息发布者
        movej_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movej>(
            "/right_arm/rm_driver/movej_cmd", 10);
        

            
    /******************************************* 定时器 ****************************************/

        //『初始化定时器』
        // 用于初始化硬件资源与规划器
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RmSclerpPlannerNode1::init_timer_callback, this));
        
        // // 『定时器』，
        // // 用于周期性检查和执行任务
        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(100),  // 100ms检查一次
        //     std::bind(&RmSclerpPlannerNode1::timer_callback, this));

        // 『轨迹定时器』
        // 用于发布轨迹点，提高轨迹执行频率
// #region 测试运动学接口功能
        // @note 若注释，不执行轨迹点运动功能
        // @note 下文的 trajectory_timer_callback() 中的注释保持同步
        // 固定频率
        // 创建20ms定时器用于发布轨迹点，与插值频率(50Hz)更好地匹配
        // trajectory_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(20),
        //     std::bind(&RmSclerpPlannerNode1::trajectory_timer_callback, this));
        // 动态频率
        // int execution_rate;
        // this->get_parameter("trajectory_execution_rate", execution_rate);
        // auto timer_period = std::chrono::microseconds(1000000 / execution_rate);
        // @note 若注释，不执行轨迹点运动功能
        // @note 下文的 trajectory_timer_callback() 中的注释保持同步        
        // trajectory_timer_ = this->create_wall_timer(
        //     timer_period,
        //     std::bind(&RmSclerpPlannerNode1::trajectory_timer_callback, this));
// #endregion



    /******************************************* 外设模块 ****************************************/

        /************************************** 变量初始化 **************************************/
        // 初始化轨迹点索引
        trajectory_point_index_ = 0;
        trajectory_executing_ = false;
        // 初始化 current_joint_positions_
        current_joint_positions_.fill(0.0);

        RCLCPP_INFO(this->get_logger(), "rm_sclerp_planner_node_1节点已经启动.");


    }


  private:
    
    // 发布者：用于发布关节轨迹点到rm_driver
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr joint_pos_publisher_;

    // 发布者：用于发布MoveJ消息到rm_driver
    rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr movej_publisher_;

    // 订阅者：用于订阅当前关节状态
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

    // 订阅者：用于订阅盒子中心点位姿
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr box_centerPoint_subscriber_;

    // 『初始化定时器』
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    // 『定时器』：用于定期执行规划和控制任务
    rclcpp::TimerBase::SharedPtr timer_;

    // 『轨迹定时器』：用于发布轨迹点
    rclcpp::TimerBase::SharedPtr trajectory_timer_;

    // 🧪 DH参数服务客户端（功能调试用）
    // 确保客户端在构造函数中初始化
    // rclcpp::Client<rm_ros_interfaces::srv::GetDH>::SharedPtr get_dh_client_;

    // ScLERP规划器接口实例
    std::unique_ptr<sclerp_interface::ScLERPInterface> planner_interface_;
    
    // 规划的轨迹
    trajectory_msgs::msg::JointTrajectory planned_trajectory_;
    

    // 轨迹执行相关变量
    bool trajectory_executing_ = false;
    bool trajectory_planned_ = false;  // 添加轨迹规划状态标志
    size_t trajectory_point_index_ = 0;
    
    
    // 当前关节位置（从/joint_states话题获取）
    std::array<double, 7> current_joint_positions_;
    std::mutex joint_state_mutex_;  // 保护current_joint_positions_的互斥量
    
    // 存储最新接收到的盒子中心点位姿
    geometry_msgs::msg::PoseStamped latest_box_pose_;
    std::mutex pose_mutex_;  // 保护latest_box_pose_的互斥量
    bool new_pose_received_ = false;  // 标记是否有新位姿到达

    bool first_received_ = false;
    // 轨迹点
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points_;
    size_t current_point_index_ = 0;
    // 工作起点关节角度数组，单位：度
    // std::vector<double> initial_joint_position = {6.004, 94.961, -126.276, -111.946, -2.235, -59.221, -124.293};


    // DH参数相关
    // std::vector<double> dh_params_;
    // bool dh_params_received_ = false;

    // 法兰坐标系到吸盘工具坐标系的静态变换矩阵
    // 吸盘底面中心点距离法兰中心点沿Z轴正向0.1318mm
    Eigen::Matrix4d matrix_flange2sucker = []() {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform(2, 3) = 0.1318; // Z轴平移0.1318米
        return transform;
    }();

    std::unique_ptr<RmInterface::RmRobotAPI> rmRobotAPI_;

// 📦 #region realman API 相关接口二次封装成员函数 
    void initialize_rmRobot();  

    // 关节透传控制成员函数
    void rm_movej_canfd(const std::vector<double>& joint_positions);

    // 笛卡尔空间直线运动
    // 笛卡尔空间直线运动
    void rm_movel_cmd(const std::vector<float>& tcp_pose, int speed);

    // 获取关节状态信息
    void rm_get_arm_state(rm_current_arm_state_t& state);
// #endregion

    // 演示测试 
    void tempForShow();

    /**
    * @brief 关节状态回调函数
    *
    * 当收到关节状态消息时，该函数将被调用。
    *
    * @param msg 关节状态消息指针
    */
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 检查消息中是否包含7个关节的位置数据
        if (msg->position.size() >= 7) 
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            for (int i = 0; i < 7; i++) 
            {
                current_joint_positions_[i] = msg->position[i];
            }
            // 添加调试信息，确认收到关节状态更新
            RCLCPP_DEBUG(this->get_logger(), "收到关节状态更新: [%f, %f, %f, %f, %f, %f, %f]",
                        current_joint_positions_[0], current_joint_positions_[1], current_joint_positions_[2],
                        current_joint_positions_[3], current_joint_positions_[4], current_joint_positions_[5],
                        current_joint_positions_[6]);

        } else 
        {
            RCLCPP_WARN(this->get_logger(), "收到的关节状态数据不足7个关节，当前数量: %zu", msg->position.size());
        }
    }
        
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

    /**
     * @brief 主动从话题中获取一次位姿数据
     * 
     * @return std::shared_ptr<geometry_msgs::msg::PoseStamped> 获取到的位姿数据，如果超时则返回空指针
     */
    std::shared_ptr<geometry_msgs::msg::PoseStamped> getLatestPoseFromTopic();

    // 同步阻塞方式调用服务（会阻塞当前线程直到完成）
    void call_service_sync(const std::string& cmd) {
        RCLCPP_INFO(this->get_logger(), "执行命令: %s", cmd.c_str());
        int result = system(cmd.c_str());
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "服务调用成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用失败");
        }
    }

    // 检查服务是否可用的函数
    bool is_service_available(const std::string& service_name) {
        std::string cmd = "bash -c 'source /opt/ros/humble/setup.bash && ros2 service list | grep \"" + 
                        service_name + "\"'";
        
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            return false;
        }
        
        char buffer[256];
        std::string result = "";
        while (fgets(buffer, sizeof buffer, pipe) != nullptr) {
            result += buffer;
        }
        
        pclose(pipe);
        
        // 如果结果包含服务名称，则服务可用
        return result.find(service_name) != std::string::npos;
    }

    // 带重试机制的服务调用函数
    bool call_service_with_retry(const std::string& service_name,
                                const std::string& service_type,
                                const std::string& request_data = "{}",
                                int max_retries = 5,
                                int retry_delay_ms = 1000) {
        // 首先检查服务是否可用
        for (int i = 0; i < max_retries; ++i) {
            if (is_service_available(service_name)) {
                RCLCPP_INFO(this->get_logger(), "服务 %s 已就绪", service_name.c_str());
                break;
            } else {
                RCLCPP_WARN(this->get_logger(), "服务 %s 尚未就绪，等待中... (%d/%d)", 
                        service_name.c_str(), i+1, max_retries);
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
            }
        }
        
        // 执行服务调用
        std::string cmd = "bash -c 'source /opt/ros/humble/setup.bash && source " + 
                        std::string(getenv("HOME")) + "/.bashrc && timeout 10 ros2 service call " + 
                        service_name + " " + service_type + " \"" + request_data + "\"'";
        
        RCLCPP_INFO(this->get_logger(), "执行服务调用命令: %s", cmd.c_str());
        
        int result = system(cmd.c_str());
        
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "服务调用成功: %s", service_name.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用失败: %s, 返回码: %d", service_name.c_str(), result);
            return false;
        }
    }
// /*
    // 初始化定时器回调函数
    void init_timer_callback()
    {

        
        // 初始化机械臂
        initialize_rmRobot();

#pragma region 获取目标物体位姿信息
        // 方案A


        // // 方案B：
        // auto pose_msg = getLatestPoseFromTopic();
        // if (pose_msg) {
        //     RCLCPP_INFO(this->get_logger(), "主动获取到盒子位姿 - 位置: [%f, %f, %f], 方向: [%f, %f, %f, %f]",
        //                 pose_msg->pose.position.x,
        //                 pose_msg->pose.position.y,
        //                 pose_msg->pose.position.z,
        //                 pose_msg->pose.orientation.x,
        //                 pose_msg->pose.orientation.y,
        //                 pose_msg->pose.orientation.z,
        //                 pose_msg->pose.orientation.w);
            
        //     // 存储到类成员变量中供后续使用
        //     {
        //         std::lock_guard<std::mutex> lock(pose_mutex_);
        //         latest_box_pose_ = *pose_msg;
        //         new_pose_received_ = true;
        //     }
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "主动获取盒子中心点位姿超时，未接收到数据");
        // }

        // 方案C
        // bool pose_received = false;
        // int max_wait_count = 50; // 最多等待5秒(50*100ms)
        // int wait_count = 0;
        
        // while (!pose_received && wait_count < max_wait_count) {
        //     {
        //         std::lock_guard<std::mutex> lock(pose_mutex_);
        //         if (new_pose_received_) {
        //             RCLCPP_INFO(this->get_logger(), "在 init_timer_callback 中获取到最新盒子位姿 - 位置: [%f, %f, %f], 方向: [%f, %f, %f, %f]",
        //                         latest_box_pose_.pose.position.x,
        //                         latest_box_pose_.pose.position.y,
        //                         latest_box_pose_.pose.position.z,
        //                         latest_box_pose_.pose.orientation.x,
        //                         latest_box_pose_.pose.orientation.y,
        //                         latest_box_pose_.pose.orientation.z,
        //                         latest_box_pose_.pose.orientation.w);
        //             new_pose_received_ = false; // 重置标志
        //             pose_received = true;
        //         }
        //     }
            
        //     if (!pose_received) {
        //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //         wait_count++;
        //     }
        // }
        
        // if (!pose_received) {
        //     RCLCPP_ERROR(this->get_logger(), "在 init_timer_callback 中等待超时，未接收到盒子中心点位姿，节点将退出");
        //     rclcpp::shutdown();
        //     return;
        // }

        // 硬编码用于测试作业流程
        // latest_box_pose_.pose.position.x = -0.37405;
        // latest_box_pose_.pose.position.y = -0.3109;
        // latest_box_pose_.pose.position.z = 0.003628;
        // latest_box_pose_.pose.orientation.x = 0.12225;
        // latest_box_pose_.pose.orientation.y = -0.68583;
        // latest_box_pose_.pose.orientation.z = -0.1303;
        // latest_box_pose_.pose.orientation.w = 0.7054;
#pragma endregion

        // 初始化规划器
        initialize_planner();
    
        // tempShow();

        // 取消初始化定时器
        init_timer_->cancel();

        // 移除rclcpp::shutdown()调用，保持节点运行以继续监听话题
        // rclcpp::shutdown();
    }
// */

//#region 轨迹规划与执行定时器回调（旧版）
/*
    void timer_callback()
    {
        // 只有在轨迹尚未规划时才执行规划
        // @note : 测试从URDF文件中解析DH参数时使用，测试完成后注释
        // if (dh_params_received_ && !trajectory_planned_)
        if (!trajectory_planned_) 
        {
            // 执行规划
            plan_trajectory();
        }
    }


    void trajectory_timer_callback(){
        // 如果正在执行轨迹
        if (trajectory_executing_ && !planned_trajectory_.points.empty()){
            // 检查是否还有轨迹点需要发布
            if (trajectory_point_index_ < planned_trajectory_.points.size()) 
            {
                // 获取当前轨迹点数据
                const auto& point = planned_trajectory_.points[trajectory_point_index_];
                
                // 创建消息并填充数据
                auto joint_msg = rm_ros_interfaces::msg::Jointpos();
                joint_msg.dof = 7;
                joint_msg.follow = true;
                joint_msg.expand = 0.0;
                
                // 填充关节位置数据
                if (point.positions.size() >= 7) 
                {

                    joint_msg.joint.assign(point.positions.begin(), point.positions.begin() + 7);
                    
                    // 发布消息
                    // 📝 🧪 #region 测试运动学接口功能
                    // 📝 @note 若注释：，不执行轨迹点运动功能
                    // 📝 @note 上文的节点构造函数中的注释保持同步
                    joint_pos_publisher_->publish(joint_msg);
                    // #endregion
                    
                    RCLCPP_DEBUG(this->get_logger(), "发布轨迹点[%d]: [%f, %f, %f, %f, %f, %f, %f]",
                                static_cast<int>(trajectory_point_index_),
                                joint_msg.joint[0], joint_msg.joint[1], joint_msg.joint[2],
                                joint_msg.joint[3], joint_msg.joint[4], joint_msg.joint[5],
                                joint_msg.joint[6]);
                }
                
                // 增加索引
                trajectory_point_index_++;
            }else{
                // 轨迹执行完成
                trajectory_executing_ = false;
                RCLCPP_INFO(this->get_logger(), "轨迹执行完成");
            
                // 吸附物体并到达交付位置点
                // /*    
                // 激活吸盘
                std::string jodell_activate_cmd = "ros2 service call /right_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \""
                                    "{}\"";
                call_service_sync(jodell_activate_cmd);
     
                // 延时一段时间让吸盘工作
                std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
                
                // 上升运动
                // 创建状态结构体
                rm_current_arm_state_t rm_robot_current_state;
                rm_get_arm_state(rm_robot_current_state);
                std::vector<float> gripper_tcp_pose(6, 0.0f);
                gripper_tcp_pose = {rm_robot_current_state.pose.position.x + 0.15f,
                                    rm_robot_current_state.pose.position.y,
                                    rm_robot_current_state.pose.position.z,
                                    rm_robot_current_state.pose.euler.rx,
                                    rm_robot_current_state.pose.euler.ry,
                                    rm_robot_current_state.pose.euler.rz
                            };
                RCLCPP_INFO(this->get_logger(), "旋转前位姿欧拉角: [%f, %f, %f]",
                            gripper_tcp_pose[3], gripper_tcp_pose[4], gripper_tcp_pose[5]);
                rm_movel_cmd(gripper_tcp_pose, 30);

                // 翻转物体实现换手
                Eigen::Vector3f gripper_orientation(gripper_tcp_pose[3], gripper_tcp_pose[4], gripper_tcp_pose[5]); // 翻转向量

                Eigen::Matrix3f RotYMatrix, gripper_RotMatrix_current,  gripper_RotMatrix_target;
                RotYMatrix = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY());
                gripper_RotMatrix_current = eulerToRotationMatrix(gripper_orientation);

                gripper_RotMatrix_target = gripper_RotMatrix_current * RotYMatrix;

                gripper_orientation = rotationMatrixToEuler(gripper_RotMatrix_target);
                gripper_tcp_pose[3] = gripper_orientation[0];
                gripper_tcp_pose[4] = gripper_orientation[1];
                gripper_tcp_pose[5] = gripper_orientation[2];
                RCLCPP_INFO(this->get_logger(), "旋转后位姿欧拉角: [%f, %f, %f]",
                            gripper_tcp_pose[3], gripper_tcp_pose[4], gripper_tcp_pose[5]);
                rm_movel_cmd(gripper_tcp_pose, 100);


                // // 延时一段时间让吸盘工作
                std::this_thread::sleep_for(std::chrono::milliseconds(5000)); 

                std::string jodell_deactivate_cmd = "ros2 service call /right_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \""
                                    "{}\"";
                call_service_sync(jodell_deactivate_cmd);

                std::string jodell_CloseModbus_cmd = "ros2 service call /right_arm/close_modbus rm_ros_interfaces/srv/CloseModbus \""
                                    "{}\"";
                call_service_sync(jodell_CloseModbus_cmd);


                // // 发布最后一个轨迹点几次以确保到位
                // if (!planned_trajectory_.points.empty()) 
                // {
                //     const auto& last_point = planned_trajectory_.points.back();
                //     if (last_point.positions.size() >= 7) 
                //     {
                //         auto joint_msg = rm_ros_interfaces::msg::Jointpos();
                //         joint_msg.dof = 7;
                //         joint_msg.follow = true;
                //         joint_msg.expand = 0.0;
                //         joint_msg.joint.assign(last_point.positions.begin(), last_point.positions.begin() + 7);
                        
                //         // 发布5次最终位置以确保到位
                //         for (int i = 0; i < 5; i++) {
                //             joint_pos_publisher_->publish(joint_msg);
                //             std::this_thread::sleep_for(std::chrono::milliseconds(10));
                //         }
                //     }
                // }
                
                // 等待一小段时间确保关节状态更新
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                // 获取并打印当前关节角度
                {
                    std::lock_guard<std::mutex> lock(joint_state_mutex_);
                    RCLCPP_INFO(this->get_logger(), "执行完成后的当前关节角度: [%f, %f, %f, %f, %f, %f, %f]",
                                current_joint_positions_[0], 
                                current_joint_positions_[1], 
                                current_joint_positions_[2],
                                current_joint_positions_[3], 
                                current_joint_positions_[4], 
                                current_joint_positions_[5],
                                current_joint_positions_[6] 
                            );
                    // 停止轨迹执行
                    // trajectory_executing_ = false;

                    // 添加调试信息，检查是否有收到关节状态数据
                    bool all_zeros = true;
                    for (int i = 0; i < 7; i++) 
                    {
                        if (current_joint_positions_[i] != 0.0) {
                            all_zeros = false;
                            break;
                        }
                    }
                    
                    if (all_zeros)
                    {
                        RCLCPP_WARN(this->get_logger(), "警告：当前关节角度全为零，可能未收到来自机械臂的关节状态更新");
                        RCLCPP_WARN(this->get_logger(), "请检查：1. rm_driver是否正常运行 2. /rm_driver/joint_pos话题是否发布数据");
                    }
                }
                
                // 关闭节点
                // rclcpp::shutdown();
            }
        }
    }

*/ 
//#endregion    


// #region 轨迹规划与执行定时器回调 (改进)
  /*
    // 轨迹规划定时器回调函数
    void timer_callback()
    {
        // 只有在轨迹尚未规划时才执行规划

        if (!trajectory_planned_) 
        {
            RCLCPP_INFO(this->get_logger(), "开始规划轨迹");

            Eigen::Quaterniond quaternion;
            Eigen::Vector3d tcpPos;
            
            // Solution 01
            // 从 /right_posedemo 中获取纸箱中心点位姿
            // auto pose_msg = getLatestPoseFromTopic();
            // if (pose_msg) 
            // {
            //     RCLCPP_INFO(this->get_logger(), "主动获取到盒子位姿 - 位置: [%f, %f, %f], 方向: [%f, %f, %f, %f]",
            //                 pose_msg->pose.position.x,
            //                 pose_msg->pose.position.y,
            //                 pose_msg->pose.position.z,
            //                 pose_msg->pose.orientation.x,
            //                 pose_msg->pose.orientation.y,
            //                 pose_msg->pose.orientation.z,
            //                 pose_msg->pose.orientation.w);
        
            //     // 存储到类成员变量中供后续使用
            //     {
            //         std::lock_guard<std::mutex> lock(pose_mutex_);
            //         latest_box_pose_ = *pose_msg;
            //         new_pose_received_ = true;
            //     }
                
            //     // 提取四元数和位置数据
            //     quaternion = Eigen::Quaterniond(
            //         pose_msg->pose.orientation.w,
            //         pose_msg->pose.orientation.x,
            //         pose_msg->pose.orientation.y,
            //         pose_msg->pose.orientation.z);
                
            //     tcpPos = Eigen::Vector3d(
            //         pose_msg->pose.position.x + 0.09 + 0.1318,
            //         pose_msg->pose.position.y,
            //         pose_msg->pose.position.z);
            // } else {
            //     RCLCPP_WARN(this->get_logger(), "主动获取盒子中心点位姿超时，未接收到数据");
            //     //调试用
            //     return; 
            // }

            // Solution02:
            // 提取四元数和位置数据
            quaternion = Eigen::Quaterniond(
                latest_box_pose_.pose.orientation.w,
                latest_box_pose_.pose.orientation.x,
                latest_box_pose_.pose.orientation.y,
                latest_box_pose_.pose.orientation.z);
            
            tcpPos = Eigen::Vector3d(
                latest_box_pose_.pose.position.x + 0.075 + 0.1318,
                latest_box_pose_.pose.position.y,
                latest_box_pose_.pose.position.z);            

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
        }
    }

    // 轨迹发布定时器回调函数
    void trajectory_timer_callback()
    {
        if (trajectory_executing_ && !planned_trajectory_.points.empty()) 
        {
            // 发布轨迹
            publishJointTrajectory(planned_trajectory_,execution_rate);
            
            // 轨迹执行完成
            trajectory_executing_ = false;
            RCLCPP_INFO(this->get_logger(), "轨迹执行完成");
            
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
            
          

            // 等待一小段时间确保关节状态更新
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // 获取并打印当前关节角度
            // {
            //     std::lock_guard<std::mutex> lock(joint_state_mutex_);
            //     RCLCPP_INFO(this->get_logger(), "执行完成后的当前关节角度: [%f, %f, %f, %f, %f, %f, %f]",
            //                 current_joint_positions_[0], 
            //                 current_joint_positions_[1], 
            //                 current_joint_positions_[2],
            //                 current_joint_positions_[3], 
            //                 current_joint_positions_[4], 
            //                 current_joint_positions_[5],
            //                 current_joint_positions_[6] 
            //             );
                
            //     // 添加调试信息，检查是否有收到关节状态数据
            //     bool all_zeros = true;
            //     for (int i = 0; i < 7; i++) 
            //     {
            //         if (current_joint_positions_[i] != 0.0) {
            //             all_zeros = false;
            //             break;
            //         }
            //     }
                
            //     if (all_zeros)
            //     {
            //         RCLCPP_WARN(this->get_logger(), "警告：当前关节角度全为零，可能未收到来自机械臂的关节状态更新");
            //         RCLCPP_WARN(this->get_logger(), "请检查：1. rm_driver是否正常运行 2. /rm_driver/joint_pos话题是否发布数据");
            //     }

            // }
            
            trajectory_timer_->cancel();

            // 移除rclcpp::shutdown()调用，保持节点运行以继续监听话题
            // rclcpp::shutdown();
        }
    }
  */
// #endregion
   
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


    // #region 初始化路径规划器（不强制指定urdf_path)

        // // 如果提供了 URDF 路径，则尝试从 URDF 文件中获取实际的链接名称
        // if (!urdf_path.empty()) 
        // {
        //     std::string actual_base_link, actual_tip_link;
        //     if (getLinkNamesFromURDF(urdf_path, actual_base_link, actual_tip_link)) {
        //         base_link = actual_base_link;
        //         tip_link = actual_tip_link;
        //         RCLCPP_INFO(this->get_logger(), "从 URDF 文件中获取链接名称: base='%s', tip='%s'", 
        //                     base_link.c_str(), tip_link.c_str());
        //     } else {
        //         RCLCPP_WARN(this->get_logger(), "无法从 URDF 文件中获取链接名称，使用默认值: base='%s', tip='%s'", 
        //                     base_link.c_str(), tip_link.c_str());
        //     }
        // } else {
        //     RCLCPP_INFO(this->get_logger(), "未提供 URDF 路径，使用默认链接名称: base='%s', tip='%s'", 
        //                 base_link.c_str(), tip_link.c_str());
        // }
        // // 创建ScLERP运动规划器实例
        // try 
        // {
        //     if (!urdf_path.empty()) {
        //         RCLCPP_INFO(this->get_logger(), "使用指定的URDF文件: %s", urdf_path.c_str());

        //         planner_interface_ = std::make_unique<sclerp_interface::ScLERPInterface>(
        //             base_link, tip_link, shared_from_this(), urdf_path);
        //     } else {
        //         RCLCPP_INFO(this->get_logger(), "使用默认的MoveIt参数服务器");
                
        //         planner_interface_ = std::make_unique<sclerp_interface::ScLERPInterface>(
        //             base_link, tip_link, shared_from_this(), "");
        //     }
            
        //     if (!planner_interface_) {
        //         RCLCPP_ERROR(this->get_logger(), "无法创建规划器接口实例");
        //     } else {
        //         RCLCPP_INFO(this->get_logger(), "成功创建规划器接口实例");
        //     }

        // } catch (const std::exception& e) {
        //     RCLCPP_ERROR(this->get_logger(), "创建规划器接口时出错: %s", e.what());
        //     planner_interface_ = nullptr;
        // }    

    // #endregion

    
    }


    void sclerp_planner(const Eigen::VectorXd &jointAngles_start,
                        const Eigen::Matrix4d &matrix_base2tcp_target,
                        trajectory_msgs::msg::JointTrajectory &joint_trajectory )
    {

        // 检查 planner_interface_ 是否已正确初始化
        if (!planner_interface_) {
            RCLCPP_ERROR(this->get_logger(), "sclerp_planner：规划器接口未初始化");
            return;
        }

        // 使用solve方法生成轨迹数据
        bool success = false;
        
    
        try {
            success = planner_interface_->solve(jointAngles_start, matrix_base2tcp_target, joint_trajectory);
            
            RCLCPP_INFO(this->get_logger(), "规划器返回结果: %s", success ? "成功" : "失败");
            RCLCPP_INFO(this->get_logger(), "规划成功，轨迹点数量: %ld", joint_trajectory.points.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "调用规划器solve方法时出错: %s", e.what());
            return;
        }
        
        if (success && !joint_trajectory.points.empty()) 
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


/**
 * @brief 发布关节轨迹到机械臂驱动节点
 * 
 * @param trajectory 要发布的关节轨迹数据
 */
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
    
    // 轨迹规划函数（正式版）
    void plan_trajectory()
    {
        // 检查planner_interface_是否已正确初始化
        if (!planner_interface_) {
            RCLCPP_ERROR(this->get_logger(), "plan_trajectory：规划器接口未初始化");
            return;
        }
        
        
        // 获取参数
        std::string start_joints_str, target_joints_str;       
        this->get_parameter("start_joint_positions", start_joints_str);
        this->get_parameter("target_joint_positions", target_joints_str);
        // 打印参数
        RCLCPP_INFO(this->get_logger(), "原始起始关节角度字符串: %s", start_joints_str.c_str());
        RCLCPP_INFO(this->get_logger(), "原始目标关节角度字符串: %s", target_joints_str.c_str());       
        // 解析目标关节位置参数
        Eigen::VectorXd jointAngles_target(7);
        parse_joint_positions(target_joints_str, jointAngles_target);
        // 解析起始关节位置参数
        Eigen::VectorXd jointAngles_start(7);
        parse_joint_positions(start_joints_str, jointAngles_start);
        // 打印起始和目标关节位置参数
        RCLCPP_INFO(this->get_logger(), "开始关节角度: [%f, %f, %f, %f, %f, %f, %f]",
                    jointAngles_start(0), jointAngles_start(1), jointAngles_start(2),
                    jointAngles_start(3), jointAngles_start(4), jointAngles_start(5), jointAngles_start(6));
                    
        RCLCPP_INFO(this->get_logger(), "目标关节角度: [%f, %f, %f, %f, %f, %f, %f]",
                    jointAngles_target(0), jointAngles_target(1), jointAngles_target(2),
                    jointAngles_target(3), jointAngles_target(4), jointAngles_target(5), jointAngles_target(6));

        // 末端执行器的目标位姿矩阵（4x4齐次变换矩阵）
        Eigen::Matrix4d matrix_base2tcp_target;
        
        // 计算目标关节角度对应的末端执行器位姿（作为运动规划的目标位姿）        
        // 使用规划器接口计算正向运动学
        try {
            planner_interface_->kinlib_solver_.getFK(jointAngles_target, matrix_base2tcp_target);
            // 打印 matrix_base2tcp_target 矩阵
            // 机械臂基座到末端法兰坐标系（Arm_tip)
            std::stringstream ss;
            RCLCPP_INFO(this->get_logger(), "机械臂基座到末端法兰坐标系的齐次矩阵:");
            ss << "\n matrix_base2tcp_target: \n" << matrix_base2tcp_target << std::endl;
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
            

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "计算正向运动学时出错: %s", e.what());
            return;
        }

        // 检验getFK 函数的准确性
        RCLCPP_INFO(this->get_logger(), "=== 第三方库 Kinlib::getFK 正运动学接口计算结果 ===");
        // 将齐次矩阵转换为6x1列向量并打印
        Eigen::Matrix<double, 6, 1> tcp_pose_vector;
        // 提取平移部分（前3个元素）
        tcp_pose_vector.block<3, 1>(0, 0) = matrix_base2tcp_target.block<3, 1>(0, 3);
        // 提取旋转部分并转换为欧拉角（后3个元素）
        Eigen::Matrix3d rotation_matrix = matrix_base2tcp_target.block<3, 3>(0, 0);
        double roll = atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
        double pitch = atan2(-rotation_matrix(2, 0), sqrt(rotation_matrix(2, 1)*rotation_matrix(2, 1) + rotation_matrix(2, 2)*rotation_matrix(2, 2)));
        double yaw = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
        // ZYX 内旋欧拉角，与 Realman Robot TCP 姿态表示一致
        tcp_pose_vector(3) = roll;
        tcp_pose_vector(4) = pitch;
        tcp_pose_vector(5) = yaw;
        // 打印6x1列向量
        RCLCPP_INFO(this->get_logger(), "末端执行器目标位姿 (x,y,z,roll,pitch,yaw): [%f, %f, %f, %f, %f, %f]", 
                    tcp_pose_vector(0), tcp_pose_vector(1), tcp_pose_vector(2),
                    tcp_pose_vector(3), tcp_pose_vector(4), tcp_pose_vector(5)); 


        // 使用solve方法生成轨迹数据
        trajectory_msgs::msg::JointTrajectory joint_trajectory;
        bool success = false;
        
        // 在调用solve前，检查关节限制
        RCLCPP_INFO(this->get_logger(), "检查目标关节角度是否在限制范围内:");
        for (int i = 0; i < 7; i++) {
            double angle = jointAngles_target(i);
            RCLCPP_INFO(this->get_logger(), "  关节%d: %f radians (%f degrees)", i+1, angle, angle * 180.0 / M_PI);
        }
    
        try {
            success = planner_interface_->solve(jointAngles_start, matrix_base2tcp_target, joint_trajectory);
            
            RCLCPP_INFO(this->get_logger(), "规划器返回结果: %s", success ? "成功" : "失败");
            RCLCPP_INFO(this->get_logger(), "规划成功，轨迹点数量: %ld", joint_trajectory.points.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "调用规划器solve方法时出错: %s", e.what());
            return;
        }
        
        if (success && !joint_trajectory.points.empty()) 
        {
            // RCLCPP_INFO(this->get_logger(), "规划成功，轨迹点数量: %ld", joint_trajectory.points.size());
            
            // 直接将规划器生成的轨迹点复制到 planned_trajectory_ 中
            // 这样即使注释掉插值和时间戳设置部分代码，planned_trajectory_ 仍然有数据
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

    /**
     * @brief 解析关节位置字符串参数
     * 
     * 该函数将形如 "0.1,0.2,0.3,0.4,0.5,0.6,0.7" 的字符串解析为Eigen向量
     * 
     * @param str 包含关节位置的字符串，各关节位置用逗号分隔
     * @param joints 输出参数，存储解析后的关节位置值的Eigen向量
     */
    void parse_joint_positions(const std::string& str, Eigen::VectorXd& joints)
    {
        // 确保向量大小正确
        if (joints.size() != 7) {
            joints.resize(7);
        }
        
        std::stringstream ss(str);
        std::string item;
        int i = 0;
        while (std::getline(ss, item, ',') && i < joints.size()) {
            try {
                // 清除空格
                item.erase(0, item.find_first_not_of(' '));
                item.erase(item.find_last_not_of(' ') + 1);
                // 尝试将字符串转换为double值
                joints(i) = std::stod(item);
            } catch (const std::exception& e) {
                // 转换失败时，默认设置为0.0
                joints(i) = 0.0;
                RCLCPP_WARN(this->get_logger(), "无法解析关节值 '%s'，设置为0.0: %s", item.c_str(), e.what());
            }
            i++;
        }
        
        // 如果解析的值少于7个，将剩余的值设置为0.0
        while (i < joints.size()) {
            joints(i) = 0.0;
            i++;
        }
    }
 

    // 从URDF文件中获取基座和末端执行器链接名称  
    bool getLinkNamesFromURDF(const std::string& urdf_path, std::string& base_link, std::string& tip_link) {
        urdf::Model urdf_model;
        
        if (!urdf_model.initFile(urdf_path)) {
            RCLCPP_ERROR(this->get_logger(), "无法加载 URDF 文件: %s", urdf_path.c_str());
            return false;
        }
        
        // 获取根链接作为基座链接
        base_link = urdf_model.getRoot()->name;
        
        // 查找最后一个链接作为末端执行器链接
        std::string last_link = base_link;
        for (const auto& link : urdf_model.links_) {
            // 查找没有子链接的链接作为末端链接
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

    
    /**
     * @brief 将四元数和位置向量转换为4x4齐次变换矩阵
     * 
     * @param quaternion 四元数(w, x, y, z)
     * @param tcpPos 位置向量(x, y, z)
     * @return Eigen::Matrix4d 4x4齐次变换矩阵
     */
    Eigen::Matrix4d tcpPose2HomogenMatrix(const Eigen::Quaterniond& quaternion, const Eigen::Vector3d& tcpPos)
    {
        Eigen::Matrix4d homogenMatrix = Eigen::Matrix4d::Identity();
        
        // 设置位置部分
        homogenMatrix(0, 3) = tcpPos[0];  // x
        homogenMatrix(1, 3) = tcpPos[1];  // y
        homogenMatrix(2, 3) = tcpPos[2];  // z
        
        // 从四元数构建旋转矩阵
        Eigen::Matrix3d rotation = quaternion.normalized().toRotationMatrix();
        
        // 将旋转矩阵填充到齐次变换矩阵中
        homogenMatrix.block<3, 3>(0, 0) = rotation;
        
        return homogenMatrix;
    }

  
     /**
     * @brief 将6D位姿（x,y,z,Rx,Ry,Rz）转换为4x4齐次变换矩阵
     * 
     * @param tcpPose 6D位姿向量，包含位置(x,y,z)和欧拉角(Rx,Ry,Rz)
     * @return Eigen::Matrix4d 4x4齐次变换矩阵
     */
    Eigen::Matrix4d tcpPose2HomogenMatrix(const Eigen::Matrix<double, 6, 1>& tcpPose)
    {
        Eigen::Matrix4d homogenMatrix = Eigen::Matrix4d::Identity();
        
        // 提取位置部分
        homogenMatrix(0, 3) = tcpPose[0];  // x
        homogenMatrix(1, 3) = tcpPose[1];  // y
        homogenMatrix(2, 3) = tcpPose[2];  // z
        
        // 提取欧拉角（RPY顺序：Rx, Ry, Rz 对应 Roll, Pitch, Yaw）
        double roll = tcpPose[3];   // Rx
        double pitch = tcpPose[4];  // Ry
        double yaw = tcpPose[5];    // Rz
        
        // 计算旋转矩阵（XYZ顺序，即RPY顺序）
        Eigen::Matrix3d rotation;
        double cr = cos(roll);
        double sr = sin(roll);
        double cp = cos(pitch);
        double sp = sin(pitch);
        double cy = cos(yaw);
        double sy = sin(yaw);
        
        // 正确的XYZ旋转顺序公式（Roll-Pitch-Yaw）
        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        rotation(0, 0) = cp * cy;
        rotation(0, 1) = -cr * sy + sr * sp * cy;
        rotation(0, 2) = sr * sy + cr * sp * cy;
        
        rotation(1, 0) = cp * sy;
        rotation(1, 1) = cr * cy + sr * sp * sy;
        rotation(1, 2) = -sr * cy + cr * sp * sy;
        
        rotation(2, 0) = -sp;
        rotation(2, 1) = sr * cp;
        rotation(2, 2) = cr * cp;
        
        // 将旋转矩阵填充到齐次变换矩阵中
        homogenMatrix.block<3, 3>(0, 0) = rotation;
        
        return homogenMatrix;
    }
  
        // ZYX内旋欧拉角转旋转矩阵（左乘顺序：R = Rz * Ry * Rx）
    Eigen::Matrix3f eulerToRotationMatrix(const Eigen::Vector3f& euler) {
        float roll  = euler[0];   // X轴旋转角(roll)
        float pitch = euler[1];  // Y轴旋转角(pitch)
        float yaw   = euler[2];    // Z轴旋转角(yaw)
        
        // 各轴旋转矩阵
        Eigen::Matrix3f Rz, Ry, Rx;
        Rz = Eigen::AngleAxisf(yaw,   Eigen::Vector3f::UnitZ());
        Ry = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());
        Rx = Eigen::AngleAxisf(roll,  Eigen::Vector3f::UnitX());
        
        // ZYX内旋：R = Rz * Ry * Rx
        return Rz * Ry * Rx;
    }


    // 旋转矩阵转ZYX内旋欧拉角（返回值：[yaw, pitch, roll]）
    Eigen::Vector3f rotationMatrixToEuler(const Eigen::Matrix3f& R) {
        float yaw, pitch, roll;
        
        // 计算pitch（范围：-π/2 ~ π/2）
        pitch = atan2(-R(2,0), sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)));
        
        // 计算yaw和roll
        if (fabs(pitch - M_PI/2) < 1e-6) {
            // 奇异点：pitch = 90度
            yaw = 0;
            roll = yaw + atan2(R(0,1), R(1,1));
        } else if (fabs(pitch + M_PI/2) < 1e-6) {
            // 奇异点：pitch = -90度
            yaw = 0;
            roll = -yaw + atan2(-R(0,1), R(1,1));
        } else {
            // 正常情况
            yaw = atan2(R(1,0), R(0,0));
            roll = atan2(R(2,1), R(2,2));
        }
        
        return Eigen::Vector3f(roll, pitch, yaw);
    }


    /**
     * @brief 将轨迹点关节位置信息保存到文件
     * 
     * @param joint_trajectory 轨迹数据
     * @param filename 保存文件名
     */
    void saveJointTrajectory(const trajectory_msgs::msg::JointTrajectory& joint_trajectory, 
                             const std::string& filename = "joint_trajectoryPoints.txt") 
    {
        // 指定保存文件的目录
        std::string directory = "/home/byd/hrt2/ros2_rm_ws/src/ros2_rm_robot-humble/rm_motion_planner/Data";
        
        // 创建目录（如果不存在）
        std::string mkdir_cmd = "mkdir -p " + directory;
        int mkdir_result = system(mkdir_cmd.c_str());
        if (mkdir_result != 0) {
            RCLCPP_WARN(this->get_logger(), "创建目录失败: %s", directory.c_str());
        }
        
        // 构建完整文件路径
        std::string full_path = directory + "/" + filename;
        
        std::ofstream file(full_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件 %s 进行写入", full_path.c_str());
            return;
        }
        
        // 写入关节名称作为标题行
        if (!joint_trajectory.joint_names.empty()) {
            for (size_t i = 0; i < joint_trajectory.joint_names.size(); ++i) {
                file << joint_trajectory.joint_names[i];
                if (i < joint_trajectory.joint_names.size() - 1) {
                    file << ",";
                }
            }
            file << std::endl;
        }
        
        // 写入每个轨迹点的关节位置数据
        for (const auto& point : joint_trajectory.points) {
            if (!point.positions.empty()) {
                for (size_t i = 0; i < point.positions.size(); ++i) {
                    file << point.positions[i];
                    if (i < point.positions.size() - 1) {
                        file << ",";
                    }
                }
                file << std::endl;
            }
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "已将轨迹点关节位置信息保存到文件: %s", full_path.c_str());
    }
    
};

// #region realman robot interface 二次封装
void RmSclerpPlannerNode1::initialize_rmRobot() {
    try {
        // 初始化机械臂API
        rmRobotAPI_ = std::make_unique<RmInterface::RmRobotAPI>();
        // 使用默认IP地址初始化，如果需要可以修改为实际的机器人IP
        rmRobotAPI_->initialize_RmRobot("192.168.1.19");
        // rmRobotAPI_->initialize_RmRobot();        
        if (!rmRobotAPI_->isInitialized()) {
            RCLCPP_ERROR(this->get_logger(), "机械臂API初始化失败");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "机械臂API初始化成功");
        
        // 机械臂复位到初始位置
        // 注意：在实际使用中，确保机械臂周围没有障碍物再执行此操作
        // 遮挡相机
        // std::vector<double> initial_joint_position = {-52.958, 86.146, -21.246, 107.473, -54.920, 64.500, -2.646};
        // 不遮挡相机
        std::vector<double> initial_joint_position = {-44.811, 69.029, -48.806, 71.437, -40.008, 83.439, -62.740};
        int result = rmRobotAPI_->rm_movej(initial_joint_position, 50);
        
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "机械臂复位成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "机械臂复位失败，错误码: %d", result);
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "机械臂初始化过程中发生异常: %s", e.what());
    }
}


void RmSclerpPlannerNode1::rm_movej_canfd(const std::vector<double>& joint_positions) {
    if ( rmRobotAPI_ && rmRobotAPI_->isInitialized() ) 
    {
        int result = rmRobotAPI_->movej_canfd(joint_positions);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "发送关节角度失败，错误码: %d", result);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "机器人API未初始化");
    }
}

void RmSclerpPlannerNode1::rm_movel_cmd(const std::vector<float>& tcp_pose, int speed) 
{
    if ( rmRobotAPI_ && rmRobotAPI_->isInitialized() ) 
    {
        int result = rmRobotAPI_->rm_movel(tcp_pose, speed);
        if (result != 0) {}
    }else {
        RCLCPP_ERROR(this->get_logger(), "机器人API未初始化");
    }
}

void RmSclerpPlannerNode1::rm_get_arm_state(rm_current_arm_state_t& state) {
    if ( rmRobotAPI_ && rmRobotAPI_->isInitialized() ) 
    {
        int result = rmRobotAPI_->rm_get_arm_state(state);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "获取机械臂状态失败，错误码: %d", result);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "机器人API未初始化");
    }
}
// #endregion

std::shared_ptr<geometry_msgs::msg::PoseStamped> RmSclerpPlannerNode1::getLatestPoseFromTopic()
{
    auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    bool data_received = false;
    std::mutex data_mutex;

    // 创建一次性订阅器来主动获取话题数据
    auto one_time_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/right_posedemo", 1, 
        [&pose_msg, &data_received, &data_mutex](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (!data_received) {
                *pose_msg = *msg;
                data_received = true;
            }
        });

    // 等待一段时间以接收数据
    rclcpp::Time start_time = this->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(2.0); // 2秒超时

    while (!data_received && (this->now() - start_time) < timeout) {
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 销毁订阅器
    one_time_sub.reset();

    if (data_received) {
        return pose_msg;
    } else {
        return nullptr;
    }
}

void  RmSclerpPlannerNode1::tempForShow() {

    // 使用系统命令调用服务替代直接客户端调用
    std::string jodell_initialize_cmd = "ros2 service call /left_arm/initialize_gripper rm_ros_interfaces/srv/InitializeGripper \""
        "{ip: '192.168.1.18', port: 8080, register_address: 1000, comm_port: 1, device_address: 9}\"";
    call_service_sync(jodell_initialize_cmd); 

    std::vector<double> target_joint_position = {-67.902, -54.464, -56.450, -49.018, -138.739, 89.207, 73.986};
    rmRobotAPI_->rm_movej(target_joint_position, 50);

    rm_current_arm_state_t rm_robot_current_state;
    rm_get_arm_state(rm_robot_current_state);
    std::vector<float> gripper_tcp_pose(6, 0.0f);
    gripper_tcp_pose = {rm_robot_current_state.pose.position.x ,
                        rm_robot_current_state.pose.position.y,
                        rm_robot_current_state.pose.position.z - 0.110f,
                        rm_robot_current_state.pose.euler.rx,
                        rm_robot_current_state.pose.euler.ry,
                        rm_robot_current_state.pose.euler.rz
                };
    RCLCPP_INFO(this->get_logger(), "旋转前位姿欧拉角: [%f, %f, %f]",
                gripper_tcp_pose[3], gripper_tcp_pose[4], gripper_tcp_pose[5]);
    rm_movel_cmd(gripper_tcp_pose, 50);


    // // 延时一段时间让吸盘工作
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // 启动吸盘
    std::string jodell_activate_cmd = "ros2 service call /left_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \""
                        "{}\"";
    call_service_sync(jodell_activate_cmd);   
    // // 延时一段时间让吸盘工作
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // 运动到释放点
    std::vector<double> release_joint_position = {-113.882, -52.867, -33.962, -29.249, -55.023, 126.469, 105.515};
    rmRobotAPI_->rm_movej(release_joint_position, 50);        
    // 关闭吸盘
    std::string jodell_deactivate_cmd = "ros2 service call /left_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \""
                        "{}\"";
    call_service_sync(jodell_deactivate_cmd);
    // 关闭 Modbus
    std::string jodell_CloseModbus_cmd = "ros2 service call /left_arm/close_modbus rm_ros_interfaces/srv/CloseModbus \""
                        "{}\"";
    call_service_sync(jodell_CloseModbus_cmd);        
}


// 主函数保持不变
int main(int argc, char **argv)
{
    // 初始化ROS客户端库
    rclcpp::init(argc, argv);

    // 创建并初始化 RmSclerpPlannerNode1 节点
    auto planner_node = std::make_shared<RmSclerpPlannerNode1>();
    
    // 使用显式执行器管理节点，避免通过launch启动时重复添加到执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(planner_node);
    executor.spin();
    
    // 关闭ROS客户端库
    rclcpp::shutdown();

    // 返回 0 表示程序正常结束
    return 0;    
}

