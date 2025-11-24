/**
 * @brief realman robot 单臂控制节点
 * @note 开发调试版本(模块功能测试与演示版本)
 * @note *（左）机械臂IP：192.168.1.19
 * @updates 2025/9/16, 添加基于动作驱动的执行逻辑
 * @updates 2025/10/16,双臂间动作请求的执行函数中吸盘关闭逻辑改为动作执行的状态反馈
 * @updates 2025/10/16,新增吸盘异步非阻塞操作函数，并在作业流程的开始，结束以及换手环节调用

 */
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "kinlib/collision_utils.h"
#include "sclerp_motion_planner/utils.h"
#include "sclerp_motion_planner/interpolator.h"
#include "sclerp_motion_planner/sclerp_interface.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "rm_ros_interfaces/msg/movej.hpp"
#include "rm_ros_interfaces/msg/jointpos.hpp"
#include "sorting_robot_interfaces/action/grasp_box.hpp"
#include "sorting_robot_interfaces/action/place_box.hpp"
#include "sorting_robot_interfaces/action/trigger_arm.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// 添加 URDF 相关的头文件
// 添加 Eigen 头文件
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

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
#include "rm_ros_interfaces/srv/get_paras_gripper.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using GraspBox = sorting_robot_interfaces::action::GraspBox;
using PlaceBox = sorting_robot_interfaces::action::PlaceBox;
using TriggerArm = sorting_robot_interfaces::action::TriggerArm;
using GraspBoxHandle = rclcpp_action::ServerGoalHandle<GraspBox>;
using PlaceBoxHandle = rclcpp_action::ServerGoalHandle<PlaceBox>;
using TriggerArmHandle = rclcpp_action::ServerGoalHandle<TriggerArm>;


class RmSclerpPlannerNode1 : public rclcpp::Node
{
public:
    explicit RmSclerpPlannerNode1(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("rm_sclerp_planner_node_leftArm", options)
    {

    /************************ 参数申明与获取 ************************/
        // 声明参数
        this->declare_parameter<std::string>("start_joint_positions",  "-1.734, -0.859, -0.484, -1.210, -1.458, 2.116, 1.104");
        this->declare_parameter<std::string>("target_joint_positions", "-0.973, -1.125, -0.899, -0.978, -2.299, 1.304, 1.449");
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

        // 初始化机械臂及吸盘
        initialize_hardware_and_planner();

        
    /***************** 通信：话题，服务，动作 ******************/        
        // 发布者，用于直接向rm_driver发送轨迹点
        joint_pos_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Jointpos>(
            "/left_arm/rm_driver/movej_canfd_cmd", 20);         

        // 订阅者，用于获取当前关节位置
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/left_arm/joint_states", 20,
            std::bind(&RmSclerpPlannerNode1::joint_state_callback, this, std::placeholders::_1));

        
        // 订阅者，用于获取右机械臂状态   
        // rightArm_state_subscriber_ = this->create_subscription<rm_ros_interfaces::msg::ArmState>(
        //     "/left_arm/arm_state", 10,
        //     std::bind(&RmSclerpPlannerNode1::rightArm_state_callback, this, std::placeholders::_1));      
                  

        // 『动作服务端』
        // 抓取动作服务端
        grasp_box_server_ = rclcpp_action::create_server<GraspBox>(
            this,
            "grasp_action",
            std::bind(&RmSclerpPlannerNode1::handle_grasp_box_goal, this, _1, _2),
            std::bind(&RmSclerpPlannerNode1::handle_grasp_box_cancel, this, _1),
            std::bind(&RmSclerpPlannerNode1::handle_grasp_box_accepted, this, _1));

        // 放置动作服务端   
        place_box_server_ = rclcpp_action::create_server<PlaceBox>(
            this,   
            "place_action", 
            std::bind(&RmSclerpPlannerNode1::handle_place_box_goal, this, _1, _2),
            std::bind(&RmSclerpPlannerNode1::handle_place_box_cancel, this, _1),
            std::bind(&RmSclerpPlannerNode1::handle_place_box_accepted, this, _1));

        // 接收右臂触发信号的动作服务端
        trigger_left_arm_server_ = rclcpp_action::create_server<TriggerArm>(
            this,
            "trigger_arm_action",
            std::bind(&RmSclerpPlannerNode1::handle_trigger_left_arm_goal, this, _1, _2),
            std::bind(&RmSclerpPlannerNode1::handle_trigger_left_arm_cancel, this, _1),
            std::bind(&RmSclerpPlannerNode1::handle_trigger_left_arm_accepted, this, _1)
        );

        // 『动作客户端』
        // 以触发右臂任务
        trigger_right_arm_client_ = rclcpp_action::create_client<TriggerArm>(
            this,
            "/right_arm/trigger_arm_action"  // 连接到左臂的动作服务
        );  

        // 吸盘服务客户端的定义/初始化
        init_gripper_client_ = this->create_client<rm_ros_interfaces::srv::InitializeGripper>("/left_arm/getParas_gripper");

        activate_gripper_client_ = this->create_client<rm_ros_interfaces::srv::ActivateGripper>("/left_arm/activate_gripper");
        
        deactivate_gripper_client_ = this->create_client<rm_ros_interfaces::srv::DeactivateGripper>("/left_arm/deactivate_gripper");
        
        closeModbus_gripper_client_ = this->create_client<rm_ros_interfaces::srv::CloseModbus>("/left_arm/close_modbus");
        
        getParas_gripper_client_ = this->create_client<rm_ros_interfaces::srv::GetParasGripper>("/left_arm/getParas_gripper");          
        
        /************************ 变量初始化 ***********************/
        // 初始化轨迹点索引
        trajectory_point_index_ = 0;
        trajectory_executing_ = false;
        // 初始化 current_joint_positions_
        current_joint_positions_.fill(0.0);

        RCLCPP_INFO(this->get_logger(), "%s rm_sclerp_planner_node_leftArm 节点已经启动", SUCCESS_ICON.c_str());
    
    }  


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

    const std::string LAUNCH_ICON = "🚀";
    const std::string WAIT_ICON = "⏳";  
    const std::string TIMER_ICON = "⏱️"; 
    const std::string WORKING_ICON = "🔄";
    const std::string INITIALIZE_ICON = "⚙️";
    const std::string STOP_ICON = "🛑";
    const std::string COMPLETE_ICON = "🏁";  
    const std::string SIGNAL_ICON = "🚥"; 

    const std::string MSG_ICON = "💬";
    const std::string DATA_ICON = "🔢";
    const std::string TOPIC_ICON = "📡"; 
    const std::string ACTION_ICON = "🤖";
    const std::string SERVICE_ICON = "🛠️";
    const std::string REQUEST_ICON = "📨";
    const std::string FEEDBACK_ICON = "🤝";
    const std::string IMPORTANT_ICON = "💡"; 


    // 发布者：用于发布关节轨迹点到rm_driver
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr joint_pos_publisher_;

    // 发布者：用于发布MoveJ消息到rm_driver
    rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr movej_publisher_;

    // 订阅者：用于订阅当前关节状态
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;


    // 『动作服务端』
    rclcpp_action::Server<GraspBox>::SharedPtr grasp_box_server_;
    rclcpp_action::Server<PlaceBox>::SharedPtr place_box_server_;
    rclcpp_action::Server<TriggerArm>::SharedPtr trigger_left_arm_server_;

    // 动作客户端 - 用于触发左臂任务
    rclcpp_action::Client<TriggerArm>::SharedPtr trigger_right_arm_client_;

    // 右臂任务动作执行状态
    struct RightArmTaskState {
        bool completed{false};           // 任务是否完成
        bool success{false};             // 最终结果
        std::string message{""};         // 最终消息
        std::string current_status{""};  // 当前执行状态
        std::mutex mutex;                // 保护结构的互斥锁
    };
    RightArmTaskState right_arm_task_state_;
    std::condition_variable right_arm_state_cv_;

    // ScLERP规划器接口实例
    std::unique_ptr<sclerp_interface::ScLERPInterface> sclerp_planner_iface_;
    
    // 规划的轨迹
    trajectory_msgs::msg::JointTrajectory planned_trajectory_;
    
    // 规划器初始化状态标志
    bool planner_initialized_ = false;

    // 轨迹执行相关变量
    bool trajectory_executing_ = false;
    bool trajectory_planned_ = false;  // 添加轨迹规划状态标志
    size_t trajectory_point_index_ = 0;
    
    
    // 当前关节位置（从/joint_states话题获取）
    std::array<double, 7> current_joint_positions_;
    std::mutex joint_state_mutex_;  // 保护current_joint_positions_的互斥量
    
    // 轨迹点
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points_;
    size_t current_point_index_ = 0;

    // 吸盘操作指令
    std::string jodell_initialize_cmd = "ros2 service call /left_arm/initialize_gripper rm_ros_interfaces/srv/InitializeGripper \""
        "{ip: '192.168.1.18', port: 8080, register_address: 1000, comm_port: 1, device_address: 9}\"";
    std::string jodell_activate_cmd = "ros2 service call /left_arm/activate_gripper rm_ros_interfaces/srv/ActivateGripper \""
    "{}\""; 
    std::string jodell_deactivate_cmd = "ros2 service call /left_arm/deactivate_gripper rm_ros_interfaces/srv/DeactivateGripper \""
    "{}\"";
    std::string jodell_CloseModbus_cmd = "ros2 service call /left_arm/close_modbus rm_ros_interfaces/srv/CloseModbus \""
                        "{}\"";

    
    // 障碍物
    std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> obstacles_;
    std::shared_ptr<CollisionUtils::ObstacleBase> grasped_object_;                    

    // 法兰坐标系到吸盘工具坐标系的静态变换矩阵
    // 吸盘底面中心点距离法兰中心点沿Z轴正向0.1318mm
    Eigen::Matrix4d matrix_flange2sucker = []() 
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform(2, 3) = 0.1318; // Z轴平移0.1318米
        return transform;
    }();

    std::unique_ptr<RmInterface::RmRobotAPI> rmRobotAPI_;

    void initialize_rmRobot();  

    void initialize_hardware_and_planner();


    void tempForShow();

// #region realman robot interface 二次封装
    // 关节透传控制成员函数
    void rm_movej_canfd(const std::vector<double>& joint_positions);
    // 笛卡尔空间直线运动
    // 笛卡尔空间直线运动
    void rm_movel_cmd(const std::vector<float>& tcp_pose, int speed);
    // 获取关节状态信息
    void rm_get_arm_state(rm_current_arm_state_t& state);
// #endregion


 

// #region GraspBox 动作服务端相关函数   
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
    void execute_grasp_box(const std::shared_ptr<GraspBoxHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing grasp box action");
        rclcpp::Rate loop_rate(1);    
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<GraspBox::Feedback>();
        auto result = std::make_shared<GraspBox::Result>();

        // 重置右臂接收盒子任务状态
        {
            std::lock_guard<std::mutex> lock(right_arm_task_state_.mutex);
            right_arm_task_state_.completed = false;
            right_arm_task_state_.success = false;
            right_arm_task_state_.message = "";
            right_arm_task_state_.current_status = "Initializing";
        }    

        // 1. 初始化
        try {
            // 左臂复位
            std::vector<double> initial_joint_position = {-99.334, -49.228, -27.725, -69.327, -83.567, 121.227, 63.262};      
            rmRobotAPI_->rm_movej(initial_joint_position, 50);
    
    
            // 使用类成员变量跟踪是否已初始化规划器
            if (!planner_initialized_) {
                init_sclerp_planner();
                // 检查规划器是否已初始化
                if (!sclerp_planner_iface_) {
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
        
        } catch (const std::exception& e) {
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
            
            // TODO 该部分代码待分析是否多余
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

            // 计算正向运动学
            //#region 目标姿态为感知获取的盒子姿态
            /*
            Eigen::Vector3d tcpPos;
            Eigen::Quaterniond quaternion;
                
            tcpPos = Eigen::Vector3d(
                goal->pose.position.x + 0.085 + 0.1318,                
                goal->pose.position.y,
                goal->pose.position.z - 0.020);            
            quaternion = Eigen::Quaterniond(
                goal->pose.orientation.x,
                goal->pose.orientation.y,
                goal->pose.orientation.z,
                goal->pose.orientation.w);
            
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
           // /*
           rm_current_arm_state_t rm_robot_current_state_1;
           rm_get_arm_state(rm_robot_current_state_1);
           Eigen::Matrix<double, 6, 1> gripper_tcp_pose_1;
           // TODO: 位置数据待修正，此处未修改URDF模型加入吸盘link，后续加入后应去除偏移量
           gripper_tcp_pose_1 << goal->pose.position.x + 0.075 + 0.1318,
                                 goal->pose.position.y,
                                 goal->pose.position.z - 0.040,
                                 rm_robot_current_state_1.pose.euler.rx,
                                 rm_robot_current_state_1.pose.euler.ry,
                                 rm_robot_current_state_1.pose.euler.rz;
            Eigen::Matrix4d matrix_base2tcp_target;  
            matrix_base2tcp_target = tcpPose2HomogenMatrix(gripper_tcp_pose_1);   
            // */
            // #endregion      

            // 调用重构后的sclerp_planner函数进行轨迹规划
            trajectory_msgs::msg::JointTrajectory joint_trajectory;
            bool is_planned_trajectory = plan_joint_trajectory(jointAngles_start, matrix_base2tcp_target, joint_trajectory);
            if (!is_planned_trajectory) 
            {
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
            if(trajectory_planned_ ){
                int execution_rate;
                this->get_parameter("trajectory_execution_rate", execution_rate);
                publish_joint_trajectory(joint_trajectory, execution_rate);    
                
            } 
    
            trajectory_planned_ = false;
    
        }
    
    /*    
        // 3. 轨迹规划
        // bool planning_success = plan_trajectory_for_task("grasp_box",
        //                                                 goal->start_joint_positions,
        //                                                 goal->tcp_pose,
        //                                                 planned_trajectory_);
        // if (!planning_success) {
        //     result->success = false;
        //     result->message = "Trajectory planning failed";
        //     goal_handle->abort(result);
        //     RCLCPP_ERROR(this->get_logger(), "Trajectory planning failed");
        //     return;
        // }
        
        // 发布反馈
        auto feedback = std::make_shared<GraspBox::Feedback>();
        feedback->status = 2; // 规划完成
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Trajectory planning completed");
        
        // 4. 检查是否需要取消
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Action canceled during planning";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Action canceled during planning");
            return;
        }
        
        // 5. 轨迹执行
        bool execution_success = execute_trajectory_with_feedback(goal_handle);
        if (!execution_success) {
            result->success = false;
            result->message = "Trajectory execution canceled";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Trajectory execution canceled");
            return;
        }
    */   
        
        // 启动吸盘（同步阻塞模式）
        // RCLCPP_INFO(this->get_logger(), "%s 启动吸盘服务", LAUNCH_ICON.c_str());
        // call_jodell_service_sync(jodell_activate_cmd);   

        // 延时让左臂吸盘吸紧物体
        std::this_thread::sleep_for(std::chrono::milliseconds(2000)); 
    
        // 上升用于避障
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
            // 左臂运动到交接位置
            std::vector<double> target_joint_position = {-67.902, -54.464, -56.450, -49.018, -138.739, 89.207, 73.986};
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
            // 触发右臂动作
            RCLCPP_INFO(this->get_logger(), "%s 触发右臂动作服务", REQUEST_ICON.c_str());  
            trigger_right_arm_task("开始执行抓取、扫码、放置任务");   

            // 右臂就位后，关闭左臂吸盘    
            // 根据右臂动作执行状态（吸取结果）决定右臂吸盘关闭
            bool right_arm_success = false;
            std::string right_arm_message = "";
            

            // 等待右臂执行过程中间状态更新和最终结果
            {
                std::unique_lock<std::mutex> lock(right_arm_task_state_.mutex);
                auto wait_start = std::chrono::steady_clock::now();
                auto total_timeout = std::chrono::seconds(25); // 总超时时间
                bool suction_success_handled = false;
                bool arm_arrived_handled = false;
                
                RCLCPP_INFO(this->get_logger(), "⏳ 开始等待右臂任务状态更新");
                
                while (std::chrono::steady_clock::now() - wait_start < total_timeout) 
                {
                    // 检查是否收到吸盘吸取成功的状态且尚未处理
                    if (right_arm_task_state_.current_status == "右臂成功接触到纸箱" &&
                    !arm_arrived_handled)  
                    {   
                        RCLCPP_INFO(this->get_logger(), "✅ 右臂成功接触到纸箱");
                        arm_arrived_handled = true;
                        call_jodell_service_async(jodell_deactivate_cmd);
                    }
                    
                    if (right_arm_task_state_.current_status == "吸盘吸取物体成功" && !suction_success_handled) 
                    {
                        RCLCPP_INFO(this->get_logger(), "✅ 检测到右臂吸盘吸取成功，执行相应操作");
                        suction_success_handled = true;

                        // call_jodell_service_sync(jodell_deactivate_cmd);
                        // RCLCPP_INFO(this->get_logger(), "🔧 右臂吸盘已关闭");
                        // auto suck_end_time = std::chrono::high_resolution_clock::now();
                        // auto suck_duration = std::chrono::duration<double>(suck_end_time - suck_start_time);
                        // RCLCPP_INFO(this->get_logger(), "%s 从触发右臂动作到关闭右臂吸盘耗时: %.6f 秒", TIMER_ICON.c_str(), suck_duration.count());

                        // 机械臂复位到初始位置
                        std::vector<double> initial_joint_position = {-99.334, -49.228, -27.725, -69.327, -83.567, 121.227, 63.262};
                        rmRobotAPI_->rm_movej(initial_joint_position, 50);
                    }
                    
                    // 检查任务是否完成
                    if (right_arm_task_state_.completed) {
                        RCLCPP_INFO(this->get_logger(), "🏁 右臂任务已完成，最终状态: %s", right_arm_task_state_.current_status.c_str());
                        right_arm_success = right_arm_task_state_.success;
                        right_arm_message = right_arm_task_state_.message;
                        break;
                    }
                    
                    // 等待通知或短暂超时，避免忙等待
                    right_arm_state_cv_.wait_for(lock, std::chrono::milliseconds(50));
                }
                
                // 检查是否超时
                if (!right_arm_task_state_.completed) {
                    RCLCPP_WARN(this->get_logger(), "⚠️ 等待右臂任务状态超时");
                    right_arm_success = false;
                    right_arm_message = "Timeout waiting for right arm task result";
                } else if (!suction_success_handled) 
                {
                    RCLCPP_WARN(this->get_logger(), "⚠️ 右臂任务完成但未检测到吸盘成功状态，当前状态: %s", 
                            right_arm_task_state_.current_status.c_str());
                    // 即使未明确收到吸盘成功状态，也执行清理操作
                    call_jodell_service_sync(jodell_deactivate_cmd);
                    RCLCPP_INFO(this->get_logger(), "🔧 右臂吸盘已关闭（默认操作）");
                    // 机械臂复位到初始位置
                    std::vector<double> initial_joint_position = {-99.334, -49.228, -27.725, -69.327, -83.567, 121.227, 63.262};
                    rmRobotAPI_->rm_movej(initial_joint_position, 50);
                }
            }

            // 根据右臂任务结果决定后续操作
            if (!right_arm_success) {
                // 右臂任务失败，关闭右臂吸盘并报告失败
                call_jodell_service_sync(jodell_deactivate_cmd);
                
                result->success = false;
                result->message = "right arm task failed: " + right_arm_message;
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "❌ 因右臂任务失败，终止抓取任务: %s", right_arm_message.c_str());
                return;
            } 
        }   

        // 输出最终结果
        if(rclcpp::ok()){
            result->success = true;
            result->message = "Grasp box action completed successfully";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Grasp box action succeeded");
        }

    }
    
// #endregion  

// #region PlaceBox 动作服务端相关函数   
    // Place Box 目标处理回调函数
    rclcpp_action::GoalResponse handle_place_box_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PlaceBox::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received place box goal: %s", goal->target.c_str());
        (void)uuid;
        // 可以在这里添加目标验证逻辑
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Place Box  取消处理回调函数
    rclcpp_action::CancelResponse handle_place_box_cancel( const std::shared_ptr<PlaceBoxHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel place box goal");
        (void)goal_handle;
        // 可以在这里添加取消处理逻辑
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Place Box 接受目标回调函数
    void handle_place_box_accepted( const std::shared_ptr<PlaceBoxHandle> goal_handle)
    {
        // 使用线程执行实际任务，避免阻塞
        std::thread{std::bind(&RmSclerpPlannerNode1::execute_place_box, this, _1), goal_handle}.detach();
    }

    // Place box 实际执行函数
    void execute_place_box( const std::shared_ptr<PlaceBoxHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing place box action");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<PlaceBox::Feedback>();
        auto result = std::make_shared<PlaceBox::Result>();
        
        // 这里实现具体的检测材料逻辑
        // 可以调用现有的轨迹规划和执行功能
        // 发布反馈信息
        feedback->status = 1;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "place box feedback published");
        
        // 模拟执行过程
        rclcpp::Rate loop_rate(1);
        loop_rate.sleep();
        
        // 设置结果
        result->success = true;
        result->message = "Place box successfully";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Place box action succeeded");
    }
// #endregion   

// #region TriggerLeftArmServer 动作服务端相关函数
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
        const std::shared_ptr<TriggerArmHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "收到取消触发信号请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 处理触发信号的接受回调
    void handle_trigger_left_arm_accepted(
        const std::shared_ptr<TriggerArmHandle> goal_handle)
    {
        // 使用线程执行实际任务，避免阻塞
        std::thread{std::bind(&RmSclerpPlannerNode1::execute_trigger_left_arm, this, _1), goal_handle}.detach();


    }

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

    // 左臂接收纸箱及后续任务动作执行函数 
    bool execute_left_arm_task() {
        // 实现左臂的吸取物体、扫码、放置等任务
        // 这里包含原有的左臂任务逻辑
        tempForShow();
        RCLCPP_INFO(this->get_logger(), "执行左臂抓取、扫码、放置任务");
        return true; // 简化示例
    }
// #endregion       

// #region TriggerRightArmClient 动作客户端相关函数

    // 触发左臂任务的函数
    void trigger_right_arm_task(const std::string& task_description) 
    {
        if (!trigger_right_arm_client_->wait_for_action_server(std::chrono::seconds(5))) 
        {
            RCLCPP_ERROR(this->get_logger(), "❌ 右臂触发动作服务不可用");
            // 更新状态
            {
                std::lock_guard<std::mutex> lock(right_arm_task_state_.mutex);
                right_arm_task_state_.completed = true;
                right_arm_task_state_.success = false;
                right_arm_task_state_.message = "right arm action server not available";
                right_arm_task_state_.current_status = "Failed to connect";
            }
            right_arm_state_cv_.notify_one();         
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
                RCLCPP_ERROR(this->get_logger(), "❌ 请求右臂任务动作被拒绝");
            } else {
                RCLCPP_INFO(this->get_logger(), "✅ 请求右臂任务动作已接受");
            }
        };

        // 反馈回调
        send_goal_options.feedback_callback = [this](
            typename rclcpp_action::ClientGoalHandle<TriggerArm>::SharedPtr goal_handle,
            const std::shared_ptr<const TriggerArm::Feedback> feedback)
        {
            
            // 更新过程状态
            std::lock_guard<std::mutex> lock(right_arm_task_state_.mutex);
            right_arm_task_state_.current_status = feedback->status;
            RCLCPP_INFO(this->get_logger(), "🔄 收到右臂反馈: %s", feedback->status.c_str());
        };

        // 结果回调
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<TriggerArm>::WrappedResult & result) 
        {
            std::lock_guard<std::mutex> lock(right_arm_task_state_.mutex);
            right_arm_task_state_.completed = true;
            
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    right_arm_task_state_.success = true;
                    right_arm_task_state_.message = "right arm succeeded: " + result.result->message;
                    RCLCPP_INFO(this->get_logger(), "✅ 右臂任务成功: %s", result.result->message.c_str());
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    right_arm_task_state_.success = false;
                    right_arm_task_state_.message = "right arm aborted: " + result.result->message;
                    RCLCPP_ERROR(this->get_logger(), "❌ 右臂任务中止: %s", result.result->message.c_str());
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    right_arm_task_state_.success = false;
                    right_arm_task_state_.message = "right arm canceled: " + result.result->message;
                    RCLCPP_WARN(this->get_logger(), "⚠️ 右臂任务取消: %s", result.result->message.c_str());
                    break;
                default:
                    right_arm_task_state_.success = false;
                    right_arm_task_state_.message = "right arm unknown result";
                    RCLCPP_ERROR(this->get_logger(), "❌ 右臂任务未知结果");
                    break;
            }
            right_arm_state_cv_.notify_one();
        };    

        // 发送目标
        trigger_right_arm_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "已发送触发右臂任务请求: %s", task_description.c_str());
    }
// #endregion

    bool plan_trajectory_for_task(const std::string& task_type,
                                  const Eigen::VectorXd &jointAngles_start,
                                  const Eigen::Matrix<double, 6, 1>& tcpPose,
                                  trajectory_msgs::msg::JointTrajectory &joint_trajectory);

    bool execute_trajectory_with_feedback(const std::shared_ptr<GraspBoxHandle> goal_handle);  
                                    

    // 同步阻塞方式调用服务（会阻塞当前线程直到完成）
    void call_jodell_service_sync(const std::string& cmd) {
        RCLCPP_INFO(this->get_logger(), "执行命令: %s", cmd.c_str());
        int result = system(cmd.c_str());
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "服务调用成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用失败");
        }
    }

    // 异步非阻塞方式调用服务（不会阻塞当前线程）
    void call_jodell_service_async(const std::string& cmd) {
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

    // 更新障碍物信息
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

    //
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



    void init_sclerp_planner() 
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
            sclerp_planner_iface_ = nullptr;
            return;
        }
        
        // 检查URDF文件是否存在
        std::ifstream urdf_file(urdf_path);
        if (!urdf_file.good()) {
            RCLCPP_ERROR(this->get_logger(), "指定的URDF文件不存在或无法访问: %s", urdf_path.c_str());
            sclerp_planner_iface_ = nullptr;
            return;
        }
        urdf_file.close();
        RCLCPP_INFO(this->get_logger(), "URDF文件检查通过: %s", urdf_path.c_str());
        // #endregion
        
        // #region 初始化STL模型路径（用于碰撞检测）
        // 定义STL模型路径向量
        std::string rm_description_path = ament_index_cpp::get_package_share_directory("rm_description");
        std::string meshes_base_path = rm_description_path + "/meshes/rm_75_arm";
        
        std::vector<std::string> stl_paths;
        stl_paths.push_back(meshes_base_path + "/base_link.STL");
        stl_paths.push_back(meshes_base_path + "/link1.STL");
        stl_paths.push_back(meshes_base_path + "/link2.STL");
        stl_paths.push_back(meshes_base_path + "/link3.STL");
        stl_paths.push_back(meshes_base_path + "/link4.STL");
        stl_paths.push_back(meshes_base_path + "/link5.STL");
        stl_paths.push_back(meshes_base_path + "/link6.STL");
        stl_paths.push_back(meshes_base_path + "/link7_6fb.STL");

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

            sclerp_planner_iface_ = std::make_unique<sclerp_interface::ScLERPInterface>(
                base_link, tip_link, shared_from_this(), urdf_path);
            
            if (!sclerp_planner_iface_) {
                RCLCPP_ERROR(this->get_logger(), "无法创建规划器接口实例");
            } else {
                RCLCPP_INFO(this->get_logger(), "成功创建规划器接口实例");
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "创建规划器接口时出错: %s", e.what());
            sclerp_planner_iface_ = nullptr;
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

        //         sclerp_planner_iface_ = std::make_unique<sclerp_interface::ScLERPInterface>(
        //             base_link, tip_link, shared_from_this(), urdf_path);
        //     } else {
        //         RCLCPP_INFO(this->get_logger(), "使用默认的MoveIt参数服务器");
                
        //         sclerp_planner_iface_ = std::make_unique<sclerp_interface::ScLERPInterface>(
        //             base_link, tip_link, shared_from_this(), "");
        //     }
            
        //     if (!sclerp_planner_iface_) {
        //         RCLCPP_ERROR(this->get_logger(), "无法创建规划器接口实例");
        //     } else {
        //         RCLCPP_INFO(this->get_logger(), "成功创建规划器接口实例");
        //     }

        // } catch (const std::exception& e) {
        //     RCLCPP_ERROR(this->get_logger(), "创建规划器接口时出错: %s", e.what());
        //     sclerp_planner_iface_ = nullptr;
        // }    

    // #endregion

    
    }


    bool plan_joint_trajectory(const Eigen::VectorXd &jointAngles_start,
                               const Eigen::Matrix4d &matrix_base2tcp_target,
                               trajectory_msgs::msg::JointTrajectory &joint_trajectory )
    {

        // 检查sclerp_planner_iface_是否已正确初始化
        if (!sclerp_planner_iface_) {
            RCLCPP_ERROR(this->get_logger(), "规划器接口未初始化");
            return false;
        }

        // 使用solve方法生成轨迹数据
        bool result = false;
    
        try {
            result = sclerp_planner_iface_->solve(jointAngles_start, matrix_base2tcp_target, joint_trajectory);
        
            
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

    }
    
    bool plan_joint_trajectory(const Eigen::VectorXd &jointAngles_start,
                               const Eigen::Matrix4d &matrix_base2tcp_target,
                               const std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> &obstacles,
                               std::shared_ptr<CollisionUtils::ObstacleBase> grasped_object,
                               trajectory_msgs::msg::JointTrajectory &joint_trajectory )
    {

        // 检查sclerp_planner_iface_是否已正确初始化
        if (!sclerp_planner_iface_) {
            RCLCPP_ERROR(this->get_logger(), "规划器接口未初始化");
            return false;
        }

        // 使用solve方法生成轨迹数据
        bool result = false;

        try {


            // 新的solve函数参数包括碰撞检测相关参数
            bool is_check_self_collision = true;  // 启用自碰撞检测
            int num_links_ignore = 0;  // 不忽略任何链接
            // std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> obstacles;  // 空障碍物列表
            // std::shared_ptr<CollisionUtils::ObstacleBase> grasped_object = nullptr;  // 无被抓取物体
            
            result = sclerp_planner_iface_->solve(jointAngles_start, 
                                                matrix_base2tcp_target, 
                                                is_check_self_collision, 
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
            
            // 直接将规划器生成的轨迹点复制到 planned_joint_trajectory_ 中
            planned_joint_trajectory_.points.clear();
            planned_joint_trajectory_.header = joint_trajectory.header;
            planned_joint_trajectory_.joint_names = joint_trajectory.joint_names;
            
            // 复制轨迹点数据
            for (const auto& point : joint_trajectory.points) {
                trajectory_msgs::msg::JointTrajectoryPoint new_point;
                new_point.positions = point.positions;
                new_point.velocities = point.velocities;
                new_point.accelerations = point.accelerations;
                new_point.effort = point.effort;
                new_point.time_from_start = point.time_from_start;
                planned_joint_trajectory_.points.push_back(new_point);
            }
                        
            trajectory_point_index_ = 0;
            is_trajectory_executing_ = true;
            is_trajectory_planned_ = true;  // 标记轨迹已规划
            
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
    void publish_joint_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory, int execution_rate)
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
        std::string directory = "/home/byd/hrt2/ros2_rm_ws/src/ros2_rm_robot-humble/rm_motion_planner/data";
        
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


void RmSclerpPlannerNode1::initialize_rmRobot() {
    try {
        // 初始化机械臂API
        rmRobotAPI_ = std::make_unique<RmInterface::RmRobotAPI>();
        // 使用默认IP地址初始化，如果需要可以修改为实际的机器人IP
        rmRobotAPI_->initialize_RmRobot("192.168.1.18");
        // rmRobotAPI_->initialize_RmRobot();        
        if (!rmRobotAPI_->isInitialized()) {
            RCLCPP_ERROR(this->get_logger(), "机械臂API初始化失败");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "机械臂API初始化成功");
        
        // 机械臂复位到初始位置
        // ⚠️ 注意：在实际使用中，确保机械臂周围没有障碍物再执行此操作
        // ⚠️ 初始关节角度（注意：不是弧度制单位）
        // std::vector<double> initial_joint_position = {-80.939, -92.121, -30.263, -104.716, -85.436, 122.161, -15.161};
        // 不遮挡相机（易规划失败）
        // std::vector<double> initial_joint_position = {-113.882, -52.867, -33.962, -29.249, -55.023, 126.469, 105.515};
        std::vector<double> initial_joint_position = {-99.334, -49.228, -27.725, -69.327, -83.567, 121.227, 63.262};            
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

void  RmSclerpPlannerNode1::tempForShow() 
{

        // 使用系统命令调用服务替代直接客户端调用
        // RCLCPP_INFO(this->get_logger(), "%s 开始初始化吸盘机械臂",LAUNCH_ICON.c_str());
        // call_jodell_service_sync(jodell_initialize_cmd); 
        // RCLCPP_INFO(this->get_logger(), "%s 吸盘初始化完成",COMPLETE_ICON.c_str());

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


        // 启动吸盘
        auto suck_start_time = std::chrono::high_resolution_clock::now();
        call_jodell_service_sync(jodell_activate_cmd);  
        auto suck_end_time = std::chrono::high_resolution_clock::now();
        auto suck_duration = std::chrono::duration<double>(suck_end_time - suck_start_time);
        RCLCPP_INFO(this->get_logger(), "%s 吸盘启动耗时: %.6f 秒", TIMER_ICON.c_str(), suck_duration.count());

        // 延时一段时间让吸盘工作
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

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
        rmRobotAPI_->rm_movej(release_joint_position, 40);        

        // 关闭吸盘
        call_jodell_service_sync(jodell_deactivate_cmd);

        // 关闭 Modbus（可注释🗒）
        // call_jodell_service_sync(jodell_CloseModbus_cmd);        
}

// #region Realman API 相关函数
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

// #region 面向动作机制的函数接口
// ♻️ 重构初始化逻辑为独立函数
void RmSclerpPlannerNode1::initialize_hardware_and_planner() 
{
    // 原 init_timer_callback 中的硬件初始化逻辑
    RCLCPP_INFO(this->get_logger(), "%s 开始初始化机械臂与吸盘",LAUNCH_ICON.c_str());
    initialize_rmRobot();

//#region 吸盘初始化
    // 服务客户端方式控制吸盘
    // 初始化吸盘
    auto init_gripper_request = std::make_shared<rm_ros_interfaces::srv::InitializeGripper::Request>();
    init_gripper_request->ip = "192.168.1.18";
    init_gripper_request->port = 8080;
    init_gripper_request->register_address = 1000;
    init_gripper_request->comm_port = 1;
    init_gripper_request->device_address = 9;
    
    auto init_gripper_future = init_client_->async_send_request(init_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), init_gripper_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = init_gripper_future.get();
        if (!response->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize gripper: %s", response->message.c_str());
            return;
        }
    }
     
    // 系统命令方式控制吸盘
    // 初始化电吸盘
    call_jodell_service_async(jodell_initialize_cmd);


    // call_jodell_service_sync(jodell_initialize_cmd);
    // //延时一段时间让吸盘工作
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // RCLCPP_INFO(this->get_logger(), "%s 吸盘初始化完成",COMPLETE_ICON.c_str());

//#endregion
}

// 重构轨迹规划逻辑为独立函数
bool RmSclerpPlannerNode1::plan_trajectory_for_task(const std::string& task_type,
                                                    const Eigen::VectorXd &jointAngles_start,
                                                    const Eigen::Matrix<double, 6, 1>& tcpPose,
                                                    trajectory_msgs::msg::JointTrajectory &joint_trajectory
) 
{
    Eigen::Matrix4d matrix_base2tcp_target;
    matrix_base2tcp_target = tcpPose2HomogenMatrix(tcpPose);

    // 原 timer_callback 中的轨迹规划逻辑
    // 根据 task_type 确定规划参数
    if (task_type == "grasp_box") 
    {
        // 设置抓取相关的起始和目标关节角度
        plan_joint_trajectory(jointAngles_start,
                        matrix_base2tcp_target,
                        joint_trajectory );

        // 检查规划结果
        if (joint_trajectory.points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan trajectory for grasp_box task");
            return false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully planned trajectory with %ld points for grasp_box task", 
                        joint_trajectory.points.size());
        }        

    } else if (task_type == "place_box") {
        // 设置放置相关的起始和目标关节角度
        // 设置抓取相关的起始和目标关节角度
        plan_joint_trajectory(jointAngles_start,
                            matrix_base2tcp_target,
                            joint_trajectory );
        
        // 检查规划结果
        if (joint_trajectory.points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan trajectory for grasp_box task");
            return false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully planned trajectory with %ld points for grasp_box task", 
                        joint_trajectory.points.size());
        }

    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown task type: %s", task_type.c_str());
        return false;
    }
    // 检查轨迹点是否存在来判断规划是否成功
    bool planning_success = !joint_trajectory.points.empty();
    return planning_success;    

}

// 重构轨迹执行逻辑为独立函数
bool RmSclerpPlannerNode1::execute_trajectory_with_feedback(
    const std::shared_ptr<GraspBoxHandle> goal_handle) 
{
    
    // 原 trajectory_timer_callback 中的轨迹执行逻辑
    trajectory_point_index_ = 0;
    trajectory_executing_ = true;
    
    while (rclcpp::ok() && trajectory_executing_ && 
           trajectory_point_index_ < planned_trajectory_.points.size()) {
        
        // 检查是否需要取消
        if (goal_handle->is_canceling()) {
            // 处理取消请求
            return false;
        }
        
        // 发布轨迹点
        const auto& point = planned_trajectory_.points[trajectory_point_index_];
        auto joint_msg = rm_ros_interfaces::msg::Jointpos();
        joint_msg.dof = 7;
        joint_msg.follow = true;
        joint_msg.expand = 0.0;
        
        if (point.positions.size() >= 7) {
            joint_msg.joint.assign(point.positions.begin(), point.positions.begin() + 7);
            joint_pos_publisher_->publish(joint_msg);
        }
        
        // 发布反馈
        auto feedback = std::make_shared<GraspBox::Feedback>();
        feedback->status = static_cast<int32_t>(trajectory_point_index_);
        goal_handle->publish_feedback(feedback);
        
        trajectory_point_index_++;
        
        // 控制发布频率
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    return true;
}


// #endregion


// 主函数保持不变
int main(int argc, char **argv)
{
    // 初始化ROS客户端库
    rclcpp::init(argc, argv);

    // 创建并初始化 RmSclerpPlannerNode1 节点
    auto planner_node = std::make_shared<RmSclerpPlannerNode1>();
    
    // // 进入ROS自旋模式，等待消息
    rclcpp::spin(planner_node);

    
    // 关闭ROS客户端库
    rclcpp::shutdown();

    // 返回 0 表示程序正常结束
    return 0;    
}

