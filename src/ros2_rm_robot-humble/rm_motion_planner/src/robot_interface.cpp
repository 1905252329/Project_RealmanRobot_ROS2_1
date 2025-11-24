#include "rm_motion_planner/robot_interface.h"
#include <cmath>
#include <vector>
#include <memory>
#include <urdf/model.h>

namespace RmInterface 
{

// #region 类ParseURDF 成员函数
/**
 * @brief 从URDF关节变换中提取DH参数
 * @param transform URDF中的关节变换
 * @param joint_index 关节索引
 * @return DH参数
 */
DHParameters ParseURDF::extractDHParameters(const urdf::Pose& transform, size_t joint_index) {
    DHParameters dh;
    
    // 初始化默认值
    dh.d = 0.0;
    dh.a = 0.0;
    dh.alpha = 0.0;
    dh.theta = 0.0;
    
    // 提取位置和旋转信息
    double x = transform.position.x;
    double y = transform.position.y;
    double z = transform.position.z;
    
    double roll, pitch, yaw;
    transform.rotation.getRPY(roll, pitch, yaw);
    
    // 根据标准DH参数定义计算
    // d (link offset): 沿前一个关节z轴的距离
    dh.d = z;
    
    // a (link length): 沿当前关节x轴的距离
    dh.a = sqrt(x*x + y*y);
    
    // alpha (link twist): 绕当前关节x轴的扭转角
    dh.alpha = pitch;
    
    // theta (joint angle): 绕当前关节z轴的角度
    dh.theta = yaw;
    
    // 特殊处理：如果x和y都接近0，说明是沿z轴的移动
    if (fabs(x) < 1e-6 && fabs(y) < 1e-6) {
        // 纯z轴移动，a=0
        dh.a = 0.0;
    }
    
    // 特殊处理：如果z接近0，说明是在xy平面内
    if (fabs(z) < 1e-6) {
        dh.d = 0.0;
    }
    
    return dh;
}


/**
 * @brief 更准确的DH参数提取方法
 * @param joint URDF关节
 * @param parent_link 父链接
 * @param child_link 子链接
 * @return DH参数
 */
DHParameters ParseURDF::computeDHParameters(const urdf::JointConstSharedPtr& joint,
                                        const urdf::LinkConstSharedPtr& parent_link,
                                        const urdf::LinkConstSharedPtr& child_link) {
    DHParameters dh;
    
    // 初始化默认值
    dh.d = 0.0;
    dh.a = 0.0;
    dh.alpha = 0.0;
    dh.theta = 0.0;
    
    // 获取关节变换
    urdf::Pose transform = joint->parent_to_joint_origin_transform;
    
    // 提取位置信息
    double x = transform.position.x;
    double y = transform.position.y;
    double z = transform.position.z;
    
    // 获取旋转信息
    double roll, pitch, yaw;
    transform.rotation.getRPY(roll, pitch, yaw);
    
    // 根据关节类型计算DH参数
    if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
        // 对于旋转关节，theta是关节变量（这里我们只计算固定偏移）
        // 在标准DH参数中：
        // d 是沿前一个z轴的距离
        dh.d = z;
        
        // a 是沿当前x轴的距离
        dh.a = sqrt(x*x + y*y);
        
        // alpha 是绕当前x轴的扭转角
        dh.alpha = pitch;
        
        // theta 是绕当前z轴的旋转角
        dh.theta = yaw;
        
        // 特殊处理：如果x和y都接近0，说明是沿z轴的移动
        if (fabs(x) < 1e-6 && fabs(y) < 1e-6) {
            // 纯z轴移动，a=0
            dh.a = 0.0;
        }
        
        // 特殊处理：如果z接近0，说明是在xy平面内
        if (fabs(z) < 1e-6) {
            dh.d = 0.0;
        }
    } else if (joint->type == urdf::Joint::PRISMATIC) {
        // 对于移动关节，d是关节变量
        dh.d = z;
        dh.a = sqrt(x*x + y*y);
        dh.alpha = pitch;
        dh.theta = yaw;
    } else if (joint->type == urdf::Joint::FIXED) {
        // 对于固定关节
        dh.d = z;
        dh.a = sqrt(x*x + y*y);
        dh.alpha = pitch;
        dh.theta = yaw;
    }
    
    return dh;
}    


/**
 * @brief 从URDF模型中解析所有关节的DH参数
    * @param urdf_model URDF模型
    * @param logger ROS2日志记录器
    */
void ParseURDF::parseDHParams(const urdf::Model& urdf_model, 
                                        rclcpp::Logger logger) {
    // 定义关节名称
    std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", 
                                            "joint_4", "joint_5", "joint_6", "joint7"};
    
    RCLCPP_INFO(logger, "从URDF中计算的DH参数:");
    RCLCPP_INFO(logger, "  关节  |    d    |    a    |  alpha  | theta_offset");
    RCLCPP_INFO(logger, "--------|---------|---------|---------|-------------");
    
    for (size_t i = 0; i < joint_names.size(); ++i) {
        const std::string& joint_name = joint_names[i];
        if (urdf_model.joints_.find(joint_name) != urdf_model.joints_.end()) {
            const auto& joint = urdf_model.joints_.at(joint_name);
            
            // 获取父链接和子链接
            urdf::LinkConstSharedPtr parent_link = urdf_model.getLink(joint->parent_link_name);
            urdf::LinkConstSharedPtr child_link = urdf_model.getLink(joint->child_link_name);
            
            // 使用高级方法计算DH参数
            auto dh_params = ParseURDF::computeDHParameters(joint, parent_link, child_link);
            
            // 打印DH参数表
            RCLCPP_INFO(logger, "   %zu    | %7.4f | %7.4f | %7.4f | %7.4f", 
                        i+1, dh_params.d, dh_params.a, dh_params.alpha*180/M_PI, dh_params.theta);

        } else {
            RCLCPP_WARN(logger, "未找到关节: %s", joint_name.c_str());
        }
    }
}


/**
 * @brief 从URDF文件路径加载模型并解析DH参数，作为外部调用的入口
    * @param urdf_path URDF文件路径
    * @param logger ROS2日志记录器
    * @return 是否成功解析
    * @TODO 能否去除 logger 形参？
    */
bool ParseURDF::loadDHParams(const std::string& urdf_path, 
                                    rclcpp::Logger logger) {
    RCLCPP_INFO(logger, "========== 从URDF文件中解析DH参数(高级方法) ==========");
    
    if (urdf_path.empty()) {
        RCLCPP_WARN(logger, "未提供URDF路径参数，无法解析DH参数");
        return false;
    }
    
    urdf::Model urdf_model;
    if (!urdf_model.initFile(urdf_path)) {
        RCLCPP_ERROR(logger, "无法加载URDF文件: %s", urdf_path.c_str());
        return false;
    }
    
    RCLCPP_INFO(logger, "成功加载URDF文件: %s", urdf_path.c_str());
    
    // 调用解析函数
    parseDHParams(urdf_model, logger);
    return true;
}



bool ParseURDF::getLinkNamesFromURDF(const std::string& urdf_path, std::string& base_link, std::string& tip_link) {
    urdf::Model urdf_model;
    
    if (!urdf_model.initFile(urdf_path)) {

        RCLCPP_ERROR(rclcpp::get_logger("getLinkNamesFromURDF"), "无法加载 URDF 文件: %s", urdf_path.c_str());
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

// #endregion



// #region 类RmRobotAPI 成员函数
RmRobotAPI::RmRobotAPI() : robot_handle_(nullptr), rm_api_initialized_(false)
{
}

void RmRobotAPI::initialize_RmRobot(const std::string& robot_ip)
// void RmRobotAPI::initialize_RmRobot()
{
    // 初始化RM机器人API
    RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "开始初始化RM API...");
    
    // 初始化线程模式
    int init_result = rm_init(RM_TRIPLE_MODE_E);
    if (init_result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("initialize_RmRobot"), "初始化线程模式失败，错误码: %d", init_result);
        rm_api_initialized_ = false;
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "线程模式初始化成功");    


    robot_handle_ = rm_create_robot_arm(robot_ip.c_str(), 8080);
    // robot_handle_ = rm_create_robot_arm("169.254.247.19", 8080);

    // 检查机器人句柄
    if (robot_handle_ == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("initialize_RmRobot"), "无法创建RM机器人句柄，返回空指针");
        RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "可能的原因:");
        RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "  1. 机器人IP地址或端口不正确");
        RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "  2. 机器人未开机或处于错误状态");
        RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "  3. 网络连接问题");
        RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "  4. 机器人固件版本与SDK不兼容");
        rm_api_initialized_ = false;
        return;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "机器人句柄地址: %p", (void*)robot_handle_); 
    RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "机器人句柄ID: %d", robot_handle_->id); 

    if (robot_handle_->id <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("initialize_RmRobot"), "无效的机器人句柄ID: %d", robot_handle_->id);
        RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "根据错误信息分析，可能是工具坐标系配置问题导致:");
        RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "  [get_current_tool_frame] Tool frame position parse err");
        RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "解决方案:");
        RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "  1. 在示教器上将工具坐标系设置为出厂默认的Arm_tip");
        RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "  2. 检查自定义工具坐标系的配置是否正确");
        RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "  3. 如果必须使用自定义工具坐标系，请在连接成功后再设置");
        rm_api_initialized_ = false;
        return;
    }
        
    RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "RM机器人句柄创建成功");

    // 初始化算法依赖数据(不连接机械臂时调用)
    // 注意：即使连接了真实的机械臂，在使用RM API算法功能时仍需要调用此函数
    // 来初始化算法库所需的系统数据(如DH参数等)
    RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "使用RM_MODEL_RM_75_E型号初始化算法数据...");
    rm_algo_init_sys_data(RM_MODEL_RM_75_E, RM_MODEL_RM_ISF_E);
    
    rm_api_initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "RM API 初始化成功");
    
    // // 尝试设置默认工具坐标系，避免后续操作中出现工具坐标系相关问题
    // RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "设置默认工具坐标系...");
    // rm_frame_t default_tool_frame;
    // memset(&default_tool_frame, 0, sizeof(rm_frame_t));
    // strncpy(default_tool_frame.frame_name, "Arm_tip", sizeof(default_tool_frame.frame_name) - 1);
    // default_tool_frame.pose.position.x = 0.0f;
    // default_tool_frame.pose.position.y = 0.0f;
    // default_tool_frame.pose.position.z = 0.0f;
    // default_tool_frame.pose.quaternion.w = 1.0f;
    // default_tool_frame.pose.quaternion.x = 0.0f;
    // default_tool_frame.pose.quaternion.y = 0.0f;
    // default_tool_frame.pose.quaternion.z = 0.0f;
    // default_tool_frame.pose.euler.rx = 0.0f;
    // default_tool_frame.pose.euler.ry = 0.0f;
    // default_tool_frame.pose.euler.rz = 0.0f;
    // default_tool_frame.payload = 0.0f;
    
    // // 检查robot_handle_是否有效再调用API
    // if (robot_handle_ != nullptr) {
    //     int tool_result = rm_set_manual_tool_frame(robot_handle_, default_tool_frame);
    //     if (tool_result != 0) {
    //         RCLCPP_WARN(rclcpp::get_logger("initialize_RmRobot"), "设置默认工具坐标系失败，错误码: %d", tool_result);
    //     } else {
    //         RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "成功设置默认工具坐标系");
    //     }
    // } else {
    //     RCLCPP_WARN(rclcpp::get_logger("initialize_RmRobot"), "机器人句柄为空，跳过设置默认工具坐标系");
    // }
    
    // // 切换到Jodell工具坐标系
    // RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "切换到Jodell工具坐标系...");
    // // 检查robot_handle_是否有效再调用API
    // if (robot_handle_ != nullptr) {
    //     int change_tool_result = rm_change_tool_frame("Jodell");
    //     if (change_tool_result != 0) {
    //         RCLCPP_WARN(rclcpp::get_logger("initialize_RmRobot"), "切换到Jodell工具坐标系失败，错误码: %d", change_tool_result);
    //         RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "将继续使用默认工具坐标系");
    //     } else {
    //         RCLCPP_INFO(rclcpp::get_logger("initialize_RmRobot"), "成功切换到Jodell工具坐标系");
    //     }
    // } else {
    //     RCLCPP_WARN(rclcpp::get_logger("initialize_RmRobot"), "机器人句柄为空，跳过切换到Jodell工具坐标系");
    // }
}
        
rm_pose_t RmRobotAPI::compute_FK(const Eigen::VectorXd& joint_angles)
{
    if (!rm_api_initialized_) {
        RCLCPP_WARN(rclcpp::get_logger("compute_FK"), "RM API未初始化");
        rm_pose_t empty_pose = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
        return empty_pose;
    }
    
    // 检查输入参数
    if (joint_angles.size() < 7) {
        RCLCPP_WARN(rclcpp::get_logger("compute_FK"), "输入关节角度向量大小不足7个元素: %ld", joint_angles.size());
        rm_pose_t empty_pose = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
        return empty_pose;
    }
    
    // 打印输入的关节角度
    // RCLCPP_INFO(rclcpp::get_logger("compute_FK"), "输入关节角度 (弧度):");
    // for (int i = 0; i < 7; i++) {
    //     RCLCPP_INFO(rclcpp::get_logger("compute_FK"), "  关节%d: %f", i+1, joint_angles(i));
    // }
    
    // 将Eigen向量转换为float数组，单位从弧度转换为度
    float joint_angles_deg[7];
    for (int i = 0; i < 7; i++) {
        joint_angles_deg[i] = static_cast<float>(joint_angles(i) * 180.0 / M_PI);
    }
    
    // 打印转换后的关节角度（度）
    // RCLCPP_INFO(rclcpp::get_logger("compute_FK"), "输入关节角度 (度):");
    // for (int i = 0; i < 7; i++) {
    //     RCLCPP_INFO(rclcpp::get_logger("compute_FK"), "  关节%d: %f", i+1, joint_angles_deg[i]);
    // }
    
    // 调用RM API正向运动学函数
    rm_pose_t pose = rm_algo_forward_kinematics(robot_handle_, joint_angles_deg);
    
    // 检查返回的姿态数据
    RCLCPP_INFO(rclcpp::get_logger("compute_FK"), "RM API返回的位姿数据:");
    RCLCPP_INFO(rclcpp::get_logger("compute_FK"), " 位置: x=%f, y=%f, z=%f", 
               pose.position.x, pose.position.y, pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("compute_FK"), " 欧拉角: rx=%f, ry=%f, rz=%f", 
               pose.euler.rx, pose.euler.ry, pose.euler.rz);
    RCLCPP_INFO(rclcpp::get_logger("compute_FK"), "  四元数: x=%f, y=%f, z=%f, w=%f", 
               pose.quaternion.x, pose.quaternion.y, pose.quaternion.z, pose.quaternion.w);
    
    // 检查位置是否为零
    if (pose.position.x == 0.0 && pose.position.y == 0.0 && pose.position.z == 0.0) {
        RCLCPP_WARN(rclcpp::get_logger("compute_FK"), "警告：RM API返回的位置数据为零，可能机械臂型号配置不正确");
    }
    
    return pose;
}



/**
* @brief 通过CANFD总线发送MoveJ指令，控制机器人以指定的关节角度进行运动
*
* 该函数通过CANFD总线发送MoveJ指令给机器人，使其按照给定的关节角度进行运动。
*
* @param joint_positions 包含机器人各关节目标角度的向量，长度为6或7（取决于机器人的自由度）。
* @return 成功返回0，失败返回负数。
*         -1: RM API未初始化
*         -2: 关节角度向量大小错误
*         其他负数: rm_movej_canfd函数返回的错误码
*/
int RmRobotAPI::movej_canfd(const std::vector<double>& joint_positions) {
    if (!rm_api_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "RM API未初始化");
        return -1;
    }
    
    if (robot_handle_ == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "机器人句柄为空指针");
        return -1;
    }
    
    if (joint_positions.size() < 6 || joint_positions.size() > 7) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "关节角度向量大小错误: %ld", joint_positions.size());
        return -2;
    }
    
    // 创建MoveJ角度透传结构体
    rm_movej_canfd_mode_t movej_cmd = {0};
    
    // 设置关节角度
    for (size_t i = 0; i < joint_positions.size() && i < 7; i++) {
        movej_cmd.joint[i] = static_cast<float>(joint_positions[i]);
    }
    
    // 设置其他参数
    movej_cmd.expand = 0.0f; //目标关节角度，单位：°
    movej_cmd.follow = true;
    movej_cmd.trajectory_mode = 2;
    movej_cmd.radio = 50;
    
    // 调用SDK函数
    int result = rm_movej_canfd(robot_handle_, movej_cmd);
    
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "rm_movej_canfd调用失败，错误码: %d", result);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("RmRobotAPI"), "rm_movej_canfd调用成功");
    }
    
    return result;
}


/**
* @brief 使用指定关节角度控制机器人移动到指定位置
*
* 此函数使用指定的关节角度将机器人移动到目标位置。
*
* @param joint_positions 关节角度向量(单位度），包含六个或七个关节角度，顺序为：[J1, J2, J3, J4, J5, J6, (J7)]
* @param speed 移动速度，取值范围为1到100
*
* @return 返回执行结果，0表示成功，-1表示RM API未初始化，-2表示关节角度向量大小错误
*/
int RmRobotAPI::rm_movej(const std::vector<double>& joint_positions, int speed)   
{
    int radius = 0;  // 交融半径
    int trajectory_connect = 0; // 立即规划并执行轨迹
    int block = 1; // 阻塞模式（默认线程模式为多线程）   

    if (!rm_api_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "RM API未初始化");
        return -1;
    }
    
    if (robot_handle_ == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "机器人句柄为空指针");
        return -1;
    }
    //检查向量大小是否正确
    if (joint_positions.size() < 6 || joint_positions.size() > 7) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "关节角度向量大小错误: %ld", joint_positions.size());
        return -2;
    }

    // 检查速度参数范围
    if (speed < 1 || speed > 100) {
        RCLCPP_WARN(rclcpp::get_logger("RmRobotAPI"), "速度参数超出范围[1-100]: %d，将使用默认值20", speed);
        speed = 20;
    }
    
    // 检查交融半径参数范围
    if (radius < 0 || radius > 100) {
        RCLCPP_WARN(rclcpp::get_logger("RmRobotAPI"), "交融半径参数超出范围[0-100]: %d，将使用默认值0", radius);
        radius = 0;
    }
 
    
    // 创建关节角数组
    float joint[6] = {0.0, 0.0, 0.0, 0.0, 90.0, 0.0};
    
    // 设置关节角度
    for (size_t i = 0; i < joint_positions.size() && i < 7; i++) {
        joint[i] = static_cast<float>(joint_positions[i]);
    }
    

    int result = ::rm_movej(robot_handle_, joint, speed, radius, trajectory_connect, block);
    
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "rm_movej 调用失败，错误码: %d", result);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("RmRobotAPI"), "rm_movej 调用成功");
    }
    
    return result;
}


int RmRobotAPI::rm_movel(const std::vector<float>& tcp_pose, int speed) {
    int radius = 0;  // 交融半径
    int trajectory_connect = 0; // 立即规划并执行轨迹
    int block = 1; // 阻塞模式（默认线程模式为多线程）

    if (!rm_api_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "RM API未初始化");
        return -1;
    }
    
    if (robot_handle_ == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "机器人句柄为空指针");
        return -1;
    }
    
    // 检查tcp_pose参数，应该包含6个元素：x, y, z, rx, ry, rz
    if (tcp_pose.size() != 6) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "TCP位姿向量大小错误: %ld，应该为6个元素(x,y,z,rx,ry,rz)", tcp_pose.size());
        return -2;
    }
    
    // 检查速度参数范围
    if (speed < 1 || speed > 100) {
        RCLCPP_WARN(rclcpp::get_logger("RmRobotAPI"), "速度参数超出范围[1-100]: %d，将使用默认值20", speed);
        speed = 20;
    }
    
    // 检查交融半径参数范围
    if (radius < 0 || radius > 100) {
        RCLCPP_WARN(rclcpp::get_logger("RmRobotAPI"), "交融半径参数超出范围[0-100]: %d，将使用默认值0", radius);
        radius = 0;
    }
    
    // 创建位姿结构体
    rm_pose_t pose;
    pose.position.x = tcp_pose[0];  // X坐标 (米)
    pose.position.y = tcp_pose[1];  // Y坐标 (米)
    pose.position.z = tcp_pose[2];  // Z坐标 (米)
    pose.euler.rx = tcp_pose[3];    // RX欧拉角 (弧度)
    pose.euler.ry = tcp_pose[4];    // RY欧拉角 (弧度)
    pose.euler.rz = tcp_pose[5];    // RZ欧拉角 (弧度)
    
    // 设置四元数为默认值（API可能会重新计算）
    pose.quaternion.w = 1.0f;
    pose.quaternion.x = 0.0f;
    pose.quaternion.y = 0.0f;
    pose.quaternion.z = 0.0f;


    
    // 调用RM API的rm_movel函数
    int result = ::rm_movel(robot_handle_, pose, speed, radius, trajectory_connect, block);
    
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "rm_movel调用失败，错误码: %d", result);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("RmRobotAPI"), "rm_movel调用成功");
    }
    
    return result;
}


int RmRobotAPI::rm_get_arm_state(rm_current_arm_state_t& state) {
    if (!rm_api_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "RM API未初始化");
        return -1;
    }
    
    if (robot_handle_ == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "机器人句柄为空指针");
        return -1;
    }
    
    // 调用RM API的rm_get_current_arm_state函数
    int result = ::rm_get_current_arm_state(robot_handle_, &state);
    
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "rm_get_current_arm_state调用失败，错误码: %d", result);
        
        // 添加详细的错误码解释
        switch(result) {
            case -1:
                RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "错误详情: 未找到对应句柄，句柄为空或已被删除");
                break;
            case -2:
                RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "错误详情: 获取到的机械臂基本信息非法");
                break;
            default:
                RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "错误详情: 未知错误");
                break;
        }
        
        return result;
    } else {
        RCLCPP_DEBUG(rclcpp::get_logger("RmRobotAPI"), "rm_get_current_arm_state调用成功");
        
        // 打印机械臂状态信息（调试用）
        RCLCPP_DEBUG(rclcpp::get_logger("RmRobotAPI"), "当前位置相关信息:");
        RCLCPP_DEBUG(rclcpp::get_logger("RmRobotAPI"), "  位置: (%.3f, %.3f, %.3f) m", 
                    state.pose.position.x, state.pose.position.y, state.pose.position.z);
        RCLCPP_DEBUG(rclcpp::get_logger("RmRobotAPI"), "  欧拉角: (%.3f, %.3f, %.3f) rad", 
                    state.pose.euler.rx, state.pose.euler.ry, state.pose.euler.rz);
        RCLCPP_DEBUG(rclcpp::get_logger("RmRobotAPI"), "  四元数: (%.3f, %.3f, %.3f, %.3f)", 
                    state.pose.quaternion.w, state.pose.quaternion.x, state.pose.quaternion.y, state.pose.quaternion.z);
        RCLCPP_DEBUG(rclcpp::get_logger("RmRobotAPI"), "关节角度: ");
        for (int i = 0; i < 7; i++) {
            RCLCPP_DEBUG(rclcpp::get_logger("RmRobotAPI"), "  关节%d: %.2f", i+1, state.joint[i]);
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("RmRobotAPI"), "机械臂错误码: %d", state.err.err[0]);
        RCLCPP_DEBUG(rclcpp::get_logger("RmRobotAPI"), "错误码长度: %u", state.err.err_len);
    }
    
    return result;
}


int RmRobotAPI::rm_change_tool_frame(const std::string& tool_name) {
    // 检查输入参数
    if (tool_name.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "工具坐标系名称为空");
        return -1;
    }
    
    if (robot_handle_ == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "机器人句柄为空指针");
        return -1;
    }
    
    // 检查句柄ID是否有效
    if (robot_handle_->id <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "机器人句柄ID无效: %d", robot_handle_->id);
        return -1;
    }
    
    // 调用RM API的rm_change_tool_frame函数
    RCLCPP_DEBUG(rclcpp::get_logger("RmRobotAPI"), "调用rm_change_tool_frame，工具坐标系名称: %s", tool_name.c_str());
    int result = ::rm_change_tool_frame(robot_handle_, tool_name.c_str());
    
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "rm_change_tool_frame调用失败，错误码: %d", result);
        RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "目标工具坐标系名称: %s", tool_name.c_str());
        
        // 添加详细的错误码解释
        switch(result) {
            case 1:
                RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "错误详情: 控制器返回false，传递参数错误或机械臂状态发生错误");
                break;
            case -1:
                RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "错误详情: 数据发送失败，通信过程中出现问题");
                break;
            case -2:
                RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "错误详情: 数据接收失败，通信过程中出现问题或者控制器超时没有返回");
                break;
            case -3:
                RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "错误详情: 返回值解析失败，接收到的数据格式不正确或不完整");
                break;
            default:
                RCLCPP_ERROR(rclcpp::get_logger("RmRobotAPI"), "错误详情: 未知错误");
                break;
        }
        
        return result;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("RmRobotAPI"), "成功切换到工具坐标系: %s", tool_name.c_str());
    }
    
    return result;
}


// #endregion

}
