#define RUCKIG_ENABLE_DYNAMIC_API

#include <cstring>
#include <fstream>
#include <math.h>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <array>
#include <boost/asio.hpp>
#include <boost/array.hpp>


#include "utils.h"
#include "gripper.h"
#include "interpolator.h"
#include "aubo_sdk/rpc.h"           
#include "sclerp_interface.h"
#include "kinlib/collision_utils.h"
#include "comm_api/srv/get_pose.hpp"
#include "comm_api/get_pose_client.hpp"
#include "comm_api/command_status_server.hpp"
#include "aubo_servoj2_example/trajectory_generator.hpp"


using namespace boost::asio;
using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define LOCAL_IP "169.254.4.70"
constexpr size_t DOF = 6;

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const std::vector<T> &list)
{
    for (size_t i = 0; i < list.size(); i++) {
        os << list.at(i);
        if (i != (list.size() - 1)) {
            os << ",";
        }
    }
    return os;
}


#define CHECK_OR_BREAK(expr)                   \
    success = (expr);                          \
    if (!success) {                            \
        std::cerr << "Execution Failed: "      \
                  << #expr << std::endl;       \
        break;                                 \
    }


std::vector<double> tote_home_pose = {-0.6343832733457146, -0.5028918966127485, 0.7739226190885716,
                                        -179.2134124152658 * M_PI / 180,
                                        0.6184657449346161 * M_PI / 180,
                                        -0.23688742709644958 * M_PI / 180};

std::vector<double> tray_home_pose = {-0.04572375042378578, -0.44705759778270315, 0.6777666665739195,
                                    -178.45561409688642 * M_PI / 180,
                                    -0.19914911459279652 * M_PI / 180,
                                    -90.17899931417811 * M_PI / 180};

std::vector<double> place_home_pose = {0.58215, -0.69256, 0.36799,
                                    180.0 * M_PI / 180,
                                    0.0 * M_PI / 180,
                                    0.0 * M_PI / 180};


Eigen::Matrix4d parsePoseMessage(const std::string &message, unsigned &flag)
{
    Eigen::Matrix4d poses;
    std::vector<std::string> tokens;
    std::istringstream ss(message);
    std::string token;

    // Remove trailing "#" if present
    std::string clean_message = message;
    if (!message.empty() && message.back() == '#')
    {
        clean_message.pop_back();
    }

    // Split the string by '_'
    while (std::getline(ss, token, '_'))
    {
        // std::cout << token << std::endl;
        tokens.push_back(token);
    }

    if (tokens.size() != 8)
    {
        throw std::runtime_error("Invalid server message format. Expected 7 values(including #)).");
    }

    // left(frame) Extract pose to left base
    double x = std::stod(tokens[0]);
    double y = std::stod(tokens[1]);
    double z = std::stod(tokens[2]);
    double roll = std::stod(tokens[3]) * M_PI / 180;
    double pitch = std::stod(tokens[4]) * M_PI / 180;
    double yaw = std::stod(tokens[5]) * M_PI / 180;
    flag = std::stoi(tokens[6]);

    return EulerToTransformation(x, y, z, roll, pitch, yaw);
}

// 实现阻塞功能: 当机械臂运动到目标路点时，程序再往下执行
int waitArrival(RobotInterfacePtr impl)
{
    const int max_retry_count = 5;
    int cnt = 0;

    // 接口调用: 获取当前的运动指令 ID
    int exec_id = impl->getMotionControl()->getExecId();

    // 等待机械臂开始运动
    while (exec_id == -1) {
        if (cnt++ > max_retry_count) {
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    // 等待机械臂动作完成
    while (impl->getMotionControl()->getExecId() != -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 1;
}

// move to home positions
/**
 * @brief 规划并执行机械臂回到指定home位置的运动
 * 
 * 该函数首先获取机械臂当前关节位置，然后根据指定的home位置名称计算目标位姿，
 * 使用ScLERP规划器进行路径规划，并通过servo模式执行规划的轨迹。
 * 
 * @param rpc_cli RPC客户端指针，用于与机器人通信
 * @param planner_interface ScLERP规划器接口引用，用于路径规划
 * @param home 目标home位置名称，可选值："home_tote"、"home_tray"、"home_paper"
 * @return int 执行结果，1表示成功，0表示失败
 */
int plan_to_home(RpcClientPtr rpc_cli, sclerp_interface::ScLERPInterface &planner_interface, const std::string &home)
{
    Eigen::Matrix4d g_base_tool;
    
    Eigen::VectorXd joint_values(6);
    std::vector<double> joint_positions;
    trajectory_msgs::msg::JointTrajectory motion_plan;

    // 获取当前关节位置
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

    // 根据home参数设置目标位姿
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


    std::cout << "Initial left pose is:\n"
            << g_base_tool << '\n';
    std::cout << "Target left pose is:\n"
            << goal << '\n';

    motion_plan.points.clear();

    // <---- 规划阶段 ---->
    // 调用规划器进行路径规划
    bool result = planner_interface.solve(joint_values, goal, motion_plan);

    std::cout << "Motion Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << motion_plan.points.size() << "\n";

    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    // 轨迹插值处理
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

    // <---- 执行阶段 ---->
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

    // Loop through the trajectory and send each position at 500 Hz (2 ms interval)

    std::cout << "Starting ServoJ motion..." << std::endl;

    std::cout << "Trajectory size: " <<  trajectory.size() << "\n";

    // 以250Hz频率执行关节轨迹
    for (size_t i = 1; i < trajectory.size(); i++) {
        // 接口调用: 关节伺服运动
        rpc_cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->servoJoint(trajectory[i], 3, 6, 1/frequency, 0.2, 200);

        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }

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

int getVisionTote(std::shared_ptr<comm_api::GetPoseClient> get_pose_client, Eigen::Matrix4d &server_poses, const std::string &request_type, unsigned &flag) {
    try {
        std::string raw_poses;
    
        // First request
        raw_poses = get_pose_client->send_request(request_type);
        std::cout << "Received poses from server:\n" << raw_poses << std::endl;
        server_poses = parsePoseMessage(raw_poses, flag);
    
        // Retry once if flag == 0
        if (flag == 0) {
            std::cout << "Flag == 0 detected, retrying request once..." << std::endl;
            raw_poses = get_pose_client->send_request(request_type);
            std::cout << "Received poses from server (retry):\n" << raw_poses << std::endl;
            server_poses = parsePoseMessage(raw_poses, flag);

            if (flag == 0) {
                std::cerr << "Empty tote. Aborting.\n";
                return 0;
            }
        }
    
    } catch (const std::exception &e) {
        std::cerr << "Error receiving poses from server: " << e.what() << std::endl;
        return 0;
    }

    return 1;
}

int getVisionTray(std::shared_ptr<comm_api::GetPoseClient> get_pose_client, Eigen::Matrix4d &server_poses, const std::string &request_type, unsigned &flag) {
    try {
        std::string raw_poses;
    
        // First request
        raw_poses = get_pose_client->send_request(request_type);
        std::cout << "Received poses from server:\n" << raw_poses << std::endl;
        server_poses = parsePoseMessage(raw_poses, flag);
    
        // Retry once if flag == 0
        if (flag == 0) {
            std::cout << "Flag == 0 detected, retrying request once..." << std::endl;
            raw_poses = get_pose_client->send_request(request_type);
            std::cout << "Received poses from server (retry):\n" << raw_poses << std::endl;
            server_poses = parsePoseMessage(raw_poses, flag);

            if (flag == 0) {
                std::cerr << "Flag is still 0 after retry. Aborting.\n";
                // should return 0 but handled outside
                return 1;
            }
        }
    
    } catch (const std::exception &e) {
        std::cerr << "Error receiving poses from server: " << e.what() << std::endl;
        return 0;
    }

    return 1;
}

// move to grasp pose based on vision
int pick_in(RpcClientPtr rpc_cli, sclerp_interface::ScLERPInterface &planner_interface, Eigen::Matrix4d &server_poses) {


    Eigen::Matrix4d g_base_tool, intermediate_pose;

    Eigen::VectorXd joint_values(6);
    std::vector<double> joint_positions;
    trajectory_msgs::msg::JointTrajectory intermediate_plan, motion_plan;

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

    // use when need to start from current pose
    planner_interface.kinlib_solver_.getFK(joint_values, g_base_tool);
                    
    std::cout << "Initial pose is:\n"
                          << g_base_tool << '\n';
    std::cout << "Target pose is:\n"
                          << server_poses << '\n';

    std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> collision_objects;

    Eigen::Vector3d dimension_l, dimension_s;
    dimension_l << 0.600, 0.020, 0.178;
    dimension_s << 0.400, 0.020, 0.178;

    Eigen::Vector3d position_1, position_2, position_3, position_4;
    position_1 << -0.27407,-0.56798, 0.089;
    position_2 << 0.01882,-0.37523, 0.089;
    position_3 << 0.02199,-0.76288, 0.089;
    position_4 << 0.31226,-0.56534, 0.089;

    Eigen::Matrix3d rotation;
    rotation << 0, -1, 0,
                1, 0, 0,
                0, 0, 1;

    auto box_1 = CollisionUtils::createBox(dimension_s, position_1, rotation);
    auto box_2 = CollisionUtils::createBox(dimension_l, position_2, Eigen::Matrix3d::Identity());
    auto box_3 = CollisionUtils::createBox(dimension_l, position_3, Eigen::Matrix3d::Identity());
    auto box_4 = CollisionUtils::createBox(dimension_s, position_4, rotation);

    // collision_objects.push_back(box_1);
    // collision_objects.push_back(box_2);
    // collision_objects.push_back(box_3);
    // collision_objects.push_back(box_4);

    intermediate_pose = server_poses;
    intermediate_pose(2, 3) += 0.020; // lift the end effector by 2 cm
    int result = planner_interface.solve(joint_values, intermediate_pose, false, 1, collision_objects, nullptr, intermediate_plan);

    std::cout << "Intermediate Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << intermediate_plan.points.size() << "\n";

    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    std::vector<std::vector<double>> intermediate_path, path, intermediate_trajectory, trajectory;
    Convert_to_2dVector(intermediate_plan, intermediate_path);

    // Maximum velocities (rad/s) for each joint
    std::vector<double> max_velocity = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

    // Maximum accelerations (rad/s^2) for each joint 
    std::vector<double> max_acceleration = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};

    double frequency = 250;

    QuinticInterpolator interpolator_1st(intermediate_path, max_velocity, max_acceleration, frequency);

    interpolator_1st.enableResampling(true, {0, 5, intermediate_path.size() - 10, intermediate_path.size() - 1}, 5.0);
    interpolator_1st.computeTrajectory();
    intermediate_trajectory = interpolator_1st.getTrajectory();

    for (size_t i = 0; i < 6; ++i)
    {
        joint_values[i] = intermediate_plan.points.back().positions[i];
    }

    result = planner_interface.solve(joint_values, server_poses, false, 1, collision_objects, nullptr, motion_plan);
    
    std::cout << "Pick Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << motion_plan.points.size() << "\n";
    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    Convert_to_2dVector(motion_plan, path);

    QuinticInterpolator interpolator_2nd(path, max_velocity, max_acceleration, frequency);

    interpolator_2nd.computeTrajectory();
    trajectory = interpolator_2nd.getTrajectory();

    std::vector<std::vector<double>> concatenated;
    concatenated.reserve(intermediate_trajectory.size() + trajectory.size());
    concatenated.insert(concatenated.end(), intermediate_trajectory.begin(), intermediate_trajectory.end());
    concatenated.insert(concatenated.end(), trajectory.begin(), trajectory.end());

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

    // Loop through the trajectory and send each position at 500 Hz (2 ms interval)

    std::cout << "Starting ServoJ motion..." << std::endl;

    std::cout << "Trajectory size: " <<  concatenated.size() << "\n";

    for (size_t i = 1; i < concatenated.size(); i++) {
        // 接口调用: 关节伺服运动
        rpc_cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->servoJoint(concatenated[i], 3, 6, 1/frequency, 0.2, 200);

        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }

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


/**
* @brief 在新的位置进行拾取操作
*
* 使用给定的RPC客户端和路径规划接口，将机械臂移动到指定的目标位置进行拾取操作。
*
* @param rpc_cli RPC客户端指针，用于与机器人进行通信
* @param planner_interface 路径规划接口，用于生成机械臂的运动轨迹
* @param server_poses 目标位置的位姿矩阵
*
* @return 操作成功返回1，失败返回0
*/
int pick_in_new(RpcClientPtr rpc_cli, sclerp_interface::ScLERPInterface &planner_interface, Eigen::Matrix4d &server_poses) 
{

    Eigen::Matrix4d g_base_tool, intermediate_pose;

    Eigen::VectorXd joint_values(6);
    std::vector<double> joint_positions;
    trajectory_msgs::msg::JointTrajectory intermediate_plan, motion_plan;

    // 获取当前关节位置
    // Get current joints
    auto robot_name = rpc_cli->getRobotNames().front();
    joint_positions =
        rpc_cli->getRobotInterface(robot_name)->getRobotState()->getJointPositions();

    // 🧪 打印当前关节位置
    // std::cout << "Current joints: ";
    for (size_t i = 0; i < joint_positions.size(); ++i)
    {
        joint_values[i] = joint_positions[i];
        // std::cout << joint_values[i] << " ";
    }
    // std::cout << "\n";

    // 使用当前位姿作为起点
    // use when need to start from current pose
    planner_interface.kinlib_solver_.getFK(joint_values, g_base_tool);
                    
    std::cout << "Initial pose is:\n"
                          << g_base_tool << '\n';
    std::cout << "Target pose is:\n"
                          << server_poses << '\n';

    std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> collision_objects;

    Eigen::Vector3d dimension_l, dimension_s;
    dimension_l << 0.600, 0.020, 0.178;
    dimension_s << 0.400, 0.020, 0.178;

    Eigen::Vector3d position_1, position_2, position_3, position_4;
    position_1 << -0.27407,-0.56798, 0.089;
    position_2 << 0.01882,-0.37523, 0.089;
    position_3 << 0.02199,-0.76288, 0.089;
    position_4 << 0.31226,-0.56534, 0.089;

    Eigen::Matrix3d rotation;
    rotation << 0, -1, 0,
                1, 0, 0,
                0, 0, 1;

    auto box_1 = CollisionUtils::createBox(dimension_s, position_1, rotation);
    auto box_2 = CollisionUtils::createBox(dimension_l, position_2, Eigen::Matrix3d::Identity());
    auto box_3 = CollisionUtils::createBox(dimension_l, position_3, Eigen::Matrix3d::Identity());
    auto box_4 = CollisionUtils::createBox(dimension_s, position_4, rotation);

    // 📝 ✨ 添加碰撞对象到列表（暂时注释掉）
    // collision_objects.push_back(box_1);
    // collision_objects.push_back(box_2);
    // collision_objects.push_back(box_3);
    // collision_objects.push_back(box_4);

    intermediate_pose = server_poses;
    intermediate_pose(2, 3) += 0.020; // 将末端执行器上抬2厘米
    int result = planner_interface.solve(joint_values, intermediate_pose, false, 1, collision_objects, nullptr, intermediate_plan);

    std::cout << "Intermediate Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << intermediate_plan.points.size() << "\n";

    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    std::vector<std::vector<double>> intermediate_path, path, intermediate_trajectory, trajectory;
    Convert_to_2dVector(intermediate_plan, intermediate_path);

    // 定义每个关节的最大速度（弧度/秒）
    // Maximum velocities (rad/s) for each joint
    std::vector<double> max_velocity = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

    // ⚡ 定义每个关节的最大加速度（弧度/秒^2）
    // Maximum accelerations (rad/s^2) for each joint 
    std::vector<double> max_acceleration = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};

    double frequency = 250;

    TrajectoryInterpolator interpolator_1st(intermediate_path, max_velocity, max_acceleration, frequency);

    interpolator_1st.enableResampling(true, {0, 5, intermediate_path.size() - 10, intermediate_path.size() - 1}, 5.0);
    interpolator_1st.computeTrajectory();
    intermediate_trajectory = interpolator_1st.getTrajectory();

    interpolator_1st.exportSpeedProfileToCSV("/home/terry/speed.csv");
    interpolator_1st.exportAccelerationProfileToCSV("/home/terry/acceleration.csv");


    for (size_t i = 0; i < 6; ++i)
    // 设置最终关节位置为中间轨迹的最后一个点
    {
        joint_values[i] = intermediate_plan.points.back().positions[i];
    }

    result = planner_interface.solve(joint_values, server_poses, false, 1, collision_objects, nullptr, motion_plan);
    
    std::cout << "Pick Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << motion_plan.points.size() << "\n";
    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    Convert_to_2dVector(motion_plan, path);

    TrajectoryInterpolator interpolator_2nd(path, max_velocity, max_acceleration, frequency);
    // 创建第二个轨迹插值器
        
    interpolator_2nd.computeTrajectory();
    trajectory = interpolator_2nd.getTrajectory();

    std::vector<std::vector<double>> concatenated;
    concatenated.reserve(intermediate_trajectory.size() + trajectory.size());
    concatenated.insert(concatenated.end(), intermediate_trajectory.begin(), intermediate_trajectory.end());
    concatenated.insert(concatenated.end(), trajectory.begin(), trajectory.end());

    // 接口调用: 开启 servo 模式
    // 开启伺服模式
    rpc_cli->getRobotInterface(robot_name)->getMotionControl()->setServoMode(true);

    // 等待进入 servo 模式
    // 等待进入伺服模式
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

    // Loop through the trajectory and send each position at 500 Hz (2 ms interval)
    // 遍历轨迹，并以500Hz（2毫秒间隔）发送每个位置

    std::cout << "Starting ServoJ motion..." << std::endl;

    std::cout << "Trajectory size: " <<  concatenated.size() << "\n";

    for (size_t i = 1; i < concatenated.size(); i++) {
        // 接口调用: 关节伺服运动
        rpc_cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->servoJoint(concatenated[i], 3, 6, 1/frequency, 0.2, 200);

        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }

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

// move translationally based on current pose
int pick_out_from_tote(RpcClientPtr rpc_cli, sclerp_interface::ScLERPInterface &planner_interface)
{
    Eigen::Matrix4d g_base_tool;
    // Eigen::Matrix4d intermediate_pose = EulerToTransformation(-0.20117, -0.58864, 0.62871, -178.65336246053766 * M_PI / 180, -4.901632968961911 * M_PI / 180, -90.08761310422234 * M_PI / 180);
    // Eigen::Matrix4d place_pose = EulerToTransformation(0.1260447540394012, -0.588638107126707, 0.4888580310484811, -178.65336246053766 * M_PI / 180, -4.901632968961911 * M_PI / 180, -90.08761310422234 * M_PI / 180);

    Eigen::VectorXd joint_values(6);
    std::vector<double> joint_positions;
    trajectory_msgs::msg::JointTrajectory motion_plan, trans_plan, intermediate_plan;

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

    // use when need to start from current pose
    planner_interface.kinlib_solver_.getFK(joint_values, g_base_tool);

    Eigen::Matrix4d goal = g_base_tool;
    goal(2, 3) += 0.2;
    // Eigen::Matrix4d intermediate_pose = TransWithFixedRz(goal, -0.20117, -0.58864, 0.62871);
    Eigen::Matrix4d intermediate_pose = TransWithFixedRz(goal, -0.41773, -0.49578, 0.69165);


    Eigen::Matrix4d place_pose = intermediate_pose;
    place_pose(0, 3) = 0.01472;
    place_pose(1, 3) = -0.54751;
    place_pose(2, 3) = 0.43279;

    std::cout << "Initial left pose is:\n"
            << g_base_tool << '\n';
    std::cout << "Target left pose is:\n"
            << goal << '\n';

    trans_plan.points.clear();
    motion_plan.points.clear();
    intermediate_plan.points.clear();

    bool result = planner_interface.solve(joint_values, goal, trans_plan);

    std::cout << "Trans Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << trans_plan.points.size() << "\n";

    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    for (size_t i = 0; i < 6; ++i)
    {
        joint_values[i] = trans_plan.points.back().positions[i];
    }

    result = planner_interface.solve(joint_values, intermediate_pose, intermediate_plan);

    std::cout << "Intermediate Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << intermediate_plan.points.size() << "\n";

    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    for (size_t i = 0; i < 6; ++i)
    {
        joint_values[i] = intermediate_plan.points.back().positions[i];
    }

    result = planner_interface.solve(joint_values, place_pose, motion_plan);

    std::cout << "Place Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << motion_plan.points.size() << "\n";
    
    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    std::set<size_t> key_indices = {0};  // start
    key_indices.insert(trans_plan.points.size() - 1);  // trans
    key_indices.insert(trans_plan.points.size() + intermediate_plan.points.size() - 1);  // intermediate
    key_indices.insert(trans_plan.points.size() + intermediate_plan.points.size() + motion_plan.points.size() - 1);  // goal
    
    for (const auto &point : intermediate_plan.points)
    {
        trans_plan.points.push_back(point);
    }

    for (const auto &point : motion_plan.points)
    {
        trans_plan.points.push_back(point);
    }

    std::vector<std::vector<double>> path, trajectory;
    Convert_to_2dVector(trans_plan, path);

    // Maximum velocities (rad/s) for each joint
    std::vector<double> max_velocity = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

    // Maximum accelerations (rad/s^2) for each joint 
    std::vector<double> max_acceleration = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};

    double frequency = 250;

    QuinticInterpolator interpolator(path, max_velocity, max_acceleration, frequency);

    interpolator.enableResampling(true, key_indices, 5.0);
    interpolator.computeTrajectory();

    trajectory = interpolator.getTrajectory();

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

int place_out_from_tray(RpcClientPtr rpc_cli, sclerp_interface::ScLERPInterface &planner_interface, Eigen::Matrix4d &server_poses)
{
    Eigen::Matrix4d g_base_tool;

    // actual place pose
    // Eigen::Matrix4d intermediate_pose = EulerToTransformation(0.6525857315264451, -0.17918206641141036, 0.44778926971286787, -178.49419833842015 * M_PI / 180, 0.3772647612475788 * M_PI / 180, -90.08414442948163 * M_PI / 180);
    // Eigen::Matrix4d place_pose = EulerToTransformation(0.6525857315264451, -0.07918206641141036, 0.44778926971286787, -178.49419833842015 * M_PI / 180, 0.3772647612475788 * M_PI / 180, -90.08414442948163 * M_PI / 180);
    // Eigen::Matrix4d place_pose = EulerToTransformation(place_home_pose[0], place_home_pose[1], place_home_pose[2], server_poses[3], server_poses[4], server_poses[5]);
    Eigen::Matrix4d place_pose = server_poses;
    place_pose(0, 3) = place_home_pose[0];
    place_pose(1, 3) = place_home_pose[1];
    place_pose(2, 3) = place_home_pose[2];

    Eigen::Matrix4d intermediate_pose;
    intermediate_pose = place_pose;
    intermediate_pose(0, 3) -= 0.10;
    // intermediate_pose(1, 3) = place_home_pose[1];
    // intermediate_pose(2, 3) = place_home_pose[2];
    // test place pose
    // Eigen::Matrix4d intermediate_pose = EulerToTransformation(0.3, -0.3472580181245007, 0.5132206068845714, -178.65060941942505 * M_PI / 180, -4.892888487486562 * M_PI / 180, -0.6279396678268753 * M_PI / 180);
    // Eigen::Matrix4d place_pose = EulerToTransformation(0.3463718027282355, -0.3472580181245007, 0.3859027606726624, -178.65060941942505 * M_PI / 180, -4.892888487486562 * M_PI / 180, -0.6279396678268753 * M_PI / 180);

    Eigen::VectorXd joint_values(6);
    std::vector<double> joint_positions;
    trajectory_msgs::msg::JointTrajectory motion_plan, trans_plan, intermediate_plan;

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

    // use when need to start from current pose
    planner_interface.kinlib_solver_.getFK(joint_values, g_base_tool);

    Eigen::Matrix4d goal = g_base_tool;
    goal(2, 3) += 0.1;

    std::cout << "Initial left pose is:\n"
            << g_base_tool << '\n';
    std::cout << "Target left pose is:\n"
            << goal << '\n';

    trans_plan.points.clear();
    motion_plan.points.clear();
    intermediate_plan.points.clear();

    bool result = planner_interface.solve(joint_values, goal, trans_plan);

    std::cout << "Trans Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << trans_plan.points.size() << "\n";

    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    for (size_t i = 0; i < 6; ++i)
    {
        joint_values[i] = trans_plan.points.back().positions[i];
    }

    result = planner_interface.solve(joint_values, intermediate_pose, intermediate_plan);

    std::cout << "Intermediate Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << intermediate_plan.points.size() << "\n";

    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    for (size_t i = 0; i < 6; ++i)
    {
        joint_values[i] = intermediate_plan.points.back().positions[i];
    }

    result = planner_interface.solve(joint_values, place_pose, motion_plan);

    std::cout << "Place Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << motion_plan.points.size() << "\n";
    
    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    std::set<size_t> key_indices = {0};  // start
    key_indices.insert(trans_plan.points.size() - 1);  // trans
    key_indices.insert(trans_plan.points.size() + intermediate_plan.points.size() - 1);  // intermediate
    key_indices.insert(trans_plan.points.size() + intermediate_plan.points.size() + motion_plan.points.size() - 1);  // goal
    
    for (const auto &point : intermediate_plan.points)
    {
        trans_plan.points.push_back(point);
    }

    for (const auto &point : motion_plan.points)
    {
        trans_plan.points.push_back(point);
    }

    std::vector<std::vector<double>> path, trajectory;
    Convert_to_2dVector(trans_plan, path);

    // Maximum velocities (rad/s) for each joint
    std::vector<double> max_velocity = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

    // Maximum accelerations (rad/s^2) for each joint 
    std::vector<double> max_acceleration = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
    
    double frequency = 250;

    QuinticInterpolator interpolator(path, max_velocity, max_acceleration, frequency);

    interpolator.enableResampling(true, key_indices, 5.0);
    interpolator.computeTrajectory();

    trajectory = interpolator.getTrajectory();

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

// move to home positions
int move_to_home_purej(RpcClientPtr cli, const std::string &home)
{
    std::vector<double> joint_positions;
    
    if (home == "home_tote") {
        joint_positions = {
            -127.15462561881189 * (M_PI / 180),  -7.597694925742575 * (M_PI / 180), 76.32735020661157 * (M_PI / 180),
            -6.477258663366336 * (M_PI / 180), 88.970838490099 * (M_PI / 180), -36.87105507425742 * (M_PI / 180)
        };
    }
    else if (home == "home_tray") {
        joint_positions = {
            -69.68982054455445 * (M_PI / 180),  27.49334777227723 * (M_PI / 180), 114.33877840909092 * (M_PI / 180),
            -2.57085396039604 * (M_PI / 180), 91.34050123762377 * (M_PI / 180),  110.52181311881186 * (M_PI / 180)
        };
    }
    else if (home == "home_safe") {
        joint_positions = {
            349.0 * (M_PI / 180),  27.39 * (M_PI / 180), 112.81 * (M_PI / 180),
            -4.86 * (M_PI / 180), 91.07 * (M_PI / 180),  27.22 * (M_PI / 180)
        };
    }
    else if (home == "home_place") {
        joint_positions = {
            -28.226717202970296 * (M_PI / 180),  -16.29950495049505 * (M_PI / 180), 99.66851756198349 * (M_PI / 180),
            24.927212252475247 * (M_PI / 180), 88.20668316831683 * (M_PI / 180),  -67.06319616336634 * (M_PI / 180)
        };
    }
    else {
        std::cerr << "Invalid home position specified: " << home << std::endl;
        return 0;
    }

    // 接口调用: 获取机器人的名字
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // 接口调用: 设置机械臂的速度比率
    // robot_interface->getMotionControl()->setSpeedFraction(1);

    // 接口调用: 关节运动
    robot_interface->getMotionControl()->moveJoint(
        joint_positions, 60 * (M_PI / 180), 90 * (M_PI / 180), 0, 0);
    // 阻塞
    int ret = waitArrival(robot_interface);
    if (ret == 1) {
        std::cout << "Pure movej to home succeed" << std::endl;
    } else {
        std::cout << "Pure movej to home failed" << std::endl;
        // return 0;
    }

    return 1;
}

// move to home positions
int move_to_home_purel(RpcClientPtr cli, const std::string &home)
{
    std::vector<double> home_pose;
    
    if (home == "home_tote") {
        home_pose = tote_home_pose;
    }
    else if (home == "home_tote") {
        home_pose = tray_home_pose;
    }
    else {
        std::cerr << "Invalid home position specified: " << home << std::endl;
        return 0;
    }

    // 接口调用: 获取机器人的名字
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // 接口调用: 设置机械臂的速度比率
    // robot_interface->getMotionControl()->setSpeedFraction(1);

    // 接口调用: 关节运动
    robot_interface->getMotionControl()->moveLine(home_pose, 1.2, 0.25, 0.025, 0);
    // 阻塞
    int ret = waitArrival(robot_interface);
    if (ret == 1) {
        std::cout << "Pure movel to home succeed" << std::endl;
    } else {
        std::cout << "Pure movel to home failed" << std::endl;
        // return 0;
    }

    return 1;
}

int replace_in_from_tray(RpcClientPtr rpc_cli, sclerp_interface::ScLERPInterface &planner_interface)
{
    Eigen::Matrix4d g_base_tool;

    Eigen::VectorXd joint_values(6);
    std::vector<double> joint_positions;
    trajectory_msgs::msg::JointTrajectory motion_plan, trans_plan, intermediate_plan;

    // Eigen::Matrix4d intermediate_pose = EulerToTransformation(-0.5249855913405773, -0.4704908713361779, 0.7656270509746197, -178.65500489034304 * M_PI / 180, -4.902095002338989 * M_PI / 180, -90.08900665917471 * M_PI / 180);
    // Eigen::Matrix4d place_pose = intermediate_pose;
    // place_pose(2, 3) -= 0.2;

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

    // use when need to start from current pose
    planner_interface.kinlib_solver_.getFK(joint_values, g_base_tool);

    Eigen::Matrix4d goal = g_base_tool;
    goal(2, 3) += 0.1;

    std::cout << "Initial left pose is:\n"
            << g_base_tool << '\n';
    std::cout << "Target left pose is:\n"
            << goal << '\n';

    trans_plan.points.clear();
    motion_plan.points.clear();
    intermediate_plan.points.clear();

    bool result = planner_interface.solve(joint_values, goal, trans_plan);

    std::cout << "Trans Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << trans_plan.points.size() << "\n";

    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    Eigen::Matrix4d intermediate_pose = goal;
    intermediate_pose(0, 3) = -0.476724;
    intermediate_pose(1, 3) = -0.5028918966127485;
    intermediate_pose(2, 3) = 0.7739038520799802;
    
    Eigen::Matrix4d place_pose = intermediate_pose;
    place_pose(2, 3) -= 0.2;

    
    for (size_t i = 0; i < 6; ++i)
    {
        joint_values[i] = trans_plan.points.back().positions[i];
    }

    result = planner_interface.solve(joint_values, intermediate_pose, intermediate_plan);

    std::cout << "Intermediate Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << intermediate_plan.points.size() << "\n";

    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    for (size_t i = 0; i < 6; ++i)
    {
        joint_values[i] = intermediate_plan.points.back().positions[i];
    }

    result = planner_interface.solve(joint_values, place_pose, motion_plan);

    std::cout << "Place Plan Result: " << result << "\n";
    std::cout << "Number of waypoints: " << motion_plan.points.size() << "\n";
    
    if (result != 1) {
        std::cout << "Motion Plan Failed: " << result << "\n";
        return 0;
    }

    std::set<size_t> key_indices = {0};  // start
    key_indices.insert(trans_plan.points.size() - 1);  // trans
    key_indices.insert(trans_plan.points.size() + intermediate_plan.points.size() - 1);  // intermediate
    key_indices.insert(trans_plan.points.size() + intermediate_plan.points.size() + motion_plan.points.size() - 1);  // goal
    
    for (const auto &point : intermediate_plan.points)
    {
        trans_plan.points.push_back(point);
    }

    for (const auto &point : motion_plan.points)
    {
        trans_plan.points.push_back(point);
    }

    std::vector<std::vector<double>> path, trajectory;
    Convert_to_2dVector(trans_plan, path);

    // Maximum velocities (rad/s) for each joint
    std::vector<double> max_velocity = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

    // Maximum accelerations (rad/s^2) for each joint 
    std::vector<double> max_acceleration = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};

    double frequency = 250;

    QuinticInterpolator interpolator(path, max_velocity, max_acceleration, frequency);

    interpolator.enableResampling(true, key_indices, 5.0);
    interpolator.computeTrajectory();

    trajectory = interpolator.getTrajectory();

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


// logic of full workflow
bool handle_command(int64_t command, RpcClientPtr rpc_cli, sclerp_interface::ScLERPInterface &planner_interface, GripperController &gripper, std::shared_ptr<comm_api::GetPoseClient> get_pose_client) {
    
    unsigned flag = 0;
    bool success = true;
    Eigen::Matrix4d server_poses;

    auto robot_name = rpc_cli->getRobotNames().front();
    auto robot_interface = rpc_cli->getRobotInterface(robot_name);
    
    switch (command) {
        case 1: // start from tote
            while (rclcpp::ok)
            {    
                CHECK_OR_BREAK(move_to_home_purej(rpc_cli, "home_tote"));
                // CHECK_OR_BREAK(move_to_home_purel(rpc_cli, "home_tote"));
            
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
                CHECK_OR_BREAK(getVisionTote(get_pose_client, server_poses, "req_1", flag));
                CHECK_OR_BREAK(plan_to_home(rpc_cli, planner_interface, "home_tote"));
                CHECK_OR_BREAK(pick_in(rpc_cli, planner_interface, server_poses));
                CHECK_OR_BREAK(gripper.setGripperPosition(10));
                CHECK_OR_BREAK(pick_out_from_tote(rpc_cli, planner_interface));
                CHECK_OR_BREAK(gripper.setGripperPosition(210));
                CHECK_OR_BREAK(move_to_home_purej(rpc_cli, "home_tray"));
            
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
                CHECK_OR_BREAK(getVisionTray(get_pose_client, server_poses, "req_2", flag));
                
                if (flag == 1) {
                    CHECK_OR_BREAK(plan_to_home(rpc_cli, planner_interface, "home_tray"));
                    CHECK_OR_BREAK(pick_in(rpc_cli, planner_interface, server_poses));
                    CHECK_OR_BREAK(gripper.setGripperPosition(10));
                    CHECK_OR_BREAK(place_out_from_tray(rpc_cli, planner_interface, server_poses));
                    CHECK_OR_BREAK(gripper.setGripperPosition(210));
                    CHECK_OR_BREAK(move_to_home_purel(rpc_cli, "home_tote"));
                }
                else if (flag == 2) {
                    CHECK_OR_BREAK(plan_to_home(rpc_cli, planner_interface, "home_tray"));
                    CHECK_OR_BREAK(pick_in(rpc_cli, planner_interface, server_poses));
                    CHECK_OR_BREAK(gripper.setGripperPosition(10));
                    CHECK_OR_BREAK(replace_in_from_tray(rpc_cli, planner_interface));
                    CHECK_OR_BREAK(gripper.setGripperPosition(210));
                    std::cerr << "Failed to place a ordered object, retrying." << std::endl;
                    continue;
                }
                else {
                    std::cerr << "Invalid flag No., retrying" << std::endl;
                    continue;
                }
                
                break;
            }

            break;
        case 2: // start from tray
            CHECK_OR_BREAK(move_to_home_purej(rpc_cli, "home_tray"));
        
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
            CHECK_OR_BREAK(getVisionTray(get_pose_client, server_poses, "req_2", flag));
            
            if (flag == 1) {
                CHECK_OR_BREAK(plan_to_home(rpc_cli, planner_interface, "home_tray"));
                CHECK_OR_BREAK(pick_in(rpc_cli, planner_interface, server_poses));
                CHECK_OR_BREAK(gripper.setGripperPosition(10));
                CHECK_OR_BREAK(place_out_from_tray(rpc_cli, planner_interface, server_poses));
                CHECK_OR_BREAK(gripper.setGripperPosition(210));
                CHECK_OR_BREAK(move_to_home_purej(rpc_cli, "home_tray"));
            }
            else {
                std::cerr << "There is no required object." << std::endl;
                CHECK_OR_BREAK(move_to_home_purej(rpc_cli, "home_tray"));
            }

            break;
        case 3:
            CHECK_OR_BREAK(move_to_home_purej(rpc_cli, "home_tote"));
            // CHECK_OR_BREAK(move_to_home_purel(rpc_cli, "home_tote"));
        
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
            CHECK_OR_BREAK(getVisionTote(get_pose_client, server_poses, "req_1", flag));
            CHECK_OR_BREAK(plan_to_home(rpc_cli, planner_interface, "home_tote"));
            CHECK_OR_BREAK(pick_in(rpc_cli, planner_interface, server_poses));
            CHECK_OR_BREAK(gripper.setGripperPosition(10));
            CHECK_OR_BREAK(pick_out_from_tote(rpc_cli, planner_interface));
            CHECK_OR_BREAK(gripper.setGripperPosition(210));
            CHECK_OR_BREAK(move_to_home_purej(rpc_cli, "home_tray"));
            break;
        case 4:
            CHECK_OR_BREAK(move_to_home_purej(rpc_cli, "home_tote"));
            // CHECK_OR_BREAK(move_to_home_purel(rpc_cli, "home_tote"));
        
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
            CHECK_OR_BREAK(getVisionTote(get_pose_client, server_poses, "req_1", flag));
            CHECK_OR_BREAK(plan_to_home(rpc_cli, planner_interface, "home_tote"));
            CHECK_OR_BREAK(pick_in_new(rpc_cli, planner_interface, server_poses));
            CHECK_OR_BREAK(gripper.setGripperPosition(10));
            CHECK_OR_BREAK(pick_out_from_tote(rpc_cli, planner_interface));
            CHECK_OR_BREAK(gripper.setGripperPosition(210));
            CHECK_OR_BREAK(move_to_home_purej(rpc_cli, "home_tray"));
            break;

        default:
            std::cerr << "Invalid command received: " << command << std::endl;
            return false;
    }
    return success;
}

int main(int argc, char* argv[])
{
    auto rpc_cli = std::make_shared<RpcClient>();
    rpc_cli->setRequestTimeout(2000);
    rpc_cli->connect(LOCAL_IP, 30004);
    rpc_cli->login("aubo", "123456");
 
    auto robot_name = rpc_cli->getRobotNames().front();
    auto robot_interface = rpc_cli->getRobotInterface(robot_name);

    robot_interface->getMotionControl()->setSpeedFraction(1);

    GripperController gripper("/dev/ttyUSB0");
    if (!gripper.connect(1)) {
        return 0;
    }

    // if (!gripper.activateGripper(0x0100)) { 
    //     return 0;
    // }

    if (!gripper.setGripperPosition(210)) {
        std::cerr << "Failed to set the gripper position.\n";
        return 0;
    }

    rclcpp::init(argc, argv);

    auto sclerp = std::make_shared<rclcpp::Node>("sclerp_planner", rclcpp::NodeOptions().use_intra_process_comms(true));
    auto get_pose_client = std::make_shared<comm_api::GetPoseClient>(rclcpp::NodeOptions());

    std::vector<std::string> stl_files;
    stl_files = { "/home/terry/Documents/aubo/link_0.stl",
                  "/home/terry/Documents/aubo/link_1.stl",
                  "/home/terry/Documents/aubo/link_2.stl",
                  "/home/terry/Documents/aubo/link_3.stl",
                  "/home/terry/Documents/aubo/link_4.stl",
                  "/home/terry/Documents/aubo/link_5.stl",
                  "/home/terry/Documents/aubo/link_6.stl"};

    sclerp_interface::ScLERPInterface planner_interface("base_link", "wrist3_Link", sclerp, "/home/terry/Documents/aubo/aubo_i10.urdf", stl_files);

    auto handle_command_lambda = [rpc_cli, &planner_interface, &gripper, get_pose_client](int64_t command) -> bool
    {
        return handle_command(command, rpc_cli, planner_interface, gripper, get_pose_client);
    };

    auto command_status_server = std::make_shared<comm_api::CommandStatusServer>(handle_command_lambda);

    rclcpp::spin(command_status_server);
    rclcpp::shutdown();

    rpc_cli->logout();
    rpc_cli->disconnect();
    
    return 0;
}
