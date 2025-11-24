/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-09-01 19:46:40
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-09-12 08:32:50
 * @FilePath: /ros2_rm_ws/src/ros2_rm_robot-humble/rm_motion_planner/include/rm_motion_planner/jodell_evs08_interfaces.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file jodell_evs08_interfaces.h
 * @brief Jodell EVS08真空吸盘控制器接口类
 * @author your name
 * @date 2025-09-01
 */

#ifndef RM_MOTION_PLANNER_JODELL_EVS08_INTERFACES_H
#define RM_MOTION_PLANNER_JODELL_EVS08_INTERFACES_H

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <string>

// 使用标准服务类型或自定义服务类型
#include "rm_ros_interfaces/srv/initialize_gripper.hpp"
#include "rm_ros_interfaces/srv/activate_gripper.hpp"
#include "rm_ros_interfaces/srv/deactivate_gripper.hpp"
#include "rm_ros_interfaces/srv/close_modbus.hpp"

using namespace std::chrono_literals;

namespace rm_motion_planner {

/**
 * @class JodellEvs08Controller
 * @brief Jodell EVS08真空吸盘控制器类
 * 
 * 提供对Jodell EVS08真空吸盘的完整控制接口，包括初始化、激活、停用和关闭功能
 */
class JodellEvs08Controller : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param node_name 节点名称
     */
    // explicit JodellEvs08Controller(const std::string& node_name = "jodell_evs08_controller");

    explicit JodellEvs08Controller(
        const std::string& node_name = "jodell_evs08_controller",
        const std::string& namespace_name = "");
    /**
     * @brief 析构函数
     */
    ~JodellEvs08Controller() = default;

    /**
     * @brief 初始化夹爪
     * @param ip IP地址
     * @param port 端口号
     * @param register_address 寄存器地址
     * @param comm_port 通信端口
     * @param device_address 设备地址
     * @return 操作是否成功
     */
    bool initialize_gripper(const std::string& ip, int port, int register_address, int comm_port, int device_address);

    /**
     * @brief 激活夹爪（启动真空）
     * @return 操作是否成功
     */
    bool activate_gripper();

    /**
     * @brief 停用夹爪（释放真空）
     * @return 操作是否成功
     */
    bool deactivate_gripper();

    /**
     * @brief 关闭Modbus连接
     * @return 操作是否成功
     */
    bool close_modbus();

private:
    // 命名空间名称
    std::string namespace_name_;
    
    // 服务名称前缀
    std::string service_prefix_;

    // 客户端
    rclcpp::Client<rm_ros_interfaces::srv::InitializeGripper>::SharedPtr init_client_;
    rclcpp::Client<rm_ros_interfaces::srv::ActivateGripper>::SharedPtr   activate_client_;
    rclcpp::Client<rm_ros_interfaces::srv::DeactivateGripper>::SharedPtr deactivate_client_;
    rclcpp::Client<rm_ros_interfaces::srv::CloseModbus>::SharedPtr       close_client_;

    std::string get_service_name(const std::string& service_name);

};

} // namespace rm_motion_planner

#endif // RM_MOTION_PLANNER_JODELL_EVS08_INTERFACES_H