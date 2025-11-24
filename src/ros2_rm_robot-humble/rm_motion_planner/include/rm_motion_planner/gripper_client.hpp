#pragma once
#include <string>
#include <chrono>
#include <future>
#include <optional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rm_ros_interfaces/srv/initialize_gripper.hpp"
#include "rm_ros_interfaces/srv/activate_gripper.hpp"
#include "rm_ros_interfaces/srv/deactivate_gripper.hpp"
#include "rm_ros_interfaces/srv/close_modbus.hpp"
#include "rm_ros_interfaces/srv/get_paras_gripper.hpp"

namespace rm_motion_planner {

struct GripperResult {
    enum class Status { OK, TIMEOUT, FAILED, NOT_AVAILABLE };
    Status status;
    std::string message;
};

class GripperClient {
public:
    explicit GripperClient(rclcpp::Node::SharedPtr node, bool enable_logging = true);
    ~GripperClient();

    // 初始化（可选） — 返回成功与否
    GripperResult initialize(const std::string& ip, int port, int register_address, int comm_port, int device_address, std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

    // 激活吸盘（同步）
    GripperResult activate(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));
    // 非阻塞激活，返回 future
    std::future<GripperResult> activate_async(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

    GripperResult deactivate(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));
    std::future<GripperResult> deactivate_async(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

    GripperResult closeModbus(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));
    std::future<GripperResult> closeModbus_async(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

    std::optional<rm_ros_interfaces::srv::GetParasGripper::Response> getParameters(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

    // If service unavailable, allow fallback to system command (optional)
    void set_fallback_cmds(const std::string& init_cmd, const std::string& activate_cmd, const std::string& deactivate_cmd, const std::string& close_cmd);

private:
    rclcpp::Node::SharedPtr node_;
    bool enable_logging_;

    rclcpp::Client<rm_ros_interfaces::srv::InitializeGripper>::SharedPtr init_client_;
    rclcpp::Client<rm_ros_interfaces::srv::ActivateGripper>::SharedPtr activate_client_;
    rclcpp::Client<rm_ros_interfaces::srv::DeactivateGripper>::SharedPtr deactivate_client_;
    rclcpp::Client<rm_ros_interfaces::srv::CloseModbus>::SharedPtr close_client_;
    rclcpp::Client<rm_ros_interfaces::srv::GetParasGripper>::SharedPtr getparas_client_;

    // fallback system commands
    std::string init_cmd_;
    std::string activate_cmd_;
    std::string deactivate_cmd_;
    std::string close_cmd_;

    GripperResult call_activate_sync(std::chrono::milliseconds timeout, bool activate);
    GripperResult call_closemodbus_sync(std::chrono::milliseconds timeout);

    template<typename ServiceT, typename ReqT>
    std::pair<bool, std::string> call_service_sync(rclcpp::Client<ServiceT>& client, typename ServiceT::Request::SharedPtr req, std::chrono::milliseconds timeout);
};

} // namespace rm_motion_planner
