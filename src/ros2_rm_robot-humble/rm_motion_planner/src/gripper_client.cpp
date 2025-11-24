#include "rm_motion_planner/gripper_client.hpp"
#include <cstdlib> // system
#include <thread>

using namespace std::chrono_literals;

namespace rm_motion_planner {

GripperClient::GripperClient(rclcpp::Node::SharedPtr node, bool enable_logging)
: node_(node), enable_logging_(enable_logging)
{
    init_client_ = node_->create_client<rm_ros_interfaces::srv::InitializeGripper>("/left_arm/initialize_gripper");
    activate_client_ = node_->create_client<rm_ros_interfaces::srv::ActivateGripper>("/left_arm/activate_gripper");
    deactivate_client_ = node_->create_client<rm_ros_interfaces::srv::DeactivateGripper>("/left_arm/deactivate_gripper");
    close_client_ = node_->create_client<rm_ros_interfaces::srv::CloseModbus>("/left_arm/close_modbus");
    getparas_client_ = node_->create_client<rm_ros_interfaces::srv::GetParasGripper>("/left_arm/getParas_gripper");
}

GripperClient::~GripperClient() = default;

// (internal) removed generic template helper to keep implementation simple and avoid
// template/linkage errors in this example. Each service call is performed inline below.

GripperResult GripperClient::initialize(const std::string& ip, int port, int register_address, int comm_port, int device_address, std::chrono::milliseconds timeout)
{
    auto req = std::make_shared<rm_ros_interfaces::srv::InitializeGripper::Request>();
    req->ip = ip;
    req->port = port;
    req->register_address = register_address;
    req->comm_port = comm_port;
    req->device_address = device_address;

    if (init_client_->wait_for_service(500ms)) {
        auto fut = init_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, fut, timeout) == rclcpp::FutureReturnCode::SUCCESS) {
            return {GripperResult::Status::OK, "initialize success"};
        } else {
            return {GripperResult::Status::TIMEOUT, "initialize timeout"};
        }
    }

    // fallback
    if (!init_cmd_.empty()) {
        if (enable_logging_) RCLCPP_WARN(node_->get_logger(), "initialize service unavailable, using fallback cmd");
        int rc = std::system(init_cmd_.c_str());
        return rc == 0 ? GripperResult{GripperResult::Status::OK, "fallback initialize ok"} : GripperResult{GripperResult::Status::FAILED, "fallback init failed"};
    }

    return {GripperResult::Status::NOT_AVAILABLE, "service not available"};
}

GripperResult GripperClient::call_activate_sync(std::chrono::milliseconds timeout, bool activate)
{
    if (activate) {
        if (activate_client_->wait_for_service(200ms)) {
            auto req = std::make_shared<rm_ros_interfaces::srv::ActivateGripper::Request>();
            auto fut = activate_client_->async_send_request(req);
            if (rclcpp::spin_until_future_complete(node_, fut, timeout) == rclcpp::FutureReturnCode::SUCCESS) {
                return {GripperResult::Status::OK, "activate ok"};
            } else {
                return {GripperResult::Status::TIMEOUT, "activate timeout"};
            }
        }
        if (!activate_cmd_.empty()) {
            if (enable_logging_) RCLCPP_WARN(node_->get_logger(), "activate service unavailable, using fallback cmd");
            int rc = std::system(activate_cmd_.c_str());
            return rc == 0 ? GripperResult{GripperResult::Status::OK, "fallback activate ok"} : GripperResult{GripperResult::Status::FAILED, "fallback activate failed"};
        }
        return {GripperResult::Status::NOT_AVAILABLE, "activate service not available"};
    } else {
        if (deactivate_client_->wait_for_service(200ms)) {
            auto req = std::make_shared<rm_ros_interfaces::srv::DeactivateGripper::Request>();
            auto fut = deactivate_client_->async_send_request(req);
            if (rclcpp::spin_until_future_complete(node_, fut, timeout) == rclcpp::FutureReturnCode::SUCCESS) {
                return {GripperResult::Status::OK, "deactivate ok"};
            } else {
                return {GripperResult::Status::TIMEOUT, "deactivate timeout"};
            }
        }
        if (!deactivate_cmd_.empty()) {
            if (enable_logging_) RCLCPP_WARN(node_->get_logger(), "deactivate service unavailable, using fallback cmd");
            int rc = std::system(deactivate_cmd_.c_str());
            return rc == 0 ? GripperResult{GripperResult::Status::OK, "fallback deactivate ok"} : GripperResult{GripperResult::Status::FAILED, "fallback deactivate failed"};
        }
        return {GripperResult::Status::NOT_AVAILABLE, "deactivate service not available"};
    }
}

GripperResult GripperClient::activate(std::chrono::milliseconds timeout)
{
    return call_activate_sync(timeout, true);
}

std::future<GripperResult> GripperClient::activate_async(std::chrono::milliseconds timeout)
{
    return std::async(std::launch::async, [this, timeout]() { return call_activate_sync(timeout, true); });
}

GripperResult GripperClient::deactivate(std::chrono::milliseconds timeout)
{
    return call_activate_sync(timeout, false);
}

std::future<GripperResult> GripperClient::deactivate_async(std::chrono::milliseconds timeout)
{
    return std::async(std::launch::async, [this, timeout]() { return call_activate_sync(timeout, false); });
}

GripperResult GripperClient::closeModbus(std::chrono::milliseconds timeout)
{
    if (close_client_->wait_for_service(200ms)) {
        auto req = std::make_shared<rm_ros_interfaces::srv::CloseModbus::Request>();
        auto fut = close_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, fut, timeout) == rclcpp::FutureReturnCode::SUCCESS) {
            return {GripperResult::Status::OK, "close modbus ok"};
        } else {
            return {GripperResult::Status::TIMEOUT, "close modbus timeout"};
        }
    }
    if (!close_cmd_.empty()) {
        int rc = std::system(close_cmd_.c_str());
        return rc == 0 ? GripperResult{GripperResult::Status::OK, "fallback close ok"} : GripperResult{GripperResult::Status::FAILED, "fallback close failed"};
    }
    return {GripperResult::Status::NOT_AVAILABLE, "close modbus service not available"};
}

std::future<GripperResult> GripperClient::closeModbus_async(std::chrono::milliseconds timeout)
{
    return std::async(std::launch::async, [this, timeout]() { return closeModbus(timeout); });
}

std::optional<rm_ros_interfaces::srv::GetParasGripper::Response> GripperClient::getParameters(std::chrono::milliseconds timeout)
{
    if (!getparas_client_->wait_for_service(200ms)) {
        return std::nullopt;
    }
    auto req = std::make_shared<rm_ros_interfaces::srv::GetParasGripper::Request>();
    auto fut = getparas_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, fut, timeout) == rclcpp::FutureReturnCode::SUCCESS) {
        auto resp_ptr = fut.get();
        if (resp_ptr) {
            return *resp_ptr; // copy response into optional
        }
    }
    return std::nullopt;
}

void GripperClient::set_fallback_cmds(const std::string& init_cmd, const std::string& activate_cmd, const std::string& deactivate_cmd, const std::string& close_cmd)
{
    init_cmd_ = init_cmd;
    activate_cmd_ = activate_cmd;
    deactivate_cmd_ = deactivate_cmd;
    close_cmd_ = close_cmd;
}

// Explicitly instantiate the template for InitializeGripper so the linker finds it (if used).
// no explicit template instantiation required in this simplified implementation

} // namespace rm_motion_planner
