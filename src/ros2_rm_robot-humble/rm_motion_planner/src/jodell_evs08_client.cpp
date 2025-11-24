/*@brief Jodell Evs08 电吸盘功能测试

*/

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>

// 使用标准服务类型或自定义服务类型
// 注意：需要确保服务定义在接口包中可用
#include "rm_ros_interfaces/srv/initialize_gripper.hpp"
#include "rm_ros_interfaces/srv/activate_gripper.hpp"
#include "rm_ros_interfaces/srv/deactivate_gripper.hpp"
#include "rm_ros_interfaces/srv/close_modbus.hpp"

using namespace std::chrono_literals;

class JodellEvs08Client : public rclcpp::Node
{
public:

/*
    JodellEvs08Client()
    : Node("jodell_evs08_client")
    {
        // 创建客户端
        // 注意：服务名称应该与Python服务端注册的名称一致
        init_client_ = this->create_client<rm_ros_interfaces::srv::InitializeGripper>("initialize_gripper");
        activate_client_ = this->create_client<rm_ros_interfaces::srv::ActivateGripper>("activate_gripper");
        deactivate_client_ = this->create_client<rm_ros_interfaces::srv::DeactivateGripper>("deactivate_gripper");
        close_client_ = this->create_client<rm_ros_interfaces::srv::CloseModbus>("close_modbus");

        RCLCPP_INFO(this->get_logger(), "Jodell EVS08 Client node initialized");
    }
*/

    JodellEvs08Client(const std::string& namespace_name = "")
    : Node("jodell_evs08_client"), namespace_name_(namespace_name)
    {
        // 构造服务前缀
        if (!namespace_name_.empty()) {
            service_prefix_ = namespace_name_ + "/";
        }
        
        // 创建客户端，使用带命名空间的服务名称
        init_client_ = this->create_client<rm_ros_interfaces::srv::InitializeGripper>(service_prefix_ + "initialize_gripper");
        activate_client_ = this->create_client<rm_ros_interfaces::srv::ActivateGripper>(service_prefix_ + "activate_gripper");
        deactivate_client_ = this->create_client<rm_ros_interfaces::srv::DeactivateGripper>(service_prefix_ + "deactivate_gripper");
        close_client_ = this->create_client<rm_ros_interfaces::srv::CloseModbus>(service_prefix_ + "close_modbus");

        RCLCPP_INFO(this->get_logger(), "Jodell EVS08 Client node initialized with namespace: '%s'", 
                    namespace_name_.empty() ? "none" : namespace_name_.c_str());
    }

    /**
     * @brief 初始化夹爪
     * @param ip IP地址
     * @param port 端口号
     * @param register_address 寄存器地址
     * @param comm_port 通信端口
     * @param device_address 设备地址
     * @return 操作是否成功
     */
    bool initialize_gripper(const std::string& ip, int port, int register_address, int comm_port, int device_address)
    {
        while (!init_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<rm_ros_interfaces::srv::InitializeGripper::Request>();
        request->ip = ip;
        request->port = port;
        request->register_address = register_address;
        request->comm_port = comm_port;
        request->device_address = device_address;

        auto result = init_client_->async_send_request(request);
        
        // 等待结果
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "Initialize Gripper Response: %s", response->message.c_str());
            return response->success;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service initialize_gripper");
            return false;
        }
    }

    bool activate_gripper()
    {
        while (!activate_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<rm_ros_interfaces::srv::ActivateGripper::Request>();

        auto result = activate_client_->async_send_request(request);
        
        // 等待结果
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "Activate Gripper Response: %s", response->message.c_str());
            return response->success;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service activate_gripper");
            return false;
        }
    }

    bool deactivate_gripper()
    {
        while (!deactivate_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<rm_ros_interfaces::srv::DeactivateGripper::Request>();

        auto result = deactivate_client_->async_send_request(request);
        
        // 等待结果
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "Deactivate Gripper Response: %s", response->message.c_str());
            return response->success;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service deactivate_gripper");
            return false;
        }
    }

    bool close_modbus()
    {
        while (!close_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<rm_ros_interfaces::srv::CloseModbus::Request>();

        auto result = close_client_->async_send_request(request);
        
        // 等待结果
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "Close Modbus Response: %s", response->message.c_str());
            return response->success;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service close_modbus");
            return false;
        }
    }

private:
    // 命名空间名称
    std::string namespace_name_;
    
    // 服务名称前缀
    std::string service_prefix_;

    rclcpp::Client<rm_ros_interfaces::srv::InitializeGripper>::SharedPtr init_client_;
    rclcpp::Client<rm_ros_interfaces::srv::ActivateGripper>::SharedPtr activate_client_;
    rclcpp::Client<rm_ros_interfaces::srv::DeactivateGripper>::SharedPtr deactivate_client_;
    rclcpp::Client<rm_ros_interfaces::srv::CloseModbus>::SharedPtr close_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

//#region 服务端节点启动文件未采用命名空间机制
/*
    auto client_node = std::make_shared<JodellEvs08Client>();
    
    // 示例使用
    RCLCPP_INFO(client_node->get_logger(), "Starting Jodell EVS08 Gripper Control Demo");
    
    // 初始化夹爪
    if (client_node->initialize_gripper("169.254.247.18", 8080, 1000, 1, 9)) {
        RCLCPP_INFO(client_node->get_logger(), "Gripper initialized successfully");
        
        // 激活夹爪（启动真空）
        if (client_node->activate_gripper()) {
            RCLCPP_INFO(client_node->get_logger(), "Gripper activated - vacuum enabled");
            
            // 等待一段时间模拟抓取
            rclcpp::sleep_for(2s);
            
            // 停用夹爪（释放真空）
            if (client_node->deactivate_gripper()) {
                RCLCPP_INFO(client_node->get_logger(), "Gripper deactivated - vacuum released");
            }
        }
    }

    // 关闭Modbus模式
    client_node->close_modbus();
*/
//#endregion
    
    // 创建左臂吸盘客户端
    auto left_client = std::make_shared<JodellEvs08Client>("left_arm");
    
    // 创建右臂吸盘客户端
    auto right_client = std::make_shared<JodellEvs08Client>("right_arm");
    
    // 示例使用 - 左臂操作
    RCLCPP_INFO(left_client->get_logger(), "Starting Jodell EVS08 Gripper Control Demo for Left Arm");
    
    // 初始化左臂夹爪
    if (left_client->initialize_gripper("169.254.247.18", 8080, 1000, 1, 9)) {
        RCLCPP_INFO(left_client->get_logger(), "Left gripper initialized successfully");
        
        // 激活左臂夹爪（启动真空）
        if (left_client->activate_gripper()) {
            RCLCPP_INFO(left_client->get_logger(), "Left gripper activated - vacuum enabled");
            
            // 等待一段时间模拟抓取
            rclcpp::sleep_for(2s);
            
            // 停用左臂夹爪（释放真空）
            if (left_client->deactivate_gripper()) {
                RCLCPP_INFO(left_client->get_logger(), "Left gripper deactivated - vacuum released");
            }
        }
    }
    
    // 关闭左臂Modbus模式
    left_client->close_modbus();
    
    // 示例使用 - 右臂操作
    RCLCPP_INFO(right_client->get_logger(), "Starting Jodell EVS08 Gripper Control Demo for Right Arm");
    
    // 初始化右臂夹爪
    if (right_client->initialize_gripper("169.254.247.19", 8080, 1000, 1, 9)) {
        RCLCPP_INFO(right_client->get_logger(), "Right gripper initialized successfully");
        
        // 激活右臂夹爪（启动真空）
        if (right_client->activate_gripper()) {
            RCLCPP_INFO(right_client->get_logger(), "Right gripper activated - vacuum enabled");
            
            // 等待一段时间模拟抓取
            rclcpp::sleep_for(2s);
            
            // 停用右臂夹爪（释放真空）
            if (right_client->deactivate_gripper()) {
                RCLCPP_INFO(right_client->get_logger(), "Right gripper deactivated - vacuum released");
            }
        }
    }
    
    // 关闭右臂Modbus模式
    right_client->close_modbus();
    

    
    rclcpp::shutdown();
    return 0;
}