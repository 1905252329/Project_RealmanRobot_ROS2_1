#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rm_motion_planner/gripper_client.hpp"
#include "rm_motion_planner/trajectory_executor.hpp"
#include "rm_motion_planner/grasp_place_handlers.hpp"
#include "rm_motion_planner/robot_interface.hpp"
#include "rm_motion_planner/mock_robot_interface_impl.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rm_sclerp_planner_node_leftArm_refactor");

    // create components
    auto gripper = std::make_shared<rm_motion_planner::GripperClient>(node, true);
    auto executor = std::make_shared<rm_motion_planner::TrajectoryExecutor>(node, [node](const trajectory_msgs::msg::JointTrajectoryPoint& pt){
        // publish to driver topic example (stub)
        RCLCPP_INFO(node->get_logger(), "[refactor] publishing joint point");
    });

    // For now use mock robot interface implementation; later inject real implementation
    auto robot = std::make_shared<rm_motion_planner::MockRobotInterfaceImpl>();

    // create action handlers and servers
    auto grasp_handler = std::make_shared<rm_motion_planner::GraspBoxHandler>(node, gripper, executor, robot);
    auto place_handler = std::make_shared<rm_motion_planner::PlaceBoxHandler>(node, gripper, robot);

    auto grasp_server = grasp_handler->create_server();
    auto place_server = place_handler->create_server();

    RCLCPP_INFO(node->get_logger(), "rm_sclerp_planner_node_leftArm_refactor started");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
