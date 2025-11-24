#include "rm_motion_planner/grasp_place_handlers.hpp"
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

using GraspBox = sorting_robot_interfaces::action::GraspBox;
using PlaceBox = sorting_robot_interfaces::action::PlaceBox;

namespace rm_motion_planner {

GraspBoxHandler::GraspBoxHandler(rclcpp::Node::SharedPtr node, std::shared_ptr<GripperClient> gripper, std::shared_ptr<TrajectoryExecutor> executor, std::shared_ptr<IRobotInterface> robot)
: ActionHandler(node, "grasp_action"), gripper_(gripper), executor_(executor), robot_(robot) {}

void GraspBoxHandler::doExecute(const std::shared_ptr<GoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<GraspBox::Result>();
    auto feedback = std::make_shared<GraspBox::Feedback>();

    // 简化示例步骤：1) activate gripper, 2) plan (skipped), 3) execute simple approach via robot interface, 4) deactivate
    feedback->status = 1;
    goal_handle->publish_feedback(feedback);

    auto gr_act = gripper_->activate(std::chrono::milliseconds(1000));
    if (gr_act.status != GripperResult::Status::OK) {
        result->success = false;
        result->message = "gripper activate failed";
        goal_handle->abort(result);
        return;
    }

    // Use robot interface to move to approach pose (mock)
    if (robot_) {
        std::vector<double> joints = {-60.0, -40.0, -50.0, -40.0, -120.0, 80.0, 70.0};
        robot_->rm_movej(joints, 40);
    }

    // pretend success
    std::this_thread::sleep_for(500ms);
    result->success = true;
    result->message = "grasp success (example)";
    goal_handle->succeed(result);
}

PlaceBoxHandler::PlaceBoxHandler(rclcpp::Node::SharedPtr node, std::shared_ptr<GripperClient> gripper, std::shared_ptr<IRobotInterface> robot)
: ActionHandler(node, "place_action"), gripper_(gripper), robot_(robot) {}

void PlaceBoxHandler::doExecute(const std::shared_ptr<GoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PlaceBox::Result>();
    auto feedback = std::make_shared<PlaceBox::Feedback>();

    feedback->status = 1;
    goal_handle->publish_feedback(feedback);

    // Move to place pose (mock)
    if (robot_) {
        std::vector<double> joints = {-110.0, -50.0, -30.0, -30.0, -60.0, 120.0, 100.0};
        robot_->rm_movej(joints, 40);
    }

    // deactivate gripper
    auto gr_deact = gripper_->deactivate(std::chrono::milliseconds(1000));
    if (gr_deact.status != GripperResult::Status::OK) {
        result->success = false;
        result->message = "gripper deactivate failed";
        goal_handle->abort(result);
        return;
    }

    result->success = true;
    result->message = "place success (example)";
    goal_handle->succeed(result);
}

} // namespace rm_motion_planner
