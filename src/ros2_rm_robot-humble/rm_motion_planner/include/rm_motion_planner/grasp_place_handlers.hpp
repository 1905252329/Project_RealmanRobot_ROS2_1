#pragma once
#include "rm_motion_planner/action_handler.hpp"
#include "rm_motion_planner/gripper_client.hpp"
#include "rm_motion_planner/trajectory_executor.hpp"
#include "rm_motion_planner/robot_interface.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "sorting_robot_interfaces/action/grasp_box.hpp"
#include "sorting_robot_interfaces/action/place_box.hpp"

namespace rm_motion_planner {

class GraspBoxHandler : public ActionHandler<sorting_robot_interfaces::action::GraspBox> {
public:
    GraspBoxHandler(rclcpp::Node::SharedPtr node, std::shared_ptr<GripperClient> gripper, std::shared_ptr<TrajectoryExecutor> executor, std::shared_ptr<IRobotInterface> robot);
    void doExecute(const std::shared_ptr<GoalHandle> goal_handle) override;

private:
    std::shared_ptr<GripperClient> gripper_;
    std::shared_ptr<TrajectoryExecutor> executor_;
    std::shared_ptr<IRobotInterface> robot_;
};

class PlaceBoxHandler : public ActionHandler<sorting_robot_interfaces::action::PlaceBox> {
public:
    PlaceBoxHandler(rclcpp::Node::SharedPtr node, std::shared_ptr<GripperClient> gripper, std::shared_ptr<IRobotInterface> robot);
    void doExecute(const std::shared_ptr<GoalHandle> goal_handle) override;

private:
    std::shared_ptr<GripperClient> gripper_;
    std::shared_ptr<IRobotInterface> robot_;
};

} // namespace rm_motion_planner
