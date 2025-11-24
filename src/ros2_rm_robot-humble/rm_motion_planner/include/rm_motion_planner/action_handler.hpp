#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

namespace rm_motion_planner {

/**
 * 简单的 ActionHandler 基类，提供规范化动作处理流程
 * 目标：减少各个动作 server 中重复的线程管理、取消判断与反馈逻辑
 *
 * 用法：
 *  - 继承并实现 doExecute(const Goal & goal, ServerGoalHandle) -> Result
 *  - 基类负责接收/取消/accept 与线程调度
 *
 */
template<typename ActionT>
class ActionHandler {
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;

    ActionHandler(rclcpp::Node::SharedPtr node, const std::string &action_name)
    : node_(node), action_name_(action_name) {}

    virtual ~ActionHandler() = default;

    typename rclcpp_action::Server<ActionT>::SharedPtr create_server() {
        auto server = rclcpp_action::create_server<ActionT>(
            node_,
            action_name_,
            std::bind(&ActionHandler::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionHandler::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionHandler::handle_accepted, this, std::placeholders::_1));
        return server;
    }

protected:
    virtual rclcpp_action::GoalResponse handle_goal_validate(const Goal &goal) {
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    virtual rclcpp_action::CancelResponse handle_cancel_custom(const std::shared_ptr<GoalHandle> goal_handle) {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    virtual void doExecute(const std::shared_ptr<GoalHandle> goal_handle) = 0;

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Goal> goal) {
        (void)uuid;
        return handle_goal_validate(*goal);
    }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
        return handle_cancel_custom(goal_handle);
    }
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        // run in separate thread using rclcpp executor if needed
        std::thread([this, goal_handle]() {
            this->doExecute(goal_handle);
        }).detach();
    }

    rclcpp::Node::SharedPtr node_;
    std::string action_name_;
};

} // namespace rm_motion_planner
