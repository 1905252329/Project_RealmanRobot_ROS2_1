#include "rm_motion_planner/mock_robot_interface_impl.hpp"

namespace rm_motion_planner {

bool MockRobotInterfaceImpl::rm_movej(const std::vector<double>& joint_positions, int speed)
{
    last_joints_ = joint_positions;
    last_speed_ = speed;
    return true;
}

bool MockRobotInterfaceImpl::rm_movel(const std::vector<float>& tcp_pose, int speed)
{
    last_tcp_ = tcp_pose;
    last_speed_ = speed;
    return true;
}

std::optional<ArmState> MockRobotInterfaceImpl::get_current_state()
{
    ArmState s;
    s.valid = true;
    s.joints = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    s.pose = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    return s;
}

} // namespace rm_motion_planner
