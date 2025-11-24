#pragma once
#include "rm_motion_planner/robot_interface.hpp"
#include <vector>

namespace rm_motion_planner {

class MockRobotInterfaceImpl : public IRobotInterface {
public:
    MockRobotInterfaceImpl() = default;
    bool rm_movej(const std::vector<double>& joint_positions, int speed) override;
    bool rm_movel(const std::vector<float>& tcp_pose, int speed) override;
    std::optional<ArmState> get_current_state() override;

private:
    std::vector<double> last_joints_;
    std::vector<float> last_tcp_;
    int last_speed_ = 0;
};

} // namespace rm_motion_planner
