#pragma once
#include "rm_motion_planner/robot_interface.hpp"

namespace rm_motion_planner { namespace test {

class MockRobotInterface : public IRobotInterface {
public:
    MockRobotInterface() = default;
    bool rm_movej(const std::vector<double>& joint_positions, int speed) override {
        last_joints_ = joint_positions;
        last_speed_ = speed;
        return true;
    }
    bool rm_movel(const std::vector<float>& tcp_pose, int speed) override {
        last_tcp_ = tcp_pose;
        last_speed_ = speed;
        return true;
    }
    std::optional<ArmState> get_current_state() override {
        ArmState s;
        s.valid = true;
        s.joints = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        s.pose = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
        return s;
    }

    std::vector<double> last_joints_;
    std::vector<float> last_tcp_;
    int last_speed_ = 0;
};

}} // namespace
