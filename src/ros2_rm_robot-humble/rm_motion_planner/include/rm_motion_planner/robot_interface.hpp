#pragma once
#include <vector>
#include <optional>
#include <array>
#include <rclcpp/rclcpp.hpp>

namespace rm_motion_planner {

struct ArmState {
    struct Pose { float x,y,z; float rx,ry,rz; } pose;
    std::array<double,7> joints{0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    bool valid{false};
};

class IRobotInterface {
public:
    virtual ~IRobotInterface() = default;

    // movej: joint positions in degrees or radians as agreed
    virtual bool rm_movej(const std::vector<double>& joint_positions, int speed) = 0;

    // movel: tcp pose x,y,z,rx,ry,rz
    virtual bool rm_movel(const std::vector<float>& tcp_pose, int speed) = 0;

    // get current arm state
    virtual std::optional<ArmState> get_current_state() = 0;
};

} // namespace rm_motion_planner
