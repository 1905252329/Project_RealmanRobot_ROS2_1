#pragma once
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <functional>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace rm_motion_planner {

/**
 * TrajectoryExecutor
 *  - 按时间从 planned trajectory 中发布 joint commands
 *  - 支持 cancel / pause / resume
 *  - 以非阻塞方式运行（内部线程或 timer）
 */
class TrajectoryExecutor {
public:
    using ProgressCb = std::function<void(double progress_ratio)>; // 0..1
    using PointPublisher = std::function<void(const trajectory_msgs::msg::JointTrajectoryPoint& point)>;

    explicit TrajectoryExecutor(rclcpp::Node::SharedPtr node, PointPublisher publisher);
    ~TrajectoryExecutor();

    // 异步执行轨迹；返回 true 表示已开始执行
    bool execute(const trajectory_msgs::msg::JointTrajectory& traj, double execution_rate_hz = 50.0, ProgressCb progress_cb = nullptr);

    // 请求取消当前执行
    void cancel();

    bool is_executing() const;

private:
    void worker_loop();

    rclcpp::Node::SharedPtr node_;
    PointPublisher publisher_;

    std::thread worker_thread_;
    std::atomic<bool> executing_{false};
    std::atomic<bool> cancel_requested_{false};

    trajectory_msgs::msg::JointTrajectory current_traj_;
    double execution_rate_hz_{50.0};

    std::mutex mutex_;
    std::condition_variable cv_;
    ProgressCb progress_cb_;
};

} // namespace rm_motion_planner
