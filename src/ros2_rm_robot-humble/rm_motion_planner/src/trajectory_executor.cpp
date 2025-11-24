#include "rm_motion_planner/trajectory_executor.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace rm_motion_planner {

TrajectoryExecutor::TrajectoryExecutor(rclcpp::Node::SharedPtr node, PointPublisher publisher)
: node_(node), publisher_(publisher)
{
}

TrajectoryExecutor::~TrajectoryExecutor()
{
    cancel();
    if (worker_thread_.joinable()) worker_thread_.join();
}

bool TrajectoryExecutor::execute(const trajectory_msgs::msg::JointTrajectory& traj, double execution_rate_hz, ProgressCb progress_cb)
{
    if (traj.points.empty()) {
        RCLCPP_WARN(node_->get_logger(), "TrajectoryExecutor: empty trajectory");
        return false;
    }
    if (executing_.load()) {
        RCLCPP_WARN(node_->get_logger(), "TrajectoryExecutor: already executing");
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(mutex_);
        current_traj_ = traj;
        execution_rate_hz_ = execution_rate_hz;
        progress_cb_ = progress_cb;
        cancel_requested_.store(false);
        executing_.store(true);
    }

    worker_thread_ = std::thread(&TrajectoryExecutor::worker_loop, this);
    return true;
}

void TrajectoryExecutor::cancel()
{
    cancel_requested_.store(true);
    executing_.store(false);
    cv_.notify_all();
}

bool TrajectoryExecutor::is_executing() const { return executing_.load(); }

void TrajectoryExecutor::worker_loop()
{
    auto start_time = std::chrono::steady_clock::now();
    const auto &points = current_traj_.points;
    size_t idx = 0;
    const size_t total = points.size();
    rclcpp::Rate rate(static_cast<int>(std::round(execution_rate_hz_)));
    // Basic strategy: publish each point in sequence, wait interval derived from time_from_start differences
    for (idx = 0; idx < total && !cancel_requested_.load() && rclcpp::ok(); ++idx) {
        const auto &pt = points[idx];

        // publish
        try {
            publisher_(pt);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "TrajectoryExecutor publish exception: %s", e.what());
        }

        // progress callback
        if (progress_cb_) {
            double prog = static_cast<double>(idx + 1) / static_cast<double>(total);
            progress_cb_(prog);
        }

        // compute wait time to next point
        if (idx + 1 < total) {
            auto t_cur = pt.time_from_start;
            auto t_next = points[idx + 1].time_from_start;
            // compute duration to sleep
            auto sleep_ns = std::chrono::nanoseconds(t_next.sec * 1000000000LL + t_next.nanosec) -
                            std::chrono::nanoseconds(t_cur.sec * 1000000000LL + t_cur.nanosec);
            if (sleep_ns.count() > 0) {
                std::unique_lock<std::mutex> lk(mutex_);
                cv_.wait_for(lk, sleep_ns, [this]() { return cancel_requested_.load(); });
            } else {
                // fallback to base rate
                rate.sleep();
            }
        }
    }

    executing_.store(false);
    if (cancel_requested_.load()) {
        RCLCPP_WARN(node_->get_logger(), "TrajectoryExecutor: execution canceled");
    } else {
        RCLCPP_INFO(node_->get_logger(), "TrajectoryExecutor: execution finished");
    }
}

} // namespace rm_motion_planner
