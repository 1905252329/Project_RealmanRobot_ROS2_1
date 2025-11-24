#include <vector>
#include <set>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>


class QuinticInterpolator {
public:
    QuinticInterpolator(const std::vector<std::vector<double>>& path,
                            const std::vector<double>& max_velocity,
                            const std::vector<double>& max_acceleration,
                            double frequency)
        : original_path_(path), max_velocity_(max_velocity),
          max_acceleration_(max_acceleration), frequency_(frequency) {}

    void enableResampling(bool enable, const std::set<size_t>& key_indices, double resample_factor) {
        resampling_enabled_ = enable;
        key_indices_ = key_indices;
        resample_factor_ = resample_factor;
    }

    void computeTrajectory() {
        path_ = resampling_enabled_ ? resamplePath(original_path_, key_indices_) : original_path_;

        if (path_.size() < 2 || frequency_ <= 0.0) {
            std::cerr << "Invalid path or frequency in computeTrajectory\n";
            return;
        }

        size_t num_joints = path_[0].size();
        size_t num_waypoints = path_.size();
        double dt = 1.0 / frequency_;

        trajectory_.clear();

        std::vector<double> time_intervals(num_waypoints - 1, dt);
        for (size_t i = 0; i < num_waypoints - 1; ++i) {
            double max_time = dt;
            for (size_t j = 0; j < num_joints; ++j) {
                double dq = std::abs(path_[i + 1][j] - path_[i][j]);
                if (dq < 1e-6) continue;
                double t_v = dq / std::max(max_velocity_[j], 1e-6);
                double t_a = std::sqrt(2 * dq / std::max(max_acceleration_[j], 1e-6));
                max_time = std::max(max_time, std::max(t_v, t_a));
            }
            time_intervals[i] = max_time;
        }

        std::vector<std::vector<double>> velocities(num_waypoints, std::vector<double>(num_joints, 0.0));
        std::vector<std::vector<double>> accelerations(num_waypoints, std::vector<double>(num_joints, 0.0));

        for (size_t j = 0; j < num_joints; ++j) {
            for (size_t i = 1; i < num_waypoints - 1; ++i) {
                double dt1 = time_intervals[i - 1];
                double dt2 = time_intervals[i];
                double denom = dt1 + dt2;
                if (denom < 1e-6) continue;
                velocities[i][j] = (path_[i + 1][j] - path_[i - 1][j]) / denom;
                velocities[i][j] = std::clamp(velocities[i][j], -max_velocity_[j], max_velocity_[j]);
                accelerations[i][j] = 0.0;
            }
            velocities[0][j] = velocities[num_waypoints - 1][j] = 0.0;
            accelerations[0][j] = accelerations[num_waypoints - 1][j] = 0.0;
        }

        for (size_t i = 0; i < num_waypoints - 1; ++i) {
            double T = time_intervals[i];
            if (T < 1e-6) continue;
            size_t num_steps = static_cast<size_t>(std::ceil(T / dt));

            const auto& q0 = path_[i];
            const auto& q1 = path_[i + 1];
            const auto& v0 = velocities[i];
            const auto& v1 = velocities[i + 1];
            const auto& a0 = accelerations[i];
            const auto& a1 = accelerations[i + 1];

            for (size_t step = 0; step <= num_steps; ++step) {
                double t = step * dt;
                double tau = std::min(t / T, 1.0);
                double tau2 = tau * tau;
                double tau3 = tau2 * tau;
                double tau4 = tau3 * tau;
                double tau5 = tau4 * tau;

                std::vector<double> q(num_joints);
                for (size_t j = 0; j < num_joints; ++j) {
                    double p0 = q0[j], p1 = q1[j];
                    double v0_t = v0[j] * T, v1_t = v1[j] * T;
                    double a0_t = a0[j] * T * T, a1_t = a1[j] * T * T;

                    double c0 = p0;
                    double c1 = v0_t;
                    double c2 = a0_t / 2.0;
                    double c3 = (20 * (p1 - p0) - (8 * v1_t + 12 * v0_t) - (3 * a0_t - a1_t)) / 2.0;
                    double c4 = (-30 * (p1 - p0) + (14 * v1_t + 16 * v0_t) + (3 * a0_t - 2 * a1_t)) / 2.0;
                    double c5 = (12 * (p1 - p0) - (6 * v1_t + 6 * v0_t) - (a0_t - a1_t)) / 2.0;

                    q[j] = c0 + c1 * tau + c2 * tau2 + c3 * tau3 + c4 * tau4 + c5 * tau5;
                }
                trajectory_.push_back(q);
            }
        }

        if (!trajectory_.empty()) {
            trajectory_.back() = path_.back();
        }
    }

    void exportSpeedProfileToCSV(const std::string& filename) const {
        if (trajectory_.size() < 2) return;
        size_t num_joints = trajectory_[0].size();
        double dt = 1.0 / frequency_;

        std::ofstream ofs(filename);
        ofs << std::fixed << std::setprecision(8);

        // CSV header
        ofs << "time";
        for (size_t j = 0; j < num_joints; ++j)
            ofs << ",joint" << j;
        ofs << "\n";

        // Compute velocities (finite difference)
        for (size_t i = 1; i < trajectory_.size(); ++i) {
            ofs << i * dt;
            for (size_t j = 0; j < num_joints; ++j) {
                double v = (trajectory_[i][j] - trajectory_[i-1][j]) / dt;
                ofs << "," << v;
            }
            ofs << "\n";
        }
        ofs.close();
    }

    // Export joint acceleration profile to CSV
    void exportAccelerationProfileToCSV(const std::string& filename) const {
        if (trajectory_.size() < 3) return;
        size_t num_joints = trajectory_[0].size();
        double dt = 1.0 / frequency_;

        std::ofstream ofs(filename);
        ofs << std::fixed << std::setprecision(8);

        // CSV header
        ofs << "time";
        for (size_t j = 0; j < num_joints; ++j)
            ofs << ",joint" << j;
        ofs << "\n";

        // Compute accelerations (finite difference of velocity)
        for (size_t i = 2; i < trajectory_.size(); ++i) {
            ofs << i * dt;
            for (size_t j = 0; j < num_joints; ++j) {
                double v_prev = (trajectory_[i-1][j] - trajectory_[i-2][j]) / dt;
                double v_now  = (trajectory_[i][j]   - trajectory_[i-1][j]) / dt;
                double a = (v_now - v_prev) / dt;
                ofs << "," << a;
            }
            ofs << "\n";
        }
        ofs.close();
    }

    const std::vector<std::vector<double>>& getTrajectory() const {
        return trajectory_;
    }

private:
    std::vector<std::vector<double>> original_path_;
    std::vector<std::vector<double>> path_;
    std::vector<std::vector<double>> trajectory_;
    std::vector<double> max_velocity_;
    std::vector<double> max_acceleration_;
    double resample_factor_;
    double frequency_;

    bool resampling_enabled_ = false;
    std::set<size_t> key_indices_;

    double computeDesiredSpacing(const std::vector<std::vector<double>>& path, double scale_factor = 1.5) {
        if (path.size() < 2 || path[0].empty()) return 0.01;
        double total_distance = 0.0, min_distance = std::numeric_limits<double>::max();
        for (size_t i = 1; i < path.size(); ++i) {
            double dist = 0.0;
            for (size_t j = 0; j < path[i].size(); ++j)
                dist += std::pow(path[i][j] - path[i - 1][j], 2);
            dist = std::sqrt(dist);
            total_distance += dist;
            min_distance = std::min(min_distance, dist);
        }
        double avg_distance = total_distance / (path.size() - 1);
        return avg_distance * scale_factor;
    }

    std::vector<std::vector<double>> resamplePath(const std::vector<std::vector<double>>& path,
                                                  const std::set<size_t>& key_indices) {
        std::vector<std::vector<double>> uniform_path;
        if (path.size() < 2) return path;

        double desired_spacing = computeDesiredSpacing(path, resample_factor_);

        for (auto it = key_indices.begin(); std::next(it) != key_indices.end(); ++it) {
            size_t start = *it, end = *std::next(it);
            if (start >= path.size() || end >= path.size()) continue;

            std::vector<double> cumulative{0.0};
            for (size_t i = start + 1; i <= end; ++i) {
                double dist = 0.0;
                for (size_t j = 0; j < path[i].size(); ++j)
                    dist += std::pow(path[i][j] - path[i - 1][j], 2);
                cumulative.push_back(cumulative.back() + std::sqrt(dist));
            }

            double total_dist = cumulative.back();
            size_t num_resample = static_cast<size_t>(total_dist / desired_spacing);
            if (num_resample == 0) continue;

            for (size_t k = 0; k <= num_resample; ++k) {
                double target_dist = k * total_dist / num_resample;
                size_t seg = 0;
                while (seg < cumulative.size() - 1 && cumulative[seg + 1] < target_dist) ++seg;
                if (seg + 1 >= cumulative.size()) continue;
                double ratio = (target_dist - cumulative[seg]) /
                               (cumulative[seg + 1] - cumulative[seg] + 1e-8);

                std::vector<double> q(path[0].size());
                for (size_t j = 0; j < q.size(); ++j)
                    q[j] = path[start + seg][j] * (1 - ratio) + path[start + seg + 1][j] * ratio;

                uniform_path.push_back(q);
            }
        }

        if (!uniform_path.empty() && uniform_path.back() != path.back())
            uniform_path.push_back(path.back());

        return uniform_path;
    }
};


class SynchronizedInterpolator {
public:
    SynchronizedInterpolator(const std::vector<std::vector<std::vector<std::vector<double>>>>& robots_paths,
                        const std::vector<double>& max_velocity,
                        const std::vector<double>& max_acceleration,
                        double frequency)
        : original_robot_paths_(robots_paths),
          max_velocity_(max_velocity),
          max_acceleration_(max_acceleration),
          frequency_(frequency) {}

    void enableResampling(bool enable) {
        resampling_enabled_ = enable;
    }

    void computeSynchronizedTrajectories() {
        if (original_robot_paths_.empty() || frequency_ <= 0.0) return;

        size_t num_robots = original_robot_paths_.size();
        size_t num_segments = original_robot_paths_[0].size();
        trajectories_.resize(num_robots);

        for (size_t seg = 0; seg < num_segments; ++seg) {
            // Collect segment[seg] from each robot
            std::vector<std::vector<std::vector<double>>> segment_group(num_robots);
            for (size_t r = 0; r < num_robots; ++r) {
                segment_group[r] = resampling_enabled_ ? resamplePathSpatially(original_robot_paths_[r][seg], 10.0) : original_robot_paths_[r][seg];
            }

            // Synchronize this segment across robots
            auto synced_segment = synchronizeMultiPath(segment_group);

            // Interpolate each robot's segment and record duration
            std::vector<std::vector<std::vector<double>>> interpolated_segments(num_robots);
            std::vector<double> segment_durations(num_robots);
            double max_duration = 0.0;

            for (size_t r = 0; r < num_robots; ++r) {
                interpolated_segments[r] = computeSingleTrajectory(synced_segment[r], segment_durations[r], seg == num_segments - 1);
                max_duration = std::max(max_duration, segment_durations[r]);
            }

            // Resample each segment to the maximum segment duration
            for (size_t r = 0; r < num_robots; ++r) {
                auto resampled = resampleToDuration(interpolated_segments[r], segment_durations[r], max_duration, frequency_);
                trajectories_[r].insert(trajectories_[r].end(), resampled.begin(), resampled.end());
            }
        }
    }

    const std::vector<std::vector<std::vector<double>>>& getTrajectories() const {
        return trajectories_;
    }

    static std::vector<std::vector<double>> resamplePathSpatially(const std::vector<std::vector<double>>& path, double desired_spacing) {
        if (path.size() < 2) return path;

        std::vector<std::vector<double>> resampled;
        resampled.push_back(path[0]);

        double accumulated = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            std::vector<double> p0 = path[i - 1];
            std::vector<double> p1 = path[i];
            double dist = 0.0;
            for (size_t j = 0; j < p0.size(); ++j)
                dist += std::pow(p1[j] - p0[j], 2);
            dist = std::sqrt(dist);

            accumulated += dist;
            if (accumulated >= desired_spacing) {
                resampled.push_back(p1);
                accumulated = 0.0;
            }
        }

        if (resampled.back() != path.back())
            resampled.push_back(path.back());

        return resampled;
    }

private:
    std::vector<std::vector<std::vector<std::vector<double>>>> original_robot_paths_;
    std::vector<std::vector<std::vector<double>>> trajectories_;
    std::vector<double> max_velocity_;
    std::vector<double> max_acceleration_;
    double frequency_;
    bool resampling_enabled_ = false;

    static std::vector<std::vector<std::vector<double>>> synchronizeMultiPath(
        const std::vector<std::vector<std::vector<double>>>& path_segments) {
        if (path_segments.empty()) return {};

        size_t num_robots = path_segments.size();
        size_t max_len = 0;
        for (const auto& seg : path_segments) max_len = std::max(max_len, seg.size());

        size_t dof = path_segments[0][0].size();
        std::vector<std::vector<std::vector<double>>> synced_segments(num_robots);

        for (size_t r = 0; r < num_robots; ++r) {
            const auto& segment = path_segments[r];
            if (segment.size() == max_len) {
                synced_segments[r] = segment;
            } else {
                std::vector<std::vector<double>> resampled;
                for (size_t k = 0; k < max_len; ++k) {
                    double t = static_cast<double>(k) / (max_len - 1);
                    double idx_f = t * (segment.size() - 1);
                    size_t idx = static_cast<size_t>(std::floor(idx_f));
                    double alpha = idx_f - idx;
                    if (idx + 1 >= segment.size()) {
                        resampled.push_back(segment.back());
                    } else {
                        std::vector<double> interp(dof);
                        for (size_t j = 0; j < dof; ++j)
                            interp[j] = segment[idx][j] * (1 - alpha) + segment[idx + 1][j] * alpha;
                        resampled.push_back(interp);
                    }
                }
                synced_segments[r] = resampled;
            }
        }
        return synced_segments;
    }

    static std::vector<std::vector<double>> resampleToDuration(
        const std::vector<std::vector<double>>& traj,
        double old_duration,
        double new_duration,
        double frequency) {
        if (traj.empty() || old_duration <= 0.0 || new_duration <= 0.0 || frequency <= 0.0)
            return {};

        size_t new_size = static_cast<size_t>(std::ceil(new_duration * frequency));
        size_t old_size = traj.size();
        std::vector<std::vector<double>> resampled;

        for (size_t i = 0; i < new_size; ++i) {
            double t = static_cast<double>(i) / (new_size - 1);
            double old_t = t * (old_size - 1);
            size_t idx = static_cast<size_t>(std::floor(old_t));
            double alpha = old_t - idx;

            if (idx + 1 >= old_size) {
                resampled.push_back(traj.back());
            } else {
                std::vector<double> interp(traj[0].size());
                for (size_t j = 0; j < interp.size(); ++j)
                    interp[j] = traj[idx][j] * (1.0 - alpha) + traj[idx + 1][j] * alpha;
                resampled.push_back(interp);
            }
        }
        return resampled;
    }

    std::vector<std::vector<double>> computeSingleTrajectory(const std::vector<std::vector<double>>& path,
                                                            double& total_duration,
                                                            bool is_last_segment,
                                                            const std::vector<double>& initial_velocity = {}) {
        std::vector<std::vector<double>> trajectory;
        if (path.size() < 2 || frequency_ <= 0.0) {
            total_duration = 0.0;
            return trajectory;
        }

        size_t num_joints = path[0].size();
        size_t num_waypoints = path.size();
        double dt = 1.0 / frequency_;

        std::vector<double> time_intervals(num_waypoints - 1, dt);
        for (size_t i = 0; i < num_waypoints - 1; ++i) {
            double max_time = dt;
            for (size_t j = 0; j < num_joints; ++j) {
                double dq = std::abs(path[i + 1][j] - path[i][j]);
                if (dq < 1e-6) continue;
                double t_v = dq / std::max(max_velocity_[j], 1e-6);
                double t_a = std::sqrt(2 * dq / std::max(max_acceleration_[j], 1e-6));
                max_time = std::max(max_time, std::max(t_v, t_a));
            }
            time_intervals[i] = max_time;
        }

        std::vector<std::vector<double>> velocities(num_waypoints, std::vector<double>(num_joints, 0.0));
        std::vector<std::vector<double>> accelerations(num_waypoints, std::vector<double>(num_joints, 0.0));

        for (size_t j = 0; j < num_joints; ++j) {
            for (size_t i = 1; i < num_waypoints - 1; ++i) {
                double dt1 = time_intervals[i - 1];
                double dt2 = time_intervals[i];
                double denom = dt1 + dt2;
                if (denom < 1e-6) continue;
                velocities[i][j] = (path[i + 1][j] - path[i - 1][j]) / denom;
                velocities[i][j] = std::clamp(velocities[i][j], -max_velocity_[j], max_velocity_[j]);
                accelerations[i][j] = 0.0;
            }
            velocities[0][j] = !initial_velocity.empty() ? initial_velocity[j] : 0.0;
            velocities[num_waypoints - 1][j] = is_last_segment ? 0.0 : velocities[num_waypoints - 2][j];
            accelerations[0][j] = accelerations[num_waypoints - 1][j] = 0.0;
        }

        total_duration = 0.0;
        for (size_t i = 0; i < num_waypoints - 1; ++i) {
            double T = time_intervals[i];
            if (T < 1e-6) continue;
            size_t num_steps = static_cast<size_t>(std::ceil(T / dt));
            total_duration += T;

            const auto& q0 = path[i];
            const auto& q1 = path[i + 1];
            const auto& v0 = velocities[i];
            const auto& v1 = velocities[i + 1];
            const auto& a0 = accelerations[i];
            const auto& a1 = accelerations[i + 1];

            for (size_t step = 0; step <= num_steps; ++step) {
                double t = step * dt;
                double tau = std::min(t / T, 1.0);
                double tau2 = tau * tau;
                double tau3 = tau2 * tau;
                double tau4 = tau3 * tau;
                double tau5 = tau4 * tau;

                std::vector<double> q(num_joints);
                for (size_t j = 0; j < num_joints; ++j) {
                    double p0 = q0[j], p1 = q1[j];
                    double v0_t = v0[j] * T, v1_t = v1[j] * T;
                    double a0_t = a0[j] * T * T, a1_t = a1[j] * T * T;

                    double c0 = p0;
                    double c1 = v0_t;
                    double c2 = a0_t / 2.0;
                    double c3 = (20 * (p1 - p0) - (8 * v1_t + 12 * v0_t) - (3 * a0_t - a1_t)) / 2.0;
                    double c4 = (-30 * (p1 - p0) + (14 * v1_t + 16 * v0_t) + (3 * a0_t - 2 * a1_t)) / 2.0;
                    double c5 = (12 * (p1 - p0) - (6 * v1_t + 6 * v0_t) - (a0_t - a1_t)) / 2.0;

                    q[j] = c0 + c1 * tau + c2 * tau2 + c3 * tau3 + c4 * tau4 + c5 * tau5;
                }
                trajectory.push_back(q);
            }
        }

        if (!trajectory.empty()) {
            trajectory.back() = path.back();
        }

        return trajectory;
    }
};