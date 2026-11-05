#pragma once
#include <Eigen/Dense>
#include <functional>
#include <string>

namespace t170::motion {
using JointVector = Eigen::Matrix<double, 1, 7>;
struct JointMotionLimits {
    double velocity = 0.9, acceleration = 2.5, jerk = 20.0;
};
// Natural cubic joint path, traversed with one rest-to-rest quintic clock.
// Limits apply to the sampled position stream, including the stationary tails.
Eigen::MatrixXd time_joint_path(const Eigen::MatrixXd &waypoints, double dt,
                               const JointMotionLimits &limits, double minimum_duration = 0.0);
struct MotionFeedbackConfig {
    double tolerance_rad = 0.12, settle_timeout_s = 4.0, feedback_timeout_s = 0.5;
    int stable_samples = 3;
};
struct MotionIO {
    std::function<bool(JointVector &)> read;
    std::function<bool(const JointVector &)> send;
    std::function<int()> stop_code;
    std::function<bool(const JointVector &)> arrived; // optional task clearance predicate
    std::function<double()> now;
    std::function<void(double)> wait;
    std::function<void(const char *, int, const JointVector &, const JointVector *, double)> observe;
};
// 0 means measured joint arrival; -8 timeout, -32 feedback, -2 write, -10 input.
// IO owns hardware and the clock; the same executor is used by lift and return.
int run_joint_motion(const Eigen::MatrixXd &trajectory, double dt,
                     const MotionFeedbackConfig &config, const MotionIO &io);
} // namespace t170::motion
