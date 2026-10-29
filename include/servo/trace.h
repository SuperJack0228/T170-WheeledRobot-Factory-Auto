#pragma once
#include "servo/visual_servo.h"
#include <condition_variable>
#include <atomic>
#include <fstream>
#include <mutex>
#include <thread>

namespace wrist_servo {
struct TraceRow {
    double time_s = 0;
    std::uint64_t session = 0;
    int hand = -1;
    int target_class = -1;
    std::string task_tag;
    std::string event, reason, profile_mode;
    PixelObservation observation;
    Mapping mapping;
    State measured;
    Output control;
    Vec2 goal = Vec2::Zero();
    Vec3 reference = Vec3::Zero();
    Vec3 requested_velocity = Vec3::Zero();
    double applied_scale = 0.0;
    std::string solver_reason;
    Joints commanded = Joints::Zero();
    double final_z = 0, dt_ms = 0, read_ms = 0, solve_ms = 0;
    double solver_xyz_m = 0, solver_rotation_rad = 0;
    double actual_rotation_error_rad = 0, reference_lead_m = 0, joint_lead_rad = 0;
    int solver_failures = 0;
    double gripper_position_rad = 0, gripper_torque_nm = 0;
    // Open-loop retract diagnostics. Feedback is read only at start/end, never per tick.
    int trajectory_row = -1;
    bool joint_feedback_valid = false;
    double command_step_rad = 0, command_speed_rad_s = 0;
    double command_accel_rad_s2 = 0, command_jerk_rad_s3 = 0;
    double hold_release_wait_ms = 0;
};

// Disk IO is confined to the writer; a bounded queue must not delay a motion tick.
class TraceWriter {
public:
    ~TraceWriter();
    bool start(const std::string &directory);
    void push(TraceRow row);
    std::string path() const;
    bool healthy() const { return healthy_.load(); }
private:
    void run();
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<TraceRow> queue_;
    std::thread thread_;
    std::ofstream file_;
    std::string path_;
    bool stopping_ = false;
    std::uint64_t dropped_ = 0;
    std::atomic<bool> healthy_{false};
};
TraceWriter &trace_writer();
}
