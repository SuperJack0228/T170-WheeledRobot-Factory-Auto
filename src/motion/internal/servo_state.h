#pragma once
#include "t170/motion/servo.h"
#include "t170/telemetry/dispatch_metrics.h"
#include <mutex>
#include <map>
#include <atomic>
#include <optional>
namespace wrist_servo::detail {
struct Runtime {
    std::mutex mutex;
    ObservationContext context;
    StateHistory history;
    std::optional<PixelObservation> pending;
    std::optional<std::pair<double,bool>> pending_view;
    Eigen::Matrix4d tcp_to_camera = Eigen::Matrix4d::Identity();
    orch::TimingContext telemetry;
    std::map<std::string,double> metrics;
    double began_s=0;
    bool succeeded = false;
    bool final_hold = false;
    Joints final_command = Joints::Zero();
    Vec2 completed_goal = Vec2::Zero();
    std::string failure_reason;
    TrayTarget staged_tray_target;
    std::optional<double> staged_tray_grasp_plane;
    bool staged_belt_grasp_reference = false;
};
extern Runtime slots[2];
extern std::atomic<std::uint64_t> next_session;
extern std::atomic<bool> group_failed;
State measured(Robot_Arm &arm, Runtime &slot, double &read_ms);
bool pair_too_close(int hand, const State &s, const Vec2 &goal, double clearance);
void send(int hand, const Joints &q);
}
